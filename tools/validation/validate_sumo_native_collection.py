"""Bounded integration probe: untouched SUMO versus read-only native collection.

Copies a small existing route/config fixture into a fresh evidence directory,
restores full default lane changing, disables every teleport option, and keeps
the remaining dynamics, seed, demand and step length.  It runs at most 40 cars.
It never modifies the supplied files or silently resumes an earlier probe.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import csv
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


STAGE = Path(__file__).resolve().parents[2]
EVIDENCE = STAGE.parent / "evidence"
TELEPORTS = (
    "time-to-teleport", "time-to-teleport.highways",
    "time-to-teleport.disconnected", "time-to-teleport.ride",
    "time-to-teleport.bidi", "time-to-teleport.railsignal-deadlock",
)
LABELS = tuple("travel_time_label_gt_" + x for x in ("0p1", "0p5", "1", "1p5", "2", "3"))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save_json(path, value):
    with Path(path).open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def set_option(root, section, name, value):
    group = root.find(section)
    if group is None:
        group = ET.SubElement(root, section)
    node = group.find(name)
    if node is None:
        node = ET.SubElement(group, name)
    node.set("value", str(value))


def write_xml(path, root):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def output_flags(directory):
    return [
        "--tripinfo-output", str(directory / "tripinfo.xml"),
        "--tripinfo-output.write-unfinished", "true",
        "--vehroute-output", str(directory / "vehroute.xml.gz"),
        "--vehroute-output.exit-times", "true",
        "--vehroute-output.write-unfinished", "true",
        "--vehroute-output.speedfactor", "true",
        "--statistic-output", str(directory / "statistics.xml"),
        "--summary-output", str(directory / "summary.xml.gz"),
        "--lanechange-output", str(directory / "lanechanges.xml.gz"),
        "--log", str(directory / "sumo.log"),
        "--error-log", str(directory / "sumo.stderr.log"),
        "--no-step-log", "true", "--duration-log.statistics", "true",
    ]


def run_child(command, stdout, stderr, timeout, env):
    start = time.monotonic()
    with stdout.open("w", encoding="utf-8") as out, stderr.open("w", encoding="utf-8") as err:
        try:
            result = subprocess.run(command, cwd=STAGE, env=env, stdout=out, stderr=err,
                                    timeout=timeout, check=False)
            return {"command": command, "returncode": result.returncode,
                    "wall_seconds": time.monotonic() - start}
        except subprocess.TimeoutExpired:
            return {"command": command, "returncode": None, "timed_out": True,
                    "wall_seconds": time.monotonic() - start}


def trips(path):
    result = {}
    for _, node in ET.iterparse(path, events=("end",)):
        if node.tag != "tripinfo":
            continue
        vehicle = node.get("id")
        if vehicle in result:
            raise ValueError(f"duplicate tripinfo ID: {vehicle}")
        result[vehicle] = dict(node.attrib)
        node.clear()
    return result


def rows(path):
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def short_fragments(pool, network):
    """Take <=8 continuous legal fragments; never use these as training demand."""
    lengths = {}
    for _, node in ET.iterparse(network, events=("end",)):
        if node.tag == "edge":
            if not node.get("id", "").startswith(":"):
                values = [float(lane.get("length")) for lane in node.findall("lane")]
                if values:
                    lengths[node.get("id")] = max(values)
            node.clear()
    candidates, seen = {"short_first": [], "short_interior": []}, set()
    with pool.open("r", encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            edges = record["edges"]
            for index, edge in enumerate(edges):
                if edge not in lengths or lengths[edge] >= 5 or index + 6 > len(edges):
                    continue
                if len(candidates["short_first"]) < 4:
                    selected = tuple(edges[index:index + 10])
                    key = ("short_first", selected)
                    if key not in seen:
                        seen.add(key)
                        candidates["short_first"].append(dict(kind="short_first", path_id=record.get("path_id", record.get("route_id")),
                            source_start_index=index, roads=selected, lengths=[lengths[x] for x in selected]))
                if (len(candidates["short_interior"]) < 4 and index >= 2 and lengths[edge] < 2
                        and lengths.get(edges[index - 1], 0) >= 30):
                    selected = tuple(edges[index - 2:index + 8])
                    key = ("short_interior", selected)
                    if key not in seen:
                        seen.add(key)
                        candidates["short_interior"].append(dict(kind="short_interior", path_id=record.get("path_id", record.get("route_id")),
                            source_start_index=index - 2, roads=selected, lengths=[lengths[x] for x in selected]))
            if all(len(group) == 4 for group in candidates.values()):
                break
    selected = candidates["short_first"] + candidates["short_interior"]
    if not candidates["short_first"] or not candidates["short_interior"]:
        raise ValueError("route pool lacks the required bounded short-road fragment cases")
    routes = ET.Element("routes")
    ET.SubElement(routes, "vType", {"id": "car", "vClass": "passenger"})
    for index, candidate in enumerate(selected):
        vehicle = f"native_short_{index:02d}"
        candidate["vehicle_id"] = vehicle
        node = ET.SubElement(routes, "vehicle", {"id": vehicle, "type": "car", "depart": str(index * 30),
                                                   "speedFactor": "1"})
        ET.SubElement(node, "route", {"edges": " ".join(candidate["roads"])})
    return routes, selected


def check_outputs(root, expected):
    errors = []
    native = trips(root / "native" / "tripinfo.xml")
    collected = trips(root / "collection" / "tripinfo.xml")
    differences = []
    for vehicle in sorted(set(native) | set(collected)):
        left, right = native.get(vehicle), collected.get(vehicle)
        if left != right:
            differences.append({"vehicle_id": vehicle, "attributes": {
                key: {"native": (left or {}).get(key), "collector": (right or {}).get(key)}
                for key in sorted(set(left or {}) | set(right or {}))
                if (left or {}).get(key) != (right or {}).get(key)}})
    if set(native) != set(expected) or set(collected) != set(expected):
        errors.append("native/collector tripinfo IDs do not match demand")
    if differences:
        errors.append("native and collected tripinfo differ")
    audit = json.loads((root / "collection" / "audit.json").read_text(encoding="utf-8"))
    if audit["status"] != "COMPLETE":
        errors.append("collector audit did not complete successfully")
    visit_path = root / "collection" / "road_visits.csv.gz"
    if not visit_path.exists():
        visit_path = root / "collection" / "road_visits.partial.csv.gz"
    visits = rows(visit_path)
    by_visit = {}
    for row in visits:
        key = (row["vehicle_id"], int(row["route_index"]))
        if key in by_visit:
            errors.append(f"duplicate road occurrence: {key}")
        by_visit[key] = row
        if row["label_valid"] == "0" and any(row.get(label, "") != "" for label in LABELS):
            errors.append(f"fabricated label for unobserved/invalid occurrence: {key}")
    required = {(vehicle, index): road for vehicle, route in expected.items() for index, road in enumerate(route)}
    if set(by_visit) != set(required):
        errors.append("road occurrence identities do not exactly cover supplied demand")
    for key in set(required) & set(by_visit):
        if by_visit[key]["road_id"] != required[key]:
            errors.append(f"incorrect road identity: {key}")
    training_path = root / "collection" / "training.csv.gz"
    if not training_path.exists():
        training_path = root / "collection" / "training.partial.csv.gz"
    training = rows(training_path)
    expected_training = {k for k, row in by_visit.items() if row["trainable"] == "1"}
    actual_training = {(r["vehicle_id"], int(r["route_index"])) for r in training}
    if actual_training != expected_training or len(actual_training) != len(training):
        errors.append("training stream is not exactly the trainable occurrence subset")
    return dict(errors=errors, native_tripinfo_count=len(native), collector_tripinfo_count=len(collected),
                tripinfo_all_attributes_exact=not differences, tripinfo_differences=differences,
                expected_occurrences=len(required), emitted_occurrences=len(visits), training_rows=len(training),
                quality_counts=dict(Counter(row["quality"] for row in visits)),
                blank_label_occurrences=[dict(vehicle_id=k[0], route_index=k[1], road_id=row["road_id"],
                    quality=row["quality"]) for k, row in by_visit.items() if row["label_valid"] == "0"],
                collector_audit=audit)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo-config", required=True)
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sumo-binary", required=True)
    parser.add_argument("--route-file", help="Optional separately prepared small legal-fragment fixture")
    parser.add_argument("--short-pool", help="Build <=8 short-road legal fragments from a path_id/edges JSONL pool")
    parser.add_argument("--timeout-seconds", type=float, default=300.0)
    args = parser.parse_args(argv)
    source_cfg, network, root = map(lambda x: Path(x).resolve(), (args.sumo_config, args.net_file, args.output_dir))
    if root.exists() or root == EVIDENCE.resolve() or not root.is_relative_to(EVIDENCE.resolve()):
        raise ValueError("output must be a fresh child of this batch's evidence directory")
    original_cfg = ET.parse(source_cfg).getroot()
    if original_cfg.find("./time/end") is not None:
        raise ValueError("fixture must not have a hard simulation end")
    if args.route_file and args.short_pool:
        raise ValueError("--route-file and --short-pool are mutually exclusive")
    short_cases = None
    if args.short_pool:
        sources = [Path(args.short_pool).resolve()]
        generated_routes, short_cases = short_fragments(sources[0], network)
    elif args.route_file:
        sources = [Path(args.route_file).resolve()]
    else:
        sources = [(source_cfg.parent / p.strip()).resolve()
                   for p in original_cfg.find("./input/route-files").get("value").split(",")]
    original_hashes = {str(path): sha256(path) for path in (source_cfg, network, *sources)}
    route_root = ET.Element("routes")
    removed_lc, expected = [], {}
    route_trees = [generated_routes] if args.short_pool else [ET.parse(source).getroot() for source in sources]
    for source_root in route_trees:
        for original in source_root:
            node = copy.deepcopy(original)
            if node.tag == "vType":
                for name in tuple(node.attrib):
                    if name.startswith("lc") or name == "laneChangeMode":
                        removed_lc.append(dict(type_id=node.get("id"), parameter=name, value=node.attrib.pop(name)))
            elif node.tag == "vehicle":
                if node.find("route") is None or node.get("id") in expected:
                    raise ValueError("fixture requires unique explicit vehicles/inline routes")
                expected[node.get("id")] = tuple(node.find("route").get("edges").split())
            elif node.tag != "route":
                raise ValueError(f"unsupported fixture root element: {node.tag}")
            route_root.append(node)
    if not 1 <= len(expected) <= 40:
        raise ValueError("bounded probe requires 1..40 explicit vehicles")
    root.mkdir(parents=True)
    copied_route = root / "fixture.rou.xml"
    write_xml(copied_route, route_root)
    cfg = copy.deepcopy(original_cfg)
    precision = original_cfg.find("./output/precision")
    for group in tuple(cfg):
        if group.tag in {"output", "report", "gui_only"}:
            cfg.remove(group)
    set_option(cfg, "input", "net-file", network)
    set_option(cfg, "input", "route-files", copied_route)
    for key in TELEPORTS:
        set_option(cfg, "processing", key, -1)
    for key in ("collision.action", "intermodal-collision.action"):
        set_option(cfg, "processing", key, "warn")
    set_option(cfg, "output", "precision", precision.get("value") if precision is not None else 12)
    new_cfg = root / "probe.sumocfg"
    write_xml(new_cfg, cfg)
    (root / "native").mkdir()
    native_command = [args.sumo_binary, "-c", str(new_cfg), *output_flags(root / "native")]
    collector_command = [sys.executable, "-B", "-m", "collection.TraCI_Python_SUMO_Native",
        "--sumo-config", str(new_cfg), "--net-file", str(network), "--output-dir", str(root / "collection"),
        "--sumo-binary", args.sumo_binary]
    env = dict(os.environ)
    env["SUMO_HOME"] = str(Path(args.sumo_binary).resolve().parents[1])
    report = dict(schema="sumo-native-integration-parity-v1", original_inputs=original_hashes,
                  copied_route_sha256=sha256(copied_route), removed_lc_attributes=removed_lc,
                  expected_vehicle_count=len(expected), expected_occurrences=sum(map(len, expected.values())),
                  all_teleports_disabled=True, full_default_lane_change=True)
    if short_cases is not None:
        report["small_fragment_cases_not_full_training_routes"] = short_cases
    save_json(root / "validation.json", report)
    report["native_run"] = run_child(native_command, root / "native.runner.stdout.log", root / "native.runner.stderr.log", args.timeout_seconds, env)
    save_json(root / "validation.json", report)
    if report["native_run"]["returncode"] == 0:
        report["collector_run"] = run_child(collector_command, root / "collector.runner.stdout.log", root / "collector.runner.stderr.log", args.timeout_seconds, env)
        try:
            report.update(check_outputs(root, expected))
        except Exception as exc:
            report["errors"] = [f"output validation raised {type(exc).__name__}: {exc}"]
    else:
        report["errors"] = ["native fixture failed; collector not run"]
    report["original_inputs_unchanged"] = all(sha256(path) == digest for path, digest in original_hashes.items())
    if not report["original_inputs_unchanged"]:
        report.setdefault("errors", []).append("original input changed")
    report["passed"] = not report.get("errors") and report.get("collector_run", {}).get("returncode") == 0
    save_json(root / "validation.json", report)
    print(json.dumps({k: report.get(k) for k in ("passed", "expected_vehicle_count", "expected_occurrences",
          "training_rows", "quality_counts", "tripinfo_all_attributes_exact", "errors")}, ensure_ascii=False))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
