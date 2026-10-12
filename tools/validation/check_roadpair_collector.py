"""Bounded SUMO integration: native vs label-only vs road-pair feature observer.

Creates fresh, explicitly marked small fixtures, never full experiments. Manhattan
fixtures are route-library fragments; passing them does not certify full routes.
"""
import argparse
import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            h.update(chunk)
    return h.hexdigest()


def fixture_routes(args):
    root = ET.Element("routes")
    paths = []
    if args.kind == "small":
        original = ET.parse(args.route_source).getroot()
        vehicle_type = copy.deepcopy(original.find("vType"))
        # Both native and collection use the same full-LC integration fixture.
        for key in list(vehicle_type.attrib):
            if key.startswith("lc"):
                del vehicle_type.attrib[key]
        vehicle_type.set("id", "car")
        root.append(vehicle_type)
        paths = [r.get("edges").split() for r in original.findall("route")][:args.vehicles]
    else:
        ET.SubElement(root, "vType", id="car", vClass="passenger")
        net = ET.parse(args.net_file).getroot()
        lengths = {e.get("id"): min(float(l.get("length")) for l in e.findall("lane"))
                   for e in net.findall("edge") if e.findall("lane") and not e.get("id", "").startswith(":")}
        # Pick 5-edge observed-safe fragments, not filtered training/test data.
        with args.route_source.open(encoding="utf-8") as stream:
            for line in stream:
                edges = json.loads(line)["edges"]
                for start in range(len(edges)-4):
                    fragment = edges[start:start+5]
                    if all(lengths.get(e, 0) >= 25 for e in fragment) and fragment not in paths:
                        paths.append(fragment)
                        break
                if len(paths) == args.vehicles:
                    break
    if len(paths) != args.vehicles:
        raise ValueError("not enough fixture routes")
    for i, edges in enumerate(paths):
        vehicle = ET.SubElement(root, "vehicle", id=f"smoke_{i:03d}", type="car",
                                depart=str(i*args.depart_spacing), speedFactor=str(0.9+0.05*(i % 5)))
        ET.SubElement(vehicle, "route", edges=" ".join(edges))
    return root, sum(map(len, paths))


def read_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    result = {(r["vehicleID"], int(r["route_index"]), r["roadID"]): r for r in rows}
    if len(result) != len(rows):
        raise AssertionError("duplicate output road occurrence")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--reference-repo", type=Path, help="Optional untouched tree, for legacy-default keyed value parity on small network")
    parser.add_argument("--net-file", type=Path, required=True)
    parser.add_argument("--route-source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sumo-binary", required=True)
    parser.add_argument("--kind", choices=("small", "manhattan"), required=True)
    parser.add_argument("--vehicles", type=int, default=16)
    parser.add_argument("--depart-spacing", type=float, default=10.)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"passed": False, "kind": args.kind, "full_lane_changing": True,
              "sampling_step_seconds": 1, "vehicles": args.vehicles, "runs": {}}
    try:
        root, expected_rows = fixture_routes(args)
        route = output / "fixture.rou.xml"
        ET.ElementTree(root).write(route, encoding="utf-8", xml_declaration=True)
        report.update(expected_road_occurrences=expected_rows,
                      input_sha256={str(args.net_file.resolve()): digest(args.net_file), str(route): digest(route)})
        env = os.environ.copy()
        env.update(PYTHONUTF8="1", PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8",
                   PYTHONPATH=str(args.repo.resolve())+os.pathsep+env.get("PYTHONPATH", ""))
        arms = ["native", "labels", "features"]
        if args.reference_repo:
            arms += ["legacy_before", "legacy_after"]
        for arm in arms:
            folder = output / arm
            folder.mkdir()
            cfg = ET.Element("configuration")
            inp = ET.SubElement(cfg, "input")
            ET.SubElement(inp, "net-file", value=str(args.net_file.resolve()))
            ET.SubElement(inp, "route-files", value=str(route))
            ET.SubElement(ET.SubElement(cfg, "time"), "step-length", value="1")
            processing = ET.SubElement(cfg, "processing")
            ET.SubElement(processing, "time-to-teleport", value="-1")
            ET.SubElement(ET.SubElement(cfg, "random_number"), "seed", value="42")
            out = ET.SubElement(cfg, "output")
            ET.SubElement(out, "tripinfo-output", value=str(folder / "tripinfo.xml"))
            ET.SubElement(out, "statistic-output", value=str(folder / "statistics.xml"))
            ET.SubElement(out, "precision", value="12")
            cfgpath = folder / "run.sumocfg"
            ET.ElementTree(cfg).write(cfgpath, encoding="utf-8", xml_declaration=True)
            env["PYTHONPATH"] = str((args.reference_repo if arm == "legacy_before" else args.repo).resolve())
            if arm == "native":
                command = [args.sumo_binary, "-c", str(cfgpath)]
            else:
                module = ("collection._shared.TraCI_Python_Threshold_Sweep" if arm == "labels"
                          else "collection.TraCI_Python_Cap_Context_ABC_RG")
                command = [sys.executable, "-B", "-m", module, "--sumo-config", str(cfgpath),
                           "--net-file", str(args.net_file.resolve()), "--sumo-binary", args.sumo_binary,
                           "--outputs", "training", "--training-output", str(folder / "features.csv"),
                           "--legacy-edge-output", str(folder / "unused.csv")]
                if arm == "labels":
                    command += ["--immediate-roadpair-labels"]
                elif arm == "features":
                    command += ["--context-lane-policy", "road-pair-reselection-v1"]
            with (folder / "run.log").open("wb") as log:
                run = subprocess.run(command, cwd=folder, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=240)
            report["runs"][arm] = {"command": command, "returncode": run.returncode}
            if run.returncode:
                raise AssertionError(f"{arm} failed; inspect {folder / 'run.log'}")
        trips = [{v.get("id"): dict(v.attrib) for v in ET.parse(output / arm / "tripinfo.xml").getroot().findall("tripinfo")}
                 for arm in arms]
        if not (all(t == trips[0] for t in trips) and len(trips[0]) == args.vehicles):
            raise AssertionError("read-only collector changed native tripinfo / missing vehicles")
        labels, features = (read_rows(output / arm / "features.csv") for arm in ("labels", "features"))
        if not (labels.keys() == features.keys() and len(labels) == expected_rows):
            raise AssertionError(f"incomplete road coverage: {len(labels)}/{len(features)}/{expected_rows}")
        columns = ["travel_time_label_gt_"+s for s in ("0p1", "0p5", "1", "1p5", "2", "3")]
        for key in labels:
            if any(labels[key][col] != features[key][col] for col in columns):
                raise AssertionError(f"changed threshold labels: {key}")
        audit = read_rows(output / "features/features.csv.audit.csv")
        statuses = {}
        for row in audit.values():
            statuses[row["exit_signal_status"]] = statuses.get(row["exit_signal_status"], 0)+1
            if int(row["shadow_lane_flow"]) > int(row["shadow_lane_capacity"]):
                raise AssertionError("over-reserved lane")
        for arm in arms:
            stats = ET.parse(output / arm / "statistics.xml").getroot()
            if int(stats.find("teleports").get("total")) or int(stats.find("safety").get("collisions")):
                raise AssertionError("teleport/collision in small fixture")
        if args.reference_repo:
            for suffix in ("", ".audit.csv"):
                # Historical row order follows a Python set and is not a data
                # identity. Compare complete occurrences, never row numbers.
                if read_rows(output / ("legacy_before/features.csv"+suffix)) != read_rows(output / ("legacy_after/features.csv"+suffix)):
                    raise AssertionError("legacy default feature/label/audit keyed values changed")
            report["legacy_default_features_labels_audit_keyed_values_exact"] = True
        report.update(passed=True, all_native_tripinfo_attributes_exact=True,
                      six_labels_exact_values=6*len(labels), complete_road_rows=len(features), exit_signal_status=statuses)
    except Exception as exc:
        report["error"] = str(exc)
        raise
    finally:
        (output / "validation.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
        print(json.dumps({"passed": report["passed"], "output": str(output)}))


if __name__ == "__main__":
    main()
