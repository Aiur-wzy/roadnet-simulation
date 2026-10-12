#!/usr/bin/env python3
"""Materialize SUMO's actual per-vehicle speedFactor without altering sources.

Replay the original inputs/seed, preserve SUMO's loading/stepping options, write
explicit vehicle factors, then replay the materialized route and compare truth.
This is not a training-label collector. All SUMO outputs are redirected into a
new, portable dataset. Dataset mode uses traci_jobs.tsv, not stale cfg globs.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import xml.etree.ElementTree as ET

SCHEMA = "explicit-speed-factor-v1"
JOB_COLUMNS = ["episode_id", "split", "sumocfg_file", "legacy_training_output",
               "full_training_output", "vehicle_event_output", "cycle_summary_output", "run_log"]


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
        return digest.hexdigest()


def read_xml(path: Path) -> ET.ElementTree:
    return ET.parse(path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))


def write_xml(tree: ET.ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".pending")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def positive_factor(value, context: str) -> float:
    try:
        factor = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"Invalid speedFactor for {context}: {value!r}") from exc
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError(f"speedFactor must be finite and positive for {context}: {value!r}")
    return factor


def relative(path: Path, directory: Path) -> str:
    return Path(os.path.relpath(path, directory)).as_posix()


def option(tree: ET.ElementTree, name: str, default=None):
    matches = tree.getroot().findall(f".//{name}")
    if len(matches) > 1:
        raise ValueError(f"Repeated SUMO option: {name}")
    return matches[0].get("value", default) if matches else default


def set_option(tree: ET.ElementTree, group: str, name: str, value) -> None:
    parent = tree.getroot().find(group)
    if parent is None:
        parent = ET.SubElement(tree.getroot(), group)
    node = parent.find(name)
    if node is None:
        node = ET.SubElement(parent, name)
    node.set("value", str(value))


def config_input(config: Path, value: str) -> Path:
    if not value or "," in value:
        raise ValueError(f"Expected one input file in {config}: {value!r}")
    path = Path(os.path.expandvars(value))
    return (path if path.is_absolute() else config.parent / path).resolve(strict=True)


def inspect_config(config: Path, split: str = "validation") -> dict:
    config = config.resolve(strict=True)
    tree = read_xml(config)
    if tree.getroot().tag != "configuration":
        raise ValueError(f"Not a SUMO configuration: {config}")
    # Fail closed: unknown file/output options must not write to old datasets.
    for group in tree.getroot():
        if not isinstance(group.tag, str):
            continue
        for node in group:
            if not isinstance(node.tag, str):
                continue
            if group.tag == "input" and node.tag not in ("net-file", "route-files"):
                raise ValueError(f"Unsupported input option {node.tag} in {config}")
            if group.tag == "output" and node.tag not in ("tripinfo-output", "statistic-output"):
                raise ValueError(f"Unsupported output option {node.tag} in {config}")
            if node.tag.startswith("save-"):
                raise ValueError(f"Unsupported saving option {node.tag} in {config}")
            if group.tag not in ("input", "output") and any(
                    key in node.tag for key in ("file", "output", "save-state", "load-state", "log")):
                if node.tag not in ("no-step-log", "duration-log.statistics", "duration-log.disable"):
                    raise ValueError(f"Unreviewed file/log option {node.tag} in {config}")
    if str(option(tree, "random", "false")).lower() not in ("false", "0"):
        raise ValueError(f"--random is incompatible with deterministic materialization: {config}")
    seed = int(option(tree, "seed", "23423"))
    route = config_input(config, option(tree, "route-files"))
    net = config_input(config, option(tree, "net-file"))
    root = read_xml(route).getroot()
    if any(root.findall(tag) for tag in ("flow", "trip", "person", "personFlow", "container", "containerFlow")):
        raise ValueError(f"Only explicit vehicle routes are supported (expand flows first): {route}")
    vehicle_ids = set()
    for vehicle in root.findall("vehicle"):
        identifier = vehicle.get("id")
        if not identifier or identifier in vehicle_ids:
            raise ValueError(f"Missing/duplicate vehicle id {identifier!r}: {route}")
        if "speedFactor" in vehicle.attrib:
            positive_factor(vehicle.get("speedFactor"), identifier)
        vehicle_ids.add(identifier)
    if not vehicle_ids:
        raise ValueError(f"No explicit vehicles: {route}")
    historical = option(tree, "tripinfo-output")
    historical_path = None
    if historical:
        raw = Path(os.path.expandvars(historical))
        historical_path = (raw if raw.is_absolute() else config.parent / raw).resolve()
    episode_meta = config.parent.parent / "manifests" / f"{config.stem}.json"
    metadata = {}
    if episode_meta.is_file():
        metadata = json.loads(episode_meta.read_text(encoding="utf-8-sig")).get("episode", {})
    return {"episode_id": config.stem, "split": split, "config": str(config), "route": str(route),
            "net": str(net), "seed": seed, "vehicle_count": len(vehicle_ids),
            "historical_tripinfo": str(historical_path) if historical_path and historical_path.is_file() else None,
            "episode_metadata": metadata,
            "source_hashes": {str(p): sha256(p) for p in (config, route, net)}}


def discover_jobs(dataset: Path | None, configs: list[Path]) -> list[dict]:
    jobs = []
    if dataset is not None:
        manifest = dataset / "commands" / "traci_jobs.tsv"
        with manifest.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream, delimiter="\t"))
        for row in rows:
            raw = Path(row["sumocfg_file"])
            path = raw if raw.is_absolute() else dataset / raw
            job = inspect_config(path, row["split"])
            if job["episode_id"] != row["episode_id"]:
                raise ValueError(f"Manifest episode/config mismatch: {row}")
            job["source_hashes"][str(manifest.resolve())] = sha256(manifest)
            jobs.append(job)
    else:
        jobs = [inspect_config(path) for path in configs]
    names = [job["episode_id"] for job in jobs]
    if not jobs or len(names) != len(set(names)):
        raise ValueError("No jobs or repeated episode names")
    for name in names:
        if not name or name in (".", "..") or any(c in name for c in '/\\:\t\n\r'):
            raise ValueError(f"Unsafe episode name: {name!r}")
    return jobs


def copy_checked(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha256(source) != sha256(destination):
            raise ValueError(f"Refusing to overwrite a changed input copy: {destination}")
    else:
        shutil.copyfile(source, destination)


def materialized_tree(original: ET.ElementTree, factors: dict[str, float]) -> ET.ElementTree:
    tree = copy.deepcopy(original)
    vehicles = tree.getroot().findall("vehicle")
    identifiers = [vehicle.get("id") for vehicle in vehicles]
    if set(identifiers) != set(factors) or len(identifiers) != len(factors):
        raise ValueError("Vehicle coverage mismatch when writing factors")
    for vehicle in vehicles:
        factor = positive_factor(factors[vehicle.get("id")], vehicle.get("id"))
        if "speedFactor" in vehicle.attrib and float(vehicle.get("speedFactor")) != factor:
            raise ValueError(f"SUMO altered an already explicit factor: {vehicle.get('id')}")
        vehicle.set("speedFactor", repr(factor))
    return tree


def route_equivalent_except_sf(original: ET.ElementTree, updated: ET.ElementTree) -> bool:
    left, right = copy.deepcopy(original), copy.deepcopy(updated)
    for tree in (left, right):
        for vehicle in tree.getroot().findall("vehicle"):
            vehicle.attrib.pop("speedFactor", None)
    return ET.tostring(left.getroot()) == ET.tostring(right.getroot())


def prepare_files(job: dict, output: Path) -> dict:
    name = job["episode_id"]
    net_source = Path(job["net"])
    net = output / "inputs" / (sha256(net_source)[:12] + "_" + net_source.name)
    source_route = output / "original" / "routes" / f"{name}.rou.xml"
    copy_checked(net_source, net)
    copy_checked(Path(job["route"]), source_route)
    copy_checked(Path(job["config"]), output / "original" / "config_as_received" / f"{name}.sumocfg")
    paths = {"net": net, "source_route": source_route,
             "route": output / "routes" / f"{name}.rou.xml"}
    for role in ("reference", "explicit"):
        config = output / ("original/sumocfg" if role == "reference" else "sumocfg") / f"{name}.sumocfg"
        trip = output / "outputs" / ("reference_tripinfo" if role == "reference" else "tripinfo") / f"tripinfo_{name}.xml"
        stats = output / "outputs" / "statistics" / f"{role}_{name}.xml"
        for path in (config, trip, stats):
            path.parent.mkdir(parents=True, exist_ok=True)
        tree = read_xml(Path(job["config"]))
        set_option(tree, "input", "net-file", relative(net, config.parent))
        set_option(tree, "input", "route-files", relative(source_route if role == "reference" else paths["route"], config.parent))
        set_option(tree, "output", "tripinfo-output", relative(trip, config.parent))
        set_option(tree, "output", "statistic-output", relative(stats, config.parent))
        set_option(tree, "random_number", "seed", job["seed"])
        write_xml(tree, config)
        paths[role + "_config"], paths[role + "_tripinfo"], paths[role + "_statistics"] = config, trip, stats
    return {key: str(path) for key, path in paths.items()}


def load_traci(sumo_binary: str):
    candidates = []
    if os.environ.get("SUMO_HOME"):
        candidates.append(Path(os.environ["SUMO_HOME"]) / "tools")
    executable = shutil.which(sumo_binary) or sumo_binary
    candidates.append(Path(executable).resolve().parent.parent / "tools")
    for path in candidates:
        if (path / "traci").is_dir():
            sys.path.insert(0, str(path))
            break
    try:
        import traci
    except ImportError as exc:
        raise RuntimeError("Set SUMO_HOME or install the matching traci Python package") from exc
    return traci


def replay(config: Path, route: Path, output: Path, role: str, binary: str, max_time: float) -> dict:
    traci = load_traci(binary)
    expected = {v.get("id"): v.attrib for v in read_xml(route).getroot().findall("vehicle")}
    log = output / "outputs" / "logs" / f"{role}_{config.stem}.log"
    errors = log.with_suffix(".errors.log")
    log.parent.mkdir(parents=True, exist_ok=True)
    command = [binary, "-c", str(config), "--log", str(log), "--error-log", str(errors)]
    factors, departed, arrived = {}, set(), set()
    loaded_departure_changes = 0
    teleports = 0
    end = float(option(read_xml(config), "end", "-1"))
    start = time.monotonic()
    with log.with_suffix(".stdout.log").open("w", encoding="utf-8") as stdout:
        traci.start(command, stdout=stdout, numRetries=30)
        try:
            version = traci.getVersion()

            def read_loaded():
                for identifier in traci.simulation.getLoadedIDList():
                    if identifier not in expected:
                        raise ValueError(f"Unexpected loaded vehicle: {identifier}")
                    factors[identifier] = positive_factor(traci.vehicle.getSpeedFactor(identifier), identifier)

            read_loaded()
            while traci.simulation.getMinExpectedNumber() > 0:
                now = traci.simulation.getTime()
                if now >= max_time or (end >= 0 and now >= end):
                    raise RuntimeError(f"Simulation ended before all vehicles completed: {config} at {now}")
                traci.simulationStep()
                read_loaded()
                for identifier in traci.simulation.getDepartedIDList():
                    if identifier not in expected or identifier in departed:
                        raise ValueError(f"Unexpected/repeated departure: {identifier}")
                    actual = positive_factor(traci.vehicle.getSpeedFactor(identifier), identifier)
                    if identifier in factors and factors[identifier] != actual:
                        loaded_departure_changes += 1
                    factors[identifier] = actual
                    departed.add(identifier)
                arrived.update(traci.simulation.getArrivedIDList())
                teleports += traci.simulation.getStartingTeleportNumber()
            simulation_end = traci.simulation.getTime()
        finally:
            traci.close()
    if set(factors) != set(expected) or departed != set(expected) or arrived != set(expected):
        raise RuntimeError(f"Incomplete vehicle coverage in {config}: expected={len(expected)}, "
                           f"factors={len(factors)}, departed={len(departed)}, arrived={len(arrived)}")
    if teleports:
        raise RuntimeError(f"Reference/materialized episode has {teleports} teleports: {config}")
    return {"version": list(version), "factors": factors, "vehicles": len(expected),
            "loaded_departure_changes": loaded_departure_changes, "teleports": teleports,
            "simulation_end": simulation_end, "elapsed_seconds": time.monotonic() - start,
            "command": command}


def tripinfo_rows(path: Path) -> dict:
    result = {}
    for _, node in ET.iterparse(path, events=("end",)):
        if node.tag == "tripinfo":
            identifier = node.get("id")
            if identifier in result:
                raise ValueError(f"Duplicate tripinfo id {identifier}: {path}")
            result[identifier] = dict(node.attrib)
            node.clear()
    return result


def compare_tripinfo(left: Path, right: Path) -> dict:
    first, second = tripinfo_rows(left), tripinfo_rows(right)
    common = first.keys() & second.keys()
    differing = [key for key in common if first[key] != second[key]]
    durations = [abs(float(first[key]["duration"]) - float(second[key]["duration"])) for key in common]
    return {"same_vehicle_ids": first.keys() == second.keys(), "vehicle_count": len(common),
            "all_attributes_equal": first == second, "differing_vehicles": len(differing),
            "examples": [{"vehicle_id": key, "changed_fields": {field: [first[key].get(field), second[key].get(field)]
                          for field in first[key].keys() | second[key].keys()
                          if first[key].get(field) != second[key].get(field)}} for key in sorted(differing)[:3]],
            "max_duration_delta": max(durations, default=0.0),
            "mean_abs_duration_delta": sum(durations) / len(durations) if durations else None}


def run_job(job: dict, output_string: str, binary: str, max_time: float, resume: bool) -> dict:
    output = Path(output_string)
    manifest = output / "manifests" / f"{job['episode_id']}.json"
    if resume and manifest.is_file():
        old = json.loads(manifest.read_text(encoding="utf-8"))
        if old.get("status") == "success":
            if old["source_hashes"] != job["source_hashes"]:
                raise ValueError(f"Source inputs changed: {job['episode_id']}")
            if not all((output / name).is_file() and sha256(output / name) == digest
                       for name, digest in old["artifact_hashes"].items()):
                raise ValueError(f"Completed artifacts changed: {job['episode_id']}")
            return old
    try:
        paths = prepare_files(job, output)
        reference = replay(Path(paths["reference_config"]), Path(paths["source_route"]), output,
                           "reference", binary, max_time)
        original = read_xml(Path(paths["source_route"]))
        updated = materialized_tree(original, reference["factors"])
        if not route_equivalent_except_sf(original, updated):
            raise AssertionError("Materialization changed attributes other than speedFactor")
        write_xml(updated, Path(paths["route"]))
        explicit = replay(Path(paths["explicit_config"]), Path(paths["route"]), output,
                          "explicit", binary, max_time)
        if reference["version"] != explicit["version"] or reference["factors"] != explicit["factors"]:
            raise AssertionError("Explicit replay changed SUMO version or per-vehicle factors")
        if any(sha256(Path(path)) != digest for path, digest in job["source_hashes"].items()):
            raise AssertionError("Source input changed during replay")
        comparison = compare_tripinfo(Path(paths["reference_tripinfo"]), Path(paths["explicit_tripinfo"]))
        history = compare_tripinfo(Path(job["historical_tripinfo"]), Path(paths["reference_tripinfo"])) if job["historical_tripinfo"] else None
        values = list(reference["factors"].values())
        result = {**job, "schema": SCHEMA, "status": "success", "paths": {k: relative(Path(v), output) for k, v in paths.items()},
                  "reference_run": {k: v for k, v in reference.items() if k != "factors"},
                  "explicit_run": {k: v for k, v in explicit.items() if k != "factors"},
                  "sf_range": [min(values), max(values)], "sf_mean": sum(values) / len(values),
                  "factors_exactly_preserved": True, "route_unchanged_except_sf": True,
                  "source_inputs_unchanged": True, "reference_vs_explicit": comparison,
                  "historical_vs_reference": history,
                  "truth_policy": "Use outputs/tripinfo generated with this dataset; old truth is not silently assumed equivalent.",
                  "artifact_hashes": {relative(Path(v), output): sha256(Path(v)) for v in paths.values()}}
        write_json(manifest, result)
        return result
    except Exception as exc:
        write_json(manifest, {**job, "status": "failed", "error": f"{type(exc).__name__}: {exc}"})
        raise


def write_job_manifest(output: Path, jobs: list[dict]) -> None:
    path = output / "commands" / "traci_jobs.tsv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=JOB_COLUMNS, delimiter="\t", lineterminator="\n")
        writer.writeheader()
        for job in jobs:
            name = job["episode_id"]
            writer.writerow(dict(zip(JOB_COLUMNS, [name, job["split"], f"sumocfg/{name}.sumocfg",
                f"outputs/traci/TraCI_output_adjusted_{name}.csv", f"outputs/traci/cams_model_training_data_{name}.csv",
                f"outputs/traci/cams_vehicle_events_{name}.csv", f"outputs/traci/cams_cycle_summary_{name}.csv",
                f"outputs/logs/traci_{name}.log"])))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--dataset-dir", type=Path, help="Use this dataset's commands/traci_jobs.tsv")
    source.add_argument("--config", action="append", type=Path, help="Explicit configuration; repeat for multiple episodes")
    parser.add_argument("--output-dir", required=True, type=Path, help="New self-contained dataset (never the source)")
    parser.add_argument("--sumo-binary", default="sumo")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--max-sim-time", type=float, default=100000)
    parser.add_argument("--resume", action="store_true", help="Resume this tool's matching run; retry owned incomplete outputs")
    parser.add_argument("--limit", type=int, help="First N manifest jobs, for a small verification run")
    args = parser.parse_args(argv)
    if args.workers < 1 or not math.isfinite(args.max_sim_time) or args.max_sim_time <= 0 or (args.limit is not None and args.limit < 1):
        parser.error("workers, max-sim-time and limit must be positive")
    jobs = discover_jobs(args.dataset_dir.resolve() if args.dataset_dir else None, args.config or [])
    if args.limit:
        jobs = jobs[:args.limit]
    output = args.output_dir.resolve()
    for job in jobs:
        for key in ("config", "route", "net"):
            if output == Path(job[key]) or output in Path(job[key]).parents:
                parser.error("Output directory must not contain source input files")
    executable = shutil.which(args.sumo_binary)
    if executable is None and Path(args.sumo_binary).is_file():
        executable = str(Path(args.sumo_binary).resolve())
    if executable is None:
        parser.error(f"SUMO executable not found: {args.sumo_binary}")
    signature = {"schema": SCHEMA, "sumo_binary_sha256": sha256(Path(executable)),
                 "jobs": [{k: job[k] for k in ("episode_id", "split", "source_hashes")} for job in jobs]}
    marker = output / "manifests" / "materialization_run.json"
    if output.exists():
        if not args.resume or not marker.is_file() or json.loads(marker.read_text(encoding="utf-8")) != signature:
            parser.error("Output exists; use a new directory or --resume with an unchanged matching run")
    else:
        output.mkdir(parents=True)
        write_json(marker, signature)
    # Pre-copy shared network inputs serially, avoiding concurrent copy races.
    for job in jobs:
        net = Path(job["net"])
        copy_checked(net, output / "inputs" / (sha256(net)[:12] + "_" + net.name))
    results, failures = [], []
    print(f"Materializing {len(jobs)} episodes / {sum(j['vehicle_count'] for j in jobs)} vehicles", flush=True)
    started = time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_job, job, str(output), executable, args.max_sim_time, args.resume): job for job in jobs}
        for future in as_completed(futures):
            job = futures[future]
            try:
                result = future.result()
                results.append(result)
                print(f"[OK {len(results) + len(failures)}/{len(jobs)}] {job['episode_id']} "
                      f"vehicles={result['vehicle_count']} tripinfo_equal={result['reference_vs_explicit']['all_attributes_equal']}", flush=True)
            except Exception as exc:
                failures.append({"episode_id": job["episode_id"], "error": f"{type(exc).__name__}: {exc}"})
                print(f"[FAIL] {job['episode_id']}: {exc}", flush=True)
    results.sort(key=lambda value: value["episode_id"])
    summary = {"schema": SCHEMA, "expected_episodes": len(jobs), "successful_episodes": len(results),
               "sumo_versions": sorted({str(j["reference_run"]["version"]) for j in results}),
               "vehicles": sum(job["vehicle_count"] for job in results), "failures": failures,
               "all_factors_preserved": len(results) == len(jobs) and all(j["factors_exactly_preserved"] for j in results),
               "all_tripinfo_attributes_equal": len(results) == len(jobs) and all(j["reference_vs_explicit"]["all_attributes_equal"] for j in results),
               "historical_comparisons": sum(j["historical_vs_reference"] is not None for j in results),
               "historical_mismatches": [j["episode_id"] for j in results if j["historical_vs_reference"] and not j["historical_vs_reference"]["all_attributes_equal"]],
               "elapsed_seconds": time.monotonic() - started,
               "episodes": [{"episode_id": j["episode_id"], "split": j["split"], "seed": j["seed"], "vehicles": j["vehicle_count"],
                             "sf_range": j["sf_range"], "sf_mean": j["sf_mean"], "reference_vs_explicit": j["reference_vs_explicit"]} for j in results]}
    write_json(output / "manifests" / "materialization_summary.json", summary)
    if not failures:
        write_job_manifest(output, jobs)
    print(json.dumps({k: v for k, v in summary.items() if k != "episodes"}, ensure_ascii=False), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
