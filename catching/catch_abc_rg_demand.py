#!/usr/bin/env python3
"""Closed-loop, scenario-scoped ABC/RG catch; intermediate runs are provisional.

Only trusted AutoGluon artifacts may be loaded. Predict missing quantized keys,
then restart every scenario. Accept results only after a zero-miss discovery
round and a separate strict zero-miss replay. No full-domain enumeration, model
training, automatic grid changes, overwrite or partial-run resume is performed.
"""
from __future__ import annotations

import argparse
import csv
from functools import lru_cache, partial
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

import catching.model_catching_sumo_cap_context_abc_rg as core

MAGIC = "CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1"
LAYOUT = "sorted-u32-u32-f64-v1"
RECORD = struct.Struct("<IId")
FIXED_ARGUMENTS = ["--use-sumo", "--travel-time-mode", "table",
                   "--movement-queue-order", "arrival-time",
                   "--lane-discharge-interval", "2", "--intersection-crossing-time", "3",
                   "--speed-net-queue-trim", "off", "--speed-net-depart-position", "on",
                   "--speed-net-headway", "0.6", "--speed-net-delay-max", "3",
                   "--speed-net-components", "both"]


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--expected-threshold", choices=["0p1", "0p5", "1", "1p5", "2", "3"])
    parser.add_argument("--simulator", required=True)
    parser.add_argument("--net-file", required=True)
    scenario_source = parser.add_mutually_exclusive_group(required=True)
    scenario_source.add_argument("--scenario", nargs=3, metavar=("NAME", "ROUTE", "TRIPINFO"),
                        action="append",
                        help="Repeat for each explicitly selected scenario; no implicit baseline paths")
    scenario_source.add_argument("--baseline-config-dir",
                                 help="Read free/tight/congested.sumocfg, verifying each configured network")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--signal-remaining-step", type=float, required=True)
    parser.add_argument("--road-flow-step", type=int, default=1)
    parser.add_argument("--road-flow-dense-until", type=int, default=10)
    parser.add_argument("--road-flow-segments", nargs="+", metavar="UPPER:STEP",
                        help="Optional piecewise grid, e.g. 10:1 60:5 100:20 222:50; "
                             "cannot combine with nondefault --road-flow-step/--road-flow-dense-until")
    parser.add_argument("--cap-step", type=float, default=.1389)
    parser.add_argument("--cap-min", type=float)
    parser.add_argument("--cap-max", type=float)
    parser.add_argument("--max-road-flow", type=int, default=222)
    parser.add_argument("--max-lane-flow", type=int, default=80)
    parser.add_argument("--chunk-size", type=int, default=50_000)
    parser.add_argument("--max-rounds", type=int, default=30)
    parser.add_argument("--max-cache-entries", type=int, default=2_000_000,
                        help="Safety ceiling for accumulated unique keys (default: 2000000)")
    parser.add_argument("--simulation-timeout", type=float, default=1800.,
                        help="Positive timeout in seconds for each simulator invocation (default: 1800)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Check contracts, inputs and signatures without loading a model or running C++")
    return parser


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def signature(path):
    path = Path(path).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"Expected a regular file: {path}")
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": core.abc.sha256_file(path)}


def model_signature(directory):
    root = Path(directory).resolve(strict=True)
    files = []
    candidates = sorted(root.rglob("*"))
    print(f"[HASH_MODEL] auditing {root}", flush=True)
    for path in candidates:
        if path.is_symlink():
            raise ValueError(f"Model tree contains a symbolic link; provide a self-contained artifact: {path}")
        if path.is_file():
            item = signature(path)
            item["relative_path"] = path.relative_to(root).as_posix()
            files.append(item)
            if len(files) % 100 == 0:
                print(f"[HASH_MODEL] hashed {len(files)} files", flush=True)
    if not files:
        raise ValueError("Empty model tree")
    print(f"[HASH_MODEL] complete files={len(files)} bytes={sum(item['bytes'] for item in files)}", flush=True)
    return files


def canonical_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False, allow_nan=False).encode("utf-8")).hexdigest()


def route_vehicles(path):
    root = ET.parse(path).getroot()
    if root.tag != "routes" or any(root.findall(tag) for tag in ("flow", "trip", "person", "personFlow")):
        raise ValueError("Demand catch requires explicit <vehicle> routes, not flows/trips/persons")
    definitions = {}
    for route in root.findall("route"):
        name = route.get("id")
        if not name or name in definitions:
            raise ValueError("Missing/duplicate route definition ID")
        definitions[name] = route.get("edges", "").split()
    vehicles = {}
    for vehicle in root.findall("vehicle"):
        name = vehicle.get("id")
        inline = vehicle.find("route")
        edges = inline.get("edges", "").split() if inline is not None else definitions.get(vehicle.get("route"))
        if not name or name in vehicles or not edges:
            raise ValueError(f"Missing/duplicate vehicle ID or unresolved route: {name}")
        vehicles[name] = edges
    if not vehicles:
        raise ValueError("No explicit vehicles in route")
    return vehicles


def trip_durations(path):
    result = {}
    root = ET.parse(path).getroot()
    for trip in root.iter("tripinfo"):
        name = trip.get("id")
        duration = float(trip.get("duration", "nan"))
        if not name or name in result or not math.isfinite(duration) or duration < 0:
            raise ValueError(f"Invalid/duplicate tripinfo record in {path}: {name}")
        result[name] = duration
    if not result:
        raise ValueError(f"Empty tripinfo: {path}")
    return result


def configured_scenarios(directory, network_sha):
    result, configs = [], []
    for name in ("free", "tight", "congested"):
        config = Path(directory).expanduser().resolve(strict=True) / (name + ".sumocfg")
        configs.append(signature(config))
        root = ET.parse(config).getroot()
        paths = []
        for option in ("input/net-file", "input/route-files", "output/tripinfo-output"):
            node = root.find(option)
            value = node.get("value") if node is not None else None
            if not value or "," in value:
                raise ValueError(f"Expected one explicit {option} in {config}")
            paths.append(str((config.parent / value).resolve(strict=True)))
        if signature(paths[0])["sha256"] != network_sha:
            raise ValueError(f"Configured network differs from --net-file: {config}")
        result.append((name, paths[1], paths[2]))
    return result, configs


def prepare(args):
    core._integer(args.max_rounds, "max rounds", 1, 10000)
    core._integer(args.max_cache_entries, "max cache entries", 1, core.MAX_PAYLOAD // RECORD.size)
    core._number(args.simulation_timeout, "simulation timeout", positive=True)
    args.output_dir = str(Path(args.output_dir).expanduser().resolve())
    args.simulator = str(Path(args.simulator).expanduser().resolve(strict=True))
    # Domain counts retain the core structural guard; a sparse cache does not
    # need the minimum payload of one run for every unvisited dense group.
    options = core.build_parser().parse_args([
        "--model-dir", args.model_dir, "--net-file", args.net_file,
        "--output-dir", args.output_dir, "--signal-remaining-step", str(args.signal_remaining_step),
        "--road-flow-step", str(args.road_flow_step), "--road-flow-dense-until", str(args.road_flow_dense_until),
        "--cap-step", str(args.cap_step), "--max-road-flow", str(args.max_road_flow),
        "--max-lane-flow", str(args.max_lane_flow), "--chunk-size", str(args.chunk_size),
        "--max-rows", str(core.MAX_CPP_ROWS),
        *(["--road-flow-segments", *args.road_flow_segments] if args.road_flow_segments is not None else []),
        *(["--expected-threshold", args.expected_threshold] if args.expected_threshold else []),
        *(["--cap-min", str(args.cap_min)] if args.cap_min is not None else []),
        *(["--cap-max", str(args.cap_max)] if args.cap_max is not None else [])])
    plan = core.build_plan(options)
    model_root = Path(plan["model"]["contract_path"]).parent
    if Path(args.output_dir).is_relative_to(model_root):
        raise ValueError("Output directory must not be inside the model artifact tree")
    core._integer(plan["expanded_rows"], "expanded rows", 1, core.MAX_CPP_ROWS)
    core._integer(plan["group_count"], "group count", 1, 2 ** 32 - 1)
    configs = []
    supplied = args.scenario
    if args.baseline_config_dir:
        supplied, configs = configured_scenarios(args.baseline_config_dir, plan["network"]["sha256"])
    names, scenarios = set(), []
    for name, route, truth in supplied:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name) or name in names:
            raise ValueError(f"Scenario name must be unique and path-safe: {name}")
        names.add(name)
        route_sig, truth_sig = signature(route), signature(truth)
        vehicles = route_vehicles(route_sig["path"])
        actual = trip_durations(truth_sig["path"])
        if set(vehicles) != set(actual):
            raise ValueError(f"Route/truth vehicle IDs differ: {name}")
        scenarios.append({"name": name, "route": route_sig, "truth": truth_sig,
                          "vehicles": vehicles, "truth_durations": actual,
                          "vehicle_count": len(vehicles), "road_count": sum(map(len, vehicles.values()))})
    sources = {"simulator": signature(args.simulator), "network": signature(args.net_file),
               "model_files": model_signature(Path(plan["model"]["contract_path"]).parent),
               "driver": signature(__file__), "exporter": signature(core.__file__),
               "baseline_configs": configs,
               "scenarios": [{key: item[key] for key in ("name", "route", "truth", "vehicle_count", "road_count")}
                             for item in scenarios], "fixed_simulation_arguments": FIXED_ARGUMENTS,
               "plan": plan}
    scope = {"schema": "abc-rg-demand-scope-v1", "sources": sources, "scope_sha256": canonical_sha(sources)}
    return plan, scope, scenarios


class KeyDecoder:
    """Cache only small per-axis catalogs, never the full Cartesian product."""
    def __init__(self, plan):
        self.plan = plan
        self.signals = list(core.iter_signal_points(plan))
        self.lanes = plan["lane_flow_max"] + 1
        self.grid = plan["free_speed_cap_grid"]
        self.caps = self.grid["index_max"] - self.grid["index_min"] + 1
        self.policy = core.validate_road_flow_metadata(plan)
        # Bound cached node sets even when callers select the largest supported
        # flow domain. The same core helper defines full-table and demand keys.
        cached_lanes = min(self.lanes, max(1, 100_000 // (plan["road_flow_max"] + 1)))
        self.legal_road_nodes = lru_cache(maxsize=cached_lanes)(
            lambda lane: frozenset(core.road_flow_nodes(
                lane, plan["road_flow_max"], self.policy.get("step", 1),
                self.policy.get("dense_until", 10), nodes=self.policy.get("nodes"))))

    def __call__(self, key):
        group, road = key
        plan = self.plan
        core._integer(group, "group index", 0, plan["group_count"] - 1)
        core._integer(road, "road flow", 0, plan["road_flow_max"])
        lane, group = group % self.lanes, group // self.lanes
        context, group = group % 6, group // 6
        cap, group = group % self.caps, group // self.caps
        length, signal = group % len(plan["road_lengths"]), group // len(plan["road_lengths"])
        if road not in self.legal_road_nodes(lane):
            raise ValueError(f"Request is not a legal quantized road-flow key: {key}")
        return (plan["road_lengths"][length], road, lane, (self.grid["index_min"] + cap) * self.grid["step"],
                context, *self.signals[signal])


def decode_key(plan, key):
    return KeyDecoder(plan)(key)


def write_cache(path, plan, scope_sha256, values):
    path = Path(path)
    if path.exists():
        raise ValueError(f"Refusing to overwrite cache: {path}")
    if len(values) * RECORD.size > core.MAX_PAYLOAD:
        raise ValueError("Demand cache payload exceeds 512 MiB limit")
    keys, digest, decode = sorted(values), hashlib.sha256(), KeyDecoder(plan)
    for key in keys:
        decode(key)
        value = float(values[key])
        if not math.isfinite(value) or value < 1:
            raise ValueError("Cache prediction must be finite and at least 1 second")
        record = RECORD.pack(*key, value)
        digest.update(record)
    header = dict(plan, format=MAGIC, payload_layout=LAYOUT, entry_count=len(values),
                  scope_sha256=scope_sha256, payload_bytes=len(values) * RECORD.size,
                  payload_sha256=digest.hexdigest())
    encoded = json.dumps(header, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(encoded) > core.MAX_HEADER:
        raise ValueError("Demand cache header exceeds limit")
    with path.open("xb") as stream:
        stream.write(MAGIC.encode("ascii") + b"\n" + encoded + b"\n")
        for key in keys:
            stream.write(RECORD.pack(*key, values[key]))
    return header


def read_requests(path, plan):
    result, count = set(), 0
    decode = KeyDecoder(plan)
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["group_index", "road_flow"]:
            raise ValueError(f"Invalid missing-key CSV schema: {path}")
        for row in reader:
            if None in row or not all(re.fullmatch(r"[0-9]+", row[name] or "") for name in reader.fieldnames):
                raise ValueError(f"Invalid missing-key CSV row: {row}")
            key = int(row["group_index"]), int(row["road_flow"])
            decode(key)
            result.add(key)
            count += 1
    return result, count


def load_predictor(path):
    from autogluon.tabular import TabularPredictor
    return TabularPredictor.load(path)


def predict_keys(predictor, plan, keys, chunk_size):
    keys = sorted(keys)
    result, decode = {}, KeyDecoder(plan)
    for start in range(0, len(keys), chunk_size):
        began = time.perf_counter()
        batch = keys[start:start + chunk_size]
        frame = core.feature_frame([decode(key) for key in batch])
        raw = np.asarray(predictor.predict(frame), dtype=np.float64)
        if raw.shape != (len(batch),) or not np.isfinite(raw).all():
            raise ValueError("Predictor returned nonfinite/wrong-shaped output")
        result.update(zip(batch, np.maximum(raw, 1.).tolist()))
        print(f"[PREDICT] completed={min(start + chunk_size, len(keys))}/{len(keys)} "
              f"chunk_seconds={time.perf_counter() - began:.3f}", flush=True)
    return result


def write_predicted_nodes(path, plan, values):
    """The sole road_flow column is already the representative model input."""
    decoder = KeyDecoder(plan)
    with Path(path).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["group_index", *core.FEATURE_COLS, core.LABEL_COL])
        for key in sorted(values):
            writer.writerow([key[0], *decoder(key), values[key]])


def run_process(command, log_path, timeout=1800.):
    previous = signal.getsignal(signal.SIGTERM)
    process = None
    def interrupted(_signum, _frame):
        raise KeyboardInterrupt("demand catch terminated")
    signal.signal(signal.SIGTERM, interrupted)
    try:
        with Path(log_path).open("x", encoding="utf-8") as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
            return process.wait(timeout=timeout)
    except BaseException as exc:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if isinstance(exc, subprocess.TimeoutExpired):
            raise RuntimeError(f"Simulator exceeded --simulation-timeout {timeout:g}s; log: {log_path}") from exc
        raise
    finally:
        signal.signal(signal.SIGTERM, previous)


def command_for(args, scenario, cache, output, discovery):
    command = [args.simulator, *FIXED_ARGUMENTS, "--sumo-net", str(Path(args.net_file).resolve()),
               "--sumo-route", scenario["route"]["path"], "--sumo-tripinfo", scenario["truth"]["path"],
               "--travel-time-table", str(cache), "--expected-signal-remaining-step", str(args.signal_remaining_step),
               "--cap-context-trace", str(output / "trace.csv"),
               "--prediction-tripinfo-output", str(output / "predictions.xml"),
               "--eval-output", str(output / "evaluation.csv"),
               "--demand-status-output", str(output / "status.json"),
               "--read-num", str(scenario["vehicle_count"])]
    if discovery:
        command.extend(["--demand-requests-output", str(output / "missing.csv")])
    return command


def check_status(path, scope, scenario, discovery=True):
    with Path(path).open(encoding="utf-8-sig") as stream:
        status = json.load(stream, object_pairs_hook=core.abc._unique_json)
    if (status.get("schema") != "abc-rg-demand-status-v1" or
            status.get("scope_sha256") != scope["scope_sha256"] or status.get("complete") is not True):
        raise ValueError(f"Incomplete or wrong-scope simulator status: {path}")
    if status.get("discovery_enabled") is not discovery:
        raise ValueError(f"Simulator discovery mode differs from requested mode: {path}")
    for field, wanted in (("vehicle_count", scenario["vehicle_count"]), ("finished_count", scenario["vehicle_count"]),
                          ("query_count", scenario["road_count"]), ("expected_queries", scenario["road_count"]),
                          ("invalid_count", 0)):
        if type(status.get(field)) is not int or status[field] != wanted:
            raise ValueError(f"Simulator {field} differs from expected {wanted}: {path}")
    for field in ("missing_queries", "missing_unique"):
        core._integer(status.get(field), field, 0, scenario["road_count"])
    if status["missing_unique"] > status["missing_queries"]:
        raise ValueError("missing_unique exceeds missing_queries")
    if status.get("exact") is not (status["missing_queries"] == 0):
        raise ValueError("Simulator exact flag disagrees with missing-query count")
    return status


def check_final(output, scenario, status):
    if status["missing_queries"] or status["missing_unique"]:
        raise ValueError("A formal run contains missing cache keys")
    prediction = trip_durations(output / "predictions.xml")
    if set(prediction) != set(scenario["vehicles"]):
        raise ValueError("Final predicted tripinfo IDs do not match route/truth")
    expected = {(name, index): edge for name, edges in scenario["vehicles"].items() for index, edge in enumerate(edges)}
    seen = set()
    with (output / "trace.csv").open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not {"sumoVehicleID", "route_index", "sumoRoadID"}.issubset(reader.fieldnames or []):
            raise ValueError("Final trace missing vehicle/road identity columns")
        for row in reader:
            key = row["sumoVehicleID"], int(row["route_index"])
            if key in seen or key not in expected or row["sumoRoadID"] != expected[key]:
                raise ValueError(f"Duplicate or wrong final vehicle/road trace: {key}")
            seen.add(key)
    if set(expected) != seen:
        raise ValueError("Final trace does not cover every vehicle x route_index")
    errors = [prediction[name] - scenario["truth_durations"][name] for name in sorted(prediction)]
    count = len(errors)
    return {"name": scenario["name"], "vehicle_count": count, "road_count": len(seen),
            "mae_seconds": math.fsum(map(abs, errors)) / count, "bias_seconds": math.fsum(errors) / count,
            "sum_absolute_error_seconds": math.fsum(map(abs, errors)), "sum_error_seconds": math.fsum(errors),
            "prediction_tripinfo": signature(output / "predictions.xml"), "trace": signature(output / "trace.csv")}


def verify_sources(scope):
    sources = scope["sources"]
    files = [sources["simulator"], sources["network"], sources["driver"], sources["exporter"],
             *sources["model_files"], *sources["baseline_configs"]]
    for scenario in sources["scenarios"]:
        files.extend([scenario["route"], scenario["truth"]])
    print(f"[VERIFY_SOURCES] rehashing {len(files)} input files", flush=True)
    for index, before in enumerate(files):
        after = signature(before["path"])
        if after["bytes"] != before["bytes"] or after["sha256"] != before["sha256"]:
            raise ValueError(f"Input changed during demand catch: {before['path']}")
        if (index + 1) % 100 == 0:
            print(f"[VERIFY_SOURCES] verified={index + 1}/{len(files)}", flush=True)
    model_root = Path(sources["plan"]["model"]["contract_path"]).parent
    current = {str(path.resolve()) for path in model_root.rglob("*") if path.is_file()}
    if current != {item["path"] for item in sources["model_files"]}:
        raise ValueError("Model file tree changed during demand catch")
    print("[VERIFY_SOURCES] all input signatures unchanged", flush=True)


def run(args, predictor_loader=None, runner=None):
    plan, scope, scenarios = prepare(args)
    policy = plan["road_flow_quantization"]
    road_grid = (f"road_flow_grid=explicit road_flow_nodes={len(policy['nodes'])} "
                 if "nodes" in policy else
                 f"road_flow_step={policy['step']}veh dense_until={policy['dense_until']}veh ")
    print(f"[PLAN] scenarios={len(scenarios)} signal_step={args.signal_remaining_step:g}s "
          f"{road_grid}"
          f"theoretical_prediction_rows={plan['prediction_rows']} scope={scope['scope_sha256']}", flush=True)
    if args.dry_run:
        print("[DRY_RUN] Inputs/contracts checked; no model loaded, no C++ run, no output written", flush=True)
        return {"status": "dry_run", "scope": scope, "plan": plan}
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "scope.json", scope)
    loader = predictor_loader or load_predictor
    invoke = runner or partial(run_process, timeout=args.simulation_timeout)
    values, predictor, rounds = {}, None, []
    try:
        for number in range(1, args.max_rounds + 1):
            round_started = time.perf_counter()
            cache = output / f"cache_{number:03d}.demand"
            write_cache(cache, plan, scope["scope_sha256"], values)
            missing, records = set(), []
            for scenario in scenarios:
                directory = output / f"round_{number:03d}" / scenario["name"]
                directory.mkdir(parents=True)
                command = command_for(args, scenario, cache, directory, True)
                write_json(directory / "command.json", {"command": command, "result_role": "provisional_only"})
                sim_started = time.perf_counter()
                code = invoke(command, directory / "simulator.log")
                sim_seconds = time.perf_counter() - sim_started
                if code != 0:
                    raise RuntimeError(f"Simulator failed ({code}): {directory / 'simulator.log'}")
                status = check_status(directory / "status.json", scope, scenario)
                keys, request_rows = read_requests(directory / "missing.csv", plan)
                if len(keys) != status["missing_unique"] or request_rows not in (len(keys), status["missing_queries"]):
                    raise ValueError("Missing-key CSV/status counts disagree")
                if any(key in values for key in keys):
                    raise ValueError("Simulator requested a key already present in the supplied cache")
                missing.update(keys)
                records.append({"name": scenario["name"], "status": status, "simulation_seconds": sim_seconds})
                print(f"[SIMULATION] round={number} scenario={scenario['name']} seconds={sim_seconds:.3f} "
                      f"missing={status['missing_queries']}", flush=True)
            record = {"round": number, "cached_entries": len(values), "missing_unique_union": len(missing),
                      "scenarios": records, "simulation_round_seconds": time.perf_counter() - round_started,
                      "result_role": "provisional_only"}
            rounds.append(record)
            write_json(output / f"round_{number:03d}" / "round.json", record)
            print(f"[PROVISIONAL] round={number} cache={len(values)} missing_union={len(missing)}; not MAE results", flush=True)
            if not missing:
                break
            if number == args.max_rounds:
                raise RuntimeError(f"No zero-miss fixed point within --max-rounds {args.max_rounds}; no formal MAE")
            if len(values) + len(missing) > args.max_cache_entries:
                raise RuntimeError(f"Accumulated keys exceed --max-cache-entries {args.max_cache_entries}; no formal MAE")
            if predictor is None:
                load_started = time.perf_counter()
                print("[LOAD_MODEL] loading trusted AutoGluon predictor once", flush=True)
                predictor = loader(plan["model"]["predictor_path"])
                core.validate_predictor(predictor, plan)
                print(f"[LOAD_MODEL] seconds={time.perf_counter() - load_started:.3f}", flush=True)
            predict_started = time.perf_counter()
            new_values = predict_keys(predictor, plan, missing, args.chunk_size)
            write_predicted_nodes(output / f"round_{number:03d}" / "predicted_nodes.csv", plan, new_values)
            values.update(new_values)
            write_json(output / f"round_{number:03d}" / "prediction.json",
                       {"new_keys": len(missing), "prediction_seconds": time.perf_counter() - predict_started})
        results = []
        for scenario in scenarios:
            directory = output / "final" / scenario["name"]
            directory.mkdir(parents=True)
            command = command_for(args, scenario, cache, directory, False)
            write_json(directory / "command.json", {"command": command, "result_role": "strict_validation"})
            sim_started = time.perf_counter()
            code = invoke(command, directory / "simulator.log")
            if code != 0:
                raise RuntimeError(f"Strict final simulator failed ({code}): {directory / 'simulator.log'}")
            status = check_status(directory / "status.json", scope, scenario, discovery=False)
            result = check_final(directory, scenario, status)
            result["simulation_seconds"] = time.perf_counter() - sim_started
            results.append(result)
        verify_sources(scope)
        count = sum(item["vehicle_count"] for item in results)
        summary = {"schema": "abc-rg-demand-result-v1", "status": "complete", "scope_sha256": scope["scope_sha256"],
                   "source_label": plan["source_label"], "cache": signature(cache), "entry_count": len(values),
                   "discovery_rounds": len(rounds), "final_strict_missing_queries": 0, "scenarios": results,
                   "execution_limits": {"max_rounds": args.max_rounds, "max_cache_entries": args.max_cache_entries,
                                        "simulation_timeout_seconds": args.simulation_timeout},
                   "discovery_placeholder": "SF-on speed-net, used only for explicitly provisional missing-key runs; "
                                            "final trajectories use fully cached model predictions without fallback.",
                   "combined": {"vehicle_count": count,
                                "mae_seconds": math.fsum(item["sum_absolute_error_seconds"] for item in results) / count,
                                "bias_seconds": math.fsum(item["sum_error_seconds"] for item in results) / count},
                   "scope_warning": "Only these completed scenarios are certified; this is not a full-domain table."}
        write_json(output / "summary.json", summary)
        for result in results:
            print(f"[RESULT] {result['name']} vehicles={result['vehicle_count']} "
                  f"MAE={result['mae_seconds']:.6f}s bias={result['bias_seconds']:+.6f}s", flush=True)
        print(f"[RESULT] combined vehicles={count} MAE={summary['combined']['mae_seconds']:.6f}s "
              f"bias={summary['combined']['bias_seconds']:+.6f}s", flush=True)
        print(f"[COMPLETE] strict replay verified; entries={len(values)} summary={output / 'summary.json'}", flush=True)
        return summary
    except BaseException as exc:
        write_json(output / "failure.json", {"status": "failed", "exception": type(exc).__name__, "message": str(exc),
                                            "formal_results_accepted": False, "completed_discovery_rounds": len(rounds)})
        raise


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        run(args)
    except KeyboardInterrupt:
        print("[INTERRUPTED] Partial outputs retained; no results accepted", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError, ET.ParseError) as exc:
        print(f"[FAILED] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
