#!/usr/bin/env python3
"""Appendable, integrity-checked speed-net SF-on parameter sweep.

Grid JSON: [{"headway": 0.2, "delay_max": 3, "name": "optional label"}].
Dataset manifest JSON (optional): [{"name": "case", "config": "case.sumocfg",
"expected_vehicles": 123}]; paths are relative to the manifest. A dict with a
"datasets" or "cases" list is also accepted. No SUMO or TraCI collection is executed.
"""

import argparse
import concurrent.futures
from decimal import Decimal, InvalidOperation
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET


FORMAT_VERSION = 1
COMMON_ARGUMENTS = [
    "--use-sumo", "--travel-time-mode", "speed-net",
    "--speed-net-speed-factor", "on", "--speed-net-components", "both",
    "--speed-net-queue-trim", "off", "--lane-discharge-interval", "2",
    "--intersection-crossing-time", "3", "--movement-queue-order", "arrival-time",
]
DEFAULT_SCENARIOS = {"free": 7778, "tight": 17778, "congested": 19444}
SOURCE_NAMES = [
    "main.cpp", "head.h", "simulation.cpp", "data_preparation.cpp",
    "data_cleaning.cpp", "config_defaults.h", "CMakeLists.txt",
]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, payload):
    path = Path(path)
    pending = path.with_suffix(path.suffix + ".pending")
    pending.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    pending.replace(path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def parameter_decimal(value):
    if isinstance(value, bool):
        raise ValueError("Boolean is not a parameter")
    try:
        number = Decimal(str(value))
    except InvalidOperation as error:
        raise ValueError(f"Invalid parameter: {value!r}") from error
    if not number.is_finite() or number < 0 or number > Decimal("1000000"):
        raise ValueError(f"Parameter must be finite in [0, 1000000]: {value!r}")
    if not number:
        return "0"
    result = format(number.normalize(), "f")
    if len(result) > 40:
        raise ValueError(f"Parameter has excessive decimal precision: {value!r}")
    return result


def parameter_id(headway, delay_max):
    return "h" + parameter_decimal(headway).replace(".", "p") + "_d" + parameter_decimal(delay_max).replace(".", "p")


def load_grid(path):
    payload = read_json(path)
    if isinstance(payload, dict):
        payload = payload["pairs"]
    if not isinstance(payload, list) or not payload:
        raise ValueError("Grid must contain a nonempty list of parameter objects")
    pairs = {}
    for item in payload:
        if not isinstance(item, dict):
            raise ValueError("Grid entries must be objects with headway and delay_max")
        headway = parameter_decimal(item["headway"])
        delay_max = parameter_decimal(item["delay_max"])
        identifier = parameter_id(headway, delay_max)
        pair = pairs.setdefault(identifier, {"id": identifier, "headway": headway, "delay_max": delay_max, "labels": []})
        label = item.get("name", item.get("case_name"))
        if label is not None and str(label) not in pair["labels"]:
            pair["labels"].append(str(label))
    return list(pairs.values())


def read_records(path, tag):
    records = {}
    for _, element in ET.iterparse(path, events=("end",)):
        if element.tag == tag:
            identifier = element.attrib["id"]
            if identifier in records:
                raise ValueError(f"Duplicate {tag} ID {identifier!r} in {path}")
            records[identifier] = dict(element.attrib)
        element.clear()
    return records


def configured_file(config, xpath):
    node = ET.parse(config).find(xpath)
    if node is None or not node.get("value") or "," in node.get("value"):
        raise ValueError(f"Expected exactly one file at {xpath}: {config}")
    return (config.parent / node.get("value")).resolve()


def load_inputs(repo, dataset_manifest=None):
    if dataset_manifest:
        rows = read_json(dataset_manifest)
        if isinstance(rows, dict):
            rows = rows["datasets"] if "datasets" in rows else rows["cases"]
        if not isinstance(rows, list) or not rows:
            raise ValueError("Dataset manifest must have a nonempty datasets list")
        base = Path(dataset_manifest).resolve().parent
    else:
        base = repo / "data/variants/speed_factor/sumocfg"
        rows = [{"name": name, "config": name + ".sumocfg", "expected_vehicles": size}
                for name, size in DEFAULT_SCENARIOS.items()]
    inputs = {}
    hashes = {}
    for row in rows:
        name = row["name"]
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name) or name in inputs:
            raise ValueError(f"Duplicate or unsafe dataset name: {name!r}")
        config = (base / row["config"]).resolve()
        net = configured_file(config, "input/net-file")
        route = configured_file(config, "input/route-files")
        truth = configured_file(config, "output/tripinfo-output")
        route_records = read_records(route, "vehicle")
        truth_records = read_records(truth, "tripinfo")
        expected = row.get("expected_vehicles", len(route_records))
        if not isinstance(expected, int) or isinstance(expected, bool) or expected <= 0:
            raise ValueError(f"Invalid expected vehicle count for {name}")
        if len(route_records) != expected or route_records.keys() != truth_records.keys():
            raise ValueError(f"{name}: route/truth vehicle coverage mismatch")
        for identifier, vehicle in route_records.items():
            factor = float(vehicle.get("speedFactor", "nan"))
            if not math.isfinite(factor) or factor <= 0:
                raise ValueError(f"{name}/{identifier}: explicit finite positive SF is required")
        durations = {}
        for identifier, vehicle in truth_records.items():
            duration = float(vehicle["duration"])
            if not math.isfinite(duration) or duration < 0:
                raise ValueError(f"{name}/{identifier}: invalid truth duration")
            durations[identifier] = duration
        for path in (config, net, route, truth):
            hashes[str(path)] = sha256(path)
        inputs[name] = {"config": str(config), "net": str(net), "route": str(route),
                        "truth": str(truth), "expected_vehicles": expected, "truth_durations": durations}
    if dataset_manifest:
        hashes[str(Path(dataset_manifest).resolve())] = sha256(dataset_manifest)
    return inputs, hashes


def quantile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return None
    position = probability * (len(ordered) - 1)
    left, right = math.floor(position), math.ceil(position)
    return ordered[left] + (position - left) * (ordered[right] - ordered[left])


def metrics(errors):
    values = list(errors)
    if not values or any(not math.isfinite(value) for value in values):
        raise ValueError("Metrics require nonempty finite errors")
    absolute = [abs(value) for value in values]
    return {
        "vehicles": len(values), "mae_s": math.fsum(absolute) / len(values),
        "bias_s": math.fsum(values) / len(values),
        "rmse_s": math.sqrt(math.fsum(value * value for value in values) / len(values)),
        "sum_abs_error_s": math.fsum(absolute), "sum_error_s": math.fsum(values),
        "p50_abs_error_s": quantile(absolute, 0.5), "p95_abs_error_s": quantile(absolute, 0.95),
        "p99_abs_error_s": quantile(absolute, 0.99), "max_abs_error_s": max(absolute),
        "too_fast_vehicles": sum(value < -1e-6 for value in values),
        "too_slow_vehicles": sum(value > 1e-6 for value in values),
    }


def validate_log(log_path, expected):
    log = Path(log_path).read_text(encoding="utf-8", errors="replace")
    finished = re.findall(r"Finished vehicles:\s*(\d+)\s*/\s*(\d+)", log)
    invalid = re.findall(r"Invalid vehicles:\s*(\d+)", log)
    if not finished or any((int(done), int(total)) != (expected, expected) for done, total in finished):
        raise ValueError(f"Missing or incomplete Finished vehicles evidence: {log_path}")
    if not invalid or any(int(value) != 0 for value in invalid):
        raise ValueError(f"Missing or nonzero Invalid vehicles evidence: {log_path}")


def evaluate_run(run_dir, input_data):
    validate_log(run_dir / "run.log", input_data["expected_vehicles"])
    prediction = read_records(run_dir / "prediction_tripinfo.xml", "tripinfo")
    truth = input_data["truth_durations"]
    if prediction.keys() != truth.keys():
        raise ValueError(f"Prediction/truth coverage mismatch in {run_dir}")
    errors = []
    for identifier, row in prediction.items():
        duration = float(row["duration"])
        if not math.isfinite(duration) or duration < 0:
            raise ValueError(f"Invalid predicted duration for {identifier}: {duration}")
        errors.append(duration - truth[identifier])
    return metrics(errors), errors


def verify_hashes(hashes):
    for name, expected in hashes.items():
        if not Path(name).is_file() or sha256(name) != expected:
            raise ValueError(f"File missing or changed: {name}")


def output_hashes(run_dir):
    return {str(path.relative_to(run_dir)): sha256(path) for path in sorted(run_dir.rglob("*"))
            if path.is_file() and path.name not in ("run.json", "run.json.pending")}


def verify_success(run_dir, pair, scenario, fingerprint):
    record = read_json(run_dir / "run.json")
    if record.get("status") != "success":
        return None
    if record.get("fingerprint") != fingerprint or record.get("pair") != pair or record.get("scenario") != scenario:
        raise ValueError(f"Successful run metadata changed: {run_dir}")
    if record.get("output_hashes") != output_hashes(run_dir):
        raise ValueError(f"Successful run output missing, added, or changed: {run_dir}")
    return record


def execute_run(output, binary, pair, scenario, input_data, fingerprint, timeout):
    # Attempts are immutable. A failed/interrupted attempt is retained when resumed.
    case_dir = output / "runs" / pair["id"] / scenario
    case_dir.mkdir(parents=True, exist_ok=True)
    successful = []
    for attempt in sorted(case_dir.iterdir()):
        if not attempt.is_dir() or not re.fullmatch(r"attempt_\d{4}", attempt.name):
            raise ValueError(f"Unrecognized file in managed run directory: {attempt}")
        if not (attempt / "run.json").is_file():
            raise ValueError(f"Unowned/incomplete attempt metadata: {attempt}")
        record = verify_success(attempt, pair, scenario, fingerprint)
        if record:
            successful.append((attempt, record))
    if successful:
        if len(successful) != 1:
            raise ValueError(f"Multiple successful attempts found: {case_dir}")
        run_dir, record = successful[0]
        result, _ = evaluate_run(run_dir, input_data)
        if result != record["independent_metrics"]:
            raise ValueError(f"Successful run metrics changed: {run_dir}")
        print(f"[REUSE] {pair['id']}/{scenario}: MAE={result['mae_s']:.6f}", flush=True)
        return record
    run_dir = case_dir / f"attempt_{len(list(case_dir.iterdir())) + 1:04d}"
    run_dir.mkdir(exist_ok=False)
    command = [str(binary)] + COMMON_ARGUMENTS + [
        "--sumo-net", input_data["net"], "--sumo-route", input_data["route"],
        "--sumo-tripinfo", input_data["truth"], "--eval-output", str(run_dir / "sumo_eval.csv"),
        "--prediction-tripinfo-output", str(run_dir / "prediction_tripinfo.xml"),
        "--speed-net-headway", pair["headway"], "--speed-net-delay-max", pair["delay_max"],
    ]
    record = {"status": "running", "pair": pair, "scenario": scenario, "command": command,
              "cwd": str(run_dir), "fingerprint": fingerprint, "started_unix": time.time()}
    write_json(run_dir / "run.json", record)
    started = time.perf_counter()
    environment = dict(os.environ)
    environment.pop("USE_SUMO_NET", None)
    environment.pop("SUMO_NET_PATH", None)
    try:
        with (run_dir / "run.log").open("wb") as log:
            process = subprocess.run(command, cwd=run_dir, stdout=log, stderr=subprocess.STDOUT,
                                     env=environment, timeout=timeout, check=False,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        record["returncode"] = process.returncode
        if process.returncode:
            raise RuntimeError(f"Simulator exit {process.returncode}; see {run_dir / 'run.log'}")
        result, _ = evaluate_run(run_dir, input_data)
        record.update(status="success", independent_metrics=result)
    except Exception as error:
        record.update(status="failed", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        record["elapsed_seconds"] = time.perf_counter() - started
        record["output_hashes"] = output_hashes(run_dir)
        write_json(run_dir / "run.json", record)
    print(f"[OK] {pair['id']}/{scenario}: N={result['vehicles']} MAE={result['mae_s']:.6f} bias={result['bias_s']:.6f}", flush=True)
    return record


def prepare_experiment(output, fingerprint, requested_pairs, resume):
    manifest_path = output / "experiment.json"
    if output.exists():
        if not resume or not manifest_path.is_file():
            raise ValueError("Output exists: refuse overwrite; --resume requires this tool's experiment.json")
        experiment = read_json(manifest_path)
        if experiment.get("fingerprint") != fingerprint or experiment.get("format_version") != FORMAT_VERSION:
            raise ValueError("Cannot resume: script, sources, binary, inputs, or constants changed")
    else:
        output.mkdir(parents=True, exist_ok=False)
        experiment = {"format_version": FORMAT_VERSION, "fingerprint": fingerprint,
                      "created_unix": time.time(), "parameters": {}, "requests": []}
    for pair in requested_pairs:
        existing = experiment["parameters"].get(pair["id"])
        if existing and any(existing[key] != pair[key] for key in ("id", "headway", "delay_max")):
            raise ValueError(f"Parameter collision: {pair['id']}")
        # Labels are descriptive only; preserve first-seen metadata so prior runs remain immutable.
        if not existing:
            experiment["parameters"][pair["id"]] = pair
    experiment["status"] = "running"
    write_json(manifest_path, experiment)
    return experiment


def summarize(output, experiment, inputs):
    parameters = []
    incomplete = []
    for pair in experiment["parameters"].values():
        datasets = {}
        pooled_errors = []
        for scenario, input_data in inputs.items():
            case_dir = output / "runs" / pair["id"] / scenario
            attempts = sorted(case_dir.glob("attempt_*/run.json")) if case_dir.exists() else []
            successful = [(path, read_json(path)) for path in attempts if read_json(path).get("status") == "success"]
            if len(successful) > 1:
                raise ValueError(f"Multiple successes for {pair['id']}/{scenario}")
            if not successful:
                continue
            path, record = successful[0]
            verify_success(path.parent, pair, scenario, experiment["fingerprint"])
            result, errors = evaluate_run(path.parent, input_data)
            if result != record["independent_metrics"]:
                raise ValueError(f"Stored metrics mismatch: {path}")
            datasets[scenario] = {**result, "run_dir": str(path.parent), "elapsed_seconds": record["elapsed_seconds"]}
            pooled_errors.extend(errors)
        section = {**pair, "datasets": datasets}
        if len(datasets) != len(inputs):
            section["missing_scenarios"] = [name for name in inputs if name not in datasets]
            incomplete.append(section)
            continue
        section["pooled"] = metrics(pooled_errors)
        section["equal_scenario_mae_s"] = math.fsum(value["mae_s"] for value in datasets.values()) / len(datasets)
        section["equal_scenario_bias_s"] = math.fsum(value["bias_s"] for value in datasets.values()) / len(datasets)
        section["worst_scenario_mae_s"] = max(value["mae_s"] for value in datasets.values())
        parameters.append(section)
    parameters.sort(key=lambda section: (section["pooled"]["mae_s"], section["id"]))
    summary = {"format_version": FORMAT_VERSION, "metric_definition": "error = C++ tripinfo duration - SUMO tripinfo duration; seconds; excludes initial departure delay",
               "pooled_definition": "Every vehicle is equally weighted across all datasets; not mean of scenario MAEs",
               "precision_note": "Independent XML-duration metrics; existing C++ exporter uses six significant digits",
               "fingerprint": experiment["fingerprint"], "dataset_counts": {name: value["expected_vehicles"] for name, value in inputs.items()},
               "completed_parameter_count": len(parameters), "requested_parameter_count": len(experiment["parameters"]),
               "parameters": parameters, "incomplete_parameters": incomplete,
               "best_pooled_mae_parameter": parameters[0]["id"] if parameters else None}
    write_json(output / "summary.json", summary)
    names = list(inputs)
    lines = ["# SF-on speed-net parameter sweep", "", summary["metric_definition"] + ".",
             "Pooled results weight vehicles equally. Equal-scenario MAE weights datasets equally.",
             "SF on; both components; queue trimming off; discharge 2 s; crossing 3 s; arrival-time queue order.",
             "This is parameter selection on these scenarios, not an unbiased held-out performance estimate.", "",
             "| Rank | ID | Headway | Delay max | " + " | ".join(name + " MAE / bias" for name in names) + " | Pooled MAE | Pooled bias | RMSE | P95 absolute | Equal-scenario MAE |",
             "| ---: | --- | ---: | ---: | " + " | ".join("---:" for _ in names) + " | ---: | ---: | ---: | ---: | ---: |"]
    for rank, item in enumerate(parameters, 1):
        pooled = item["pooled"]
        cells = [f"{item['datasets'][name]['mae_s']:.6f} / {item['datasets'][name]['bias_s']:+.6f}" for name in names]
        lines.append(f"| {rank} | {item['id']} | {item['headway']} | {item['delay_max']} | " + " | ".join(cells) +
                     f" | {pooled['mae_s']:.6f} | {pooled['bias_s']:+.6f} | {pooled['rmse_s']:.6f} | {pooled['p95_abs_error_s']:.6f} | {item['equal_scenario_mae_s']:.6f} |")
    lines += ["", f"Completed parameter sets: {len(parameters)} / {len(experiment['parameters'])}.",
              "Full commands, source/input/binary hashes, immutable attempts, logs, predictions and automatic evaluation artifacts are retained.", ""]
    (output / "overview.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--grid", type=Path, required=True)
    parser.add_argument("--dataset-manifest", type=Path)
    parser.add_argument("--workers", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    if args.workers < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("workers and timeout must be positive")
    args.repo, args.binary, args.output, args.grid = [path.resolve() for path in (args.repo, args.binary, args.output, args.grid)]
    requested_pairs = load_grid(args.grid)
    inputs, hashes = load_inputs(args.repo, args.dataset_manifest)
    for name in SOURCE_NAMES:
        path = args.repo / name
        if path.is_file():
            hashes[str(path)] = sha256(path)
    hashes[str(args.binary)] = sha256(args.binary)
    hashes[str(Path(__file__).resolve())] = sha256(__file__)
    for source in hashes:
        if args.output == Path(source) or args.output in Path(source).parents:
            raise ValueError(f"Output must not contain a source input: {source}")
    fingerprint = {"hashes": hashes, "common_arguments": COMMON_ARGUMENTS,
                   "dataset_counts": {name: value["expected_vehicles"] for name, value in inputs.items()}}
    experiment = prepare_experiment(args.output, fingerprint, requested_pairs, args.resume)
    request_number = len(experiment["requests"]) + 1
    request = {"request_number": request_number, "created_unix": time.time(), "grid": str(args.grid),
               "grid_sha256": sha256(args.grid), "pairs": requested_pairs, "workers": args.workers, "timeout": args.timeout}
    request_dir = args.output / "requests"
    request_dir.mkdir(exist_ok=True)
    request_path = request_dir / f"request_{request_number:04d}.json"
    if request_path.exists():
        raise ValueError(f"Refusing to overwrite grid request: {request_path}")
    write_json(request_path, request)
    experiment["requests"].append(str(request_path.relative_to(args.output)))
    write_json(args.output / "experiment.json", experiment)
    failures = []
    started = time.perf_counter()
    # Include all earlier incomplete parameter sets during resume; all successes are verified/reused.
    pairs = list(experiment["parameters"].values())
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(execute_run, args.output, args.binary, pair, name, data, fingerprint, args.timeout):
                   f"{pair['id']}/{name}" for pair in pairs for name, data in inputs.items()}
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as error:
                failures.append({"case": futures[future], "error": f"{type(error).__name__}: {error}"})
                print(f"[FAILED] {futures[future]}: {error}", flush=True)
    verify_hashes(hashes)
    summary = summarize(args.output, experiment, inputs)
    experiment.update(status="failed" if failures else "success", last_elapsed_seconds=time.perf_counter() - started,
                      failures=failures, completed_parameter_count=summary["completed_parameter_count"],
                      source_hashes_unchanged=True)
    write_json(args.output / "experiment.json", experiment)
    print(json.dumps({"status": experiment["status"], "completed_parameters": summary["completed_parameter_count"],
                      "best_parameter": summary["best_pooled_mae_parameter"], "summary": str(args.output / "summary.json")}), flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
