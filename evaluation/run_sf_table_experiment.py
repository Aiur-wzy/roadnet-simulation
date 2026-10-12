#!/usr/bin/env python3
"""Fixed six-threshold SF-table experiment against frozen SF-on Speed-net.

Runs C++ only. A fresh output directory is mandatory. The legacy experiment
helper is imported explicitly from --repo rather than the process search path.
"""

import argparse
import concurrent.futures
import importlib.util
import json
import math
import os
from pathlib import Path
from roadnet_paths import source_path
import re
import subprocess
import time


THRESHOLDS = ("0p1", "0p5", "1", "1p5", "2", "3")
SCENARIOS = ("free", "tight", "congested")
FORMAT = "CAMS_SF_TABLE_V1"
BASELINE = "speed_net_sf"
OLD_BASELINE = "old_speed_net_sf"
COMMON_ARGUMENTS = [
    "--use-sumo", "--speed-net-components", "both",
    "--speed-net-queue-trim", "off", "--lane-discharge-interval", "2",
    "--intersection-crossing-time", "3", "--movement-queue-order", "arrival-time",
    "--speed-net-headway", "0.12", "--speed-net-delay-max", "3",
]


def import_helper(repo):
    path = source_path("scripts/run_speednet_sf_experiment.py", root=repo)
    if not path.is_file():
        path = repo / "scripts/run_speednet_sf_experiment.py"
    spec = importlib.util.spec_from_file_location("frozen_speednet_sf_helper", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, path


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_tables(table_root, inputs, helper):
    tables = {}
    hashes = {}
    network_hashes = {helper.sha256(value["net"]) for value in inputs.values()}
    if len(network_hashes) != 1:
        raise ValueError("The fixed experiment expects identical network content across scenarios")
    for threshold in THRESHOLDS:
        table = table_root / f"gt_{threshold}/table.txt"
        manifest_path = table.parent / "manifest.json"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        if manifest.get("status") != "complete" or manifest.get("table") != "table.txt":
            raise ValueError(f"Incomplete or unexpected table manifest: {manifest_path}")
        actual_hash = helper.sha256(table)
        if actual_hash != manifest.get("table_sha256"):
            raise ValueError(f"Table content does not match export SHA-256: {table}")
        plan = manifest["plan"]
        if plan.get("format") != FORMAT or plan.get("source_label") != f"travel_time_label_gt_{threshold}":
            raise ValueError(f"Table schema/threshold mismatch: {table}")
        if plan.get("network", {}).get("sha256") not in network_hashes:
            raise ValueError(f"Export network differs from validation network: {table}")
        if manifest.get("rows", 0) <= 0 or manifest["rows"] != plan.get("planned_rows"):
            raise ValueError(f"Export row-count metadata mismatch: {table}")
        with table.open(encoding="utf-8-sig") as stream:
            if stream.readline().strip() != FORMAT:
                raise ValueError(f"Unrecognized SF table magic: {table}")
            if json.loads(stream.readline()) != plan:
                raise ValueError(f"Table metadata differs from export manifest: {table}")
            columns = stream.readline().split()
            if columns != plan.get("columns"):
                raise ValueError(f"Table columns differ from export manifest: {table}")
        grid = plan["speed_factor_grid"]
        scale = float(grid["scale"])
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError(f"Invalid SF scale: {table}")
        audit = {}
        pooled_errors = []
        for scenario, input_data in inputs.items():
            errors = []
            factors = input_data["speed_factors"]
            for vehicle, sf in factors.items():
                if sf < grid["min"] - 1e-12 or sf > grid["max"] + 1e-12:
                    raise ValueError(f"SF outside export grid: {scenario}/{vehicle}: {sf} in {table}")
                index = math.floor(sf * scale + 0.5 + 1e-9)
                if not grid["index_min"] <= index <= grid["index_max"]:
                    raise ValueError(f"Quantized SF outside export grid: {scenario}/{vehicle}")
                errors.append(index / scale - sf)
            pooled_errors.extend(errors)
            audit[scenario] = {
                "vehicles": len(factors), "raw_min": min(factors.values()), "raw_max": max(factors.values()),
                "mean_absolute_sf_quantization": math.fsum(map(abs, errors)) / len(errors),
                "max_absolute_sf_quantization": max(map(abs, errors)),
                "mean_signed_sf_quantization": math.fsum(errors) / len(errors),
            }
        audit["pooled"] = {
            "vehicles": len(pooled_errors),
            "mean_absolute_sf_quantization": math.fsum(map(abs, pooled_errors)) / len(pooled_errors),
            "max_absolute_sf_quantization": max(map(abs, pooled_errors)),
            "mean_signed_sf_quantization": math.fsum(pooled_errors) / len(pooled_errors),
        }
        tables[f"gt_{threshold}"] = {
            "path": str(table), "manifest": str(manifest_path), "rows": manifest["rows"],
            "plan": plan, "sf_quantization_audit": audit, "sha256": actual_hash,
        }
        hashes[str(table)] = actual_hash
        hashes[str(manifest_path)] = helper.sha256(manifest_path)
    return tables, hashes


def parse_run_log(text, label, expected_vehicles):
    def one(pattern, name):
        matches = re.findall(pattern, text, flags=re.MULTILINE)
        if len(matches) != 1:
            raise ValueError(f"Expected exactly one {name} log entry, found {len(matches)}")
        return matches[0]

    finished, total = one(r"^Finished vehicles:\s*(\d+)\s*/\s*(\d+)\s*$", "finished vehicle count")
    invalid = int(one(r"^Invalid vehicles:\s*(\d+)\s*$", "invalid vehicle count"))
    if int(finished) != expected_vehicles or int(total) != expected_vehicles or invalid:
        raise ValueError(f"Incomplete/invalid simulation: {finished}/{total}, invalid={invalid}")
    mode = one(r"^\[TravelTime\] mode:\s*(\S+)\s*$", "travel-time mode")
    result = {"finished_vehicles": int(finished), "total_vehicles": int(total), "invalid_vehicles": invalid, "mode": mode}
    if label.startswith("gt_"):
        if mode != "table":
            raise ValueError(f"Table task used unexpected mode: {mode}")
        table_format = one(r"^\[TravelTime\] table format:\s*(\S+)\s*$", "table format")
        hits = int(one(r"^\[TravelTime\] table hits:\s*(\d+)\s*$", "table hits"))
        misses = int(one(r"^\[TravelTime\] table misses:\s*(\d+)\s*$", "table misses"))
        if table_format != FORMAT or hits <= 0 or misses:
            raise ValueError(f"SF table was not used exclusively: format={table_format}, hits={hits}, misses={misses}")
        result.update(table_format=table_format, table_hits=hits, table_misses=misses)
        fallback = one(r"^\[TravelTime\] fallback:\s*(.+?)\s*$", "table fallback policy")
        if fallback != "disabled (strict SF table)":
            raise ValueError(f"SF table fallback was not explicitly disabled: {fallback}")
        counts = one(r"^\[SFTable\] queries=(\d+) road_flow_outside_training=(\d+) lane_flow_outside_training=(\d+) road_flow_max_seen=(\d+) lane_flow_max_seen=(\d+)\s*$", "SF training-range audit")
        names = ("sf_queries", "road_flow_outside_training", "lane_flow_outside_training", "road_flow_max_seen", "lane_flow_max_seen")
        result.update(zip(names, map(int, counts)))
        if result["sf_queries"] != hits or any(result[name] > hits for name in names[1:3]):
            raise ValueError("SF training-range query counts are inconsistent with successful table hits")
        for feature in ("road_flow", "lane_flow"):
            result[f"{feature}_outside_training_fraction"] = result[f"{feature}_outside_training"] / hits
        result["fallback_policy"] = fallback
    elif mode != "speed-net":
        raise ValueError(f"Baseline task used unexpected mode: {mode}")
    return result


def execute_run(output, binary, scenario, label, input_data, table, timeout, helper, binary_hash):
    run_dir = output / "runs" / label / scenario
    run_dir.mkdir(parents=True, exist_ok=False)
    command = [str(binary)] + COMMON_ARGUMENTS + [
        "--sumo-net", input_data["net"], "--sumo-route", input_data["route"],
        "--sumo-tripinfo", input_data["truth"], "--eval-output", str(run_dir / "sumo_eval.csv"),
        "--prediction-tripinfo-output", str(run_dir / "prediction_tripinfo.xml"),
    ]
    if table is not None:
        command += ["--travel-time-mode", "table", "--travel-time-table", table["path"], "--speed-net-speed-factor", "off"]
    else:
        command += ["--travel-time-mode", "speed-net", "--speed-net-speed-factor", "on"]
    record = {
        "scenario": scenario, "label": label, "command": command, "cwd": str(run_dir),
        "binary_sha256": binary_hash, "timeout_seconds": timeout, "status": "running",
        "table_sha256": table["sha256"] if table else None,
    }
    write_json(run_dir / "run.json", record)
    environment = dict(os.environ)
    for variable in ("USE_SUMO_NET", "SUMO_NET_PATH"):
        environment.pop(variable, None)
    started = time.perf_counter()
    try:
        with (run_dir / "run.log").open("wb") as log:
            process = subprocess.run(command, cwd=run_dir, stdout=log, stderr=subprocess.STDOUT,
                                     env=environment, timeout=timeout, check=False,
                                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        record["returncode"] = process.returncode
        if process.returncode:
            raise RuntimeError(f"Simulator exited {process.returncode}: {run_dir / 'run.log'}")
        record["log_audit"] = parse_run_log((run_dir / "run.log").read_text(encoding="utf-8", errors="replace"), label, input_data["expected_vehicles"])
        result, _, _ = helper.evaluate_run(run_dir, input_data)
        record["independent_metrics"] = result
        record["output_hashes"] = {name: helper.sha256(run_dir / name) for name in ("prediction_tripinfo.xml", "sumo_eval.csv", "run.log")}
        record["status"] = "success"
        print(f"[OK] {label}/{scenario}: N={result['vehicles']} MAE={result['mae_s']:.6f} bias={result['bias_s']:.6f}", flush=True)
    except BaseException as exc:
        record["status"] = "timeout" if isinstance(exc, subprocess.TimeoutExpired) else "failed"
        record.setdefault("returncode", None)
        record["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        record["elapsed_seconds"] = time.perf_counter() - started
        write_json(run_dir / "run.json", record)
    return record


def baseline_regression(output, inputs, helper):
    regressions = {}
    for scenario, input_data in inputs.items():
        old_dir = output / "runs" / OLD_BASELINE / scenario
        new_dir = output / "runs" / BASELINE / scenario
        old_metrics, old_errors, old_prediction = helper.evaluate_run(old_dir, input_data)
        _, new_errors, new_prediction = helper.evaluate_run(new_dir, input_data)
        mismatches = [vehicle for vehicle in old_prediction if old_prediction[vehicle] != new_prediction[vehicle]]
        equal_csv = (old_dir / "sumo_eval.csv").read_bytes() == (new_dir / "sumo_eval.csv").read_bytes()
        regressions[scenario] = {
            "vehicles": len(old_prediction), "prediction_all_fields_equal": not mismatches,
            "different_prediction_vehicles": len(mismatches), "csv_byte_identical": equal_csv,
            "errors_identical": old_errors == new_errors, "old_binary_metrics": old_metrics,
        }
        if mismatches or not equal_csv or old_errors != new_errors:
            raise ValueError(f"New binary changed frozen SF-on Speed-net predictions: {scenario}")
    return regressions


def collect_summary(output, inputs, tables, helper, regression):
    summary = {
        "metric_definition": "error = predicted trip duration minus SUMO trip duration; initial departure delay excluded",
        "precision_note": "C++ XML/CSV export uses six significant digits; interpret final comparisons to 0.001 seconds",
        "pooled_definition": "All 45,000 vehicle errors pooled, not an unweighted average of three MAEs",
        "common_arguments": COMMON_ARGUMENTS,
        "baseline_parameters": {"speed_factor": "on", "headway_s": 0.12, "delay_max_s": 3.0},
        "modes": {}, "regression": regression,
        "table_input_contract": "Five keys, exact road/flow/static values and nearest SF grid; no division by SF after table prediction",
    }
    for label in (BASELINE,) + tuple(tables):
        datasets = {}
        pooled_errors = []
        all_hits = 0
        outside = {"road_flow": 0, "lane_flow": 0}
        maxima = {"road_flow": 0, "lane_flow": 0}
        for scenario, input_data in inputs.items():
            run_dir = output / "runs" / label / scenario
            record = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
            if record.get("status") != "success" or record.get("returncode") != 0:
                raise ValueError(f"Incomplete task: {run_dir}")
            for name, expected_hash in record["output_hashes"].items():
                if helper.sha256(run_dir / name) != expected_hash:
                    raise ValueError(f"Output changed before summary: {run_dir / name}")
            result, errors, _ = helper.evaluate_run(run_dir, input_data)
            if result != record["independent_metrics"]:
                raise ValueError(f"Saved metrics differ from independently reconstructed values: {run_dir}")
            result.update(record["log_audit"])
            all_hits += result.get("table_hits", 0)
            if label in tables:
                for feature in outside:
                    outside[feature] += result[f"{feature}_outside_training"]
                    maxima[feature] = max(maxima[feature], result[f"{feature}_max_seen"])
            pooled_errors.extend(errors.values())
            datasets[scenario] = result
        pooled = helper.metrics(pooled_errors)
        if pooled["vehicles"] != 45000:
            raise ValueError(f"Expected 45,000 pooled vehicles, found {pooled['vehicles']}")
        pooled.update(finished_vehicles=45000, total_vehicles=45000, invalid_vehicles=0)
        if label in tables:
            pooled.update(table_hits=all_hits, table_misses=0, sf_queries=all_hits)
            for feature in outside:
                pooled[f"{feature}_outside_training"] = outside[feature]
                pooled[f"{feature}_outside_training_fraction"] = outside[feature] / all_hits
                pooled[f"{feature}_max_seen"] = maxima[feature]
        summary["modes"][label] = {"datasets": datasets, "pooled": pooled}
        if label in tables:
            summary["modes"][label].update(table=tables[label]["path"], table_sha256=tables[label]["sha256"],
                                           sf_quantization_audit=tables[label]["sf_quantization_audit"],
                                           export_training_ranges=tables[label]["plan"].get("training_ranges"))
    baseline = summary["modes"][BASELINE]
    for label in tables:
        section = summary["modes"][label]
        section["comparison_to_baseline"] = {}
        for scenario in SCENARIOS + ("pooled",):
            own = section["pooled"] if scenario == "pooled" else section["datasets"][scenario]
            other = baseline["pooled"] if scenario == "pooled" else baseline["datasets"][scenario]
            section["comparison_to_baseline"][scenario] = {
                "mae_change_s": own["mae_s"] - other["mae_s"],
                "bias_change_s": own["bias_s"] - other["bias_s"],
                "mae_reduction_percent": 100 * (1 - own["mae_s"] / other["mae_s"]),
            }
    return summary


def write_report(output, summary):
    lines = [
        "# SF + speed-limit catch table: three-scenario closed-loop experiment", "",
        "All time/error values are seconds. Bias = C++ trip duration minus SUMO trip duration; positive means slower.",
        "The pooled result gives every one of the 45,000 vehicles equal weight. Departure delay is excluded.", "",
        "Six new five-feature tables are compared to SF-on Speed-net at fixed h=0.12 s and cap=3 s.",
        "Discharge interval=2 s, crossing=3 s, arrival-time queue order, both road corrections, queue trim off.",
        "The table tasks set the formula-only SF flag off; the SF table reader still uses each vehicle's explicit SF as a key.",
        "No parameter tuning, SUMO reruns, label recollection or junction/queue redesign was performed.", "",
        "| Mode | free MAE | free bias | tight MAE | tight bias | congested MAE | congested bias | pooled MAE | pooled bias |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for label, section in summary["modes"].items():
        values = [section["datasets"][name] for name in SCENARIOS] + [section["pooled"]]
        cells = [item for value in values for item in (f"{value['mae_s']:.6f}", f"{value['bias_s']:.6f}")]
        lines.append("| " + " | ".join([label] + cells) + " |")
    lines += ["", "## Full metric and lookup audit", "",
              "| Mode | Dataset | Completed / total | MAE | bias | RMSE | P95 absolute error | table hits | table misses |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for label, section in summary["modes"].items():
        for scenario, value in list(section["datasets"].items()) + [("pooled", section["pooled"])]:
            lines.append(f"| {label} | {scenario} | {value['finished_vehicles']:,} / {value['total_vehicles']:,} | {value['mae_s']:.6f} | {value['bias_s']:.6f} | {value['rmse_s']:.6f} | {value['p95_abs_error_s']:.6f} | {value.get('table_hits', '-')} | {value.get('table_misses', '-')} |")
    lines += ["", "## Closed-loop flow extrapolation", "",
              "Counts are road-time queries, not unique vehicles. Training ranges are threshold-specific and saved in summary.json.", "",
              "| Table | Dataset | Queries | road_flow outside count | road_flow outside % | road_flow maximum | lane_flow outside count | lane_flow outside % | lane_flow maximum |",
              "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for label, section in summary["modes"].items():
        if label == BASELINE:
            continue
        for scenario, value in list(section["datasets"].items()) + [("pooled", section["pooled"])]:
            lines.append(f"| {label} | {scenario} | {value['sf_queries']} | {value['road_flow_outside_training']} | {100 * value['road_flow_outside_training_fraction']:.4f} | {value['road_flow_max_seen']} | {value['lane_flow_outside_training']} | {100 * value['lane_flow_outside_training_fraction']:.4f} | {value['lane_flow_max_seen']} |")
    lines += ["", "## Verification and limitations", "",
              "All three new-binary baseline runs match the old binary across every predicted tripinfo attribute and every byte of the evaluation CSV.",
              "Every run has full vehicle coverage, zero invalid vehicles, and independent XML-derived metrics cross-checked against the per-vehicle evaluation CSV.",
              "All table runs report CAMS_SF_TABLE_V1, positive hits and zero misses. Missing keys are fatal, not formula fallbacks.",
              "Input, route, truth, table, export manifest, helper and binary hashes are protected across the experiment. Commands, logs, return codes and output hashes are retained per task.",
              "SF discretization is an approximation; input quantization audits are in summary.json. Real-valued model predictions were not available locally, so this experiment does not isolate discretization error.",
              "Table grid coverage can exceed training ranges. Coverage and zero misses do not prove accurate extrapolation or training-data validity.",
              "These are closed-loop trip-duration metrics, not road-label training MAE. Changed vehicle ordering, queues and signal windows are included.",
              "The three established scenarios have already been used for parameter studies. This is a benchmark comparison, not a newly held-out certification.",
              "Existing six-significant-digit exporters can differ slightly from in-memory log metrics; interpret comparisons to three decimal places.", ""]
    (output / "report.md").write_text("\n".join(lines), encoding="utf-8")


def run_wave(tasks, workers):
    records = []
    failures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(execute_run, *task) for task in tasks]
        for future in concurrent.futures.as_completed(futures):
            try:
                records.append(future.result())
            except Exception as exc:
                failures.append(f"{type(exc).__name__}: {exc}")
    if failures:
        raise RuntimeError("One or more simulator tasks failed:\n" + "\n".join(failures))
    return records


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--baseline-binary", type=Path, required=True)
    parser.add_argument("--table-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=1200)
    parser.add_argument("--preflight-only", action="store_true", help="Validate inputs and hashes without creating output or starting C++")
    args = parser.parse_args(argv)
    if args.workers < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("workers and timeout must be positive")
    for name in ("repo", "binary", "baseline_binary", "table_root", "output"):
        setattr(args, name, getattr(args, name).resolve())
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite existing experiment output: {args.output}")
    helper, helper_path = import_helper(args.repo)
    inputs, input_hashes = helper.load_inputs(args.repo)
    if tuple(inputs) != SCENARIOS or sum(value["expected_vehicles"] for value in inputs.values()) != 45000:
        raise ValueError("Unexpected benchmark scenario definition")
    tables, table_hashes = load_tables(args.table_root, inputs, helper)
    binary_hashes = {str(path): helper.sha256(path) for path in (args.binary, args.baseline_binary)}
    protected_hashes = dict(input_hashes, **table_hashes, **binary_hashes)
    protected_hashes[str(helper_path)] = helper.sha256(helper_path)
    protected_hashes[str(Path(__file__).resolve())] = helper.sha256(__file__)
    manifest = {
        "repo": str(args.repo), "output": str(args.output), "status": "preflight",
        "protected_hashes": protected_hashes, "expected_scenarios": helper.SCENARIOS,
        "tables": tables, "common_arguments": COMMON_ARGUMENTS,
        "workers": args.workers, "timeout_seconds": args.timeout, "planned_run_count": 24,
    }
    print(f"[PREFLIGHT] 3 scenarios / 45,000 vehicles, {len(tables)} validated SF-table exports; 24 fixed-parameter C++ runs", flush=True)
    if args.preflight_only:
        return 0
    args.output.mkdir(parents=True, exist_ok=False)
    manifest["status"] = "running"
    write_json(args.output / "experiment.json", manifest)
    started = time.perf_counter()
    records = []
    try:
        baseline_tasks = []
        for label, binary in ((OLD_BASELINE, args.baseline_binary), (BASELINE, args.binary)):
            for scenario, input_data in inputs.items():
                baseline_tasks.append((args.output, binary, scenario, label, input_data, None, args.timeout, helper, binary_hashes[str(binary)]))
        records.extend(run_wave(baseline_tasks, args.workers))
        regression = baseline_regression(args.output, inputs, helper)
        write_json(args.output / "baseline_regression.json", regression)
        table_tasks = [(args.output, args.binary, scenario, label, input_data, table, args.timeout, helper, binary_hashes[str(args.binary)])
                       for label, table in tables.items() for scenario, input_data in inputs.items()]
        records.extend(run_wave(table_tasks, args.workers))
        if len(records) != 24:
            raise ValueError(f"Expected 24 completed tasks, found {len(records)}")
        summary = collect_summary(args.output, inputs, tables, helper, regression)
        summary["protected_hashes"] = protected_hashes
        for path, expected_hash in protected_hashes.items():
            if helper.sha256(path) != expected_hash:
                raise ValueError(f"Protected input or binary changed during experiment: {path}")
        write_json(args.output / "summary.json", summary)
        write_report(args.output, summary)
        manifest.update(status="success", completed_run_count=len(records), source_hashes_unchanged=True,
                        sum_simulator_elapsed_seconds=math.fsum(record["elapsed_seconds"] for record in records))
        print(f"[SUCCESS] {args.output / 'report.md'}", flush=True)
    except BaseException as exc:
        manifest.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        manifest["elapsed_seconds"] = time.perf_counter() - started
        write_json(args.output / "experiment.json", manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
