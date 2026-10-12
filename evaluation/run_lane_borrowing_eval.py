#!/usr/bin/env python3
"""Freeze executables and run the lane-borrowing 30-run regression matrix."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from roadnet_paths import ROOT
import re
import shutil
import statistics
import subprocess
import time
import xml.etree.ElementTree as ET


SCENARIOS = ("free", "tight", "congested")
PREDICTORS = ("speed_net", "table")
TELEPORT_ID = "congested_veh_013800"
SCIENCE_FILES = (
    "prediction.xml", "eval.csv", "eval_vehicle_road_timeline.csv",
    "eval_vehicle_movement_timeline.csv", "eval_vehicle_trajectory_summary.csv",
    "eval_downstream_release_competition.csv", "eval_movement_wait_statistics.csv",
)


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def metrics(rows):
    errors = [float(row["durationErrorSigned"]) for row in rows]
    recomputed = [float(row["predDuration"]) - float(row["truthDuration"]) for row in rows]
    absolute = sorted(abs(value) for value in errors)
    return {
        "count": len(rows), "mae": statistics.mean(absolute),
        "bias": statistics.mean(errors), "rmse": math.sqrt(statistics.mean(x*x for x in errors)),
        "p95_abs_error_nearest_rank": absolute[math.ceil(0.95 * len(rows)) - 1],
        "max_abs_error": max(absolute),
        "mean_pred_duration": statistics.mean(float(row["predDuration"]) for row in rows),
        "mean_truth_duration": statistics.mean(float(row["truthDuration"]) for row in rows),
        "mae_from_exported_duration_columns": statistics.mean(abs(x) for x in recomputed),
        "bias_from_exported_duration_columns": statistics.mean(recomputed),
        "max_csv_rounding_delta": max(abs(a-b) for a, b in zip(errors, recomputed)),
    }


def audit_run(directory, route, truth, predictor, variant, dataset):
    expected = {node.get("id") for node in ET.parse(route).getroot().findall("vehicle")}
    truths = {node.get("id"): node.attrib for node in ET.parse(truth).getroot().findall("tripinfo")}
    rows = list(csv.DictReader((directory / "eval.csv").open(encoding="utf-8-sig", newline="")))
    predictions = ET.parse(directory / "prediction.xml").getroot().findall("tripinfo")
    assert len(rows) == len(expected) == len(predictions)
    assert {row["vehicleID"] for row in rows} == expected == truths.keys()
    assert {node.get("id") for node in predictions} == expected
    assert all(row["validVehicle"] == "true" for row in rows)
    assert all(float(row["truthDuration"]) == float(truths[row["vehicleID"]]["duration"]) for row in rows)
    assert all(math.isfinite(float(row["predDuration"])) and float(row["predDuration"]) >= 0 for row in rows)
    log = (directory / "run.log").read_text(encoding="utf-8", errors="replace")
    table_hits = re.search(r"\[TravelTime\] table hits: (\d+)", log)
    table_misses = re.search(r"\[TravelTime\] table misses: (\d+)", log)
    borrow = re.search(r"\[LaneBorrowing\] (.*)", log)
    result = {"all_ids_complete_valid": True, "metrics_all": metrics(rows),
              "table_hits": int(table_hits.group(1)) if table_hits else None,
              "table_misses": int(table_misses.group(1)) if table_misses else None,
              "lane_borrowing": dict(re.findall(r"(\w+)=(\S+)", borrow.group(1))) if borrow else None}
    if predictor == "table":
        assert result["table_hits"] > 0 and result["table_misses"] == 0
    if variant == "new_on":
        counts = result["lane_borrowing"]
        assert counts and counts["invariants"] == "ok"
        assert int(counts["active"]) == 0
        assert int(counts["admissions"]) == int(counts["releases"])
        assert int(counts["storage_reservations"]) == int(counts["storage_releases"])
    if dataset == "full_lc" and TELEPORT_ID in expected:
        filtered = [row for row in rows if row["vehicleID"] != TELEPORT_ID]
        result["metrics_excluding_teleported_id"] = metrics(filtered)
        result["excluded_id"] = TELEPORT_ID
        result["exclusion_limitation"] = "Excluding the teleported vehicle does not remove its indirect effects."
    result["scientific_hashes"] = {name: sha(directory / name) for name in SCIENCE_FILES}
    if predictor == "table":
        result["scientific_hashes"]["cap_context_trace.csv"] = sha(directory / "cap_context_trace.csv")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("eval/lane_borrowing_20260929/cpp_eval"))
    parser.add_argument("--new-exe", type=Path, default=Path("eval/lane_borrowing_20260929/build/Simulation_Prediction.exe"))
    parser.add_argument("--old-exe", type=Path, default=Path("eval/lane_borrowing_20260929/before/build/Simulation_Prediction.exe"))
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--phase", choices=("all", "old", "remaining"), default="all")
    args = parser.parse_args()
    repo = ROOT
    output = args.output.resolve()
    if output.exists() and args.phase != "remaining":
        raise RuntimeError(f"Refusing to overwrite matrix output: {output}")
    (output / "binaries").mkdir(parents=True, exist_ok=args.phase == "remaining")
    binaries = {}
    frozen = {}
    if args.phase == "remaining":
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        frozen = manifest["binaries"]
        assert "new" not in frozen
        binaries = {name: Path(row["frozen"]) for name, row in frozen.items()}
        assert all(sha(binaries[name]) == row["sha256"] for name, row in frozen.items())
    sources = (("new", args.new_exe.resolve()),) if args.phase == "remaining" else (
        (("old", args.old_exe.resolve()),) if args.phase == "old" else
        (("old", args.old_exe.resolve()), ("new", args.new_exe.resolve())))
    for name, source in sources:
        target = output / "binaries" / f"{name}.exe"
        before = sha(source)
        shutil.copy2(source, target)
        assert sha(source) == before == sha(target)
        binaries[name] = target
        frozen[name] = {"source": str(source), "frozen": str(target), "sha256": before}
    table = repo / "catch/abc_rg_signal1_roadpiecewise_20260922_074010/gt_0p1/table.rle"
    if args.phase != "remaining":
        manifest = {"schema": "lane-borrowing-cpp-30-run-v1", "binaries": frozen,
                    "table": str(table), "table_sha256": sha(table), "runner_sha256": sha(__file__),
                    "timeout_seconds_per_run": args.timeout, "runs": [], "regressions": []}
    else:
        assert sha(table) == manifest["table_sha256"]
        assert sha(__file__) == manifest["runner_sha256"]
    completed_runs = {row["run_id"]: row for row in manifest["runs"]}
    write_json(output / "manifest.json", manifest)
    for dataset in ("historical", "full_lc"):
        dataset_root = repo / ("data/variants/speed_factor" if dataset == "historical" else
                               "eval/lane_borrowing_20260929/sumo_full_lc")
        net = dataset_root / "inputs/dfbf38f23154_test_new.net.xml"
        for scenario in SCENARIOS:
            route = dataset_root / "routes" / f"{scenario}.rou.xml"
            truth = dataset_root / (f"outputs/tripinfo/tripinfo_{scenario}.xml" if dataset == "historical" else
                                     f"runs/{scenario}/tripinfo.xml")
            for predictor in PREDICTORS:
                group = {}
                for variant in (("old", "new_off") if dataset == "historical" else ("old", "new_off", "new_on")):
                    if args.phase == "old" and variant != "old":
                        continue
                    run_id = f"{dataset}/{scenario}/{predictor}/{variant}"
                    directory = output / "runs" / run_id
                    if run_id in completed_runs:
                        saved = completed_runs[run_id]
                        assert saved["status"] == "success"
                        assert all(sha(path) == digest for path, digest in saved["input_sha256"].items())
                        assert all(sha(directory / name) == digest for name, digest in saved["scientific_hashes"].items())
                        group[variant] = saved
                        continue
                    directory.mkdir(parents=True)
                    command = [str(binaries["old" if variant == "old" else "new"]),
                        "--use-sumo", "--sumo-net", str(net), "--sumo-route", str(route),
                        "--sumo-tripinfo", str(truth), "--eval-output", str(directory / "eval.csv"),
                        "--prediction-tripinfo-output", str(directory / "prediction.xml"),
                        "--eval-trajectory-details", "--eval-split-extreme", "--eval-extreme-timeline",
                        "--intersection-crossing-time", "3", "--lane-discharge-interval", "2",
                        "--movement-queue-order", "arrival-time", "--dispatch-queue-mode", "incremental",
                        "--speed-net-continuous-speed", "off", "--speed-net-depart-position", "on",
                        "--speed-net-components", "both", "--speed-net-headway", "0.6",
                        "--speed-net-delay-max", "3"]
                    if predictor == "speed_net":
                        command += ["--travel-time-mode", "speed-net", "--speed-net-speed-factor", "on",
                                    "--speed-net-queue-trim", "off"]
                    else:
                        command += ["--travel-time-mode", "table", "--travel-time-table", str(table),
                                    "--speed-net-speed-factor", "off", "--expected-signal-remaining-step", "1",
                                    "--cap-context-trace", str(directory / "cap_context_trace.csv")]
                    if variant != "old":
                        command += ["--lane-borrowing", "on" if variant == "new_on" else "off"]
                    inputs = {str(path): sha(path) for path in (net, route, truth)}
                    invocation = {"run_id": run_id, "command": command, "input_sha256": inputs,
                                  "binary_sha256": frozen["old" if variant == "old" else "new"]["sha256"],
                                  "table_sha256": manifest["table_sha256"] if predictor == "table" else None}
                    write_json(directory / "invocation.json", invocation)
                    print(f"START {run_id}", flush=True)
                    start = time.monotonic()
                    try:
                        with (directory / "run.log").open("w", encoding="utf-8") as log:
                            completed = subprocess.run(command, cwd=repo, stdout=log, stderr=subprocess.STDOUT,
                                                       timeout=args.timeout, check=False)
                        invocation["exit_code"] = completed.returncode
                        invocation["elapsed_seconds"] = time.monotonic() - start
                        assert completed.returncode == 0, f"exit={completed.returncode}"
                        invocation.update(audit_run(directory, route, truth, predictor, variant, dataset))
                        assert all(sha(path) == digest for path, digest in inputs.items())
                        invocation["status"] = "success"
                    except Exception as exc:
                        invocation.update({"status": "failed", "exception": repr(exc),
                                           "elapsed_seconds": time.monotonic() - start})
                        manifest["runs"].append(invocation)
                        write_json(directory / "result.json", invocation)
                        write_json(output / "manifest.json", manifest)
                        print(f"FAILED {run_id}: {exc}", flush=True)
                        raise
                    write_json(directory / "result.json", invocation)
                    manifest["runs"].append(invocation)
                    group[variant] = invocation
                    print(f"PASS {run_id}: MAE={invocation['metrics_all']['mae']:.6f}, "
                          f"bias={invocation['metrics_all']['bias']:.6f}, "
                          f"wall={invocation['elapsed_seconds']:.2f}s, "
                          f"borrow={invocation['lane_borrowing']}", flush=True)
                    write_json(output / "manifest.json", manifest)
                if args.phase == "old":
                    continue
                regression = {"dataset": dataset, "scenario": scenario, "predictor": predictor,
                              "files": {name: group["old"]["scientific_hashes"][name] ==
                                        group["new_off"]["scientific_hashes"][name]
                                        for name in group["old"]["scientific_hashes"]}}
                regression["all_raw_bytes_equal"] = all(regression["files"].values())
                manifest["regressions"].append(regression)
                write_json(output / "manifest.json", manifest)
                assert regression["all_raw_bytes_equal"], regression
    if args.phase == "old":
        manifest["status"] = "old_baselines_complete"
        write_json(output / "manifest.json", manifest)
        print("COMPLETE: 12 frozen old-binary baselines; awaiting new binary", flush=True)
        return
    manifest["status"] = "success"
    manifest["run_count"] = len(manifest["runs"])
    manifest["all_off_regressions_exact"] = all(row["all_raw_bytes_equal"] for row in manifest["regressions"])
    manifest["source_input_and_table_final_hashes_unchanged"] = sha(table) == manifest["table_sha256"]
    assert manifest["run_count"] == 30 and manifest["source_input_and_table_final_hashes_unchanged"]
    write_json(output / "manifest.json", manifest)
    with (output / "metrics.csv").open("w", encoding="utf-8", newline="") as stream:
        columns = ["run_id", "count", "mae", "bias", "rmse", "elapsed_seconds", "table_hits", "table_misses",
                   "admissions", "clamped_predictions", "exclude_teleport_mae", "exclude_teleport_bias"]
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for run in manifest["runs"]:
            row = {key: run[key] for key in ("run_id", "elapsed_seconds", "table_hits", "table_misses")}
            row.update({key: run["metrics_all"][key] for key in ("count", "mae", "bias", "rmse")})
            row.update({key: (run["lane_borrowing"] or {}).get(key) for key in ("admissions", "clamped_predictions")})
            excluded = run.get("metrics_excluding_teleported_id", {})
            row.update({"exclude_teleport_mae": excluded.get("mae"), "exclude_teleport_bias": excluded.get("bias")})
            writer.writerow(row)
    print("COMPLETE: 30 runs, all off regressions raw-byte identical", flush=True)


if __name__ == "__main__":
    main()
