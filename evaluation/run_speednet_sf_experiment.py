#!/usr/bin/env python3
"""Fixed-parameter Speed-net SF ablation, with independent tripinfo evaluation.

This executes the C++ simulator, not SUMO or TraCI label collection. Source routes,
truth, and binaries are read-only; all simulator output goes into a new directory.
"""

import argparse
import concurrent.futures
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET


SCENARIOS = {"free": 7778, "tight": 17778, "congested": 19444}
PARAMETERS = {"historical_h0p2_d3": (0.2, 3.0), "defaults_h0p8_d6": (0.8, 6.0)}
COMMON_ARGUMENTS = [
    "--use-sumo", "--travel-time-mode", "speed-net",
    "--speed-net-components", "both", "--speed-net-queue-trim", "off",
    "--lane-discharge-interval", "2", "--intersection-crossing-time", "3",
    "--movement-queue-order", "arrival-time",
]
SF_BINS = [
    ("SF < 0.9", 0.0, 0.9), ("0.9 <= SF < 1.0", 0.9, 1.0),
    ("1.0 <= SF < 1.1", 1.0, 1.1), ("SF >= 1.1", 1.1, math.inf),
]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, payload):
    Path(path).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_records(path, tag):
    records = {}
    for _, element in ET.iterparse(path, events=("end",)):
        if element.tag == tag:
            identifier = element.attrib["id"]
            if identifier in records:
                raise ValueError(f"Duplicate {tag} ID {identifier} in {path}")
            records[identifier] = dict(element.attrib)
        element.clear()
    return records


def quantile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return None
    position = probability * (len(ordered) - 1)
    left = math.floor(position)
    right = math.ceil(position)
    return ordered[left] * (right - position) + ordered[right] * (position - left) if left != right else ordered[left]


def metrics(errors):
    values = list(errors)
    if not values:
        return {"vehicles": 0, "mae_s": None, "bias_s": None}
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


def half_export_ulp(value):
    """Maximum half-unit rounding for the current six-significant-digit export."""
    return 0.0 if value == 0 else 0.5 * 10.0 ** (math.floor(math.log10(abs(value))) - 5)


def paired_metrics(off_errors, on_errors):
    if off_errors.keys() != on_errors.keys():
        raise ValueError("Paired errors do not have identical vehicle coverage")
    abs_deltas = [abs(on_errors[vehicle]) - abs(off_errors[vehicle]) for vehicle in off_errors]
    duration_deltas = [on_errors[vehicle] - off_errors[vehicle] for vehicle in off_errors]
    return {
        "vehicles": len(abs_deltas), "mae_change_s": math.fsum(abs_deltas) / len(abs_deltas),
        "mean_duration_change_s": math.fsum(duration_deltas) / len(duration_deltas),
        "improved_vehicles": sum(value < -1e-6 for value in abs_deltas),
        "worsened_vehicles": sum(value > 1e-6 for value in abs_deltas),
        "unchanged_abs_error_vehicles": sum(abs(value) <= 1e-6 for value in abs_deltas),
        "faster_vehicles": sum(value < -1e-6 for value in duration_deltas),
        "slower_vehicles": sum(value > 1e-6 for value in duration_deltas),
        "unchanged_duration_vehicles": sum(abs(value) <= 1e-6 for value in duration_deltas),
        "total_abs_error_reduced_on_improved_vehicles_s": -math.fsum(value for value in abs_deltas if value < -1e-6),
        "total_abs_error_added_on_worsened_vehicles_s": math.fsum(value for value in abs_deltas if value > 1e-6),
    }


def load_inputs(repo):
    data = repo / "data/variants/speed_factor"
    inputs = {}
    hashes = {}
    for scenario, expected_count in SCENARIOS.items():
        config = data / "sumocfg" / f"{scenario}.sumocfg"
        tree = ET.parse(config)
        net = (config.parent / tree.find("input/net-file").get("value")).resolve()
        route = (config.parent / tree.find("input/route-files").get("value")).resolve()
        truth = (config.parent / tree.find("output/tripinfo-output").get("value")).resolve()
        route_records = read_records(route, "vehicle")
        truth_records = read_records(truth, "tripinfo")
        if len(route_records) != expected_count or route_records.keys() != truth_records.keys():
            raise ValueError(f"{scenario}: route/truth coverage mismatch")
        factors = {vehicle: float(row["speedFactor"]) for vehicle, row in route_records.items()}
        if any(not math.isfinite(value) or value <= 0 for value in factors.values()):
            raise ValueError(f"{scenario}: invalid/missing explicit speedFactor")
        if any(not math.isfinite(float(row["duration"])) or float(row["duration"]) < 0 for row in truth_records.values()):
            raise ValueError(f"{scenario}: invalid SUMO duration")
        for path in (config, net, route, truth):
            hashes[str(path)] = sha256(path)
        inputs[scenario] = {
            "config": str(config), "net": str(net), "route": str(route), "truth": str(truth),
            "expected_vehicles": expected_count, "speed_factors": factors, "truth_records": truth_records,
        }
    return inputs, hashes


def evaluate_run(run_dir, input_data):
    prediction = read_records(run_dir / "prediction_tripinfo.xml", "tripinfo")
    truth = input_data["truth_records"]
    if prediction.keys() != truth.keys():
        raise ValueError(f"Prediction coverage mismatch: {run_dir}")
    errors = {}
    for vehicle, row in prediction.items():
        duration = float(row["duration"])
        if not math.isfinite(duration) or duration < 0:
            raise ValueError(f"Invalid predicted duration for {vehicle}: {duration}")
        errors[vehicle] = duration - float(truth[vehicle]["duration"])
    with (run_dir / "sumo_eval.csv").open(encoding="utf-8-sig", newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    csv_by_id = {row["vehicleID"]: row for row in csv_rows}
    if len(csv_rows) != len(csv_by_id) or csv_by_id.keys() != truth.keys():
        raise ValueError(f"Evaluation CSV coverage mismatch: {run_dir}")
    for vehicle, row in csv_by_id.items():
        if row["validVehicle"].lower() not in ("1", "true"):
            raise ValueError(f"Invalid vehicle in evaluation CSV: {vehicle}")
        if float(row["predDuration"]) != float(prediction[vehicle]["duration"]):
            raise ValueError(f"XML / CSV predicted duration mismatch: {vehicle}")
        if float(row["truthDuration"]) != float(truth[vehicle]["duration"]):
            raise ValueError(f"XML / CSV truth duration mismatch: {vehicle}")
        # Existing C++ exporters use six significant digits; duration-derived
        # error and separately rounded error columns can differ slightly.
        exported_error = float(row["durationError"])
        rounding_tolerance = half_export_ulp(float(row["predDuration"])) + half_export_ulp(exported_error) + 1e-8
        if abs(exported_error - errors[vehicle]) > rounding_tolerance:
            raise ValueError(f"Independent error / CSV error mismatch: {vehicle}")
    return metrics(errors.values()), errors, prediction


def execute_run(output, binary, scenario, label, parameter_name, input_data, timeout):
    run_dir = output / parameter_name / label / scenario
    run_dir.mkdir(parents=True, exist_ok=False)
    headway, delay_max = PARAMETERS[parameter_name]
    command = [str(binary)] + COMMON_ARGUMENTS + [
        "--sumo-net", input_data["net"], "--sumo-route", input_data["route"],
        "--sumo-tripinfo", input_data["truth"], "--eval-output", str(run_dir / "sumo_eval.csv"),
        "--prediction-tripinfo-output", str(run_dir / "prediction_tripinfo.xml"),
        "--speed-net-headway", str(headway), "--speed-net-delay-max", str(delay_max),
    ]
    if label != "old_binary":
        command += ["--speed-net-speed-factor", "on" if label == "sf_on" else "off"]
    record = {"scenario": scenario, "label": label, "parameters": parameter_name,
              "command": command, "cwd": str(run_dir), "binary_sha256": sha256(binary)}
    write_json(run_dir / "run.json", record)
    started = time.perf_counter()
    environment = dict(os.environ)
    environment.pop("USE_SUMO_NET", None)
    environment.pop("SUMO_NET_PATH", None)
    with (run_dir / "run.log").open("wb") as log:
        process = subprocess.run(command, cwd=run_dir, stdout=log, stderr=subprocess.STDOUT,
                                 env=environment, timeout=timeout, check=False,
                                 creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    record.update(returncode=process.returncode, elapsed_seconds=time.perf_counter() - started)
    write_json(run_dir / "run.json", record)
    if process.returncode:
        raise RuntimeError(f"Simulator failed: {run_dir / 'run.log'} (exit {process.returncode})")
    result, _, _ = evaluate_run(run_dir, input_data)
    record["independent_metrics"] = result
    record["output_hashes"] = {path.name: sha256(path) for path in (run_dir / "prediction_tripinfo.xml", run_dir / "sumo_eval.csv")}
    write_json(run_dir / "run.json", record)
    print(f"[OK] {parameter_name}/{label}/{scenario}: N={result['vehicles']} MAE={result['mae_s']:.6f} bias={result['bias_s']:.6f}", flush=True)
    return record


def collect_summary(output, inputs):
    summary = {"metric_definition": "error = predicted trip duration - SUMO trip duration; pooled is vehicle-weighted, excludes departure delay",
               "precision_note": "Existing C++ XML/CSV exporters use six significant digits; independently reconstructed values can differ slightly from in-memory/logged metrics. Interpret main results to three decimal places.",
               "common_arguments": COMMON_ARGUMENTS, "parameters": {}, "regression": {}}
    for parameter_name in PARAMETERS:
        section = {"headway_s": PARAMETERS[parameter_name][0], "delay_max_s": PARAMETERS[parameter_name][1],
                   "datasets": {}, "sf_bins_pooled": {}}
        pooled = {"sf_off": {}, "sf_on": {}}
        pooled_sf = {}
        for scenario, input_data in inputs.items():
            scenario_result = {}
            predictions = {}
            errors = {}
            for label in ("sf_off", "sf_on"):
                result, errors[label], predictions[label] = evaluate_run(output / parameter_name / label / scenario, input_data)
                scenario_result[label] = result
                pooled[label].update({f"{scenario}/{vehicle}": value for vehicle, value in errors[label].items()})
            pooled_sf.update({f"{scenario}/{vehicle}": value for vehicle, value in input_data["speed_factors"].items()})
            scenario_result["paired"] = paired_metrics(errors["sf_off"], errors["sf_on"])
            scenario_result["sf_bins"] = {}
            for name, low, high in SF_BINS:
                ids = [vehicle for vehicle, sf in input_data["speed_factors"].items() if low <= sf < high]
                scenario_result["sf_bins"][name] = {label: metrics(errors[label][vehicle] for vehicle in ids) for label in errors}
            if parameter_name == "historical_h0p2_d3":
                old_dir = output / parameter_name / "old_binary" / scenario
                old_result, old_errors, old_predictions = evaluate_run(old_dir, input_data)
                mismatch = [vehicle for vehicle in old_predictions if old_predictions[vehicle] != predictions["sf_off"][vehicle]]
                old_csv = (old_dir / "sumo_eval.csv").read_bytes()
                off_csv = (output / parameter_name / "sf_off" / scenario / "sumo_eval.csv").read_bytes()
                regression = {"vehicles": len(old_predictions), "prediction_all_fields_equal": not mismatch,
                              "different_prediction_vehicles": len(mismatch), "csv_byte_identical": old_csv == off_csv,
                              "old_binary_metrics": old_result}
                summary["regression"][scenario] = regression
                if mismatch or old_errors != errors["sf_off"] or old_csv != off_csv:
                    raise ValueError(f"Old binary / new SF off regression: {scenario}")
            paired_tree = ET.Element("pairedTripErrors", dataset=scenario, parameters=parameter_name,
                                     errorDefinition="prediction duration minus SUMO duration")
            for vehicle, sf in input_data["speed_factors"].items():
                ET.SubElement(paired_tree, "vehicle", id=vehicle, speedFactor=repr(sf),
                              truthDuration=input_data["truth_records"][vehicle]["duration"],
                              offDuration=predictions["sf_off"][vehicle]["duration"],
                              onDuration=predictions["sf_on"][vehicle]["duration"],
                              offError=repr(errors["sf_off"][vehicle]), onError=repr(errors["sf_on"][vehicle]),
                              absErrorChange=repr(abs(errors["sf_on"][vehicle]) - abs(errors["sf_off"][vehicle])))
            ET.ElementTree(paired_tree).write(output / parameter_name / f"paired_{scenario}.xml", encoding="utf-8", xml_declaration=True)
            section["datasets"][scenario] = scenario_result
        section["pooled"] = {label: metrics(values.values()) for label, values in pooled.items()}
        section["pooled"]["paired"] = paired_metrics(pooled["sf_off"], pooled["sf_on"])
        for name, low, high in SF_BINS:
            ids = [vehicle for vehicle, sf in pooled_sf.items() if low <= sf < high]
            section["sf_bins_pooled"][name] = {label: metrics(errors[vehicle] for vehicle in ids) for label, errors in pooled.items()}
        summary["parameters"][parameter_name] = section
    return summary


def reconcile_run_records(output, inputs):
    records = []
    for path in sorted(output.glob("*/*/*/run.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        if record.get("returncode") != 0:
            raise ValueError(f"Incomplete or failed simulator run: {path}")
        result, _, _ = evaluate_run(path.parent, inputs[record["scenario"]])
        record["independent_metrics"] = result
        record["output_hashes"] = {name: sha256(path.parent / name) for name in ("prediction_tripinfo.xml", "sumo_eval.csv")}
        write_json(path, record)
        records.append(record)
    if len(records) != 15:
        raise ValueError(f"Expected all 15 fixed-parameter runs, found {len(records)}")
    return records


def write_report(output, summary):
    lines = ["# Speed-net explicit speed-factor ablation", "", "MAE and bias are seconds; bias = C++ duration minus SUMO duration.",
             "Pooled values weight all 45,000 vehicles equally. Initial departure delay is excluded from duration.", "",
             "Only the SF switch differs within each pair. Queue trimming remains off, both corrections are enabled,",
             "lane discharge = 2 s, crossing = 3 s, movement queue order = arrival-time. No parameter retuning.", ""]
    for name, section in summary["parameters"].items():
        lines += [f"## {name}", "", f"Following headway = {section['headway_s']} s; delay maximum = {section['delay_max_s']} s.", "",
                  "| Dataset | Vehicles | SF off MAE | SF off bias | SF on MAE | SF on bias | MAE change |", "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
        for dataset, values in list(section["datasets"].items()) + [("pooled", section["pooled"])]:
            off, on = values["sf_off"], values["sf_on"]
            lines.append(f"| {dataset} | {off['vehicles']:,} | {off['mae_s']:.6f} | {off['bias_s']:.6f} | {on['mae_s']:.6f} | {on['bias_s']:.6f} | {on['mae_s'] - off['mae_s']:+.6f} |")
        lines += ["", "| Paired dataset | Improved | Worsened | Unchanged absolute error | Mean duration change / s |", "| --- | ---: | ---: | ---: | ---: |"]
        for dataset, values in list(section["datasets"].items()) + [("pooled", section["pooled"])]:
            paired = values["paired"]
            lines.append(f"| {dataset} | {paired['improved_vehicles']:,} | {paired['worsened_vehicles']:,} | {paired['unchanged_abs_error_vehicles']:,} | {paired['mean_duration_change_s']:+.6f} |")
        lines += ["", "| Pooled SF group | Vehicles | Off MAE | Off bias | On MAE | On bias |", "| --- | ---: | ---: | ---: | ---: | ---: |"]
        for sf_bin, values in section["sf_bins_pooled"].items():
            off, on = values["sf_off"], values["sf_on"]
            lines.append(f"| {sf_bin} | {off['vehicles']:,} | {off['mae_s']:.6f} | {off['bias_s']:.6f} | {on['mae_s']:.6f} | {on['bias_s']:.6f} |")
        lines += [""]
    main_result = summary["parameters"]["historical_h0p2_d3"]
    main_pooled = main_result["pooled"]
    default_pooled = summary["parameters"]["defaults_h0p8_d6"]["pooled"]
    main_reduction = 100 * (1 - main_pooled["sf_on"]["mae_s"] / main_pooled["sf_off"]["mae_s"])
    default_increase = 100 * (default_pooled["sf_on"]["mae_s"] / default_pooled["sf_off"]["mae_s"] - 1)
    low_sf = main_result["sf_bins_pooled"]["SF < 0.9"]
    reduction_total = main_pooled["sf_off"]["sum_abs_error_s"] - main_pooled["sf_on"]["sum_abs_error_s"]
    low_sf_contribution = 100 * (low_sf["sf_off"]["sum_abs_error_s"] - low_sf["sf_on"]["sum_abs_error_s"]) / reduction_total
    lines += ["## Interpretation", "",
              f"The historical parameter pair improves pooled MAE by {main_reduction:.2f}%; the default following parameters instead increase it by {default_increase:.2f}%.",
              "Thus adding the missing SF information helps in the historical configuration, but does not make existing following-delay calibration universally correct.",
              f"Under the historical pair, SF < 0.9 vehicles are {100 * low_sf['sf_off']['vehicles'] / main_pooled['sf_off']['vehicles']:.2f}% of vehicles and account for {low_sf_contribution:.2f}% of net absolute-error reduction.",
              "This is a descriptive partition by the evaluated vehicle's own SF, not an isolation of direct SF effects from interactions with other vehicles.", ""]
    lines += ["## Regression and coverage", "", "Every route vehicle has explicit finite positive SF and exactly one SUMO and predicted tripinfo record.",
              "In the historical parameter group, all three new-SF-off predictions equal the old binary across every XML field and every byte of the per-vehicle evaluation CSV; the default parameter group was not run with the old binary.",
              "Metrics were independently calculated from tripinfo XML and cross-checked against the evaluation CSV.",
              "The existing exporter uses six significant digits; tiny rounding differences from in-memory/logged metrics are expected. Interpret comparisons to three decimal places.",
              "Full commands, input/binary SHA-256, simulator logs, individual outputs, and paired per-vehicle XML are retained beside this report.", "",
              "This measures closed-loop effects, including changes to vehicle order and signal-window assignment. SF-bin results are descriptive, not a causal decomposition.", ""]
    (output / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--baseline-binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=600)
    parser.add_argument("--analyze-only", action="store_true", help="Recompute metrics from an existing completed run, without rerunning simulators")
    args = parser.parse_args()
    args.repo, args.binary, args.baseline_binary, args.output = [path.resolve() for path in (args.repo, args.binary, args.baseline_binary, args.output)]
    if args.workers < 1:
        parser.error("--workers must be positive")
    inputs, input_hashes = load_inputs(args.repo)
    binary_hashes = {str(path): sha256(path) for path in (args.binary, args.baseline_binary)}
    if not args.analyze_only:
        args.output.mkdir(parents=True, exist_ok=False)
        manifest = {"repo": str(args.repo), "script_sha256": sha256(__file__), "input_hashes": input_hashes,
                    "binary_hashes": binary_hashes, "expected_vehicles": SCENARIOS, "parameters": PARAMETERS,
                    "common_arguments": COMMON_ARGUMENTS, "status": "running"}
        write_json(args.output / "experiment.json", manifest)
        tasks = []
        for parameter_name in PARAMETERS:
            labels = ("old_binary", "sf_off", "sf_on") if parameter_name == "historical_h0p2_d3" else ("sf_off", "sf_on")
            for label in labels:
                for scenario in SCENARIOS:
                    tasks.append((args.output, args.baseline_binary if label == "old_binary" else args.binary,
                                  scenario, label, parameter_name, inputs[scenario], args.timeout))
        started = time.perf_counter()
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = [pool.submit(execute_run, *task) for task in tasks]
            for future in concurrent.futures.as_completed(futures):
                future.result()
        manifest["elapsed_seconds"] = time.perf_counter() - started
    else:
        manifest = json.loads((args.output / "experiment.json").read_text(encoding="utf-8"))
        if manifest["input_hashes"] != input_hashes or manifest["binary_hashes"] != binary_hashes:
            raise ValueError("Refusing re-analysis after inputs or binaries changed")
    for path, expected_hash in {**input_hashes, **binary_hashes}.items():
        if sha256(path) != expected_hash:
            raise ValueError(f"Source input or binary changed during experiment: {path}")
    summary = collect_summary(args.output, inputs)
    completed_runs = reconcile_run_records(args.output, inputs)
    summary["input_hashes"] = input_hashes
    summary["binary_hashes"] = binary_hashes
    write_json(args.output / "summary.json", summary)
    write_report(args.output, summary)
    manifest["status"] = "success"
    manifest["source_hashes_unchanged"] = True
    manifest["completed_run_count"] = len(completed_runs)
    manifest["sum_simulator_elapsed_seconds"] = math.fsum(record["elapsed_seconds"] for record in completed_runs)
    manifest["analysis_script_sha256"] = sha256(__file__)
    write_json(args.output / "experiment.json", manifest)
    print((args.output / "report.md").read_text(encoding="utf-8"), flush=True)


if __name__ == "__main__":
    main()
