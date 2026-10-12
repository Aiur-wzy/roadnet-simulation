#!/usr/bin/env python3
"""Join per-vehicle CAMS evaluation with TraCI missed-green outcomes."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def parse_float(value, field, row_number, path, required=True):
    if value in (None, ""):
        if required:
            raise ValueError("{}:{} missing {}".format(path, row_number, field))
        return None
    try:
        result = float(value)
    except ValueError:
        raise ValueError("{}:{} invalid {}={!r}".format(path, row_number, field, value))
    if not math.isfinite(result):
        raise ValueError("{}:{} non-finite {}={!r}".format(path, row_number, field, value))
    return result


def parse_bool(value):
    return str(value or "").strip().lower() in {"1", "true", "yes"}


def percentile(values, percent):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction


def mean(values):
    return sum(values) / len(values) if values else math.nan


def require_columns(reader, required, path):
    missing = [column for column in required if column not in (reader.fieldnames or [])]
    if missing:
        raise ValueError("{} missing columns: {}".format(path, ", ".join(missing)))


def read_eval(path):
    vehicles = {}
    invalid_skipped = 0
    with open(path, newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        require_columns(reader, ["vehicleID", "predDuration", "truthDuration"], path)
        for row_number, row in enumerate(reader, start=2):
            if str(row.get("validVehicle", "true")).strip().lower() == "false":
                invalid_skipped += 1
                continue
            vehicle_id = (row.get("vehicleID") or "").strip()
            if not vehicle_id:
                raise ValueError("{}:{} missing vehicleID".format(path, row_number))
            if vehicle_id in vehicles:
                raise ValueError("{}:{} duplicate vehicleID {!r}".format(path, row_number, vehicle_id))
            pred = parse_float(row.get("predDuration"), "predDuration", row_number, path)
            truth = parse_float(row.get("truthDuration"), "truthDuration", row_number, path)
            vehicles[vehicle_id] = {
                "vehicleID": vehicle_id,
                "predDuration": pred,
                "truthDuration": truth,
                "durationError": pred - truth,
                "absDurationError": abs(pred - truth),
                "truthWaitingTime": parse_float(
                    row.get("truthWaitingTime"), "truthWaitingTime", row_number, path, required=False
                ),
            }
    return vehicles, invalid_skipped


def read_traci(path):
    vehicles = {}
    with open(path, newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        require_columns(reader, ["vehicle_id", "missed_green_rounds"], path)
        for row_number, row in enumerate(reader, start=2):
            vehicle_id = (row.get("vehicle_id") or "").strip()
            if not vehicle_id:
                raise ValueError("{}:{} missing vehicle_id".format(path, row_number))
            if vehicle_id in vehicles:
                raise ValueError("{}:{} duplicate vehicle_id {!r}".format(path, row_number, vehicle_id))
            rounds_value = parse_float(
                row.get("missed_green_rounds"), "missed_green_rounds", row_number, path
            )
            rounds = int(rounds_value)
            if rounds < 0 or abs(rounds_value - rounds) > 1e-9:
                raise ValueError(
                    "{}:{} missed_green_rounds must be a non-negative integer".format(path, row_number)
                )
            vehicles[vehicle_id] = {
                "missedGreenRounds": rounds,
                "successfulGreenRounds": parse_float(
                    row.get("successful_green_rounds"),
                    "successful_green_rounds",
                    row_number,
                    path,
                    required=False,
                ),
                "totalGreenRounds": parse_float(
                    row.get("total_green_rounds"),
                    "total_green_rounds",
                    row_number,
                    path,
                    required=False,
                ),
                "vehicleTeleported": parse_bool(row.get("vehicle_teleported")),
            }
    return vehicles


def group_label(rounds, round_cap):
    if round_cap is not None and rounds >= round_cap:
        return "{}+".format(round_cap)
    return str(rounds)


def summarize(group_name, rows, total_count):
    errors = [row["durationError"] for row in rows]
    absolute_errors = [abs(error) for error in errors]
    truths = [row["truthDuration"] for row in rows]
    predictions = [row["predDuration"] for row in rows]
    waiting = [row["truthWaitingTime"] for row in rows if row["truthWaitingTime"] is not None]
    successful = [
        row["successfulGreenRounds"]
        for row in rows
        if row["successfulGreenRounds"] is not None
    ]
    total_rounds = [row["totalGreenRounds"] for row in rows if row["totalGreenRounds"] is not None]
    squared_errors = [error * error for error in errors]
    percentage_errors = [
        abs(error) / truth * 100.0
        for error, truth in zip(errors, truths)
        if truth > 0.0
    ]
    mse = mean(squared_errors)
    return {
        "groupType": "missedGreenRounds",
        "groupName": group_name,
        "n": len(rows),
        "shareOfMatched": len(rows) / total_count if total_count else 0.0,
        "meanTruthDuration": mean(truths),
        "meanPredDuration": mean(predictions),
        "bias": mean(errors),
        "mae": mean(absolute_errors),
        "mse": mse,
        "rmse": math.sqrt(mse),
        "mape": mean(percentage_errors),
        "medianAE": percentile(absolute_errors, 50.0),
        "p90AE": percentile(absolute_errors, 90.0),
        "p95AE": percentile(absolute_errors, 95.0),
        "meanTruthWaitingTime": mean(waiting),
        "meanSuccessfulGreenRounds": mean(successful),
        "meanTotalGreenRounds": mean(total_rounds),
        "teleportedVehicles": sum(1 for row in rows if row["vehicleTeleported"]),
    }


def analyze(eval_path, traci_path, round_cap=None):
    if round_cap is not None and round_cap < 1:
        raise ValueError("round_cap must be >= 1")
    eval_vehicles, invalid_skipped = read_eval(eval_path)
    traci_vehicles = read_traci(traci_path)

    matched = []
    groups = defaultdict(list)
    for vehicle_id, eval_row in eval_vehicles.items():
        traci_row = traci_vehicles.get(vehicle_id)
        if traci_row is None:
            continue
        joined = dict(eval_row)
        joined.update(traci_row)
        joined["roundGroup"] = group_label(joined["missedGreenRounds"], round_cap)
        matched.append(joined)
        groups[joined["roundGroup"]].append(joined)

    if not matched:
        raise ValueError("no vehicle IDs matched between eval and TraCI inputs")

    def group_sort_key(label):
        return int(label.rstrip("+"))

    summaries = [summarize("ALL", matched, len(matched))]
    summaries.extend(
        summarize(label, groups[label], len(matched))
        for label in sorted(groups, key=group_sort_key)
    )
    coverage = {
        "validEvalVehicles": len(eval_vehicles),
        "invalidEvalVehiclesSkipped": invalid_skipped,
        "traciVehicles": len(traci_vehicles),
        "matchedVehicles": len(matched),
        "evalVehiclesMissingTraCI": len(set(eval_vehicles) - set(traci_vehicles)),
        "traciVehiclesMissingEval": len(set(traci_vehicles) - set(eval_vehicles)),
    }
    return summaries, matched, coverage


def format_value(value):
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return "{:.12g}".format(value)
    return value


def write_rows(path, rows):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0])
    with open(output, "w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: format_value(row.get(key)) for key in fieldnames})


def main():
    parser = argparse.ArgumentParser(
        description="Calculate duration MAE grouped by TraCI missed-green rounds."
    )
    parser.add_argument("--eval-csv", required=True, help="Per-vehicle C++ sumo_eval.csv")
    parser.add_argument("--traci-csv", required=True, help="TraCI_vehicle_green_rounds_*.csv")
    parser.add_argument("--output-csv", required=True, help="Grouped metric output CSV")
    parser.add_argument(
        "--joined-output-csv",
        help="Optional matched per-vehicle output with eval and missed-green fields",
    )
    parser.add_argument(
        "--round-cap",
        type=int,
        help="Combine this many or more missed rounds into an N+ group",
    )
    args = parser.parse_args()

    summaries, joined, coverage = analyze(args.eval_csv, args.traci_csv, args.round_cap)
    write_rows(args.output_csv, summaries)
    if args.joined_output_csv:
        write_rows(args.joined_output_csv, joined)

    print("grouped_output={}".format(args.output_csv))
    if args.joined_output_csv:
        print("joined_output={}".format(args.joined_output_csv))
    print("coverage=" + " ".join("{}={}".format(key, value) for key, value in coverage.items()))


if __name__ == "__main__":
    main()
