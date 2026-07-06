#!/usr/bin/env python3
"""Summarize SUMO evaluation CSV metrics by vehicle-id group."""

import argparse
import collections
import csv
import math
import statistics

FIELDS = [
    "dataset", "group", "comparedVehicles", "positiveErrorCount", "negativeErrorCount", "zeroErrorCount",
    "meanPredDuration", "meanTruthDuration", "bias", "MAE", "MSE", "RMSE", "MAPE",
    "medianAbsError", "p90AbsError", "p95AbsError", "maxAbsError",
    "absDurationError_gt_300", "absDurationError_gt_600", "absDurationError_gt_1000",
    "relativeDurationError_gt_1_0", "relativeDurationError_gt_3_0", "relativeDurationError_gt_5_0",
    "predDuration_gt_1000", "predDuration_gt_2000", "truthDuration_gt_1000", "truthDuration_gt_2000",
    "meanPredSpeed", "meanTruthSpeed", "speedBias", "speedMAE",
    "meanPredArrival", "meanTruthArrival", "arrivalBias", "arrivalMAE", "arrivalRMSE",
    "topAbsErrorVehicleID", "topAbsErrorPredDuration", "topAbsErrorTruthDuration",
    "topAbsErrorSignedError", "topAbsErrorAbsError",
]


def num(row, name):
    value = row.get(name, "")
    if value == "" or value is None:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def group_for(vehicle_id, single_group=None):
    if single_group:
        return single_group
    marker = "veh"
    if marker in vehicle_id:
        prefix = vehicle_id.split(marker, 1)[0].rstrip("_")
        return prefix or "ungrouped"
    if "_" in vehicle_id:
        parts = vehicle_id.split("_")
        if parts[-1].isdigit() and len(parts) > 1:
            return "_".join(parts[:-1]) or "ungrouped"
    return "ungrouped"


def percentile(values, pct):
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[int(pos)]
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (pos - lo)


def mean(values):
    vals = [v for v in values if v is not None]
    return statistics.fmean(vals) if vals else math.nan


def fmt(value):
    if isinstance(value, str):
        return value
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return ""
    if isinstance(value, int):
        return str(value)
    return f"{value:.6g}"


def summarize(dataset, group, rows):
    signed = [r["signed"] for r in rows]
    abs_errors = [abs(v) for v in signed]
    rel = [r["rel"] for r in rows if r["rel"] is not None]
    pred_d = [r["predDuration"] for r in rows]
    truth_d = [r["truthDuration"] for r in rows]
    pred_s = [r["predSpeed"] for r in rows if r["predSpeed"] is not None]
    truth_s = [r["truthSpeed"] for r in rows if r["truthSpeed"] is not None]
    speed_e = [r["speedError"] for r in rows if r["speedError"] is not None]
    pred_a = [r["predArrival"] for r in rows if r["predArrival"] is not None]
    truth_a = [r["truthArrival"] for r in rows if r["truthArrival"] is not None]
    arr_e = [r["arrivalSigned"] for r in rows if r["arrivalSigned"] is not None]
    top = max(rows, key=lambda r: abs(r["signed"]))
    return {
        "dataset": dataset, "group": group, "comparedVehicles": len(rows),
        "positiveErrorCount": sum(1 for v in signed if v > 0),
        "negativeErrorCount": sum(1 for v in signed if v < 0),
        "zeroErrorCount": sum(1 for v in signed if v == 0),
        "meanPredDuration": mean(pred_d), "meanTruthDuration": mean(truth_d), "bias": mean(signed),
        "MAE": mean(abs_errors), "MSE": mean([v * v for v in signed]), "RMSE": math.sqrt(mean([v * v for v in signed])),
        "MAPE": mean([abs(r["signed"] / r["truthDuration"]) * 100 for r in rows if r["truthDuration"] != 0]),
        "medianAbsError": statistics.median(abs_errors), "p90AbsError": percentile(abs_errors, 90),
        "p95AbsError": percentile(abs_errors, 95), "maxAbsError": max(abs_errors),
        "absDurationError_gt_300": sum(1 for v in abs_errors if v > 300),
        "absDurationError_gt_600": sum(1 for v in abs_errors if v > 600),
        "absDurationError_gt_1000": sum(1 for v in abs_errors if v > 1000),
        "relativeDurationError_gt_1_0": sum(1 for v in rel if v > 1.0),
        "relativeDurationError_gt_3_0": sum(1 for v in rel if v > 3.0),
        "relativeDurationError_gt_5_0": sum(1 for v in rel if v > 5.0),
        "predDuration_gt_1000": sum(1 for v in pred_d if v > 1000),
        "predDuration_gt_2000": sum(1 for v in pred_d if v > 2000),
        "truthDuration_gt_1000": sum(1 for v in truth_d if v > 1000),
        "truthDuration_gt_2000": sum(1 for v in truth_d if v > 2000),
        "meanPredSpeed": mean(pred_s), "meanTruthSpeed": mean(truth_s), "speedBias": mean(speed_e),
        "speedMAE": mean([abs(v) for v in speed_e]),
        "meanPredArrival": mean(pred_a), "meanTruthArrival": mean(truth_a), "arrivalBias": mean(arr_e),
        "arrivalMAE": mean([abs(v) for v in arr_e]), "arrivalRMSE": math.sqrt(mean([v * v for v in arr_e])) if arr_e else math.nan,
        "topAbsErrorVehicleID": top["vehicleID"], "topAbsErrorPredDuration": top["predDuration"],
        "topAbsErrorTruthDuration": top["truthDuration"], "topAbsErrorSignedError": top["signed"],
        "topAbsErrorAbsError": abs(top["signed"]),
    }


def md_table(title, rows, key):
    cols = ["group", "comparedVehicles", "bias", "MAE", "RMSE", "medianAbsError", "p95AbsError", "absDurationError_gt_1000", "topAbsErrorVehicleID"]
    out = [f"## {title}", "", "| " + " | ".join(cols) + " |", "| " + " | ".join(["---"] * len(cols)) + " |"]
    for row in sorted(rows, key=lambda r: r[key], reverse=True):
        out.append("| " + " | ".join(fmt(row[c]) for c in cols) + " |")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval-csv", required=True)
    ap.add_argument("--dataset-name", required=True)
    ap.add_argument("--single-group")
    ap.add_argument("--output-csv", required=True)
    ap.add_argument("--output-md", required=True)
    args = ap.parse_args()
    grouped = collections.defaultdict(list); all_rows = []
    with open(args.eval_csv, newline="") as f:
        for row in csv.DictReader(f):
            if row.get("validVehicle", "true").lower() == "false":
                continue
            signed = num(row, "durationErrorSigned")
            pred_d = num(row, "predDuration"); truth_d = num(row, "truthDuration")
            if signed is None or pred_d is None or truth_d is None:
                continue
            rec = {"vehicleID": row.get("vehicleID", ""), "signed": signed, "predDuration": pred_d, "truthDuration": truth_d,
                   "rel": num(row, "relativeDurationError"), "predSpeed": num(row, "predAvgSpeed"), "truthSpeed": num(row, "truthAvgSpeed"),
                   "speedError": num(row, "speedError"), "predArrival": num(row, "predArrival"), "truthArrival": num(row, "truthArrival"),
                   "arrivalSigned": num(row, "arrivalErrorSigned")}
            grouped[group_for(rec["vehicleID"], args.single_group)].append(rec); all_rows.append(rec)
    summaries = [summarize(args.dataset_name, g, rows) for g, rows in sorted(grouped.items())]
    summaries.append(summarize(args.dataset_name, "ALL", all_rows))
    with open(args.output_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, lineterminator="\n"); w.writeheader(); w.writerows({k: fmt(r.get(k)) for k in FIELDS} for r in summaries)
    worst_mae = max((r for r in summaries if r["group"] != "ALL"), key=lambda r: r["MAE"])
    worst_rmse = max((r for r in summaries if r["group"] != "ALL"), key=lambda r: r["RMSE"])
    worst_severe = max((r for r in summaries if r["group"] != "ALL"), key=lambda r: r["absDurationError_gt_1000"])
    md = [f"# Grouped evaluation summary: {args.dataset_name}", "", md_table("Sorted by MAE descending", summaries, "MAE"), "", md_table("Sorted by RMSE descending", summaries, "RMSE"), "", md_table("Sorted by absDurationError > 1000 descending", summaries, "absDurationError_gt_1000"), "", "## Notes", "", f"- Highest MAE group: `{worst_mae['group']}` ({fmt(worst_mae['MAE'])} s).", f"- Highest RMSE group: `{worst_rmse['group']}` ({fmt(worst_rmse['RMSE'])} s).", f"- Most severe-duration-error vehicles (>1000 s): `{worst_severe['group']}` ({worst_severe['absDurationError_gt_1000']})."]
    with open(args.output_md, "w") as f:
        f.write("\n".join(md) + "\n")

if __name__ == "__main__":
    main()
