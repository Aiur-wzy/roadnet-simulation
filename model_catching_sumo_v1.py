#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build a fast lookup table for a SUMO/TraCI travel-time model trained from one
or many TraCI CSV files.

The current model_training_sumo_v1.py trains with exactly these input features:
    has_waiting, road_length, turn_type, road_flow, lane_flow
and predicts:
    travel_time_no_waiting

This script enumerates feature combinations, calls the trained AutoGluon model
in batches, and writes a text table that can be loaded by the macroscopic
simulator instead of calling the model repeatedly.

Python version:
- Compatible with Python 3.6+; avoids PEP 585 built-in generic type annotations.

Single-CSV usage:
    python model_catching_sumo_v1.py \
        --csv TraCI_output_adjusted.csv \
        --model_path models_v1 \
        --output_txt model_catching_sumo_v1.txt \
        --max_road_flow 200

Multi-CSV usage (one model trained from 80 TraCI files):
    python model_catching_sumo_v1.py \
        --csv-glob 'train_data/outputs/traci/TraCI_output_adjusted_*.csv' \
        --expected-csv-count 80 \
        --model_path models/all80_no_q99_no_tail \
        --output_txt catch/model_catching_sumo_all80_no_q99_no_tail.txt \
        --max_road_flow 200 \
        --max_lane_flow 80 \
        --lane_flow_step 1 \
        --no-q99-filter

Important:
- All resolved CSV files are merged only to determine the observed feature
  domain.  The AutoGluon model is loaded once and one combined lookup table is
  produced.
- turn_type is read directly from the numeric CSV column using the fixed
  SUMO/C++ encoding.
- lane_flow is read directly from the CSV and enumerated as an independent
  selected-lane vehicle count constrained to lane_flow <= road_flow.
- P99 filtering is disabled by default.  Tail sample reweighting is a training
  concern already stored in the model and has no catching-time switch.
"""

import argparse
import glob
import json
import math
import os
import warnings
from pathlib import Path

import numpy as np
import pandas as pd

warnings.filterwarnings("ignore", message="load_learner` uses Python's insecure pickle module")

FEATURE_COLS = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
LABEL_COL = "travel_time_no_waiting"
EPS = 1e-6


def positive_int(value):
    out = int(value)
    if out <= 0:
        raise argparse.ArgumentTypeError("value must be a positive integer")
    return out


def positive_float(value):
    out = float(value)
    if out <= 0:
        raise argparse.ArgumentTypeError("value must be a positive float")
    return out


SLIM_COLS = FEATURE_COLS + ["travel_time_label"]
TURN_TYPE_MAP = {"l": 1, "L": 1, "s": 2, "r": 3, "R": 3, "t": 4}
DEFAULT_CSV = "TraCI_output_adjusted.csv"


def _encode_legacy_turn(value) -> int:
    if pd.isna(value):
        return 0
    return TURN_TYPE_MAP.get(str(value), 0)


def _logical_abspath(path, base_dir=""):
    """Return an absolute path without resolving symlink targets."""
    expanded = os.path.expandvars(os.path.expanduser(str(path).strip()))
    if not os.path.isabs(expanded):
        expanded = os.path.join(base_dir or os.getcwd(), expanded)
    return os.path.abspath(expanded)


def resolve_csv_inputs(csv_paths, csv_globs, csv_lists, expected_count=0):
    """Resolve literal, glob and newline-list inputs into a deterministic list."""
    candidates = []

    for csv_path in csv_paths:
        candidates.append(_logical_abspath(csv_path))

    for pattern in csv_globs:
        expanded_pattern = _logical_abspath(pattern)
        matches = sorted(
            path for path in glob.glob(expanded_pattern, recursive=True)
            if os.path.isfile(path)
        )
        if not matches:
            raise ValueError("CSV glob matched no files: {}".format(pattern))
        candidates.extend(matches)

    for list_path_raw in csv_lists:
        list_path = _logical_abspath(list_path_raw)
        if not os.path.isfile(list_path):
            raise FileNotFoundError("CSV list file not found: {}".format(list_path))
        list_dir = os.path.dirname(list_path)
        with open(list_path, "r", encoding="utf-8-sig") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                candidates.append(_logical_abspath(line, base_dir=list_dir))

    if not candidates:
        candidates.append(_logical_abspath(DEFAULT_CSV))

    unique_paths = sorted(set(candidates))
    duplicate_count = len(candidates) - len(unique_paths)
    if duplicate_count:
        print("[WARN] ignored duplicate CSV inputs: {}".format(duplicate_count))

    missing = [path for path in unique_paths if not os.path.isfile(path)]
    if missing:
        preview = "\n  ".join(missing[:10])
        suffix = "" if len(missing) <= 10 else "\n  ... and {} more".format(len(missing) - 10)
        raise FileNotFoundError("CSV input files not found:\n  {}{}".format(preview, suffix))

    if expected_count < 0:
        raise ValueError("--expected-csv-count must be >= 0")
    if expected_count and len(unique_paths) != expected_count:
        raise ValueError(
            "Expected {} CSV files, resolved {}. Refusing to build a table from "
            "an incomplete or over-broad input set.".format(expected_count, len(unique_paths))
        )

    print("[INFO] resolved CSV files: {}".format(len(unique_paths)))
    for index, path in enumerate(unique_paths, start=1):
        print("[INFO] input [{}/{}]: {}".format(index, len(unique_paths), path))
    return unique_paths


def _load_one_training_like_frame(csv_path):
    """Load one slim base-feature frame used by model_training_sumo_v1.py."""
    columns = set(pd.read_csv(csv_path, nrows=0).columns)
    if set(SLIM_COLS).issubset(columns):
        print("[INFO] {}: detected slim/full CAMS training schema".format(csv_path))
        df = pd.read_csv(csv_path, usecols=SLIM_COLS)
    else:
        legacy_required = {"E_Length", "Driving_Num", "Turn", "Travel_Time", "LowSpee_Time", "lane_flow"}
        missing = sorted(legacy_required - columns)
        if missing:
            raise ValueError(f"Missing required slim columns in {csv_path}: {SLIM_COLS}; legacy fallback missing: {missing}")
        print("[WARN] {}: detected older legacy TraCI schema; using compatibility conversion".format(csv_path))
        usecols = [c for c in legacy_required | {"turn_type", "has_waiting", "Wait_Time", "travel_time_label"} if c in columns]
        raw = pd.read_csv(csv_path, usecols=usecols)
        df = pd.DataFrame()
        df["has_waiting"] = raw["has_waiting"] if "has_waiting" in raw.columns else ((raw["Wait_Time"].fillna(0).astype(float) > 0).astype(int) if "Wait_Time" in raw.columns else 0)
        df["road_length"] = raw["E_Length"].astype(float)
        df["turn_type"] = raw["turn_type"] if "turn_type" in raw.columns else raw["Turn"].map(_encode_legacy_turn)
        df["road_flow"] = raw["Driving_Num"].astype(float)
        df["lane_flow"] = raw["lane_flow"].astype(float)
        df["travel_time_label"] = raw["travel_time_label"] if "travel_time_label" in raw.columns else raw["Travel_Time"].fillna(0).astype(float) + raw["LowSpee_Time"].fillna(0).astype(float)

    df = df.dropna(subset=SLIM_COLS).copy()
    df["has_waiting"] = df["has_waiting"].fillna(0).astype(int).astype("category")
    df["road_length"] = df["road_length"].astype(float)
    df["turn_type"] = df["turn_type"].fillna(0).astype(int)
    df["road_flow"] = df["road_flow"].fillna(0).astype(float)
    df["lane_flow"] = df["lane_flow"].fillna(0).astype(float)
    df[LABEL_COL] = df["travel_time_label"].astype(float)

    invalid = (df[LABEL_COL] <= 0) | (df["road_length"] <= 0) | (df["road_flow"] < 0) | (df["lane_flow"] < 0)
    if invalid.any():
        print("[INFO] {}: dropped invalid rows before enumeration: {}".format(csv_path, int(invalid.sum())))
        df = df.loc[~invalid].copy()

    if df.empty:
        raise ValueError("No valid catching rows remain in {}".format(csv_path))

    print("[INFO] {}: valid rows={}".format(csv_path, len(df)))
    return df[FEATURE_COLS + [LABEL_COL]].reset_index(drop=True)


def load_training_like_frame(csv_paths, q99_filter=False, q99_quantile=0.99):
    """Load and merge one or many training-like CSVs for domain enumeration."""
    if isinstance(csv_paths, str):
        normalized_paths = [csv_paths]
    else:
        normalized_paths = list(csv_paths)
    if not normalized_paths:
        raise ValueError("At least one CSV input is required")
    if not 0.0 < q99_quantile < 1.0:
        raise ValueError("--q99-quantile must be between 0 and 1")

    frames = [_load_one_training_like_frame(path) for path in normalized_paths]
    df = pd.concat(frames, ignore_index=True, copy=False)
    df["has_waiting"] = df["has_waiting"].astype(int).astype("category")
    print("[INFO] merged valid rows before optional P99 filtering: {}".format(len(df)))

    if q99_filter:
        q99 = float(df[LABEL_COL].quantile(q99_quantile))
        before = len(df)
        df = df[df[LABEL_COL] <= q99].reset_index(drop=True)
        print("[INFO] P99 filter: ENABLED (quantile={})".format(q99_quantile))
        print("[INFO] P99 threshold: {}; rows removed: {}".format(q99, before - len(df)))
    else:
        df = df.reset_index(drop=True)
        print("[INFO] P99 filter: DISABLED (all valid tail rows retained)")

    if df.empty:
        raise ValueError("No rows remain after optional P99 filtering")

    turn_mapping = {"unknown/end/missing": 0, "l/L": 1, "s": 2, "r/R": 3, "t": 4}
    return df[FEATURE_COLS + [LABEL_COL]], turn_mapping

def float_range(start, stop, step, precision):
    """Inclusive floating-point range with stable rounding."""
    if stop < start:
        return []
    n = int(math.floor((stop - start) / step + 1e-9))
    values = [round(start + i * step, precision) for i in range(n + 1)]
    if not values or values[-1] < round(stop, precision):
        values.append(round(stop, precision))
    return sorted(set(values))


def save_unique_feature_combinations(df, output_csv):
    unique_df = df[FEATURE_COLS].drop_duplicates().sort_values(FEATURE_COLS).reset_index(drop=True)
    unique_df.to_csv(output_csv, index=False)
    print(f"[INFO] unique feature combinations from training CSV: {unique_df.shape[0]} -> {output_csv}")


def save_turn_mapping(turn_mapping, output_json):
    with open(output_json, "w", encoding="utf-8") as f:
        json.dump(turn_mapping, f, ensure_ascii=False, indent=2)
    print(f"[INFO] turn_type mapping -> {output_json}: {turn_mapping}")


def get_predictor_features(predictor):
    """Read model feature list from AutoGluon when available; fall back to FEATURE_COLS."""
    try:
        feats = list(predictor.feature_metadata.get_features())
        if feats:
            return feats
    except Exception:
        pass
    return FEATURE_COLS.copy()


def validate_model_features(predictor_features):
    missing = [c for c in predictor_features if c not in FEATURE_COLS]
    extra = [c for c in FEATURE_COLS if c not in predictor_features]
    if missing:
        raise ValueError(
            "The loaded model expects features that this catching script does not generate: "
            f"{missing}. Current script is aligned with the base-feature SUMO v1 model."
        )
    if extra:
        print(f"[WARN] These base columns are generated but not required by the model: {extra}")


def iter_cases(
    road_lengths,
    turn_types,
    road_flow_values,
    lane_flow_values,
    precision,
):
    has_waiting_values = [False, True]

    for road_length in road_lengths:
        for turn_type in turn_types:
            for has_waiting in has_waiting_values:
                for road_flow in road_flow_values:
                    current_lane_flows = [lf for lf in lane_flow_values if float(lf) <= float(road_flow)]
                    for lane_flow in current_lane_flows:
                        yield {
                            "has_waiting": bool(has_waiting),
                            "road_length": float(road_length),
                            "turn_type": int(turn_type),
                            "road_flow": float(road_flow),
                            "lane_flow": float(lane_flow),
                        }


def batch_iter(iterator, batch_size):
    batch = []
    for item in iterator:
        batch.append(item)
        if len(batch) >= batch_size:
            yield batch
            batch = []
    if batch:
        yield batch


def fmt_value(v) -> str:
    if isinstance(v, (bool, np.bool_)):
        return str(bool(v)).lower()
    if isinstance(v, (float, np.floating)):
        return f"{float(v):.12g}"
    return str(v)


def write_txt_header(path, columns):
    with open(path, "w", encoding="utf-8") as f:
        f.write(" ".join(columns) + "\n")


def append_txt_rows(path, df, columns):
    with open(path, "a", encoding="utf-8") as f:
        for row in df[columns].itertuples(index=False, name=None):
            f.write(" ".join(fmt_value(v) for v in row) + "\n")


def append_csv_rows(path, df, header):
    df.to_csv(path, mode="w" if header else "a", header=header, index=False)


def ensure_parent_dir(path):
    parent = Path(path).expanduser().parent
    parent.mkdir(parents=True, exist_ok=True)


def build_cache_table(args):
    df, turn_mapping = load_training_like_frame(
        args.csv_files,
        q99_filter=args.q99_filter,
        q99_quantile=args.q99_quantile,
    )

    ensure_parent_dir(args.output_txt)
    ensure_parent_dir(args.unique_csv)
    ensure_parent_dir(args.turn_mapping_json)
    if args.output_csv:
        ensure_parent_dir(args.output_csv)

    save_unique_feature_combinations(df, args.unique_csv)
    save_turn_mapping(turn_mapping, args.turn_mapping_json)

    road_lengths = sorted(float(x) for x in df["road_length"].dropna().unique())
    turn_types = sorted(int(x) for x in df["turn_type"].dropna().unique())
    if args.max_road_flow is None:
        max_road_flow = int(math.ceil(float(df["road_flow"].max())))
    else:
        max_road_flow = args.max_road_flow
    road_flow_values = list(range(0, max_road_flow + 1, args.road_flow_step))

    max_lane_flow = args.max_lane_flow
    if max_lane_flow is None:
        max_lane_flow = float(max_road_flow)
    lane_flow_values = float_range(0.0, float(max_lane_flow), args.lane_flow_step, args.precision)

    print("[INFO] table dimensions:")
    print(f"       road_length values = {len(road_lengths)}")
    print(f"       turn_type values   = {turn_types}")
    print(f"       has_waiting values = [false, true]")
    print(f"       road_flow values   = 0..{max_road_flow} step {args.road_flow_step}")
    print(f"       lane_flow mode     = independent selected-lane count, 0..min({args.max_lane_flow or max_road_flow}, road_flow) step {args.lane_flow_step}")

    try:
        from autogluon.tabular import TabularPredictor
    except ImportError as exc:
        raise RuntimeError(
            "AutoGluon is required for catching. Run this script in the same "
            "environment used to train the model."
        ) from exc

    predictor = TabularPredictor.load(args.model_path)
    predictor_features = get_predictor_features(predictor)
    validate_model_features(predictor_features)
    predictor_features = [c for c in FEATURE_COLS if c in predictor_features]
    output_columns = FEATURE_COLS + [LABEL_COL]

    txt_path = Path(args.output_txt)
    csv_path = Path(args.output_csv) if args.output_csv else None

    write_txt_header(txt_path, output_columns)
    if csv_path is not None and csv_path.exists():
        csv_path.unlink()

    case_iterator = iter_cases(
        road_lengths=road_lengths,
        turn_types=turn_types,
        road_flow_values=road_flow_values,
        lane_flow_values=lane_flow_values,
        precision=args.precision,
    )

    total = 0
    for batch_no, batch in enumerate(batch_iter(case_iterator, args.chunk_size), start=1):
        cases_df = pd.DataFrame(batch, columns=FEATURE_COLS)
        # Keep dtypes close to the training frame.
        cases_df["has_waiting"] = cases_df["has_waiting"].astype("category")
        cases_df["turn_type"] = cases_df["turn_type"].astype(int)

        preds = predictor.predict(cases_df[predictor_features])
        preds = pd.Series(preds).astype(float)
        if args.clip_min is not None:
            preds = preds.clip(lower=float(args.clip_min))

        out_df = cases_df[FEATURE_COLS].copy()
        out_df[LABEL_COL] = preds.to_numpy()

        append_txt_rows(txt_path, out_df, output_columns)
        if csv_path is not None:
            append_csv_rows(csv_path, out_df, header=(batch_no == 1))

        total += len(out_df)
        print(f"[INFO] batch {batch_no}: wrote {len(out_df)} rows, total={total}")

    print(f"[DONE] lookup table rows: {total}")
    print(f"[DONE] txt output -> {txt_path}")
    if csv_path is not None:
        print(f"[DONE] csv output -> {csv_path}")


def build_argparser():
    p = argparse.ArgumentParser(
        description="Build one SUMO v1 AutoGluon lookup table from one or many TraCI CSV files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--csv",
        action="append",
        default=[],
        metavar="PATH",
        help="Input TraCI CSV path. Repeat --csv to combine multiple explicit files.",
    )
    p.add_argument(
        "--csv-glob",
        "--csv_glob",
        action="append",
        default=[],
        metavar="PATTERN",
        help="Quoted glob pattern for multiple TraCI CSV files; may be repeated.",
    )
    p.add_argument(
        "--csv-list",
        "--csv_list",
        action="append",
        default=[],
        metavar="PATH",
        help="Text file containing one TraCI CSV path per line; relative entries use the list file directory.",
    )
    p.add_argument(
        "--expected-csv-count",
        "--expected_csv_count",
        type=int,
        default=0,
        help="Fail unless exactly this many unique CSV files are resolved; 0 disables the check.",
    )
    p.add_argument("--model_path", default="models_v1", help="AutoGluon model directory saved by model_training_sumo_v1.py")
    p.add_argument("--output_txt", default="model_catching_sumo_v1.txt", help="Space-separated lookup table output")
    p.add_argument("--output_csv", default="", help="Optional CSV output path; leave empty to disable")
    p.add_argument("--unique_csv", default="unique_feature_combinations_sumo_v1.csv", help="Unique observed feature combinations output")
    p.add_argument("--turn_mapping_json", default="turn_type_mapping_sumo_v1.json", help="Raw Turn -> encoded turn_type mapping output")

    p.add_argument("--max_road_flow", type=int, default=200, help="Maximum road_flow to enumerate; use -1 to infer from CSV max")
    p.add_argument("--road_flow_step", type=positive_int, default=1, help="road_flow enumeration step")

    p.add_argument("--max_lane_flow", type=float, default=None, help="Maximum selected-lane lane_flow to enumerate; default=max_road_flow")
    p.add_argument("--lane_flow_step", type=positive_float, default=1.0, help="lane_flow step for independent mode")

    p.add_argument("--precision", type=int, default=6, help="Decimal precision for lane_flow values")
    p.add_argument("--chunk_size", type=positive_int, default=200000, help="Prediction batch size")
    p.add_argument("--clip_min", type=float, default=0.0, help="Clip predicted travel_time_no_waiting lower bound; set to nan to disable")

    q99_group = p.add_mutually_exclusive_group()
    q99_group.add_argument(
        "--q99-filter",
        "--enable-q99-filter",
        dest="q99_filter",
        action="store_true",
        help="Remove labels above --q99-quantile before deriving the observed feature domain.",
    )
    q99_group.add_argument(
        "--no-q99-filter",
        "--disable-q99-filter",
        dest="q99_filter",
        action="store_false",
        help="Keep all valid rows when deriving the observed feature domain.",
    )
    p.set_defaults(q99_filter=False)
    p.add_argument(
        "--q99-quantile",
        "--q99_quantile",
        type=float,
        default=0.99,
        help="Upper-label quantile used only when --q99-filter is enabled.",
    )
    return p


def main():
    parser = build_argparser()
    args = parser.parse_args()

    if args.max_road_flow is not None and args.max_road_flow < 0:
        args.max_road_flow = None
    if args.clip_min is not None and isinstance(args.clip_min, float) and math.isnan(args.clip_min):
        args.clip_min = None

    args.csv_files = resolve_csv_inputs(
        args.csv,
        args.csv_glob,
        args.csv_list,
        expected_count=args.expected_csv_count,
    )

    build_cache_table(args)


if __name__ == "__main__":
    main()
