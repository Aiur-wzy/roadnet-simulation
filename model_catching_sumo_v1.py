#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Build a fast lookup table for the SUMO/TraCI travel-time model trained by
model_training_sumo_v1.py.

The current model_training_sumo_v1.py trains with exactly these input features:
    has_waiting, road_length, turn_type, road_flow, lane_flow
and predicts:
    travel_time_no_waiting

This script enumerates feature combinations, calls the trained AutoGluon model
in batches, and writes a text table that can be loaded by the macroscopic
simulator instead of calling the model repeatedly.

Python version:
- Compatible with Python 3.6+; avoids PEP 585 built-in generic type annotations.

Default usage:
    python model_catching_sumo_v1.py \
        --csv TraCI_output_adjusted.csv \
        --model_path models_v1 \
        --output_txt model_catching_sumo_v1.txt \
        --max_road_flow 200

Important:
- turn_type is encoded with sklearn LabelEncoder, exactly like the training
  script. A mapping file is exported so the simulator can use the same integer
  codes.
- lane_flow is computed as road_flow / Lanes_Net in the training script. The
  default catching mode therefore derives lane_flow from unique lane counts in
  the training CSV. Use --lane_flow_mode independent only if your simulator
  passes lane_flow as an independently discretized lookup dimension.
"""

import argparse
import json
import math
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.preprocessing import LabelEncoder
from autogluon.tabular import TabularPredictor

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


def load_training_like_frame(csv_path):
    """
    Rebuild the same base feature frame used by model_training_sumo_v1.py.

    This intentionally mirrors the training logic:
      road_length = E_Length
      road_flow = Driving_Num
      lane_flow = road_flow / Lanes_Net
      turn_type = LabelEncoder(Turn)
      has_waiting = Wait_Time > 0
      label = Travel_Time + Delay_Time + LowSpee_Time
      q99 outlier removal on label
    """
    raw = pd.read_csv(csv_path)

    must = [
        "Edge_ID",
        "Time",
        "E_Length",
        "Turn",
        "Driving_Num",
        "Wait_Time",
        "Travel_Time",
        "Delay_Time",
        "LowSpee_Time",
        "Lanes_Net",
    ]
    missing = [c for c in must if c not in raw.columns]
    if missing:
        raise ValueError(f"Missing required columns in {csv_path}: {missing}")

    df = raw.dropna(subset=["Edge_ID", "Time", "E_Length", "Turn", "Driving_Num", "Lanes_Net"]).copy()

    df["road_length"] = df["E_Length"].astype(float)
    df["road_flow"] = df["Driving_Num"].astype(float)
    df["lane_count"] = df["Lanes_Net"].astype(float)
    df["lane_flow"] = df["road_flow"] / (df["lane_count"] + EPS)

    le = LabelEncoder()
    df["turn_type"] = le.fit_transform(df["Turn"].astype(str))
    turn_mapping = {str(cls): int(code) for code, cls in enumerate(le.classes_)}

    df["Wait_Time"] = df["Wait_Time"].fillna(0).astype(float)
    df["Travel_Time"] = df["Travel_Time"].fillna(0).astype(float)
    df["Delay_Time"] = df["Delay_Time"].fillna(0).astype(float)
    df["LowSpee_Time"] = df["LowSpee_Time"].fillna(0).astype(float)

    df["has_waiting"] = (df["Wait_Time"] > 0).astype("category")
    df[LABEL_COL] = df["Travel_Time"] + df["Delay_Time"] + df["LowSpee_Time"]

    if (df[LABEL_COL] < 0).any():
        raise ValueError(f"{LABEL_COL} has negative values; please check the input CSV.")

    q99 = df[LABEL_COL].quantile(0.99)
    df = df[df[LABEL_COL] <= q99].reset_index(drop=True)

    return df, turn_mapping


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
    lane_counts,
    lane_flow_values,
    lane_flow_mode,
    precision,
):
    has_waiting_values = [False, True]

    for road_length in road_lengths:
        for turn_type in turn_types:
            for has_waiting in has_waiting_values:
                for road_flow in road_flow_values:
                    if lane_flow_mode == "derived":
                        current_lane_flows = sorted(
                            {
                                round(float(road_flow) / (float(lc) + EPS), precision)
                                for lc in lane_counts
                                if float(lc) > 0
                            }
                        )
                    else:
                        current_lane_flows = lane_flow_values

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


def build_cache_table(args):
    df, turn_mapping = load_training_like_frame(args.csv)

    save_unique_feature_combinations(df, args.unique_csv)
    save_turn_mapping(turn_mapping, args.turn_mapping_json)

    road_lengths = sorted(float(x) for x in df["road_length"].dropna().unique())
    turn_types = sorted(int(x) for x in df["turn_type"].dropna().unique())
    lane_counts = sorted(float(x) for x in df["lane_count"].dropna().unique() if float(x) > 0)

    if args.max_road_flow is None:
        max_road_flow = int(math.ceil(float(df["road_flow"].max())))
    else:
        max_road_flow = args.max_road_flow
    road_flow_values = list(range(0, max_road_flow + 1, args.road_flow_step))

    if args.lane_flow_mode == "independent":
        max_lane_flow = args.max_lane_flow
        if max_lane_flow is None:
            max_lane_flow = float(max_road_flow)
        lane_flow_values = float_range(0.0, float(max_lane_flow), args.lane_flow_step, args.precision)
    else:
        lane_flow_values = []

    print("[INFO] table dimensions:")
    print(f"       road_length values = {len(road_lengths)}")
    print(f"       turn_type values   = {turn_types}")
    print(f"       has_waiting values = [false, true]")
    print(f"       road_flow values   = 0..{max_road_flow} step {args.road_flow_step}")
    if args.lane_flow_mode == "derived":
        print(f"       lane_flow mode     = derived from lane counts {lane_counts}")
    else:
        print(f"       lane_flow mode     = independent, 0..{args.max_lane_flow or max_road_flow} step {args.lane_flow_step}")

    predictor = TabularPredictor.load(args.model_path)
    predictor_features = get_predictor_features(predictor)
    validate_model_features(predictor_features)
    predictor_features = [c for c in FEATURE_COLS if c in predictor_features]
    output_columns = predictor_features + [LABEL_COL]

    txt_path = Path(args.output_txt)
    csv_path = Path(args.output_csv) if args.output_csv else None

    write_txt_header(txt_path, output_columns)
    if csv_path is not None and csv_path.exists():
        csv_path.unlink()

    case_iterator = iter_cases(
        road_lengths=road_lengths,
        turn_types=turn_types,
        road_flow_values=road_flow_values,
        lane_counts=lane_counts,
        lane_flow_values=lane_flow_values,
        lane_flow_mode=args.lane_flow_mode,
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

        out_df = cases_df[predictor_features].copy()
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
    p = argparse.ArgumentParser(description="Build SUMO v1 AutoGluon travel-time lookup table.")
    p.add_argument("--csv", default="TraCI_output_adjusted.csv", help="Training CSV used by model_training_sumo_v1.py")
    p.add_argument("--model_path", default="models_v1", help="AutoGluon model directory saved by model_training_sumo_v1.py")
    p.add_argument("--output_txt", default="model_catching_sumo_v1.txt", help="Space-separated lookup table output")
    p.add_argument("--output_csv", default="", help="Optional CSV output path; leave empty to disable")
    p.add_argument("--unique_csv", default="unique_feature_combinations_sumo_v1.csv", help="Unique observed feature combinations output")
    p.add_argument("--turn_mapping_json", default="turn_type_mapping_sumo_v1.json", help="Raw Turn -> encoded turn_type mapping output")

    p.add_argument("--max_road_flow", type=int, default=200, help="Maximum road_flow to enumerate; use -1 to infer from CSV max")
    p.add_argument("--road_flow_step", type=positive_int, default=1, help="road_flow enumeration step")

    p.add_argument(
        "--lane_flow_mode",
        choices=["derived", "independent"],
        default="derived",
        help="derived: lane_flow=road_flow/Lanes_Net; independent: enumerate lane_flow directly",
    )
    p.add_argument("--max_lane_flow", type=float, default=None, help="Maximum lane_flow for independent mode; default=max_road_flow")
    p.add_argument("--lane_flow_step", type=positive_float, default=1.0, help="lane_flow step for independent mode")

    p.add_argument("--precision", type=int, default=6, help="Decimal precision for derived lane_flow values")
    p.add_argument("--chunk_size", type=positive_int, default=200000, help="Prediction batch size")
    p.add_argument("--clip_min", type=float, default=0.0, help="Clip predicted travel_time_no_waiting lower bound; set to nan to disable")
    return p


def main():
    parser = build_argparser()
    args = parser.parse_args()

    if args.max_road_flow is not None and args.max_road_flow < 0:
        args.max_road_flow = None
    if args.clip_min is not None and isinstance(args.clip_min, float) and math.isnan(args.clip_min):
        args.clip_min = None

    build_cache_table(args)


if __name__ == "__main__":
    main()
