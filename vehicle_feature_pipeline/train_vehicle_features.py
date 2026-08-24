#!/usr/bin/env python3
"""Train the SUMO road-time model with vehicle attributes.

The base five-feature pipeline remains untouched. This trainer requires six
additional numeric columns emitted by ``traci_vehicle_features.py``.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import model_training_sumo_v1 as base  # noqa: E402


BASE_FEATURES = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
VEHICLE_FEATURES = [
    "vehicle_length",
    "vehicle_min_gap",
    "vehicle_max_speed",
    "vehicle_accel",
    "vehicle_decel",
    "vehicle_sigma",
]
FEATURE_COLS = BASE_FEATURES + VEHICLE_FEATURES
SOURCE_LABEL = "travel_time_label"
LABEL_COL = "travel_time_no_waiting"
SAMPLE_WEIGHT_COL = base.SAMPLE_WEIGHT_COL


def _load_one_frame(csv_path: str) -> pd.DataFrame:
    columns = list(pd.read_csv(csv_path, nrows=0).columns)
    required = FEATURE_COLS + [SOURCE_LABEL]
    missing = [column for column in required if column not in columns]
    if missing:
        raise ValueError(
            f"{csv_path}: missing vehicle-feature training columns: {missing}. "
            "Generate this CSV with traci_vehicle_features.py."
        )

    df = pd.read_csv(csv_path, usecols=required)
    rows_read = len(df)
    df = df.dropna(subset=required).copy()
    df["has_waiting"] = df["has_waiting"].astype(int).astype("category")
    df["turn_type"] = df["turn_type"].astype(int)
    for column in [c for c in FEATURE_COLS if c not in {"has_waiting", "turn_type"}]:
        df[column] = df[column].astype(float)
    df[LABEL_COL] = df[SOURCE_LABEL].astype(float)

    finite_columns = [c for c in FEATURE_COLS if c != "has_waiting"] + [LABEL_COL]
    finite = np.isfinite(df[finite_columns]).all(axis=1)
    valid = (
        finite
        & (df[LABEL_COL] > 0)
        & (df["road_length"] > 0)
        & (df["road_flow"] >= 0)
        & (df["lane_flow"] >= 0)
        & (df["vehicle_length"] > 0)
        & (df["vehicle_min_gap"] >= 0)
        & (df["vehicle_max_speed"] > 0)
        & (df["vehicle_accel"] > 0)
        & (df["vehicle_decel"] > 0)
        & (df["vehicle_sigma"] >= 0)
        & (df["vehicle_sigma"] <= 1)
    )
    dropped = int((~valid).sum())
    if dropped:
        print(f"[INFO] {csv_path}: dropped invalid rows={dropped}")
    df = df.loc[valid, FEATURE_COLS + [LABEL_COL]].reset_index(drop=True)
    if df.empty:
        raise ValueError(f"{csv_path}: no valid vehicle-feature rows remain")
    print(f"[INFO] {csv_path}: rows read={rows_read}, valid={len(df)}")
    return df


def prepare_vehicle_training_frame(
    csv_paths: Sequence[str],
    q99_filter: bool = False,
    q99_quantile: float = 0.99,
) -> pd.DataFrame:
    frames = [_load_one_frame(path) for path in csv_paths]
    df = pd.concat(frames, ignore_index=True, copy=False)
    df["has_waiting"] = df["has_waiting"].astype(int).astype("category")
    print(f"[INFO] merged valid rows before optional P99 filtering: {len(df)}")

    if q99_filter:
        threshold = float(df[LABEL_COL].quantile(q99_quantile))
        before = len(df)
        df = df[df[LABEL_COL] <= threshold].reset_index(drop=True)
        print(f"[INFO] P99 filter: ENABLED; threshold={threshold}")
        print(f"[INFO] P99 rows removed: {before - len(df)}")
    else:
        print("[INFO] P99 filter: DISABLED")

    if df.empty:
        raise ValueError("No training rows remain after preprocessing")
    print(f"[INFO] selected feature columns: {FEATURE_COLS}")
    profiles = df[VEHICLE_FEATURES].drop_duplicates().shape[0]
    print(f"[INFO] observed vehicle profiles: {profiles}")
    return df[FEATURE_COLS + [LABEL_COL]]


def train(args, csv_paths: Sequence[str]):
    df = prepare_vehicle_training_frame(
        csv_paths,
        q99_filter=args.q99_filter,
        q99_quantile=args.q99_quantile,
    )
    train_data, test_data = train_test_split(
        df,
        test_size=args.test_size,
        random_state=args.random_state,
    )
    print(f"[INFO] train/test rows: {len(train_data)} / {len(test_data)}")

    predictor_kwargs = {
        "label": LABEL_COL,
        "problem_type": "regression",
        "eval_metric": "mae",
        "path": args.save_path,
    }
    if args.tail_reweight:
        train_data, _threshold = base.add_tail_sample_weights(
            train_data,
            label=LABEL_COL,
            tail_quantile=args.tail_quantile,
            tail_weight_factor=args.tail_weight_factor,
        )
        predictor_kwargs["sample_weight"] = SAMPLE_WEIGHT_COL
        predictor_kwargs["weight_evaluation"] = False
    else:
        print("[INFO] tail reweighting: DISABLED")

    try:
        from autogluon.tabular import TabularPredictor
    except ImportError as exc:
        raise RuntimeError(
            "AutoGluon is required for training; use the existing training environment."
        ) from exc

    Path(args.save_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
    predictor = TabularPredictor(**predictor_kwargs).fit(
        train_data,
        presets="best_quality",
        num_bag_folds=10,
        num_stack_levels=2,
        excluded_model_types=["NN_TORCH"],
        time_limit=args.time_limit,
        ag_args_fit={"num_gpus": args.num_gpus},
    )
    print("feature metadata:\n", predictor.feature_metadata)
    print("test eval:\n", predictor.evaluate(test_data))
    print("feature importance:\n", predictor.feature_importance(test_data))
    return predictor


def build_argparser():
    parser = base._build_argparser()
    parser.description = "Train a SUMO road-time model with numeric vehicle features."
    parser.set_defaults(save_path="models_vehicle_features_v1")
    return parser


def main() -> None:
    args = build_argparser().parse_args()
    base._validate_cli_options(args)
    csv_paths = base.resolve_csv_inputs(
        csv_paths=args.csv,
        csv_globs=args.csv_glob,
        csv_lists=args.csv_list,
        expected_count=args.expected_csv_count,
    )
    print("[Vehicle Features] " + ",".join(VEHICLE_FEATURES))

    if args.dry_run:
        processed = prepare_vehicle_training_frame(
            csv_paths,
            q99_filter=args.q99_filter,
            q99_quantile=args.q99_quantile,
        )
        if args.tail_reweight:
            dry_train, _dry_test = train_test_split(
                processed,
                test_size=args.test_size,
                random_state=args.random_state,
            )
            base.add_tail_sample_weights(
                dry_train,
                label=LABEL_COL,
                tail_quantile=args.tail_quantile,
                tail_weight_factor=args.tail_weight_factor,
            )
        print("[DRY-RUN] vehicle-feature preprocessing validation passed")
        return

    train(args, csv_paths)


if __name__ == "__main__":
    main()
