#!/usr/bin/env python3
"""Train new cap/context models; legacy SF models and entrypoints stay unchanged.

New collection output and its .contract.json sidecar are required. Raw SF,
speed_limit and P/E diagnostics never enter fit or predict. Choose weighting
explicitly; six-threshold mode uses 0.1, 3, 1, 0.5, 1.5, 2 in that order.
"""
from __future__ import annotations

import argparse
import copy
import json
import random
import sys
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np
import pandas as pd

import training._shared.model_training_sumo_sf as legacy
from features.cap_context_features import FEATURE_COLS, FEATURE_SCHEMA, validate_context_contract

SCHEMA = FEATURE_SCHEMA
META_COLS = ["vehicleID", "route_index", "is_final_road"]
LABEL_COL = legacy.LABEL_COL
LABEL_COLUMNS = list(legacy.LABEL_COLUMNS)
THRESHOLD_ORDER = ["0p1", "3", "1", "0p5", "1p5", "2"]
CATEGORICAL_FEATURES = {"platoon_context": [0, 1, 2, 3]}
WEIGHT_COL = legacy.WEIGHT_COL

# These helpers do not depend on the legacy feature schema. Never monkeypatch
# legacy.FEATURE_COLS/SCHEMA: old entrypoints may be imported in the same process.
file_sha256 = legacy.file_sha256
resolve_splits = legacy.resolve_splits
reject_duplicate_contents = legacy.reject_duplicate_contents
make_sample_weights = legacy.make_sample_weights


def encode_feature_frame(frame):
    """Return ONLY raw5 with one fixed unordered categorical column.

    Collector metadata may coexist in the input frame, but is never selected.
    This is the same input encoding required when exporting the trained model.
    """
    missing = [column for column in FEATURE_COLS if column not in frame.columns]
    if missing:
        raise ValueError(f"Missing cap/context features {missing}; recollect new features, do not infer from old SF/flow")
    result = frame[FEATURE_COLS].copy()
    for column in FEATURE_COLS:
        if pd.api.types.is_bool_dtype(result[column].dtype):
            raise ValueError(f"{column} must be numeric, not boolean")
        result[column] = pd.to_numeric(result[column], errors="raise")
        values = result[column].to_numpy(dtype=float)
        if not np.isfinite(values).all():
            raise ValueError(f"{column} contains missing/non-finite values")
    for column in ("road_length", "free_speed_cap"):
        if (result[column] <= 0).any():
            raise ValueError(f"{column} must be strictly positive; no fallback is allowed")
    for column in ("road_flow", "lane_flow", "platoon_context"):
        values = result[column].to_numpy(dtype=float)
        if (values < 0).any() or (values != np.floor(values)).any():
            raise ValueError(f"{column} must be a nonnegative integer")
    if (result["lane_flow"] > result["road_flow"]).any():
        raise ValueError("lane_flow exceeds road_flow; audit occupancy definitions")
    if not result["platoon_context"].isin(CATEGORICAL_FEATURES["platoon_context"]).all():
        raise ValueError("platoon_context must be exactly one of 0, 1, 2, 3")
    # CSV lexical formatting must not change the fit/predict numeric dtypes.
    for column in ("road_length", "free_speed_cap"):
        result[column] = result[column].astype("float64")
    for column in ("road_flow", "lane_flow"):
        if (result[column].to_numpy(dtype=float) >= 2**63).any():
            raise ValueError(f"{column} exceeds the signed 64-bit integer domain")
        result[column] = result[column].astype("int64")
    result["platoon_context"] = pd.Categorical(
        result["platoon_context"].astype(int), categories=[0, 1, 2, 3], ordered=False)
    return result


def feature_summary(frame):
    summary = {}
    for column in FEATURE_COLS:
        values = frame[column].astype(float)
        summary[column] = {
            "min": float(values.min()), "max": float(values.max()),
            "unique": int(values.nunique()), "constant": bool(values.nunique() == 1),
        }
    summary["platoon_context"].update({
        "type": "categorical", "categories": [0, 1, 2, 3], "ordered": False,
        "observed_categories": sorted(int(value) for value in frame["platoon_context"].unique()),
        "counts": {str(value): int((frame["platoon_context"] == value).sum()) for value in range(4)},
    })
    return summary


def read_collection_contract(csv_path):
    csv_path = Path(csv_path)
    path = Path(str(csv_path) + ".contract.json")
    if not path.is_file():
        raise ValueError(f"Missing collection contract: {path}; use new cap/context collector output")
    try:
        contract = json.loads(path.read_text(encoding="utf-8-sig"))
    except (ValueError, OSError) as exc:
        raise ValueError(f"Invalid collection contract: {path}") from exc
    if not isinstance(contract, dict):
        raise ValueError(f"{path}: collection contract must be a JSON object")
    if contract.get("schema") != SCHEMA or contract.get("raw_features") != FEATURE_COLS:
        raise ValueError(f"{path}: collection schema/raw_features mismatch")
    context = validate_context_contract(contract.get("context_contract"))
    digest = file_sha256(csv_path)
    if contract.get("data_sha256") != digest:
        raise ValueError(f"{path}: collection data_sha256 does not match CSV; stale or unpaired metadata")
    return context, {
        "path": str(path.resolve()), "sha256": file_sha256(path),
        "data_sha256": digest, "source": contract.get("source"),
    }


def load_frame(csv_path, label_column):
    """Load verified new data, never reconstruct missing cap/context from old CSV."""
    if label_column not in LABEL_COLUMNS:
        raise ValueError(f"Unsupported road-time label: {label_column}")
    header = pd.read_csv(csv_path, nrows=0)
    required = FEATURE_COLS + META_COLS + [label_column]
    missing = [column for column in required if column not in header.columns]
    if missing:
        raise ValueError(f"{csv_path}: missing cap/context full-training columns {missing}; recollect with the new collector")
    context, provenance = read_collection_contract(csv_path)
    frame = pd.read_csv(csv_path, usecols=required, dtype={"vehicleID": str},
                        float_precision="round_trip")
    rows_read = len(frame)
    if not rows_read:
        raise ValueError(f"{csv_path}: empty training data")
    features = encode_feature_frame(frame)
    for column in (label_column, "route_index", "is_final_road"):
        if pd.api.types.is_bool_dtype(frame[column].dtype):
            raise ValueError(f"{csv_path}: {column} must be numeric, not boolean")
        frame[column] = pd.to_numeric(frame[column], errors="raise")
        if not np.isfinite(frame[column].to_numpy(dtype=float)).all():
            raise ValueError(f"{csv_path}: {column} contains missing/non-finite values")
    indices = frame["route_index"].to_numpy(dtype=float)
    if (indices < 0).any() or (indices != np.floor(indices)).any():
        raise ValueError(f"{csv_path}: route_index must be a nonnegative integer")
    if not frame["is_final_road"].isin([0, 1]).all():
        raise ValueError(f"{csv_path}: is_final_road must be 0 or 1")
    if frame["vehicleID"].isna().any() or frame["vehicleID"].str.strip().eq("").any():
        raise ValueError(f"{csv_path}: missing vehicleID")
    if frame.duplicated(["vehicleID", "route_index"]).any():
        raise ValueError(f"{csv_path}: duplicate vehicleID x route_index event")
    valid = frame[label_column] > 0
    if not valid.any():
        raise ValueError(f"{csv_path}: no positive {label_column} labels remain")
    audit = {
        "path": str(Path(csv_path).resolve()), "sha256": provenance["data_sha256"],
        "collection_contract_file": provenance, "context_contract": context,
        "rows_read": rows_read, "nonpositive_label_rows_removed": int((~valid).sum()),
        "rows_retained": int(valid.sum()), "vehicles_observed": int(frame["vehicleID"].nunique()),
        "final_rows_observed": int(frame["is_final_road"].sum()),
        "final_rows_retained": int(frame.loc[valid, "is_final_road"].sum()),
    }
    result = features.loc[valid].copy()
    result[LABEL_COL] = frame.loc[valid, label_column]
    for column in META_COLS:
        result[column] = frame.loc[valid, column]
    result["__source_file"] = audit["path"]
    result = result.reset_index(drop=True)
    audit["features"] = feature_summary(result)
    result.attrs.update(audit=audit, context_contract=context)
    print(f"[INPUT] {audit['path']} rows={rows_read} retained={len(result)} "
          f"final={audit['final_rows_retained']} dropped_label={audit['nonpositive_label_rows_removed']}")
    return result


def load_split(paths, label_column):
    frames, audits, shared = [], [], None
    for path in paths:
        frame = load_frame(path, label_column)
        context = frame.attrs["context_contract"]
        if shared is not None and context != shared:
            raise ValueError("Mixed collection context_contract in one split; recollect with one frozen contract")
        shared = context
        audits.append(frame.attrs["audit"])
        frame.attrs = {}
        frames.append(frame)
    if not frames:
        raise ValueError("Empty training/validation split")
    result = pd.concat(frames, ignore_index=True)
    result.attrs["context_contract"] = shared
    return result, audits


def shared_context_contract(train_frame, validation_frame):
    train = validate_context_contract(train_frame.attrs.get("context_contract"))
    validation = validate_context_contract(validation_frame.attrs.get("context_contract"))
    if train != validation:
        raise ValueError("Training/validation context_contract mismatch; time/geometry/threshold versions must agree")
    return train


def validation_metrics(frame, prediction, low_flow_cutoff=5):
    result = legacy.validation_metrics(frame, prediction, low_flow_cutoff)
    prediction = np.asarray(prediction, dtype=float)
    result["by_platoon_context"] = {}
    for category in range(4):
        mask = (frame["platoon_context"] == category).to_numpy()
        result["by_platoon_context"][str(category)] = (
            legacy.metric_summary(frame.loc[mask, LABEL_COL], prediction[mask]) if mask.any() else {"n": 0})
    return result


def fit_model(train_frame, validation_frame, args, provenance):
    output = Path(args.save_path).expanduser().resolve()
    if output.exists():
        raise ValueError(f"Output already exists; choose a new --save-path: {output}")
    validate_options(args)
    context = shared_context_contract(train_frame, validation_frame)
    train_features = encode_feature_frame(train_frame)
    validation_features = encode_feature_frame(validation_frame)
    weights, weight_audit = make_sample_weights(train_frame, args)
    from autogluon.tabular import TabularPredictor

    dependencies = {}
    for package in ("numpy", "pandas", "scikit-learn", "autogluon.tabular"):
        try:
            dependencies[package] = version(package)
        except PackageNotFoundError:
            dependencies[package] = None
    excluded = list(dict.fromkeys(args.excluded_model_types))
    if weights is not None:
        if dependencies["autogluon.tabular"] != legacy.WEIGHT_AUDITED_AUTOGLUON_VERSION:
            raise ValueError(f"Weighted training is audited for autogluon.tabular=={legacy.WEIGHT_AUDITED_AUTOGLUON_VERSION}; "
                             f"found {dependencies['autogluon.tabular']!r}")
        excluded = list(dict.fromkeys(excluded + ["NN_TORCH", "FASTAI"]))
        weight_audit["learner_support"] = {
            "audited_autogluon_tabular": legacy.WEIGHT_AUDITED_AUTOGLUON_VERSION,
            "excluded_ignoring_weights": ["NN_TORCH", "FASTAI"],
            "caveat": "XGB fits with sample_weight but native early-stopping metric does not use validation weights",
        }
    contract = dict(provenance)
    contract.update({
        "status": "fit_started", "schema": SCHEMA, "raw_features": list(FEATURE_COLS),
        "source_label": args.label_column, "model_label": LABEL_COL,
        "context_contract": context, "categorical_features": CATEGORICAL_FEATURES,
        "categorical_ordered": False, "trainer_sha256": file_sha256(__file__),
        "python_version": sys.version, "dependency_versions": dependencies,
        "units": {"road_length": "m", "free_speed_cap": "m/s", "platoon_context": "categorical"},
        "free_speed_cap_semantics": "min(vType.maxSpeed, SF * desiredMaxSpeed, SF * current external-road speed_limit)",
        "road_length_semantics": "physical full road length; F13 free-clock geometry separately frozen in context_contract",
        "training_features": feature_summary(train_frame),
        "validation_features": feature_summary(validation_frame),
        "constant_training_features": [column for column in FEATURE_COLS if train_frame[column].nunique() == 1],
        "validation_role": "evaluation only; not supplied to fit or model selection",
        "internal_cv": "AutoGluon folds within training files; not certified episode-grouped",
        "p99_filter": False, "tail_reweight": False, "sample_weighting": weight_audit,
        "effective_excluded_model_types": excluded, "training_options": vars(args),
        "predictor_directory": "predictor",
    })
    output.mkdir(parents=True)
    contract_path = output / "training_contract.json"

    def save_contract():
        contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")

    save_contract()
    try:
        random.seed(args.random_state)
        np.random.seed(args.random_state)
        fit_frame = train_features.copy()
        fit_frame[LABEL_COL] = train_frame[LABEL_COL]
        predictor_options = {}
        if weights is not None:
            fit_frame[WEIGHT_COL] = weights
            predictor_options.update(sample_weight=WEIGHT_COL, weight_evaluation=True)
        predictor = TabularPredictor(label=LABEL_COL, problem_type="regression", eval_metric="mae",
                                     path=str(output / "predictor"), **predictor_options).fit(
            train_data=fit_frame, presets="best_quality", num_bag_folds=args.bag_folds,
            num_stack_levels=args.stack_levels, excluded_model_types=excluded,
            time_limit=args.time_limit, ag_args_fit={"num_gpus": args.num_gpus})
        metadata = getattr(predictor, "feature_metadata", None)
        if metadata is None:
            raise ValueError("Missing completed-fit feature metadata")
        used = list(metadata.get_features())
        if not used or len(set(used)) != len(used) or not set(used).issubset(FEATURE_COLS):
            raise ValueError(f"Unexpected model inputs (metadata/weight leakage): {used}")
        contract["framework_features"] = used
        contract["raw_schema_preserved_even_if_framework_prunes_constants"] = True
        prediction = predictor.predict(validation_features)
        metrics = validation_metrics(validation_frame, prediction, args.low_flow_cutoff)
        (output / "validation_metrics.json").write_text(
            json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        contract["status"] = "complete"
        save_contract()
    except Exception as exc:
        contract.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        save_contract()
        raise
    print("[VALIDATION] Road-label metrics, unweighted; not closed-loop trip metrics:")
    print(json.dumps(metrics["overall"], ensure_ascii=False))
    print(f"[SAVED] {output / 'predictor'}")
    print("[NOTICE] Export with the new cap/context catcher; do not feed this model to the legacy SF catcher.")
    return predictor


def build_parser():
    parser = legacy.build_parser()
    parser.description = __doc__
    # Required explicit choice avoids silently changing the chosen old training
    # population: none=equal, low-flow defaults to strict road_flow<5, raw 3:1.
    for action in parser._actions:
        if action.dest == "sample_weighting":
            action.required = True
            action.default = None
        elif action.dest == "label_column":
            action.default = None
            action.choices = LABEL_COLUMNS
    parser.add_argument("--all-thresholds", action="store_true",
                        help="Train sequential gt_0p1, gt_3, gt_1, gt_0p5, gt_1p5, gt_2 under the new save-path root")
    return parser


def validate_options(args):
    legacy.validate_options(args)
    if args.sample_weighting not in ("none", "low-flow"):
        raise ValueError("Select --sample-weighting none (equal) or low-flow explicitly")
    if args.all_thresholds and args.label_column is not None:
        raise ValueError("Do not combine --all-thresholds with --label-column")


def main(argv=None):
    args = build_parser().parse_args(argv)
    validate_options(args)
    root = Path(args.save_path).expanduser().resolve()
    if not args.dry_run and root.exists():
        raise ValueError(f"Output already exists; choose a new --save-path: {root}")
    train_paths, validation_paths, split_kind = resolve_splits(args)
    labels = [("gt_" + threshold, "travel_time_label_gt_" + threshold) for threshold in THRESHOLD_ORDER] if args.all_thresholds else [
        (None, args.label_column or LABEL_COLUMNS[0])]
    if args.all_thresholds:
        # Fail on a missing later threshold before spending hours fitting an
        # earlier one. Selected-label validity is still checked on each run.
        for path in train_paths + validation_paths:
            columns = pd.read_csv(path, nrows=0).columns
            missing = [label for _, label in labels if label not in columns]
            if missing:
                raise ValueError(f"{path}: six-threshold input missing labels {missing}")
    results = []
    for subdirectory, label in labels:
        run = copy.copy(args)
        run.all_thresholds = False
        run.label_column = label
        run.save_path = str(root / subdirectory) if subdirectory else str(root)
        print(f"[CONFIG] schema={SCHEMA} label={label} raw5={FEATURE_COLS} weighting={run.sample_weighting}")
        print(f"[CONFIG] split={split_kind} train_files={len(train_paths)} validation_files={len(validation_paths)}")
        train_frame, train_audits = load_split(train_paths, label)
        validation_frame, validation_audits = load_split(validation_paths, label)
        context = shared_context_contract(train_frame, validation_frame)
        reject_duplicate_contents(train_audits, validation_audits)
        _, weight_audit = make_sample_weights(train_frame, run)
        print("[WEIGHTS] " + json.dumps(weight_audit, ensure_ascii=False))
        observed = set(int(value) for value in train_frame["platoon_context"].unique())
        missing = sorted(set(range(4)) - observed)
        if missing:
            print(f"[WARN] training lacks platoon_context categories {missing}; retained as declared categories, no coverage claim")
        for column in FEATURE_COLS[:-1]:
            lo, hi = train_frame[column].min(), train_frame[column].max()
            outside = int(((validation_frame[column] < lo) | (validation_frame[column] > hi)).sum())
            if outside:
                print(f"[WARN] validation {column}: {outside} rows outside training min/max")
        unseen = sorted(set(int(value) for value in validation_frame["platoon_context"].unique()) - observed)
        if unseen:
            print(f"[WARN] validation platoon_context categories unseen in training: {unseen}")
        provenance = {
            "split_kind": split_kind, "train_files": train_audits, "validation_files": validation_audits,
            "entrypoint": {"path": str(Path(__file__).resolve()), "sha256": file_sha256(__file__)},
        }
        if args.dataset_dir:
            manifest = Path(args.dataset_dir) / "commands" / "traci_jobs.tsv"
            provenance["manifest"] = {"path": str(manifest.resolve()), "sha256": file_sha256(manifest)}
        result = {"source_label": label, "save_path": run.save_path, "train_rows": len(train_frame),
                  "validation_rows": len(validation_frame), "context_contract": context,
                  "sample_weighting": weight_audit, "dry_run": bool(args.dry_run)}
        if args.dry_run:
            print(f"[DRY-RUN] Passed {label}; no AutoGluon import, training or output-file writes")
        else:
            fit_model(train_frame, validation_frame, run, provenance)
        results.append(result)
    if args.all_thresholds and not args.dry_run:
        (root / "six_threshold_training.json").write_text(
            json.dumps({"status": "complete", "schema": SCHEMA, "runs": results},
                       ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return results


if __name__ == "__main__":
    main()
