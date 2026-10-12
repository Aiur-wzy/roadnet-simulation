#!/usr/bin/env python3
"""Train the opt-in core3 + SF + road-speed-limit model from full TraCI output.

The legacy v2 trainer and all C++/catch interfaces remain unchanged.
External validation files are evaluation-only; otherwise split whole files,
never individual road rows. Constant inputs stay in the declared raw schema;
AutoGluon's internal feature pruning is recorded separately.
"""
from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import os
import random
import re
import sys
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split

FEATURE_COLS = ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"]
META_COLS = ["vehicleID", "route_index", "is_final_road"]
LABEL_COL = "travel_time_no_waiting"
LABEL_COLUMNS = ["travel_time_label_gt_" + t for t in ("0p1", "0p5", "1", "1p5", "2", "3")]
SCHEMA = "road-time-core3-sf-speed-limit-v1"
WEIGHT_COL = "__sample_weight"
WEIGHT_AUDITED_AUTOGLUON_VERSION = "1.6.1"


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_paths(paths):
    resolved = [str(Path(p).resolve(strict=True)) for p in paths]
    if len({os.path.normcase(p) for p in resolved}) != len(resolved):
        raise ValueError("Duplicate/aliased CSV input paths")
    for path in resolved:
        if not Path(path).is_file() or Path(path).stat().st_size == 0:
            raise ValueError(f"Missing or empty CSV: {path}")
    return resolved


def expand_inputs(paths, patterns, lists):
    """Expand without silently deduplicating: repeated rows/files are errors."""
    def absolute(value, parent=None):
        path = Path(os.path.expandvars(os.path.expanduser(str(value))))
        return str(path if path.is_absolute() else (parent or Path.cwd()) / path)

    candidates = [absolute(path) for path in paths]
    for pattern in patterns:
        matched = sorted(path for path in glob.glob(absolute(pattern), recursive=True)
                         if Path(path).is_file())
        if not matched:
            raise ValueError(f"CSV glob matched no files: {pattern}")
        candidates.extend(matched)
    for listing in lists:
        listing = Path(absolute(listing))
        for line in listing.read_text(encoding="utf-8-sig").splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                candidates.append(absolute(line, listing.parent))
    return canonical_paths(candidates)


def manifest_splits(dataset_dir, run_dir):
    manifest = Path(dataset_dir) / "commands" / "traci_jobs.tsv"
    root = (Path(run_dir) / "training").resolve()
    groups = {"train": [], "validation": []}
    episodes = set()
    with manifest.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        required = {"episode_id", "split", "full_training_output"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"Manifest missing columns: {sorted(required)}")
        for row in reader:
            episode = (row["episode_id"] or "").strip()
            split = (row["split"] or "").strip()
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", episode):
                raise ValueError(f"Invalid episode_id: {episode!r}")
            if episode in episodes:
                raise ValueError(f"Duplicate episode_id: {episode}")
            episodes.add(episode)
            if split not in groups:
                raise ValueError(f"Unknown manifest split: {split!r}")
            basename = (row["full_training_output"] or "").replace("\\", "/").rsplit("/", 1)[-1]
            if basename != f"cams_model_training_data_{episode}.csv":
                raise ValueError(f"Unexpected full CSV basename for {episode}: {basename}")
            path = (root / basename).resolve(strict=True)
            if root not in path.parents:
                raise ValueError(f"Full CSV resolves outside training directory: {path}")
            groups[split].append(str(path))
    return groups["train"], groups["validation"]


def resolve_splits(args):
    train_specs = bool(args.csv or args.csv_glob or args.csv_list)
    val_specs = bool(args.validation_csv or args.validation_csv_glob or args.validation_csv_list)
    if args.dataset_dir or args.run_dir:
        if not (args.dataset_dir and args.run_dir):
            raise ValueError("--dataset-dir and --run-dir must be provided together")
        if train_specs or val_specs:
            raise ValueError("Do not mix manifest mode and explicit CSV inputs")
        train_paths, val_paths = manifest_splits(args.dataset_dir, args.run_dir)
        split_kind = "manifest_external_validation"
    else:
        if not train_specs:
            raise ValueError("Provide a dataset/run directory pair or explicit training CSV inputs")
        train_paths = expand_inputs(args.csv, args.csv_glob, args.csv_list)
        if val_specs:
            val_paths = expand_inputs(
                args.validation_csv, args.validation_csv_glob, args.validation_csv_list)
            split_kind = "explicit_external_validation"
        else:
            train_paths = canonical_paths(train_paths)
            if len(train_paths) < 2:
                raise ValueError("Single-file input needs an external validation CSV; row splitting is disabled")
            train_paths, val_paths = train_test_split(
                sorted(train_paths), test_size=args.validation_fraction, random_state=args.random_state)
            split_kind = "whole_file_holdout"
    train_paths, val_paths = canonical_paths(train_paths), canonical_paths(val_paths)
    if not train_paths or not val_paths:
        raise ValueError("Both training and validation must contain files")
    if {os.path.normcase(p) for p in train_paths} & {os.path.normcase(p) for p in val_paths}:
        raise ValueError("Training/validation CSV overlap")
    for name, paths, expected in (
        ("train", train_paths, args.expected_train_files),
        ("validation", val_paths, args.expected_validation_files),
    ):
        if expected and len(paths) != expected:
            raise ValueError(f"Expected {expected} {name} files, found {len(paths)}")
    return sorted(train_paths), sorted(val_paths), split_kind


def load_frame(csv_path, label_column):
    """Return five raw features, label and audit-only metadata; reject bad inputs."""
    header = pd.read_csv(csv_path, nrows=0)
    required = FEATURE_COLS + META_COLS + [label_column]
    missing = [c for c in required if c not in header.columns]
    if missing:
        raise ValueError(f"{csv_path}: missing full-training columns {missing}; use full output, not slim")
    df = pd.read_csv(csv_path, usecols=required, dtype={"vehicleID": str},
                     float_precision="round_trip")
    rows_read = len(df)
    if not rows_read:
        raise ValueError(f"{csv_path}: empty training data")
    for col in FEATURE_COLS + [label_column, "route_index", "is_final_road"]:
        df[col] = pd.to_numeric(df[col], errors="raise")
        if not np.isfinite(df[col].to_numpy(dtype=float)).all():
            raise ValueError(f"{csv_path}: {col} contains missing/non-finite values")
    for col in ("road_length", "speed_factor", "speed_limit"):
        if (df[col] <= 0).any():
            raise ValueError(f"{csv_path}: {col} must be strictly positive; no fallback is allowed")
    for col in ("road_flow", "lane_flow", "route_index"):
        values = df[col].to_numpy(dtype=float)
        if (values < 0).any() or (values != np.floor(values)).any():
            raise ValueError(f"{csv_path}: {col} must be a nonnegative integer")
    if (df["lane_flow"] > df["road_flow"]).any():
        raise ValueError(f"{csv_path}: lane_flow exceeds road_flow; audit occupancy definitions")
    if not df["is_final_road"].isin([0, 1]).all():
        raise ValueError(f"{csv_path}: is_final_road must be 0 or 1")
    if df["vehicleID"].isna().any() or df["vehicleID"].str.strip().eq("").any():
        raise ValueError(f"{csv_path}: missing vehicleID")
    if df.duplicated(["vehicleID", "route_index"]).any():
        raise ValueError(f"{csv_path}: duplicate vehicleID x route_index event")
    sf_counts = df.groupby("vehicleID", sort=False)["speed_factor"].nunique()
    if (sf_counts > 1).any():
        raise ValueError(f"{csv_path}: speed_factor changes within a vehicle; static-SF contract violated")
    valid = df[label_column] > 0
    audit = {
        "path": str(Path(csv_path).resolve()), "sha256": file_sha256(csv_path),
        "rows_read": rows_read, "nonpositive_label_rows_removed": int((~valid).sum()),
        "rows_retained": int(valid.sum()), "vehicles_observed": int(df["vehicleID"].nunique()),
        "final_rows_observed": int(df["is_final_road"].sum()),
        "final_rows_retained": int(df.loc[valid, "is_final_road"].sum()),
    }
    if not valid.any():
        raise ValueError(f"{csv_path}: no positive {label_column} labels remain")
    df = df.loc[valid].copy()
    df[LABEL_COL] = df[label_column]
    df["__source_file"] = audit["path"]
    df = df[FEATURE_COLS + [LABEL_COL] + META_COLS + ["__source_file"]].reset_index(drop=True)
    audit["features"] = feature_summary(df)
    df.attrs["audit"] = audit
    print(f"[INPUT] {audit['path']} rows={rows_read} retained={len(df)} "
          f"final={audit['final_rows_retained']} dropped_label={audit['nonpositive_label_rows_removed']}")
    return df


def feature_summary(frame):
    return {c: {"min": float(frame[c].min()), "max": float(frame[c].max()),
                "unique": int(frame[c].nunique()), "constant": bool(frame[c].nunique() == 1)}
            for c in FEATURE_COLS}


def load_split(paths, label_column):
    frames, audits = [], []
    for path in paths:
        frame = load_frame(path, label_column)
        audits.append(frame.attrs["audit"])
        frame.attrs = {}
        frames.append(frame)
    return pd.concat(frames, ignore_index=True), audits


def reject_duplicate_contents(train_audits, val_audits):
    seen = {}
    for split, audits in (("train", train_audits), ("validation", val_audits)):
        for audit in audits:
            key = audit["sha256"]
            if key in seen:
                raise ValueError(f"Duplicate CSV content: {seen[key]} and {split}:{audit['path']}")
            seen[key] = f"{split}:{audit['path']}"


def metric_summary(truth, prediction):
    residual = np.asarray(prediction, dtype=float) - np.asarray(truth, dtype=float)
    if not len(residual) or not np.isfinite(residual).all():
        raise ValueError("Empty or non-finite validation predictions")
    return {"n": len(residual), "mae": float(np.mean(np.abs(residual))),
            "bias": float(np.mean(residual)), "rmse": float(np.sqrt(np.mean(residual**2))),
            "p95_absolute_error": float(np.quantile(np.abs(residual), 0.95))}


def validation_metrics(frame, prediction, low_flow_cutoff=5):
    prediction = np.asarray(prediction, dtype=float)
    if prediction.shape != (len(frame),):
        raise ValueError("Prediction shape does not match validation rows")
    result = {"overall": metric_summary(frame[LABEL_COL], prediction)}
    for column, name in (("__source_file", "by_file"), ("is_final_road", "by_final_road")):
        result[name] = {}
        for key, indices in frame.groupby(column, sort=True).indices.items():
            result[name][str(key)] = metric_summary(frame.iloc[indices][LABEL_COL], prediction[indices])
    # External metrics always use ordinary row counts, never training weights.
    flow = frame["road_flow"].to_numpy()
    groups = {
        f"lt_{low_flow_cutoff}": flow < low_flow_cutoff,
        f"ge_{low_flow_cutoff}": flow >= low_flow_cutoff,
    }
    result["by_low_flow"] = {
        name: metric_summary(frame.loc[mask, LABEL_COL], prediction[mask]) if mask.any() else {"n": 0}
        for name, mask in groups.items()
    }
    groups = {"lt_5": flow < 5, "5_to_10": (flow >= 5) & (flow <= 10),
              "11_to_20": (flow > 10) & (flow <= 20), "gt_20": flow > 20}
    result["by_road_flow"] = {
        name: metric_summary(frame.loc[mask, LABEL_COL], prediction[mask]) if mask.any() else {"n": 0}
        for name, mask in groups.items()
    }
    result["metric_weighting"] = "unweighted road rows; bias = prediction - label"
    return result


def make_sample_weights(frame, args):
    """Use retained TRAIN inputs only; weight metadata is never a sixth feature."""
    if args.sample_weighting == "none":
        return None, {"mode": "none", "weight_evaluation": False}
    if args.sample_weighting != "low-flow":
        raise ValueError(f"Unknown sample weighting: {args.sample_weighting}")
    if not len(frame):
        raise ValueError("Cannot compute sample weights for an empty training frame")
    flow = frame["road_flow"].to_numpy(dtype=float)
    if not np.isfinite(flow).all() or (flow < 0).any() or (flow != np.floor(flow)).any():
        raise ValueError("Sample weighting requires finite nonnegative integer road_flow")
    selected = flow < args.low_flow_cutoff
    raw = np.where(selected, args.low_flow_weight, 1.0)
    mean = float(raw.mean())
    weights = pd.Series(raw / mean, index=frame.index, name=WEIGHT_COL)
    audit = {
        "mode": "low-flow", "column": WEIGHT_COL, "input": "road_flow", "operator": "<",
        "cutoff": args.low_flow_cutoff, "low_flow_raw_weight": args.low_flow_weight,
        "other_raw_weight": 1.0, "normalization": "divide by retained training raw-weight mean",
        "normalization_source": "training split only", "raw_weight_mean": mean,
        "rows": len(frame), "low_flow_rows": int(selected.sum()),
        "other_rows": int((~selected).sum()), "low_flow_row_fraction": float(selected.mean()),
        "low_flow_weight_fraction": float(raw[selected].sum() / raw.sum()),
        "normalized_low_flow_weight": args.low_flow_weight / mean,
        "normalized_other_weight": 1.0 / mean, "normalized_weight_sum": float(weights.sum()),
        "weight_evaluation": True, "external_metrics": "unweighted",
        "adds_predictive_feature": False, "resampling": False,
    }
    return weights, audit


def fit_model(train_frame, validation_frame, args, provenance):
    """Fit only training files; held-out files never enter fit/tuning_data."""
    output = Path(args.save_path).expanduser().resolve()
    if output.exists():
        raise ValueError(f"Output already exists; choose a new --save-path: {output}")
    validate_options(args)
    weights, weight_audit = make_sample_weights(train_frame, args)
    from autogluon.tabular import TabularPredictor
    contract = dict(provenance)
    dependency_versions = {}
    for package in ("numpy", "pandas", "scikit-learn", "autogluon.tabular"):
        try:
            dependency_versions[package] = version(package)
        except PackageNotFoundError:
            dependency_versions[package] = None
    excluded_models = list(dict.fromkeys(args.excluded_model_types))
    if weights is not None:
        # The 1.6.1 zeroshot preset includes FASTAI as well as NN_TORCH;
        # both ignore training sample weights. No public support tag exists.
        for name in ("NN_TORCH", "FASTAI"):
            if name not in excluded_models:
                excluded_models.append(name)
        installed = dependency_versions["autogluon.tabular"]
        if installed != WEIGHT_AUDITED_AUTOGLUON_VERSION:
            raise ValueError(
                f"Weighted training is audited for autogluon.tabular=={WEIGHT_AUDITED_AUTOGLUON_VERSION}; "
                f"found {installed!r}. Use the matching server environment or audit its learner weight support first."
            )
        weight_audit["learner_support"] = {
            "audited_autogluon_tabular": WEIGHT_AUDITED_AUTOGLUON_VERSION,
            "excluded_ignoring_weights": ["NN_TORCH", "FASTAI"],
            "caveat": "XGB fits with sample_weight but its native early-stopping metric does not use validation weights",
        }
    output.mkdir(parents=True)
    contract.update({
        "schema": SCHEMA, "raw_features": FEATURE_COLS, "source_label": args.label_column,
        "trainer_sha256": file_sha256(__file__), "python_version": sys.version,
        "dependency_versions": dependency_versions,
        "model_label": LABEL_COL, "units": {"speed_limit": "m/s", "speed_factor": "dimensionless"},
        "speed_limit_semantics": "external-road mean lane limit; same current valid-network convention as C++",
        "training_features": feature_summary(train_frame),
        "validation_features": feature_summary(validation_frame),
        "constant_training_features": [c for c in FEATURE_COLS if train_frame[c].nunique() == 1],
        "validation_role": "evaluation only; not supplied to fit or model selection",
        "internal_cv": "AutoGluon folds within training files; not certified episode-grouped",
        "p99_filter": False, "tail_reweight": False,
        "sample_weighting": weight_audit,
        "effective_excluded_model_types": excluded_models,
        "training_options": vars(args),
        "predictor_directory": "predictor",
    })
    contract_path = output / "training_contract.json"
    contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2), encoding="utf-8")
    random.seed(args.random_state)
    np.random.seed(args.random_state)
    fit_frame = train_frame[FEATURE_COLS + [LABEL_COL]].copy()
    predictor_options = {}
    if weights is not None:
        fit_frame[WEIGHT_COL] = weights
        predictor_options.update(sample_weight=WEIGHT_COL, weight_evaluation=True)
    predictor = TabularPredictor(
        label=LABEL_COL, problem_type="regression", eval_metric="mae",
        path=str(output / "predictor"),
        **predictor_options,
    ).fit(
        train_data=fit_frame,
        presets="best_quality", num_bag_folds=args.bag_folds,
        num_stack_levels=args.stack_levels, excluded_model_types=excluded_models,
        time_limit=args.time_limit, ag_args_fit={"num_gpus": args.num_gpus},
    )
    used = None
    if getattr(predictor, "feature_metadata", None) is not None:
        used = list(predictor.feature_metadata.get_features())
    if used is not None and not set(used).issubset(FEATURE_COLS):
        raise ValueError(f"Unexpected model inputs (possible weight/metadata leakage): {used}")
    contract["framework_features"] = used
    contract["raw_schema_preserved_even_if_framework_prunes_constants"] = True
    contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2), encoding="utf-8")
    prediction = predictor.predict(validation_frame[FEATURE_COLS].copy())
    results = validation_metrics(validation_frame, prediction, args.low_flow_cutoff)
    (output / "validation_metrics.json").write_text(
        json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
    print("[VALIDATION] Road-label metrics, not closed-loop trip metrics:")
    print(json.dumps(results["overall"], ensure_ascii=False))
    print(f"[SAVED] predictor={output / 'predictor'}")
    print("[NOTICE] Export with model_catching_sumo_sf.py; legacy SUMO_V1 tables do not support this schema.")
    return predictor


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    for prefix in ("", "validation-"):
        for suffix in ("csv", "csv-glob", "csv-list"):
            parser.add_argument("--" + prefix + suffix, action="append", default=[])
    parser.add_argument("--dataset-dir", default="", help="Dataset containing commands/traci_jobs.tsv")
    parser.add_argument("--run-dir", default="", help="Collection run containing training/full CSVs")
    parser.add_argument("--expected-train-files", type=int, default=0)
    parser.add_argument("--expected-validation-files", type=int, default=0)
    parser.add_argument("--validation-fraction", type=float, default=0.2,
                        help="Whole-file holdout fraction only when no external split is provided")
    parser.add_argument("--label-column", choices=LABEL_COLUMNS + ["travel_time_label"],
                        default=LABEL_COLUMNS[0])
    parser.add_argument("--save-path", required=True, help="New run directory; model is saved in predictor/")
    parser.add_argument("--time-limit", type=int, default=5400, help="AutoGluon fit budget; excludes loading/evaluation")
    parser.add_argument("--random-state", type=int, default=42)
    parser.add_argument("--num-gpus", type=int, default=0)
    parser.add_argument("--bag-folds", type=int, default=10)
    parser.add_argument("--stack-levels", type=int, default=2)
    parser.add_argument("--excluded-model-types", nargs="+", default=["NN_TORCH"],
                        help="AutoGluon exclusions; low-flow always also excludes FASTAI, which ignores weights")
    parser.add_argument("--sample-weighting", choices=("none", "low-flow"), default="none",
                        help="Training sample weights only; external metrics stay unweighted")
    parser.add_argument("--low-flow-cutoff", type=int, default=5,
                        help="Strict road_flow < cutoff; equality is NOT upweighted")
    parser.add_argument("--low-flow-weight", type=float, default=3.0,
                        help="Raw weight below cutoff; other rows have weight 1; normalized mean is 1")
    parser.add_argument("--dry-run", action="store_true", help="Validate/load only; no models or files are written")
    return parser


def validate_options(args):
    if args.low_flow_cutoff <= 0:
        raise ValueError("low-flow-cutoff must be a positive integer")
    if not np.isfinite(args.low_flow_weight) or args.low_flow_weight < 1:
        raise ValueError("low-flow-weight must be finite and at least 1")
    if args.time_limit <= 0 or args.num_gpus < 0:
        raise ValueError("time-limit must be positive and num-gpus nonnegative")
    if args.expected_train_files < 0 or args.expected_validation_files < 0:
        raise ValueError("Expected file counts must be nonnegative")
    if not 0 < args.validation_fraction < 1:
        raise ValueError("validation-fraction must be between 0 and 1")
    if args.bag_folds not in (0,) and args.bag_folds < 2:
        raise ValueError("bag-folds must be 0 or at least 2")
    if args.stack_levels < 0 or (args.stack_levels and args.bag_folds < 2):
        raise ValueError("Positive stack-levels require at least 2 bag-folds")


def main(argv=None, parser=None, entrypoint_file=None):
    args = (parser or build_parser()).parse_args(argv)
    validate_options(args)
    if not args.dry_run and Path(args.save_path).expanduser().exists():
        raise ValueError(f"Output already exists; choose a new --save-path: {args.save_path}")
    train_paths, val_paths, split_kind = resolve_splits(args)
    print(f"[CONFIG] schema={SCHEMA} features={FEATURE_COLS} label={args.label_column}")
    print(f"[CONFIG] split={split_kind} train_files={len(train_paths)} validation_files={len(val_paths)}")
    print("[CONFIG] P99 filtering OFF; tail weighting OFF; validation files are evaluation-only")
    train_frame, train_audits = load_split(train_paths, args.label_column)
    val_frame, val_audits = load_split(val_paths, args.label_column)
    reject_duplicate_contents(train_audits, val_audits)
    _, weight_audit = make_sample_weights(train_frame, args)
    print("[WEIGHTS] " + json.dumps(weight_audit, ensure_ascii=False))
    constants = [c for c in FEATURE_COLS if train_frame[c].nunique() == 1]
    print(f"[FEATURES] constant training columns={constants}; raw schema still keeps all five")
    for column in FEATURE_COLS:
        lo, hi = train_frame[column].min(), train_frame[column].max()
        outside = int(((val_frame[column] < lo) | (val_frame[column] > hi)).sum())
        if outside:
            print(f"[WARN] validation {column}: {outside} rows outside training min/max")
    provenance = {"split_kind": split_kind, "train_files": train_audits, "validation_files": val_audits}
    entrypoint = Path(entrypoint_file or __file__).resolve()
    provenance["entrypoint"] = {"path": str(entrypoint), "sha256": file_sha256(entrypoint)}
    if args.dataset_dir:
        manifest = Path(args.dataset_dir) / "commands" / "traci_jobs.tsv"
        provenance["manifest"] = {"path": str(manifest.resolve()), "sha256": file_sha256(manifest)}
    if args.dry_run:
        print("[DRY-RUN] Passed; no AutoGluon training or output-file writes")
        return
    fit_model(train_frame, val_frame, args, provenance)


if __name__ == "__main__":
    main()
