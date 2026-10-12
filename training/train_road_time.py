"""Train current native road-time models from complete, hash-bound releases.

Preflight and external evaluation are chunked. AutoGluon fitting is explicitly
in-memory, gated by a conservative declared budget, never hidden sampling.
"""
from __future__ import annotations

import argparse
import gc
from datetime import datetime, timezone
from importlib.metadata import version
import json
import math
from pathlib import Path
import random
import sys

import numpy as np
import pandas as pd

from training.native_contract import (CATEGORICAL_FEATURES, FEATURE_COLUMNS, LABEL_COLUMNS,
    MODEL_LABEL, MODEL_SCHEMA, NUMERIC_COLUMNS, SOURCE_SCHEMA, THRESHOLD_ORDER, WEIGHT_COLUMN,
    deployment_contract, encode_features, feature_ranges, model_directory_name,
    sha256_file, validate_model_contract)
from training.native_data import iter_chunks, load_batch, preflight, verify_file

WEIGHT_AUDITED_AUTOGLUON_VERSION = "1.6.1"


def _validate_random_state(value):
    """Match the NumPy/AutoGluon scalar seed domain before loading data."""
    if type(value) is not int or not 0 <= value <= 2**32 - 1:
        raise ValueError("--random-state must be an integer in [0, 2**32 - 1]")


def memory_plan(rows, budget_gb, chunk_size, bag_folds, stack_levels):
    """Conservative admission estimate, not a promise about arbitrary estimators.

    Account for multiple pandas/model copies, bagging/stacking, bounded decode
    buffers and one GiB process overhead. Fit can still fail under OS pressure.
    """
    if type(rows) is not int or rows <= 0:
        raise ValueError("Selected label has no positive training rows")
    if not math.isfinite(budget_gb) or budget_gb <= 0:
        raise ValueError("An explicit positive --memory-budget-gb is required")
    copies = 3 + stack_levels + max(1, math.ceil(bag_folds / 2))
    estimate = 1024**3 + rows * 512 * copies + chunk_size * 4096
    budget = int(budget_gb * 1024**3)
    return {"fit_rows": rows, "estimated_peak_bytes": estimate, "budget_bytes": budget,
            "admitted": estimate <= budget, "bytes_per_row_per_copy": 512,
            "copy_allowance": copies, "mode": "in_memory_no_sampling",
            "limitation": "conservative admission estimate, not guaranteed peak RSS; model allocation may still fail"}


def require_memory(plan):
    if not plan["admitted"]:
        raise ValueError("In-memory fit exceeds declared budget before materialization: "
                         + json.dumps(plan, sort_keys=True))


def materialize_training(batch, label, expected_rows, chunk_size, memory):
    require_memory(memory)
    if memory["fit_rows"] != expected_rows:
        raise ValueError("Memory plan and selected training count differ")
    arrays = {name: np.empty(expected_rows, dtype="int8" if name in CATEGORICAL_FEATURES
              else "int64" if name in {"road_flow", "lane_flow"} else "float64")
              for name in FEATURE_COLUMNS}
    target = np.empty(expected_rows, dtype="float64")
    offset = 0
    for scenario in batch.scenarios:
        if scenario.split != "train":
            continue
        verify_file(scenario.training_path, scenario.training_sha256)
        for raw in iter_chunks(scenario, chunk_size):
            labels = pd.to_numeric(raw[label], errors="raise").to_numpy(dtype="float64")
            keep = labels > 0
            selected = encode_features(raw.loc[keep])
            end = offset + len(selected)
            if end > expected_rows:
                raise ValueError("Training source grew after preflight")
            for name in FEATURE_COLUMNS:
                values = selected[name]
                arrays[name][offset:end] = (values.cat.codes.to_numpy() if name in CATEGORICAL_FEATURES
                                             else values.to_numpy())
            target[offset:end] = labels[keep]
            offset = end
        verify_file(scenario.training_path, scenario.training_sha256)
    if offset != expected_rows:
        raise ValueError("Training count changed after preflight")
    for name, categories in CATEGORICAL_FEATURES.items():
        arrays[name] = pd.Categorical.from_codes(arrays[name], categories=categories, ordered=False)
    frame = pd.DataFrame(arrays, copy=False)[FEATURE_COLUMNS]
    frame[MODEL_LABEL] = target
    return frame


def sample_weights(frame, mode, cutoff=5, low_weight=3.0):
    if mode == "none":
        return None, {"mode": "none", "external_metrics": "unweighted", "resampling": False}
    if mode != "low-flow" or not math.isfinite(low_weight) or low_weight <= 0 or cutoff < 1:
        raise ValueError("Invalid explicit sample-weighting options")
    selected = frame["road_flow"].to_numpy() < cutoff
    raw = np.where(selected, low_weight, 1.0)
    mean = float(raw.mean())
    if not math.isfinite(mean) or mean <= 0:
        raise ValueError("Invalid retained-training weight normalization")
    return raw / mean, {"mode": mode, "column": WEIGHT_COLUMN, "input": "road_flow",
        "operator": "<", "cutoff": cutoff, "low_flow_raw_weight": low_weight,
        "other_raw_weight": 1.0, "normalization": "retained training mean only",
        "raw_weight_mean": mean, "rows": len(raw), "low_flow_rows": int(selected.sum()),
        "external_metrics": "unweighted", "weight_evaluation": True,
        "adds_predictive_feature": False, "resampling": False}


class MetricAccumulator:
    def __init__(self):
        self.n = 0
        self.sums = [0.0, 0.0, 0.0]
        self.corrections = [0.0, 0.0, 0.0]

    def add(self, truth, prediction):
        residual = np.asarray(prediction, dtype="float64") - np.asarray(truth, dtype="float64")
        if residual.ndim != 1 or not np.isfinite(residual).all():
            raise ValueError("Invalid validation prediction shape/value")
        if not len(residual):
            return
        terms = [float(np.abs(residual).sum()), float(residual.sum()), float(np.dot(residual, residual))]
        if not all(math.isfinite(value) for value in terms):
            raise ValueError("Validation metric overflow")
        self.n += len(residual)
        for index, value in enumerate(terms):
            old = self.sums[index]
            new = old + value
            self.corrections[index] += (old - new) + value if abs(old) >= abs(value) else (value - new) + old
            self.sums[index] = new

    def finish(self):
        if not self.n:
            return {"n": 0}
        absolute, bias, squared = [value + correction for value, correction in zip(self.sums, self.corrections)]
        return {"n": self.n, "mae": absolute / self.n, "bias": bias / self.n,
                "rmse": math.sqrt(max(0., squared / self.n))}


def evaluate_streaming(batch, predictor, label, chunk_size, low_flow_cutoff=5):
    accumulators = {"overall": {"all": MetricAccumulator()}, "by_scenario": {}, "by_family": {},
                    "by_final_road": {}, "by_low_flow": {}, "by_road_flow": {},
                    "by_platoon_context": {}, "by_signal_pair": {}}
    for scenario in batch.scenarios:
        if scenario.split != "validation":
            continue
        verify_file(scenario.training_path, scenario.training_sha256)
        for raw in iter_chunks(scenario, chunk_size):
            labels = pd.to_numeric(raw[label], errors="raise").to_numpy(dtype="float64")
            keep = labels > 0
            if not keep.any():
                continue
            features = encode_features(raw.loc[keep])
            truth = labels[keep]
            prediction = np.asarray(predictor.predict(features), dtype="float64")
            if prediction.shape != truth.shape or not np.isfinite(prediction).all():
                raise ValueError("Invalid external validation predictions")
            flow = features["road_flow"].to_numpy()
            final = raw.loc[keep, "is_final_road"].to_numpy()
            context = features["platoon_context"].to_numpy()
            signal_pair = (raw.loc[keep, "entry_signal_type"] + raw.loc[keep, "next_signal_type"]).to_numpy()
            groups = {
                "overall": {"all": np.ones(len(truth), dtype=bool)},
                "by_scenario": {scenario.scenario_id: np.ones(len(truth), dtype=bool)},
                "by_family": {scenario.family_id: np.ones(len(truth), dtype=bool)},
                "by_final_road": {key: final == key for key in ("0", "1")},
                "by_low_flow": {"lt_" + str(low_flow_cutoff): flow < low_flow_cutoff,
                                "ge_" + str(low_flow_cutoff): flow >= low_flow_cutoff},
                "by_road_flow": {"lt_5": flow < 5, "5_to_9": (flow >= 5) & (flow < 10),
                                 "10_to_19": (flow >= 10) & (flow < 20),
                                 "20_to_29": (flow >= 20) & (flow < 30), "ge_30": flow >= 30},
                "by_platoon_context": {str(key): context == key for key in range(6)},
                "by_signal_pair": {key: signal_pair == key for key in ("RR", "RG", "GR", "GG")},
            }
            for group, masks in groups.items():
                for key, mask in masks.items():
                    accumulators[group].setdefault(key, MetricAccumulator()).add(truth[mask], prediction[mask])
        verify_file(scenario.training_path, scenario.training_sha256)
    if not accumulators["overall"]["all"].n:
        raise ValueError("No positive external-validation rows for selected label")
    result = {group: {key: value.finish() for key, value in mapping.items()}
              for group, mapping in accumulators.items()}
    result["overall"] = result["overall"]["all"]
    result["metric_weighting"] = "unweighted road rows; bias = prediction - label"
    result["p95_status"] = "not computed: bounded streaming evaluation does not approximate exact quantiles"
    return result


def _write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, ensure_ascii=True, allow_nan=False)
        stream.write("\n")
    temporary.replace(path)


def _predictor_hashes(directory):
    hashes = {}
    for path in sorted(Path(directory).rglob("*")):
        if path.is_symlink():
            raise ValueError("Predictor artifact symlink refused")
        if path.is_file():
            hashes[path.relative_to(directory).as_posix()] = sha256_file(path)
    if not hashes:
        raise ValueError("No fitted predictor artifacts")
    return hashes


def fit_one(batch, report, threshold, output, args, memory, predictor_class=None):
    _validate_random_state(args.random_state)
    require_memory(memory)
    label = "travel_time_label_gt_" + threshold
    if predictor_class is None:
        # Lazy import: --dry-run never imports AutoGluon or creates model files.
        from autogluon.tabular import TabularPredictor
        predictor_class = TabularPredictor
    ag_version = version("autogluon.tabular") if predictor_class.__module__.startswith("autogluon") else "test-double"
    if args.sample_weighting == "low-flow" and ag_version not in {WEIGHT_AUDITED_AUTOGLUON_VERSION, "test-double"}:
        raise ValueError("low-flow is audited only for AutoGluon " + WEIGHT_AUDITED_AUTOGLUON_VERSION)
    frame = materialize_training(batch, label, memory["fit_rows"], args.chunk_size, memory)
    weights, weight_audit = sample_weights(frame, args.sample_weighting, args.low_flow_cutoff, args.low_flow_weight)
    ranges = feature_ranges(frame)
    options = {}
    if weights is not None:
        frame[WEIGHT_COLUMN] = weights
        options = {"sample_weight": WEIGHT_COLUMN, "weight_evaluation": True}
    excluded = sorted(set(args.excluded_model_types + (["NN_TORCH", "FASTAI"] if weights is not None else [])))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    contract = {
        "schema": MODEL_SCHEMA, "status": "FIT_STARTED", "source_schema": SOURCE_SCHEMA,
        "source_label_column": label, "model_label": MODEL_LABEL,
        "feature_columns": FEATURE_COLUMNS, "categorical": CATEGORICAL_FEATURES,
        "feature_ranges": ranges, "collection_contract": batch.collection_contract,
        "deployment_contract": deployment_contract(), "predictor_directory": "predictor",
        "provenance": batch.provenance, "preflight": report, "memory_plan": memory,
        "sample_weighting": weight_audit, "training_options": vars(args),
        "validation_role": "fixed family-disjoint external validation; never supplied to fit/model selection",
        "internal_cv": "AutoGluon row folds within training only; not certified family-grouped",
        "sampling": "all released positive-label training rows; no resampling or tail filtering",
        "effective_excluded_model_types": excluded,
        "dependencies": {"python": sys.version, "numpy": np.__version__, "pandas": pd.__version__,
                         "autogluon.tabular": ag_version},
        "training_sources_sha256": {name: sha256_file(Path(__file__).with_name(name))
                                     for name in ("train_road_time.py", "native_contract.py", "native_data.py", "native_release.py")},
        "started_utc": datetime.now(timezone.utc).isoformat(),
    }
    _write_json(output / "model_contract.json", contract)
    try:
        random.seed(args.random_state)
        np.random.seed(args.random_state)
        predictor = predictor_class(label=MODEL_LABEL, problem_type="regression", eval_metric="mae",
                                    path=str(output / "predictor"),
                                    learner_kwargs={"random_state": args.random_state}, **options).fit(
            train_data=frame, presets="best_quality", num_bag_folds=args.bag_folds,
            num_stack_levels=args.stack_levels, excluded_model_types=excluded,
            time_limit=args.time_limit, num_gpus=args.num_gpus, memory_limit=args.memory_budget_gb,
            ag_args_fit={"num_gpus": args.num_gpus})
        metadata = getattr(predictor, "feature_metadata", None)
        used = metadata.get_features() if metadata is not None else []
        if not used or len(set(used)) != len(used) or not set(used).issubset(FEATURE_COLUMNS):
            raise ValueError("Fitted model uses missing/duplicate/unapproved native inputs")
        if getattr(predictor, "label", None) != MODEL_LABEL:
            raise ValueError("Fitted predictor label differs from model contract")
        contract.update(framework_raw_features=list(used),
                        framework_removed_raw_features=[name for name in FEATURE_COLUMNS if name not in used])
        del frame
        metrics = evaluate_streaming(batch, predictor, label, args.chunk_size, args.low_flow_cutoff)
        expected_validation = report["rows_by_split_and_label"]["validation"][label]["positive"]
        if metrics["overall"]["n"] != expected_validation:
            raise ValueError("Validation row count changed after preflight")
        _write_json(output / "validation_metrics.json", metrics)
        contract.update(status="COMPLETE", finished_utc=datetime.now(timezone.utc).isoformat(),
                        validation_metrics_file="validation_metrics.json",
                        validation_metrics_sha256=sha256_file(output / "validation_metrics.json"),
                        predictor_files_sha256=_predictor_hashes(output / "predictor"))
        validate_model_contract(contract)
        _write_json(output / "model_contract.json", contract)
        result = {"threshold": threshold, "model_directory": str(output),
                "model_contract_sha256": sha256_file(output / "model_contract.json"),
                "validation": metrics["overall"]}
        del predictor
        gc.collect()
        return result
    except Exception as error:
        contract.update(status="FAILED", error=type(error).__name__ + ": " + str(error))
        _write_json(output / "model_contract.json", contract)
        raise


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch-root", required=True)
    parser.add_argument("--dataset-manifest", help="Optional relocated frozen dataset manifest; verify its bytes")
    parser.add_argument("--output-dir", required=True, help="Fresh directory; no resume or overwrite")
    parser.add_argument("--sample-weighting", required=True, choices=["none", "low-flow"])
    parser.add_argument("--memory-budget-gb", required=True, type=float)
    choice = parser.add_mutually_exclusive_group()
    choice.add_argument("--threshold", choices=THRESHOLD_ORDER, default="1")
    choice.add_argument("--all-thresholds", action="store_true")
    parser.add_argument("--chunk-size", type=int, default=100000)
    parser.add_argument("--expected-train-scenarios", type=int)
    parser.add_argument("--expected-validation-scenarios", type=int)
    parser.add_argument("--dry-run", action="store_true", help="Hash+quality preflight and memory plan only; no AutoGluon import/writes")
    parser.add_argument("--low-flow-cutoff", type=int, default=5)
    parser.add_argument("--low-flow-weight", type=float, default=3.)
    parser.add_argument("--time-limit", type=int, default=5400)
    parser.add_argument("--bag-folds", type=int, default=10)
    parser.add_argument("--stack-levels", type=int, default=2)
    parser.add_argument("--num-gpus", type=int, default=0)
    parser.add_argument("--random-state", type=int, default=0)
    parser.add_argument("--excluded-model-types", nargs="*", default=["NN_TORCH"])
    return parser


def run(args):
    _validate_random_state(args.random_state)
    if (args.chunk_size <= 0 or args.time_limit <= 0 or args.bag_folds < 0 or args.bag_folds == 1
            or args.stack_levels < 0 or (args.stack_levels and not args.bag_folds) or args.num_gpus < 0
            or args.low_flow_cutoff < 1 or not math.isfinite(args.low_flow_weight) or args.low_flow_weight <= 0
            or not math.isfinite(args.memory_budget_gb) or args.memory_budget_gb <= 0):
        raise ValueError("Invalid training resource/options")
    for value in (args.expected_train_scenarios, args.expected_validation_scenarios):
        if value is not None and value <= 0:
            raise ValueError("Expected split count must be positive")
    output = Path(args.output_dir)
    if output.exists():
        raise ValueError("Output exists; use a fresh model directory")
    if 1024**3 + args.chunk_size * 4096 > args.memory_budget_gb * 1024**3:
        raise ValueError("Declared budget is below the bounded preflight chunk/overhead allowance; reduce --chunk-size or increase budget")
    progress = lambda message: print(message, file=sys.stderr, flush=True)
    batch = load_batch(args.batch_root, args.expected_train_scenarios,
                       args.expected_validation_scenarios, args.dataset_manifest, progress=progress)
    report = preflight(batch, args.chunk_size, progress=progress)
    thresholds = THRESHOLD_ORDER if args.all_thresholds else [args.threshold]
    plans = {}
    for threshold in thresholds:
        label = "travel_time_label_gt_" + threshold
        rows = report["rows_by_split_and_label"]["train"][label]["positive"]
        if not report["rows_by_split_and_label"]["validation"][label]["positive"]:
            raise ValueError("Selected target has no positive external validation rows")
        plans[threshold] = memory_plan(rows, args.memory_budget_gb, args.chunk_size, args.bag_folds, args.stack_levels)
    plan = {"status": "DRY_RUN_COMPLETE", "threshold_order": thresholds,
            "memory_plans": plans, "preflight": report}
    if args.dry_run:
        return plan
    for memory in plans.values():
        require_memory(memory)
    output.mkdir(parents=True, exist_ok=False)
    _write_json(output / "preflight.json", report)
    results = []
    for threshold in thresholds:
        results.append(fit_one(batch, report, threshold, output / model_directory_name(threshold), args, plans[threshold]))
    summary = {"status": "COMPLETE", "schema": MODEL_SCHEMA, "threshold_order": thresholds, "runs": results}
    _write_json(output / "training_summary.json", summary)
    return summary


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        result = run(args)
    except (ValueError, OSError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
