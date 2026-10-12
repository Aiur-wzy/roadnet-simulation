#!/usr/bin/env python3
"""Verify actual GPU fitting THROUGH AutoGluon, using tiny synthetic data only.

No pip installs, production data, model overwrites or automatic cleanup.
Default: CAT/XGB/GBM, single-model and two-fold bagging, 20 rounds each.
Exit 0 means all requested smoke cases proved GPU use; 1 means failed/partial;
2 means environment blocked or invalid options. --inventory-only is NOT a GPU pass.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import math
import os
from pathlib import Path
from roadnet_paths import ROOT, source_path
import platform
import subprocess
import sys
import time
import traceback
import uuid

FEATURES = ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"]
LABEL = "travel_time_no_waiting"
WEIGHT = "__sample_weight"
AUDITED_VERSION = "1.6.1"
EVIDENCE_FILE = "gpu_fit_evidence.json"


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def collect_inventory():
    result = {"python": sys.executable, "python_version": sys.version,
              "platform": platform.platform(), "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
              "versions": {}, "note": "Inventory alone does not prove GPU training."}
    for package in ("autogluon.tabular", "autogluon.core", "autogluon.common", "numpy", "pandas",
                    "scikit-learn", "torch", "ray", "catboost", "xgboost", "lightgbm"):
        try:
            result["versions"][package] = version(package)
        except PackageNotFoundError:
            result["versions"][package] = None
    try:
        proc = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,name,uuid,driver_version,memory.total", "--format=csv"],
            capture_output=True, text=True, errors="replace", timeout=15)
        result["nvidia_smi"] = {"returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}
    except (OSError, subprocess.TimeoutExpired) as exc:
        result["nvidia_smi"] = {"error": str(exc)}
    # CPU-only torch is diagnostic information, not proof that CatBoost cannot use CUDA.
    try:
        import torch
        result["torch_cuda"] = {"built_cuda": torch.version.cuda, "available": torch.cuda.is_available(),
                                "visible_count": torch.cuda.device_count()}
    except Exception as exc:
        result["torch_cuda"] = {"error": f"{type(exc).__name__}: {exc}"}
    return result


def make_data(rows=2000, seed=42):
    import numpy as np
    import pandas as pd
    rng = np.random.default_rng(seed)
    flow = rng.integers(1, 41, size=rows)
    flow[:3] = [4, 5, 6]
    lane = np.array([rng.integers(1, count + 1) for count in flow])
    frame = pd.DataFrame({
        "road_length": rng.choice([320.0, 545.6], size=rows),
        "road_flow": flow, "lane_flow": lane,
        "speed_factor": rng.uniform(0.8, 1.2, size=rows), "speed_limit": 13.89,
    })
    frame[LABEL] = (frame.road_length / (frame.speed_factor * frame.speed_limit)
                    + 0.12 * (flow - 1) + 0.03 * lane + rng.normal(0, 0.15, size=rows))
    raw = np.where(flow < 5, 3.0, 1.0)
    frame[WEIGHT] = raw / raw.mean()
    return frame


def case_parameters(family, stage, args):
    if family == "CAT":
        # Logical device 0 in the worker's visible range; do not use all GPUs by default.
        params = {"iterations": args.rounds, "depth": 4, "devices": "0", "allow_writing_files": False}
    elif family == "XGB":
        params = {"n_estimators": args.rounds, "max_depth": 4}
    elif family == "GBM":
        params = {"num_boost_round": args.rounds, "num_leaves": 15, "max_bin": 63}
        if args.lgb_gpu_platform_id is not None:
            params["gpu_platform_id"] = args.lgb_gpu_platform_id
        if args.lgb_gpu_device_id is not None:
            params["gpu_device_id"] = args.lgb_gpu_device_id
    else:
        raise ValueError(f"Unknown family: {family}")
    folds, stacks = {"single": (0, 0), "bagged": (2, 0), "stacked": (2, 1)}[stage]
    fit_options = {
        "num_gpus": 1, "num_cpus": args.num_cpus, "ag_args_fit": {"num_gpus": 1},
        "num_bag_folds": folds, "num_bag_sets": 1, "num_stack_levels": stacks,
        "fit_strategy": "sequential", "dynamic_stacking": False,
        "fit_weighted_ensemble": False, "time_limit": args.time_limit,
    }
    if folds:
        fit_options["ag_args_ensemble"] = {"fold_fitting_strategy": args.fold_strategy, "num_folds_parallel": 2}
    return params, fit_options


def read_successful_evidence(predictor):
    """Follow successful models/children only; never count discarded fit artifacts."""
    names = predictor.model_names()
    evidence, seen = [], set()

    def visit(model):
        if hasattr(model, "models") and hasattr(model, "load_child"):
            if not model.models:
                evidence.append({"status": "UNKNOWN", "error": "Successful bagged model has no children"})
            for child in model.models:
                visit(model.load_child(child))
            return
        path = Path(model.path).resolve() / EVIDENCE_FILE
        if path in seen:
            return
        seen.add(path)
        try:
            item = json.loads(path.read_text(encoding="utf-8"))
            item["evidence_path"] = str(path)
            if not item.get("captured_before_save") or Path(item["model_path"]).resolve() != path.parent:
                raise ValueError("Not matching live pre-save evidence for this model")
            evidence.append(item)
        except Exception as exc:
            evidence.append({"status": "UNKNOWN", "evidence_path": str(path), "error": str(exc)})

    for name in names:
        visit(predictor._trainer.load_model(name))
    return names, evidence


def classify_case(stage, family, names, evidence):
    expected_models, expected_leaves = {"single": (1, 1), "bagged": (1, 2), "stacked": (2, 4)}[stage]
    if any(item.get("status") == "CPU_FALLBACK" for item in evidence):
        return "CPU_FALLBACK"
    if len(names) != expected_models or len(evidence) != expected_leaves:
        return "INCOMPLETE"
    for item in evidence:
        try:
            device = item.get("device", "").lower()
            details = item["native_details"]
            native_device = str(details.get("task_type") if family == "CAT" else
                                details.get("device", details.get("device_type", ""))).lower()
            valid_device = device in ("gpu", "cuda") or device.startswith("cuda:")
            w_min, w_max, w_mean = (float(item[key]) for key in ("weight_min", "weight_max", "weight_mean"))
            valid_weight = (all(math.isfinite(value) for value in (w_min, w_max, w_mean))
                            and 0 < w_min <= w_mean <= w_max)
        except (KeyError, TypeError, ValueError, AttributeError):
            return "UNKNOWN"
        if (item.get("status") != "GPU_PASS" or item.get("family") != family
                or item.get("captured_before_save") is not True or item.get("sample_weight_received") is not True
                or item.get("weight_count") != item.get("rows") or not item.get("rows")
                or not valid_weight or not valid_device or native_device != device):
            return "UNKNOWN"
    return "GPU_PASS"


def run_case(family, stage, args, output, train, heldout, predictor_class, model_class):
    import numpy as np
    case_dir = output / f"{family.lower()}_{stage}"
    case_dir.mkdir()
    started = time.time()
    params, fit_options = case_parameters(family, stage, args)
    result = {"family": family, "stage": stage, "fit_options": fit_options,
              "hyperparameters": params, "status": "ERROR"}
    write_json(case_dir / "case_config.json", result)
    try:
        predictor = predictor_class(
            label=LABEL, problem_type="regression", eval_metric="mae",
            sample_weight=WEIGHT, weight_evaluation=True, path=str(case_dir / "predictor"),
            verbosity=3, log_to_file=True,
        ).fit(train_data=train.copy(), hyperparameters={model_class: params}, **fit_options)
        used = predictor.feature_metadata.get_features()
        if not set(used).issubset(FEATURES):
            raise ValueError(f"Weight/metadata leaked into predictor raw features: {used}")
        prediction = np.asarray(predictor.predict(heldout[FEATURES]), dtype=float)
        if prediction.shape != (len(heldout),) or not np.isfinite(prediction).all():
            raise ValueError("Invalid predictions after smoke fit")
        names, evidence = read_successful_evidence(predictor)
        result.update({"model_names": names, "live_fit_evidence": evidence,
                       "framework_features": used, "status": classify_case(stage, family, names, evidence),
                       "prediction_rows": len(prediction), "prediction_finite": True})
    except Exception as exc:
        result.update({"status": "ERROR", "error": f"{type(exc).__name__}: {exc}",
                       "traceback": traceback.format_exc()})
    result["seconds"] = time.time() - started
    write_json(case_dir / "case_result.json", result)
    print(f"[CASE] {family}/{stage}: {result['status']}", flush=True)
    return result


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--output-dir", default="", help="New directory only; default eval/ag_gpu_check_<timestamp>_<id>")
    parser.add_argument("--families", nargs="+", choices=["CAT", "XGB", "GBM"], default=["CAT", "XGB", "GBM"])
    parser.add_argument("--stages", nargs="+", choices=["single", "bagged", "stacked"], default=["single", "bagged"])
    parser.add_argument("--rows", type=int, default=2000)
    parser.add_argument("--rounds", type=int, default=20)
    parser.add_argument("--time-limit", type=int, default=300, help="Per-case fit budget, not a hard process timeout")
    parser.add_argument("--num-cpus", type=int, default=4)
    parser.add_argument("--fold-strategy", choices=["parallel_local", "sequential_local"], default="parallel_local")
    parser.add_argument("--lgb-gpu-platform-id", type=int)
    parser.add_argument("--lgb-gpu-device-id", type=int)
    parser.add_argument("--inventory-only", action="store_true", help="No fit; this is NOT a GPU training pass")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not 200 <= args.rows <= 10000 or not 1 <= args.rounds <= 100:
        raise ValueError("Smoke only: rows must be 200..10000 and rounds 1..100")
    if args.time_limit <= 0 or args.num_cpus <= 0:
        raise ValueError("time-limit and num-cpus must be positive")
    if len(set(args.families)) != len(args.families) or len(set(args.stages)) != len(args.stages):
        raise ValueError("Duplicate family/stage would reuse output directories")
    for value in (args.lgb_gpu_platform_id, args.lgb_gpu_device_id):
        if value is not None and value < 0:
            raise ValueError("Explicit LightGBM OpenCL device/platform IDs must be nonnegative")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    output = Path(args.output_dir or f"eval/ag_gpu_check_{stamp}_{uuid.uuid4().hex[:8]}").resolve()
    if output.exists():
        raise ValueError(f"Output already exists; choose a new directory: {output}")
    output.mkdir(parents=True)
    inventory = collect_inventory()
    write_json(output / "environment.json", inventory)
    report = {"schema": "sf-autogluon-gpu-smoke-v1", "status": "ENVIRONMENT_BLOCKED",
              "config": vars(args), "cases": [], "environment": inventory,
              "raw_features": FEATURES, "weight_rule": "road_flow < 5: 3; otherwise 1; normalize train mean to 1",
              "data": "synthetic only; no project CSV, route or SUMO labels read",
              "requested_resources": {"total_gpus": 1, "gpus_per_fold": 1},
              "checker_sha256": sha256(__file__),
              "limitations": ["Not the full best_quality portfolio or production 10-fold/2-stack run",
                              "No assertion of prediction quality, speedup, GPU peak-memory capacity or multi-GPU correctness",
                              "GPU status requires live post-fit/pre-save native evidence from all successful children",
                              "LightGBM OpenCL device IDs need not match CUDA/nvidia-smi device IDs"]}
    report_path = output / "gpu_report.json"
    print(f"[OUTPUT] {output}", flush=True)
    if args.inventory_only:
        report["status"] = "INVENTORY_ONLY_NOT_VERIFIED"
        write_json(report_path, report)
        print("[NOT VERIFIED] Inventory only; no AutoGluon fit was attempted.")
        return 0
    if inventory["versions"].get("autogluon.tabular") != AUDITED_VERSION:
        report["error"] = f"This probe requires installed autogluon.tabular=={AUDITED_VERSION}; no packages were installed or changed."
        write_json(report_path, report)
        print("[ENVIRONMENT_BLOCKED] " + report["error"])
        return 2
    try:
        # Make top-level observation classes importable in Ray workers before initialization.
        scripts_dir = str(ROOT)
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        os.environ["PYTHONPATH"] = scripts_dir + os.pathsep + os.environ.get("PYTHONPATH", "")
        from autogluon.tabular import TabularPredictor
        from autogluon.common.utils.resource_utils import ResourceManager
        from tools.environment.ag_gpu_probe_models import ProbeCatBoostModel, ProbeXGBoostModel, ProbeLGBModel
        report["probe_models_sha256"] = sha256(source_path('scripts/ag_gpu_probe_models.py'))
        # AutoGluon 1.6.1 exposes get_gpu_count(); inventory is not fit evidence.
        report["autogluon_gpu_count_api"] = "ResourceManager.get_gpu_count"
        detected = ResourceManager.get_gpu_count()
        report["autogluon_detected_gpu_count"] = detected
        if detected < 1:
            raise RuntimeError("AutoGluon detected zero GPUs; no GPU fit can be scheduled")
        models = {"CAT": ProbeCatBoostModel, "XGB": ProbeXGBoostModel, "GBM": ProbeLGBModel}
        train, heldout = make_data(args.rows, 42), make_data(400, 43)
        report["synthetic_train_rows"] = len(train)
        report["synthetic_holdout_rows"] = len(heldout)
        report["low_flow_rows"] = int((train.road_flow < 5).sum())
        report["training_weight_mean"] = float(train[WEIGHT].mean())
    except Exception as exc:
        report.update({"error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()})
        write_json(report_path, report)
        print("[ENVIRONMENT_BLOCKED] " + report["error"])
        return 2
    report["status"] = "RUNNING"
    write_json(report_path, report)
    for stage in args.stages:
        for family in args.families:
            case = run_case(family, stage, args, output, train, heldout, TabularPredictor, models[family])
            report["cases"].append(case)
            write_json(report_path, report)
    all_pass = all(case["status"] == "GPU_PASS" for case in report["cases"])
    report["status"] = "ALL_REQUESTED_GPU_CASES_PASS" if all_pass else "NOT_ALL_GPU_CASES_PASSED"
    write_json(report_path, report)
    print("[RESULT] " + report["status"])
    print(f"[REPORT] {report_path}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
