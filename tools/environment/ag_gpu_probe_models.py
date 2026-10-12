"""Observation-only subclasses of AutoGluon 1.6.1 GPU-capable learners.

Import only in an AutoGluon environment. Capture the live native model before
AutoGluon saves it: XGBoost's model-only serialization does not preserve the
complete training device configuration. These are diagnostic models, not new
production model families.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import time

import numpy as np
from autogluon.tabular.models.catboost.catboost_model import CatBoostModel
from autogluon.tabular.models.lgb.lgb_model import LGBModel
from autogluon.tabular.models.xgboost.xgboost_model import XGBoostModel

EVIDENCE_FILE = "gpu_fit_evidence.json"


def native_device_evidence(family, native):
    """Inspect fitted native state, never infer GPU use from requested resources."""
    try:
        if family == "CAT":
            params = native.get_all_params()
            device = str(params.get("task_type", "")).lower()
            details = {"task_type": params.get("task_type"), "devices": params.get("devices")}
        elif family == "XGB":
            config = json.loads(native.get_booster().save_config())
            params = config.get("learner", {}).get("generic_param", {})
            device = str(params.get("device", "")).lower()
            details = {"device": params.get("device")}
        elif family == "GBM":
            params = native.params
            device = str(params.get("device", params.get("device_type", ""))).lower()
            details = {key: params.get(key) for key in
                       ("device", "device_type", "gpu_platform_id", "gpu_device_id")}
        else:
            raise ValueError(f"Unknown backend: {family}")
        if device in ("gpu", "cuda") or device.startswith("cuda:"):
            status = "GPU_PASS"
        elif device == "cpu":
            status = "CPU_FALLBACK"
        else:
            status = "UNKNOWN"
        return {"status": status, "device": device, "native_details": details}
    except Exception as exc:
        return {"status": "UNKNOWN", "error": f"{type(exc).__name__}: {exc}"}


class GPUFitEvidenceMixin:
    probe_family = None

    def _fit(self, X, y, **kwargs):
        started = time.time()
        result = super()._fit(X=X, y=y, **kwargs)
        evidence = native_device_evidence(self.probe_family, self.model)
        weights = kwargs.get("sample_weight")
        values = np.asarray(weights, dtype=float) if weights is not None else np.array([])
        evidence.update({
            "schema": "autogluon-live-gpu-fit-evidence-v1",
            "family": self.probe_family, "model_name": self.name,
            "model_path": str(Path(self.path).resolve()),
            "captured_before_save": True, "requested_num_gpus": kwargs.get("num_gpus"),
            "pid": os.getpid(), "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
            "rows": len(X), "input_columns": list(X.columns),
            "sample_weight_received": weights is not None,
            "weight_count": len(values),
            "weight_min": float(values.min()) if len(values) else None,
            "weight_max": float(values.max()) if len(values) else None,
            "weight_mean": float(values.mean()) if len(values) else None,
            "fit_seconds": time.time() - started,
            "capture_time_unix": time.time(),
        })
        directory = Path(self.path)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / EVIDENCE_FILE).write_text(
            json.dumps(evidence, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        return result


class ProbeCatBoostModel(GPUFitEvidenceMixin, CatBoostModel):
    probe_family = "CAT"


class ProbeXGBoostModel(GPUFitEvidenceMixin, XGBoostModel):
    probe_family = "XGB"


class ProbeLGBModel(GPUFitEvidenceMixin, LGBModel):
    probe_family = "GBM"
