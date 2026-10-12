"""Current native road-time model contract; no historical pipeline imports."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

SOURCE_SCHEMA = "road-time-sumo-native-euler-streaming-quality-v2"
MODEL_SCHEMA = "road-time-native-model-v2"
CLOCK_SCHEMA = "sumo-euler-last-completed-snapshot-v2"
FEATURE_COLUMNS = ["road_length", "road_flow", "lane_flow", "free_speed_cap",
                   "platoon_context", "entry_signal_type", "entry_signal_remaining_seconds",
                   "next_signal_type", "next_signal_duration_seconds"]
CATEGORICAL_FEATURES = {"platoon_context": list(range(6)),
                        "entry_signal_type": ["R", "G"], "next_signal_type": ["R", "G"]}
NUMERIC_COLUMNS = [name for name in FEATURE_COLUMNS if name not in CATEGORICAL_FEATURES]
THRESHOLD_ORDER = ["1", "3", "0p1", "0p5", "1p5", "2"]
LABEL_COLUMNS = ["travel_time_label_gt_" + item for item in THRESHOLD_ORDER]
MODEL_LABEL = "road_time_seconds"
WEIGHT_COLUMN = "__sample_weight"
COLLECTION_SEMANTICS = {
    "schema": SOURCE_SCHEMA,
    "formal_features": FEATURE_COLUMNS,
    "label_columns": ["travel_time_label_gt_" + item for item in ("0p1", "0p5", "1", "1p5", "2", "3")],
    "flow": "physical external edge/lane snapshot at first observation of occurrence; includes ego",
    "lane": "SUMO actual lane, no projected selection, no capacity or pending records",
    "cap": "SUMO getAllowedSpeed at entry; runtime speedFactor and lane limit are audit columns",
    "context": "same actual lane, strictly ahead by front position; entry-frozen free clocks; no reset on lane change",
    "clock": "first observed snapshot time (API getTime - step) + observed remaining lane distance / entry cap",
    "signal": "same observed snapshot clock; live immediate selected link and nextSwitch; no C++ shadow",
    "labels": "verified no-internal Euler geometry allocation; six strict speed > thresholds; unknown remains blank",
    "boundary": "road occurrence identity, odometer, geometry and elapsed budget checked; unknown contaminates all affected visits",
    "zero": "valid zero and subsecond values retained; no target-magnitude exclusion",
    "clock_schema": CLOCK_SCHEMA,
    "actual_event_time_policy": "actual departure/arrival remain SUMO event timestamps, not API return time",
    "feature_anchor": "first genuine visible SUMO snapshot; record lag to estimated entry; no synthetic features",
    "quality": "per-road candidate only; entire run released only after successful close, native identity and provenance audits",
}


def model_directory_name(threshold):
    if threshold not in THRESHOLD_ORDER:
        raise ValueError("Unsupported model threshold")
    return "gt_" + threshold


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=True,
                                     separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def validate_collection_contract(contract):
    if not isinstance(contract, dict):
        raise ValueError("Missing native collection contract")
    for key, expected in COLLECTION_SEMANTICS.items():
        if contract.get(key) != expected:
            raise ValueError("Unsupported native collection semantics: " + key)
    return contract


def deployment_contract():
    """Describe training support, not an assertion that C++ proxy states match."""
    return {
        "feature_anchor": "first_genuine_visible_snapshot",
        "flow": "physical_external_snapshot_including_ego", "lane": "actual_sumo_lane",
        "cap": "sumo_getAllowedSpeed", "context": "native_entry_frozen_free_clock",
        "signal": "same_observed_snapshot_live_immediate_link",
        "labels": "no_internal_euler_geometry", "cpp_proxy_equivalence": False,
        "prediction_floor_seconds": 1.0,
        "zero_label_training_policy": "exclude_nonpositive_without_rewriting_source",
        "subsecond_policy": "positive_retained_for_training_export_floor_one_second",
    }


def encode_features(frame):
    """Validate and select exactly nine features with fixed unordered categories."""
    if frame.columns.duplicated().any() or any(name not in frame for name in FEATURE_COLUMNS):
        raise ValueError("Missing or duplicate native feature columns")
    result = pd.DataFrame(index=frame.index)
    for name in NUMERIC_COLUMNS + ["platoon_context"]:
        if pd.api.types.is_bool_dtype(frame[name].dtype):
            raise ValueError(name + " must not be boolean")
        values = pd.to_numeric(frame[name], errors="raise").astype("float64")
        if not np.isfinite(values.to_numpy()).all():
            raise ValueError(name + " must be finite")
        if name in {"road_length", "free_speed_cap"} and (values <= 0).any():
            raise ValueError(name + " must be positive")
        if name in {"road_flow", "lane_flow", "platoon_context"}:
            if (values != np.floor(values)).any() or (values < 0).any() or (values > 2**53).any():
                raise ValueError(name + " must be an exactly representable nonnegative integer")
            values = values.astype("int64")
        elif (values < 0).any():
            raise ValueError(name + " must be nonnegative")
        result[name] = values
    if (result["lane_flow"] > result["road_flow"]).any():
        raise ValueError("lane_flow exceeds road_flow")
    for name, categories in CATEGORICAL_FEATURES.items():
        values = result[name] if name in result else frame[name]
        if not values.isin(categories).all():
            raise ValueError("Unsupported category: " + name)
        result[name] = pd.Categorical(values, categories=categories, ordered=False)
    first = result["entry_signal_remaining_seconds"]
    second = result["next_signal_duration_seconds"]
    sentinel = ((result["entry_signal_type"] == "G") & (first == 0)
                & (result["next_signal_type"] == "G") & (second == 0))
    if ((~sentinel) & ((first <= 0) | (second <= 0))).any():
        raise ValueError("Only final-road G0G0 sentinel permits zero signal durations")
    return result[FEATURE_COLUMNS]


def feature_ranges(frame):
    result = {}
    for name in FEATURE_COLUMNS:
        if name in CATEGORICAL_FEATURES:
            categories = CATEGORICAL_FEATURES[name]
            result[name] = {"categories": categories,
                            "observed": [item for item in categories if (frame[name] == item).any()]}
        else:
            result[name] = {"min": float(frame[name].min()), "max": float(frame[name].max())}
    return result


def validate_model_contract(contract):
    if not isinstance(contract, dict) or contract.get("schema") != MODEL_SCHEMA:
        raise ValueError("Unsupported native model schema")
    for key, value in (("status", "COMPLETE"), ("source_schema", SOURCE_SCHEMA),
                       ("feature_columns", FEATURE_COLUMNS), ("categorical", CATEGORICAL_FEATURES),
                       ("model_label", MODEL_LABEL), ("predictor_directory", "predictor")):
        if contract.get(key) != value:
            raise ValueError("Invalid model contract field: " + key)
    if contract.get("source_label_column") not in LABEL_COLUMNS:
        raise ValueError("Unsupported source label column")
    validate_collection_contract(contract.get("collection_contract"))
    if contract.get("deployment_contract") != deployment_contract():
        raise ValueError("Unsupported native deployment contract")
    used = contract.get("framework_raw_features")
    if (not isinstance(used, list) or not used or len(set(used)) != len(used)
            or not set(used).issubset(FEATURE_COLUMNS)
            or contract.get("framework_removed_raw_features") != [name for name in FEATURE_COLUMNS if name not in used]):
        raise ValueError("Invalid framework raw-feature whitelist/removal audit")
    ranges = contract.get("feature_ranges")
    if not isinstance(ranges, dict) or set(ranges) != set(FEATURE_COLUMNS):
        raise ValueError("Missing native feature ranges")
    for name in FEATURE_COLUMNS:
        item = ranges[name]
        if not isinstance(item, dict):
            raise ValueError("Invalid feature range: " + name)
        if name in CATEGORICAL_FEATURES:
            categories = CATEGORICAL_FEATURES[name]
            if (item.get("categories") != categories or not isinstance(item.get("observed"), list)
                    or not item["observed"] or len(set(item["observed"])) != len(item["observed"])
                    or any(value not in categories for value in item["observed"])):
                raise ValueError("Invalid categorical feature range: " + name)
        else:
            low, high = item.get("min"), item.get("max")
            if (type(low) not in (int, float) or type(high) not in (int, float)
                    or not math.isfinite(low) or not math.isfinite(high) or low < 0 or high < low
                    or (name in {"road_length", "free_speed_cap"} and low <= 0)):
                raise ValueError("Invalid numeric feature range: " + name)
    return contract
