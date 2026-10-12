#!/usr/bin/env python3
"""Train nine-input ABC + two R/G segment models, with explicit weighting.

Default target is gt_1; --all-thresholds is opt-in. Requires current collection
CSV, audit CSV and matched contracts. No signal duration is added to the label.
"""
from __future__ import annotations

import copy
import json
import math
from numbers import Real
import random
import sys
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from roadnet_paths import source_path

import numpy as np
import pandas as pd

import training._shared.model_training_sumo_sf as legacy
import training._shared.model_training_sumo_cap_context_abc as abc
from features.cap_context_features import FEATURE_COLS as BASE_FEATURE_COLS
from features.signal_rg_features import SIGNAL_FEATURE_COLS, SIGNAL_FEATURE_SCHEMA, signal_feature_contract, longest_finite_green_seconds, signal_runtime_support_rule
from features.signal_rg_features import signal_entry_reference_rule
from features.cap_context_abc_features import (
    ABC_CONTEXT_IDS, validate_abc_context_contract,
)

SCHEMA = SIGNAL_FEATURE_SCHEMA
FEATURE_COLS = list(BASE_FEATURE_COLS) + list(SIGNAL_FEATURE_COLS)
DEFAULT_LABEL_COLUMN = "travel_time_label_gt_1"
META_COLS = ["vehicleID", "route_index", "is_final_road"]
LABEL_COL = legacy.LABEL_COL
LABEL_COLUMNS = list(legacy.LABEL_COLUMNS)
THRESHOLD_ORDER = ["0p1", "3", "1", "0p5", "1p5", "2"]
CATEGORICAL_FEATURES = {"platoon_context": list(ABC_CONTEXT_IDS),
                        "entry_signal_type": ["R", "G"], "next_signal_type": ["R", "G"]}
NUMERIC_FEATURES = [name for name in FEATURE_COLS if name not in CATEGORICAL_FEATURES]
CONTEXT_NAMES = {"0": "A", "1": "legacy_1", "2": "legacy_2", "3": "legacy_3", "4": "B", "5": "C"}
WEIGHT_COL = legacy.WEIGHT_COL
COLLECTION_SAMPLING = {
    "road_flow_lane_flow": "original-actual-external-entry-snapshot-including-ego",
    "platoon_context": "upstream-stopline-decision-with-fixed-crossing-proxy; first=observed-insertion",
    "same_time_order": "SUMO observation batch: vehicleID lexical; no future batch events included",
    "lane_selection": "incoming-toLane/next-fromLane intersection else next-fromLane; least shadow occupancy",
    "labels": "unchanged threshold-sweep external-road sampling; independent of F13 ledger",
}

# These helpers do not depend on the legacy feature schema. Never monkeypatch
# legacy.FEATURE_COLS/SCHEMA: old entrypoints may be imported in the same process.
file_sha256 = legacy.file_sha256
resolve_splits = legacy.resolve_splits
reject_duplicate_contents = legacy.reject_duplicate_contents
make_sample_weights = legacy.make_sample_weights


def encode_feature_frame(frame):
    """Select all nine raw inputs, keeping three fixed unordered categories."""
    if frame.columns.duplicated().any():
        raise ValueError("Duplicate input feature columns")
    missing = [column for column in FEATURE_COLS if column not in frame.columns]
    if missing:
        raise ValueError(f"Missing ABC/RG features {missing}; old five-feature CSV is incompatible")
    result = abc.encode_feature_frame(frame)
    for column in ("entry_signal_type", "next_signal_type"):
        values = frame[column]
        if values.isna().any() or not values.isin(["R", "G"]).all():
            raise ValueError(f"{column} must contain exactly R/G, never numeric, yellow or unknown")
        result[column] = pd.Categorical(values, categories=["R", "G"], ordered=False)
    for column in ("entry_signal_remaining_seconds", "next_signal_duration_seconds"):
        if pd.api.types.is_bool_dtype(frame[column].dtype):
            raise ValueError(f"{column} must be numeric seconds, not boolean")
        values = pd.to_numeric(frame[column], errors="raise").astype("float64")
        if not np.isfinite(values.to_numpy()).all() or (values < 0).any():
            raise ValueError(f"{column} must be finite nonnegative seconds")
        result[column] = values
    first, second = result["entry_signal_remaining_seconds"], result["next_signal_duration_seconds"]
    sentinel = ((result["entry_signal_type"] == "G") & (first == 0)
                & (result["next_signal_type"] == "G") & (second == 0))
    if ((~sentinel) & ((first <= 0) | (second <= 0))).any():
        raise ValueError("Only G0G0 no-exit placeholder permits zero durations; all real segments must be positive")
    return result[FEATURE_COLS]


def context_counts(frame):
    counts = frame["platoon_context"].value_counts(sort=False)
    return {str(value): int(counts.get(value, 0)) for value in ABC_CONTEXT_IDS}


def feature_summary(frame):
    summary = {}
    for column in FEATURE_COLS:
        values = frame[column]
        item = {"unique": int(values.nunique()), "constant": bool(values.nunique() == 1)}
        if column in CATEGORICAL_FEATURES:
            categories = CATEGORICAL_FEATURES[column]
            item.update(type="categorical", categories=list(categories), ordered=False,
                        observed_categories=[value for value in categories if (values == value).any()],
                        counts={str(value): int((values == value).sum()) for value in categories})
        else:
            item.update(min=float(values.min()), max=float(values.max()))
        summary[column] = item
    summary["platoon_context"]["category_names"] = dict(CONTEXT_NAMES)
    return summary


def expected_signal_contract(legacy=False, *, version=None):
    """Public stable collector/deployment semantics; no run-specific clock data."""
    expected = signal_feature_contract(legacy=legacy, version=version)
    legacy = expected["version"] == "static-movement-rg-segments-v1"
    expected.update(
        reference_time=signal_entry_reference_rule(expected["version"]),
        movement="current_external_road -> next_route_road; ABC logical fromLane; live linkIndex checked",
        anchor="observed active STATIC program/phase/absolute nextSwitch; half-open phase boundaries",
        causality="known fixed plan at decision; no future actual entry, trajectory or release information",
        supported_programs=signal_runtime_support_rule(legacy=legacy))
    return expected


def _same_rule(actual, expected):
    """JSON rule equality without treating a boolean as the number zero/one."""
    if isinstance(expected, dict):
        return (isinstance(actual, dict) and set(actual) == set(expected)
                and all(_same_rule(actual[key], value) for key, value in expected.items()))
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(actual) == len(expected)
                and all(_same_rule(a, e) for a, e in zip(actual, expected)))
    if isinstance(expected, Real) and not isinstance(expected, bool):
        return isinstance(actual, Real) and not isinstance(actual, bool) and actual == expected
    return type(actual) is type(expected) and actual == expected


def validate_signal_contract(contract):
    """Validate exact stable semantics, optionally validate per-run provenance.

    Returns a fresh stable dictionary for equality checks and catch/C++ headers.
    runtime_programs and clock_queries are provenance, not shared feature rules.
    """
    if not isinstance(contract, dict):
        raise ValueError("signal_contract must be an object")
    legacy = contract.get("version") == "static-movement-rg-segments-v1"
    expected = expected_signal_contract(version=contract.get("version"))
    horizon_keys = {"runtime_unsignalized_green_seconds", "runtime_green_horizon_movements"}
    allowed = set(expected) | {"runtime_programs", "clock_queries"}
    topology_clock = expected["version"] == "static-movement-rg-segments-v4"
    if topology_clock:
        allowed.add("runtime_network_without_internal_links")
    if not legacy:
        allowed |= horizon_keys
    if set(contract) - allowed or set(expected) - set(contract):
        raise ValueError("signal_contract missing or unexpected rule fields")
    for key, value in expected.items():
        if not _same_rule(contract[key], value):
            raise ValueError(f"signal_contract rule mismatch: {key}")
    runtime_keys = {"runtime_programs", "clock_queries"} & set(contract)
    if topology_clock and runtime_keys:
        if type(contract.get("runtime_network_without_internal_links")) is not bool:
            raise ValueError("v4 collection requires boolean runtime_network_without_internal_links")
    elif "runtime_network_without_internal_links" in contract:
        raise ValueError("network crossing provenance requires runtime programs and clock_queries")
    if runtime_keys and runtime_keys != {"runtime_programs", "clock_queries"}:
        raise ValueError("signal_contract runtime provenance must include programs and clock_queries together")
    if runtime_keys:
        queries = contract["clock_queries"]
        programs = contract["runtime_programs"]
        if type(queries) is not int or queries < 0 or not isinstance(programs, dict):
            raise ValueError("signal_contract invalid runtime clock provenance")
        for tls, program in programs.items():
            if not isinstance(tls, str) or not tls or not isinstance(program, dict):
                raise ValueError("signal_contract invalid runtime program")
            if set(program) != {"program_id", "durations", "states", "initial_cycle_origin"}:
                raise ValueError("signal_contract runtime program fields mismatch")
            durations, states = program["durations"], program["states"]
            if (not isinstance(program["program_id"], str) or not program["program_id"]
                    or not isinstance(durations, (tuple, list)) or not durations
                    or not isinstance(states, (tuple, list)) or len(states) != len(durations)):
                raise ValueError("signal_contract runtime program dimensions/ID invalid")
            for value in [*durations, program["initial_cycle_origin"]]:
                if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
                    raise ValueError("signal_contract runtime duration/origin must be finite numeric")
            if any(value <= 0 for value in durations) or not math.isfinite(sum(durations)):
                raise ValueError("signal_contract runtime duration must be positive and finite")
            if (any(not isinstance(state, str) or not state or any(c not in "rRgGyY" for c in state) for state in states)
                    or len({len(state) for state in states}) != 1):
                raise ValueError("signal_contract runtime state width/characters invalid")
    if horizon_keys & set(contract):
        if not horizon_keys.issubset(contract) or not runtime_keys:
            raise ValueError("signal_contract incomplete green-horizon provenance")
        horizon = contract["runtime_unsignalized_green_seconds"]
        movements = contract["runtime_green_horizon_movements"]
        if (isinstance(horizon, bool) or not isinstance(horizon, Real) or not math.isfinite(horizon)
                or horizon <= 0 or not isinstance(movements, dict) or not movements):
            raise ValueError("signal_contract invalid green-horizon provenance")
        measured = 0.0
        for tls, links in movements.items():
            if tls not in programs or not isinstance(links, (tuple, list)) or not links:
                raise ValueError("green-horizon movement not bound to a runtime program")
            program = programs[tls]
            for link in links:
                if type(link) is not int or not 0 <= link < len(program["states"][0]):
                    raise ValueError("invalid green-horizon movement link")
                measured = max(measured, longest_finite_green_seconds(program["durations"], [s[link] for s in program["states"]]))
        if not math.isclose(measured, horizon, rel_tol=0, abs_tol=1e-9):
            raise ValueError("green horizon disagrees with runtime movement programs")
    return copy.deepcopy(expected)


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
    if contract.get("sampling") != COLLECTION_SAMPLING:
        raise ValueError(f"{path}: collection sampling/time/label rules mismatch")
    context = validate_abc_context_contract(contract.get("context_contract"))
    signal = validate_signal_contract(contract.get("signal_contract"))
    if not {"runtime_programs", "clock_queries"}.issubset(contract["signal_contract"]):
        raise ValueError(f"{path}: missing per-run signal runtime provenance")
    digest = file_sha256(csv_path)
    if contract.get("data_sha256") != digest:
        raise ValueError(f"{path}: collection data_sha256 does not match CSV; stale or unpaired metadata")
    return context, signal, {
        "path": str(path.resolve()), "sha256": file_sha256(path),
        "data_sha256": digest, "source": contract.get("source"),
        "audit": contract.get("audit"), "sampling": contract.get("sampling"),
        "signal_contract": copy.deepcopy(contract["signal_contract"]),
    }


def validate_collection_audit(csv_path, frame, features, context, provenance):
    """Verify frozen e/d timing from the paired audit, never use it as input."""
    audit_path = Path(str(csv_path) + ".audit.csv")
    declared = provenance.get("audit")
    if not isinstance(declared, dict) or not audit_path.is_file():
        raise ValueError(f"{csv_path}: missing paired audit CSV/contract")
    digest = file_sha256(audit_path)
    if declared.get("sha256") != digest:
        raise ValueError(f"{csv_path}: audit sha256 mismatch")
    columns = META_COLS + SIGNAL_FEATURE_COLS + [
        "decision_time", "planned_enter_time", "signal_anchor_time", "signal_reference_time",
        "signal_projection_seconds", "signal_status", "signal_tls_id", "signal_program_id"]
    zero_crossing = provenance["signal_contract"]["version"] in {"static-movement-rg-segments-v3", "static-movement-rg-segments-v4"}
    if zero_crossing:
        columns += ["incoming_junction_status", "incoming_crossing_seconds"]
    try:
        audit = pd.read_csv(audit_path, usecols=columns, dtype={"vehicleID": str, "signal_program_id": str},
                            float_precision="round_trip", keep_default_na=False)
    except ValueError as exc:
        raise ValueError(f"{csv_path}: audit missing signal timing/identity columns") from exc
    if len(audit) != len(frame) or audit.duplicated(["vehicleID", "route_index"]).any():
        raise ValueError(f"{csv_path}: audit identity count/duplicates mismatch")
    for column in META_COLS:
        if not np.array_equal(frame[column].to_numpy(), audit[column].to_numpy()):
            raise ValueError(f"{csv_path}: audit row identity mismatch: {column}")
    for column in SIGNAL_FEATURE_COLS:
        if not np.array_equal(features[column].to_numpy(), audit[column].to_numpy()):
            raise ValueError(f"{csv_path}: formal/frozen audit signal mismatch: {column}")
    for column in ("decision_time", "planned_enter_time", "signal_anchor_time",
                   "signal_reference_time", "signal_projection_seconds"):
        values = pd.to_numeric(audit[column], errors="raise").to_numpy(dtype=float)
        if not np.isfinite(values).all():
            raise ValueError(f"{csv_path}: audit non-finite time: {column}")
        audit[column] = values
    decision = audit["decision_time"].to_numpy()
    planned = audit["planned_enter_time"].to_numpy()
    crossing = context["base_context_contract"]["crossing_time_seconds"]
    expected_delta = np.where(frame["route_index"].to_numpy() == 0, 0.0, crossing)
    if zero_crossing:
        incoming = audit["incoming_junction_status"]
        if (not incoming.isin(["route_start", "unsignalized_node", "signalized_node", "unknown_node"]).all()
                or not np.array_equal((incoming == "route_start").to_numpy(), frame["route_index"].to_numpy() == 0)):
            raise ValueError(f"{csv_path}: invalid incoming junction status")
        expected_delta = np.where(incoming.isin(["route_start", "unsignalized_node"]), 0.0, crossing)
        if (provenance["signal_contract"]["version"] == "static-movement-rg-segments-v4"
                and provenance["signal_contract"]["runtime_network_without_internal_links"]):
            expected_delta = np.zeros(len(frame), dtype=float)
        actual_crossing = pd.to_numeric(audit["incoming_crossing_seconds"], errors="raise").to_numpy(dtype=float)
        if not np.isfinite(actual_crossing).all() or not np.allclose(actual_crossing, expected_delta, rtol=0, atol=1e-9):
            raise ValueError(f"{csv_path}: incoming crossing does not match junction policy")
    for actual, expected, name in (
            (planned - decision, expected_delta, "planned_enter_time"),
            (audit["signal_anchor_time"], decision, "signal_anchor_time"),
            (audit["signal_reference_time"], planned, "signal_reference_time"),
            (audit["signal_projection_seconds"], expected_delta, "signal_projection_seconds")):
        if not np.allclose(actual, expected, rtol=0, atol=1e-7):
            raise ValueError(f"{csv_path}: noncausal/mismatched audit {name}")
    sentinel = ((features["entry_signal_remaining_seconds"] == 0)
                & (features["next_signal_duration_seconds"] == 0)).to_numpy()
    statuses = audit["signal_status"]
    allowed = ["controlled_static", "no_outgoing_route", "unsignalized_connection"]
    legacy = provenance["signal_contract"]["version"] == "static-movement-rg-segments-v1"
    expected_sentinel = statuses != "controlled_static" if legacy else statuses == "no_outgoing_route"
    if not statuses.isin(allowed).all() or not np.array_equal(sentinel, expected_sentinel.to_numpy()):
        raise ValueError(f"{csv_path}: no-exit placeholder/status mismatch")
    final = (frame["is_final_road"] == 1).to_numpy()
    if not np.array_equal(final, (statuses == "no_outgoing_route").to_numpy()):
        raise ValueError(f"{csv_path}: final-road/no-outgoing status mismatch")
    programs = provenance["signal_contract"]["runtime_programs"]
    unsignalized = (statuses == "unsignalized_connection").to_numpy()
    if not legacy and unsignalized.any():
        horizon = provenance["signal_contract"].get("runtime_unsignalized_green_seconds")
        if horizon is None:
            raise ValueError(f"{csv_path}: missing green-horizon provenance for uncontrolled exits")
        for column in ("entry_signal_type", "next_signal_type"):
            if not (features.loc[unsignalized, column] == "G").all():
                raise ValueError(f"{csv_path}: uncontrolled exit must be green-green")
        for column in ("entry_signal_remaining_seconds", "next_signal_duration_seconds"):
            if not np.allclose(features.loc[unsignalized, column], horizon, rtol=0, atol=1e-9):
                raise ValueError(f"{csv_path}: uncontrolled exit must use constant network green horizon")
        if (audit.loc[unsignalized, ["signal_tls_id", "signal_program_id"]] != "").any().any():
            raise ValueError(f"{csv_path}: uncontrolled exit cannot claim a physical TLS")
    for tls, program in audit.loc[statuses == "controlled_static", ["signal_tls_id", "signal_program_id"]].drop_duplicates().itertuples(index=False, name=None):
        if tls not in programs or programs[tls]["program_id"] != program:
            raise ValueError(f"{csv_path}: audited TLS/program missing from runtime provenance")
    provenance["verified_audit"] = {"path": str(audit_path.resolve()), "sha256": digest, "rows": len(audit)}
    return provenance["verified_audit"]


def load_frame(csv_path, label_column):
    """Load verified new data, never reconstruct missing cap/context from old CSV."""
    if label_column not in LABEL_COLUMNS:
        raise ValueError(f"Unsupported road-time label: {label_column}")
    header = pd.read_csv(csv_path, nrows=0)
    required = FEATURE_COLS + META_COLS + [label_column]
    missing = [column for column in required if column not in header.columns]
    if missing:
        raise ValueError(f"{csv_path}: missing cap/context full-training columns {missing}; recollect with the new collector")
    context, signal, provenance = read_collection_contract(csv_path)
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
    validate_collection_audit(csv_path, frame, features, context, provenance)
    valid = frame[label_column] > 0
    if not valid.any():
        raise ValueError(f"{csv_path}: no positive {label_column} labels remain")
    audit = {
        "path": str(Path(csv_path).resolve()), "sha256": provenance["data_sha256"],
        "collection_contract_file": provenance, "context_contract": context, "signal_contract": signal,
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
    result.attrs.update(audit=audit, context_contract=context, signal_contract=signal)
    print(f"[INPUT] {audit['path']} rows={rows_read} retained={len(result)} "
          f"final={audit['final_rows_retained']} dropped_label={audit['nonpositive_label_rows_removed']}")
    return result


def load_split(paths, label_column):
    frames, audits, shared, shared_signal = [], [], None, None
    for path in paths:
        frame = load_frame(path, label_column)
        context = frame.attrs["context_contract"]
        if shared is not None and context != shared:
            raise ValueError("Mixed collection context_contract in one split; recollect with one frozen contract")
        signal = frame.attrs["signal_contract"]
        if shared_signal is not None and signal != shared_signal:
            raise ValueError("Mixed collection signal_contract in one split")
        shared, shared_signal = context, signal
        audits.append(frame.attrs["audit"])
        frame.attrs = {}
        frames.append(frame)
    if not frames:
        raise ValueError("Empty training/validation split")
    result = pd.concat(frames, ignore_index=True)
    result.attrs.update(context_contract=shared, signal_contract=shared_signal)
    return result, audits


def shared_context_contract(train_frame, validation_frame):
    train = validate_abc_context_contract(train_frame.attrs.get("context_contract"))
    validation = validate_abc_context_contract(validation_frame.attrs.get("context_contract"))
    if train != validation:
        raise ValueError("Training/validation context_contract mismatch; time/geometry/threshold versions must agree")
    return train


def shared_signal_contract(train_frame, validation_frame):
    train = validate_signal_contract(train_frame.attrs.get("signal_contract"))
    validation = validate_signal_contract(validation_frame.attrs.get("signal_contract"))
    if train != validation:
        raise ValueError("Training/validation signal_contract mismatch")
    return train


def validation_metrics(frame, prediction, low_flow_cutoff=5):
    result = legacy.validation_metrics(frame, prediction, low_flow_cutoff)
    prediction = np.asarray(prediction, dtype=float)
    result["by_platoon_context"] = {}
    result["platoon_context_names"] = dict(CONTEXT_NAMES)
    for category in ABC_CONTEXT_IDS:
        mask = (frame["platoon_context"] == category).to_numpy()
        result["by_platoon_context"][str(category)] = (
            legacy.metric_summary(frame.loc[mask, LABEL_COL], prediction[mask]) if mask.any() else {"n": 0})
    result["by_signal_pair"] = {}
    for first in ("R", "G"):
        for second in ("R", "G"):
            mask = ((frame["entry_signal_type"] == first) & (frame["next_signal_type"] == second)).to_numpy()
            result["by_signal_pair"][first + second] = (
                legacy.metric_summary(frame.loc[mask, LABEL_COL], prediction[mask]) if mask.any() else {"n": 0})
    sentinel = (frame["entry_signal_remaining_seconds"] == 0).to_numpy()
    result["by_signal_exit"] = {
        name: legacy.metric_summary(frame.loc[mask, LABEL_COL], prediction[mask]) if mask.any() else {"n": 0}
        for name, mask in (("no_exit", sentinel), ("positive_duration_exit", ~sentinel))}
    return result


def fit_model(train_frame, validation_frame, args, provenance):
    output = Path(args.save_path).expanduser().resolve()
    if output.exists():
        raise ValueError(f"Output already exists; choose a new --save-path: {output}")
    validate_options(args)
    context = shared_context_contract(train_frame, validation_frame)
    signal = shared_signal_contract(train_frame, validation_frame)
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
        "context_contract": context, "signal_contract": signal, "categorical_features": CATEGORICAL_FEATURES,
        "categorical_ordered": False, "trainer_sha256": file_sha256(__file__),
        "trainer_dependencies": {
            name: {"path": str(source_path(name).resolve()),
                   "sha256": file_sha256(source_path(name))}
            for name in ("model_training_sumo_sf.py", "model_training_sumo_cap_context_abc.py",
                         "cap_context_features.py", "cap_context_abc_features.py", "signal_rg_features.py")
        },
        "python_version": sys.version, "dependency_versions": dependencies,
        "units": {"road_length": "m", "free_speed_cap": "m/s", "platoon_context": "categorical",
                  "entry_signal_type": "categorical", "next_signal_type": "categorical",
                  "entry_signal_remaining_seconds": "s", "next_signal_duration_seconds": "s"},
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
        contract["framework_removed_raw_features"] = [name for name in FEATURE_COLS if name not in used]
        if contract["framework_removed_raw_features"]:
            print("[WARN] Framework removed raw features: "
                  + repr(contract["framework_removed_raw_features"])
                  + "; all nine remain declared and are passed to predict")
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
    print("[NOTICE] Requires ABC/RG nine-feature catcher and C++ reader; old five-feature catchers are incompatible.")
    return predictor


def build_parser():
    parser = legacy.build_parser()
    parser.description = __doc__
    parser.epilog = "Default target: travel_time_label_gt_1. No six-threshold training unless --all-thresholds."
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
        (None, args.label_column or DEFAULT_LABEL_COLUMN)]
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
        print(f"[CONFIG] schema={SCHEMA} label={label} raw9={FEATURE_COLS} weighting={run.sample_weighting}")
        print(f"[CONFIG] split={split_kind} train_files={len(train_paths)} validation_files={len(validation_paths)}")
        train_frame, train_audits = load_split(train_paths, label)
        validation_frame, validation_audits = load_split(validation_paths, label)
        context = shared_context_contract(train_frame, validation_frame)
        signal = shared_signal_contract(train_frame, validation_frame)
        reject_duplicate_contents(train_audits, validation_audits)
        _, weight_audit = make_sample_weights(train_frame, run)
        print("[WEIGHTS] " + json.dumps(weight_audit, ensure_ascii=False))
        observed = set(int(value) for value in train_frame["platoon_context"].unique())
        missing = sorted(set(ABC_CONTEXT_IDS) - observed)
        if missing:
            print(f"[WARN] training lacks platoon_context categories {missing}; retained as declared categories, no coverage claim")
        for column in NUMERIC_FEATURES:
            lo, hi = train_frame[column].min(), train_frame[column].max()
            outside = int(((validation_frame[column] < lo) | (validation_frame[column] > hi)).sum())
            if outside:
                print(f"[WARN] validation {column}: {outside} rows outside training min/max")
        unseen = sorted(set(int(value) for value in validation_frame["platoon_context"].unique()) - observed)
        if unseen:
            print(f"[WARN] validation platoon_context categories unseen in training: {unseen}")
        for column in ("entry_signal_type", "next_signal_type"):
            unseen_types = sorted(set(validation_frame[column].unique()) - set(train_frame[column].unique()))
            if unseen_types:
                print(f"[WARN] validation {column} categories unseen in training: {unseen_types}")
        print("[CATEGORIES] " + json.dumps({
            "names": CONTEXT_NAMES, "ordered": False,
            "train": context_counts(train_frame),
            "validation": context_counts(validation_frame),
        }, ensure_ascii=False))
        provenance = {
            "split_kind": split_kind, "train_files": train_audits, "validation_files": validation_audits,
            "entrypoint": {"path": str(Path(__file__).resolve()), "sha256": file_sha256(__file__)},
        }
        if args.dataset_dir:
            manifest = Path(args.dataset_dir) / "commands" / "traci_jobs.tsv"
            provenance["manifest"] = {"path": str(manifest.resolve()), "sha256": file_sha256(manifest)}
        result = {"source_label": label, "save_path": run.save_path, "train_rows": len(train_frame),
                  "validation_rows": len(validation_frame), "context_contract": context, "signal_contract": signal,
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
