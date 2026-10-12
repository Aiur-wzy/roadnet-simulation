#!/usr/bin/env python3
"""Export ABC six-state cap/context models to independently versioned exact RLE.

Only load trusted AutoGluon models. --dry-run validates the model contract and
network without importing AutoGluon or creating any output. No true fit occurs.
The old four-state exporter and C++ format are intentionally not reused: an ABC
table requires a reader implementing its six-state, nested ABC feature contract.
"""
from __future__ import annotations

import argparse
from array import array
from contextlib import ExitStack
from decimal import Decimal, ROUND_CEILING, ROUND_FLOOR, ROUND_HALF_UP
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd

from features.cap_context_features import FEATURE_COLS
from features.cap_context_abc_features import (
    ABC_FEATURE_SCHEMA, ABC_CONTEXT_IDS, validate_abc_context_contract,
)
from catching._shared.model_catching_sumo_sf import (KEY_SCALE, THRESHOLDS, batches, network_pairs,
                                    positive_number, quantize_positive, sha256_file)

TABLE_MAGIC = "CAMS_CAP_CONTEXT_ABC_RLE_V1"
FEATURE_SCHEMA = TRAINING_SCHEMA = ABC_FEATURE_SCHEMA
CONTEXT_COUNT = len(ABC_CONTEXT_IDS)
CATEGORICAL_FEATURES = {"platoon_context": list(ABC_CONTEXT_IDS)}
CONTEXT_NAMES = {"0": "A", "1": "legacy_1", "2": "legacy_2", "3": "legacy_3", "4": "B", "5": "C"}
LABEL_COL = "travel_time_no_waiting"
PAYLOAD_LAYOUT = "offsets_u32_le+records_u16_u16_f64_le-v1"
RUN = struct.Struct("<HHd")
MAX_CPP_ROWS = 50_000_000


def validate_context_contract(value):
    """Preserve the exporter API, but never accept an old four-state contract."""
    return validate_abc_context_contract(value)


def _require_context_categories(value, name):
    if (type(value) is not list or any(type(item) is not int for item in value)
            or value != list(ABC_CONTEXT_IDS)):
        raise ValueError(f"{name} must contain the six unordered ABC IDs 0..5")


def validate_training_categories(contract):
    """Reject numeric/ordered, five-state and old four-state training contracts."""
    categorical = contract.get("categorical_features")
    if type(categorical) is not dict or set(categorical) != {"platoon_context"}:
        raise ValueError("categorical_features must declare only platoon_context")
    _require_context_categories(categorical["platoon_context"], "categorical_features.platoon_context")
    if contract.get("categorical_ordered") is not False:
        raise ValueError("categorical_ordered must be explicitly false")
    ranges = contract.get("training_features")
    if type(ranges) is not dict:
        raise ValueError("Missing/invalid training_features")
    context = ranges.get("platoon_context")
    if type(context) is not dict or context.get("type") != "categorical" or context.get("ordered") is not False:
        raise ValueError("training_features.platoon_context must be explicitly unordered categorical")
    _require_context_categories(context.get("categories"), "training_features.platoon_context.categories")
    if context.get("category_names") != CONTEXT_NAMES:
        raise ValueError("training_features.platoon_context.category_names differs from the ABC mapping")
    observed = context.get("observed_categories")
    if (type(observed) is not list or not observed
            or any(type(item) is not int or item not in ABC_CONTEXT_IDS for item in observed)
            or observed != sorted(set(observed))):
        raise ValueError("training_features.platoon_context.observed_categories must be a nonempty unique sorted ABC subset")


def feature_frame(rows):
    """Match ABC training's raw5 values, dtypes and complete unordered categories."""
    frame = pd.DataFrame(rows, columns=FEATURE_COLS)
    for column in FEATURE_COLS:
        if (pd.api.types.is_bool_dtype(frame[column].dtype)
                or (pd.api.types.is_object_dtype(frame[column].dtype)
                    and any(isinstance(value, (bool, np.bool_)) for value in frame[column]))):
            raise ValueError(f"{column} must be numeric, not boolean")
        frame[column] = pd.to_numeric(frame[column], errors="raise")
        if not np.isfinite(frame[column].to_numpy(dtype=float)).all():
            raise ValueError(f"{column} contains missing/non-finite values")
    for column in ("road_length", "free_speed_cap"):
        if (frame[column] <= 0).any():
            raise ValueError(f"{column} must be strictly positive")
        frame[column] = frame[column].astype("float64")
    for column in ("road_flow", "lane_flow", "platoon_context"):
        values = frame[column].to_numpy(dtype=float)
        if (values < 0).any() or (values != np.floor(values)).any():
            raise ValueError(f"{column} must be a nonnegative integer")
    if (frame["lane_flow"] > frame["road_flow"]).any():
        raise ValueError("lane_flow exceeds road_flow")
    if not frame["platoon_context"].isin(ABC_CONTEXT_IDS).all():
        raise ValueError("ABC platoon_context must be exactly one of 0, 1, 2, 3, 4, 5")
    for column in ("road_flow", "lane_flow"):
        if (frame[column].to_numpy(dtype=float) >= 2**63).any():
            raise ValueError(f"{column} exceeds the signed 64-bit integer domain")
        frame[column] = frame[column].astype("int64")
    frame["platoon_context"] = pd.Categorical(
        frame["platoon_context"].astype(int), categories=list(ABC_CONTEXT_IDS), ordered=False)
    return frame


def resolve_model(model_dir, expected_threshold=None):
    supplied = Path(model_dir).expanduser().resolve(strict=True)
    run = supplied.parent if supplied.name == "predictor" else supplied
    contract_path = run / "training_contract.json"
    contract = json.loads(contract_path.read_text(encoding="utf-8-sig"),
                          object_pairs_hook=_unique_json,
                          parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    if not isinstance(contract, dict):
        raise ValueError("Training contract must be a JSON object")
    if contract.get("schema") != FEATURE_SCHEMA or contract.get("raw_features") != FEATURE_COLS:
        raise ValueError("Expected ABC six-state cap/context five-feature training schema")
    if contract.get("status") != "complete":
        raise ValueError("Only a successfully completed training contract can be exported")
    if contract.get("model_label") != LABEL_COL or contract.get("predictor_directory") != "predictor":
        raise ValueError("Training label/predictor_directory mismatch")
    label = contract.get("source_label")
    if label not in ["travel_time_label_gt_" + token for token in THRESHOLDS]:
        raise ValueError("Unsupported source label")
    if expected_threshold and label != "travel_time_label_gt_" + expected_threshold:
        raise ValueError("Requested threshold differs from model")
    validate_context_contract(contract.get("context_contract"))
    validate_training_categories(contract)
    actual = contract.get("framework_features")
    if (not isinstance(actual, list) or not actual or any(type(column) is not str for column in actual)
            or len(set(actual)) != len(actual) or not set(actual).issubset(FEATURE_COLS)):
        raise ValueError("Missing/invalid completed-fit framework_features")
    for column in FEATURE_COLS:
        info = contract.get("training_features", {}).get(column, {})
        if (type(info) is not dict or any(type(info.get(bound)) not in (int, float)
                                         for bound in ("min", "max"))):
            raise ValueError(f"Training range {column} requires JSON numbers, not bool/string")
        try:
            lo, hi = float(info["min"]), float(info["max"])
        except (TypeError, ValueError, KeyError, OverflowError) as exc:
            raise ValueError(f"Missing/invalid training range {column}") from exc
        if not math.isfinite(lo) or not math.isfinite(hi) or lo < 0 or lo > hi:
            raise ValueError(f"Invalid range {column}")
        if column in ("road_length", "free_speed_cap") and lo <= 0:
            raise ValueError(f"Nonpositive range {column}")
        if column in ("road_flow", "lane_flow", "platoon_context") and (lo != int(lo) or hi != int(hi)):
            raise ValueError(f"Noninteger range {column}")
        if column == "platoon_context" and (lo < min(ABC_CONTEXT_IDS) or hi > max(ABC_CONTEXT_IDS)):
            raise ValueError("Context range outside 0..5")
    predictor_path = run / "predictor"
    if not (predictor_path / "predictor.pkl").is_file():
        raise ValueError(f"Missing predictor.pkl in {predictor_path}")
    return predictor_path, contract_path, contract


def cap_grid(step, lower, upper):
    step_d = Decimal(str(positive_number(step, "cap-step")))
    lo = Decimal(str(positive_number(lower, "cap-min")))
    hi = Decimal(str(positive_number(upper, "cap-max")))
    if lo > hi:
        raise ValueError("cap-min exceeds cap-max")

    def snapped_ratio(value):
        ratio = value / step_d
        nearest = ratio.to_integral_value(rounding=ROUND_HALF_UP)
        return nearest if abs(ratio - nearest) <= Decimal("1e-9") else ratio

    first = int(snapped_ratio(lo).to_integral_value(rounding=ROUND_FLOOR))
    last = int(snapped_ratio(hi).to_integral_value(rounding=ROUND_CEILING))
    if first < 1 or last > 2**31 - 1:
        raise ValueError("Cap grid includes zero or exceeds index range")
    return {"index_min": first, "index_max": last, "step": float(step_d),
            "min": float(first * step_d), "max": float(last * step_d),
            "scaled_boundary_tolerance": 1e-9, "raw_boundary_tolerance": 1e-12}


def cap_index(value, grid):
    value = positive_number(value, "free_speed_cap")
    if value < grid["min"] - 1e-12 or value > grid["max"] + 1e-12:
        raise ValueError("free_speed_cap outside table domain; no clamping")
    index = math.floor(value / grid["step"] + 0.5 + 1e-9)
    if not grid["index_min"] <= index <= grid["index_max"]:
        raise ValueError("Cap index outside table domain")
    return index


def count_rows(length_count, grid, road_max, lane_max):
    if not 0 <= lane_max <= road_max <= 65535:
        raise ValueError("Require 0 <= max-lane-flow <= max-road-flow <= 65535")
    flows = (lane_max + 1) * (road_max + 1) - lane_max * (lane_max + 1) // 2
    return length_count * (grid["index_max"] - grid["index_min"] + 1) * CONTEXT_COUNT * flows


def validate_export_network(path):
    """Use the new collector/runtime scope, not the legacy export tolerance."""
    for edge in ET.parse(path).getroot().findall("edge"):
        if edge.get("function") == "internal" or edge.get("id", "").startswith(":"):
            continue
        lanes = edge.findall("lane")
        if not lanes:
            raise ValueError(f"External road has no lanes: {edge.get('id')}")
        for attr in ("length", "speed"):
            try:
                values = [positive_number(lane.attrib[attr], attr) for lane in lanes]
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"Invalid road lane {attr}: {edge.get('id')}") from exc
            if max(values) - min(values) > 1e-9:
                raise ValueError(f"cap/context requires uniform lane {attr}: {edge.get('id')}")


def build_plan(args):
    if args.chunk_size < 1 or not 1 <= args.max_rows <= MAX_CPP_ROWS:
        raise ValueError("chunk-size must be positive; max-rows must be in 1..50000000")
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError("Output exists; choose a new directory (no overwrite/resume)")
    predictor_path, contract_path, contract = resolve_model(args.model_dir, args.expected_threshold)
    net = Path(args.net_file).expanduser().resolve(strict=True)
    validate_export_network(net)
    pairs, edge_count = network_pairs(net)
    lengths = sorted(set(length for length, _ in pairs))
    if len({quantize_positive(value) for value in lengths}) != len(lengths):
        raise ValueError("Road length key collision")
    ranges = contract["training_features"]
    grid = cap_grid(args.cap_step,
                    args.cap_min if args.cap_min is not None else ranges["free_speed_cap"]["min"],
                    args.cap_max if args.cap_max is not None else ranges["free_speed_cap"]["max"])
    rows = count_rows(len(lengths), grid, args.max_road_flow, args.max_lane_flow)
    if rows > args.max_rows:
        raise ValueError(f"Planned {rows} rows exceeds --max-rows {args.max_rows}")
    cap_count = grid["index_max"] - grid["index_min"] + 1
    flow_count = rows // (len(lengths) * cap_count * CONTEXT_COUNT)
    warnings = []
    missing_contexts = sorted(set(ABC_CONTEXT_IDS) - set(ranges["platoon_context"]["observed_categories"]))
    if missing_contexts:
        warnings.append(f"Unobserved training platoon_context categories {missing_contexts}; "
                        "exporting all six categories preserves the schema but does not establish accuracy")
    for length in lengths:
        if not float(ranges["road_length"]["min"]) - 1e-8 <= length <= float(ranges["road_length"]["max"]) + 1e-8:
            if not args.allow_static_extrapolation:
                raise ValueError(f"Network road_length {length} outside training range")
            warnings.append(f"Network road_length {length} outside training range")
    for column, domain in {"road_flow": (0, args.max_road_flow),
                           "lane_flow": (0, args.max_lane_flow),
                           "free_speed_cap": (grid["min"], grid["max"]),
                           "platoon_context": (min(ABC_CONTEXT_IDS), max(ABC_CONTEXT_IDS))}.items():
        lo, hi = float(ranges[column]["min"]), float(ranges[column]["max"])
        if domain[0] < lo - 1e-12 or domain[1] > hi + 1e-12:
            warnings.append(f"Grid {column}={domain} extends training min/max [{lo},{hi}]; coverage is not accuracy")
        if domain[0] > lo + 1e-12 or domain[1] < hi - 1e-12:
            warnings.append(f"Grid {column} does not cover entire training min/max")
    return {"format": TABLE_MAGIC, "training_schema": FEATURE_SCHEMA,
            "columns": FEATURE_COLS + [LABEL_COL], "source_label": contract["source_label"],
            "road_lengths": lengths, "static_key_scale": KEY_SCALE,
            "free_speed_cap_grid": grid, "context_contract": contract["context_contract"],
            "context_count": CONTEXT_COUNT,
            "categorical_features": CATEGORICAL_FEATURES,
            "categorical_ordered": False,
            "road_flow_max": args.max_road_flow, "lane_flow_max": args.max_lane_flow,
            "flow_count": flow_count, "expanded_rows": rows,
            "group_count": len(lengths) * cap_count * CONTEXT_COUNT * (args.max_lane_flow + 1),
            "prediction_floor_seconds": 1.0, "payload_layout": PAYLOAD_LAYOUT,
            "network": {"path": str(net), "sha256": sha256_file(net), "external_edges": edge_count},
            "model": {"predictor_path": str(predictor_path), "contract_path": str(contract_path),
                      "contract_sha256": sha256_file(contract_path),
                      "predictor_pickle_sha256": sha256_file(predictor_path / "predictor.pkl"),
                      "framework_features": contract["framework_features"],
                      "dependency_versions": contract.get("dependency_versions", {})},
            "exporter_sha256": sha256_file(__file__), "training_ranges": ranges, "warnings": warnings}


def iter_cases(plan):
    grid = plan["free_speed_cap_grid"]
    for length in plan["road_lengths"]:
        for index in range(grid["index_min"], grid["index_max"] + 1):
            for context in ABC_CONTEXT_IDS:
                for lane in range(plan["lane_flow_max"] + 1):
                    for road in range(lane, plan["road_flow_max"] + 1):
                        yield (length, road, lane, index * grid["step"], context)


def validate_predictor(predictor, plan):
    if predictor.label != LABEL_COL or predictor.problem_type != "regression":
        raise ValueError("Predictor label/problem_type mismatch")
    actual = list(predictor.feature_metadata.get_features())
    if set(actual) != set(plan["model"]["framework_features"]) or not set(actual).issubset(FEATURE_COLS):
        raise ValueError("Predictor feature schema mismatch")
    # Input metadata, when available, is the authoritative pre-transform dtype.
    # The generated feature metadata may legitimately convert a binary category
    # to bool or drop a constant, so do not confuse such transforms with numeric
    # training input. Fallback category checks apply only without input metadata.
    metadata = getattr(predictor, "feature_metadata_in", None)
    input_metadata = metadata is not None
    if metadata is None:
        metadata = predictor.feature_metadata
    metadata_features = list(metadata.get_features())
    if "platoon_context" in metadata_features:
        getter = getattr(metadata, "get_feature_type_raw", None)
        raw_types = getattr(metadata, "type_map_raw", None)
        if callable(getter):
            raw_type = getter("platoon_context")
        elif isinstance(raw_types, dict):
            raw_type = raw_types.get("platoon_context")
        else:
            raw_type = None
        if raw_type is not None and raw_type != "category":
            scope = "input" if input_metadata else "framework"
            raise ValueError(f"Predictor {scope} platoon_context is not categorical: {raw_type!r}")


def export_table(args, plan, predictor=None):
    """Dependency injection is for mock tests; CLI always loads a real predictor."""
    if predictor is None:
        from autogluon.tabular import TabularPredictor
        predictor = TabularPredictor.load(plan["model"]["predictor_path"])
    validate_predictor(predictor, plan)
    output = Path(args.output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    run_path = output / "records.bin.partial"
    dense_path = output / "reference.f64.partial"
    offsets = array("I")
    if offsets.itemsize != 4:
        raise RuntimeError("This writer requires 32-bit unsigned array elements")
    count = run_count = clipped = 0
    raw_min, raw_max = math.inf, -math.inf
    previous_key = None
    run_lo = run_hi = None
    previous_value = None
    dense_stream = dense_path.open("xb") if args.dense_reference else None
    dense_hash = hashlib.sha256()
    try:
        with run_path.open("xb") as stream:
            for batch in batches(iter_cases(plan), args.chunk_size):
                frame = feature_frame(batch)
                raw = np.asarray(predictor.predict(frame), dtype=np.float64)
                if raw.shape != (len(frame),) or not np.isfinite(raw).all():
                    raise ValueError("Predictor returned nonfinite/wrong-shape values")
                raw_min, raw_max = min(raw_min, float(raw.min())), max(raw_max, float(raw.max()))
                clipped += int((raw < 1.0).sum())
                values = np.maximum(raw, 1.0)
                dense_bytes = np.asarray(values, dtype="<f8").tobytes()
                dense_hash.update(dense_bytes)
                if dense_stream:
                    dense_stream.write(dense_bytes)
                for row, value in zip(batch, values):
                    length, road, lane, cap, context = row
                    key = (length, cap, context, lane)
                    value = float(value)
                    if key != previous_key:
                        if previous_key is not None:
                            stream.write(RUN.pack(run_lo, run_hi, previous_value))
                            run_count += 1
                        offsets.append(run_count)
                        previous_key, run_lo, run_hi, previous_value = key, road, road, value
                    elif value == previous_value:
                        run_hi = road
                    else:
                        stream.write(RUN.pack(run_lo, run_hi, previous_value))
                        run_count += 1
                        run_lo = run_hi = road
                        previous_value = value
                    count += 1
            if previous_key is not None:
                stream.write(RUN.pack(run_lo, run_hi, previous_value))
                run_count += 1
        offsets.append(run_count)
    finally:
        if dense_stream:
            dense_stream.close()
    if count != plan["expanded_rows"] or len(offsets) != plan["group_count"] + 1:
        raise ValueError("Generated grid count mismatch; partial artifacts retained")
    if sys.byteorder != "little":
        offsets.byteswap()
    offset_bytes = offsets.tobytes()
    digest = hashlib.sha256(offset_bytes)
    with run_path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    header = dict(plan, run_count=run_count,
                  payload_bytes=len(offset_bytes) + RUN.size * run_count,
                  payload_sha256=digest.hexdigest())
    header_bytes = json.dumps(header, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(header_bytes) > 1024 * 1024:
        raise ValueError("Header exceeds 1MiB limit")
    partial = output / "table.rle.partial"
    with partial.open("xb") as stream:
        stream.write(TABLE_MAGIC.encode("ascii") + b"\n" + header_bytes + b"\n")
        stream.write(offset_bytes)
        with run_path.open("rb") as records:
            for block in iter(lambda: records.read(1024 * 1024), b""):
                stream.write(block)
    # Independently decode all runs without allocating a second dense table.
    loaded_header, expanded_sha = scan_table(partial)
    if expanded_sha != dense_hash.hexdigest():
        raise ValueError("RLE round-trip changed dense float64 outputs")
    final = output / "table.rle"
    partial.rename(final)
    if args.dense_reference:
        dense_path.rename(output / "reference.f64")
    run_path.unlink()  # Only this newly generated, explicitly named spool file.
    manifest = {"status": "complete", "table": "table.rle", "table_sha256": sha256_file(final),
                "rows": count, "run_count": run_count, "file_bytes": final.stat().st_size,
                "dense_value_bytes": count * 8, "dense_sha256": dense_hash.hexdigest(),
                "raw_prediction_min": raw_min, "raw_prediction_max": raw_max,
                "predictions_floored_to_one_second": clipped,
                "roundtrip_exact": True, "plan": loaded_header}
    text_format = getattr(args, "text_format", "none")
    if text_format != "none":
        # Read the finalized RLE, never predict a second time for a text output.
        manifest["readable_tables"] = write_readable_tables(
            final, output, text_format, expected_dense_sha=dense_hash.hexdigest())
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(f"[DONE] {final} values={count} runs={run_count} exact_roundtrip=true")
    return manifest


def _unique_json(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key {key}")
        result[key] = value
    return result


def _json_number(mapping, key):
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"Header {key} must be a finite JSON number, not bool/string")
    return value


def _decode_table(path, *, materialize=False, on_run=None):
    """Shared strict decoder; streaming mode retains no expanded value array.

    The payload hash is checked before callbacks. Offsets and records use
    separate read handles so neither complete payload nor offsets are buffered.
    A callback receives (header, group, inclusive_low, inclusive_high, value).
    Inputs must not be modified concurrently. A complete-file hash recheck also
    detects persistent changes between validation and decoding passes.
    """
    with Path(path).open("rb") as stream:
        magic = stream.readline(100)
        if magic != TABLE_MAGIC.encode() + b"\n":
            raise ValueError("Wrong table magic")
        line = stream.readline(1024 * 1024 + 2)
        if len(line) > 1024 * 1024 + 1 or not line.endswith(b"\n"):
            raise ValueError("Oversized/unterminated header")
        header = json.loads(line, object_pairs_hook=_unique_json,
                            parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        if type(header) is not dict:
            raise ValueError("Table header must be a JSON object")
        validate_context_contract(header.get("context_contract"))
        if header.get("format") != TABLE_MAGIC or header.get("training_schema") != FEATURE_SCHEMA or header.get("columns") != FEATURE_COLS + [LABEL_COL]:
            raise ValueError("Header feature/format schema mismatch")
        if header.get("payload_layout") != PAYLOAD_LAYOUT or header.get("source_label") not in ["travel_time_label_gt_" + t for t in THRESHOLDS]:
            raise ValueError("Unsupported payload/label")
        if (type(header.get("context_count")) is not int
                or header["context_count"] != CONTEXT_COUNT):
            raise ValueError("ABC context_count must be integer 6")
        if header.get("categorical_features") != CATEGORICAL_FEATURES:
            raise ValueError("Header categorical_features differs from ABC categories")
        _require_context_categories(header["categorical_features"]["platoon_context"], "header ABC categories")
        if header.get("categorical_ordered") is not False:
            raise ValueError("Header categorical_ordered must be explicitly false")
        if _json_number(header, "static_key_scale") != KEY_SCALE or _json_number(header, "prediction_floor_seconds") != 1.0:
            raise ValueError("Unsupported constants")
        for key in ("road_flow_max", "lane_flow_max", "expanded_rows", "group_count", "run_count", "flow_count", "payload_bytes"):
            if type(header.get(key)) is not int or header[key] < 0:
                raise ValueError(f"Invalid integer header {key}")
        lengths = header.get("road_lengths", [])
        if type(lengths) is not list or not lengths or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in lengths):
            raise ValueError("Invalid road lengths")
        keys = [quantize_positive(v) for v in lengths]
        if keys != sorted(set(keys)):
            raise ValueError("Unsorted/duplicate length keys")
        if any(not math.isclose(v, key / KEY_SCALE, rel_tol=1e-10, abs_tol=1e-10)
               for v, key in zip(lengths, keys)):
            raise ValueError("Road length is not a four-decimal static key")
        grid = header.get("free_speed_cap_grid")
        if type(grid) is not dict:
            raise ValueError("free_speed_cap_grid must be a JSON object")
        if any(type(grid.get(key)) is not int for key in ("index_min", "index_max")):
            raise ValueError("Noninteger cap indices")
        if not 1 <= grid["index_min"] <= grid["index_max"] <= 2**31 - 1:
            raise ValueError("Cap indices out of bounds")
        step = positive_number(_json_number(grid, "step"), "cap step")
        for key, index in (("min", grid["index_min"]), ("max", grid["index_max"])):
            value = positive_number(_json_number(grid, key), "cap bound")
            if not math.isclose(value, index * step, rel_tol=1e-10, abs_tol=1e-10):
                raise ValueError("Cap bound does not match lattice")
        if _json_number(grid, "scaled_boundary_tolerance") != 1e-9 or _json_number(grid, "raw_boundary_tolerance") != 1e-12:
            raise ValueError("Unsupported rounding tolerances")
        rows = count_rows(len(lengths), grid, header["road_flow_max"], header["lane_flow_max"])
        caps = grid["index_max"] - grid["index_min"] + 1
        groups = len(lengths) * caps * CONTEXT_COUNT * (header["lane_flow_max"] + 1)
        if not 1 <= rows <= MAX_CPP_ROWS or rows != header["expanded_rows"] or groups != header["group_count"]:
            raise ValueError("Invalid dimensions")
        if header["flow_count"] != rows // (len(lengths) * caps * CONTEXT_COUNT):
            raise ValueError("Flow count mismatch")
        runs = header["run_count"]
        expected = 4 * (groups + 1) + 12 * runs
        if not groups <= runs <= rows or header["payload_bytes"] != expected:
            raise ValueError("Invalid run count/bytes")
        payload_start = stream.tell()
        digest, remaining = hashlib.sha256(), expected
        file_digest = hashlib.sha256(magic + line)
        while remaining:
            block = stream.read(min(1024 * 1024, remaining))
            if not block:
                raise ValueError("Truncated/extra payload")
            digest.update(block)
            file_digest.update(block)
            remaining -= len(block)
        if stream.read(1):
            raise ValueError("Truncated/extra payload")
        if digest.hexdigest() != header.get("payload_sha256"):
            raise ValueError("Payload SHA256 mismatch")
    dense = np.empty(rows, dtype="<f8") if materialize else None
    position, last_offset, dense_hash = 0, 0, hashlib.sha256()
    with Path(path).open("rb") as indices, Path(path).open("rb") as records:
        indices.seek(payload_start)
        records.seek(payload_start + 4 * (groups + 1))
        if indices.read(4) != b"\0\0\0\0":
            raise ValueError("Bad group offsets")
        for group in range(groups):
            offset_bytes = indices.read(4)
            if len(offset_bytes) != 4:
                raise ValueError("Truncated group offsets")
            offset = struct.unpack("<I", offset_bytes)[0]
            if not last_offset < offset <= runs:
                raise ValueError("Bad group offsets")
            lane = group % (header["lane_flow_max"] + 1)
            next_road, previous = lane, None
            for record in range(last_offset, offset):
                packed = records.read(RUN.size)
                if len(packed) != RUN.size:
                    raise ValueError("Truncated run record")
                lo, hi, value = RUN.unpack(packed)
                if lo != next_road or hi < lo or hi > header["road_flow_max"]:
                    raise ValueError("Run gap/overlap/out-of-range")
                if not math.isfinite(value) or value < 1.0 or previous == value:
                    raise ValueError("Invalid/noncanonical run value")
                size = hi - lo + 1
                if dense is not None:
                    dense[position:position + size] = value
                # At most 65536 float64 values (512 KiB) per run.
                dense_hash.update(struct.pack("<d", value) * size)
                if on_run is not None:
                    on_run(header, group, lo, hi, value)
                position += size
                next_road, previous = hi + 1, value
            if next_road != header["road_flow_max"] + 1:
                raise ValueError("Incomplete group coverage")
            last_offset = offset
        if last_offset != runs or records.read(1):
            raise ValueError("Bad final offset/extra payload")
    if position != rows:
        raise ValueError("Decoded count mismatch")
    if sha256_file(path) != file_digest.hexdigest():
        raise ValueError("Source RLE changed during strict decoding")
    return header, dense, dense_hash.hexdigest()


def read_table(path):
    """Strict reference decoder with the existing (header, dense) interface."""
    header, dense, _ = _decode_table(path, materialize=True)
    return header, dense


def scan_table(path, on_run=None):
    """Strict bounded-memory decode returning (header, expanded_float64_sha256)."""
    header, _, digest = _decode_table(path, on_run=on_run)
    return header, digest


def _run_keys(header, group):
    lanes = header["lane_flow_max"] + 1
    caps = header["free_speed_cap_grid"]["index_max"] - header["free_speed_cap_grid"]["index_min"] + 1
    lane = group % lanes
    context_block = group // lanes
    context = ABC_CONTEXT_IDS[context_block % CONTEXT_COUNT]
    static_cap = context_block // CONTEXT_COUNT
    cap_index_value = header["free_speed_cap_grid"]["index_min"] + static_cap % caps
    length = header["road_lengths"][static_cap // caps]
    cap = cap_index_value * header["free_speed_cap_grid"]["step"]
    return length, lane, cap, context


def validate_readable_table(path, rle_path, text_format, expected_dense_sha=None):
    """Parse every six-column row and compare all float64 bits against the RLE.

    Python float parsing implements round-trip decimal conversion. A spreadsheet
    application's display/import precision is not covered by this validation.
    """
    if text_format not in ("txt", "csv"):
        raise ValueError("Readable format must be txt or csv")
    separator = " " if text_format == "txt" else ","
    count, row_hash, time_hash = 0, hashlib.sha256(), hashlib.sha256()
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        if stream.readline().rstrip("\r\n").split(separator) != FEATURE_COLS + [LABEL_COL]:
            raise ValueError("Readable table column header mismatch")

        def compare(header, group, lo, hi, value):
            nonlocal count
            length, lane, cap, context = _run_keys(header, group)
            for road in range(lo, hi + 1):
                parts = stream.readline().rstrip("\r\n").split(separator)
                if len(parts) != 6:
                    raise ValueError(f"Readable row {count} missing/malformed")
                try:
                    actual = [float(item) for item in parts]
                except ValueError as exc:
                    raise ValueError(f"Readable row {count} not numeric") from exc
                expected = struct.pack("<6d", length, road, lane, cap, context, value)
                actual_bytes = struct.pack("<6d", *actual)
                if actual_bytes != expected:
                    raise ValueError(f"Readable row {count} changes keys or float64 value")
                row_hash.update(actual_bytes)
                time_hash.update(actual_bytes[-8:])
                count += 1

        header, dense_sha = scan_table(rle_path, compare)
        if stream.read(1):
            raise ValueError("Readable table has extra rows/data")
    if count != header["expanded_rows"] or time_hash.hexdigest() != dense_sha:
        raise ValueError("Readable count/dense SHA mismatch")
    if expected_dense_sha is not None and dense_sha != expected_dense_sha:
        raise ValueError("Readable table differs from expected RLE dense SHA")
    return {"rows": count, "all_columns_float64_roundtrip_exact": True,
            "row_float64_sha256": row_hash.hexdigest(), "dense_sha256": dense_sha}


def write_readable_tables(rle_path, output_dir, text_format, expected_dense_sha=None):
    """Create new TXT/CSV alongside RLE or in an exclusive conversion directory.

    This helper requires its output directory to exist, refuses all existing
    target/partial files, and never marks a manifest complete itself.
    """
    formats = ["txt", "csv"] if text_format == "both" else [text_format]
    if any(fmt not in ("txt", "csv") for fmt in formats):
        raise ValueError("text-format must be txt, csv or both")
    rle_path, output = Path(rle_path), Path(output_dir)
    source_sha = sha256_file(rle_path)
    header, dense_sha = scan_table(rle_path)
    if expected_dense_sha is not None and expected_dense_sha != dense_sha:
        raise ValueError("Source RLE differs from expected dense SHA")
    targets = {fmt: output / ("table." + fmt) for fmt in formats}
    partials = {fmt: output / ("table." + fmt + ".partial") for fmt in formats}
    if any(path.exists() for path in list(targets.values()) + list(partials.values())):
        raise ValueError("Readable output already exists; no overwrite/resume")
    with ExitStack() as stack:
        streams = {fmt: stack.enter_context(partials[fmt].open("x", encoding="utf-8", newline=""))
                   for fmt in formats}
        for fmt, stream in streams.items():
            stream.write(("," if fmt == "csv" else " ").join(FEATURE_COLS + [LABEL_COL]) + "\n")

        def emit(header, group, lo, hi, value):
            length, lane, cap, context = _run_keys(header, group)
            prefix = format(float(length), ".17g")
            suffix = [str(lane), format(float(cap), ".17g"), str(context), format(value, ".17g")]
            # Chunk even unusually long runs; no complete table/DataFrame in RAM.
            for begin in range(lo, hi + 1, 4096):
                rows = [[prefix, str(road), *suffix] for road in range(begin, min(hi + 1, begin + 4096))]
                for fmt, stream in streams.items():
                    separator = "," if fmt == "csv" else " "
                    stream.write("".join(separator.join(row) + "\n" for row in rows))

        loaded, emitted_sha = scan_table(rle_path, emit)
        if loaded != header or emitted_sha != dense_sha:
            raise ValueError("Source RLE changed during readable export")
    results = {}
    for fmt in formats:
        verified = validate_readable_table(partials[fmt], rle_path, fmt, dense_sha)
        results[fmt] = dict(verified, file=targets[fmt].name,
                            sha256=sha256_file(partials[fmt]), file_bytes=partials[fmt].stat().st_size,
                            separator="comma" if fmt == "csv" else "space",
                            decimal_format=".17g; exact float64 round trip",
                            columns=FEATURE_COLS + [LABEL_COL], source_rle_sha256=source_sha,
                            cpp_input=False)
    if sha256_file(rle_path) != source_sha:
        raise ValueError("Source RLE changed during readable export")
    # No final readable filename until every requested format has passed.
    for fmt in formats:
        if targets[fmt].exists():
            raise ValueError("Readable final path appeared during export")
        partials[fmt].rename(targets[fmt])
    return results


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--expected-threshold", choices=THRESHOLDS)
    parser.add_argument("--cap-step", type=float, default=0.1389, help="Uniform m/s precision; explicit choice for other networks")
    parser.add_argument("--cap-min", type=float)
    parser.add_argument("--cap-max", type=float)
    parser.add_argument("--max-road-flow", type=int, default=222)
    parser.add_argument("--max-lane-flow", type=int, default=80)
    parser.add_argument("--chunk-size", type=int, default=50000)
    parser.add_argument("--max-rows", type=int, default=10000000)
    parser.add_argument("--allow-static-extrapolation", action="store_true")
    parser.add_argument("--dense-reference", action="store_true", help="Also save exact expanded little-endian float64 values for tests")
    parser.add_argument("--text-format", choices=("none", "txt", "csv", "both"), default="none",
                        help="Optional readable table(s), decoded from RLE and verified bit-exact; C++ still reads RLE")
    parser.add_argument("--dry-run", action="store_true", help="No model load or output files")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    plan = build_plan(args)
    print(f"[PLAN] {plan['source_label']} raw_features=5 contexts={plan['context_count']} "
          f"rows={plan['expanded_rows']} groups={plan['group_count']} cap={plan['free_speed_cap_grid']}")
    for warning in plan["warnings"]:
        print("[WARN] " + warning)
    if args.dry_run:
        print("[DRY-RUN] Passed; no AutoGluon load or output writes")
        return plan
    return export_table(args, plan)


if __name__ == "__main__":
    main()
