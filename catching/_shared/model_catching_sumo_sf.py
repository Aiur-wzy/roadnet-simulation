#!/usr/bin/env python3
"""Export a versioned five-feature SF table; legacy C++ cannot load this format.

Use the same AutoGluon environment as training. Only load trusted models.
No training CSV, labels, refitting or SF correction of predicted time is needed.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from decimal import Decimal, ROUND_CEILING, ROUND_FLOOR
from itertools import islice
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd

FEATURE_COLS = ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"]
LABEL_COL = "travel_time_no_waiting"
TRAINING_SCHEMA = "road-time-core3-sf-speed-limit-v1"
TABLE_MAGIC = "CAMS_SF_TABLE_V1"
KEY_SCALE = 10000
THRESHOLDS = ("0p1", "0p5", "1", "1p5", "2", "3")


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def positive_number(value, name):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return number


def quantize_positive(value, scale=KEY_SCALE):
    scaled = positive_number(value, "key") * scale
    if not math.isfinite(scaled) or scaled >= 2**63 - 1:
        raise ValueError("Quantized key exceeds signed 64-bit range")
    # The future C++ reader must use this same scaled-boundary tolerance.
    return math.floor(scaled + 0.5 + 1e-9)


def resolve_model(model_dir, expected_threshold=None):
    supplied = Path(model_dir).expanduser().resolve(strict=True)
    if (supplied / "training_contract.json").is_file():
        run = supplied
    elif supplied.name == "predictor" and (supplied.parent / "training_contract.json").is_file():
        run = supplied.parent
    else:
        raise ValueError("Model must be a training run containing training_contract.json and predictor/, "
                         "or that run's predictor/ directory")
    contract_path = run / "training_contract.json"
    contract = json.loads(contract_path.read_text(encoding="utf-8-sig"))
    if contract.get("schema") != TRAINING_SCHEMA or contract.get("raw_features") != FEATURE_COLS:
        raise ValueError("Training schema/raw_features mismatch; the legacy five-feature model is not supported")
    if contract.get("model_label") != LABEL_COL or contract.get("predictor_directory") != "predictor":
        raise ValueError("Training label/predictor_directory mismatch")
    label = contract.get("source_label")
    if label not in ["travel_time_label_gt_" + t for t in THRESHOLDS] + ["travel_time_label"]:
        raise ValueError("Unknown source_label in training contract")
    if expected_threshold and label != "travel_time_label_gt_" + expected_threshold:
        raise ValueError(f"Threshold mismatch: requested {expected_threshold}, model label is {label}")
    ranges = contract.get("training_features", {})
    for column in FEATURE_COLS:
        info = ranges.get(column, {})
        try:
            lo, hi = float(info["min"]), float(info["max"])
        except (TypeError, KeyError, ValueError) as exc:
            raise ValueError(f"Missing/invalid training range: {column}") from exc
        if not math.isfinite(lo) or not math.isfinite(hi) or lo > hi or lo < 0:
            raise ValueError(f"Invalid training range: {column}")
        if column not in ("road_flow", "lane_flow") and lo <= 0:
            raise ValueError(f"Nonpositive training range: {column}")
    predictor_path = run / "predictor"
    if not (predictor_path / "predictor.pkl").is_file():
        raise ValueError(f"Missing predictor.pkl: {predictor_path}")
    if not isinstance(contract.get("framework_features"), list):
        raise ValueError("Missing completed-fit framework_features in training contract")
    if not set(contract["framework_features"]).issubset(FEATURE_COLS):
        raise ValueError("Unknown framework_features in training contract")
    return predictor_path, contract_path, contract


def network_pairs(net_file):
    """Current aligned contract: equal lane lengths; external-road mean speed."""
    pairs, edge_count, seen_edges = {}, 0, set()
    for edge in ET.parse(net_file).getroot().findall("edge"):
        edge_id = edge.get("id", "")
        if edge_id.startswith(":") or edge.get("function") == "internal":
            continue
        if not edge_id or edge_id in seen_edges:
            raise ValueError("Missing or duplicate external edge id")
        seen_edges.add(edge_id)
        lanes = edge.findall("lane")
        if not lanes:
            raise ValueError(f"External edge has no lanes: {edge_id}")
        lengths = [positive_number(lane.get("length", "nan"), "lane length") for lane in lanes]
        speeds = [positive_number(lane.get("speed", "nan"), "lane speed") for lane in lanes]
        if max(lengths) - min(lengths) > 1e-6:
            raise ValueError(f"Unequal lane lengths on {edge_id}: collector uses selected lane but C++ uses mean; "
                             "resolve this contract before export")
        length, speed = sum(lengths) / len(lengths), sum(speeds) / len(speeds)
        key = (quantize_positive(length), quantize_positive(speed))
        pair = (length, speed)
        if key in pairs and any(abs(a - b) > 1e-8 for a, b in zip(pair, pairs[key])):
            raise ValueError(f"Static feature key collision at 1/{KEY_SCALE}: {edge_id}")
        pairs[key] = pair
        edge_count += 1
    if not pairs:
        raise ValueError("No external roads in network")
    if any(abs(value - round(value, 4)) > 1e-8 for pair in pairs.values() for value in pair):
        raise ValueError("Static precision exceeds collector's four decimals; align collector/C++ rounding first")
    return [pairs[key] for key in sorted(pairs)], edge_count


def sf_grid(step, lower, upper):
    """Use an integer scale; endpoints align outward, queries round half up."""
    step_decimal = Decimal(str(positive_number(step, "sf-step")))
    scale_decimal = Decimal(1) / step_decimal
    if scale_decimal != scale_decimal.to_integral_value() or not 1 <= scale_decimal <= 1000000:
        raise ValueError("sf-step must be the reciprocal of an integer scale in 1..1000000")
    scale = int(scale_decimal)
    lo = Decimal(str(positive_number(lower, "sf-min")))
    hi = Decimal(str(positive_number(upper, "sf-max")))
    if lo > hi:
        raise ValueError("sf-min exceeds sf-max")
    first = int((lo * scale).to_integral_value(rounding=ROUND_FLOOR))
    last = int((hi * scale).to_integral_value(rounding=ROUND_CEILING))
    if first < 1:
        raise ValueError("SF grid would contain zero; choose a smaller step or a higher sf-min")
    return {"scale": scale, "index_min": first, "index_max": last,
            "min": first / scale, "max": last / scale, "step": 1 / scale,
            "rounding": "nearest, positive half-up: floor(SF * scale + 0.5 + 1e-9)",
            "scaled_boundary_tolerance": 1e-9,
            "raw_boundary_tolerance": 1e-12,
            "out_of_range": "error; no clamp and no implicit formula fallback"}


def sf_index(value, grid):
    value = positive_number(value, "speed_factor")
    tolerance = grid["raw_boundary_tolerance"]
    if value < grid["min"] - tolerance or value > grid["max"] + tolerance:
        raise ValueError("SF outside table domain")
    index = quantize_positive(value, grid["scale"])
    if not grid["index_min"] <= index <= grid["index_max"]:
        raise ValueError("SF index outside table domain")
    return index


def count_rows(pair_count, grid, max_road_flow, max_lane_flow):
    # Sum_{r=0..R} (min(r,L)+1), without allocating the grid.
    capped = min(max_lane_flow, max_road_flow)
    flow_pairs = (capped + 1) * (capped + 2) // 2 + (max_road_flow - capped) * (capped + 1)
    return pair_count * flow_pairs * (grid["index_max"] - grid["index_min"] + 1)


def build_plan(args):
    if args.max_road_flow < 0 or args.max_lane_flow < 0:
        raise ValueError("Flow maxima must be nonnegative integers")
    if args.chunk_size < 1 or args.max_rows < 1:
        raise ValueError("chunk-size and max-rows must be positive")
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError(f"Output already exists; choose a new --output-dir: {output}")
    predictor_path, contract_path, contract = resolve_model(args.model_dir, args.expected_threshold)
    net = Path(args.net_file).expanduser().resolve(strict=True)
    pairs, edge_count = network_pairs(net)
    ranges = contract["training_features"]
    lower = args.sf_min if args.sf_min is not None else ranges["speed_factor"]["min"]
    upper = args.sf_max if args.sf_max is not None else ranges["speed_factor"]["max"]
    grid = sf_grid(args.sf_step, lower, upper)
    rows = count_rows(len(pairs), grid, args.max_road_flow, args.max_lane_flow)
    if rows > args.max_rows:
        raise ValueError(f"Planned {rows} rows exceeds --max-rows {args.max_rows}; inspect dimensions first")
    warnings = []
    for length, speed in pairs:
        for column, value in (("road_length", length), ("speed_limit", speed)):
            info = ranges[column]
            if value < float(info["min"]) - 1e-8 or value > float(info["max"]) + 1e-8:
                message = f"Network {column}={value} outside training range [{info['min']}, {info['max']}]"
                if not args.allow_static_extrapolation:
                    raise ValueError(message + "; check --net-file, or explicitly allow static extrapolation")
                warnings.append(message)
    domains = {"road_flow": (0, args.max_road_flow), "lane_flow": (0, min(args.max_lane_flow, args.max_road_flow)),
               "speed_factor": (grid["min"], grid["max"])}
    for column, (lo, hi) in domains.items():
        info = ranges[column]
        if lo < float(info["min"]) - 1e-12 or hi > float(info["max"]) + 1e-12:
            warnings.append(f"Grid {column}=[{lo}, {hi}] extends training range [{info['min']}, {info['max']}]; "
                            "coverage is not a guarantee of prediction accuracy")
        if lo > float(info["min"]) + 1e-12 or hi < float(info["max"]) - 1e-12:
            warnings.append(f"Grid {column} does not cover the full training min/max; deployment bounds must be checked")
    return {"format": TABLE_MAGIC, "training_schema": TRAINING_SCHEMA,
            "columns": FEATURE_COLS + [LABEL_COL], "source_label": contract["source_label"],
            "network": {"path": str(net), "sha256": sha256_file(net), "external_edges": edge_count},
            "static_pairs": [{"road_length": p[0], "speed_limit": p[1]} for p in pairs],
            "static_key_scale": KEY_SCALE,
            "static_key_rounding": "floor(value * scale + 0.5 + 1e-9)",
            "speed_factor_grid": grid, "road_flow_max": args.max_road_flow,
            "lane_flow_max": min(args.max_lane_flow, args.max_road_flow),
            "flow_min": 0, "flow_step": 1, "flow_constraint": "lane_flow <= road_flow",
            "planned_rows": rows, "prediction_floor_seconds": 1.0,
            "lookup_policy": "exact static/flow keys and nearest SF node; no interpolation or clamping",
            "scope": "external-road time only; no junction/wait addition and no division by SF",
            "legacy_cpp_compatible": False, "exporter_sha256": sha256_file(__file__),
            "model": {"predictor_path": str(predictor_path), "contract_path": str(contract_path),
                      "contract_sha256": sha256_file(contract_path),
                      "predictor_pickle_sha256": sha256_file(predictor_path / "predictor.pkl"),
                      "hash_scope": "predictor.pkl only, not all ensemble weight files",
                      "framework_features": contract["framework_features"],
                      "dependency_versions": contract.get("dependency_versions", {})},
            "training_ranges": ranges, "warnings": warnings}


def iter_cases(plan):
    grid = plan["speed_factor_grid"]
    for pair in plan["static_pairs"]:
        for index in range(grid["index_min"], grid["index_max"] + 1):
            for road_flow in range(plan["road_flow_max"] + 1):
                for lane_flow in range(min(plan["lane_flow_max"], road_flow) + 1):
                    yield (pair["road_length"], road_flow, lane_flow,
                           index / grid["scale"], pair["speed_limit"])


def batches(iterator, size):
    iterator = iter(iterator)
    while batch := list(islice(iterator, size)):
        yield batch


def validate_predictor(predictor, plan):
    if predictor.label != LABEL_COL or predictor.problem_type != "regression":
        raise ValueError("Loaded predictor label/problem_type mismatch")
    actual = list(predictor.feature_metadata.get_features())
    if set(actual) != set(plan["model"]["framework_features"]):
        raise ValueError("Loaded predictor feature metadata differs from training contract")
    if not set(actual).issubset(FEATURE_COLS):
        raise ValueError("Predictor requires unsupported features")


def export_table(args, plan):
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError("Output already exists; no overwrite or resume is performed")
    try:
        from autogluon.tabular import TabularPredictor
    except ImportError as exc:
        raise RuntimeError("Use the same AutoGluon environment as training") from exc
    predictor = TabularPredictor.load(plan["model"]["predictor_path"])
    validate_predictor(predictor, plan)
    output.mkdir(parents=True)  # Exclusive run directory; existing tables are never overwritten.
    partial = output / "table.txt.partial"
    count, clipped, raw_min, raw_max = 0, 0, math.inf, -math.inf
    with partial.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(TABLE_MAGIC + "\n")
        stream.write(json.dumps(plan, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")
        stream.write(" ".join(plan["columns"]) + "\n")
        for batch in batches(iter_cases(plan), args.chunk_size):
            frame = pd.DataFrame(batch, columns=FEATURE_COLS)
            # Send every raw feature, including constant speed_limit, exactly as in the trainer.
            raw = np.asarray(predictor.predict(frame), dtype=float)
            if raw.shape != (len(frame),) or not np.isfinite(raw).all():
                raise ValueError("Non-finite/wrong-shape model predictions; partial output is not a complete table")
            raw_min, raw_max = min(raw_min, float(raw.min())), max(raw_max, float(raw.max()))
            clipped += int((raw < plan["prediction_floor_seconds"]).sum())
            prediction = np.maximum(raw, plan["prediction_floor_seconds"])
            for values, travel_time in zip(batch, prediction):
                stream.write(" ".join(format(float(v), ".17g") for v in (*values, travel_time)) + "\n")
            count += len(batch)
            print(f"[PROGRESS] {count}/{plan['planned_rows']}", flush=True)
    if count != plan["planned_rows"]:
        raise ValueError("Export row count mismatch; partial table retained")
    final = output / "table.txt"
    if final.exists():
        raise ValueError("Unexpected table.txt in new output directory; refusing overwrite")
    partial.rename(final)
    report = {"status": "complete", "table": "table.txt", "table_sha256": sha256_file(final),
              "rows": count, "raw_prediction_min": raw_min, "raw_prediction_max": raw_max,
              "predictions_floored_to_one_second": clipped, "plan": plan}
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(f"[DONE] {final} rows={count} floor_count={clipped}")
    print("[NOTICE] New versioned format: do not pass it to the legacy C++ table reader.")
    return report


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--model-dir", required=True, help="Run with training_contract.json and predictor/, or predictor/ itself")
    parser.add_argument("--net-file", required=True, help="Deployment SUMO network; not the default old test.net.xml")
    parser.add_argument("--output-dir", required=True, help="New directory for table.txt and manifest.json")
    parser.add_argument("--expected-threshold", choices=THRESHOLDS)
    parser.add_argument("--sf-step", type=float, default=0.01)
    parser.add_argument("--sf-min", type=float, help="Default: training minimum, grid rounded outward")
    parser.add_argument("--sf-max", type=float, help="Default: training maximum, grid rounded outward")
    parser.add_argument("--max-road-flow", type=int, default=222)
    parser.add_argument("--max-lane-flow", type=int, default=80)
    parser.add_argument("--chunk-size", type=int, default=50000)
    parser.add_argument("--max-rows", type=int, default=10000000)
    parser.add_argument("--allow-static-extrapolation", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Check contract/network and count rows; no AutoGluon load or output writes")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    plan = build_plan(args)
    for warning in plan["warnings"]:
        print("[WARN] " + warning)
    print(f"[PLAN] label={plan['source_label']} pairs={len(plan['static_pairs'])} "
          f"SF={plan['speed_factor_grid']} rows={plan['planned_rows']}")
    if args.dry_run:
        print("[DRY-RUN] Passed; no model load or output writes")
        return plan
    return export_table(args, plan)


if __name__ == "__main__":
    main()
