#!/usr/bin/env python3
"""Nine-input ABC/RG catch, explicit remaining-time quantization, exact output RLE.

The old ABC reader/50M limit are unchanged. This separate compressed format has
a 4B expanded-row ceiling and 512MiB payload ceiling; CLI defaults to 50M rows.
Raw seconds are trained unchanged. Nearest-node deployment is explicitly lossy.
Optional road-flow thinning predicts representative nodes, then fills every
legal integer query through the original V1 RLE. Training inputs are unchanged.
Only load trusted AutoGluon models. A dry run never imports AutoGluon or writes.
"""
from __future__ import annotations

import argparse
from array import array
from bisect import bisect_left
import csv
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd

import catching._shared.model_catching_sumo_cap_context_abc as abc
import training.model_training_sumo_cap_context_abc_rg as trainer
from features.cap_context_abc_features import validate_abc_context_contract
from features.signal_rg_features import RGSignalProgram, network_green_horizon, network_without_internal_links

FEATURE_COLS = list(trainer.FEATURE_COLS)
FEATURE_SCHEMA = TRAINING_SCHEMA = trainer.SCHEMA
CATEGORICAL_FEATURES = dict(trainer.CATEGORICAL_FEATURES)
LABEL_COL = abc.LABEL_COL
TABLE_MAGIC = "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1"
PAYLOAD_LAYOUT = abc.PAYLOAD_LAYOUT
RUN = struct.Struct("<HHd")
MAX_CPP_ROWS = 4_000_000_000
MAX_PAYLOAD = 512 * 1024 * 1024
MAX_HEADER = 1024 * 1024
MAX_NODES = 100_000
QUANTIZATION = {"name": "nearest-positive-node-v1", "tie": "higher",
                "minimum_node_seconds": .001, "next_duration_key_scale": 1_000_000_000,
                "next_duration_tolerance_seconds": 1e-9, "entry_upper_tolerance_ulps": 8}
ROAD_FLOW_QUANTIZATION = {
    "name": "nearest-integer-road-flow-v1", "unit": "vehicles", "tie": "higher",
    "grid_origin": 0, "preserve_lane_lower_bound": True, "preserve_road_upper_bound": True,
    "output_coverage": "all-legal-integer-road-flow",
}


def _number(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be finite {'positive' if positive else 'nonnegative'} numeric")
    return float(value)


def _integer(value, name, minimum=0, maximum=MAX_CPP_ROWS):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be integer in {minimum}..{maximum}")
    return value


def _exact_json(actual, expected, name):
    """JSON numbers must not make booleans pass contract/category equality."""
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        valid = type(actual) is type(expected) and actual == expected
    elif type(expected) is int:
        valid = type(actual) is int and actual == expected
    elif type(expected) is float:
        valid = type(actual) in (int, float) and math.isfinite(actual) and actual == expected
    elif type(expected) is list:
        valid = type(actual) is list and len(actual) == len(expected)
        if valid:
            for index, (left, right) in enumerate(zip(actual, expected)):
                _exact_json(left, right, f"{name}[{index}]")
    elif type(expected) is dict:
        valid = type(actual) is dict and set(actual) == set(expected)
        if valid:
            for key in expected:
                _exact_json(actual[key], expected[key], name + "." + key)
    else:
        raise TypeError("Unsupported expected JSON type")
    if not valid:
        raise ValueError("Wrong exact contract value/type: " + name)


def road_flow_quantization(step=1, dense_until=10, nodes=None):
    """Policy only; step=1 preserves the original predictor input exactly."""
    _integer(step, "road flow step", 1)
    _integer(dense_until, "road flow dense until")
    if nodes is not None:
        if step != 1 or dense_until != 10:
            raise ValueError("Explicit road-flow nodes conflict with step/dense-until")
        if type(nodes) is not list or not nodes or len(nodes) > 65536:
            raise ValueError("road flow nodes must be a nonempty bounded list")
        previous = -1
        for node in nodes:
            _integer(node, "road flow node", 0, 65535)
            if node <= previous:
                raise ValueError("road flow nodes must be strictly increasing")
            previous = node
        if nodes[0] != 0:
            raise ValueError("road flow nodes must start at zero")
        return dict(ROAD_FLOW_QUANTIZATION, name="nearest-explicit-integer-road-flow-v1",
                    nodes=nodes.copy())
    return dict(ROAD_FLOW_QUANTIZATION, step=step, dense_until=dense_until)


def road_flow_segment_nodes(segments, maximum):
    """Compile inclusive UPPER:STEP bands, using global step multiples.

    The last upper bound must equal maximum. Band endpoints are preserved,
    as are zero/maximum; per-lane lower bounds are added later.
    """
    _integer(maximum, "road max", 0, 65535)
    if type(segments) is not list or not segments or len(segments) > 65536:
        raise ValueError("road flow segments require a nonempty bounded list")
    nodes, previous = {0, maximum}, -1
    for segment in segments:
        fields = segment.split(":") if type(segment) is str else []
        if len(fields) != 2 or not all(field.isascii() and field.isdecimal() for field in fields):
            raise ValueError("road flow segments must use integer UPPER:STEP")
        upper, step = map(int, fields)
        _integer(upper, "road flow segment upper", 0, maximum)
        _integer(step, "road flow segment step", 1, MAX_CPP_ROWS)
        if upper <= previous:
            raise ValueError("road flow segment upper bounds must be strictly increasing")
        start = ((previous + 1 + step - 1) // step) * step
        nodes.update(range(start, upper + 1, step))
        nodes.add(upper)
        previous = upper
    if previous != maximum:
        raise ValueError("Last road flow segment upper must equal --max-road-flow")
    return sorted(nodes)


def road_flow_nodes(lane, maximum, step=1, dense_until=10, nodes=None):
    """Global step multiples, dense prefix and both legal group endpoints."""
    _integer(maximum, "road max", 0, 65535)
    _integer(lane, "lane flow", 0, maximum)
    road_flow_quantization(step, dense_until, nodes)
    if nodes is not None:
        if nodes[-1] != maximum:
            raise ValueError("road flow nodes must end at road_flow_max")
        return sorted({lane, *nodes[bisect_left(nodes, lane):]})
    return sorted({lane, maximum, *range(lane, min(dense_until, maximum) + 1),
                   *range(((lane + step - 1) // step) * step, maximum + 1, step)})


def road_flow_spans(lane, maximum, step=1, dense_until=10, nodes=None):
    """Yield (representative, inclusive low, inclusive high), with ties high."""
    nodes = road_flow_nodes(lane, maximum, step, dense_until, nodes)
    for index, node in enumerate(nodes):
        low = lane if index == 0 else (nodes[index - 1] + node + 1) // 2
        high = maximum if index + 1 == len(nodes) else (node + nodes[index + 1] - 1) // 2
        yield node, low, high


def prediction_flow_count(maximum, lane_max, step=1, dense_until=10, nodes=None):
    """Count nodes without building a potentially huge expanded integer grid."""
    _integer(maximum, "road max", 0, 65535)
    _integer(lane_max, "lane max", 0, maximum)
    road_flow_quantization(step, dense_until, nodes)
    if nodes is not None:
        if nodes[-1] != maximum:
            raise ValueError("road flow nodes must end at road_flow_max")
        count = 0
        for lane in range(lane_max + 1):
            index = bisect_left(nodes, lane)
            count += len(nodes) - index + int(nodes[index] != lane)
        return count
    dense = min(dense_until, maximum)
    count = 0
    for lane in range(lane_max + 1):
        count += max(0, dense - lane + 1)
        lower = max(lane, dense + 1)
        count += max(0, maximum // step - (lower - 1) // step)
        count += int(lane > dense and lane % step != 0)
        count += int(maximum != lane and maximum > dense and maximum % step != 0)
    return count


def validate_road_flow_metadata(plan):
    """Legacy headers omit both fields; partial or noncanonical metadata fails."""
    fields = ("road_flow_quantization", "prediction_rows")
    present = [name in plan for name in fields]
    if not any(present):
        return road_flow_quantization()
    if not all(present):
        raise ValueError("Incomplete road-flow quantization/prediction_rows metadata")
    policy = plan["road_flow_quantization"]
    if type(policy) is not dict:
        raise ValueError("Invalid road_flow_quantization")
    if policy.get("name") == "nearest-explicit-integer-road-flow-v1":
        expected = road_flow_quantization(nodes=policy.get("nodes"))
    else:
        expected = road_flow_quantization(policy.get("step"), policy.get("dense_until"))
    _exact_json(policy, expected, "road_flow_quantization")
    grid = plan["free_speed_cap_grid"]
    wanted = (prediction_flow_count(plan["road_flow_max"], plan["lane_flow_max"],
                                    policy.get("step", 1), policy.get("dense_until", 10), policy.get("nodes"))
              * len(plan["road_lengths"]) * (grid["index_max"] - grid["index_min"] + 1)
              * 6 * plan["signal_point_count"])
    if _integer(plan["prediction_rows"], "prediction rows", 1) != wanted:
        raise ValueError("prediction_rows mismatch")
    if wanted > plan["expanded_rows"]:
        raise ValueError("prediction_rows exceeds expanded coverage")
    return policy


def validated_signal_contract(value):
    stable = trainer.validate_signal_contract(value)
    supplied = {key: item for key, item in value.items() if key not in (
        "runtime_programs", "clock_queries", "runtime_unsignalized_green_seconds", "runtime_green_horizon_movements",
        "runtime_network_without_internal_links")}
    _exact_json(supplied, stable, "signal_contract")
    return stable


def duration_key(value):
    value = _number(value, "next duration")
    scaled = value * QUANTIZATION["next_duration_key_scale"]
    if not math.isfinite(scaled) or scaled > 9e15:
        raise ValueError("Next duration key exceeds exact integer safety domain")
    return math.floor(scaled + .5)


def remaining_nodes(maximum, step):
    maximum = _number(maximum, "entry maximum", True)
    step = _number(step, "signal remaining step", True)
    ratio = maximum / step
    if not math.isfinite(ratio) or ratio > MAX_NODES - 2:
        raise ValueError("Signal node count exceeds safety limit")
    # Match the C++ double k*step expression exactly, including the endpoint.
    return sorted({min(.001, maximum), maximum,
                   *(k * step for k in range(1, math.floor(ratio) + 1)
                     if min(.001, maximum) < k * step < maximum)})


def signal_catalog(net_file, step, legacy=False):
    root = ET.parse(net_file).getroot()
    if any(element.tag in ("WAUT", "waut", "wautJunction", "wautSwitch") for element in root.iter()):
        raise ValueError("WAUT signals are unsupported")
    edges = {edge.get("id"): edge for edge in root.findall("edge")}
    external_lanes = {(edge.get("id"), int(lane.get("index", "0")))
                      for edge in root.findall("edge") if edge.get("function") != "internal" and not edge.get("id", "").startswith(":")
                      for lane in edge.findall("lane")}
    programs = {}
    for tls in root.findall("tlLogic"):
        key = tls.get("id")
        if not key or key in programs or tls.get("type") != "static":
            raise ValueError("Require one fixed static sequential program per TLS")
        phases = tls.findall("phase")
        if not phases:
            raise ValueError("TLS has no phases")
        for index, phase in enumerate(phases):
            if phase.get("next") not in (None, "", str((index + 1) % len(phases))):
                raise ValueError("Nonsequential TLS phase successor")
        programs[key] = phases
    combinations = {}
    seen_movements = set()
    has_unsignalized = False
    for connection in root.findall("connection"):
        source, target = connection.get("from"), connection.get("to")
        if (source or "").startswith(":") or (source in edges and edges[source].get("function") == "internal"):
            continue  # Internal connectors are not this feature's external exit.
        if source not in edges or target not in edges:
            raise ValueError("Unknown external connection edge; cannot infer no-TLS")
        if (target or "").startswith(":") or edges[target].get("function") == "internal":
            continue
        try:
            lanes = ((source, int(connection.get("fromLane", "0"))),
                     (target, int(connection.get("toLane", "0"))))
        except ValueError as exc:
            raise ValueError("Invalid external connection lane") from exc
        if any(lane not in external_lanes for lane in lanes):
            raise ValueError("Unknown external connection lane; cannot infer no-TLS")
        tls = connection.get("tl", "")
        if not tls:
            has_unsignalized = True
            continue
        try:
            index = int(connection.attrib["linkIndex"])
            phases = programs[tls]
            if index < 0:
                raise ValueError()
            states = [phase.attrib["state"][index] for phase in phases]
            durations = [float(phase.attrib["duration"]) for phase in phases]
        except (ValueError, KeyError, IndexError) as exc:
            raise ValueError("Unknown TLS/linkIndex or invalid movement phases") from exc
        if (tls, index) in seen_movements:
            continue
        seen_movements.add((tls, index))
        program = RGSignalProgram(durations, states)
        for j, state in enumerate(program._segment_states):
            if state == "Y":
                continue
            nxt = program._next_nonyellow[j]
            next_duration = program._segment_durations[nxt]
            key = (state, program._segment_states[nxt], next_duration)
            combinations[key] = max(combinations.get(key, 0), program._segment_durations[j])
    if has_unsignalized:
        if legacy:
            raise ValueError("Legacy model lacks the v3 zero-crossing uncontrolled-junction contract; recollect/retrain before this network")
        horizon, _ = network_green_horizon(net_file)
        key = ("G", "G", horizon)
        combinations[key] = max(combinations.get(key, 0), horizon)
    result = [{"entry_type": "G", "next_type": "G", "next_duration_seconds": 0.,
               "duration_key": 0, "entry_max_seconds": 0., "nodes": [0.],
               "point_start": 0, "sentinel": True}]
    for (entry, nxt, duration), maximum in combinations.items():
        result.append({"entry_type": entry, "next_type": nxt, "next_duration_seconds": duration,
                       "duration_key": duration_key(duration), "entry_max_seconds": maximum,
                       "nodes": remaining_nodes(maximum, step), "sentinel": False})
    result.sort(key=lambda item: (item["entry_type"], item["next_type"], item["duration_key"]))
    offset = 0
    for item in result:
        item["point_start"] = offset
        offset += len(item["nodes"])
    validate_catalog(result, step)
    return result


def validate_catalog(catalog, step):
    _number(step, "signal remaining step", True)
    if type(catalog) is not list or not catalog or len(catalog) > MAX_NODES:
        raise ValueError("Invalid signal catalog")
    fields = {"entry_type", "next_type", "next_duration_seconds", "duration_key",
              "entry_max_seconds", "nodes", "point_start", "sentinel"}
    previous, offset, seen = None, 0, {}
    for position, item in enumerate(catalog):
        if type(item) is not dict or set(item) != fields:
            raise ValueError("Wrong signal catalog fields")
        if item["entry_type"] not in ("G", "R") or item["next_type"] not in ("G", "R"):
            raise ValueError("Signal types must be G/R")
        duration = _number(item["next_duration_seconds"], "next duration")
        maximum = _number(item["entry_max_seconds"], "entry maximum")
        key = _integer(item["duration_key"], "duration key", maximum=9_000_000_000_000_000)
        if key != duration_key(duration):
            raise ValueError("Signal duration key mismatch")
        order = (item["entry_type"], item["next_type"], key)
        if previous is not None and order <= previous:
            raise ValueError("Duplicate or unordered signal catalog key")
        previous = order
        pair = order[:2]
        if pair in seen and duration - seen[pair] <= 2e-9:
            raise ValueError("Colliding/ambiguous next duration keys")
        seen[pair] = duration
        _integer(item["point_start"], "point start")
        if item["point_start"] != offset or type(item["sentinel"]) is not bool:
            raise ValueError("Invalid signal point offset/sentinel")
        nodes = item["nodes"]
        if type(nodes) is not list or not nodes or len(nodes) > MAX_NODES:
            raise ValueError("Invalid remaining nodes")
        for value in nodes:
            _number(value, "remaining node")
        if item["sentinel"]:
            if position != 0 or order != ("G", "G", 0) or duration != 0 or maximum != 0 or nodes != [0.]:
                raise ValueError("Invalid G0G0 sentinel")
        elif position == 0 or duration <= 0 or maximum <= 0 or nodes != remaining_nodes(maximum, step):
            raise ValueError("Noncanonical positive remaining nodes")
        offset += len(nodes)
    if not catalog[0]["sentinel"]:
        raise ValueError("Missing explicit no-TLS sentinel")
    return offset


def quantize_signal(catalog, entry_type, remaining, next_type, next_duration):
    remaining = _number(remaining, "entry remaining")
    next_duration = _number(next_duration, "next duration")
    key = duration_key(next_duration)
    for item in catalog:
        if (entry_type, next_type, key) != (item["entry_type"], item["next_type"], item["duration_key"]):
            continue
        if abs(next_duration - item["next_duration_seconds"]) > 1e-9:
            continue
        if item["sentinel"]:
            if remaining != 0 or next_duration != 0:
                raise ValueError("G0G0 sentinel mismatch")
            return item["point_start"], ("G", 0., "G", 0.)
        maximum = item["entry_max_seconds"]
        if remaining > maximum and remaining - maximum <= 8 * math.ulp(maximum):
            remaining = maximum  # Floating summation normalization, not seconds-scale extrapolation.
        if not 0 < remaining <= maximum:
            raise ValueError("Entry remaining outside catalog domain; no extrapolation")
        nodes = item["nodes"]
        index = bisect_left(nodes, remaining)
        if index == len(nodes):
            raise ValueError("Remaining exceeds last node")
        if index and remaining - nodes[index - 1] < nodes[index] - remaining:
            index -= 1
        return item["point_start"] + index, (entry_type, nodes[index], next_type, item["next_duration_seconds"])
    raise ValueError("Unknown signal combination; no duration/type fallback")


def feature_frame(rows):
    return trainer.encode_feature_frame(pd.DataFrame(rows, columns=FEATURE_COLS))


def resolve_model(model_dir, expected_threshold=None):
    supplied = Path(model_dir).expanduser().resolve(strict=True)
    run = supplied.parent if supplied.name == "predictor" else supplied
    path = run / "training_contract.json"
    contract = json.loads(path.read_text(encoding="utf-8-sig"), object_pairs_hook=abc._unique_json,
                          parse_constant=lambda text: (_ for _ in ()).throw(ValueError(text)))
    if contract.get("schema") != FEATURE_SCHEMA or contract.get("raw_features") != FEATURE_COLS:
        raise ValueError("Expected nine-feature ABC/RG training schema; old tables/models are incompatible")
    if contract.get("status") != "complete" or contract.get("model_label") != LABEL_COL or contract.get("predictor_directory") != "predictor":
        raise ValueError("Incomplete or incompatible training contract")
    source = contract.get("source_label")
    if source not in ["travel_time_label_gt_" + token for token in abc.THRESHOLDS] or (expected_threshold and source != "travel_time_label_gt_" + expected_threshold):
        raise ValueError("Source label/threshold mismatch")
    validate_abc_context_contract(contract.get("context_contract"))
    validated_signal_contract(contract.get("signal_contract"))
    _exact_json(contract.get("categorical_features"), CATEGORICAL_FEATURES, "categorical_features")
    if contract.get("categorical_ordered") is not False:
        raise ValueError("Require fixed unordered ABC/RG categories")
    framework = contract.get("framework_features")
    if type(framework) is not list or not framework or len(set(framework)) != len(framework) or not set(framework).issubset(FEATURE_COLS):
        raise ValueError("Invalid framework feature whitelist")
    for column in FEATURE_COLS:
        info = contract.get("training_features", {}).get(column, {})
        if column in CATEGORICAL_FEATURES:
            cats = CATEGORICAL_FEATURES[column]
            _exact_json(info.get("categories"), cats, "training_features." + column + ".categories")
            if info.get("type") != "categorical" or info.get("ordered") is not False or info.get("categories") != cats:
                raise ValueError(f"Wrong training categorical definition: {column}")
            observed = info.get("observed_categories")
            if type(observed) is not list or not observed or observed != [x for x in cats if x in observed]:
                raise ValueError(f"Wrong observed categories: {column}")
            if any(type(x) is not type(cats[0]) for x in observed):
                raise ValueError(f"Wrong observed category types: {column}")
            if column == "platoon_context" and info.get("category_names") != trainer.CONTEXT_NAMES:
                raise ValueError("Wrong ABC category mapping")
        else:
            lo, hi = [_number(info.get(bound), f"{column}.{bound}", column in ("road_length", "free_speed_cap")) for bound in ("min", "max")]
            if lo > hi or (column in ("road_flow", "lane_flow") and (lo != int(lo) or hi != int(hi))):
                raise ValueError(f"Invalid training range: {column}")
    predictor = run / "predictor"
    if not (predictor / "predictor.pkl").is_file():
        raise ValueError("Missing predictor.pkl")
    return predictor, path, contract


def validate_network_crossing_contract(net, contract):
    """Reject deploying an old positive incoming clock on a zero-crossing net."""
    if (network_without_internal_links(net)
            and contract["signal_contract"]["version"] != "static-movement-rg-segments-v4"
            and contract["context_contract"]["base_context_contract"]["crossing_time_seconds"] > 0):
        raise ValueError("Legacy model crossing clock is incompatible with no-internal-links network; use v4 collection/training or an audited zero-crossing model")


def build_plan(args):
    _integer(args.max_rows, "max rows", 1, MAX_CPP_ROWS)
    _integer(args.chunk_size, "chunk size", 1, 1_000_000)
    step = _number(args.signal_remaining_step, "signal remaining step", True)
    segments = getattr(args, "road_flow_segments", None)
    nodes = road_flow_segment_nodes(segments, args.max_road_flow) if segments is not None else None
    flow_policy = road_flow_quantization(args.road_flow_step, args.road_flow_dense_until, nodes)
    if Path(args.output_dir).expanduser().resolve().exists():
        raise ValueError("Output exists; no overwrite/resume")
    predictor, contract_path, contract = resolve_model(args.model_dir, args.expected_threshold)
    net = Path(args.net_file).expanduser().resolve(strict=True)
    abc.validate_export_network(net)
    pairs, edge_count = abc.network_pairs(net)
    lengths = sorted({length for length, _ in pairs})
    if len({abc.quantize_positive(x) for x in lengths}) != len(lengths):
        raise ValueError("Road length key collision")
    ranges = contract["training_features"]
    grid = abc.cap_grid(args.cap_step, args.cap_min if args.cap_min is not None else ranges["free_speed_cap"]["min"],
                        args.cap_max if args.cap_max is not None else ranges["free_speed_cap"]["max"])
    base_rows = abc.count_rows(len(lengths), grid, args.max_road_flow, args.max_lane_flow)
    validate_network_crossing_contract(net, contract)
    catalog = signal_catalog(net, step, legacy=contract["signal_contract"]["version"] not in {
        "static-movement-rg-segments-v3", "static-movement-rg-segments-v4"})
    points = sum(len(item["nodes"]) for item in catalog)
    rows = base_rows * points
    caps = grid["index_max"] - grid["index_min"] + 1
    groups = points * len(lengths) * caps * 6 * (args.max_lane_flow + 1)
    predicted_rows = (prediction_flow_count(args.max_road_flow, args.max_lane_flow,
                                           flow_policy.get("step", 1), flow_policy.get("dense_until", 10),
                                           flow_policy.get("nodes"))
                      * points * len(lengths) * caps * 6)
    warnings = ["Raw seconds were trained unchanged; nearest remaining-node catch is lossy input quantization, not interpolation.",
                "Exact output RLE does not imply exact raw-input prediction; coverage does not establish accuracy."]
    if predicted_rows < rows:
        warnings.append("Road-flow nearest-node prediction is lossy; RLE/readable/reference retain full integer query coverage. "
                        "Predictor rows shrink, but expanded --max-rows coverage and resource ceilings do not.")
    for length in lengths:
        if not ranges["road_length"]["min"] - 1e-8 <= length <= ranges["road_length"]["max"] + 1e-8:
            if not args.allow_static_extrapolation:
                raise ValueError("Network length outside training range")
            warnings.append(f"Static road length outside training range: {length}")
    for column, domain in {"road_flow": (0, args.max_road_flow), "lane_flow": (0, args.max_lane_flow),
                           "free_speed_cap": (grid["min"], grid["max"]),
                           "entry_signal_remaining_seconds": (0, max(x["entry_max_seconds"] for x in catalog)),
                           "next_signal_duration_seconds": (0, max(x["next_duration_seconds"] for x in catalog))}.items():
        if domain[0] < ranges[column]["min"] or domain[1] > ranges[column]["max"]:
            warnings.append(f"Grid {column} {domain} extends training range; not accuracy certification")
    for column, cats in CATEGORICAL_FEATURES.items():
        missing = [x for x in cats if x not in ranges[column]["observed_categories"]]
        if missing:
            warnings.append(f"Unobserved {column} categories: {missing}")
    plan = {"format": TABLE_MAGIC, "training_schema": FEATURE_SCHEMA, "columns": FEATURE_COLS + [LABEL_COL],
            "source_label": contract["source_label"], "road_lengths": lengths, "static_key_scale": 10000,
            "free_speed_cap_grid": grid, "context_contract": contract["context_contract"],
            "signal_contract": validated_signal_contract(contract["signal_contract"]),
            "context_count": 6, "categorical_features": CATEGORICAL_FEATURES, "categorical_ordered": False,
            "signal_catalog": catalog, "signal_remaining_step": step, "signal_quantization": dict(QUANTIZATION),
            "signal_point_count": points, "road_flow_max": args.max_road_flow, "lane_flow_max": args.max_lane_flow,
            "flow_count": base_rows // (len(lengths) * caps * 6), "expanded_rows": rows, "group_count": groups,
            "road_flow_quantization": flow_policy, "prediction_rows": predicted_rows,
            "prediction_floor_seconds": 1., "payload_layout": PAYLOAD_LAYOUT,
            "resource_limits": {"expanded_rows": MAX_CPP_ROWS, "payload_bytes": MAX_PAYLOAD,
                                "requested_max_rows": args.max_rows},
            "network": {"path": str(net), "sha256": abc.sha256_file(net), "external_edges": edge_count},
            "model": {"predictor_path": str(predictor), "contract_path": str(contract_path),
                      "contract_sha256": abc.sha256_file(contract_path),
                      "predictor_pickle_sha256": abc.sha256_file(predictor / "predictor.pkl"),
                      "framework_features": contract["framework_features"]},
            "exporter_sha256": abc.sha256_file(__file__), "training_ranges": ranges, "warnings": warnings}
    return plan


def enforce_budget(plan):
    rows, groups = plan["expanded_rows"], plan["group_count"]
    if rows > min(MAX_CPP_ROWS, plan["resource_limits"]["requested_max_rows"]):
        raise ValueError(f"Planned {rows:,} rows exceeds explicit --max-rows {plan['resource_limits']['requested_max_rows']:,}; no automatic coarsening")
    if 16 * groups + 4 > MAX_PAYLOAD:
        raise ValueError("Even one run per group exceeds 512MiB compressed payload budget")
    if len(json.dumps(plan, separators=(",", ":"), allow_nan=False).encode()) > MAX_HEADER - 512:
        raise ValueError("Planned metadata exceeds 1MiB header budget")


def compare_signal_steps(plan, steps):
    """Read-only sizes on the selected model/base grid; payload is a lower bound."""
    steps = list(dict.fromkeys(_number(step, "comparison signal step", True) for step in steps))

    def point_count(step):
        return sum(1 if item["sentinel"] else len(remaining_nodes(item["entry_max_seconds"], step))
                   for item in plan["signal_catalog"])

    baseline_points = point_count(1.)
    base_rows = plan["expanded_rows"] // plan["signal_point_count"]
    base_prediction_rows = plan.get("prediction_rows", plan["expanded_rows"]) // plan["signal_point_count"]
    base_groups = plan["group_count"] // plan["signal_point_count"]
    row_limit = min(MAX_CPP_ROWS, plan["resource_limits"]["requested_max_rows"])
    result = []
    for step in steps:
        points = point_count(step)
        rows, groups = base_rows * points, base_groups * points
        minimum_payload = 16 * groups + 4
        result.append({"signal_remaining_step": step, "signal_point_count": points,
                       "expanded_rows": rows, "group_count": groups,
                       "prediction_rows": base_prediction_rows * points,
                       "relative_to_1s": points / baseline_points,
                       "min_rle_bytes": minimum_payload,
                       "explicit_row_budget_pass": rows <= row_limit,
                       "minimum_payload_budget_pass": minimum_payload <= MAX_PAYLOAD})
    return result


def iter_signal_points(plan):
    for item in plan["signal_catalog"]:
        for node in item["nodes"]:
            yield (item["entry_type"], node, item["next_type"], item["next_duration_seconds"])


def iter_cases(plan):
    """All legal original integer queries, including approximated positions."""
    grid = plan["free_speed_cap_grid"]
    for signal in iter_signal_points(plan):
        for length in plan["road_lengths"]:
            for index in range(grid["index_min"], grid["index_max"] + 1):
                for context in range(6):
                    for lane in range(plan["lane_flow_max"] + 1):
                        for road in range(lane, plan["road_flow_max"] + 1):
                            yield (length, road, lane, index * grid["step"], context, *signal)


def iter_prediction_cases(plan):
    """Only representative inputs, each paired with its complete query span."""
    policy = validate_road_flow_metadata(plan)
    grid = plan["free_speed_cap_grid"]
    # Typical 222/80 grids use only a few thousand spans. Bound the optional
    # cache for unusually large legal domains instead of materializing all rows.
    cached, cached_nodes = {}, 0
    for signal in iter_signal_points(plan):
        for length in plan["road_lengths"]:
            for index in range(grid["index_min"], grid["index_max"] + 1):
                for context in range(6):
                    for lane in range(plan["lane_flow_max"] + 1):
                        spans = cached.get(lane)
                        if spans is None:
                            spans = tuple(road_flow_spans(lane, plan["road_flow_max"],
                                                         policy.get("step", 1), policy.get("dense_until", 10),
                                                         policy.get("nodes")))
                            if cached_nodes + len(spans) <= 100_000:
                                cached[lane] = spans
                                cached_nodes += len(spans)
                        for road, low, high in spans:
                            yield ((length, road, lane, index * grid["step"], context, *signal), low, high)


def expanded_prediction_blocks(values, widths):
    """Bound memory while hashing/writing full coverage, not only node values."""
    total = int(widths.sum())
    if total == len(values):
        yield np.asarray(values, dtype="<f8").tobytes()
    elif total <= 1_000_000:
        yield np.asarray(np.repeat(values, widths), dtype="<f8").tobytes()
    else:
        cumulative = np.cumsum(widths, dtype=np.int64)
        for start in range(0, total, 131_072):
            indexes = np.searchsorted(cumulative, np.arange(start, min(total, start + 131_072)), side="right")
            yield np.asarray(values[indexes], dtype="<f8").tobytes()


def validate_predictor(predictor, plan):
    if predictor.label != LABEL_COL or predictor.problem_type != "regression":
        raise ValueError("Predictor label/problem type mismatch")
    actual = list(predictor.feature_metadata.get_features())
    if set(actual) != set(plan["model"]["framework_features"]) or not set(actual).issubset(FEATURE_COLS):
        raise ValueError("Predictor feature whitelist mismatch")
    metadata = getattr(predictor, "feature_metadata_in", None)
    if metadata is None:
        # Generated metadata may encode binary categories as bool; do not mistake
        # legitimate internal encoding/constant pruning for wrong raw inputs.
        return
    for name in CATEGORICAL_FEATURES:
        if name not in metadata.get_features():
            continue
        getter = getattr(metadata, "get_feature_type_raw", None)
        raw = getter(name) if callable(getter) else getattr(metadata, "type_map_raw", {}).get(name)
        if raw is not None and raw != "category":
            raise ValueError(f"Predictor raw input {name} is not categorical")


def export_table(args, plan, predictor=None):
    enforce_budget(plan)
    if predictor is None:
        from autogluon.tabular import TabularPredictor
        predictor = TabularPredictor.load(plan["model"]["predictor_path"])
    validate_predictor(predictor, plan)
    output = Path(args.output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    records_path = output / "records.bin.partial"
    dense_path = output / "reference.f64.partial"
    offsets = array("I")
    if offsets.itemsize != 4:
        raise RuntimeError("32-bit unsigned offsets required")
    dense = dense_path.open("xb") if args.dense_reference else None
    digest = hashlib.sha256()
    count = run_count = clipped = predicted_count = clipped_nodes = 0
    last_key = None
    low = high = None
    previous = None
    raw_min, raw_max = math.inf, -math.inf
    offset_bytes_count = 4 * (plan["group_count"] + 1)
    try:
        with records_path.open("xb") as records:
            def save_run():
                nonlocal run_count
                if offset_bytes_count + 12 * (run_count + 1) > MAX_PAYLOAD:
                    raise ValueError("Actual RLE exceeds 512MiB payload; partial only, no complete manifest")
                records.write(RUN.pack(low, high, previous))
                run_count += 1
            for spans in abc.batches(iter_prediction_cases(plan), args.chunk_size):
                batch = [item[0] for item in spans]
                raw = np.asarray(predictor.predict(feature_frame(batch)), dtype=np.float64)
                if raw.shape != (len(batch),) or not np.isfinite(raw).all():
                    raise ValueError("Predictor returned nonfinite/wrong-shape predictions")
                raw_min, raw_max = min(raw_min, float(raw.min())), max(raw_max, float(raw.max()))
                widths = np.asarray([high - low + 1 for _, low, high in spans], dtype=np.int64)
                clipped_nodes += int((raw < 1).sum())
                clipped += int(widths[raw < 1].sum())
                predicted_count += len(batch)
                values = np.maximum(raw, 1.)
                for encoded in expanded_prediction_blocks(values, widths):
                    digest.update(encoded)
                    if dense:
                        dense.write(encoded)
                for (row, span_low, span_high), value in zip(spans, values):
                    key = (row[0], row[2], *row[3:])
                    value = float(value)
                    if key != last_key:
                        if last_key is not None:
                            save_run()
                        offsets.append(run_count)
                        low, high = span_low, span_high
                        previous, last_key = value, key
                    elif value == previous:
                        high = span_high
                    else:
                        save_run()
                        low, high = span_low, span_high
                        previous = value
                    count += span_high - span_low + 1
            if last_key is not None:
                save_run()
        offsets.append(run_count)
    finally:
        if dense:
            dense.close()
    if (count != plan["expanded_rows"] or predicted_count != plan.get("prediction_rows", plan["expanded_rows"])
            or len(offsets) != plan["group_count"] + 1):
        raise ValueError("Export grid mismatch")
    if sys.byteorder != "little":
        offsets.byteswap()
    offset_bytes = offsets.tobytes()
    payload_sha = hashlib.sha256(offset_bytes)
    with records_path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            payload_sha.update(block)
    header = dict(plan, run_count=run_count, payload_bytes=len(offset_bytes) + 12 * run_count,
                  payload_sha256=payload_sha.hexdigest())
    header_bytes = json.dumps(header, separators=(",", ":"), allow_nan=False).encode()
    if len(header_bytes) > MAX_HEADER:
        raise ValueError("Header exceeds 1MiB")
    partial = output / "table.rle.partial"
    with partial.open("xb") as stream:
        stream.write(TABLE_MAGIC.encode() + b"\n" + header_bytes + b"\n" + offset_bytes)
        with records_path.open("rb") as records:
            for block in iter(lambda: records.read(1024 * 1024), b""):
                stream.write(block)
    _, expanded_sha = scan_table(partial)
    if expanded_sha != digest.hexdigest():
        raise ValueError("RLE round-trip changed predictions")
    text_outputs = write_readable_tables(partial, output, args.text_format)
    partial.rename(output / "table.rle")
    if args.dense_reference:
        dense_path.rename(output / "reference.f64")
    records_path.unlink()  # Exact newly created spool only; never source artifacts.
    manifest = {"status": "complete", "table": "table.rle", "table_sha256": abc.sha256_file(output / "table.rle"),
                "rows": count, "run_count": run_count, "file_bytes": (output / "table.rle").stat().st_size,
                "prediction_rows": predicted_count,
                "dense_value_bytes": 8 * count, "dense_sha256": digest.hexdigest(),
                "raw_prediction_min": raw_min, "raw_prediction_max": raw_max,
                "raw_prediction_population": "representative-node predictions before coverage expansion",
                "predictions_floored_to_one_second": clipped,
                "prediction_nodes_floored_to_one_second": clipped_nodes,
                "floor_count_population": "predictions_floored_to_one_second counts full expanded query coverage",
                "roundtrip_exact": True,
                "text_outputs": text_outputs, "plan": header}
    with (output / "manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, allow_nan=False)
    return manifest


def validate_header(header):
    if header.get("format") != TABLE_MAGIC or header.get("training_schema") != FEATURE_SCHEMA or header.get("columns") != FEATURE_COLS + [LABEL_COL]:
        raise ValueError("Wrong ABC/RG table schema, magic or feature order")
    _exact_json(header.get("categorical_features"), CATEGORICAL_FEATURES, "categorical_features")
    _exact_json(header.get("signal_quantization"), QUANTIZATION, "signal_quantization")
    _exact_json(header.get("prediction_floor_seconds"), 1., "prediction_floor_seconds")
    _integer(header.get("static_key_scale"), "static key scale", 10000, 10000)
    if header.get("payload_layout") != PAYLOAD_LAYOUT or header.get("categorical_ordered") is not False:
        raise ValueError("Wrong payload/category contract")
    if header.get("source_label") not in ["travel_time_label_gt_" + t for t in abc.THRESHOLDS]:
        raise ValueError("Invalid source label")
    if header.get("signal_quantization") != QUANTIZATION or header.get("static_key_scale") != 10000 or header.get("prediction_floor_seconds") != 1.:
        raise ValueError("Wrong quantization/static/floor contract")
    _integer(header.get("context_count"), "context count", 6, 6)
    validate_abc_context_contract(header.get("context_contract"))
    validated_signal_contract(header.get("signal_contract"))
    points = validate_catalog(header.get("signal_catalog"), header.get("signal_remaining_step"))
    if _integer(header.get("signal_point_count"), "signal point count", 1) != points:
        raise ValueError("Signal point count mismatch")
    limits = header.get("resource_limits")
    if type(limits) is not dict or set(limits) != {"expanded_rows", "payload_bytes", "requested_max_rows"} or limits["expanded_rows"] != MAX_CPP_ROWS or limits["payload_bytes"] != MAX_PAYLOAD:
        raise ValueError("Wrong compressed resource contract")
    _integer(limits["requested_max_rows"], "requested max rows", 1)
    lengths = header.get("road_lengths")
    if type(lengths) is not list or not lengths:
        raise ValueError("Missing road lengths")
    keys = []
    for length in lengths:
        _number(length, "length", True)
        key = abc.quantize_positive(length)
        if abs(length - key / 10000.) > 1e-10 * max(1, length):
            raise ValueError("Non-four-decimal length")
        keys.append(key)
    if keys != sorted(set(keys)):
        raise ValueError("Unordered/duplicate length keys")
    grid = header.get("free_speed_cap_grid", {})
    lo = _integer(grid.get("index_min"), "cap index min", 1, 2**31 - 1)
    hi = _integer(grid.get("index_max"), "cap index max", lo, 2**31 - 1)
    step = _number(grid.get("step"), "cap step", True)
    for name, wanted in (("min", lo * step), ("max", hi * step)):
        value = _number(grid.get(name), "cap " + name, True)
        if abs(value - wanted) > 1e-10 * max(1, abs(wanted)):
            raise ValueError("Cap lattice endpoint mismatch")
    if grid.get("scaled_boundary_tolerance") != 1e-9 or grid.get("raw_boundary_tolerance") != 1e-12:
        raise ValueError("Cap tolerance mismatch")
    road = _integer(header.get("road_flow_max"), "road max", 0, 65535)
    lane = _integer(header.get("lane_flow_max"), "lane max", 0, road)
    rows = abc.count_rows(len(lengths), grid, road, lane) * points
    groups = points * len(lengths) * (hi - lo + 1) * 6 * (lane + 1)
    flows = (lane + 1) * (road + 1) - lane * (lane + 1) // 2
    for name, expected in (("expanded_rows", rows), ("group_count", groups), ("flow_count", flows)):
        if _integer(header.get(name), name, 1) != expected:
            raise ValueError(f"{name} mismatch")
    validate_road_flow_metadata(header)
    runs = _integer(header.get("run_count"), "run count", groups, rows)
    payload = _integer(header.get("payload_bytes"), "payload bytes", 1, MAX_PAYLOAD)
    if payload != 4 * (groups + 1) + 12 * runs:
        raise ValueError("Payload dimensions mismatch")
    digest = header.get("payload_sha256")
    if type(digest) is not str or len(digest) != 64 or any(x not in "0123456789abcdef" for x in digest):
        raise ValueError("Invalid SHA256")
    enforce_budget(header)


def _open_table(path):
    stream = Path(path).open("rb")
    try:
        if stream.readline(128) != TABLE_MAGIC.encode() + b"\n":
            raise ValueError("Wrong ABC/RG magic")
        line = stream.readline(MAX_HEADER + 2)
        if len(line) > MAX_HEADER + 1 or not line.endswith(b"\n"):
            raise ValueError("Oversize/truncated metadata")
        header = json.loads(line, object_pairs_hook=abc._unique_json,
                            parse_constant=lambda text: (_ for _ in ()).throw(ValueError(text)))
        validate_header(header)
        start = stream.tell()
        if Path(path).stat().st_size != start + header["payload_bytes"]:
            raise ValueError("Truncated/trailing payload bytes")
        digest = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
        if digest.hexdigest() != header["payload_sha256"]:
            raise ValueError("Payload SHA256 mismatch")
        stream.seek(start)
        return stream, header, start
    except BaseException:
        stream.close()
        raise


def iter_runs(path):
    # Both scans use the same open file identity; hash the bytes being decoded
    # again, rather than assuming a file cannot change after the initial scan.
    offsets, header, start = _open_table(path)
    try:
        with Path(path).open("rb") as records:
            records.seek(start + 4 * (header["group_count"] + 1))
            digest = hashlib.sha256()
            previous_offset = None
            offset_sha = hashlib.sha256()
            # Hash ordered offsets first, then record bytes while decoding.
            for _ in range(header["group_count"] + 1):
                raw = offsets.read(4)
                if len(raw) != 4:
                    raise ValueError("Truncated offsets")
                digest.update(raw)
            initial_offsets_sha = digest.copy().digest()
            offsets.seek(start)
            first = offsets.read(4)
            offset_sha.update(first)
            previous_offset = struct.unpack("<I", first)[0]
            if previous_offset != 0:
                raise ValueError("First offset is not zero")
            for group in range(header["group_count"]):
                raw = offsets.read(4)
                offset_sha.update(raw)
                if len(raw) != 4:
                    raise ValueError("Truncated offsets")
                current = struct.unpack("<I", raw)[0]
                if not previous_offset < current <= header["run_count"]:
                    raise ValueError("Bad group offsets")
                expected_low = group % (header["lane_flow_max"] + 1)
                previous = None
                for _ in range(previous_offset, current):
                    raw = records.read(12)
                    if len(raw) != 12:
                        raise ValueError("Truncated record")
                    digest.update(raw)
                    low, high, value = RUN.unpack(raw)
                    if low != expected_low or high < low or high > header["road_flow_max"]:
                        raise ValueError("Run bounds gap/overlap/outside domain")
                    if not math.isfinite(value) or value < 1 or value == previous:
                        raise ValueError("Nonfinite/below-floor/noncanonical equal runs")
                    yield header, group, low, high, value
                    previous, expected_low = value, high + 1
                if expected_low != header["road_flow_max"] + 1:
                    raise ValueError("Incomplete road-flow group")
                previous_offset = current
            if previous_offset != header["run_count"] or records.read(1):
                raise ValueError("Final offset/trailing bytes mismatch")
            # A second offsets hash detects modifications between the two passes.
            offsets.seek(start)
            check = hashlib.sha256()
            for _ in range(header["group_count"] + 1):
                check.update(offsets.read(4))
            if (check.digest() != offset_sha.digest() or check.digest() != initial_offsets_sha
                    or digest.hexdigest() != header["payload_sha256"]):
                raise ValueError("Payload changed during decoding")
    finally:
        offsets.close()


def scan_table(path):
    digest = hashlib.sha256()
    header = None
    for header, _, low, high, value in iter_runs(path):
        digest.update(struct.pack("<d", value) * (high - low + 1))
    return header, digest.hexdigest()


def read_table(path):
    stream, header, _ = _open_table(path)
    stream.close()
    if header["expanded_rows"] > 50_000_000:
        raise ValueError("read_table dense inspection exceeds 50M; use scan_table/iter_runs")
    values = np.empty(header["expanded_rows"], dtype="<f8")
    offset = 0
    for _, _, low, high, value in iter_runs(path):
        count = high - low + 1
        values[offset:offset + count] = value
        offset += count
    return header, values


def iter_readable_rows(path):
    cases = None
    for header, _, low, high, value in iter_runs(path):
        if cases is None:
            cases = iter(iter_cases(header))
        for _ in range(high - low + 1):
            yield (*next(cases), value)


def write_readable_tables(path, output, text_format):
    if text_format not in ("none", "txt", "csv", "both"):
        raise ValueError("Invalid text format")
    kinds = [] if text_format == "none" else ["txt", "csv"] if text_format == "both" else [text_format]
    streams, reports = {}, {}
    try:
        for kind in kinds:
            streams[kind] = (Path(output) / ("table." + kind + ".partial")).open("x", encoding="utf-8", newline="")
        writers = {kind: csv.writer(stream, delimiter=" " if kind == "txt" else ",", lineterminator="\n") for kind, stream in streams.items()}
        for writer in writers.values():
            writer.writerow(FEATURE_COLS + [LABEL_COL])
        count = 0
        if writers:
            for row in iter_readable_rows(path):
                encoded = [repr(value) if isinstance(value, float) else value for value in row]
                for writer in writers.values():
                    writer.writerow(encoded)
                count += 1
    finally:
        for stream in streams.values():
            stream.close()
    for kind in kinds:
        partial = Path(output) / ("table." + kind + ".partial")
        final = Path(output) / ("table." + kind)
        if final.exists():
            raise ValueError("Readable destination exists")
        partial.rename(final)
        reports[kind] = {"file": final.name, "rows": count, "bytes": final.stat().st_size,
                         "sha256": abc.sha256_file(final), "float_format": "Python repr binary64 roundtrip"}
    return reports


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--expected-threshold", "--threshold", choices=list(abc.THRESHOLDS))
    parser.add_argument("--signal-remaining-step", type=float, required=True,
                        help="Explicit lossy nearest-node seconds precision; no default")
    parser.add_argument("--compare-signal-steps", type=float, nargs="+",
                        help="Dry-run only: compare positive step sizes on this model/grid, relative to 1s; "
                             "duplicates are removed in input order; never changes the selected step")
    parser.add_argument("--cap-step", type=float, default=.1389)
    parser.add_argument("--cap-min", type=float)
    parser.add_argument("--cap-max", type=float)
    parser.add_argument("--max-road-flow", type=int, default=222)
    parser.add_argument("--max-lane-flow", type=int, default=80)
    parser.add_argument("--road-flow-step", type=int, default=1,
                        help="Integer vehicle-count node spacing above dense prefix; 1 disables flow approximation")
    parser.add_argument("--road-flow-dense-until", type=int, default=10,
                        help="Keep every integer road_flow from 0 through this inclusive value (default 10)")
    parser.add_argument("--road-flow-segments", nargs="+", metavar="UPPER:STEP",
                        help="Optional piecewise global-multiple grid, e.g. 10:1 60:5 100:20 222:50. "
                             "Inclusive uppers strictly increase and end at --max-road-flow; endpoints retained. "
                             "Cannot combine with nondefault --road-flow-step/--road-flow-dense-until")
    parser.add_argument("--chunk-size", type=int, default=50000)
    parser.add_argument("--max-rows", type=int, default=50_000_000,
                        help="Full integer query coverage budget, not sparse predictor rows; hard ceiling 4B")
    parser.add_argument("--allow-static-extrapolation", action="store_true")
    parser.add_argument("--dense-reference", action="store_true")
    parser.add_argument("--text-format", choices=["none", "txt", "csv", "both"], default="none")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.compare_signal_steps is not None:
        if not args.dry_run:
            parser.error("--compare-signal-steps requires --dry-run")
        for step in args.compare_signal_steps:
            _number(step, "comparison signal step", True)
    plan = build_plan(args)
    policy = plan["road_flow_quantization"]
    road_grid = (f"road_flow_nodes={policy['nodes']} " if "nodes" in policy else
                 f"road_flow_step_vehicles={policy['step']} "
                 f"road_flow_dense_until_inclusive={policy['dense_until']} ")
    print(f"[PLAN] schema={FEATURE_SCHEMA} label={plan['source_label']} "
          f"signal_remaining_step_seconds={plan['signal_remaining_step']:g} signal_points={plan['signal_point_count']} "
          f"{road_grid}"
          f"road_flow_quantization={json.dumps(plan['road_flow_quantization'], separators=(',', ':'))} "
          f"prediction_rows={plan['prediction_rows']} expanded_rows={plan['expanded_rows']} "
          f"groups={plan['group_count']} hypothetical_dense_bytes={8 * plan['expanded_rows']} "
          f"min_rle_bytes={16 * plan['group_count'] + 4} payload_limit={MAX_PAYLOAD}")
    for warning in plan["warnings"]:
        print("[WARN] " + warning)
    if args.compare_signal_steps is not None:
        comparison = compare_signal_steps(plan, args.compare_signal_steps)
        print("[COMPARE] Same model/base grid; relative_to_1s compares expanded rows; minimum payload pass "
              "does not establish actual compression/accuracy. Selected step is unchanged.")
        for item in comparison:
            print(f"[SCALE] step_seconds={item['signal_remaining_step']:g} "
                  f"signal_points={item['signal_point_count']} rows={item['expanded_rows']} "
                  f"prediction_rows={item['prediction_rows']} expanded_rows={item['expanded_rows']} "
                  f"relative_to_1s={item['relative_to_1s']:.6g} min_rle_bytes={item['min_rle_bytes']} "
                  f"explicit_row_budget={'PASS' if item['explicit_row_budget_pass'] else 'FAIL'} "
                  f"minimum_payload_budget={'PASS' if item['minimum_payload_budget_pass'] else 'FAIL'}")
    enforce_budget(plan)
    if args.dry_run:
        print("[DRY_RUN] Contract/grid/budget only; predictor not loaded, compression/accuracy not measured, no files written")
        return plan
    return export_table(args, plan)


if __name__ == "__main__":
    main()
