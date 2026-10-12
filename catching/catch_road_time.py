"""Independent native-model, demand-only road-time table exporter.

Planning never loads a predictor or allocates a Cartesian table. The native
training contract and C++ proxy input contract are deliberately different.
Only explicit discovery runs may use missing-key placeholders; they are never
evaluation results. Completed scenario scores require a separate strict replay.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from decimal import Decimal, ROUND_CEILING, ROUND_FLOOR, ROUND_HALF_UP
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from training.native_contract import (
    SOURCE_SCHEMA, MODEL_SCHEMA, FEATURE_COLUMNS, CATEGORICAL_FEATURES,
    THRESHOLD_ORDER, MODEL_LABEL, encode_features, validate_model_contract,
    sha256_file, model_directory_name,
)
from training import native_contract as native_contract_module
from catching import resume_store

MAGIC = "CAMS_NATIVE_ABC_RG_DEMAND_V2"
LAYOUT = "sorted-u64-u32-f64-v2"
RECORD = struct.Struct("<QId")
MAX_JSON_INTEGER = 2**53 - 1
MAX_PAYLOAD = 512 * 1024 * 1024
MAX_HEADER = 1024 * 1024
MAX_NODES = 100_000
MAX_FLOW = 2**31 - 1  # C++ query API uses signed int; payload stores uint32.
SIGNAL_QUANTIZATION = {
    "name": "nearest-positive-node-v1", "tie": "higher",
    "minimum_node_seconds": .001, "next_duration_key_scale": 1_000_000_000,
    "next_duration_tolerance_seconds": 1e-9, "entry_upper_tolerance_ulps": 8,
}
FLOW_QUANTIZATION = {
    "name": "nearest-integer-road-flow-v1", "unit": "vehicles", "tie": "higher",
    "grid_origin": 0, "preserve_lane_lower_bound": True,
    "preserve_road_upper_bound": True, "output_coverage": "all-legal-integer-road-flow",
}


def number(value, name, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (positive and value == 0):
        raise ValueError(f"{name} must be finite {'positive' if positive else 'nonnegative'} numeric")
    return float(value)


def integer(value, name, minimum=0, maximum=MAX_JSON_INTEGER):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be integer in {minimum}..{maximum}")
    return value


def unique_json(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        return json.load(stream, object_pairs_hook=unique_json,
                         parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))


def encoded_json(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, allow_nan=False,
                      separators=(",", ":")).encode("utf-8")


def canonical_sha(value):
    return hashlib.sha256(encoded_json(value)).hexdigest()


def write_json(path, value):
    with Path(path).open("xb") as stream:
        stream.write(encoded_json(value) + b"\n")


def signature(path):
    path = Path(path).resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"Not a regular file: {path}")
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def duration_key(value):
    scaled = number(value, "next duration") * 1_000_000_000
    if not math.isfinite(scaled) or scaled > MAX_JSON_INTEGER:
        raise ValueError("Signal duration key exceeds exact JSON integer range")
    return math.floor(scaled + .5)


def length_key(value):
    scaled = number(value, "road length", True) * 10000
    if not math.isfinite(scaled) or scaled > MAX_JSON_INTEGER:
        raise ValueError("Road length key exceeds exact JSON integer range")
    return math.floor(scaled + .5 + 1e-9)


def remaining_nodes(maximum, step):
    maximum = number(maximum, "entry maximum", True)
    step = number(step, "signal remaining step", True)
    ratio = maximum / step
    if not math.isfinite(ratio) or ratio > MAX_NODES - 2:
        raise ValueError("Signal remaining node count exceeds safety limit")
    return sorted({min(.001, maximum), maximum,
                   *(k * step for k in range(1, math.floor(ratio) + 1)
                     if min(.001, maximum) < k * step < maximum)})


def cyclic_segments(durations, states):
    """Normalize and merge cyclic colors; never merge across a yellow segment."""
    if not durations or len(durations) != len(states):
        raise ValueError("Missing/mismatched signal phases")
    segments = []
    total = 0.
    for duration, state in zip(durations, states):
        duration = number(duration, "phase duration", True)
        if state not in ("r", "R", "g", "G", "y", "Y"):
            raise ValueError("Unsupported movement signal state")
        if not math.isfinite(total + duration) or total + duration <= total:
            raise ValueError("Signal cycle loses finite time precision")
        total += duration
        state = state.upper()
        if segments and segments[-1][0] == state:
            segments[-1] = (state, segments[-1][1] + duration)
        else:
            segments.append((state, duration))
    if len(segments) > 1 and segments[0][0] == segments[-1][0]:
        segments[0] = (segments[0][0], segments[-1][1] + segments[0][1])
        segments.pop()
    if len(segments) == 1 or all(state == "Y" for state, _ in segments):
        raise ValueError("Constant-color/all-yellow movement lacks finite RG segments")
    return segments


def network_catalog(path, step):
    """Static geometry and network-derived RG grid, with no vehicle/model work."""
    root = ET.parse(path).getroot()
    if root.tag != "net" or any(item.tag in ("WAUT", "waut", "wautJunction", "wautSwitch") for item in root.iter()):
        raise ValueError("Require SUMO network with fixed sequential static signals")
    edges, lanes, internal_lanes, lengths = {}, set(), set(), {}
    internal = False
    for edge in root.findall("edge"):
        name = edge.get("id")
        if not name or name in edges:
            raise ValueError("Missing/duplicate edge ID")
        edges[name] = edge
        is_internal = edge.get("function") == "internal" or name.startswith(":")
        internal |= is_internal
        lane_lengths, lane_speeds = [], []
        for lane in edge.findall("lane"):
            if is_internal:
                internal_lanes.add(lane.get("id"))
                continue
            index = int(lane.get("index", "-1"))
            if index < 0 or (name, index) in lanes:
                raise ValueError("Missing/duplicate external lane index")
            lanes.add((name, index))
            lane_lengths.append(number(float(lane.get("length", "nan")), "lane length", True))
            lane_speeds.append(number(float(lane.get("speed", "nan")), "lane speed", True))
        if not is_internal:
            if not lane_lengths or len(set(lane_lengths)) != 1 or len(set(lane_speeds)) != 1:
                raise ValueError("Require equal physical lane lengths/limits within each road")
            value = lane_lengths[0]  # No averaging: identical XML lanes must not acquire float tails.
            key = length_key(value)
            canonical = key / 10000.
            if abs(value - canonical) > 1e-10 * max(1., abs(value), abs(canonical)):
                raise ValueError("Physical road length is outside transport four-decimal precision; never silently round features")
            if key in lengths and lengths[key] != value:
                raise ValueError("Distinct physical road lengths collide at static key precision")
            lengths[key] = value
    if not lengths:
        raise ValueError("No external road geometry")
    programs = {}
    for tls in root.findall("tlLogic"):
        name, phases = tls.get("id"), tls.findall("phase")
        if not name or name in programs or tls.get("type") != "static" or not phases:
            raise ValueError("Require one nonempty static program per TLS")
        if not math.isfinite(float(tls.get("offset", "0"))):
            raise ValueError("Nonfinite TLS offset")
        for index, phase in enumerate(phases):
            if phase.get("next", "").strip() not in ("", str((index + 1) % len(phases))):
                raise ValueError("Nonsequential TLS phase successor")
        states = [phase.get("state", "") for phase in phases]
        if not states[0] or len({len(state) for state in states}) != 1:
            raise ValueError("Inconsistent TLS state width")
        programs[name] = ([float(phase.get("duration", "nan")) for phase in phases], states)
    combinations, seen, horizon, uncontrolled, has_via = {}, set(), 0., False, False
    for connection in root.findall("connection"):
        via = connection.get("via", "").strip()
        has_via |= bool(via)
        if via and via not in internal_lanes:
            raise ValueError("Connection references missing internal via lane")
        source, target = connection.get("from"), connection.get("to")
        if source not in edges or target not in edges:
            raise ValueError("Connection references unknown edge")
        if any(name.startswith(":") or edges[name].get("function") == "internal" for name in (source, target)):
            continue
        if any((name, int(connection.get(field, "-1"))) not in lanes
               for name, field in ((source, "fromLane"), (target, "toLane"))):
            raise ValueError("Connection references unknown external lane")
        tls = connection.get("tl", "")
        if not tls:
            uncontrolled = True
            continue
        index = int(connection.get("linkIndex", "-1"))
        if tls not in programs or index < 0 or index >= len(programs[tls][1][0]):
            raise ValueError("Unknown TLS/linkIndex")
        if (tls, index) in seen:
            continue
        seen.add((tls, index))
        durations, states = programs[tls]
        segments = cyclic_segments(durations, [state[index] for state in states])
        horizon = max(horizon, max((duration for state, duration in segments if state == "G"), default=0.))
        for position, (state, duration) in enumerate(segments):
            if state == "Y":
                continue
            following = (position + 1) % len(segments)
            while segments[following][0] == "Y":
                following = (following + 1) % len(segments)
            next_state, next_duration = segments[following]
            key = state, next_state, next_duration
            combinations[key] = max(combinations.get(key, 0.), duration)
    for junction in root.findall("junction"):
        if set(junction.get("intLanes", "").split()) - internal_lanes:
            raise ValueError("Junction references missing internal lane")
    if uncontrolled:
        if horizon <= 0:
            raise ValueError("No finite network green horizon for uncontrolled movement")
        combinations["G", "G", horizon] = max(combinations.get(("G", "G", horizon), 0.), horizon)
    catalog = [{"entry_type": "G", "next_type": "G", "next_duration_seconds": 0.,
                "duration_key": 0, "entry_max_seconds": 0., "nodes": [0.],
                "point_start": 0, "sentinel": True}]
    for (entry, nxt, duration), maximum in combinations.items():
        catalog.append({"entry_type": entry, "next_type": nxt, "next_duration_seconds": duration,
                        "duration_key": duration_key(duration), "entry_max_seconds": maximum,
                        "nodes": remaining_nodes(maximum, step), "point_start": 0, "sentinel": False})
    catalog.sort(key=lambda item: (item["entry_type"], item["next_type"], item["duration_key"]))
    offset, previous = 0, {}
    for item in catalog:
        pair = item["entry_type"], item["next_type"]
        if pair in previous and item["next_duration_seconds"] - previous[pair] <= 2e-9:
            raise ValueError("Ambiguous signal next-duration keys")
        previous[pair] = item["next_duration_seconds"]
        item["point_start"] = offset
        offset += len(item["nodes"])
    if offset > MAX_NODES:
        raise ValueError("Total signal points exceed safety limit")
    return sorted(lengths.values()), catalog, {
        "path": str(Path(path).resolve()), "sha256": sha256_file(path),
        "external_edges": len({name for name, _ in lanes}), "without_internal_links": not internal and not has_via,
    }


def cap_grid(step, lower, upper):
    step = Decimal(str(number(step, "cap step", True)))
    lower = Decimal(str(number(lower, "cap min", True)))
    upper = Decimal(str(number(upper, "cap max", True)))
    if lower > upper:
        raise ValueError("cap min exceeds max")
    def ratio(value):
        raw = value / step
        nearest = raw.to_integral_value(rounding=ROUND_HALF_UP)
        return nearest if abs(raw - nearest) <= Decimal("1e-9") else raw
    first = int(ratio(lower).to_integral_value(rounding=ROUND_FLOOR))
    last = int(ratio(upper).to_integral_value(rounding=ROUND_CEILING))
    if first < 1 or last > 2**31 - 1:
        raise ValueError("Cap grid includes zero or exceeds signed index range")
    return {"index_min": first, "index_max": last, "step": float(step),
            "min": float(first * step), "max": float(last * step),
            "scaled_boundary_tolerance": 1e-9, "raw_boundary_tolerance": 1e-12}


def flow_policy(step=1, dense_until=10, segments=None, maximum=222):
    integer(step, "road flow step", 1, 2**32 - 1)
    integer(dense_until, "road flow dense until", 0, MAX_FLOW)
    if segments is None:
        return dict(FLOW_QUANTIZATION, step=step, dense_until=dense_until)
    if step != 1 or dense_until != 10 or not segments:
        raise ValueError("Flow segments conflict with step/dense-until or are empty")
    nodes, previous = {0, maximum}, -1
    for segment in segments:
        if not re.fullmatch(r"[0-9]+:[0-9]+", segment):
            raise ValueError("Flow segments require UPPER:STEP integers")
        upper, increment = map(int, segment.split(":"))
        integer(upper, "segment upper", previous + 1, maximum)
        integer(increment, "segment step", 1, 2**32 - 1)
        start = ((previous + increment) // increment) * increment
        count = max(0, (upper - start) // increment + 1)
        if len(nodes) + count + 1 > MAX_NODES:
            raise ValueError("Explicit road nodes exceed bounded plan budget")
        nodes.update(range(start, upper + 1, increment))
        nodes.add(upper)
        previous = upper
    if previous != maximum:
        raise ValueError("Last flow segment must end at max road flow")
    return dict(FLOW_QUANTIZATION, name="nearest-explicit-integer-road-flow-v1", nodes=sorted(nodes))


def flow_nodes(lane, maximum, policy):
    if maximum - lane + 1 > MAX_NODES:
        raise ValueError("Explicit flow node enumeration exceeds budget; use representative_road_flow")
    if "nodes" in policy:
        nodes = policy["nodes"]
        return sorted({lane, *nodes[bisect_left(nodes, lane):]})
    step, dense = policy["step"], policy["dense_until"]
    return sorted({lane, maximum, *range(lane, min(dense, maximum) + 1),
                   *range(((lane + step - 1) // step) * step, maximum + 1, step)})


def representative_road_flow(road, lane, maximum, policy):
    """Nearest legal node without constructing a per-lane grid; ties go high."""
    if not 0 <= lane <= road <= maximum:
        raise ValueError("Illegal lane/road flow")
    if "nodes" in policy:
        nodes = policy["nodes"]
        position = bisect_left(nodes, road)
        high = nodes[position]
        low = max(lane, nodes[position - 1]) if position else lane
        return low if road - low < high - road else high
    step, dense = policy["step"], policy["dense_until"]
    if road <= dense:
        return road
    low = max(lane, min(maximum, dense), (road // step) * step)
    high = min(maximum, ((road + step - 1) // step) * step)
    return low if road - low < high - road else high


def nearest_index(nodes, value):
    index = bisect_left(nodes, value)
    if index == len(nodes):
        return index - 1
    if index and value - nodes[index - 1] < nodes[index] - value:
        index -= 1
    return index


def resolve_model(directory, threshold=None):
    directory = Path(directory).resolve(strict=True)
    contract_path = directory / "model_contract.json"
    contract = validate_model_contract(read_json(contract_path))
    if threshold is not None and contract["source_label_column"] != "travel_time_label_gt_" + threshold:
        raise ValueError("Model label does not match expected threshold")
    predictor = (directory / contract["predictor_directory"]).resolve(strict=True)
    if directory not in predictor.parents or not (predictor / "predictor.pkl").is_file():
        raise ValueError("Predictor must be a self-contained native model subdirectory")
    if any(path.is_symlink() for path in directory.rglob("*")):
        raise ValueError("Model artifact contains symbolic link")
    expected_files = contract.get("predictor_files_sha256")
    actual_files = {path.relative_to(predictor).as_posix(): sha256_file(path)
                    for path in predictor.rglob("*") if path.is_file()}
    if type(expected_files) is not dict or expected_files != actual_files:
        raise ValueError("Predictor artifact hashes/file set differ from native model contract")
    if contract.get("validation_metrics_sha256") != sha256_file(directory / "validation_metrics.json"):
        raise ValueError("Native validation metrics hash mismatch")
    files = []
    for path in sorted(directory.rglob("*")):
        if path.is_symlink():
            raise ValueError("Model artifact contains symbolic link")
        if path.is_file():
            files.append(signature(path))
    return contract, {"contract_path": str(contract_path), "contract_sha256": sha256_file(contract_path),
                      "predictor_path": str(predictor), "files": files}


def build_plan(args, source_files=None):
    integer(args.max_road_flow, "max road flow", 0, MAX_FLOW)
    integer(args.max_lane_flow, "max lane flow", 0, args.max_road_flow)
    integer(args.chunk_size, "chunk size", 1, 1_000_000)
    integer(args.max_cache_entries, "max cache entries", 0, MAX_FLOW)
    number(args.signal_remaining_step, "signal remaining step", True)
    number(args.crossing_time, "crossing time")
    if args.first_road_position_correction not in ("on", "off") or args.cap_out_of_range not in ("clamp", "error"):
        raise ValueError("Unsupported native query policy")
    contract, model = resolve_model(args.model_dir, args.expected_threshold)
    files = model.pop("files")
    if source_files is not None:
        source_files.extend(files)
    model["file_inventory_sha256"] = canonical_sha(files)
    model["file_count"] = len(files)
    lengths, catalog, network = network_catalog(args.net_file, args.signal_remaining_step)
    ranges = contract["feature_ranges"]
    grid = cap_grid(args.cap_step, args.cap_min if args.cap_min is not None else ranges["free_speed_cap"]["min"],
                    args.cap_max if args.cap_max is not None else ranges["free_speed_cap"]["max"])
    policy = flow_policy(args.road_flow_step, args.road_flow_dense_until, args.road_flow_segments, args.max_road_flow)
    points = sum(len(item["nodes"]) for item in catalog)
    caps, lanes = grid["index_max"] - grid["index_min"] + 1, args.max_lane_flow + 1
    blocks = points * len(lengths) * caps * 6
    flows = lanes * (args.max_road_flow + 1) - args.max_lane_flow * lanes // 2
    groups, expanded = blocks * lanes, blocks * flows
    integer(groups, "virtual group count", 1)
    integer(expanded, "virtual row count", 1)
    warnings = ["SUMO native training inputs and C++ decision proxies are not equivalent.",
                "Sparse coverage is scenario/request scoped, not full-domain coverage or accuracy certification.",
                "Input node quantization is lossy; predictions retain the existing one-second floor.",
                "Only positive finite cap may clamp to table endpoints; flow/signal/categories remain strict."]
    for length in lengths:
        if not ranges["road_length"]["min"] - 1e-8 <= length <= ranges["road_length"]["max"] + 1e-8:
            if not args.allow_static_extrapolation:
                raise ValueError("Network road length outside training range; explicit override required")
            warnings.append("Network contains road lengths outside training range.")
            break
    domains = {"road_flow": (0, args.max_road_flow), "lane_flow": (0, args.max_lane_flow),
               "free_speed_cap": (grid["min"], grid["max"]),
               "entry_signal_remaining_seconds": (0, max(item["entry_max_seconds"] for item in catalog)),
               "next_signal_duration_seconds": (0, max(item["next_duration_seconds"] for item in catalog))}
    for name, (lower, upper) in domains.items():
        if lower < ranges[name]["min"] or upper > ranges[name]["max"]:
            warnings.append(f"Planned {name} domain extends observed training range.")
    plan = {
        "format": MAGIC, "payload_layout": LAYOUT, "training_schema": MODEL_SCHEMA, "source_schema": SOURCE_SCHEMA,
        "columns": list(FEATURE_COLUMNS) + [MODEL_LABEL], "source_label": contract["source_label_column"],
        # The original complete contract is SHA-bound. Only its semantic fields
        # belong in a bounded transport header; full file inventories stay in scope.json.
        "model_contract": {key: contract[key] for key in (
            "schema", "status", "source_schema", "source_label_column", "feature_columns", "categorical",
            "model_label", "predictor_directory", "collection_contract", "deployment_contract", "feature_ranges",
            "framework_raw_features", "framework_removed_raw_features")},
        "deployment_contract": {"version": "sumo-native-training-to-cpp-proxy-v3",
            "training_inputs": "sumo-native-first-visible-snapshot", "query_inputs": "cpp-entry-decision-proxy",
            "equivalent": False, "abc_algorithm": "entry-frozen-free-clock-six-contexts",
            "crossing_time_seconds": float(args.crossing_time), "overdue_threshold_seconds": 0.,
            "crossing_policy": "first-or-known-unsignalized-or-no-internal-zero",
            "first_road_position_correction": args.first_road_position_correction,
            "cap_out_of_range": args.cap_out_of_range},
        "road_lengths": lengths, "static_key_scale": 10000, "free_speed_cap_grid": grid,
        "context_count": 6, "categorical_features": CATEGORICAL_FEATURES, "categorical_ordered": False,
        "signal_catalog": catalog, "signal_remaining_step": float(args.signal_remaining_step),
        "signal_quantization": dict(SIGNAL_QUANTIZATION), "signal_point_count": points,
        "road_flow_max": args.max_road_flow, "lane_flow_max": args.max_lane_flow,
        "flow_count": flows, "expanded_rows": expanded, "group_count": groups,
        "road_flow_quantization": policy, "prediction_floor_seconds": 1.,
        # Zero removes the old 512 MiB application ceiling. Readers must still
        # verify actual file size before allocation and run under an OS budget.
        "resource_limits": {"max_payload_bytes": 0, "max_json_integer": MAX_JSON_INTEGER},
        "network": network, "model": model, "training_ranges": ranges, "warnings": warnings,
    }
    if len(encoded_json(plan)) > MAX_HEADER - 1024:
        raise ValueError("Native table header exceeds bounded size; reduce artifact metadata")
    return plan


class KeyCodec:
    def __init__(self, plan):
        self.plan = plan
        self.grid = plan["free_speed_cap_grid"]
        deployment = plan["deployment_contract"]
        if (deployment["version"] != "sumo-native-training-to-cpp-proxy-v3"
                or deployment["first_road_position_correction"] not in ("on", "off")
                or deployment["cap_out_of_range"] not in ("clamp", "error")):
            raise ValueError("Unsupported native query policy")
        self.clamp_cap = deployment["cap_out_of_range"] == "clamp"
        self.caps = self.grid["index_max"] - self.grid["index_min"] + 1
        self.lanes = plan["lane_flow_max"] + 1
        self.lengths = {length_key(value): index for index, value in enumerate(plan["road_lengths"])}
        self.signals = []
        self.combinations = {}
        for item in plan["signal_catalog"]:
            self.combinations[item["entry_type"], item["next_type"], item["duration_key"]] = item
            self.signals.extend((item["entry_type"], value, item["next_type"], item["next_duration_seconds"])
                                for value in item["nodes"])

    def decode(self, key):
        group, road = key
        integer(group, "group index", 0, self.plan["group_count"] - 1)
        integer(road, "road flow", 0, self.plan["road_flow_max"])
        lane, group = group % self.lanes, group // self.lanes
        context, group = group % 6, group // 6
        cap, group = group % self.caps, group // self.caps
        length, signal = group % len(self.plan["road_lengths"]), group // len(self.plan["road_lengths"])
        if representative_road_flow(road, lane, self.plan["road_flow_max"], self.plan["road_flow_quantization"]) != road:
            raise ValueError("Requested key is not a legal road-flow representative")
        return (self.plan["road_lengths"][length], road, lane,
                (self.grid["index_min"] + cap) * self.grid["step"], context, *self.signals[signal])

    def encode(self, row):
        if isinstance(row, dict):
            row = tuple(row[name] for name in FEATURE_COLUMNS)
        if len(row) != 9:
            raise ValueError("Expected nine native features")
        length, road, lane, cap, context, entry, remaining, nxt, duration = row
        integer(road, "road flow", 0, self.plan["road_flow_max"])
        integer(lane, "lane flow", 0, min(road, self.plan["lane_flow_max"]))
        integer(context, "context", 0, 5)
        key = length_key(length)
        if key not in self.lengths:
            raise ValueError("Unknown network road length")
        cap = number(cap, "free speed cap", True)
        if self.clamp_cap:
            # Lookup-only projection. Keep raw speed for physical motion/ABC
            # clocks; never silently alter flow, signal or categorical inputs.
            cap = min(self.grid["max"], max(self.grid["min"], cap))
        elif cap < self.grid["min"] - 1e-12 or cap > self.grid["max"] + 1e-12:
            raise ValueError("Cap outside declared table domain")
        cap_index = math.floor(cap / self.grid["step"] + .5 + 1e-9)
        if not self.grid["index_min"] <= cap_index <= self.grid["index_max"]:
            raise ValueError("Cap index outside declared table domain")
        remaining, duration = number(remaining, "signal remaining"), number(duration, "next duration")
        item = self.combinations.get((entry, nxt, duration_key(duration)))
        if item is None or abs(duration - item["next_duration_seconds"]) > 1e-9:
            raise ValueError("Unknown signal combination")
        if item["sentinel"]:
            if remaining != 0 or duration != 0:
                raise ValueError("Invalid final-route G0G0 sentinel")
            signal = item["point_start"]
        else:
            maximum = item["entry_max_seconds"]
            if maximum < remaining <= maximum + 8 * math.ulp(maximum):
                remaining = maximum
            if not 0 < remaining <= maximum:
                raise ValueError("Signal remaining outside declared domain")
            signal = item["point_start"] + nearest_index(item["nodes"], remaining)
        group = (((signal * len(self.lengths) + self.lengths[key]) * self.caps
                  + cap_index - self.grid["index_min"]) * 6 + context) * self.lanes + lane
        return group, representative_road_flow(road, lane, self.plan["road_flow_max"], self.plan["road_flow_quantization"])


def read_requests(path, plan, max_entries=2_000_000):
    codec, keys, count = KeyCodec(plan), set(), 0
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["group_index", "road_flow"]:
            raise ValueError("Requests require group_index,road_flow CSV")
        for row in reader:
            if None in row or any(not re.fullmatch(r"[0-9]+", row.get(name) or "") for name in reader.fieldnames):
                raise ValueError("Malformed native demand request")
            key = int(row["group_index"]), int(row["road_flow"])
            codec.decode(key)
            keys.add(key)
            count += 1
            if max_entries and len(keys) > max_entries:
                raise ValueError("Demand request count exceeds explicit memory budget")
    return keys, count


def write_cache(path, plan, scope, values):
    payload_limit = plan["resource_limits"]["max_payload_bytes"]
    if len(values) * RECORD.size > (payload_limit or MAX_JSON_INTEGER):
        raise ValueError("Sparse payload exceeds declared transport range")
    codec, digest, keys = KeyCodec(plan), hashlib.sha256(), sorted(values)
    for key in keys:
        codec.decode(key)
        value = number(values[key], "prediction", True)
        if value < 1:
            raise ValueError("Prediction below preserved one-second floor")
        digest.update(RECORD.pack(*key, value))
    header = dict(plan, scope_sha256=scope, entry_count=len(values), payload_bytes=len(values) * RECORD.size,
                  payload_sha256=digest.hexdigest())
    data = encoded_json(header)
    if len(data) > MAX_HEADER:
        raise ValueError("Native header exceeds 1MiB")
    with Path(path).open("xb") as stream:
        stream.write(MAGIC.encode("ascii") + b"\n" + data + b"\n")
        for key in keys:
            stream.write(RECORD.pack(*key, values[key]))
    return header


def read_cache(path, expected_plan=None, expected_scope=None, retain_values=True):
    with Path(path).open("rb") as stream:
        if stream.readline(128) != MAGIC.encode("ascii") + b"\n":
            raise ValueError("Not a native-v2 demand table")
        line = stream.readline(MAX_HEADER + 2)
        if not line.endswith(b"\n") or len(line) > MAX_HEADER + 1:
            raise ValueError("Oversized/incomplete native header")
        header = json.loads(line, object_pairs_hook=unique_json,
                            parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        if header.get("format") != MAGIC or header.get("payload_layout") != LAYOUT:
            raise ValueError("Wrong native payload format")
        if header.get("training_schema") != MODEL_SCHEMA or header.get("source_schema") != SOURCE_SCHEMA:
            raise ValueError("Legacy/unknown schemas cannot enter native pipeline")
        validate_model_contract(header.get("model_contract"))
        if header.get("columns") != list(FEATURE_COLUMNS) + [MODEL_LABEL]:
            raise ValueError("Wrong native table columns")
        if not re.fullmatch(r"[a-f0-9]{64}", header.get("scope_sha256", "")):
            raise ValueError("Invalid native scope fingerprint")
        if expected_scope is not None and header.get("scope_sha256") != expected_scope:
            raise ValueError("Wrong demand scope")
        if expected_plan is not None and canonical_sha({name: header.get(name) for name in expected_plan}) != canonical_sha(expected_plan):
            raise ValueError("Table differs from frozen plan")
        limit = header.get("resource_limits", {}).get("max_payload_bytes")
        if limit not in (0, MAX_PAYLOAD):
            raise ValueError("Unsupported payload limit policy")
        count = integer(header.get("entry_count"), "entry count", 0, (limit or MAX_JSON_INTEGER) // RECORD.size)
        if header.get("payload_bytes") != count * RECORD.size:
            raise ValueError("Payload size/count mismatch")
        codec, digest, values, previous = KeyCodec(header), hashlib.sha256(), {}, None
        for _ in range(count):
            record = stream.read(RECORD.size)
            if len(record) != RECORD.size:
                raise ValueError("Truncated demand payload")
            digest.update(record)
            group, road, value = RECORD.unpack(record)
            key = group, road
            codec.decode(key)
            if previous is not None and key <= previous:
                raise ValueError("Unsorted/duplicate demand keys")
            if not math.isfinite(value) or value < 1:
                raise ValueError("Invalid demand prediction")
            previous = key
            if retain_values:
                values[key] = value
        if stream.read(1) or digest.hexdigest() != header.get("payload_sha256"):
            raise ValueError("Trailing/corrupt demand payload")
    return header, values


def load_predictor(path):
    from autogluon.tabular import TabularPredictor
    return TabularPredictor.load(path)


def predict_keys(predictor, plan, keys, chunk_size):
    import numpy as np
    import pandas as pd
    codec, keys, values = KeyCodec(plan), sorted(keys), {}
    if getattr(predictor, "label", None) != MODEL_LABEL:
        raise ValueError("Predictor label differs from native contract")
    metadata = getattr(predictor, "feature_metadata", None)
    observed = metadata.get_features() if metadata is not None else None
    expected = plan["model_contract"]["framework_raw_features"]
    if (type(observed) is not list or not observed or len(observed) != len(set(observed))
            or not set(observed).issubset(FEATURE_COLUMNS) or observed != expected):
        raise ValueError("Predictor effective raw features differ from native model contract")
    for start in range(0, len(keys), chunk_size):
        batch = keys[start:start + chunk_size]
        frame = encode_features(pd.DataFrame([codec.decode(key) for key in batch], columns=FEATURE_COLUMNS))
        raw = np.asarray(predictor.predict(frame), dtype=np.float64)
        if raw.shape != (len(batch),):
            raise ValueError("Predictor returned wrong result shape")
        for key, value in zip(batch, raw):
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("Predictor returned nonfinite value")
            values[key] = max(1., value)
    return values


def write_readable(path, plan, values, delimiter=","):
    codec = KeyCodec(plan)
    with Path(path).open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, delimiter=delimiter)
        writer.writerow(["group_index", *FEATURE_COLUMNS, MODEL_LABEL])
        for key in sorted(values):
            writer.writerow([key[0], *codec.decode(key), values[key]])


def route_vehicles(path):
    definitions, vehicles = {}, {}
    stack, root = [], None
    for event, element in ET.iterparse(path, events=("start", "end")):
        if event == "start":
            stack.append(element)
            if root is None:
                root = element
                if root.tag != "routes":
                    raise ValueError("Expected routes XML root")
            if element.tag in ("flow", "trip", "person", "personFlow", "routeDistribution", "include"):
                raise ValueError("Demand replay requires resolved explicit vehicle routes")
            continue
        if len(stack) == 2:
            if element.tag == "route":
                name = element.get("id")
                edges = tuple(map(sys.intern, element.get("edges", "").split()))
                if not name or name in definitions or not edges:
                    raise ValueError("Invalid/duplicate route definition")
                definitions[name] = edges
            elif element.tag == "vehicle":
                name, inline = element.get("id"), element.find("route")
                edges = (tuple(map(sys.intern, inline.get("edges", "").split())) if inline is not None
                         else definitions.get(element.get("route")))
                if not name or name in vehicles or not edges:
                    raise ValueError("Invalid/duplicate vehicle or unresolved route")
                if element.get("departEdge") not in (None, "0") or element.get("arrivalEdge") is not None:
                    raise ValueError("Partial vehicle routes are outside demand replay scope")
                vehicles[name] = edges
            root.remove(element)
            element.clear()
        stack.pop()
    if not vehicles:
        raise ValueError("Empty explicit route input")
    return vehicles


def trip_durations(path):
    result = {}
    root = None
    for event, trip in ET.iterparse(path, events=("start", "end")):
        if event == "start":
            if root is None:
                root = trip
                if root.tag != "tripinfos":
                    raise ValueError("Expected tripinfos XML root")
            continue
        if trip.tag != "tripinfo":
            continue
        name, value = trip.get("id"), float(trip.get("duration", "nan"))
        if not name or name in result or not math.isfinite(value) or value < 0:
            raise ValueError("Invalid/duplicate tripinfo record")
        if trip.get("vaporized", "") or float(trip.get("arrival", "0")) < 0:
            raise ValueError("Non-normal/unfinished tripinfo")
        result[name] = value
        root.remove(trip)
        trip.clear()
    if not result:
        raise ValueError("Empty tripinfo")
    return result


def prepare_scenarios(args):
    scenarios, names = [], set()
    for name, route, truth in args.scenario or []:
        if not re.fullmatch(r"[A-Za-z0-9_-]+", name) or name in names:
            raise ValueError("Unsafe/duplicate scenario name")
        names.add(name)
        vehicles, durations = route_vehicles(route), trip_durations(truth)
        if set(vehicles) != set(durations):
            raise ValueError("Scenario route/truth vehicle IDs differ")
        scenarios.append({"name": name, "route": signature(route), "truth": signature(truth),
                          "vehicle_count": len(vehicles), "road_count": sum(map(len, vehicles.values()))})
        # Keep only signatures/counts between scenarios. Full route identities
        # are loaded one scenario at a time for its final streaming validation.
        del vehicles, durations
    return scenarios


def command_for(args, scenario, cache, directory, discovery):
    command = [str(Path(args.simulator).resolve()), "--use-sumo", "--travel-time-mode", "table",
               "--movement-queue-order", "arrival-time", "--lane-discharge-interval", str(args.lane_discharge_interval),
               "--movement-grouping", args.movement_grouping,
               "--lane-entry-reselection", args.lane_entry_reselection, "--lane-borrowing", args.lane_borrowing,
               "--intersection-crossing-time", str(args.crossing_time),
               "--speed-net-queue-trim", "off",
               "--first-road-position-correction", args.first_road_position_correction,
               "--speed-net-headway", "0.6", "--speed-net-delay-max", "3", "--speed-net-components", "both",
               *args.simulator_arg,
               "--sumo-net", str(Path(args.net_file).resolve()), "--sumo-route", scenario["route"]["path"],
               "--sumo-tripinfo", scenario["truth"]["path"], "--travel-time-table", str(cache),
               "--expected-signal-remaining-step", str(args.signal_remaining_step),
               "--prediction-tripinfo-output", str(directory / "predictions.xml"),
               "--eval-output", str(directory / "evaluation.csv"),
               "--demand-status-output", str(directory / "status.json"),
               "--demand-max-missing-keys", str(args.max_cache_entries),
               "--read-num", str(scenario["vehicle_count"])]
    if discovery:
        command.extend(["--demand-requests-output", str(directory / "missing.csv")])
    else:
        command.extend(["--cap-context-trace", str(directory / "trace.csv")])
    return command


def run_process(command, log, timeout, min_free_bytes=0):
    with Path(log).open("x", encoding="utf-8") as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
        try:
            started = time.monotonic()
            while True:
                resume_store.require_space(Path(log).parent, min_free_bytes)
                left = timeout - (time.monotonic() - started)
                if left <= 0:
                    raise subprocess.TimeoutExpired(command, timeout)
                try:
                    return process.wait(timeout=min(5., left))
                except subprocess.TimeoutExpired:
                    pass
        except BaseException:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            raise


def check_status(path, scope, scenario, discovery):
    status = read_json(path)
    if (status.get("schema") != "native-road-time-demand-status-v2" or status.get("scope_sha256") != scope
            or status.get("complete") is not True or status.get("discovery_enabled") is not discovery):
        raise ValueError("Wrong/incomplete native replay status")
    for field, expected in (("vehicle_count", scenario["vehicle_count"]), ("finished_count", scenario["vehicle_count"]),
                            ("query_count", scenario["road_count"]), ("expected_queries", scenario["road_count"]),
                            ("invalid_count", 0)):
        if type(status.get(field)) is not int or status[field] != expected:
            raise ValueError(f"Replay {field} does not match exact scenario coverage")
    for field in ("missing_queries", "missing_unique"):
        integer(status.get(field), field, 0, scenario["road_count"])
    if status["missing_unique"] > status["missing_queries"] or status.get("exact") is not (status["missing_queries"] == 0):
        raise ValueError("Replay missing/exact counts disagree")
    if not discovery and (status["missing_queries"] or status["missing_unique"]):
        raise ValueError("Strict native replay contains missing keys")
    return status


def check_final(directory, scenario):
    vehicles = scenario.get("vehicles")
    if vehicles is None:
        vehicles = route_vehicles(scenario["route"]["path"])
    truth = scenario.get("truth_durations")
    if truth is None:
        truth = trip_durations(scenario["truth"]["path"])
    predictions = trip_durations(directory / "predictions.xml")
    if set(predictions) != set(vehicles) or set(truth) != set(vehicles):
        raise ValueError("Strict predicted vehicle IDs do not match scenario")
    # O(number of vehicles) validation state, never a set/dict per road visit.
    last_index = dict.fromkeys(vehicles, -1)
    count = 0
    with (directory / "trace.csv").open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not {"sumoVehicleID", "route_index", "sumoRoadID"}.issubset(reader.fieldnames or []):
            raise ValueError("Strict trace lacks occurrence identity")
        for row in reader:
            name, index = row["sumoVehicleID"], int(row["route_index"])
            route = vehicles.get(name)
            if (route is None or index != last_index[name] + 1 or index >= len(route)
                    or row["sumoRoadID"] != route[index]):
                raise ValueError("Duplicate/wrong strict trace occurrence")
            last_index[name] = index
            count += 1
    if count != scenario["road_count"] or any(last_index[name] != len(route) - 1 for name, route in vehicles.items()):
        raise ValueError("Strict trace misses route occurrences")
    errors = [predictions[name] - truth[name] for name in sorted(predictions)]
    return {"name": scenario["name"], "vehicle_count": len(errors), "road_count": count,
            "mae_seconds": math.fsum(map(abs, errors)) / len(errors), "bias_seconds": math.fsum(errors) / len(errors),
            "sum_absolute_error_seconds": math.fsum(map(abs, errors)), "sum_error_seconds": math.fsum(errors)}


def verify_sources(sources):
    for before in sources:
        if signature(before["path"]) != before:
            raise ValueError("Source changed during native catch: " + before["path"])


def verify_model_tree(plan, files):
    root = Path(plan["model"]["contract_path"]).parent
    current = {str(path.resolve()) for path in root.rglob("*") if path.is_file()}
    if current != {item["path"] for item in files}:
        raise ValueError("Model artifact file set changed during native catch")


def run(args, predictor_loader=None, runner=None):
    model_files = []
    plan = build_plan(args, source_files=model_files)
    integer(args.max_rounds, "max rounds", 1, 1000)
    number(args.simulation_timeout, "simulation timeout", True)
    number(args.lane_discharge_interval, "lane discharge interval", True)
    if (args.movement_grouping == "road-pair" and args.lane_borrowing != "off") or (
            args.lane_entry_reselection == "on" and (args.movement_grouping != "road-pair" or args.lane_borrowing != "off")):
        raise ValueError("Incompatible movement/lane proxy deployment options")
    scenarios = prepare_scenarios(args)
    if args.requests and scenarios:
        raise ValueError("Use either request export or closed-loop scenarios")
    if scenarios and not args.simulator:
        raise ValueError("Closed-loop demand requires explicit simulator")
    reserved = {"--travel-time-table", "--demand-requests-output", "--demand-status-output", "--sumo-net", "--sumo-route",
                "--sumo-tripinfo", "--cap-context-trace", "--prediction-tripinfo-output", "--eval-output", "--read-num",
                "--expected-signal-remaining-step", "--travel-time-mode", "--intersection-crossing-time",
                "--lane-discharge-interval", "--movement-grouping", "--lane-entry-reselection", "--lane-borrowing",
                "--demand-max-missing-keys", "--first-road-position-correction", "--speed-net-depart-position",
                "--demand-run-guard", "--demand-run-token"}
    if any(value.split("=", 1)[0] in reserved for value in args.simulator_arg):
        raise ValueError("Simulator extra args may not override frozen IO/contract options")
    sources = [signature(args.net_file), signature(__file__), signature(native_contract_module.__file__),
               signature(resume_store.__file__), *model_files]
    if args.requests:
        sources.append(signature(args.requests))
    if scenarios:
        sources.append(signature(args.simulator))
        sources.extend(item[field] for item in scenarios for field in ("route", "truth"))
    runtime = {"extra_arguments": args.simulator_arg, "movement_grouping": args.movement_grouping,
               "first_road_position_correction": args.first_road_position_correction,
               "cap_out_of_range": args.cap_out_of_range,
               "lane_entry_reselection": args.lane_entry_reselection, "lane_borrowing": args.lane_borrowing,
               "lane_discharge_interval": args.lane_discharge_interval, "intersection_crossing_time": args.crossing_time,
               "max_cache_entries": args.max_cache_entries, "max_rounds": args.max_rounds,
               "simulation_timeout": args.simulation_timeout, "prediction_chunk_size": args.chunk_size,
               "text_format": args.text_format, "min_free_disk_gib": args.min_free_disk_gib,
               "checkpoint_version": "native-catch-sqlite-v1", "discovery_trace": False}
    plan_sha = canonical_sha(plan)
    if args.requests:
        if args.requests_plan_sha256 != plan_sha:
            raise ValueError("Requests require --requests-plan-sha256 from the exact native dry-run plan")
    scope = canonical_sha({"plan": plan, "sources": sources, "runtime": runtime,
                           "scenarios": [{name: item[name] for name in ("name", "route", "truth", "vehicle_count", "road_count")}
                                         for item in scenarios]})
    output = Path(args.output_dir)
    if output.exists() and not args.resume:
        raise ValueError("Output already exists; no overwrite/resume")
    if args.resume and not output.is_dir():
        raise ValueError("Resume requires an existing checkpoint directory")
    if args.dry_run:
        return {"status": "dry_run", "scope_sha256": scope, "plan_sha256": plan_sha, "plan": plan, "runtime": runtime,
                "size_estimate": resume_store.estimate(plan, scenarios, {"total": 0, "predicted": 0, "pending": 0}, []),
                "scenarios": len(scenarios), "model_loaded": False, "files_written": False}
    if not args.requests and not scenarios:
        raise ValueError("Export requires --requests or --scenario; use --dry-run for planning")
    return resume_store.execute(args, plan, scope, plan_sha, sources, model_files, scenarios,
                                predictor_loader or load_predictor, runner)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--expected-threshold", choices=THRESHOLD_ORDER)
    parser.add_argument("--all-thresholds", action="store_true", help="Model root contains gt_1,gt_3,gt_0p1,gt_0p5,gt_1p5,gt_2")
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--signal-remaining-step", type=float, required=True)
    parser.add_argument("--road-flow-step", type=int, default=1)
    parser.add_argument("--road-flow-dense-until", type=int, default=10)
    parser.add_argument("--road-flow-segments", nargs="+", metavar="UPPER:STEP")
    parser.add_argument("--cap-step", type=float, default=.1389)
    parser.add_argument("--cap-min", type=float)
    parser.add_argument("--cap-max", type=float)
    parser.add_argument("--cap-out-of-range", choices=("clamp", "error"), default="clamp",
                        help="Positive finite cap outside table nodes uses nearest endpoint by default; other inputs remain strict")
    parser.add_argument("--first-road-position-correction", "--speed-net-depart-position",
                        choices=("on", "off"), default="on",
                        help="C++ first-road distance correction for formula fallback and native ABC clocks; use off for Manhattan")
    parser.add_argument("--max-road-flow", type=int, default=222)
    parser.add_argument("--max-lane-flow", type=int, default=80)
    parser.add_argument("--chunk-size", type=int, default=50_000)
    parser.add_argument("--max-cache-entries", type=int, default=0,
                        help="Optional unique-key limit; 0 (default) means no application key ceiling")
    parser.add_argument("--resume", action="store_true", help="Verify and reuse this exact scope's committed checkpoint")
    parser.add_argument("--min-free-disk-gib", type=float, default=10.,
                        help="Stop safely below this remaining output-disk space; not a key-count ceiling")
    parser.add_argument("--max-rounds", type=int, default=30)
    parser.add_argument("--simulation-timeout", type=float, default=1800.)
    parser.add_argument("--intersection-crossing-time", "--crossing-time", dest="crossing_time", type=float, default=3.,
                        help="Explicit deployment proxy constant; default 3 is not a claim of optimality")
    parser.add_argument("--lane-discharge-interval", type=float, default=2.)
    parser.add_argument("--movement-grouping", choices=("road-pair", "legacy-link"), default="road-pair")
    parser.add_argument("--lane-entry-reselection", choices=("on", "off"), default="on")
    parser.add_argument("--lane-borrowing", choices=("on", "off"), default="off")
    parser.add_argument("--allow-static-extrapolation", action="store_true")
    parser.add_argument("--requests", help="group_index,road_flow CSV produced for the exact printed plan")
    parser.add_argument("--requests-plan-sha256", help="Required request-export binding to dry-run plan_sha256")
    parser.add_argument("--simulator")
    parser.add_argument("--scenario", nargs=3, action="append", metavar=("NAME", "ROUTE", "TRIPINFO"))
    parser.add_argument("--simulator-arg", action="append", default=[], help="Additional simulator argument; pass option tokens with = syntax")
    parser.add_argument("--text-format", choices=("none", "csv", "txt", "both"), default="none")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print plan; no model load, simulation, or output")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        if args.all_thresholds:
            if args.expected_threshold:
                raise ValueError("all-thresholds conflicts with expected-threshold")
            if args.requests:
                raise ValueError("all-thresholds request export needs distinct per-model plan bindings; invoke each threshold separately")
            root, output = Path(args.model_dir), Path(args.output_dir)
            configurations = []
            for threshold in THRESHOLD_ORDER:
                current = argparse.Namespace(**vars(args))
                current.model_dir, current.output_dir = str(root / model_directory_name(threshold)), str(output / model_directory_name(threshold))
                current.expected_threshold = threshold
                if args.resume:
                    current.resume = Path(current.output_dir).exists()
                configurations.append(current)
            previews = []
            for current in configurations:
                preview = argparse.Namespace(**vars(current))
                preview.dry_run = True
                previews.append(run(preview))
            results = previews if args.dry_run else [run(current) for current in configurations]
            print(json.dumps(results, ensure_ascii=False, allow_nan=False))
        else:
            print(json.dumps(run(args), ensure_ascii=False, allow_nan=False))
    except KeyboardInterrupt:
        print("Interrupted; partial artifacts are not accepted results", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError, ET.ParseError, subprocess.TimeoutExpired) as exc:
        print(f"Native catch failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
