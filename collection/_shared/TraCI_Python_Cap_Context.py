#!/usr/bin/env python3
"""Opt-in cap/context collector; original collectors and labels stay unchanged.

The five-feature files never contain SF, speed_limit or P/E. Their derivation
sources and causal feature timestamps go to separate audit CSVs and contracts.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from roadnet_paths import source_path

from features.cap_context_features import (
    FEATURE_COLS, FEATURE_SCHEMA, ContextConfig, ShadowLedger, ShadowRecord,
    free_speed_cap, travel_distance,
)
from features.turn_context import TURN_CONTEXT_COLUMNS, TURN_CONTEXT_CONTRACT, event_turn_metadata


IDENTITY_COLS = ["time", "vehicleID", "roadID", "laneIndex", "route_index", "is_final_road"]
DEFAULT_DESIRED_MAX_SPEED = 10000.0 / 3.6


def sumo_type_defaults(vehicle_class="passenger"):
    """SUMO 1.27 road-class defaults consumed by C++ and these features.

    Match data_preparation.cpp, not the legacy non-SUMO VehicleType defaults.
    Unsupported classes must not silently inherit passenger capabilities.
    """
    values = dict(maxSpeed=200. / 3.6, length=5., minGap=2.5,
                  desiredMaxSpeed=DEFAULT_DESIRED_MAX_SPEED)
    if vehicle_class in {"passenger", "private", "vip", "hov", "taxi", "evehicle", "custom1", "custom2"}:
        return values
    if vehicle_class in {"delivery", "emergency"}:
        values["length"] = 6.5
    elif vehicle_class in {"truck", "trailer"}:
        values.update(maxSpeed=130. / 3.6, length=7.1 if vehicle_class == "truck" else 16.5)
    elif vehicle_class in {"bus", "coach"}:
        values.update(maxSpeed=100. / 3.6, length=12. if vehicle_class == "bus" else 14.)
    elif vehicle_class == "motorcycle":
        values["length"] = 2.2
    elif vehicle_class == "moped":
        values.update(maxSpeed=60. / 3.6, length=2.1)
    elif vehicle_class == "bicycle":
        values.update(maxSpeed=50. / 3.6, desiredMaxSpeed=20. / 3.6, length=1.6, minGap=.5)
    else:
        raise ValueError(f"unsupported SUMO vClass {vehicle_class!r}; no passenger default substitution")
    return values


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source_file(path):
    resolved = Path(path).resolve()
    return {"path": str(resolved), "sha256": file_sha256(resolved)}


class RouteMetadata:
    """Small per-run metadata, separate from the label-validity filter."""

    def __init__(self, sumo_config):
        config_path = Path(sumo_config).resolve()
        root = ET.parse(config_path).getroot()
        self.source_paths = []
        self.vehicle_options = {}
        self.type_options = {}
        for tag in ("additional-files", "route-files"):
            node = root.find(f"./input/{tag}")
            if node is None:
                continue
            for token in node.get("value", "").split(","):
                if not token.strip():
                    continue
                path = (config_path.parent / token.strip()).resolve()
                self.source_paths.append(path)
                for _, element in ET.iterparse(path, events=("end",)):
                    if element.tag == "vType":
                        self.type_options[element.get("id", "")] = dict(element.attrib)
                    elif element.tag == "vehicle":
                        vehicle_id = element.get("id", "")
                        if vehicle_id in self.vehicle_options:
                            raise ValueError(f"duplicate route vehicle ID {vehicle_id!r}")
                        self.vehicle_options[vehicle_id] = dict(element.attrib)
                    elif element.tag in ("flow", "trip", "vTypeDistribution"):
                        raise ValueError("cap/context v1 requires explicit vehicle routes and vTypes")
                    element.clear()
        if not self.vehicle_options:
            raise ValueError("no explicit route vehicles found in SUMO configuration")

    def for_vehicle(self, vehicle_id, runtime_type_id):
        if vehicle_id not in self.vehicle_options:
            raise ValueError(f"vehicle {vehicle_id!r} is missing explicit route metadata")
        options = self.vehicle_options[vehicle_id]
        try:
            declared_sf = float(options["speedFactor"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"cap/context requires an explicit numeric route speedFactor: {vehicle_id}") from exc
        if not math.isfinite(declared_sf) or declared_sf <= 0:
            raise ValueError(f"invalid explicit route speedFactor: {vehicle_id}")
        if options.get("departPos", "base") != "base":
            raise ValueError(f"base-first-v1 requires departPos=base: {vehicle_id}")
        if options.get("departEdge", "0") not in ("", "0"):
            raise ValueError("base-first-v1 does not support nonzero departEdge")
        if options.get("arrivalPos", "max") not in ("", "max"):
            raise ValueError("base-first-v1 does not support non-max arrivalPos")
        if "arrivalEdge" in options:
            raise ValueError("base-first-v1 does not support arrivalEdge")
        type_id = options.get("type", "DEFAULT_VEHTYPE")
        if type_id != runtime_type_id:
            raise ValueError(f"runtime vType differs from route: {vehicle_id}: {type_id}/{runtime_type_id}")
        type_options = self.type_options.get(type_id)
        if type_options is None and type_id != "DEFAULT_VEHTYPE":
            raise ValueError(f"missing explicit vType {type_id!r}")
        type_options = type_options or {}
        defaults = sumo_type_defaults(type_options.get("vClass", "passenger"))
        desired = float(type_options.get("desiredMaxSpeed", defaults["desiredMaxSpeed"]))
        return options, desired


class LogicalNetwork:
    def __init__(self, net_file):
        self.lengths, self.speed_limits, self.lane_counts = {}, {}, {}
        self.from_lanes, self.to_lanes = {}, {}
        root = ET.parse(net_file).getroot()
        for edge in root.findall("edge"):
            if "function" in edge.attrib:
                continue
            edge_id = edge.get("id")
            lanes = edge.findall("lane")
            if not lanes:
                raise ValueError(f"external edge has no lanes: {edge_id}")
            lengths = [float(lane.get("length", "0")) for lane in lanes]
            speeds = [float(lane.get("speed", "0")) for lane in lanes]
            if min(lengths) <= 0 or min(speeds) <= 0 or not all(math.isfinite(x) for x in lengths + speeds):
                raise ValueError(f"invalid external lane geometry: {edge_id}")
            # C++ stores an edge average, legacy labels use the selected lane.
            # Current data are uniform; do not silently pretend future unequal
            # lanes have an already-defined cap/context geometry contract.
            if max(lengths) - min(lengths) > 1e-9 or max(speeds) - min(speeds) > 1e-9:
                raise ValueError(f"cap/context v1 requires uniform lane length/speed within {edge_id}")
            self.lengths[edge_id] = sum(lengths) / len(lengths)
            self.speed_limits[edge_id] = sum(speeds) / len(speeds)
            self.lane_counts[edge_id] = len(lanes)
        for connection in root.findall("connection"):
            pair = connection.get("from"), connection.get("to")
            if pair[0] not in self.lengths or pair[1] not in self.lengths:
                continue
            self.from_lanes.setdefault(pair, set()).add(int(connection.get("fromLane", "0")))
            self.to_lanes.setdefault(pair, set()).add(int(connection.get("toLane", "0")))

    def choose_lane(self, route, route_index, ledger):
        road = route[route_index]
        outgoing = sorted(self.from_lanes.get((road, route[route_index + 1]), ())) if route_index + 1 < len(route) else []
        if route_index == 0:
            candidates = outgoing or list(range(self.lane_counts[road]))
        else:
            incoming = sorted(self.to_lanes.get((route[route_index - 1], road), ()))
            intersection = sorted(set(incoming).intersection(outgoing))
            candidates = (intersection or outgoing) if outgoing else incoming
            candidates = candidates or incoming or [0]
        if any(lane < 0 or lane >= self.lane_counts[road] for lane in candidates):
            raise ValueError(f"invalid connection lane on {road}: {candidates}")
        # The shadow ledger never blocks a real SUMO transition. It mirrors
        # the candidate mapping and least occupied selector, not C++ storage
        # admission decisions or microscopic lane changes.
        return min(candidates, key=lambda lane: (*ledger.lane_load(road, lane), lane))


class CapContextObserver:
    """Causal shadow snapshots made before the existing label loop runs."""

    # Versioned collectors may extend feature-only diagnostics without changing
    # SUMO stepping, label accumulation, flow snapshots or the v1 defaults.
    feature_schema = FEATURE_SCHEMA
    feature_columns = tuple(FEATURE_COLS)
    context_ids = tuple(range(4))
    extra_audit_fields = ()

    def feature_context_contract(self):
        return self.context_config.as_contract()

    def __init__(self, api, sumo_config, net_file, config=ContextConfig(), *, network_factory=None):
        self.api = api
        self.context_config = config
        self.metadata = RouteMetadata(sumo_config)
        self.network = (LogicalNetwork(net_file) if network_factory is None
                        else network_factory(net_file, self.metadata))
        self.ledger = ShadowLedger(config)
        self.previous = {}
        self.routes = {}
        self.derivation = {}
        self.frozen = {}
        self.interrupted = set()
        self.last_time = -math.inf
        self.reservations = 0
        self.feature_row_counts = {}

    def _vehicle_data(self, vehicle_id):
        if vehicle_id not in self.derivation:
            runtime_type = self.api.vehicle.getTypeID(vehicle_id)
            options, desired = self.metadata.for_vehicle(vehicle_id, runtime_type)
            sf = float(self.api.vehicle.getSpeedFactor(vehicle_id))
            if sf != float(options["speedFactor"]):
                raise ValueError(f"runtime speedFactor differs from explicit route value: {vehicle_id}")
            maximum = float(self.api.vehicle.getMaxSpeed(vehicle_id))
            length = float(self.api.vehicle.getLength(vehicle_id))
            gap = float(self.api.vehicle.getMinGap(vehicle_id))
            declared_type = self.metadata.type_options.get(runtime_type, {})
            defaults = sumo_type_defaults(declared_type.get("vClass", "passenger"))
            # Compare live SUMO getters to the same route/default resolution as
            # C++, so a silent type override cannot contaminate formal inputs.
            for name, actual in (("maxSpeed", maximum), ("length", length), ("minGap", gap)):
                expected = float(declared_type.get(name, defaults[name]))
                if not math.isfinite(expected) or not math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-12):
                    raise ValueError(f"SUMO/C++ vType {name} mismatch for {vehicle_id}: {actual}/{expected}")
            # Validate all source values, including negative occupied lengths.
            free_speed_cap(sf, maximum, desired, 1.0)
            if not math.isfinite(length) or length <= 0 or not math.isfinite(gap) or gap < 0:
                raise ValueError(f"invalid vehicle dimensions: {vehicle_id}")
            self.derivation[vehicle_id] = {
                "speed_factor": sf, "vehicle_max_speed": maximum,
                "desired_max_speed": desired, "vehicle_length": length,
                "vehicle_min_gap": gap, "depart_pos": options.get("departPos", "base"),
                "vehicle_type": runtime_type,
            }
            self.routes[vehicle_id] = tuple(self.api.vehicle.getRoute(vehicle_id))
        return self.derivation[vehicle_id]

    def entry_crossing_seconds(self, route, route_index):
        """Historical default; versioned collectors may override this clock rule."""
        return self.context_config.crossing_time_seconds if route_index else 0.0

    def reserve(self, vehicle_id, route_index, decision_time):
        source = self._vehicle_data(vehicle_id)
        route = self.routes[vehicle_id]
        if route_index < 0 or route_index >= len(route):
            raise ValueError(f"unsupported route transition: {vehicle_id}/{route_index}")
        road = route[route_index]
        if road not in self.network.lengths:
            raise ValueError(f"unknown normal road {road!r}")
        limit = self.network.speed_limits[road]
        cap = free_speed_cap(source["speed_factor"], source["vehicle_max_speed"], source["desired_max_speed"], limit)
        distance = travel_distance(self.network.lengths[road], route_index=route_index,
                                   vehicle_length=source["vehicle_length"], depart_pos=source["depart_pos"])
        lane = self.network.choose_lane(route, route_index, self.ledger)
        enter = decision_time + self.entry_crossing_seconds(route, route_index)
        record = ShadowRecord(vehicle_id, route_index, road, lane, enter, distance, cap,
                              source["vehicle_length"] + source["vehicle_min_gap"])
        values = self.ledger.query(record, decision_time)
        values.update(source)
        values.update(free_speed_cap=cap, speed_limit=limit, road_id=road)
        if record.key in self.frozen:
            raise ValueError(f"duplicate frozen F13 occurrence: {record.key}")
        self.frozen[record.key] = values
        self.ledger.reserve(record)
        self.reservations += 1

    def on_step(self):
        now = float(self.api.simulation.getTime())
        if now < self.last_time:
            raise ValueError("SUMO observation time moved backwards")
        self.last_time = now
        ids = set(self.api.vehicle.getIDList())
        interrupted = set(self.api.simulation.getStartingTeleportIDList())
        self.interrupted.update(interrupted)
        # Disappearance removes storage even if its label is absent/invalid.
        for vehicle_id in sorted((set(self.previous) - ids) | interrupted):
            self.ledger.release(vehicle_id)
            self.previous.pop(vehicle_id, None)
        # Stable observation batch order; no Python set order affects snapshots.
        # Each simulator retains its own event stream, so this does not claim
        # that SUMO and C++ produce identical simultaneous release order.
        for vehicle_id in sorted(ids):
            if vehicle_id in self.interrupted:
                continue
            edge = self.api.vehicle.getRoadID(vehicle_id)
            index = int(self.api.vehicle.getRouteIndex(vehicle_id))
            old = self.previous.get(vehicle_id)
            if old is None:
                if edge.startswith(":") or index != 0:
                    raise ValueError(f"missing initial external-road decision: {vehicle_id}")
                self.reserve(vehicle_id, 0, now)
            elif not old[0].startswith(":") and (edge != old[0] or index != old[1]):
                self.ledger.release(vehicle_id, old[1])
                next_index = old[1] + 1
                # At the stop line, the next road/route/vehicle capability are
                # already known. Its future true edge-enter time is NOT read.
                self.reserve(vehicle_id, next_index, now)
                if not edge.startswith(":") and index != next_index:
                    raise ValueError(f"skipped route occurrence: {vehicle_id}")
            elif old[0].startswith(":") and not edge.startswith(":"):
                if (vehicle_id, index) not in self.frozen:
                    raise ValueError(f"missing upstream frozen context: {vehicle_id}/{index}")
            self.previous[vehicle_id] = edge, index

    def attach(self, event):
        key = event.vehicle_id, event.route_index
        if key not in self.frozen:
            raise ValueError(f"no causal cap/context snapshot for training event {key}")
        values = self.frozen[key]
        if values["road_id"] != event.from_edge:
            raise ValueError(f"road/context mismatch for {key}")
        if event.speed_factor != values["speed_factor"]:
            raise ValueError(f"dynamic speedFactor is not supported by cap/context v1: {key}")
        event.cap_context_snapshot = dict(values)


def _identity(event):
    return {
        "time": round(event.sampled_time if event.sampled_time is not None else event.edge_enter_time, 2),
        "vehicleID": event.vehicle_id, "roadID": event.from_edge,
        "laneIndex": event.lane_index if event.lane_index is not None else "",
        "route_index": event.route_index, "is_final_road": int(event.is_final_road),
    }


def write_feature_rows(output_path, completed_events, *, base, sweep, observer, source):
    path = Path(output_path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    audit_path = Path(str(path) + ".audit.csv")
    label_cols = sweep.candidate_label_columns(sweep.DEFAULT_LABEL_SPEED_THRESHOLDS)
    feature_cols = list(observer.feature_columns)
    if feature_cols[:len(FEATURE_COLS)] != FEATURE_COLS or len(set(feature_cols)) != len(feature_cols):
        raise ValueError("versioned collector must retain original feature prefix without duplicates")
    extension_cols = feature_cols[len(FEATURE_COLS):]
    if set(extension_cols).intersection(IDENTITY_COLS + ["travel_time_label"] + label_cols):
        raise ValueError("extension feature collides with identity or label")
    audit_fields = IDENTITY_COLS + TURN_CONTEXT_COLUMNS + [
        "speed_factor", "speed_limit", "vehicle_max_speed", "desired_max_speed", "vehicle_length",
        "vehicle_min_gap", "vehicle_type", "depart_pos", "free_speed_cap", "platoon_context", "P", "E",
        "decision_time", "planned_enter_time", "free_finish_time", "horizon_seconds", "travel_distance",
        "max_deficit_seconds", "sum_deficit_seconds", "candidate_count", "positive_count", "pending_count",
        "scanned_count", "logical_lane_index", "actual_edge_enter_time", "label_end_time",
    ] + sweep.speed_bin_columns(sweep.DEFAULT_LABEL_SPEED_THRESHOLDS) + ["speed_bin_time_total", "speed_bin_conservation_residual"]
    audit_fields += list(observer.extra_audit_fields)
    counts = {str(index): 0 for index in observer.context_ids}
    row_count = 0
    with path.open("w", encoding="utf-8", newline="") as stream, audit_path.open("w", encoding="utf-8", newline="") as audit:
        writer = csv.DictWriter(stream, fieldnames=IDENTITY_COLS + feature_cols + ["travel_time_label"] + label_cols)
        audit_writer = csv.DictWriter(audit, fieldnames=audit_fields)
        writer.writeheader()
        audit_writer.writeheader()
        for event in base.iter_valid_training_events(completed_events):
            values = getattr(event, "cap_context_snapshot", None)
            if values is None:
                raise ValueError(f"training event missing frozen F13: {event.vehicle_id}/{event.route_index}")
            extras = sweep._event_extra_values(event)
            row = _identity(event)
            row.update(road_length=round(event.road_length, 4), road_flow=event.road_flow or 0,
                       lane_flow=event.lane_flow or 0, free_speed_cap=values["free_speed_cap"],
                       platoon_context=values["platoon_context"],
                       travel_time_label=round(event.driving_time + event.low_speed_time, 4))
            row.update({column: extras[column] for column in label_cols})
            row.update({column: values[column] for column in extension_cols})
            if hasattr(observer, "formal_features"):
                row.update(observer.formal_features(event, values))
            writer.writerow(row)
            audit_row = _identity(event)
            audit_row.update(event_turn_metadata(event))
            audit_row.update({name: values[name] for name in audit_fields if name in values})
            audit_row.update(actual_edge_enter_time=event.edge_enter_time,
                             label_end_time=base.training_event_end_time(event))
            audit_row.update({name: extras[name] for name in audit_fields if name in extras})
            audit_writer.writerow(audit_row)
            counts[str(values["platoon_context"])] += 1
            row_count += 1
    contract = {
        "schema": observer.feature_schema, "raw_features": feature_cols,
        "context_contract": observer.feature_context_contract(),
        "turn_context_contract": TURN_CONTEXT_CONTRACT,
        "data_sha256": file_sha256(path), "source": source,
        "rows": row_count, "context_counts": counts,
        "sampling": {
            "road_flow_lane_flow": "original-actual-external-entry-snapshot-including-ego",
            "platoon_context": "upstream-stopline-decision-with-fixed-crossing-proxy; first=observed-insertion",
            "same_time_order": "SUMO observation batch: vehicleID lexical; no future batch events included",
            "lane_selection": "incoming-toLane/next-fromLane intersection else next-fromLane; least shadow occupancy",
            "labels": "unchanged threshold-sweep external-road sampling; independent of F13 ledger",
        },
        "audit": _source_file(audit_path),
        "shadow": {"reservations_all_vehicles": observer.reservations,
                   "queries": observer.ledger.query_count, "scanned_records": observer.ledger.scanned_records,
                   "remaining_records": len(observer.ledger)},
    }
    if hasattr(observer, "feature_signal_contract"):
        contract["signal_contract"] = observer.feature_signal_contract()
    if hasattr(observer, "collection_sampling"):
        contract["sampling"] = observer.collection_sampling()
    Path(str(path) + ".contract.json").write_text(json.dumps(contract, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    observer.feature_row_counts[str(path)] = row_count
    print(f"cap/context rows={row_count}, context_counts={counts}, audit={audit_path}")
    return row_count


def main(argv=None, *, observer_factory=CapContextObserver, collector_path=None,
         extra_source_paths=None, default_output_prefix="cap_context"):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--context-crossing-time", type=float, default=3.0)
    custom, remaining = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    context_config = ContextConfig(crossing_time_seconds=custom.context_crossing_time)
    import collection._shared.TraCI_Python_Adjusted as base
    import collection._shared.TraCI_Python_Threshold_Sweep as sweep
    _, thresholds, remaining = sweep._extract_custom_args(remaining)
    if tuple(thresholds) != tuple(sweep.DEFAULT_LABEL_SPEED_THRESHOLDS):
        raise ValueError("cap/context v1 requires the six fixed 0.1/0.5/1/1.5/2/3 thresholds")
    if "--help" in remaining or "-h" in remaining:
        print("Cap/context options (new, separate feature/audit outputs):")
        parser.print_help()
    for option, value in (("--outputs", "training"),
                          ("--training-output", f"cams_{default_output_prefix}_training_data.csv"),
                          ("--legacy-edge-output", f"{default_output_prefix}_training_data.csv")):
        if not any(arg == option or arg.startswith(option + "=") for arg in remaining):
            remaining += [option, value]

    original_argv = sys.argv
    original_parse = base.parse_args
    original_sample = base.sample_training_features
    original_step = base.traci.simulationStep
    original_accumulate = base.accumulate_waiting_time
    original_training = base.write_training_rows
    original_lightweight = base.write_lightweight_training_rows
    holder = {}

    def parse_config():
        config = original_parse()
        holder["config"] = config
        if "training" not in config.outputs and "legacy" not in config.outputs:
            raise ValueError("cap/context collector requires training and/or legacy feature output")
        observer = observer_factory(base.traci, config.sumo_config, config.net_file, context_config)
        holder["observer"] = observer
        if hasattr(observer, "collection_sampling"):
            for output in (config.training_output, config.legacy_edge_output):
                for suffix in ("", ".audit.csv", ".contract.json", ".failure.json"):
                    if Path(str(output) + suffix).exists():
                        raise ValueError(f"new sampling policy refuses an existing output: {output}{suffix}")
            for kind, output in (("events", config.vehicle_event_output), ("cycles", config.cycle_summary_output)):
                if kind in config.outputs and Path(output).exists():
                    raise ValueError(f"new sampling policy refuses an existing output: {output}")
            holder["outputs_validated"] = True
        holder["source"] = {
            "sumo_config": _source_file(config.sumo_config), "net_file": _source_file(config.net_file),
            "route_and_additional_files": [_source_file(path) for path in observer.metadata.source_paths],
            "collector": _source_file(collector_path or __file__),
            "common_features": _source_file(source_path("cap_context_features.py")),
            "base_collector": _source_file(base.__file__), "threshold_collector": _source_file(sweep.__file__),
            "turn_context": _source_file(source_path("turn_context.py")),
        }
        for name, path in (extra_source_paths or {}).items():
            if name in holder["source"]:
                raise ValueError(f"duplicate collection source name: {name}")
            holder["source"][name] = _source_file(path)
        return config

    def simulation_step(*args, **kwargs):
        result = original_step(*args, **kwargs)
        holder["observer"].on_step()
        return result

    def sample(event, *args, **kwargs):
        result = original_sample(event, *args, **kwargs)
        holder["observer"].attach(event)
        if hasattr(holder["observer"], "attach_observation"):
            holder["observer"].attach_observation(event, args[0], args[1])
        return result

    def writer(path, events):
        if hasattr(holder["observer"], "validate_completion"):
            holder["observer"].validate_completion()
            holder["observer"].validate_training_events(events, base)
        return write_feature_rows(path, events, base=base, sweep=sweep,
                                  observer=holder["observer"], source=holder["source"])

    sys.argv = [original_argv[0]] + list(remaining)
    base.parse_args = parse_config
    base.traci.simulationStep = simulation_step
    base.sample_training_features = sample
    base.accumulate_waiting_time = sweep.accumulate_waiting_time
    base.write_training_rows = base.write_lightweight_training_rows = writer
    previous_thresholds = sweep._label_speed_thresholds
    sweep._label_speed_thresholds = sweep.DEFAULT_LABEL_SPEED_THRESHOLDS
    try:
        base.main()
        print("cap/context formal model features=" + ",".join(holder["observer"].feature_columns))
        print("speed_factor/speed_limit/P/E are separate audit metadata, never formal model inputs")
    except Exception as exc:
        observer = holder.get("observer")
        if observer is not None and hasattr(observer, "collection_sampling") and holder.get("outputs_validated"):
            failure_path = Path(str(holder["config"].training_output) + ".failure.json")
            failure_path.parent.mkdir(parents=True, exist_ok=True)
            if not failure_path.exists():
                failure_path.write_text(json.dumps({
                    "status": "failed_no_valid_training_contract", "error": str(exc),
                    "sampling": observer.collection_sampling(),
                    "last_observed_time": observer.last_time if math.isfinite(observer.last_time) else None,
                    "reservations": observer.reservations, "remaining_records": len(observer.ledger),
                }, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        raise
    finally:
        base.parse_args = original_parse
        base.traci.simulationStep = original_step
        base.sample_training_features = original_sample
        base.accumulate_waiting_time = original_accumulate
        base.write_training_rows = original_training
        base.write_lightweight_training_rows = original_lightweight
        sweep._label_speed_thresholds = previous_thresholds
        sys.argv = original_argv
        try:
            base.traci.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
