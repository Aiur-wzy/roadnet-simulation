"""Batch-only transport adapter preserving the v1 discharge label algorithm.

The v1 implementation remains untouched. Only E1 reads are subscription-backed;
all lanes, physical gate decisions, vehicle getters and event ordering remain the
same. Natural drain is required; a safety-cap stop cannot be called complete.
Full route-ID/tripinfo certification remains the outer batch runner's duty.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import math
from pathlib import Path
import shutil
import uuid
import xml.etree.ElementTree as ET

from collection.discharge_sumo import (
    SCHEMA, LaneState, _hash, _import_traci, _prepare_isolated_inputs, _xml,
    _write_detectors, build_network_registry, classify_gate,
    connection_candidates, internal_path, permission_class,
    resolve_inputs, resolve_passage_connection, stopped_cohort, valid_speed,
)

ADAPTER_VERSION = "sumo-lane-outlet-batch-subscriptions-v1"


class ObservationHorizonExceeded(RuntimeError):
    def __init__(self, now, cap, remaining):
        self.now = float(now)
        self.cap = float(cap)
        self.remaining = int(remaining)
        super().__init__(
            "observation safety cap reached at %.6fs (cap %.6fs), %d expected "
            "vehicles remain; partial output is not a complete dataset"
            % (self.now, self.cap, self.remaining))


class E1SubscriptionReader:
    """Read exactly the getter variable for every physical lane, each step.

    SUMO's inductionloop domain decodes LAST_STEP_VEHICLE_DATA with the same
    readVehicleData function for getter and subscription replies. Missing
    subscription evidence is an error, never an assumed empty detector.
    """

    def __init__(self, domain, sections, variable):
        self.domain = domain
        self.variable = variable
        self.detector_ids = tuple(section["detector_id"] for section in sections.values())
        if len(set(self.detector_ids)) != len(self.detector_ids):
            raise ValueError("duplicate detector IDs")
        self.count = len(self.detector_ids)
        self.results = None
        for detector_id in self.detector_ids:
            domain.subscribe(detector_id, (variable,))

    def refresh(self):
        self.results = self.domain.getAllSubscriptionResults()
        if not isinstance(self.results, dict):
            raise RuntimeError("missing E1 subscription results")
        # Empty lanes must still have a known empty tuple. Validate all, so a
        # skipped lane can never silently turn absent evidence into no traffic.
        for detector_id in self.detector_ids:
            values = self.results.get(detector_id)
            if not isinstance(values, dict) or self.variable not in values:
                raise RuntimeError("missing E1 subscription for " + detector_id)
            if not isinstance(values[self.variable], (tuple, list)):
                raise RuntimeError("invalid E1 vehicle-data reply for " + detector_id)

    def vehicle_data(self, detector_id):
        if self.results is None:
            raise RuntimeError("E1 subscription cache has not been refreshed")
        return self.results[detector_id][self.variable]


def _record_simulation_counters(simulation, counts):
    counts["departed"] += simulation.getDepartedNumber()
    counts["arrived"] += simulation.getArrivedNumber()
    counts["teleport_start"] += simulation.getStartingTeleportNumber()
    counts["teleport_end"] += simulation.getEndingTeleportNumber()
    # Number of collision records is distinct from number of involved vehicles.
    counts["collision_events"] += len(simulation.getCollisions())
    counts["colliding_vehicle_reports"] += simulation.getCollidingVehiclesNumber()


def _prepare_batch_isolated_inputs(inputs, output_dir):
    """Compatibility for a boolean which the historical isolator treats as a path.

    Only a private clone is changed. Both historical and new adapters force
    --no-step-log true on their command line, so removing that redundant XML
    entry cannot alter the effective option. Original input hashes and config
    values stay in the metadata; unknown file/log options remain rejected.
    """
    adjusted = dict(inputs)
    changes = []
    if inputs["sumo_config"]:
        root = _xml(inputs["sumo_config"])
        for group in root:
            for element in list(group):
                if element.tag == "no-step-log":
                    changes.append({"tag": element.tag,
                                    "declared_value": element.attrib.get("value", ""),
                                    "effective_value": "true",
                                    "reason": "same explicit CLI override as historical adapter"})
                    group.remove(element)
        if changes:
            clone = Path(output_dir) / "input_config_log_compat.sumocfg"
            ET.ElementTree(root).write(clone, encoding="utf-8", xml_declaration=True)
            adjusted["sumo_config"] = str(clone)
    isolated_config, additions = _prepare_isolated_inputs(adjusted, output_dir)
    return isolated_config, additions, changes


def iter_sumo_events(config):
    """Observe unchanged dynamics until natural drain, with a safety cap.

    Same config contract as the v1 adapter; end is a required finite safety cap,
    not a successful partial observation target. Any remaining demand at that
    cap raises ObservationHorizonExceeded, unless diagnostic_horizon=True.
    Diagnostic horizon output is explicitly incomplete and not training data.
    No existing SUMO connection is used or modified.
    """
    end = float(config["end"])
    if not math.isfinite(end) or end <= 0:
        raise ValueError("end must be finite and positive")
    progress_callback = config.get("progress_callback")
    if progress_callback is not None and not callable(progress_callback):
        raise ValueError("progress_callback must be callable")
    progress_interval = float(config.get("progress_interval", 30.0))
    if not math.isfinite(progress_interval) or progress_interval <= 0:
        raise ValueError("progress_interval must be finite and positive")
    diagnostic_horizon = config.get("diagnostic_horizon", False)
    if not isinstance(diagnostic_horizon, bool):
        raise ValueError("diagnostic_horizon must be boolean")
    epsilon = float(config.get("epsilon_zero", 1e-7))
    stop_duration = float(config.get("stop_duration", 1.0))
    if epsilon < 0 or stop_duration < 0 or not math.isfinite(epsilon + stop_duration):
        raise ValueError("Invalid zero-speed or stopped-duration threshold")
    inputs = resolve_inputs(config)
    if end <= float(inputs["config_values"].get("begin", 0)):
        raise ValueError("end must be after the configured simulation begin")
    for option, default in (("cohort_distance", 100), ("queue_gap", 12), ("max_start_distance", 15)):
        value = float(config.get(option, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(option + " must be finite and positive")
    output_dir = Path(config["output_dir"]).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    registry = build_network_registry(inputs["net"], float(config.get("detector_offset", 0.0)))
    detector_file = _write_detectors(registry, output_dir)
    isolated_config, isolated_additionals, isolation_changes = _prepare_batch_isolated_inputs(inputs, output_dir)
    traci = _import_traci()
    command = [str(config.get("sumo_binary") or shutil.which("sumo") or "sumo")]
    if isolated_config:
        command += ["--configuration-file", isolated_config]
    command += ["--net-file", inputs["net"], "--route-files", ",".join(inputs["routes"]),
                "--additional-files", ",".join([*isolated_additionals, detector_file]),
                "--end", str(end), "--no-step-log", "true",
                "--threads", "1", "--tripinfo-output", str(output_dir / "tripinfo.xml"),
                "--tripinfo-output.write-unfinished", "true",
                "--error-log", str(output_dir / "sumo.stderr.log")]
    if config.get("seed") is not None:
        command += ["--seed", str(config["seed"])]
    if config.get("step_length") is not None:
        step = float(config["step_length"])
        if not math.isfinite(step) or step <= 0:
            raise ValueError("step_length must be finite and positive")
        command += ["--step-length", str(step)]
    metadata = config.setdefault("metadata", {})
    paths = [inputs["net"], *inputs["routes"], *inputs["additional_files"]]
    if inputs["sumo_config"]:
        paths.append(inputs["sumo_config"])
    run_id = str(config.get("run_id") or uuid.uuid4())
    metadata.update({"adapter_version": ADAPTER_VERSION, "run_id": run_id,
                     "completed": False,
                     "output_isolation_adjustments": isolation_changes,
                     "tripinfo_path": str(output_dir / "tripinfo.xml"),
                     "error_log": str(output_dir / "sumo.stderr.log"),
                     "command": command, "input_files": [{"path": path, "name": Path(path).name,
                        "role": "net" if path == inputs["net"] else "route" if path in inputs["routes"] else "sumo_config" if path == inputs["sumo_config"] else "additional",
                        "sha256": _hash(path)} for path in paths],
                     "config_values": inputs["config_values"],
                     "measurement_method": "E1-front-entry", "detector_offset": float(config.get("detector_offset", 0.0)),
                     "gate_policy_version": "strict-sampled-space-v1",
                     "observation": {"detector_offset": float(config.get("detector_offset", 0.0)),
                                     "epsilon_zero": epsilon, "stop_duration": stop_duration,
                                     "cohort_distance": float(config.get("cohort_distance", 100)),
                                     "queue_gap": float(config.get("queue_gap", 12)),
                                     "max_start_distance": float(config.get("max_start_distance", 15))},
                     "network_lane_count": len(registry["lanes"]),
                     "observed_lane_count": len(registry["sections"]),
                     "connection_count": sum(map(len, registry["connections"].values())),
                     "unsignalized_connection_count": sum(not item.tls for values in registry["connections"].values() for item in values),
                     "sections": registry["sections"], "vehicle_types": {},
                     "limitations": ["Clear labels are sampled evidence, not a guarantee between simulation steps.",
                                     "Finite complete stopped cohorts only; later arrivals never extend an old startup prefix.",
                                     "Ambiguous connections and external internal-lane traffic are conservatively excluded."]})
    label = "discharge-" + uuid.uuid4().hex
    traci.start(command, label=label, doSwitch=False)
    connection = traci.getConnection(label)
    counts = Counter()
    lane_states = {lane_id: LaneState() for lane_id in registry["sections"]}
    previous, visit_number, latest_lane_key = {}, Counter(), {}
    detector_seen = set()
    now = float(connection.simulation.getTime())
    try:
        metadata["sumo_version"] = list(connection.getVersion())
        metadata["effective_step_length"] = connection.simulation.getDeltaT()
        detectors = E1SubscriptionReader(
            connection.inductionloop, registry["sections"],
            traci.constants.LAST_STEP_VEHICLE_DATA)
        metadata["transport"] = "traci-e1-subscriptions-v1"
        metadata["detector_subscriptions"] = detectors.count
        metadata["completed"] = False
    except BaseException:
        connection.close()
        raise
    metadata["observation_begin"] = now
    values = inputs["config_values"]
    metadata["effective_options"] = {
        "seed": config["seed"] if config.get("seed") is not None else values.get("seed", "SUMO-default"),
        "random": values.get("random", "SUMO-default-false"),
        "step_length": metadata["effective_step_length"],
        "integrator": values.get("step-method.ballistic", "SUMO-default-semi-implicit-Euler"),
        "action_step_length": "per-type-observed-in-vehicle_types",
        "time_to_teleport": values.get("time-to-teleport", "SUMO-default"),
        "ignore_junction_blocker": values.get("ignore-junction-blocker", "SUMO-default"),
        "no_internal_links": values.get("no-internal-links", "SUMO-default"),
        "end": end, "verified": False,
        "threads": 1,
        "verification_limit": "Unspecified SUMO defaults and per-vehicle overrides retained, not all getter-verifiable."}

    cumulative = Counter({name: 0 for name in (
        "departed", "arrived", "teleport_start", "teleport_end",
        "collision_events", "colliding_vehicle_reports")})
    metadata["cumulative_counts"] = dict(cumulative)
    metadata["diagnostic_horizon"] = diagnostic_horizon
    last_progress_time = None

    def event(lane_id, time, kind, **fields):
        return {"schema": SCHEMA, "run_id": run_id, "lane_id": lane_id,
                "section_id": registry["sections"][lane_id]["section_id"],
                "time": float(time), "event": kind, "kind": kind, **fields}

    try:
        while True:
            remaining = int(connection.simulation.getMinExpectedNumber())
            metadata["remaining"] = remaining
            if remaining == 0:
                break
            if now >= end:
                if diagnostic_horizon:
                    metadata["termination"] = "diagnostic_horizon"
                    break
                metadata["termination"] = "safety_cap_with_remaining"
                raise ObservationHorizonExceeded(now, end, remaining)
            previous_time = now
            connection.simulationStep()
            now = float(connection.simulation.getTime())
            _record_simulation_counters(connection.simulation, cumulative)
            metadata["cumulative_counts"] = dict(cumulative)
            detectors.refresh()
            current, by_lane = {}, {lane: [] for lane in registry["lanes"]}
            # Complete snapshots make an absent lane empty only after every
            # currently running vehicle was read successfully.
            failed_snapshot = False
            for vehicle_id in connection.vehicle.getIDList():
                try:
                    lane_id = connection.vehicle.getLaneID(vehicle_id)
                    route_index = connection.vehicle.getRouteIndex(vehicle_id)
                    old = previous.get(vehicle_id)
                    if old is None or (lane_id != old["lane_id"] and lane_id in registry["sections"]):
                        visit_number[vehicle_id] += 1
                    key = "%s|visit=%d|route=%d" % (vehicle_id, visit_number[vehicle_id], route_index)
                    # Preserve the source visit key while on its internal path.
                    if old and registry["lanes"].get(lane_id, {}).get("internal"):
                        key = old["vehicle_key"]
                    item = {"vehicle_id": vehicle_id, "vehicle_key": key, "lane_id": lane_id,
                            "position": connection.vehicle.getLanePosition(vehicle_id),
                            "speed": connection.vehicle.getSpeed(vehicle_id),
                            "type_id": connection.vehicle.getTypeID(vehicle_id),
                            "speed_factor": connection.vehicle.getSpeedFactor(vehicle_id),
                            "stop_state": connection.vehicle.getStopState(vehicle_id),
                            "length": connection.vehicle.getLength(vehicle_id),
                            "min_gap": connection.vehicle.getMinGap(vehicle_id),
                            "route_index": route_index, "route": tuple(connection.vehicle.getRoute(vehicle_id))}
                    current[vehicle_id] = item
                    if lane_id in by_lane:
                        by_lane[lane_id].append(item)
                    if lane_id in registry["sections"]:
                        latest_lane_key[(vehicle_id, lane_id)] = key
                    if item["type_id"] not in metadata["vehicle_types"]:
                        type_domain = connection.vehicletype
                        metadata["vehicle_types"][item["type_id"]] = {
                            "length": type_domain.getLength(item["type_id"]),
                            "min_gap": type_domain.getMinGap(item["type_id"]),
                            "tau": type_domain.getTau(item["type_id"]),
                            "accel": type_domain.getAccel(item["type_id"]),
                            "decel": type_domain.getDecel(item["type_id"]),
                            "action_step_length": type_domain.getActionStepLength(item["type_id"])}
                except traci.TraCIException:
                    failed_snapshot = True
                    counts["snapshot_errors"] += 1
            teleported = set(connection.simulation.getStartingTeleportIDList())
            previous_by_lane = defaultdict(list)
            lane_changes = defaultdict(list)
            for vehicle_id, old in previous.items():
                previous_by_lane[old["lane_id"]].append(old)
                item = current.get(vehicle_id)
                if not item or item["lane_id"] == old["lane_id"]:
                    continue
                old_edge = registry["lanes"].get(old["lane_id"], {}).get("edge_id")
                new_edge = registry["lanes"].get(item["lane_id"], {}).get("edge_id")
                if old_edge is not None and old_edge == new_edge:
                    lane_changes[old["lane_id"]].append((old, item))
                    lane_changes[item["lane_id"]].append((old, item))
            batch = []
            detector_current = set()
            for lane_id, lane_state in lane_states.items():
                section = registry["sections"][lane_id]
                lane_batch, cut_reasons = [], []
                lane_vehicles = by_lane[lane_id]
                active_ids = {item["vehicle_id"] for item in lane_state.cohort.values()}
                if failed_snapshot:
                    cut_reasons.append("snapshot_unknown")
                if teleported.intersection(active_ids):
                    cut_reasons.append("teleport")
                if lane_state.active:
                    for old, item in lane_changes[lane_id]:
                        if min(abs(section["position"] - item["position"]),
                               abs(section["position"] - old["position"])) <= metadata["observation"]["cohort_distance"]:
                            cut_reasons.append("near_outlet_lane_change")
                passed_this_step = set()
                for datum in detectors.vehicle_data(section["detector_id"]):
                    vehicle_id, length, entry_time, exit_time, type_id = datum
                    identity = (section["detector_id"], vehicle_id, entry_time)
                    detector_current.add(identity)
                    if identity in detector_seen or entry_time < 0:
                        continue
                    detector_seen.add(identity)
                    counts["passages"] += 1
                    source = previous.get(vehicle_id)
                    if not source or source["lane_id"] != lane_id:
                        source = next((item for item in lane_state.cohort.values()
                                       if item["vehicle_id"] == vehicle_id and item["vehicle_key"] == latest_lane_key.get((vehicle_id, lane_id))), None)
                    observed = current.get(vehicle_id)
                    if source is None and observed and observed["lane_id"] == lane_id:
                        source = observed
                    key = latest_lane_key.get((vehicle_id, lane_id), vehicle_id + "|unobserved-source")
                    candidates = connection_candidates(source, registry, lane_id) if source else []
                    selected, resolution = resolve_passage_connection(
                        candidates, {observed["lane_id"]} if observed else set(), registry)
                    if resolution == "ambiguous_connection":
                        cut_reasons.append("ambiguous_connection")
                    passage = event(lane_id, max(float(entry_time), previous_time), "passage",
                                    vehicle_key=key, vehicle_id=vehicle_id, type_id=type_id,
                                    detector_entry_time=float(entry_time), detector_exit_time=float(exit_time),
                                    speed_factor=source.get("speed_factor") if source else None,
                                    timing_quality="e1_front_entry", connection_resolution=resolution,
                                    connection_candidates=[item.key for item in candidates],
                                    speed_limit=registry["lanes"][lane_id]["speed_limit"],
                                    road_lane_count=section["road_lane_count"])
                    if selected:
                        passage.update(selected.record())
                        passage["quality"] = "observed"
                        lane_state.outlet_predecessors[vehicle_id] = {
                            "connection": selected, "source_route_index": source["route_index"]}
                        actual_links = connection.lane.getLinks(lane_id, extended=True)
                        matches = [item for item in actual_links if len(item) >= 8
                                   and item[0] == selected.to_lane and (item[4] or "") == selected.via_lane]
                        passage["permission_class"] = permission_class(selected, matches[0][5]) if len(matches) == 1 else "unknown"
                    else:
                        passage.update({"movement_id": None, "turn": "unknown"})
                        passage["quality"] = "unknown"
                        passage["permission_class"] = "unknown"
                    if entry_time < previous_time - 1e-7:
                        cut_reasons.append("late_detector_event")
                        passage["timing_quality"] = "late_e1_entry_unknown"
                    lane_batch.append(passage)
                    passed_this_step.add(key)
                    if key in lane_state.cohort:
                        lane_state.passed.add(key)
                own_keys = set(lane_state.cohort)
                gate_own_keys = set(own_keys)
                for vehicle_id, predecessor in list(lane_state.outlet_predecessors.items()):
                    predecessor_connection = predecessor["connection"]
                    observed = current.get(vehicle_id)
                    path = internal_path(predecessor_connection, registry) or []
                    if observed and observed["route_index"] in {predecessor["source_route_index"], predecessor["source_route_index"] + 1} and (observed["lane_id"] in path or (
                            observed["lane_id"] == predecessor_connection.to_lane and
                            observed["position"] - observed["length"] <= observed["length"] + observed["min_gap"] + 1)):
                        gate_own_keys.add(observed["vehicle_key"])
                    else:
                        lane_state.outlet_predecessors.pop(vehicle_id)
                for key, original in list(lane_state.cohort.items()) if lane_state.active else []:
                    observed = current.get(original["vehicle_id"])
                    if observed is None:
                        if key not in lane_state.passed:
                            cut_reasons.append("cohort_vehicle_missing")
                        continue
                    influence = observed["lane_id"] == lane_id
                    if key in lane_state.passed:
                        for candidate in connection_candidates(original, registry, lane_id):
                            path = internal_path(candidate, registry) or []
                            if observed["lane_id"] in path:
                                influence = True
                            if observed["lane_id"] == candidate.to_lane and observed["position"] - observed["length"] <= 0:
                                influence = True
                    if not influence:
                        continue
                    motion = event(lane_id, now, "motion", vehicle_key=key,
                                   vehicle_id=original["vehicle_id"], speed=observed["speed"],
                                   position=observed["position"], present=True,
                                   influence="outlet_tail" if key in lane_state.passed else "upstream")
                    lane_batch.append(motion)
                    if not valid_speed(observed["speed"]):
                        cut_reasons.append("invalid_speed")
                    elif observed["speed"] > epsilon:
                        lane_state.started.add(key)
                    elif key in lane_state.started:
                        cut_reasons.append("started_vehicle_stopped")
                # Candidate gate is evaluated for the next unpassed member, or
                # for the closest stopped vehicle when waiting for a new start.
                unpassed = [item for item in lane_vehicles if item["vehicle_key"] not in lane_state.passed]
                head = max(unpassed, key=lambda item: item["position"], default=None)
                gates = []
                candidates = connection_candidates(head, registry) if head else []
                if head and candidates and not failed_snapshot:
                    links = connection.lane.getLinks(lane_id, extended=True)
                    for candidate in candidates:
                        evidence = classify_gate(candidate, links, by_lane, head, registry,
                                                 own_keys=gate_own_keys, zero_epsilon=epsilon)
                        evidence["connection_id"] = candidate.key
                        gates.append(evidence)
                if gates and all(gate["status"] == "clear" for gate in gates):
                    permissions = {item["permission_class"] for item in gates}
                    if len(permissions) == 1:
                        gate = {"status": "clear", "reason": "all_candidate_paths_clear", "paths": gates,
                                "permission_class": next(iter(permissions))}
                    else:
                        gate = {"status": "unknown", "reason": "ambiguous_permission_class", "paths": gates}
                else:
                    gate = next((item for item in gates if item["status"] != "clear"),
                                {"status": "unknown", "reason": "no_resolved_head_path"})
                if lane_state.active and head and gate["status"] != "clear":
                    cut_reasons.append(gate["reason"])
                if lane_state.active and gate["status"] == "clear" and gate["permission_class"] != lane_state.permission_class:
                    cut_reasons.append("permission_class_changed")
                if lane_state.active and not head and len(lane_state.passed.intersection(own_keys)) < len(own_keys):
                    cut_reasons.append("cohort_continuity_unknown")
                if lane_state.active and cut_reasons:
                    cut_time = min(previous_time, lane_state.last_clear_time or previous_time)
                    lane_batch.append(event(lane_id, now, "cut", reason=sorted(set(cut_reasons))[0],
                                            reasons=sorted(set(cut_reasons)), cut_time=cut_time,
                                            cut_time_lo=cut_time, cut_time_hi=now, gate_evidence=gate))
                    lane_state.active = False
                    counts["cuts"] += 1
                elif lane_state.active and own_keys.issubset(lane_state.passed):
                    lane_batch.append(event(lane_id, now, "end", reason="cohort_end"))
                    lane_state.active = False
                if lane_state.active and gate["status"] == "clear":
                    lane_state.last_clear_time = now
                previous_stop_since = dict(lane_state.stop_since)
                present_keys = {item["vehicle_key"] for item in lane_vehicles}
                lane_state.stop_since = {key: value for key, value in lane_state.stop_since.items() if key in present_keys}
                for item in lane_vehicles:
                    key = item["vehicle_key"]
                    if valid_speed(item["speed"]) and item["speed"] <= epsilon:
                        lane_state.stop_since.setdefault(key, now)
                    else:
                        lane_state.stop_since.pop(key, None)
                if not lane_state.active and not cut_reasons and gate["status"] == "clear":
                    old_vehicles = previous_by_lane[lane_id]
                    cohort = stopped_cohort(old_vehicles, section["position"], epsilon,
                                            metadata["observation"]["cohort_distance"],
                                            metadata["observation"]["queue_gap"],
                                            stop_since=previous_stop_since, observed_time=previous_time,
                                            min_stopped_duration=stop_duration)
                    if (cohort and section["position"] - cohort[0]["position"] <= metadata["observation"]["max_start_distance"]
                            and all(item["vehicle_key"] in present_keys for item in cohort)
                            and not passed_this_step.intersection(item["vehicle_key"] for item in cohort)
                            and lane_state.previous_gate and lane_state.previous_gate["status"] == "blocked"):
                        lane_state.active = True
                        lane_state.cohort = {item["vehicle_key"]: dict(item) for item in cohort}
                        lane_state.started.clear()
                        lane_state.passed.clear()
                        lane_state.last_clear_time = now
                        lane_state.sequence_candidate_count += 1
                        lane_state.permission_class = gate["permission_class"]
                        kind = ("stopped_green_start" if lane_state.previous_gate["reason"] == "signal"
                                else "stopped_unblock_start")
                        lane_batch.append(event(lane_id, now, "start", valid_start=True,
                                                anchor_time=now, start_kind=kind, cohort=cohort,
                                                permission_class=gate["permission_class"],
                                                head_distance_to_section=section["position"] - cohort[0]["position"],
                                                gate_evidence=gate, previous_gate=lane_state.previous_gate,
                                                anchor_precision="sampled_permission_time",
                                                anchor_evidence={"stationary_at": previous_time,
                                                                 "permission_clear_at": now,
                                                                 "permission_change_interval": [previous_time, now]}))
                        for item in cohort:
                            observed = current[item["vehicle_id"]]
                            lane_batch.append(event(lane_id, now, "motion", vehicle_key=item["vehicle_key"],
                                                    vehicle_id=item["vehicle_id"], speed=observed["speed"],
                                                    position=observed["position"], present=True))
                            if valid_speed(observed["speed"]) and observed["speed"] > epsilon:
                                lane_state.started.add(item["vehicle_key"])
                        counts["starts"] += 1
                if config.get("_debug_gate_sink"):
                    config["_debug_gate_sink"]({"time": now, "lane_id": lane_id, "gate": gate,
                        "previous_gate": lane_state.previous_gate, "active": lane_state.active,
                        "head": head, "cut_reasons": cut_reasons,
                        "stopped": [(item["vehicle_id"], item["speed"], item["position"],
                                     now - lane_state.stop_since.get(item["vehicle_key"], now)) for item in lane_vehicles]})
                motions = [item for item in lane_batch if item["kind"] == "motion"]
                evidence_event = next((item for item in motions if head and item["vehicle_id"] == head["vehicle_id"]),
                                      motions[0] if motions else None)
                if evidence_event is not None:
                    evidence_event["gate_evidence"] = gate
                    evidence_event["gate_observed_at"] = now
                lane_state.previous_gate = gate
                batch.extend(lane_batch)
            order = {"start": -1, "passage": 0, "motion": 1, "cut": 2, "end": 3}
            for observation in sorted(batch, key=lambda row: (row["time"], row["lane_id"], order[row["kind"]])):
                counts["events"] += 1
                yield observation
            detector_seen = detector_current
            live_ids = set(current)
            recent_visits = {(vehicle_id, item["lane_id"]) for snapshot in (previous, current)
                             for vehicle_id, item in snapshot.items()}
            latest_lane_key = {key: value for key, value in latest_lane_key.items()
                               if key[0] in live_ids and key in recent_visits}
            visit_number = Counter({key: value for key, value in visit_number.items() if key in live_ids})
            previous = current
            if progress_callback is not None and (
                    last_progress_time is None or now - last_progress_time >= progress_interval):
                progress_callback({"simulation_time": now,
                                   "remaining": int(connection.simulation.getMinExpectedNumber()),
                                   "running": len(current),
                                   "observed_passages": counts["passages"],
                                   "cumulative_counts": dict(cumulative)})
                last_progress_time = now
        for lane_id, state in lane_states.items():
            if state.active:
                yield event(lane_id, now, "end", reason="observation_end")
        metadata["completed"] = metadata["remaining"] == 0
        metadata.setdefault("termination", "natural_drain")
    except BaseException as error:
        metadata["completed"] = False
        metadata.setdefault("termination", "adapter_failure")
        metadata["failure"] = "%s: %s" % (type(error).__name__, error)
        raise
    finally:
        metadata["observation_end"] = now
        metadata["cumulative_counts"] = dict(cumulative)
        try:
            metadata["remaining"] = int(connection.simulation.getMinExpectedNumber())
        except BaseException:
            metadata["remaining_at_close_unavailable"] = True
        metadata["counts"] = dict(counts)
        metadata["lane_start_counts"] = {lane_id: state.sequence_candidate_count for lane_id, state in lane_states.items()}
        connection.close()


