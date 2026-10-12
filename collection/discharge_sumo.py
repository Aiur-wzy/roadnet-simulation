"""Read-only SUMO adapter for physical-lane discharge observations.

This module never changes vehicle, lane-change, teleport or signal settings.
TraCI is imported only when a live run starts.  Gate evidence is deliberately
conservative: ambiguous paths, unobserved space and route/lane discontinuities
are unknown, never silently certified as clear.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass, field
import gzip
import hashlib
import importlib
import math
import os
from pathlib import Path
import shutil
import sys
import uuid
import xml.etree.ElementTree as ET


SCHEMA = "lane-discharge-event-v1"
ADAPTER_VERSION = "sumo-lane-outlet-v1"


def _xml(path):
    opener = gzip.open if str(path).lower().endswith(".gz") else open
    with opener(path, "rb") as stream:
        return ET.parse(stream).getroot()


def _hash(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _paths(value, base):
    if not value:
        return []
    entries = value.split(",") if isinstance(value, str) else value
    return [str((base / str(item).strip()).resolve()) for item in entries]


def resolve_inputs(config):
    """Resolve net/routes/additionals without replacing config input semantics."""
    cwd = Path.cwd()
    cfg = config.get("sumo_config")
    values = {}
    base = cwd
    if cfg:
        cfg = str(Path(cfg).resolve())
        base = Path(cfg).parent
        for element in _xml(cfg).iter():
            if "value" in element.attrib:
                values[element.tag] = element.attrib["value"]
    net = config.get("net") or config.get("net_file")
    if net:
        net = str(Path(net).resolve())
    elif values.get("net-file"):
        net = str((base / values["net-file"]).resolve())
    else:
        raise ValueError("A net file or a SUMO configuration with net-file is required")
    routes = (_paths(config["routes"], cwd) if config.get("routes")
              else _paths(values.get("route-files"), base))
    additionals = _paths(values.get("additional-files"), base)
    additionals.extend(_paths(config.get("additional_files"), cwd))
    if not routes:
        raise ValueError("No route-files were specified")
    for item in [net, *routes, *additionals, *([cfg] if cfg else [])]:
        if not Path(item).is_file():
            raise FileNotFoundError(item)
    return {"net": net, "routes": routes, "additional_files": additionals,
            "sumo_config": cfg, "config_values": values}


@dataclass(frozen=True)
class Connection:
    from_edge: str
    from_lane: str
    via_lane: str
    to_edge: str
    to_lane: str
    tls: str
    link_index: int | None
    turn: str

    @property
    def key(self):
        return "|".join((self.from_edge, self.from_lane, self.via_lane,
                         self.to_edge, self.to_lane, self.tls,
                         str(self.link_index) if self.link_index is not None else ""))

    def record(self):
        return {"movement_id": self.key, "from_edge": self.from_edge,
                "from_lane": self.from_lane, "via_lane": self.via_lane,
                "to_edge": self.to_edge, "to_lane": self.to_lane,
                "tls": self.tls, "link_index": self.link_index, "turn": self.turn}


def build_network_registry(net_file, detector_offset=0.0):
    """Include every external lane with an outgoing connection, including no-TLS.

    Lane length is SUMO's effective lane length; endOffset must not be subtracted
    from it again.  The explicit detector offset is retained in the manifest.
    """
    if not math.isfinite(detector_offset) or detector_offset < 0:
        raise ValueError("detector_offset must be finite and non-negative")
    root = _xml(net_file)
    lanes, edge_lanes = {}, {}
    for edge in root.findall("edge"):
        edge_lanes[edge.attrib["id"]] = {}
        for lane in edge.findall("lane"):
            lane_id = lane.attrib["id"]
            lanes[lane_id] = {"lane_id": lane_id, "edge_id": edge.attrib["id"],
                              "length": float(lane.attrib["length"]),
                              "speed_limit": float(lane.attrib["speed"]),
                              "internal": edge.attrib.get("function") == "internal",
                              "end_offset_xml": lane.attrib.get("endOffset", edge.attrib.get("endOffset")),
                              "stop_offset_xml": [child.attrib for child in lane.findall("stopOffset")]}
            edge_lanes[edge.attrib["id"]][lane.attrib["index"]] = lane_id
    connections = defaultdict(list)
    internal_successors = defaultdict(list)
    for node in root.findall("connection"):
        item = node.attrib
        from_lane = edge_lanes.get(item["from"], {}).get(item["fromLane"])
        to_lane = edge_lanes.get(item["to"], {}).get(item["toLane"])
        if from_lane is None or to_lane is None:
            raise ValueError("Connection references an absent lane: %r" % item)
        if lanes[from_lane]["internal"]:
            internal_successors[from_lane].append(item.get("via") or to_lane)
            continue
        connections[from_lane].append(Connection(
            item["from"], from_lane, item.get("via", ""), item["to"],
            to_lane, item.get("tl", ""),
            int(item["linkIndex"]) if "linkIndex" in item else None,
            item.get("dir", "unknown")))
    sections = {}
    for number, lane_id in enumerate(sorted(connections)):
        lane = lanes[lane_id]
        if lane["length"] <= detector_offset:
            raise ValueError("Detector offset exceeds lane length: " + lane_id)
        sections[lane_id] = {"section_id": "lane_outlet:" + lane_id,
                             "detector_id": "discharge_observer_%08d" % number,
                             "position": lane["length"] - detector_offset,
                             "offset_from_lane_end": detector_offset,
                             "road_lane_count": len(edge_lanes[lane["edge_id"]])}
    return {"lanes": lanes, "connections": dict(connections),
            "internal_successors": dict(internal_successors), "sections": sections}


def internal_path(connection, registry):
    """Return the complete unique via path, or None when it cannot be certified."""
    if not connection.via_lane:
        return []
    path, cursor = [], connection.via_lane
    while cursor != connection.to_lane:
        if cursor in path or cursor not in registry["lanes"]:
            return None
        if not registry["lanes"][cursor]["internal"]:
            return None
        path.append(cursor)
        successors = registry["internal_successors"].get(cursor, [])
        if len(successors) != 1:
            return None
        cursor = successors[0]
    return path


def valid_speed(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and value >= 0


def permission_class(connection, state):
    if not connection.tls:
        return "unsignalized"
    return "protected" if state == "G" else "yield" if state == "g" else "unavailable"


def stopped_cohort(states, section_position, zero_epsilon=1e-6,
                   cohort_distance=100.0, queue_gap=12.0, stop_since=None,
                   observed_time=None, min_stopped_duration=0.0):
    """The maximal contiguous *qualified* stopped prefix nearest the outlet.

    A rear car that only just reached zero cannot veto an already qualified
    head, nor may it be skipped to admit a farther vehicle. Duration is checked
    at the stationary snapshot, before the sampled permission-clear instant.
    """
    vehicles = sorted((item for item in states
                       if 0 <= section_position - item["position"] <= cohort_distance),
                      key=lambda item: item["position"], reverse=True)
    cohort = []
    for item in vehicles:
        if not valid_speed(item.get("speed")) or item["speed"] > zero_epsilon:
            break
        if stop_since is not None:
            since = stop_since.get(item["vehicle_key"])
            if since is None or observed_time is None or observed_time - since < min_stopped_duration - 1e-9:
                break
        if cohort:
            leader = cohort[-1]
            gap = leader["position"] - leader["length"] - item["position"]
            if gap < -1e-3 or gap > queue_gap:
                break
        cohort.append(item)
    return cohort


def connection_candidates(state, registry, source_lane=None):
    lane = source_lane or state["lane_id"]
    route, index = state.get("route", ()), state.get("route_index", -1)
    next_edge = route[index + 1] if 0 <= index < len(route) - 1 else None
    candidates = registry["connections"].get(lane, [])
    return [connection for connection in candidates if connection.to_edge == next_edge]


def resolve_passage_connection(candidates, observed_lanes, registry):
    """Resolve by actual via/to trajectory; an unobserved unique path is labeled."""
    matched = []
    for connection in candidates:
        path = internal_path(connection, registry)
        if connection.to_lane in observed_lanes or (path and set(path).intersection(observed_lanes)):
            matched.append(connection)
    if len(matched) == 1:
        return matched[0], "observed_path"
    if not matched and len(candidates) == 1 and (not observed_lanes or observed_lanes == {candidates[0].from_lane}):
        return candidates[0], "unique_route_connection"
    return None, "ambiguous_connection"


def classify_gate(connection, links, states_by_lane, vehicle, registry,
                  own_keys=frozenset(), zero_epsilon=1e-6):
    """Combine right-of-way with actual receiving-lane and internal observations.

    Moving vehicles of this same cohort inside the junction are ordinary queue
    service predecessors.  They remain motion-tracked; merely occupying internal
    pavement is not itself a downstream block.  Other internal occupants are
    conservatively unknown, even if the TLS is green.
    """
    if vehicle.get("stop_state", 0) & 1:
        return {"status": "blocked", "reason": "planned_stop"}
    matching = [link for link in links
                if len(link) >= 8 and link[0] == connection.to_lane
                and (link[4] or "") == connection.via_lane]
    if len(matching) != 1:
        return {"status": "unknown", "reason": "link_mapping_unknown"}
    link = matching[0]
    state = link[5]
    permission = permission_class(connection, state)
    if state in ("r", "R", "y", "Y", "u"):
        return {"status": "blocked", "reason": "signal", "signal_state": state}
    if not link[2] or link[3]:
        return {"status": "blocked", "reason": "right_of_way", "signal_state": state}
    path = internal_path(connection, registry)
    if path is None:
        return {"status": "unknown", "reason": "internal_path_unknown", "signal_state": state}
    receiver = states_by_lane.get(connection.to_lane)
    if receiver is None:
        return {"status": "unknown", "reason": "receiver_unobserved", "signal_state": state}
    required = vehicle["length"] + vehicle["min_gap"]
    lane_length = registry["lanes"][connection.to_lane]["length"]
    nearest_rear = min((item["position"] - item["length"] for item in receiver),
                       default=lane_length)
    if nearest_rear + 1e-9 < required:
        # A moving predecessor of this very cohort is intrinsic following, not
        # an external storage obstruction. A stopped predecessor still blocks.
        nearest = min(receiver, key=lambda item: item["position"] - item["length"], default=None)
        if not (nearest and nearest["vehicle_key"] in own_keys
                and valid_speed(nearest.get("speed")) and nearest["speed"] > zero_epsilon):
            return {"status": "blocked", "reason": "receiver_space", "signal_state": state,
                    "entry_free_m": nearest_rear, "required_m": required}
        # Follow the physically connected receiving queue. A moving own leader
        # crawling into a closely stopped vehicle is external blocking evidence,
        # even before that own vehicle itself reaches exact zero speed.
        connected_front = nearest
        for leader in sorted(receiver, key=lambda item: item["position"]):
            if leader is nearest:
                continue
            gap = leader["position"] - leader["length"] - connected_front["position"]
            if gap > connected_front["min_gap"] + 1.0:
                break
            if not valid_speed(leader.get("speed")):
                return {"status": "unknown", "reason": "receiver_queue_speed_unknown", "signal_state": state}
            if leader["speed"] <= zero_epsilon:
                return {"status": "blocked", "reason": "receiver_connected_stopped_queue", "signal_state": state}
            connected_front = leader
    for lane_id in path:
        occupants = states_by_lane.get(lane_id)
        if occupants is None:
            return {"status": "unknown", "reason": "internal_unobserved", "signal_state": state}
        for occupant in occupants:
            if occupant["vehicle_key"] not in own_keys:
                return {"status": "unknown", "reason": "unrelated_internal_vehicle", "signal_state": state}
            if not valid_speed(occupant.get("speed")):
                return {"status": "unknown", "reason": "invalid_internal_speed", "signal_state": state}
            if occupant["speed"] <= zero_epsilon:
                return {"status": "blocked", "reason": "internal_stopped", "signal_state": state}
    return {"status": "clear", "reason": "observed_space_and_right_of_way",
            "permission_class": permission,
            "signal_state": state, "entry_free_m": nearest_rear, "required_m": required,
            "internal_path": path}


@dataclass
class LaneState:
    cohort: dict = field(default_factory=dict)
    active: bool = False
    started: set = field(default_factory=set)
    last_clear_time: float | None = None
    stop_since: dict = field(default_factory=dict)
    previous_gate: dict | None = None
    passed: set = field(default_factory=set)
    sequence_candidate_count: int = 0
    permission_class: str | None = None
    outlet_predecessors: dict = field(default_factory=dict)


def _import_traci():
    try:
        return importlib.import_module("traci")
    except ImportError:
        sumo_home = os.environ.get("SUMO_HOME")
        binary = shutil.which("sumo")
        candidates = ([Path(sumo_home) / "tools"] if sumo_home else [])
        if binary:
            candidates.append(Path(binary).resolve().parent.parent / "tools")
        for candidate in candidates:
            if (candidate / "traci").is_dir():
                sys.path.insert(0, str(candidate))
                return importlib.import_module("traci")
        raise RuntimeError("TraCI is unavailable; set SUMO_HOME or install the matching SUMO Python tools")


def _write_detectors(registry, output_dir):
    root = ET.Element("additional")
    for section in registry["sections"].values():
        lane_id = section["section_id"].split(":", 1)[1]
        ET.SubElement(root, "inductionLoop", {
            "id": section["detector_id"], "lane": lane_id,
            "pos": str(section["position"]), "period": "3600",
            "file": str(output_dir / "detector_aggregate.xml")})
    path = output_dir / "discharge_detectors.add.xml"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return str(path)


def _prepare_isolated_inputs(inputs, output_dir):
    """Clone XML configuration and known observer outputs into this run only.

    Unknown external file references are rejected instead of guessing whether
    they are inputs or outputs. The route/net content is never rewritten.
    """
    originals = []
    additional_files = []
    output_tags = {"inductionLoop", "instantInductionLoop", "laneAreaDetector",
                   "entryExitDetector", "edgeData", "laneData", "meanData"}
    for index, source in enumerate(inputs["additional_files"]):
        root = _xml(source)
        for number, element in enumerate(root.iter()):
            for name in ("file", "output"):
                if name not in element.attrib:
                    continue
                if element.tag not in output_tags:
                    raise ValueError("Cannot safely isolate additional XML external file on <%s>: %s" % (element.tag, source))
                element.set(name, str(output_dir / ("additional_%d_%d.xml" % (index, number))))
        clone = output_dir / ("input_additional_%d.xml" % index)
        ET.ElementTree(root).write(clone, encoding="utf-8", xml_declaration=True)
        additional_files.append(str(clone))
        originals.append(source)
    cfg = inputs["sumo_config"]
    if cfg:
        root = _xml(cfg)
        input_group = root.find("input")
        allowed_inputs = {"net-file", "route-files", "additional-files"}
        if input_group is not None:
            for element in input_group:
                if element.tag not in allowed_inputs and element.attrib.get("value"):
                    raise ValueError("Unsupported external SUMO input option for reproducible isolation: " + element.tag)
        for group in root:
            for element in group:
                value = element.attrib.get("value", "")
                tag = element.tag
                if group.tag == "input" or not value:
                    continue
                # Every configured output with a filename is remapped. Boolean
                # switches such as tripinfo-output.write-unfinished stay intact.
                is_file_output = (tag.endswith("-output") or tag in {"log", "message-log", "error-log", "save-state.files"})
                if is_file_output:
                    names = value.split(",") if tag == "save-state.files" else [value]
                    element.set("value", ",".join(str(output_dir / ("sumo_" + tag.replace(".", "_") + "_%d" % n + "_" + Path(name).name))
                                                      for n, name in enumerate(names)))
                elif tag in {"output-prefix", "save-state.prefix"}:
                    element.set("value", str(output_dir / "sumo_state_"))
                elif "file" in tag or tag.endswith("-log"):
                    raise ValueError("Cannot safely resolve configured external file option: " + tag)
        clone = output_dir / "isolated.sumocfg"
        ET.ElementTree(root).write(clone, encoding="utf-8", xml_declaration=True)
        cfg = str(clone)
    return cfg, additional_files


def iter_sumo_events(config):
    """Yield lane-discharge-event-v1 dictionaries from a private SUMO run.

    Required: output_dir, end; net/routes or sumo_config. Optional: run_id,
    sumo_binary, seed, step_length, detector_offset, epsilon_zero,
    stop_duration, cohort_distance, queue_gap, max_start_distance.
    `metadata` may be a caller-owned dict, populated throughout and on close.
    The finite end is mandatory; no pre-existing simulator connection is used.
    """
    end = float(config["end"])
    if not math.isfinite(end) or end <= 0:
        raise ValueError("end must be finite and positive")
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
    isolated_config, isolated_additionals = _prepare_isolated_inputs(inputs, output_dir)
    traci = _import_traci()
    command = [str(config.get("sumo_binary") or shutil.which("sumo") or "sumo")]
    if isolated_config:
        command += ["--configuration-file", isolated_config]
    command += ["--net-file", inputs["net"], "--route-files", ",".join(inputs["routes"]),
                "--additional-files", ",".join([*isolated_additionals, detector_file]),
                "--end", str(end), "--no-step-log", "true"]
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
        "verification_limit": "Unspecified SUMO defaults and per-vehicle overrides retained, not all getter-verifiable."}

    def event(lane_id, time, kind, **fields):
        return {"schema": SCHEMA, "run_id": run_id, "lane_id": lane_id,
                "section_id": registry["sections"][lane_id]["section_id"],
                "time": float(time), "event": kind, "kind": kind, **fields}

    try:
        while now < end and connection.simulation.getMinExpectedNumber() > 0:
            previous_time = now
            connection.simulationStep()
            now = float(connection.simulation.getTime())
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
                for datum in connection.inductionloop.getVehicleData(section["detector_id"]):
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
        for lane_id, state in lane_states.items():
            if state.active:
                yield event(lane_id, now, "end", reason="observation_end")
        metadata["completed"] = True
    finally:
        metadata["observation_end"] = now
        metadata["counts"] = dict(counts)
        metadata["lane_start_counts"] = {lane_id: state.sequence_candidate_count for lane_id, state in lane_states.items()}
        connection.close()
