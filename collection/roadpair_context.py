"""Read-only SUMO observer using the C++ road-pair/reselection feature rules.

The traffic event stream is SUMO's, not a second C++ simulation. Admission
disagreement fails closed; this module never moves a real vehicle or fabricates
a label. Historical collectors keep their separate sampling contract.
"""
from __future__ import annotations

from collections import defaultdict, deque
import math
import xml.etree.ElementTree as ET

from collection.TraCI_Python_Cap_Context_ABC_RG import ABCRGCapContextObserver
from collection._shared.TraCI_Python_Cap_Context import sumo_type_defaults
from features.cap_context_abc_features import ABCShadowLedger
from features.cap_context_features import ShadowRecord, free_speed_cap, travel_distance


POLICY = "road-pair-reselection-v1"
SAMPLING = {
    "road_flow_lane_flow": "shadow-reservations-including-ego-and-pending-v1",
    "platoon_context": "upstream-stopline-decision-with-fixed-crossing-proxy; first=observed-insertion",
    "same_time_order": "observed disappearances; due departures assigned; existing-ID lexical transitions; new insertions by schedule/XML order",
    "lane_selection": POLICY,
    "labels": "immediate-road-pair-external-road-threshold-sweep-v1",
    "first_lane": "observation-batched planned-departure-queue-count/flow/lane; observed-admission-v1",
    "admission_disagreement": "fail-closed; never force SUMO or over-reserve",
}
AUDIT_FIELDS = (
    "lane_policy", "shadow_source_lane", "shadow_entry_lane",
    "selected_connection_index", "shadow_lane_capacity", "shadow_road_flow",
    "shadow_lane_flow", "actual_road_flow", "actual_lane_flow", "actual_lane_index",
    "physical_label_road_length", "scheduled_departure", "entry_queue_size_at_assignment", "exit_signal_status",
)


def permitted(attributes, vehicle_class):
    allow = attributes.get("allow", "all").split()
    disallow = attributes.get("disallow", "").split()
    return vehicle_class == "ignoring" or (
        ("all" in allow or vehicle_class in allow)
        and "all" not in disallow and vehicle_class not in disallow)


class RoadPairNetwork:
    """Original connection indices, permissions and C++ edge-mean geometry."""

    def __init__(self, net_file, representative):
        root = ET.parse(net_file).getroot()
        self.lengths, self.speed_limits, self.lane_counts = {}, {}, {}
        self.lanes, self.internal = {}, {}
        for edge in root.findall("edge"):
            eid = edge.get("id", "")
            lanes = edge.findall("lane")
            if edge.get("function") == "internal" or eid.startswith(":"):
                for lane in lanes:
                    self.internal[lane.get("id")] = (eid, int(lane.get("index", "0")), dict(lane.attrib))
                continue
            if "function" in edge.attrib:
                continue
            lengths = [float(l.get("length", "0")) for l in lanes]
            speeds = [float(l.get("speed", "0")) for l in lanes]
            if not lanes or any(not math.isfinite(x) or x <= 0 for x in lengths + speeds):
                raise ValueError(f"invalid external geometry: {eid}")
            indices = [int(l.get("index", "0")) for l in lanes]
            if sorted(indices) != list(range(len(lanes))):
                raise ValueError(f"noncontiguous/duplicate lane indices: {eid}")
            self.lengths[eid] = sum(lengths) / len(lanes)
            self.speed_limits[eid] = sum(speeds) / len(lanes)
            self.lane_counts[eid] = len(lanes)
            for lane in lanes:
                self.lanes[eid, int(lane.get("index", "0"))] = dict(lane.attrib)
        self.connections = [dict(c.attrib) for c in root.findall("connection")]
        self.pairs, continuations = defaultdict(list), defaultdict(list)
        for index, c in enumerate(self.connections):
            c["fromLane"], c["toLane"] = int(c.get("fromLane", "0")), int(c.get("toLane", "0"))
            continuations[c["from"], c["fromLane"]].append(index)
            if c["from"] in self.lengths and c["to"] in self.lengths:
                self.pairs[c["from"], c["to"]].append(index)
        self.internal_permissions = {}
        for indices in self.pairs.values():
            for index in indices:
                c = self.connections[index]
                via, seen, permissions = c.get("via", ""), set(), []
                complete = True
                while via:
                    if via in seen or via not in self.internal:
                        complete = False
                        break
                    seen.add(via)
                    edge, lane, attrs = self.internal[via]
                    permissions.append(attrs)
                    next_indices = continuations[edge, lane]
                    if len(next_indices) != 1:
                        complete = False
                        break
                    nxt = self.connections[next_indices[0]]
                    if (nxt["to"], nxt["toLane"]) != (c["to"], c["toLane"]):
                        complete = False
                        break
                    permissions.append(nxt)
                    via = nxt.get("via", "")
                self.internal_permissions[index] = permissions if complete else None
        length, gap = float(representative["length"]), float(representative["minGap"])
        if not math.isfinite(length) or length <= 0 or not math.isfinite(gap) or gap < 0:
            raise ValueError("invalid representative storage dimensions")
        self.capacities = {road: max(1, math.floor((value + gap) / (length + gap)) + 1)
                           for road, value in self.lengths.items()}
        self._source_cache = {}

    def connection_permitted(self, index, vehicle_class):
        c = self.connections[index]
        internal = self.internal_permissions[index]
        return (internal is not None and permitted(c, vehicle_class)
                and (c["from"], c["fromLane"]) in self.lanes
                and (c["to"], c["toLane"]) in self.lanes
                and permitted(self.lanes[c["from"], c["fromLane"]], vehicle_class)
                and permitted(self.lanes[c["to"], c["toLane"]], vehicle_class)
                and all(permitted(p, vehicle_class) for p in internal))

    def source_lanes(self, road, target, vehicle_class):
        key = road, target, vehicle_class
        if key not in self._source_cache:
            if target is None:
                values = [i for i in range(self.lane_counts[road])
                          if permitted(self.lanes[road, i], vehicle_class)]
            else:
                values = [self.connections[i]["fromLane"] for i in self.pairs[road, target]
                          if self.connection_permitted(i, vehicle_class)]
            self._source_cache[key] = tuple(sorted(set(values)))
        return self._source_cache[key]

    def has_capacity(self, road, lane, ledger):
        return ledger.lane_load(road, lane)[0] < self.capacities[road]

    def select(self, route, index, source_lane, vehicle_class, ledger):
        """Pure read-only selection BEFORE releasing the source or reserving ego."""
        road = route[index]
        target = route[index + 1] if index + 1 < len(route) else None
        next_lanes = self.source_lanes(road, target, vehicle_class) if target else ()
        choices = []
        for ci in self.pairs[route[index - 1], road]:
            c = self.connections[ci]
            if c["fromLane"] != source_lane or not self.connection_permitted(ci, vehicle_class):
                continue
            entry = c["toLane"]
            for logical in next_lanes if target else (entry,):
                if not self.has_capacity(road, entry, ledger) or not self.has_capacity(road, logical, ledger):
                    continue
                key = (*ledger.lane_load(road, logical), abs(entry - logical), logical, ci)
                choices.append((key, ci, entry, logical))
        if not choices:
            raise ValueError(f"shadow admission disagreement at {route[index-1]}->{road}, "
                             f"source_lane={source_lane}, vClass={vehicle_class}; no legal e/s capacity")
        _, ci, entry, logical = min(choices)
        return ci, entry, logical


class ReservationLedger(ABCShadowLedger):
    """Incremental occupancy mirrors reserve/release, including pending records."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.road_counts, self.loads = defaultdict(int), {}

    def lane_load(self, road, lane):
        return self.loads.get((road, lane), (0, 0.0))

    def record_for(self, vid):
        key = self._vehicle_keys.get(vid)
        return self._records.get(key)

    def reserve(self, record):
        super().reserve(record)
        key = record.road_id, record.lane_index
        count, length = self.lane_load(*key)
        self.loads[key] = count + 1, length + record.occupied_length
        self.road_counts[record.road_id] += 1

    def release(self, vehicle_id, route_index=None):
        record = self.record_for(vehicle_id)
        result = super().release(vehicle_id, route_index)
        if result:
            key = record.road_id, record.lane_index
            count, length = self.loads[key]
            self.loads[key] = count - 1, max(0.0, length - record.occupied_length)
            self.road_counts[record.road_id] -= 1
        return result


def departure_plan(metadata):
    """Read only each route's first two edges; do not retain full XML trees."""
    named, vehicles, ordinal = {}, [], 0
    for path in metadata.source_paths:
        current, first_edges = None, None
        for event, element in ET.iterparse(path, events=("start", "end")):
            if event == "start" and element.tag == "vehicle":
                current, first_edges = dict(element.attrib), None
            if event != "end":
                continue
            if element.tag == "route" and "edges" in element.attrib:
                edges = tuple(element.get("edges").split()[:2])
                if current is not None:
                    first_edges = edges
                elif element.get("id"):
                    named[element.get("id")] = edges
            if element.tag == "vehicle":
                vehicles.append((ordinal, current, first_edges))
                ordinal += 1
                current, first_edges = None, None
            element.clear()
    result = []
    for ordinal, attrs, edges in vehicles:
        edges = edges or named.get(attrs.get("route"))
        depart = float(attrs.get("depart", "nan"))
        if not edges or not math.isfinite(depart) or depart < 0:
            raise ValueError(f"reselection collector requires explicit route and numeric departure: {attrs.get('id')}")
        result.append((depart, ordinal, attrs["id"], edges))
    return sorted(result)


class RoadPairCapContextObserver(ABCRGCapContextObserver):
    extra_audit_fields = tuple(dict.fromkeys(ABCRGCapContextObserver.extra_audit_fields + AUDIT_FIELDS))

    def __init__(self, *args, **kwargs):
        # The legacy network rejects unequal lane geometry; this version uses
        # the same arithmetic edge means as C++ without changing label sampling.
        super().__init__(*args, **kwargs, network_factory=self._network_factory)
        self.ledger = ReservationLedger(self.context_config, overdue_threshold_seconds=self.overdue_threshold_seconds)
        self.departures = departure_plan(self.metadata)
        self.departure_rank = {vid: (depart, ordinal) for depart, ordinal, vid, _ in self.departures}
        self.departure_cursor = 0
        self.entry_queues = defaultdict(deque)
        self.entry_assignments = {}

    @staticmethod
    def _network_factory(net_file, metadata):
        name = "car" if "car" in metadata.type_options else "DEFAULT_VEHTYPE"
        attrs = metadata.type_options.get(name, {})
        representative = sumo_type_defaults(attrs.get("vClass", "passenger"))
        representative.update({k: attrs[k] for k in ("length", "minGap") if k in attrs})
        return RoadPairNetwork(net_file, representative)

    def collection_sampling(self):
        return dict(SAMPLING)

    def formal_features(self, event, values):
        return {"road_length": values["road_length"], "road_flow": values["shadow_road_flow"],
                "lane_flow": values["shadow_lane_flow"]}

    def assign_due_departures(self, now):
        while self.departure_cursor < len(self.departures) and self.departures[self.departure_cursor][0] <= now:
            depart, _, vid, edges = self.departures[self.departure_cursor]
            attrs = self.metadata.vehicle_options[vid]
            cls = self.metadata.type_options.get(attrs.get("type", "DEFAULT_VEHTYPE"), {}).get("vClass", "passenger")
            road, target = edges[0], edges[1] if len(edges) > 1 else None
            candidates = self.network.source_lanes(road, target, cls)
            if not candidates:
                raise ValueError(f"no legal first-road lane: {vid}/{road}")
            lane = min(candidates, key=lambda i: (len(self.entry_queues[road, i]), self.ledger.lane_load(road, i)[0], i))
            queue = self.entry_queues[road, lane]
            self.entry_assignments[vid] = (road, lane, len(queue), depart)
            queue.append(vid)
            self.departure_cursor += 1

    def reserve(self, vehicle_id, route_index, decision_time):
        source = self._vehicle_data(vehicle_id)
        route = self.routes[vehicle_id]
        road = route[route_index]
        cls = self.metadata.type_options.get(source["vehicle_type"], {}).get("vClass", "passenger")
        source_lane, ci, queue_size = -1, -1, 0
        depart = float(self.metadata.vehicle_options[vehicle_id]["depart"])
        if route_index == 0:
            assigned_road, lane, queue_size, depart = self.entry_assignments[vehicle_id]
            if assigned_road != road or self.entry_queues[road, lane][0] != vehicle_id:
                raise ValueError(f"shadow departure order disagreement: {vehicle_id}")
            if not self.network.has_capacity(road, lane, self.ledger):
                raise ValueError(f"shadow first-road admission disagreement: {vehicle_id}/{road}/{lane}")
            entry = lane
        else:
            previous = self.ledger.record_for(vehicle_id)
            if previous is None or previous.route_index != route_index - 1:
                raise ValueError(f"missing shadow source reservation: {vehicle_id}/{route_index}")
            source_lane = previous.lane_index
            try:
                ci, entry, lane = self.network.select(route, route_index, source_lane, cls, self.ledger)
            except ValueError as exc:
                raise ValueError(f"observation batch vehicle={vehicle_id}, route_index={route_index}, "
                                 f"time={decision_time}: {exc}") from exc
        limit = self.network.speed_limits[road]
        cap = free_speed_cap(source["speed_factor"], source["vehicle_max_speed"], source["desired_max_speed"], limit)
        distance = travel_distance(self.network.lengths[road], route_index=route_index,
                                   vehicle_length=source["vehicle_length"], depart_pos=source["depart_pos"])
        enter = decision_time + self.entry_crossing_seconds(route, route_index)
        record = ShadowRecord(vehicle_id, route_index, road, lane, enter, distance, cap,
                              source["vehicle_length"] + source["vehicle_min_gap"])
        # Compute everything that can fail before mutating the occupancy ledger.
        if record.key in self.frozen:
            raise ValueError(f"duplicate occurrence: {record.key}")
        signal_values = self.signals.features(route, route_index, lane, decision_time, enter)
        if route_index:
            self.ledger.release(vehicle_id, route_index - 1)
        values = self.ledger.query(record, decision_time)
        values.update(source)
        values.update(signal_values)
        values.update(road_id=road, road_length=self.network.lengths[road], speed_limit=limit,
                      free_speed_cap=cap, lane_policy=POLICY, shadow_source_lane=source_lane,
                      shadow_entry_lane=entry, selected_connection_index=ci,
                      shadow_lane_capacity=self.network.capacities[road], scheduled_departure=depart,
                      entry_queue_size_at_assignment=queue_size,
                      incoming_junction_status=self.signals.incoming_junction_status(route, route_index),
                      incoming_crossing_seconds=enter-decision_time)
        if not route_index:
            self.entry_queues[road, lane].popleft()
        self.ledger.reserve(record)
        values.update(shadow_road_flow=self.ledger.road_counts[road],
                      shadow_lane_flow=self.ledger.lane_load(road, lane)[0])
        self.frozen[record.key] = values
        self.reservations += 1

    def on_step(self):
        now = float(self.api.simulation.getTime())
        if now < self.last_time:
            raise ValueError("SUMO observation time moved backwards")
        self.last_time = now
        ids = set(self.api.vehicle.getIDList())
        interrupted = set(self.api.simulation.getStartingTeleportIDList())
        if interrupted:
            raise ValueError(f"reselection collection requires zero teleport; interrupted={sorted(interrupted)[:10]}")
        arrived = set(self.api.simulation.getArrivedIDList())
        for vid in sorted(set(self.previous)-ids):
            if vid not in arrived:
                raise ValueError(f"abnormal vehicle disappearance: {vid}")
            if self.previous[vid][0].startswith(":") or self.previous[vid][1] != len(self.routes[vid])-1:
                raise ValueError(f"unobserved terminal road; cannot fabricate final label: {vid}")
            self.ledger.release(vid)
            self.previous.pop(vid)
        self.assign_due_departures(now)
        existing = sorted(ids & set(self.previous))
        inserted = sorted(ids - set(self.previous), key=lambda v: self.departure_rank[v])
        for vid in existing + inserted:
            edge, index = self.api.vehicle.getRoadID(vid), int(self.api.vehicle.getRouteIndex(vid))
            old = self.previous.get(vid)
            if old is None:
                if edge.startswith(":") or index != 0:
                    raise ValueError(f"unobserved first road; cannot fabricate context/label: {vid}/{index}")
                self.reserve(vid, 0, now)
            elif not old[0].startswith(":") and (edge != old[0] or index != old[1]):
                next_index = old[1]+1
                if not edge.startswith(":") and index != next_index:
                    raise ValueError(f"skipped route occurrence; cannot fabricate context/label: {vid}/{old[1]}->{index}")
                self.reserve(vid, next_index, now)
            elif old[0].startswith(":") and not edge.startswith(":"):
                if (vid, index) not in self.frozen:
                    raise ValueError(f"unobserved internal-to-external transition: {vid}/{index}")
            self.previous[vid] = edge, index

    def attach(self, event):
        super().attach(event)
        event.cap_context_snapshot.update(actual_road_flow=event.road_flow,
                                          actual_lane_flow=None, actual_lane_index=None,
                                          physical_label_road_length=None,
                                          exit_signal_status=getattr(event, "exit_signal_status", "unknown"))

    def attach_observation(self, event, snapshot, network_info):
        # The legacy event.lane_index/flow are a next-movement projection, not
        # necessarily the lane the real SUMO vehicle currently occupies.
        event.cap_context_snapshot.update(
            actual_lane_flow=int(self.api.lane.getLastStepVehicleNumber(snapshot.lane_id)),
            actual_lane_index=int(self.api.vehicle.getLaneIndex(event.vehicle_id)),
            physical_label_road_length=network_info.lane_lengths[snapshot.lane_id])

    def validate_completion(self):
        missing = set(self.metadata.vehicle_options) - set(self.derivation)
        if missing or len(self.ledger) or any(self.entry_queues.values()):
            raise ValueError(f"incomplete observation/entry/storage: unseen={len(missing)}, "
                             f"storage={len(self.ledger)}, external={sum(map(len, self.entry_queues.values()))}")
        expected = sum(len(route) for route in self.routes.values())
        if len(self.frozen) != expected:
            raise ValueError(f"incomplete route occurrences: {len(self.frozen)}/{expected}")

    def validate_training_events(self, events, base):
        """Do not certify shadow completeness while the label writer drops visits."""
        expected = set(self.frozen)
        actual = [(event.vehicle_id, event.route_index) for event in events]
        valid = [(event.vehicle_id, event.route_index) for event in base.iter_valid_training_events(events)]
        if len(actual) != len(set(actual)) or set(actual) != expected:
            raise ValueError(f"incomplete/duplicate label occurrences: events={len(actual)}, expected={len(expected)}, "
                             f"missing={sorted(expected-set(actual))[:10]}")
        if len(valid) != len(actual) or set(valid) != expected:
            raise ValueError(f"legacy label validity would drop observed occurrences: {len(valid)}/{len(actual)}; "
                             "zero/incomplete labels need an explicit training-contract migration, not silent filtering")
