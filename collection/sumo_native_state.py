"""SUMO observations only: no projected lane, reservation or capacity gate.

One complete physical snapshot is supplied before any entry feature is frozen.
Labels integrate right-end speeds only between consecutive observations of the
same external road occurrence. Unseen boundary intervals are not invented.
"""
from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass, field
import math

from features.cap_context_features import classify_context

SCHEMA = "road-time-sumo-native-abc-rg-v1"
THRESHOLDS = (0.1, 0.5, 1.0, 1.5, 2.0, 3.0)
LABELS = tuple("travel_time_label_gt_" + format(x, "g").replace(".", "p") for x in THRESHOLDS)
FEATURES = ("road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context",
            "entry_signal_type", "entry_signal_remaining_seconds", "next_signal_type",
            "next_signal_duration_seconds")


@dataclass(frozen=True)
class Observation:
    vehicle_id: str
    road_id: str
    route_index: int
    lane_id: str
    position: float
    speed: float
    cap: float
    length: float
    limit: float


@dataclass
class Visit:
    observation: Observation
    enter: float
    previous_time: float
    free_finish: float
    features: dict = field(default_factory=dict)
    labels: list = field(default_factory=lambda: [0.0] * len(THRESHOLDS))
    sampled_seconds: float = 0.0
    intervals: int = 0
    lane_changes: int = 0
    last_lane: str = ""


def context(ego: Visit, ahead, now):
    """Retain ABC arithmetic, not the old shadow-membership semantics."""
    horizon = ego.free_finish - ego.enter
    if horizon <= 1e-9:
        raise ValueError("nonpositive observed remaining free horizon")
    deficits = [max(0.0, x.free_finish - ego.free_finish) for x in ahead]
    overdue = [max(0.0, now - x.free_finish) for x in ahead]
    result = classify_context(max(deficits, default=0.0), math.fsum(deficits), horizon)
    result["legacy_platoon_context"] = result["platoon_context"]
    subclass = "not_context0"
    if result["platoon_context"] == 0:
        if not ahead:
            result["platoon_context"], subclass = 0, "A"
        elif max(overdue, default=0.0) <= 1e-9:
            result["platoon_context"], subclass = 4, "B"
        else:
            result["platoon_context"], subclass = 5, "C"
    result.update(context0_subclass=subclass, candidate_count=len(ahead),
                  max_overdue_seconds=max(overdue, default=0.0),
                  sum_overdue_seconds=math.fsum(overdue),
                  overdue_count=sum(x > 1e-9 for x in overdue))
    return result


class NativeState:
    def __init__(self, routes, vehicle_metadata, signals, emit):
        self.routes = routes
        self.metadata = vehicle_metadata
        self.signals = signals
        self.emit = emit
        self.active = {}
        self.last_closed = defaultdict(lambda: -1)
        self.last_time = None
        self.counts = Counter()

    def _base(self, vehicle, index):
        route = self.routes[vehicle]
        return dict(schema=SCHEMA, vehicle_id=vehicle, route_index=index, road_id=route[index],
                    is_first_road=int(index == 0), is_final_road=int(index == len(route) - 1),
                    **self.metadata[vehicle])

    def _missing(self, vehicle, index, now, reason):
        row = self._base(vehicle, index)
        row.update(quality=reason, feature_valid=0, label_valid=0, trainable=0,
                   observed_exit_time=now, sampled_seconds=0, sample_intervals=0)
        self.emit(row)
        self.counts[reason] += 1
        self.counts["road_occurrences"] += 1
        self.last_closed[vehicle] = index

    def _close(self, vehicle, now, reason="normal_transition"):
        visit = self.active.pop(vehicle)
        index = visit.observation.route_index
        row = self._base(vehicle, index)
        valid = visit.intervals > 0 and reason in {"normal_transition", "normal_arrival"}
        row.update(visit.features)
        row.update(observed_enter_time=visit.enter, observed_last_time=visit.previous_time,
                   observed_exit_time=now, sampled_seconds=visit.sampled_seconds,
                   sample_intervals=visit.intervals, lane_changes_within_road=visit.lane_changes,
                   exit_boundary_unallocated_seconds=max(0, now - visit.previous_time),
                   label_valid=int(valid), trainable=int(valid and row.get("feature_valid", 0)),
                   quality=reason if valid else ("no_observed_interval" if reason.startswith("normal_") else reason))
        # An observed zero is valid; an unobserved interval is blank, never zero.
        if valid:
            row.update(zip(LABELS, visit.labels))
            row["travel_time_label"] = visit.labels[0]
        self.emit(row)
        self.last_closed[vehicle] = index
        self.counts["road_occurrences"] += 1
        self.counts[row["quality"]] += 1
        self.counts["trainable"] += row["trainable"]
        self.counts["valid_label"] += row["label_valid"]
        self.counts["unusable_feature_rows"] += int(valid and not row.get("feature_valid", 0))

    def step(self, now, observations, arrivals=(), abnormal=(), internal=()):
        if not math.isfinite(now) or now < 0 or (self.last_time is not None and now <= self.last_time):
            raise ValueError("nonmonotone observation clock")
        for o in observations:
            route = self.routes[o.vehicle_id]
            if not 0 <= o.route_index < len(route) or route[o.route_index] != o.road_id:
                raise ValueError("observed edge disagrees with actual route")
            if (not all(math.isfinite(x) for x in (o.position, o.speed, o.cap, o.length, o.limit))
                    or min(o.cap, o.length, o.limit) <= 0 or min(o.position, o.speed) < 0):
                raise ValueError("invalid lane/cap observation")
        self.last_time = now
        arrived, abnormal, internal = set(arrivals), set(abnormal), set(internal)
        physical = {o.vehicle_id: o for o in observations}
        if len(physical) != len(observations):
            raise ValueError("duplicate vehicle observation")
        for vehicle, visit in list(self.active.items()):
            new = physical.get(vehicle)
            if vehicle in abnormal:
                self._close(vehicle, now, "abnormal_exit")
            elif new is None:
                normal_internal = vehicle in internal and visit.observation.route_index < len(self.routes[vehicle]) - 1
                reason = "normal_arrival" if vehicle in arrived else "normal_transition" if normal_internal else "unexplained_disappearance"
                self._close(vehicle, now, reason)
                self.counts["unexplained_disappearances"] += int(reason == "unexplained_disappearance")
            elif new.route_index != visit.observation.route_index:
                self._close(vehicle, now, "normal_transition")
            elif new.road_id != visit.observation.road_id:
                raise ValueError("edge changed without route occurrence index changing")
            else:
                dt = now - visit.previous_time
                if not math.isfinite(new.speed) or new.speed < 0:
                    raise ValueError("invalid physical speed")
                for i, threshold in enumerate(THRESHOLDS):
                    if new.speed > threshold:
                        visit.labels[i] += dt
                visit.intervals += 1
                visit.sampled_seconds += dt
                visit.previous_time = now
                visit.lane_changes += int(visit.last_lane != new.lane_id)
                visit.last_lane = new.lane_id
        for vehicle in arrived:
            if vehicle not in self.routes:
                raise ValueError("arrival without tracked departure")
            for index in range(self.last_closed[vehicle] + 1, len(self.routes[vehicle])):
                self._missing(vehicle, index, now, "unobserved_road_occurrence")
        road_counts = Counter(o.road_id for o in observations)
        lane_members = defaultdict(list)
        newly_entered = []
        for o in observations:
            route = self.routes[o.vehicle_id]
            if o.vehicle_id not in self.active:
                if o.route_index <= self.last_closed[o.vehicle_id]:
                    raise ValueError("nonmonotone route occurrence (teleport or reroute)")
                for index in range(self.last_closed[o.vehicle_id] + 1, o.route_index):
                    self._missing(o.vehicle_id, index, now, "unobserved_road_occurrence")
                distance = max(0.0, o.length - o.position)
                self.active[o.vehicle_id] = Visit(o, now, now, now + distance / o.cap, last_lane=o.lane_id)
                newly_entered.append(o.vehicle_id)
            lane_members[o.lane_id].append(o)
        # All memberships are real simultaneous snapshots, including ego.
        for vehicle in newly_entered:
            visit = self.active[vehicle]
            o = visit.observation
            members = lane_members[o.lane_id]
            ahead = [self.active[j.vehicle_id] for j in members if j.position > o.position and j.vehicle_id != vehicle]
            values = dict(road_length=o.length, road_flow=road_counts[o.road_id], lane_flow=len(members),
                          free_speed_cap=o.cap, speed_limit=o.limit, entry_lane_id=o.lane_id,
                          entry_position=o.position, entry_speed=o.speed,
                          observed_remaining_distance=max(0.0, o.length - o.position),
                          nominal_free_finish_time=visit.free_finish, feature_valid=1, feature_error="")
            try:
                values.update(context(visit, ahead, now))
                values.update(self.signals.features(vehicle, self.routes[vehicle], o.route_index, o.lane_id, now))
            except ValueError as exc:
                values.update(feature_valid=0, feature_error=str(exc))
                self.counts["unknown_features"] += 1
            visit.features = values

    def discard_arrived(self, vehicles):
        for vehicle in vehicles:
            self.routes.pop(vehicle, None)
            self.metadata.pop(vehicle, None)
            self.last_closed.pop(vehicle, None)

    def censor(self, now):
        for vehicle in list(self.active):
            self._close(vehicle, now, "run_censored")


def sampling_contract():
    return {
        "schema": SCHEMA, "formal_features": list(FEATURES), "label_columns": list(LABELS),
        "flow": "physical external edge/lane snapshot at first observation of occurrence; includes ego",
        "lane": "SUMO actual lane, no projected selection, no capacity or pending records",
        "cap": "SUMO getAllowedSpeed at entry; runtime speedFactor and lane limit are audit columns",
        "context": "same actual lane, strictly ahead by front position; entry-frozen free clocks; no reset on lane change",
        "clock": "first observed external occurrence time + (lane length - observed position)/entry cap",
        "signal": "immediate outgoing road pair, live selected link or full-calendar-equivalent fallback, observed entry time",
        "labels": "sum(dt * (right endpoint speed > threshold)) only on consecutive same external occurrence observations",
        "boundary": "first/last sub-step and skipped roads are not reconstructed; coverage and unallocated exit interval explicit",
        "zero": "observed intervals all below a threshold yield valid 0; absent intervals yield missing, not 0",
        "legacy_compatibility": "new sampling contract; must not be mixed silently with old shadow-trained models",
    }
