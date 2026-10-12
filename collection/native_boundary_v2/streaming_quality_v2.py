"""Active-vehicle boundary labels and fail-closed, per-visit quality admission.

No TraCI calls or behavior setters. The caller supplies one complete native
snapshot and its first-visible feature rows. Outputs are provisional until the
caller has closed and audited the whole run; ``trainable`` is NOT permission to
publish an output from a failed run. Terminal observations wait on disk for the
closed native tripinfo file. This does not extend the validated Euler domain.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field, replace
import json
import math
from pathlib import Path
import sqlite3
import xml.etree.ElementTree as ET

from collection.sumo_native_state import FEATURES
from collection.native_boundary_v2.sumo_native_boundary_v2 import allocate_step, LABELS, THRESHOLDS
from collection.native_boundary_v2.sumo_native_path_v2 import NoInternalGeometry, GeometryUnknown


SCHEMA = "road-time-sumo-native-euler-streaming-quality-v2"
EXTRA_FIELDS = (
    *("v1_" + name for name in LABELS),
    "snapshot_clock_schema", "snapshot_clock_step_seconds", "signal_clock_schema",
    "signal_api_return_time", "signal_snapshot_time", "signal_step_seconds",
    "v2_label_valid", "v2_quality", "v2_partial_seconds", "v2_enter", "v2_exit",
    "feature_snapshot_lag_seconds", "v2_trainable_observed_snapshot",
    "v2_exact_entry_features_known", "quality_reasons", "training_release_scope",
    "native_arrival_untravelled_target_m", "v1_quality",
)
POSITION_EPS_METRES = 0.1


@dataclass
class _Vehicle:
    route: tuple
    metadata: dict
    previous: dict | None = None
    rows: dict = field(default_factory=dict)
    features: dict = field(default_factory=dict)
    budget: Counter = field(default_factory=Counter)
    last_emitted: int = -1
    arrival_time: float | None = None


def _number(value):
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _bin(value, boundaries):
    if not _number(value):
        return "unknown"
    value = float(value)
    low = 0
    for high in boundaries:
        if value < high:
            return f"{low:g}_to_{high:g}"
        low = high
    return f"{low:g}_plus"


class StreamingQualityV2:
    """Stream closed road visits; keep only active visits and native geometry.

    Call ``depart`` before a vehicle's first frame. Call ``step`` AFTER the
    existing NativeStateV2 has produced that frame's closed feature rows. It is
    valid to use :memory: for a small test; production must use a fresh sqlite
    path. ``finalize_tripinfo`` may only be called after native output closure.
    ``finish`` requires every vehicle to have a normal audited terminal record.
    """

    def __init__(self, graph_or_net, step_seconds, pending_db_path, emit):
        if not _number(step_seconds) or float(step_seconds) <= 0:
            raise ValueError("positive finite step required")
        self.graph = (graph_or_net if isinstance(graph_or_net, NoInternalGeometry)
                      else NoInternalGeometry(graph_or_net))
        self.dt, self.emit = float(step_seconds), emit
        db_path = str(pending_db_path)
        if db_path != ":memory:" and Path(db_path).exists():
            raise ValueError("fresh pending database required; no implicit resume")
        self.db = sqlite3.connect(db_path)
        self.db.execute("CREATE TABLE vehicles (id TEXT PRIMARY KEY, status TEXT NOT NULL, payload TEXT)")
        self.db.commit()
        self.active = {}
        self.counts, self.seconds, self.exclusion_reasons = Counter(), Counter(), Counter()
        self._second_sums = {}
        self.length_bins, self.flow_bins = {}, {}
        self.last_time = None
        self.failed = None
        self.closed = False
        self.maximum_vehicle_time_residual = 0.0

    def _fail(self, reason):
        self.failed = str(reason)
        raise ValueError(reason)

    def _ready(self):
        if self.closed or self.failed:
            raise ValueError("quality engine closed or failed: " + str(self.failed))

    def _add_seconds(self, name, value):
        # Neumaier accumulation keeps memory bounded and avoids run-size drift
        # when many independently verified vehicle budgets are combined.
        total, correction = self._second_sums.get(name, (0.0, 0.0))
        updated = total + value
        correction += ((total - updated) + value if abs(total) >= abs(value)
                       else (value - updated) + total)
        self._second_sums[name] = (updated, correction)
        self.seconds[name] = updated + correction

    def depart(self, vehicle_id, route, metadata):
        self._ready()
        if not isinstance(vehicle_id, str) or not vehicle_id or not route:
            self._fail("invalid departure identity/route")
        if not _number(metadata.get("actual_depart")) or float(metadata["actual_depart"]) < 0:
            self._fail("invalid actual departure")
        if any(road not in self.graph.road_lanes for road in route):
            self._fail("departure references unknown road")
        try:
            self.db.execute("INSERT INTO vehicles VALUES (?, 'active', NULL)", (vehicle_id,))
        except sqlite3.IntegrityError:
            self._fail("duplicate departure identity")
        self.active[vehicle_id] = _Vehicle(tuple(route), dict(metadata))
        self.counts["departures"] += 1
        self.counts["expected_occurrences"] += len(route)

    def _row(self, state, index):
        if not 0 <= index < len(state.route) or index <= state.last_emitted:
            self._fail("attempt to change a closed or invalid occurrence")
        if index not in state.rows:
            state.rows[index] = dict(index=index, labels=[0.0] * len(THRESHOLDS),
                                     seconds=0.0, valid=True, touched=False,
                                     reasons=[], enter=None, exit=None, intervals=0,
                                     arrival_shortfall=0.0)
        return state.rows[index]

    def _unknown(self, state, indices, reason, elapsed):
        if not _number(elapsed) or elapsed < 0:
            self._fail("invalid unresolved time budget")
        for index in indices:
            row = self._row(state, index)
            row["valid"] = False
            if reason not in row["reasons"]:
                row["reasons"].append(str(reason))
        state.budget["unresolved_seconds"] += elapsed
        self.counts["unresolved_steps"] += 1

    def _apply(self, state, result, affected):
        if result.status != "RESOLVED":
            self._unknown(state, affected, result.reason.value, result.budget.elapsed or 0.0)
            return False
        state.budget["allocated_seconds"] += result.audit.allocated_elapsed
        for part in result.parts:
            row = self._row(state, part.occurrence)
            row["touched"] = True
            row["seconds"] += part.time
            row["intervals"] += 1
            row["enter"] = part.t_enter if row["enter"] is None else min(row["enter"], part.t_enter)
            row["exit"] = part.t_exit if row["exit"] is None else max(row["exit"], part.t_exit)
            row["labels"] = [a + b for a, b in zip(row["labels"], part.labels)]
        self.counts["resolved_steps"] += 1
        return True

    def _features(self, rows):
        for value in rows:
            vehicle, index = value.get("vehicle_id"), value.get("route_index")
            if vehicle not in self.active:
                self._fail("feature row for untracked vehicle")
            state = self.active[vehicle]
            if (not isinstance(index, int) or isinstance(index, bool)
                    or not 0 <= index < len(state.route)
                    or value.get("road_id") != state.route[index]
                    or index <= state.last_emitted or index in state.features):
                self._fail("feature occurrence identity/duplication mismatch")
            state.features[index] = dict(value)

    @staticmethod
    def _feature_quality(feature, row, dt, is_final):
        reasons = []
        declared = str(feature.get("feature_valid", "0")) == "1"
        if not declared:
            reasons.append("missing_or_unknown_features")
        if declared:
            for name in FEATURES:
                value = feature.get(name)
                if name in {"entry_signal_type", "next_signal_type"}:
                    valid = value in {"R", "G"}
                else:
                    valid = _number(value)
                if not valid:
                    reasons.append("incomplete_required_features")
                    break
            if not reasons:
                road_flow, lane_flow = float(feature["road_flow"]), float(feature["lane_flow"])
                context = float(feature["platoon_context"])
                if (float(feature["road_length"]) <= 0 or float(feature["free_speed_cap"]) <= 0
                        or road_flow < 1 or lane_flow < 1 or lane_flow > road_flow
                        or not road_flow.is_integer() or not lane_flow.is_integer()
                        or not context.is_integer() or not 0 <= context <= 5):
                    reasons.append("invalid_required_feature_domain")
                durations = (float(feature["entry_signal_remaining_seconds"]),
                             float(feature["next_signal_duration_seconds"]))
                if (min(durations) < 0 or 0 in durations and not
                        (is_final and durations == (0.0, 0.0)
                         and feature["entry_signal_type"] == feature["next_signal_type"] == "G")):
                    reasons.append("invalid_signal_feature_duration")
        lag = None
        anchor = feature.get("observed_enter_time")
        if declared:
            if not _number(anchor):
                reasons.append("invalid_feature_snapshot_time")
            elif row["enter"] is not None:
                lag = float(anchor) - row["enter"]
                if lag < -1e-7 or lag > dt + 1e-7:
                    reasons.append("feature_snapshot_lag_outside_step")
        return not reasons, lag, reasons

    def _emit_until(self, vehicle, state, exclusive):
        for index in range(state.last_emitted + 1, exclusive):
            row = self._row(state, index)
            feature = dict(state.features.pop(index, {}))
            valid = row["valid"] and row["touched"]
            feature_ok, lag, feature_reasons = self._feature_quality(
                feature, row, self.dt, index == len(state.route) - 1)
            reasons = list(row["reasons"])
            if not row["touched"] and not reasons:
                reasons.append("unobserved_interval")
            reasons.extend(feature_reasons)
            trainable = valid and feature_ok
            values = dict(vehicle_id=vehicle, route_index=index, road_id=state.route[index],
                          is_first_road=int(index == 0), is_final_road=int(index == len(state.route) - 1),
                          **state.metadata)
            values.update(feature)
            values.update({"v1_" + name: feature.get(name, "") for name in LABELS})
            values.update(zip(LABELS, row["labels"] if valid else [""] * len(LABELS)))
            label_quality = ("estimated_euler_geometry" if valid else
                             ";".join(sorted(row["reasons"])) or "unobserved_interval")
            values.update(schema=SCHEMA, travel_time_label=row["labels"][0] if valid else "",
                          label_valid=int(valid), v2_label_valid=int(valid), feature_valid=int(feature_ok),
                          trainable=int(trainable), v2_trainable_observed_snapshot=int(trainable),
                          v2_exact_entry_features_known=0, v2_quality=label_quality,
                          v2_partial_seconds=row["seconds"], v2_enter=row["enter"], v2_exit=row["exit"],
                          feature_snapshot_lag_seconds=lag, quality_reasons=";".join(sorted(set(reasons))),
                          v1_quality=feature.get("quality", ""),
                          quality="accepted_observed_snapshot" if trainable else "excluded_quality",
                          native_arrival_untravelled_target_m=row["arrival_shortfall"],
                          training_release_scope="provisional_until_run_audit_passes")
            self.emit(values)
            self.counts["road_occurrences"] += 1
            self.counts["trainable"] += int(trainable)
            self.counts["excluded"] += int(not trainable)
            self.counts["valid_v2_labels"] += int(valid)
            self.counts["valid_labels_without_features"] += int(valid and not feature_ok)
            if not trainable:
                self.exclusion_reasons.update(set(reasons))
            length = values.get("road_length")
            if not _number(length):
                length = self.graph.road_lengths.get(state.route[index])
            for mapping, key in ((self.length_bins, _bin(length, (5, 10, 25, 50, 100, 250, 500))),
                                 (self.flow_bins, _bin(values.get("road_flow"), (1, 5, 10, 25, 50, 100)))):
                bucket = mapping.setdefault(key, Counter())
                bucket["total"] += 1
                bucket["trainable"] += int(trainable)
                bucket["excluded"] += int(not trainable)
            del state.rows[index]
            state.last_emitted = index

    def step(self, state_time, observations, arrived=(), feature_rows=(), expected_live_ids=None):
        self._ready()
        try:
            self._step(state_time, observations, arrived, feature_rows, expected_live_ids)
        except Exception as exc:
            self.failed = str(exc)
            raise

    def _step(self, state_time, observations, arrived, feature_rows, expected_live_ids):
        if (not _number(state_time) or state_time < 0
                or self.last_time is not None and state_time <= self.last_time):
            self._fail("invalid/nonmonotone snapshot time")
        if self.last_time is not None and not math.isclose(
                state_time - self.last_time, self.dt, rel_tol=0, abs_tol=1e-8):
            self._fail("incomplete native frame: nonconsecutive global snapshot clock")
        self.last_time = float(state_time)
        observations, arrived = list(observations), list(arrived)
        current = {o["id"]: o for o in observations}
        arrivals = set(arrived)
        if len(current) != len(observations) or len(arrivals) != len(arrived):
            self._fail("duplicate snapshot/arrival identity")
        if not arrivals.issubset(self.active) or arrivals.intersection(current):
            self._fail("unexpected arrival or departed-and-arrived without snapshot")
        expected = set(self.active) - arrivals
        if set(current) != expected or (expected_live_ids is not None and set(expected_live_ids) != expected):
            self._fail("incomplete native frame: subscription/active/physical identity mismatch")
        self._features(feature_rows)
        for vehicle, observation in current.items():
            state = self.active[vehicle]
            if not _number(observation.get("time")) or abs(observation["time"] - state_time) > 1e-8:
                self._fail("snapshot clock mismatch")
            index = observation.get("route_index")
            # Identity/time invalidity is run-wide; nonidentifiable geometry is
            # local and must not erase independent later visits.
            if (not isinstance(index, int) or isinstance(index, bool)
                    or not 0 <= index < len(state.route) or state.route[index] != observation.get("edge")):
                self._fail("snapshot route identity mismatch")
            self.graph._observation(state.route, observation)
            if "distance" not in observation or not _number(observation["distance"]):
                self._fail("missing required native odometer")
            old = state.previous
            if old is None:
                gap = state_time - float(state.metadata["actual_depart"])
                if gap < -1e-7:
                    self._fail("snapshot before actual departure")
                if abs(gap) > 1e-7 or index != 0:
                    self._unknown(state, range(index + 1), "unobserved_departure_interval", max(0.0, gap))
                row = self._row(state, index)
                row["touched"] = True
                if index == 0 and abs(gap) <= 1e-7:
                    row["enter"] = state_time
                state.previous = dict(observation)
                self._emit_until(vehicle, state, index)
                continue
            if index < old["route_index"]:
                self._fail("route occurrence regression")
            affected = range(old["route_index"], index + 1)
            elapsed = state_time - old["time"]
            if not math.isclose(elapsed, self.dt, rel_tol=0, abs_tol=1e-8):
                self._unknown(state, affected, "missing_snapshot_step", elapsed)
            else:
                try:
                    segments = self.graph.step_segments(state.route, old, observation)
                    if not math.isclose(observation["distance"] - old["distance"],
                                        observation["speed"] * elapsed, abs_tol=1e-6, rel_tol=1e-9):
                        raise GeometryUnknown("odometer_not_closed")
                    self._apply(state, allocate_step(t0=old["time"], t1=state_time,
                                speed=observation["speed"], segments=segments), affected)
                except GeometryUnknown as exc:
                    self._unknown(state, affected, str(exc), elapsed)
            state.previous = dict(observation)
            self._emit_until(vehicle, state, index)
        for vehicle in arrived:
            state = self.active[vehicle]
            if state.previous is None:
                self._fail("arrival without an observable departure snapshot")
            state.arrival_time = float(state_time)
            payload = dict(route=state.route, metadata=state.metadata, previous=state.previous,
                           rows=list(state.rows.values()), features=list(state.features.values()),
                           budget=dict(state.budget), last_emitted=state.last_emitted,
                           arrival_time=state.arrival_time)
            self.db.execute("UPDATE vehicles SET status='pending', payload=? WHERE id=?",
                            (json.dumps(payload, allow_nan=False, separators=(",", ":")), vehicle))
            del self.active[vehicle]
            self.counts["arrivals"] += 1
        if arrived:
            self.db.commit()

    def _terminal(self, vehicle, state, trip):
        old, route = state.previous, state.route
        end, speed = float(trip["arrival"]), float(trip["arrivalSpeed"])
        elapsed = end - old["time"]
        affected = range(old["route_index"], len(route))
        if elapsed < -1e-8 or not math.isclose(end, state.arrival_time, rel_tol=0, abs_tol=1e-7):
            self._fail("terminal native/snapshot clock mismatch")
        if float(trip["depart"]) != float(state.metadata["actual_depart"]):
            self._fail("native departure audit mismatch")
        try:
            if not math.isclose(elapsed, self.dt, rel_tol=0, abs_tol=1e-8):
                raise GeometryUnknown("missing_terminal_snapshot_step")
            segments = self.graph.terminal_segments(route, old, trip["arrivalLane"], float(trip["arrivalPos"]))
            distance = math.fsum(part.distance for part in segments)
            if not math.isclose(float(trip["routeLength"]) - old["distance"], distance,
                                abs_tol=1e-6, rel_tol=1e-9):
                raise GeometryUnknown("terminal_odometer_not_closed")
            shortfall = distance - speed * elapsed
            acceptance = None
            if shortfall > 1e-7:
                predicted_front = old["position"] + speed * elapsed
                for index in range(old["route_index"], len(route) - 1):
                    predicted_front -= self.graph.road_length(route[index])
                boundary = min(self.graph.lane_length(trip["arrivalLane"]), float(trip["arrivalPos"]))
                if not predicted_front > boundary - POSITION_EPS_METRES or shortfall > segments[-1].distance:
                    raise GeometryUnknown("terminal_distance_speed_not_closed")
                segments = (*segments[:-1], replace(segments[-1], distance=segments[-1].distance - shortfall))
                distance = math.fsum(part.distance for part in segments)
                acceptance = shortfall
            if speed <= 0 or distance < 0 or distance > speed * elapsed + 1e-7:
                raise GeometryUnknown("terminal_distance_speed_not_closed")
            moving = distance / speed
            if moving > 1e-9:
                resolved = self._apply(state, allocate_step(t0=old["time"], t1=old["time"] + moving,
                                       speed=speed, segments=segments), affected)
            else:
                for index in affected:
                    row = self._row(state, index)
                    row["touched"] = True
                    if row["enter"] is None:
                        row["enter"] = old["time"]
                    row["exit"] = old["time"]
                resolved = True
            # A failed allocation has already charged only the moving interval
            # as unresolved; the remainder still belongs to native rounding.
            state.budget["terminal_rounding_seconds"] += max(0.0, elapsed - moving)
            if resolved and acceptance is not None:
                self._row(state, len(route) - 1)["arrival_shortfall"] = acceptance
                self.counts["native_arrival_acceptance_band"] += 1
        except GeometryUnknown as exc:
            self._unknown(state, affected, str(exc), elapsed)
        residual = float(trip["duration"]) - math.fsum(state.budget.values())
        if abs(residual) > 1e-6:
            self._fail(f"vehicle time budget failed: {vehicle}: {residual}")
        self.maximum_vehicle_time_residual = max(self.maximum_vehicle_time_residual, abs(residual))
        for name, value in state.budget.items():
            self._add_seconds(name, value)
        self._add_seconds("tripinfo_total_duration", float(trip["duration"]))
        self._emit_until(vehicle, state, len(route))

    def finalize_tripinfo(self, tripinfo_path):
        self._ready()
        if self.active:
            self._fail("cannot finalize while vehicles remain active")
        try:
            for event, node in ET.iterparse(tripinfo_path, events=("start", "end")):
                if event == "start":
                    if node.tag == "tripinfos":
                        root = node
                    continue
                if node.tag != "tripinfo":
                    continue
                trip = dict(node.attrib)
                vehicle = trip.get("id")
                record = self.db.execute("SELECT status, payload FROM vehicles WHERE id=?", (vehicle,)).fetchone()
                if not record or record[0] != "pending" or trip.get("vaporized", "") not in {"", "false"}:
                    self._fail("unexpected/duplicate/abnormal native terminal identity")
                for name in ("depart", "arrival", "duration", "arrivalSpeed", "arrivalPos", "routeLength"):
                    if not _number(trip.get(name)) or float(trip[name]) < 0:
                        self._fail("unfinished/nonfinite native terminal data")
                if abs(float(trip["arrival"]) - float(trip["depart"]) - float(trip["duration"])) > 1e-6:
                    self._fail("native tripinfo time identity failed")
                payload = json.loads(record[1])
                state = _Vehicle(tuple(payload["route"]), payload["metadata"], payload["previous"],
                                 {row["index"]: row for row in payload["rows"]},
                                 {row["route_index"]: row for row in payload["features"]},
                                 Counter(payload["budget"]), payload["last_emitted"], payload["arrival_time"])
                self._terminal(vehicle, state, trip)
                self.db.execute("UPDATE vehicles SET status='finalized', payload=NULL WHERE id=?", (vehicle,))
                self.counts["finalized_vehicles"] += 1
                node.clear()
                root.remove(node)
            self.db.commit()
        except Exception as exc:
            self.failed = str(exc)
            raise

    def finish(self):
        self._ready()
        pending = self.db.execute("SELECT COUNT(*) FROM vehicles WHERE status != 'finalized'").fetchone()[0]
        if self.active or pending or self.counts["departures"] != self.counts["finalized_vehicles"]:
            self._fail("incomplete native terminal ledger")
        if self.counts["road_occurrences"] != self.counts["expected_occurrences"]:
            self._fail("road occurrence ledger incomplete")
        accounted = math.fsum(self.seconds[k] for k in
                              ("allocated_seconds", "unresolved_seconds", "terminal_rounding_seconds"))
        residual = self.seconds["tripinfo_total_duration"] - accounted
        if not math.isclose(self.seconds["tripinfo_total_duration"], accounted, abs_tol=1e-5, rel_tol=1e-10):
            self._fail("global time budget failed")
        return dict(schema=SCHEMA, status="QUALITY_LEDGER_COMPLETE_PENDING_RUN_AUDIT", counts=dict(self.counts),
                    seconds=dict(self.seconds), exclusion_reasons=dict(self.exclusion_reasons),
                    coverage_by_length={k: dict(v) for k, v in self.length_bins.items()},
                    coverage_by_flow={k: dict(v) for k, v in self.flow_bins.items()},
                    maximum_vehicle_time_residual=self.maximum_vehicle_time_residual,
                    global_time_residual=residual, pending_terminal_vehicles=pending,
                    training_release_scope="caller_must_pass_complete_run_and_output_closure_audit",
                    feature_anchor="first_real_visible_snapshot; lag in [0, step]; no fabricated features",
                    quality_rule="full visit label and required native features; no duration/magnitude filtering")

    def close(self):
        if not self.closed:
            self.db.close()
            self.closed = True
