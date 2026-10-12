"""Conservative E1 observation repair; legacy sequence rules remain unchanged.

An E1 insertion/lateral occupancy is not evidence of a longitudinal passage.
Conflicting timestamps for ONE lane visit invalidate its label and the unsafe
sequence suffix, never join vehicles across the rejected observation. Local
vehicle-record errors are quarantined. Database and run-contract faults remain
fatal: continuing without a working simulator or durable output is not safe.
"""
from __future__ import annotations

import json
import math

from features.discharge_sequences import (
    MemoryStore as LegacyMemoryStore, SQLiteStore as LegacySQLiteStore,
    SequenceExtractor as LegacyExtractor, LaneState, EVENT_SCHEMA, finite_number,
    required_string,
)

OBSERVATION_POLICY = "e1-record-isolation-v2"


class PassageTimeConflict(ValueError):
    def __init__(self, prior, current):
        self.prior, self.current = prior, current
        super().__init__("same lane visit has conflicting E1 passage times")


class PassageIdentityConflict(PassageTimeConflict):
    """A local observation key reused with a different vehicle, not a DB error."""


def audit_value(value):
    """Preserve malformed values as evidence, without emitting invalid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_number": repr(value)}
    if isinstance(value, dict):
        return {str(key): audit_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [audit_value(item) for item in value]
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return {"invalid_value": repr(value)}


def usable_passage(row):
    return (not row.get("passage_quarantined")
            and row.get("quality", "observed") not in {"invalid", "unknown", "ambiguous"})


def validate_detector_record(datum, now):
    if not isinstance(datum, (tuple, list)) or len(datum) != 5:
        raise ValueError("E1 vehicle record must contain five fields")
    vehicle, length, entry, exit_time, type_id = datum
    required_string(vehicle, "E1 vehicle_id")
    required_string(type_id, "E1 type_id")
    if finite_number(length, "E1 length", 0) == 0:
        raise ValueError("E1 length must be positive")
    finite_number(entry, "E1 entry", 0)
    finite_number(exit_time, "E1 exit", -1)
    if entry > now + 1e-7 or (exit_time != -1 and (exit_time < entry - 1e-7 or exit_time > now + 1e-7)):
        raise ValueError("E1 record clock outside observed interval")


def passage_clock(row):
    # A repeated detector occupancy can be delivered in several later steps;
    # the raw E1 entry is its clock, not max(entry, previous_snapshot_time).
    return finite_number(row.get("detector_entry_time", row["time"]),
                         "detector entry time", 0)


class _QuarantineStore:
    def accept_passage(self, passage):
        prior = self.lookup_passage(passage)
        passage_clock(passage)
        if prior is None:
            return super().accept_passage(passage)
        if prior.get("passage_quarantined"):
            return False
        if prior["vehicle_id"] != passage["vehicle_id"]:
            raise PassageIdentityConflict(prior, passage)
        if passage_clock(prior) == passage_clock(passage):
            return False
        invalid = dict(prior, quality="invalid", passage_quarantined=True,
                       quarantine_reason="conflicting_e1_entry_time",
                       quarantine_observed_at=passage["time"],
                       conflicting_entry_times=[passage_clock(prior), passage_clock(passage)])
        self.replace_passage(invalid)
        raise PassageTimeConflict(prior, passage)

    def quarantine_record(self, event, timestamp, ordinal, reason):
        key = event.get("vehicle_key")
        if not isinstance(key, str) or not key.strip():
            return None  # Unknown identity: cut its lane, never invent a vehicle.
        prior = self.lookup_passage(event)
        invalid = audit_value(dict(prior or event))
        invalid.update(schema="lane-discharge-passage-v1", quality="invalid",
                       passage_quarantined=True, quarantine_reason=reason,
                       quarantine_observed_at=timestamp)
        # Keep a typed tombstone in the ordinal audit ledger. These defaults
        # are NOT measured values and never enter valid_passages or labels;
        # the original malformed fields remain in the rejection audit.
        for field in ("vehicle_id", "type_id", "turn", "permission_class"):
            if not isinstance(invalid.get(field), str) or not invalid[field].strip():
                invalid[field] = "unknown"
        if prior:
            self.replace_passage(invalid)
        else:
            invalid.update(time=timestamp, detector_entry_time=timestamp,
                           lane_passage_ordinal=ordinal, audit_clock_placeholder=True)
            # This finite clock is an audit placeholder, NEVER a label.
            super().accept_passage(invalid)
        return prior


class MemoryStore(_QuarantineStore, LegacyMemoryStore):
    def lookup_passage(self, row):
        return self.passages.get((row["lane_id"], row["section_id"], row["vehicle_key"]))

    def replace_passage(self, row):
        self.passages[(row["lane_id"], row["section_id"], row["vehicle_key"])] = dict(row)


class SQLiteStore(_QuarantineStore, LegacySQLiteStore):
    def lookup_passage(self, row):
        value = self.connection.execute(
            "SELECT payload FROM passages WHERE lane=? AND section=? AND vehicle=?",
            (row["lane_id"], row["section_id"], row["vehicle_key"])).fetchone()
        return json.loads(value[0]) if value else None

    def replace_passage(self, row):
        cursor = self.connection.execute(
            "UPDATE passages SET payload=? WHERE lane=? AND section=? AND vehicle=?",
            (json.dumps(row, allow_nan=False), row["lane_id"], row["section_id"], row["vehicle_key"]))
        if cursor.rowcount != 1:
            raise RuntimeError("quarantine target disappeared from passage store")


class SequenceExtractor(LegacyExtractor):
    def __init__(self, run_id, config=None, store=None, raw_sink=None, rejection_sink=None):
        super().__init__(run_id, config=config,
                         store=store if store is not None else MemoryStore(), raw_sink=raw_sink)
        if not isinstance(self.store, _QuarantineStore):
            raise TypeError("E1 repair requires a quarantine-capable passage store")
        self.rejection_sink = rejection_sink

    def _audit_rejection(self, event, reason, prior=None):
        self.counts["rejected_vehicle_observations"] += 1
        if self.rejection_sink is not None:
            self.rejection_sink({"schema": "lane-discharge-rejected-observation-v1",
                "run_id": self.run_id, "reason": reason,
                "observation": audit_value(event), "prior": audit_value(prior)})

    def _reject_record(self, lane_id, section_id, lane, event, reason,
                       prior=None, raw_already_written=False):
        clocks = []
        for value in (event.get("time"), lane.last_time):
            try:
                clocks.append(finite_number(value, "record boundary", 0))
            except ValueError:
                pass
        timestamp = max(clocks or [0.0])
        try:
            boundary = finite_number(event.get("time"), "record time", 0)
        except ValueError:
            boundary = lane.last_time or 0.0
        # A bad E1 datum has no crossing clock, but the adapter still knows
        # the start of the simulation interval in which it was reported.
        if "record_interval_start" in event:
            boundary = min(boundary, finite_number(event["record_interval_start"], "record interval", 0))
        vehicle_key = event.get("vehicle_key")
        if event.get("event") == "motion" and isinstance(vehicle_key, str):
            motion = lane.motions.get(vehicle_key)
            if motion and motion.last_time is not None:
                boundary = min(boundary, motion.last_time)
        if prior:
            boundary = min(boundary, prior["time"], passage_clock(prior))
        self._audit_rejection(event, reason, prior)
        if not raw_already_written and self.raw_sink is not None:
            # Only the audit envelope gets a finite observation clock; no
            # repaired timestamp or fabricated vehicle enters valid output.
            self.raw_sink({"schema": EVENT_SCHEMA, "run_id": self.run_id,
                "lane_id": lane_id, "section_id": section_id, "event": "cut", "kind": "cut",
                "time": timestamp, "cut_time": boundary, "reason": reason,
                "rejected_observation": audit_value(event)})
        if (event.get("event") == "motion" and isinstance(vehicle_key, str)
                and vehicle_key.strip() and vehicle_key not in lane.motions):
            # A corrupt unrelated vehicle must not discard a valid cohort.
            lane.last_time = timestamp
            return
        if event.get("event") == "passage":
            stored = self.store.quarantine_record(event, timestamp, lane.ordinal + 1, reason)
            if stored:
                boundary = min(boundary, stored["time"])
            elif isinstance(event.get("vehicle_key"), str) and event["vehicle_key"].strip():
                lane.ordinal += 1
        self._cut(lane_id, section_id, lane, boundary, timestamp, reason)
        lane.last_time = timestamp

    def feed(self, event):
        # Contract errors are not attributable to a single vehicle. Do not
        # blanket-catch exceptions from the legacy engine, sinks or SQLite.
        if (self.finished or not isinstance(event, dict) or event.get("schema") != EVENT_SCHEMA
                or event.get("run_id") != self.run_id or event.get("event") not in {"motion", "passage"}):
            return super().feed(event)
        lane_id = required_string(event.get("lane_id"), "lane_id")
        section_id = required_string(event.get("section_id"), "section_id")
        lane = self.lanes.setdefault((lane_id, section_id), LaneState())
        if event["event"] == "passage" and isinstance(event.get("vehicle_key"), str):
            prior = self.store.lookup_passage(event)
            if prior and prior.get("passage_quarantined"):
                self._audit_rejection(event, "already_quarantined_visit", prior)
                self.counts["duplicate_passage_observations"] += 1
                return
        try:
            timestamp = finite_number(event.get("time"), "time", 0)
            if lane.last_time is not None and timestamp < lane.last_time - self.config.time_epsilon:
                raise ValueError("late vehicle observation")
            required_string(event.get("vehicle_key"), "vehicle_key")
            if event.get("record_error"):
                raise ValueError(str(event["record_error"]))
            if event["event"] == "passage":
                required_string(event.get("quality", "observed"), "quality")
                required_string(event.get("permission_class", "unspecified"), "permission_class")
                required_string(event.get("vehicle_id"), "vehicle_id")
                required_string(event.get("type_id", "unknown"), "type_id")
                required_string(event.get("turn", "unknown"), "turn")
                passage_clock(event)
                if "detector_entry_time" in event and event["detector_entry_time"] > timestamp:
                    raise ValueError("passage clock later than observation")
                for name in ("speed_factor", "speed_limit"):
                    if event.get(name) is not None:
                        finite_number(event[name], name, 0)
            else:
                if event.get("present", True) is True or event.get("speed") is not None:
                    finite_number(event.get("speed"), "motion speed", 0)
                if event.get("position") is not None:
                    finite_number(event["position"], "motion position", 0)
                if event.get("cut_time") is not None and finite_number(event["cut_time"], "motion cut", 0) > timestamp:
                    raise ValueError("motion cut boundary cannot be in the future")
        except ValueError as error:
            self._reject_record(lane_id, section_id, lane, event, "invalid_vehicle_record: " + str(error))
            return
        if event.get("quality") in {"unknown", "ambiguous", "invalid"}:
            self._audit_rejection(event, event.get("exclusion_reason", "unknown_passage"))
        super().feed(event)

    def _passage(self, lane_id, section_id, lane, event, timestamp):
        try:
            super()._passage(lane_id, section_id, lane, event, timestamp)
        except PassageTimeConflict as conflict:
            # Includes already-closed provisional segments: a late update must
            # revoke an earlier label, not only stop the currently active one.
            reason = ("conflicting_vehicle_identity" if isinstance(conflict, PassageIdentityConflict)
                      else "conflicting_e1_entry_time")
            self.counts["quarantined_passage_identity_conflicts" if isinstance(conflict, PassageIdentityConflict)
                        else "quarantined_passage_time_conflicts"] += 1
            self._reject_record(lane_id, section_id, lane, event, reason,
                                prior=conflict.prior, raw_already_written=True)

    def summary(self):
        result = super().summary()
        result["observation_policy"] = OBSERVATION_POLICY
        return result


class E1VisitTracker:
    """Keep one identity while a vehicle's body occupies an E1 detector.

    The timestamp is NOT the visit ID. A new qualified upstream lane visit can
    start a new episode (including a repeated route edge). State is retained
    only while present in consecutive detector replies, not for the whole run.
    All changed clocks still reach the quarantine store for an explicit audit.
    """
    def __init__(self):
        self.active = {}
        self.seen = set()

    def observe(self, detector_id, vehicle_id, entry, key, upstream_key, quality):
        finite_number(entry, "E1 entry", 0)
        identity = (detector_id, vehicle_id)
        prior = self.active.get(identity)
        self.seen.add(identity)
        new_visit = (prior is not None and upstream_key is not None
                     and upstream_key != prior["key"] and entry > prior["entry"])
        if prior is None or new_visit:
            prior = {"key": key, "entry": entry, "quality": quality}
            self.active[identity] = prior
        return prior["key"], (prior["quality"] if entry == prior["entry"]
                              else "e1_entry_time_conflict")

    def end_step(self):
        self.active = {key: value for key, value in self.active.items() if key in self.seen}
        self.seen.clear()


def crossing_quality(previous, observed, lane_id, section_position, vehicle_key):
    """Require an actual sampled position upstream; never invent a crossing.

    First-step insertion already on the detector, lateral entry and an entirely
    skipped short source road lack that evidence. They remain in raw audit but
    cannot become startup/headway labels. This does not move the detector.
    """
    if previous and previous["lane_id"] == lane_id and previous["vehicle_key"] == vehicle_key:
        position = finite_number(previous["position"], "previous position", 0)
        if position < section_position - 1e-7:
            return "e1_front_entry", vehicle_key
        return "e1_boundary_without_upstream_sample", None
    if previous is None and observed and observed["lane_id"] == lane_id and observed["route_index"] == 0:
        return "e1_initial_entry_without_upstream_sample", None
    return "e1_unobserved_longitudinal_entry", None
