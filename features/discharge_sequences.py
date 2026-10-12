"""Lane-exit discharge segmentation, independent of SUMO and road predictors.

Input is an append-only observation event stream. A SQLite store keeps provisional
segments revisable when a later observation identifies an earlier interference
boundary. Only the final JSONL export is a training dataset.
"""

from __future__ import annotations

import hashlib
import json
import math
import sqlite3
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Optional


EVENT_SCHEMA = "lane-discharge-event-v1"
SEQUENCE_SCHEMA = "lane-discharge-sequence-v1"
DATASET_SCHEMA = "lane-discharge-dataset-v1"
RULE_VERSION = "lane-exit-hard-cut-v2.1"
START_KINDS = {"stopped_green_start", "stopped_unblock_start"}
PERMISSION_CLASSES = {"protected", "yield", "unsignalized", "unspecified"}


def finite_number(value: Any, name: str, minimum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        raise ValueError(f"invalid {name}: {value!r}")
    return number


def required_string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


@dataclass(frozen=True)
class SegmentationConfig:
    epsilon_zero: float = 1e-7
    progress_epsilon: float = 1e-4
    time_epsilon: float = 1e-9

    def __post_init__(self) -> None:
        finite_number(self.epsilon_zero, "epsilon_zero", 0)
        finite_number(self.progress_epsilon, "progress_epsilon", 0)
        finite_number(self.time_epsilon, "time_epsilon", 0)


class MemoryStore:
    """Small deterministic fixtures; the CLI uses SQLiteStore instead."""

    def __init__(self) -> None:
        self.sequences: dict[str, dict] = {}
        self.passages: dict[tuple, dict] = {}

    def save(self, sequence: dict) -> None:
        self.sequences[sequence["sequence_id"]] = json.loads(json.dumps(sequence))

    def iter_sequences(self) -> Iterable[dict]:
        yield from sorted(self.sequences.values(), key=lambda row: (
            row["lane_id"], row["section_id"], row["anchor_time"], row["sequence_id"]))

    def affected(self, lane_id: str, section_id: str, cut_time: float) -> Iterable[dict]:
        for row in list(self.sequences.values()):
            latest = row["passages"][-1]["time"] if row["passages"] else row["anchor_time"]
            if row["lane_id"] == lane_id and row["section_id"] == section_id and latest >= cut_time:
                yield json.loads(json.dumps(row))

    def accept_passage(self, passage: dict) -> bool:
        key = (passage["lane_id"], passage["section_id"], passage["vehicle_key"])
        prior = self.passages.get(key)
        if prior is not None:
            if prior["time"] != passage["time"]:
                raise ValueError("same lane visit has conflicting passage times")
            return False
        self.passages[key] = dict(passage)
        return True

    def iter_passages(self) -> Iterable[dict]:
        yield from sorted(self.passages.values(), key=lambda row: (
            row["lane_id"], row["section_id"], row["lane_passage_ordinal"]))

    def close(self) -> None:
        pass


class SQLiteStore:
    """Disk-backed provisional output, bounded by active cohorts in RAM."""

    def __init__(self, path: Path) -> None:
        if path.exists():
            raise FileExistsError(f"refusing existing spool: {path}")
        self.connection = sqlite3.connect(str(path))
        self.connection.executescript("""
            CREATE TABLE sequences (
                id TEXT PRIMARY KEY, lane TEXT NOT NULL, section TEXT NOT NULL,
                anchor REAL NOT NULL, latest REAL NOT NULL, payload TEXT NOT NULL);
            CREATE INDEX sequence_lane_time ON sequences(lane, section, latest);
            CREATE TABLE passages (
                lane TEXT NOT NULL, section TEXT NOT NULL, vehicle TEXT NOT NULL,
                time REAL NOT NULL, ordinal INTEGER NOT NULL, payload TEXT NOT NULL,
                PRIMARY KEY(lane, section, vehicle));
        """)

    def save(self, sequence: dict) -> None:
        latest = sequence["passages"][-1]["time"] if sequence["passages"] else sequence["anchor_time"]
        self.connection.execute(
            "INSERT OR REPLACE INTO sequences VALUES (?, ?, ?, ?, ?, ?)",
            (sequence["sequence_id"], sequence["lane_id"], sequence["section_id"],
             sequence["anchor_time"], latest, json.dumps(sequence, allow_nan=False)))

    def iter_sequences(self) -> Iterable[dict]:
        cursor = self.connection.execute(
            "SELECT payload FROM sequences ORDER BY lane, section, anchor, id")
        for (payload,) in cursor:
            yield json.loads(payload)

    def affected(self, lane_id: str, section_id: str, cut_time: float) -> Iterable[dict]:
        # Materialize identifiers before replacing indexed rows in the same DB.
        ids = [row[0] for row in self.connection.execute(
            "SELECT id FROM sequences WHERE lane=? AND section=? AND latest>=?",
            (lane_id, section_id, cut_time))]
        for sequence_id in ids:
            row = self.connection.execute("SELECT payload FROM sequences WHERE id=?", (sequence_id,)).fetchone()
            yield json.loads(row[0])

    def accept_passage(self, passage: dict) -> bool:
        key = (passage["lane_id"], passage["section_id"], passage["vehicle_key"])
        prior = self.connection.execute(
            "SELECT time FROM passages WHERE lane=? AND section=? AND vehicle=?", key).fetchone()
        if prior is not None:
            if prior[0] != passage["time"]:
                raise ValueError("same lane visit has conflicting passage times")
            return False
        self.connection.execute("INSERT INTO passages VALUES (?, ?, ?, ?, ?, ?)",
                                (*key, passage["time"], passage["lane_passage_ordinal"],
                                 json.dumps(passage, allow_nan=False)))
        return True

    def iter_passages(self) -> Iterable[dict]:
        for (payload,) in self.connection.execute(
                "SELECT payload FROM passages ORDER BY lane, section, ordinal"):
            yield json.loads(payload)

    def close(self) -> None:
        self.connection.commit()
        self.connection.close()


@dataclass
class VehicleMotion:
    started: bool = False
    last_time: Optional[float] = None
    position: Optional[float] = None


@dataclass
class LaneState:
    last_time: Optional[float] = None
    ordinal: int = 0
    serial: int = 0
    active: Optional[dict] = None
    motions: dict[str, VehicleMotion] = field(default_factory=dict)
    cohort: list[str] = field(default_factory=list)


def recompute_labels(sequence: dict) -> None:
    passages = sequence["passages"]
    sequence["startup_delay"] = (
        passages[0]["time"] - sequence["anchor_time"]
        if passages and sequence["valid_start"] else None)
    sequence["headways"] = []
    for index, passage in enumerate(passages, 1):
        passage["local_j"] = index
        if index == 1:
            continue
        previous = passages[index - 2]
        sequence["headways"].append({
            "local_j": index, "headway": passage["time"] - previous["time"],
            "prev_vehicle_key": previous["vehicle_key"], "vehicle_key": passage["vehicle_key"],
            "prev_time": previous["time"], "time": passage["time"],
            "prev_ordinal": previous["lane_passage_ordinal"], "ordinal": passage["lane_passage_ordinal"],
            "prev_turn": previous.get("turn", "unknown"), "turn": passage.get("turn", "unknown"),
            "prev_type_id": previous.get("type_id", "unknown"), "type_id": passage.get("type_id", "unknown"),
        })


class SequenceExtractor:
    """Validate observations and extract immutable-after-export lane prefixes."""

    def __init__(self, run_id: str, config: Optional[SegmentationConfig] = None,
                 store: Any = None, raw_sink: Optional[Callable[[dict], None]] = None) -> None:
        self.run_id = required_string(run_id, "run_id")
        self.config = config or SegmentationConfig()
        self.store = store if store is not None else MemoryStore()
        self.raw_sink = raw_sink
        self.lanes: dict[tuple[str, str], LaneState] = {}
        self.counts: Counter = Counter()
        self.cut_reasons: Counter = Counter()
        self.finished = False

    def feed(self, event: dict) -> None:
        if self.finished:
            raise ValueError("extractor already finalized")
        if not isinstance(event, dict) or event.get("schema") != EVENT_SCHEMA:
            raise ValueError(f"expected event schema {EVENT_SCHEMA}")
        if event.get("run_id") != self.run_id:
            raise ValueError("run_id mismatch")
        lane_id = required_string(event.get("lane_id"), "lane_id")
        section_id = required_string(event.get("section_id"), "section_id")
        timestamp = finite_number(event.get("time"), "time", 0)
        kind = event.get("event")
        if kind not in {"start", "motion", "passage", "cut", "end"}:
            raise ValueError(f"unsupported event kind: {kind!r}")
        lane = self.lanes.setdefault((lane_id, section_id), LaneState())
        if lane.last_time is not None and timestamp < lane.last_time - self.config.time_epsilon:
            raise ValueError(f"out-of-order observations on {lane_id}; sort within each simulation step")
        lane.last_time = timestamp
        self.counts[f"events_{kind}"] += 1
        if self.raw_sink is not None:
            self.raw_sink(event)
        if kind == "start":
            self._start(lane_id, section_id, lane, event, timestamp)
        elif kind == "motion":
            self._motion(lane_id, section_id, lane, event, timestamp)
        elif kind == "passage":
            self._passage(lane_id, section_id, lane, event, timestamp)
        elif kind == "cut":
            boundary = finite_number(event.get("cut_time", timestamp), "cut_time", 0)
            if boundary > timestamp + self.config.time_epsilon:
                raise ValueError("cut boundary cannot be in the future")
            self._cut(lane_id, section_id, lane, boundary, timestamp,
                      required_string(event.get("reason"), "reason"))
        else:
            self._close(lane, timestamp, event.get("reason", "observation_end"))

    def _start(self, lane_id: str, section_id: str, lane: LaneState, event: dict, timestamp: float) -> None:
        if lane.active is not None:
            raise ValueError("new start while a sequence is active; cut or finish the old segment first")
        if event.get("valid_start") is not True or event.get("start_kind") not in START_KINDS:
            raise ValueError("start requires an observed stopped-start qualification")
        permission_class = event.get("permission_class", "unspecified")
        if permission_class not in PERMISSION_CLASSES:
            raise ValueError("start requires a recognized permission_class")
        anchor = finite_number(event.get("anchor_time", timestamp), "anchor_time", 0)
        if abs(anchor - timestamp) > self.config.time_epsilon:
            raise ValueError("start must be emitted at its anchor, not backdated after passages")
        cohort = event.get("cohort")
        if not isinstance(cohort, list) or not cohort:
            raise ValueError("start requires a nonempty ordered cohort")
        keys = [required_string(member.get("vehicle_key"), "cohort vehicle_key") for member in cohort]
        if len(set(keys)) != len(keys):
            raise ValueError("duplicate cohort member")
        lane.serial += 1
        token = hashlib.sha256(json.dumps([self.run_id, lane_id, section_id], ensure_ascii=False).encode()).hexdigest()[:20]
        lane.active = {
            "schema": SEQUENCE_SCHEMA, "rule_version": RULE_VERSION,
            "run_id": self.run_id, "lane_id": lane_id, "section_id": section_id,
            "sequence_id": f"{token}:{lane.serial}", "start_kind": event["start_kind"],
            "permission_class": permission_class,
            "anchor_time": anchor, "anchor_evidence": event.get("anchor_evidence", {}),
            "cohort_size": len(keys), "cohort_vehicle_keys": keys,
            "valid_start": True, "closed": False, "cut_time": None, "cut_time_hi": None,
            "cut_reason": None, "passages": [], "startup_delay": None, "headways": [],
        }
        lane.cohort = keys
        lane.motions = {}
        for member in cohort:
            position = member.get("position")
            if position is not None:
                position = finite_number(position, "cohort position", 0)
            lane.motions[member["vehicle_key"]] = VehicleMotion(False, anchor, position)
        self.counts["starts"] += 1

    def _motion(self, lane_id: str, section_id: str, lane: LaneState, event: dict, timestamp: float) -> None:
        vehicle_key = required_string(event.get("vehicle_key"), "vehicle_key")
        if lane.active is None or vehicle_key not in lane.motions:
            return
        motion = lane.motions[vehicle_key]
        raw_speed = event.get("speed")
        if (event.get("present", True) is not True or isinstance(raw_speed, bool)
                or not isinstance(raw_speed, (int, float)) or not math.isfinite(raw_speed) or raw_speed < 0):
            self._cut(lane_id, section_id, lane, motion.last_time if motion.last_time is not None else timestamp, timestamp,
                      "unknown_motion_or_disappearance")
            return
        position = event.get("position")
        if position is not None:
            position = finite_number(position, "motion position", 0)
        progressed = (position is not None and motion.position is not None
                      and position - motion.position > self.config.progress_epsilon)
        if raw_speed > self.config.epsilon_zero or progressed:
            motion.started = True
        if motion.started and raw_speed <= self.config.epsilon_zero:
            boundary = event.get("cut_time", motion.last_time if motion.last_time is not None else timestamp)
            boundary = finite_number(boundary, "motion cut_time", 0)
            if boundary > timestamp + self.config.time_epsilon:
                raise ValueError("motion cut boundary cannot be in the future")
            self._cut(lane_id, section_id, lane, boundary, timestamp, "re_stopped_after_start")
            return
        motion.last_time = timestamp
        motion.position = position

    def _passage(self, lane_id: str, section_id: str, lane: LaneState, event: dict, timestamp: float) -> None:
        vehicle_key = required_string(event.get("vehicle_key"), "vehicle_key")
        vehicle_id = required_string(event.get("vehicle_id"), "vehicle_id")
        row = {key: value for key, value in event.items() if key not in {"event", "schema"}}
        row.update({"schema": "lane-discharge-passage-v1", "time": timestamp,
                    "vehicle_key": vehicle_key, "vehicle_id": vehicle_id,
                    "lane_passage_ordinal": lane.ordinal + 1})
        row.setdefault("type_id", "unknown")
        row.setdefault("turn", "unknown")
        row.setdefault("movement_id", "unknown")
        required_string(row["type_id"], "type_id")
        required_string(row["turn"], "turn")
        if not self.store.accept_passage(row):
            self.counts["duplicate_passage_observations"] += 1
            return
        lane.ordinal += 1
        self.counts["raw_passages"] += 1
        sequence = lane.active
        if sequence is None:
            self.counts["passages_without_stopped_segment"] += 1
            return
        index = len(sequence["passages"])
        if index >= len(lane.cohort) or lane.cohort[index] != vehicle_key:
            self._cut(lane_id, section_id, lane, timestamp, timestamp, "cohort_order_or_membership_changed")
            return
        if event.get("quality", "observed") in {"unknown", "ambiguous", "invalid"}:
            self._cut(lane_id, section_id, lane, timestamp, timestamp, "unknown_passage")
            return
        if event.get("permission_class", "unspecified") != sequence["permission_class"]:
            self._cut(lane_id, section_id, lane, timestamp, timestamp, "permission_class_changed")
            return
        previous = sequence["passages"][-1] if index else None
        if previous and (timestamp <= previous["time"] + self.config.time_epsilon
                         or lane.ordinal != previous["lane_passage_ordinal"] + 1):
            self._cut(lane_id, section_id, lane, timestamp, timestamp, "nonadjacent_or_nonpositive_headway")
            return
        row["local_j"] = index + 1
        sequence["passages"].append(row)
        lane.motions.pop(vehicle_key, None)
        recompute_labels(sequence)
        if len(sequence["passages"]) == len(lane.cohort):
            self._close(lane, timestamp, "cohort_end")

    def _close(self, lane: LaneState, timestamp: float, reason: str) -> None:
        if lane.active is None:
            return
        sequence = lane.active
        sequence.update(closed=True, cut_time=timestamp, cut_time_hi=timestamp, cut_reason=reason)
        recompute_labels(sequence)
        self.store.save(sequence)
        lane.active = None
        lane.motions.clear()
        lane.cohort = []

    def _truncate(self, sequence: dict, boundary: float, timestamp: float, reason: str) -> None:
        prior_count = len(sequence["passages"])
        sequence["passages"] = [row for row in sequence["passages"]
                                if row["time"] < boundary - self.config.time_epsilon]
        if boundary < sequence["anchor_time"] - self.config.time_epsilon:
            sequence["valid_start"] = False
        old_cut = sequence.get("cut_time")
        sequence.update(closed=True, cut_time=min(boundary, old_cut if old_cut is not None else boundary),
                        cut_time_hi=timestamp, cut_reason=reason)
        sequence.setdefault("cut_history", []).append({"time_lo": boundary, "time_hi": timestamp, "reason": reason})
        recompute_labels(sequence)
        self.counts["revoked_passage_labels"] += prior_count - len(sequence["passages"])
        self.store.save(sequence)

    def _cut(self, lane_id: str, section_id: str, lane: LaneState,
             boundary: float, timestamp: float, reason: str) -> None:
        self.cut_reasons[reason] += 1
        active_id = None
        if lane.active is not None:
            active_id = lane.active["sequence_id"]
            self._truncate(lane.active, boundary, timestamp, reason)
            lane.active = None
            lane.motions.clear()
            lane.cohort = []
        for sequence in self.store.affected(lane_id, section_id, boundary):
            if sequence["sequence_id"] != active_id:
                self._truncate(sequence, boundary, timestamp, reason)

    def finish(self) -> None:
        if self.finished:
            return
        for lane in self.lanes.values():
            self._close(lane, lane.last_time or 0.0, "observation_end")
        self.finished = True

    def summary(self) -> dict:
        coverage: dict[str, dict] = {}
        totals: Counter = Counter()
        for row in self.store.iter_sequences():
            key = json.dumps([row["lane_id"], row["section_id"]])
            lane = coverage.setdefault(key, {"lane_id": row["lane_id"], "section_id": row["section_id"],
                                            "sequences": 0, "zero_passage_sequences": 0,
                                            "startup_samples": 0, "headway_samples": 0,
                                            "headways_by_rank": {}, "cut_reasons": {}})
            lane["sequences"] += 1
            lane["zero_passage_sequences"] += int(not row["passages"])
            lane["startup_samples"] += int(row["startup_delay"] is not None)
            lane["headway_samples"] += len(row["headways"])
            reason = row["cut_reason"]
            lane["cut_reasons"][reason] = lane["cut_reasons"].get(reason, 0) + 1
            for pair in row["headways"]:
                rank = str(pair["local_j"])
                lane["headways_by_rank"][rank] = lane["headways_by_rank"].get(rank, 0) + 1
            for name in ("sequences", "zero_passage_sequences", "startup_samples", "headway_samples"):
                increment = {"sequences": 1, "zero_passage_sequences": int(not row["passages"]),
                             "startup_samples": int(row["startup_delay"] is not None),
                             "headway_samples": len(row["headways"])}[name]
                totals[name] += increment
        return {"schema": "lane-discharge-coverage-v1", "run_id": self.run_id,
                "rule_version": RULE_VERSION, "segmentation": asdict(self.config),
                "events": dict(self.counts), "observed_cut_reasons": dict(self.cut_reasons),
                "totals": dict(totals), "lanes": list(coverage.values()),
                "limitations": ["clean labels are conditional on observed gate evidence",
                                "finite-step observations cannot certify unobserved substep motion"]}
