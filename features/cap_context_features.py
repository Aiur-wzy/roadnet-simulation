"""Versioned, label-independent F13 feature arithmetic and scanning reference.

This module deliberately has no SUMO, pandas, model or filesystem dependency.
Road/lane occupancy and signal dispatch are not changed by this feature ledger.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numbers
from typing import Dict, Iterable, Mapping, Tuple


FEATURE_SCHEMA = "road-time-cap-context-v1"
FEATURE_COLS = ["road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context"]
CONTEXT_CONTRACT = {
    "version": "free-clock-context-v1",
    "p_threshold": 0.05,
    "e_threshold": 2.0,
    "epsilon_seconds": 1e-9,
    "crossing_time_seconds": 3.0,
    "sampling": "upstream-discharge-known-reservations-v1",
    "geometry": "base-first-v1",
    "candidate_lane": "incoming-logical-lane-v1",
}


def _finite(value: float, name: str, *, positive: bool = False, minimum=None) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number, not bool")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result) or (positive and result <= 0) or (minimum is not None and result < minimum):
        raise ValueError(f"invalid {name}: {value!r}")
    return result


def validate_context_contract(value: Mapping) -> dict:
    if not isinstance(value, Mapping) or set(value) != set(CONTEXT_CONTRACT):
        raise ValueError("context_contract must contain exactly the versioned contract fields")
    result = dict(value)
    for name in ("version", "sampling", "geometry", "candidate_lane"):
        if result[name] != CONTEXT_CONTRACT[name]:
            raise ValueError(f"unsupported context contract {name}: {result[name]!r}")
    for name in ("p_threshold", "e_threshold", "epsilon_seconds", "crossing_time_seconds"):
        if isinstance(result[name], bool) or not isinstance(result[name], numbers.Real):
            raise ValueError(f"context contract {name} must be a JSON number")
    _finite(result["p_threshold"], "p_threshold", positive=True)
    _finite(result["e_threshold"], "e_threshold", minimum=1.0)
    _finite(result["epsilon_seconds"], "epsilon_seconds", minimum=0.0)
    _finite(result["crossing_time_seconds"], "crossing_time_seconds", minimum=0.0)
    for name in ("p_threshold", "e_threshold", "epsilon_seconds"):
        if result[name] != CONTEXT_CONTRACT[name]:
            raise ValueError(f"free-clock-context-v1 fixes {name}={CONTEXT_CONTRACT[name]}; use a new version to change it")
    return result


def make_context_contract(*, p_threshold=0.05, e_threshold=2.0,
                          epsilon_seconds=1e-9, crossing_time_seconds=3.0) -> dict:
    result = dict(CONTEXT_CONTRACT)
    result.update(p_threshold=float(p_threshold), e_threshold=float(e_threshold),
                  epsilon_seconds=float(epsilon_seconds), crossing_time_seconds=float(crossing_time_seconds))
    return validate_context_contract(result)


@dataclass(frozen=True)
class ContextConfig:
    p_threshold: float = 0.05
    e_threshold: float = 2.0
    epsilon_seconds: float = 1e-9
    crossing_time_seconds: float = 3.0

    def __post_init__(self):
        validate_context_contract(self.as_contract())

    def as_contract(self) -> dict:
        result = dict(CONTEXT_CONTRACT)
        result.update(p_threshold=self.p_threshold, e_threshold=self.e_threshold,
                      epsilon_seconds=self.epsilon_seconds, crossing_time_seconds=self.crossing_time_seconds)
        return result


def free_speed_cap(speed_factor: float, max_speed: float,
                   desired_max_speed: float, speed_limit: float) -> float:
    sf = _finite(speed_factor, "speed_factor", positive=True)
    maximum = _finite(max_speed, "max_speed", positive=True)
    desired = _finite(desired_max_speed, "desired_max_speed", positive=True)
    limit = _finite(speed_limit, "speed_limit", positive=True)
    return _finite(min(maximum, sf * desired, sf * limit), "free_speed_cap", positive=True)


def travel_distance(road_length: float, *, route_index: int,
                    vehicle_length: float, depart_pos: str = "base") -> float:
    length = _finite(road_length, "road_length", positive=True)
    if isinstance(route_index, bool) or int(route_index) != route_index or route_index < 0:
        raise ValueError("route_index must be a nonnegative integer")
    if route_index == 0:
        if depart_pos not in ("", "base"):
            raise ValueError(f"base-first-v1 does not support departPos={depart_pos!r}")
        position = _finite(vehicle_length, "vehicle_length", positive=True) + 0.1
        if position >= length:
            raise ValueError("base-first-v1 does not support short-road insertion")
        return length - position
    return length


def classify_context(max_deficit: float, sum_deficit: float, horizon: float,
                     config: ContextConfig = ContextConfig()) -> dict:
    dmax = _finite(max_deficit, "max_deficit", minimum=0.0)
    total = _finite(sum_deficit, "sum_deficit", minimum=0.0)
    h = _finite(horizon, "horizon", positive=True)
    if total + config.epsilon_seconds < dmax:
        raise ValueError("sum_deficit cannot be smaller than max_deficit")
    if dmax <= config.epsilon_seconds:
        p = e = 0.0
        context = 0
    else:
        p, e = dmax / h, total / dmax
        context = 1 if p < config.p_threshold else (2 if e < config.e_threshold else 3)
    return {"platoon_context": context, "P": p, "E": e,
            "max_deficit_seconds": dmax, "sum_deficit_seconds": total, "horizon_seconds": h}


@dataclass(frozen=True)
class ShadowRecord:
    vehicle_id: str
    route_index: int
    road_id: str
    lane_index: int
    planned_enter_time: float
    distance: float
    free_speed_cap: float
    occupied_length: float = 7.5

    def __post_init__(self):
        if not self.vehicle_id or not self.road_id or self.route_index < 0 or self.lane_index < 0:
            raise ValueError("shadow identity/road/lane must be valid")
        _finite(self.planned_enter_time, "planned_enter_time")
        _finite(self.distance, "distance", positive=True)
        _finite(self.free_speed_cap, "free_speed_cap", positive=True)
        _finite(self.occupied_length, "occupied_length", positive=True)

    @property
    def key(self) -> Tuple[str, int]:
        return self.vehicle_id, self.route_index

    @property
    def free_finish_time(self) -> float:
        return _finite(self.planned_enter_time + self.distance / self.free_speed_cap, "free_finish_time")


class ShadowLedger:
    """O(current logical lane size) query; expected O(1) record add/remove.

    Records include pending reservations. The query filters these without an
    activation event, future state look-ahead or an advancing maintenance clock.
    No record is removed merely because its vehicle has an invalid training label.
    """

    def __init__(self, config: ContextConfig = ContextConfig()):
        self.config = config
        self._lanes: Dict[Tuple[str, int], Dict[Tuple[str, int], ShadowRecord]] = {}
        self._records: Dict[Tuple[str, int], ShadowRecord] = {}
        self._vehicle_keys: Dict[str, Tuple[str, int]] = {}
        self.scanned_records = 0
        self.query_count = 0

    def reserve(self, record: ShadowRecord) -> None:
        if record.key in self._records or record.vehicle_id in self._vehicle_keys:
            raise ValueError(f"duplicate/live shadow reservation: {record.key}")
        self._records[record.key] = record
        self._vehicle_keys[record.vehicle_id] = record.key
        self._lanes.setdefault((record.road_id, record.lane_index), {})[record.key] = record

    def release(self, vehicle_id: str, route_index=None) -> bool:
        key = self._vehicle_keys.get(vehicle_id)
        if key is None:
            return False
        if route_index is not None and key[1] != route_index:
            raise ValueError(f"wrong occurrence release: {vehicle_id}/{route_index}, current={key}")
        record = self._records.pop(key)
        del self._vehicle_keys[vehicle_id]
        lane = self._lanes[(record.road_id, record.lane_index)]
        del lane[key]
        if not lane:
            del self._lanes[(record.road_id, record.lane_index)]
        return True

    def lane_load(self, road_id: str, lane_index: int) -> Tuple[int, float]:
        # Used only by the collection-side static lane selector, not the model.
        records = self._lanes.get((road_id, lane_index), {})
        return len(records), sum(record.occupied_length for record in records.values())

    def query(self, ego: ShadowRecord, decision_time: float) -> dict:
        decision = _finite(decision_time, "decision_time")
        if ego.planned_enter_time < decision - self.config.epsilon_seconds:
            raise ValueError("ego planned entry must not precede its decision")
        horizon = ego.distance / ego.free_speed_cap
        f_i = ego.free_finish_time
        dmax = total = compensation = 0.0
        scanned = eligible = positive = pending = 0
        for key, other in self._lanes.get((ego.road_id, ego.lane_index), {}).items():
            scanned += 1
            if key == ego.key:
                continue
            # Exact planned-enter boundary is intentional; epsilon is for the
            # deficit classification only, not permission to observe the future.
            if other.planned_enter_time > decision:
                pending += 1
                continue
            eligible += 1
            deficit = max(0.0, other.free_finish_time - f_i)
            next_total = total + deficit
            if abs(total) >= abs(deficit):
                compensation += (total - next_total) + deficit
            else:
                compensation += (deficit - next_total) + total
            total = next_total
            dmax = max(dmax, deficit)
            positive += int(deficit > 0.0)
        self.query_count += 1
        self.scanned_records += scanned
        result = classify_context(dmax, total + compensation, horizon, self.config)
        result.update(decision_time=decision, planned_enter_time=ego.planned_enter_time,
                      free_finish_time=f_i, candidate_count=eligible, positive_count=positive,
                      pending_count=pending, scanned_count=scanned,
                      logical_lane_index=ego.lane_index, travel_distance=ego.distance)
        return result

    def __len__(self) -> int:
        return len(self._records)
