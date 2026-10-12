"""Collection-only ABC refinement of the versioned free-clock context feature.

Overdue is a logical free-clock overrun proxy, never measured physical waiting.
The old ledger's membership, event ordering and compensated P/E arithmetic are
preserved. No extra vehicle scan, SUMO dependency or behavior mutation is added.
"""
from __future__ import annotations

import math
import numbers

from features.cap_context_features import (
    ContextConfig, ShadowLedger, ShadowRecord, _finite, classify_context,
    validate_context_contract,
)


ABC_FEATURE_SCHEMA = "road-time-cap-context-abc-v1"
ABC_CONTEXT_IDS = (0, 1, 2, 3, 4, 5)
ABC_AUDIT_FIELDS = (
    "legacy_platoon_context", "context0_subclass", "max_overdue_seconds",
    "sum_overdue_seconds", "overdue_count", "overdue_fraction",
    "mean_overdue_seconds", "context0_overdue_threshold_seconds",
)


def _threshold(value):
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ValueError("overdue_threshold_seconds must be a finite nonnegative number")
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise ValueError("overdue_threshold_seconds must be a finite nonnegative number")
    return result


def abc_context_contract(config: ContextConfig = ContextConfig(), threshold=0.0) -> dict:
    """Independent contract: an ABC collector must never claim the old schema."""
    if not isinstance(config, ContextConfig):
        raise ValueError("config must be a validated ContextConfig")
    threshold = _threshold(threshold)
    return {
        "version": "free-clock-context-abc-v1",
        "base_context_contract": config.as_contract(),
        "context_ids": list(ABC_CONTEXT_IDS),
        "context_mapping": {
            "0": "A", "1": "legacy_1", "2": "legacy_2",
            "3": "legacy_3", "4": "B", "5": "C",
        },
        "context0_overdue_threshold_seconds": threshold,
        "overdue_epsilon_seconds": config.epsilon_seconds,
        "overdue_formula": "max(0, decision_time - candidate.free_finish_time)",
        "overdue_aggregation_population": "same eligible non-ego logical-lane records as base query; planned_enter_time <= decision_time",
        "context0_subclass_rules": {
            "A": "legacy_context == 0 and eligible_count == 0",
            "B": "legacy_context == 0 and eligible_count > 0 and max_overdue_seconds <= threshold + epsilon",
            "C": "legacy_context == 0 and eligible_count > 0 and max_overdue_seconds > threshold + epsilon",
            "not_context0": "legacy_context in (1, 2, 3); preserve original context ID",
        },
        "overdue_count_rule": "count(overdue_seconds > epsilon); independent of classification threshold",
        "overdue_fraction_denominator": "eligible_count; zero if eligible_count == 0",
        "mean_overdue_denominator": "eligible_count; zero if eligible_count == 0",
        "continuous_audit_scope": "record all contexts, including legacy 1/2/3; do not clip at threshold",
        "interpretation": "logical free-clock overrun proxy, not physical waiting or measured queue time",
    }


def _validate_abc_contract_types(value, expected, path="abc_context_contract"):
    """Require JSON types before equality: bool must never pass as 0 or 1."""
    if type(expected) is dict:
        if (type(value) is not dict or any(type(key) is not str for key in value)
                or set(value) != set(expected)):
            raise ValueError(f"{path} must contain exactly the versioned contract fields")
        for key, reference in expected.items():
            _validate_abc_contract_types(value[key], reference, f"{path}.{key}")
    elif type(expected) is list:
        if type(value) is not list or len(value) != len(expected):
            raise ValueError(f"{path} must be the versioned JSON array")
        for index, (item, reference) in enumerate(zip(value, expected)):
            _validate_abc_contract_types(item, reference, f"{path}[{index}]")
    elif type(expected) is float:
        if type(value) not in (int, float):
            raise ValueError(f"{path} must be a finite JSON number, not bool")
        try:
            finite = math.isfinite(float(value))
        except (OverflowError, TypeError, ValueError):
            finite = False
        if not finite:
            raise ValueError(f"{path} must be a finite JSON number")
    elif type(value) is not type(expected):
        raise ValueError(f"{path} must have JSON type {type(expected).__name__}")


def validate_abc_context_contract(value) -> dict:
    """Validate one ABC contract and return a fresh canonical JSON dictionary.

    Only crossing time and the nonnegative overdue threshold may vary in v1.
    Callers loading multiple files must also require equality of the returned
    dictionaries: individually valid but different thresholds cannot be mixed.
    The four-state contract on its own is deliberately not an ABC contract.
    """
    _validate_abc_contract_types(value, abc_context_contract())
    base = validate_context_contract(value["base_context_contract"])
    config = ContextConfig(
        p_threshold=float(base["p_threshold"]),
        e_threshold=float(base["e_threshold"]),
        epsilon_seconds=float(base["epsilon_seconds"]),
        crossing_time_seconds=float(base["crossing_time_seconds"]),
    )
    expected = abc_context_contract(
        config, value["context0_overdue_threshold_seconds"],
    )
    if value != expected:
        raise ValueError("unsupported ABC context contract values, mapping, or rules")
    return expected


class ABCShadowLedger(ShadowLedger):
    """Same one-pass O(current logical-lane size) query, with ABC audit values."""

    def __init__(self, config: ContextConfig = ContextConfig(), *, overdue_threshold_seconds=0.0):
        contract = abc_context_contract(config, overdue_threshold_seconds)
        super().__init__(config)
        self.overdue_threshold_seconds = contract["context0_overdue_threshold_seconds"]

    def query(self, ego: ShadowRecord, decision_time: float) -> dict:
        decision = _finite(decision_time, "decision_time")
        if ego.planned_enter_time < decision - self.config.epsilon_seconds:
            raise ValueError("ego planned entry must not precede its decision")
        horizon = ego.distance / ego.free_speed_cap
        f_i = ego.free_finish_time
        dmax = total = compensation = 0.0
        overdue_max = overdue_total = overdue_compensation = 0.0
        scanned = eligible = positive = pending = overdue_count = 0
        for key, other in self._lanes.get((ego.road_id, ego.lane_index), {}).items():
            scanned += 1
            if key == ego.key:
                continue
            # Exact base membership boundary; epsilon does not activate pending.
            if other.planned_enter_time > decision:
                pending += 1
                continue
            eligible += 1
            f_j = other.free_finish_time
            # Keep the V1 Neumaier sequence unchanged, including >0 counting.
            deficit = max(0.0, f_j - f_i)
            next_total = total + deficit
            if abs(total) >= abs(deficit):
                compensation += (total - next_total) + deficit
            else:
                compensation += (deficit - next_total) + total
            total = next_total
            dmax = max(dmax, deficit)
            positive += int(deficit > 0.0)

            overdue = max(0.0, decision - f_j)
            next_overdue_total = overdue_total + overdue
            if abs(overdue_total) >= abs(overdue):
                overdue_compensation += (overdue_total - next_overdue_total) + overdue
            else:
                overdue_compensation += (overdue - next_overdue_total) + overdue_total
            overdue_total = next_overdue_total
            overdue_max = max(overdue_max, overdue)
            overdue_count += int(overdue > self.config.epsilon_seconds)

        self.query_count += 1
        self.scanned_records += scanned
        result = classify_context(dmax, total + compensation, horizon, self.config)
        result.update(decision_time=decision, planned_enter_time=ego.planned_enter_time,
                      free_finish_time=f_i, candidate_count=eligible, positive_count=positive,
                      pending_count=pending, scanned_count=scanned,
                      logical_lane_index=ego.lane_index, travel_distance=ego.distance)
        legacy_context = result["platoon_context"]
        subclass = "not_context0"
        if legacy_context == 0:
            if eligible == 0:
                subclass, result["platoon_context"] = "A", 0
            elif overdue_max <= self.overdue_threshold_seconds + self.config.epsilon_seconds:
                subclass, result["platoon_context"] = "B", 4
            else:
                subclass, result["platoon_context"] = "C", 5
        overdue_sum = _finite(overdue_total + overdue_compensation, "sum_overdue_seconds", minimum=0.0)
        result.update(
            legacy_platoon_context=legacy_context,
            context0_subclass=subclass,
            max_overdue_seconds=_finite(overdue_max, "max_overdue_seconds", minimum=0.0),
            sum_overdue_seconds=overdue_sum,
            overdue_count=overdue_count,
            overdue_fraction=overdue_count / eligible if eligible else 0.0,
            mean_overdue_seconds=overdue_sum / eligible if eligible else 0.0,
            context0_overdue_threshold_seconds=self.overdue_threshold_seconds,
        )
        return result
