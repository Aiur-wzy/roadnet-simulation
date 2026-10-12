"""Read-only, observation-time R/G inputs for SUMO-native road samples.

No lane choice, crossing projection or simulation-control API is used. Unknown
connections/calendars raise ValueError; callers must retain an unknown reason,
not replace an unknown signal by an invented green state.
"""
from __future__ import annotations

import math
from numbers import Integral

from collection.traci_signal_rg import TraCISignalRG, SIGNAL_AUDIT_FIELDS
from features.signal_rg_features import (
    SIGNAL_FEATURE_COLS, always_green_features, no_signal_features,
    network_green_horizon, signal_feature_contract,
)


NATIVE_SIGNAL_AUDIT_FIELDS = tuple(SIGNAL_AUDIT_FIELDS) + (
    "signal_selection_method", "signal_actual_lane_id", "signal_pair_connection_count",
    "signal_selected_link_count", "signal_selection_fallback_reason",
)


def _calendar(movement, clock):
    """Canonical complete R/G/Y calendar, including its absolute cycle anchor.

    Adjacent equal normalized colors are merged by RGSignalProgram already.
    Cyclic rotations and redundant same-color phase splits do not change this
    key. Different cycle periods are conservatively treated as non-equivalent.
    """
    pairs = tuple(zip(movement._segment_states, movement._segment_durations))
    start = next(i for i, state in enumerate(movement.states)
                 if state != movement.states[i - 1])
    origin = clock["end"] - clock["plan"]["ends"][clock["phase"]]
    anchor = origin + movement._phase_starts[start]
    elapsed, rotations = 0.0, []
    for index, (_, duration) in enumerate(pairs):
        rotations.append((pairs[index:] + pairs[:index], (anchor + elapsed) % movement.cycle_seconds))
        elapsed += duration
    canonical = min(item[0] for item in rotations)
    anchors = tuple(sorted(item[1] for item in rotations if item[0] == canonical))
    return movement.cycle_seconds, canonical, anchors


def _same_calendar(left, right):
    period, pairs, anchors = left
    other_period, other_pairs, other_anchors = right
    if not math.isclose(period, other_period, rel_tol=0, abs_tol=1e-7) or len(pairs) != len(other_pairs):
        return False
    if any(a[0] != b[0] or not math.isclose(a[1], b[1], rel_tol=0, abs_tol=1e-7)
           for a, b in zip(pairs, other_pairs)):
        return False
    # Repeated identical color patterns may have several equivalent anchors.
    return any(min(abs(a - b), period - abs(a - b)) <= 1e-7
               for a in anchors for b in other_anchors)


class NativeSignalReader:
    """Select an immediate road-pair signal using SUMO's current next-TLS.

    A currently incompatible physical lane may still need a strategic lane
    change, so the exact selected link is matched against the whole road pair.
    Without such a match, every connection in that pair must have an identical
    complete absolute calendar. Equality of the four values *now* is not enough.
    """

    def __init__(self, api, net_file):
        self.api = api
        self.reader = TraCISignalRG(api, net_file)
        self.net_file = self.reader.net_file
        self.pairs = {}
        for (road, target, _), items in self.reader.connections.items():
            self.pairs.setdefault((road, target), set()).update(items)
        self.pairs = {key: tuple(sorted(items)) for key, items in self.pairs.items()}
        self.lane_roads = {lane: road for (road, _), lane in self.reader.lanes.items()}
        self._equivalent_pairs = set()
        self._horizon_attempted = False
        self._horizon_error = None
        self._horizon = None
        self._network_programs = {}
        self._all_static_verified = False

    @property
    def clock_queries(self):
        return self.reader.clock_queries

    def _check_network_program(self, tls, plan):
        static = self._network_programs.get(tls)
        if static is None or (plan["id"], plan["durations"], plan["states"]) != (
                static["program_id"], static["durations"], static["states"]):
            raise ValueError(f"native green horizon net/queried live program mismatch: {tls}")

    def _clock(self, tls, now):
        clock = self.reader._clock(tls, now)
        if self._horizon is not None:
            self._check_network_program(tls, clock["plan"])
        return clock

    def _ensure_horizon(self):
        if not self._horizon_attempted:
            self._horizon_attempted = True
            try:
                self._horizon, self._network_programs = network_green_horizon(self.net_file)
                for tls, plan in self.reader.plans.items():
                    self._check_network_program(tls, plan)
            except ValueError as exc:
                self._horizon_error = str(exc)
        if self._horizon_error is not None:
            raise ValueError(self._horizon_error)
        return self._horizon

    def verify_all_static(self, now):
        """Optional one-time full-network runtime check; never done per entry."""
        now = self._validate_now(now)
        self._ensure_horizon()
        if not self._all_static_verified:
            for tls in sorted(self._network_programs):
                self._clock(tls, now)
            self._all_static_verified = True
        return {"verified_tls_count": len(self._network_programs), "green_horizon": self._horizon}

    def _validate_now(self, now):
        if isinstance(now, bool):
            raise ValueError("native signal observation time must be finite")
        now = float(now)
        if not math.isfinite(now) or now < 0 or not math.isclose(
                float(self.api.simulation.getTime()), now, rel_tol=0, abs_tol=1e-9):
            raise ValueError("native signal reference must equal current SUMO observation time")
        return now

    def _controlled(self, connections, now):
        result = []
        for item in connections:
            clock = self._clock(item[0], now)
            movement = self.reader._movement(clock["plan"], item)
            result.append((item, clock, movement))
        return result

    def features(self, vehicle_id, route, index, actual_lane_id, now):
        now = self._validate_now(now)
        if isinstance(index, bool) or not isinstance(index, Integral) or not 0 <= index < len(route):
            raise ValueError("invalid native signal route occurrence")
        index = int(index)
        if self.lane_roads.get(actual_lane_id) != route[index]:
            raise ValueError("actual SUMO lane does not belong to the current external road")
        audit = dict(signal_status="", signal_tls_id="", signal_link_indices="", signal_program_id="",
                     signal_anchor_time=now, signal_anchor_phase="", signal_anchor_next_switch_time="",
                     signal_reference_time=now, signal_projection_seconds=0.0,
                     signal_selection_method="none", signal_actual_lane_id=actual_lane_id,
                     signal_pair_connection_count=0, signal_selected_link_count=0,
                     signal_selection_fallback_reason="")
        if index == len(route) - 1:
            audit["signal_status"] = "no_outgoing_route"
            return dict(no_signal_features(), **audit)
        pair = route[index], route[index + 1]
        connections = self.pairs.get(pair, ())
        if not connections:
            raise ValueError(f"no immediate native road-pair connection: {pair}")
        audit["signal_pair_connection_count"] = len(connections)
        controls = {item[0] for item in connections}
        if controls == {""}:
            audit["signal_status"] = "unsignalized_connection"
            return dict(always_green_features(self._ensure_horizon()), **audit)

        # Only the first upcoming TLS can describe this immediate controlled
        # exit. Searching farther could steal a future occurrence of a loop.
        upcoming = self.api.vehicle.getNextTLS(vehicle_id)
        matching = ()
        if upcoming:
            first = upcoming[0]
            if len(first) != 4:
                raise ValueError("malformed SUMO next-TLS result")
            tls, link, distance, state = first
            if isinstance(link, bool) or not isinstance(link, Integral) or not math.isfinite(float(distance)):
                raise ValueError("invalid SUMO next-TLS link/distance")
            matching = tuple(item for item in connections if (item[0], item[1]) == (str(tls), int(link)))
        if matching:
            selected = self._controlled(matching, now)
            item, clock, movement = selected[0]
            if str(state) != clock["plan"]["states"][clock["phase"]][item[1]]:
                raise ValueError("SUMO next-TLS state differs from queried static signal state")
            audit.update(signal_selection_method="sumo_next_tls", signal_status="controlled_static_selected")
        else:
            # Mixing controlled and uncontrolled exits without a selected
            # connection is not resolved by assigning the pair an arbitrary G.
            if "" in controls:
                raise ValueError(f"ambiguous controlled/uncontrolled immediate road pair: {pair}")
            selected = self._controlled(connections, now)
            if pair not in self._equivalent_pairs:
                calendars = [_calendar(movement, clock) for _, clock, movement in selected]
                if any(not _same_calendar(calendars[0], value) for value in calendars[1:]):
                    raise ValueError(f"missing selected immediate link and non-equivalent complete calendars: {pair}")
                self._equivalent_pairs.add(pair)
            item, clock, movement = selected[0]
            audit.update(signal_selection_method="whole_roadpair_calendar_equivalent",
                         signal_status="controlled_static_pair_equivalent",
                         signal_selection_fallback_reason="next_tls_not_immediate_pair" if upcoming else "next_tls_empty")
        values = movement.query(clock["phase"], clock["remaining"], 0.0)
        tls_ids = sorted({record[0][0] for record in selected})
        links = sorted({record[0][1] for record in selected})
        audit.update(signal_tls_id=";".join(tls_ids), signal_link_indices=";".join(map(str, links)),
                     signal_selected_link_count=len({(record[0][0], record[0][1]) for record in selected}),
                     signal_program_id=clock["plan"]["id"], signal_anchor_phase=clock["phase"],
                     signal_anchor_next_switch_time=clock["end"])
        return dict(values, **audit)

    def contract(self):
        result = signal_feature_contract(version="static-movement-rg-segments-v4")
        result.update(version="sumo-native-observed-rg-v1",
                      reference_time="current SUMO external-road observation time; no crossing projection",
                      movement="immediate external road pair; exact SUMO next-TLS or complete pair-calendar equivalence",
                      lookahead="zero; use current observation and already declared fixed static calendar",
                      fallback="never borrow remote TLS; unknown/ambiguous raises ValueError for caller disposition",
                      supported_programs="fixed sequential cyclic STATIC; WAUT, additional-file signal overrides and external signal mutations unsupported; queried runtime controllers checked",
                      audit_columns=list(NATIVE_SIGNAL_AUDIT_FIELDS),
                      horizon_validation="network-file static H cached once; queried controllers checked against it; optional verify_all_static",
                      runtime_unsignalized_green_seconds=self._horizon,
                      runtime_all_static_verified=self._all_static_verified,
                      runtime_programs={tls: {"program_id": plan["id"], "durations": plan["durations"],
                                             "states": plan["states"], "initial_cycle_origin": self.reader.initial_origins[tls]}
                                        for tls, plan in sorted(self.reader.plans.items())},
                      clock_queries=self.clock_queries)
        result.pop("crossing_policy", None)
        return result
