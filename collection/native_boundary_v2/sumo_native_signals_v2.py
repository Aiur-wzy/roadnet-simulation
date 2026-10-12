"""Independent snapshot-clock correction; production native-v1 is unchanged.

For the verified SUMO Euler observation convention, one completed simulation
step returned at API time T exposes the vehicle/FCD and live phase snapshot
labelled T-dt. This module gives that timestamp a separate explicit identity.
It does not shift actual departure/arrival event timestamps, reconstruct road
boundaries, change dynamics, or alter native-v1's interval-label algorithm.

This isolated candidate deliberately lives in one file so it can be loaded
with importlib without shadowing the production ``collection`` package.
"""
from __future__ import annotations

import math

from collection.sumo_native_signals import NativeSignalReader
from collection.sumo_native_state import NativeState, sampling_contract


CLOCK_SCHEMA = "sumo-euler-last-completed-snapshot-v2"
STATE_SCHEMA = "road-time-sumo-native-snapshot-clock-v2"


def snapshot_clock(api):
    """Return clocks after a completed step; never silently clamp below zero."""
    raw_time = api.simulation.getTime()
    raw_delta = api.simulation.getDeltaT()
    if isinstance(raw_time, bool) or isinstance(raw_delta, bool):
        raise ValueError("snapshot clock requires numeric API time and step")
    api_time, delta = float(raw_time), float(raw_delta)
    if (not math.isfinite(api_time) or not math.isfinite(delta)
            or delta <= 0 or api_time < delta):
        raise ValueError("snapshot clock requires a completed positive SUMO step")
    return {"api_return_time": api_time, "snapshot_time": api_time - delta,
            "step_seconds": delta, "clock_schema": CLOCK_SCHEMA}


def _checked_snapshot_reference(api, now):
    if isinstance(now, bool):
        raise ValueError("snapshot reference must be a finite numeric time")
    now = float(now)
    clock = snapshot_clock(api)
    if not math.isfinite(now) or not math.isclose(
            now, clock["snapshot_time"], rel_tol=0, abs_tol=1e-9):
        raise ValueError("snapshot reference must equal SUMO API time minus step; do not pass API return time")
    return now, clock


class NativeSignalReaderV2(NativeSignalReader):
    """Use the live selected connection/phase at its actual snapshot clock.

    Call features(..., snapshot_time), NOT features(..., api_return_time).
    The inherited _clock receives snapshot_time, so nextSwitch - reference
    retains the last visible phase's remaining time, including boundary steps.
    The source phase, link selection, R/G/Y grouping and yellow policy remain
    unchanged. No controller setter is used.
    """

    def _validate_now(self, now):
        # Also used by inherited verify_all_static. Deliberately do not call
        # v1's validation, which requires reference == API return time.
        return _checked_snapshot_reference(self.api, now)[0]

    def features(self, vehicle_id, route, index, actual_lane_id, now):
        values = super().features(vehicle_id, route, index, actual_lane_id, now)
        _, clock = _checked_snapshot_reference(self.api, now)
        values.update(signal_clock_schema=CLOCK_SCHEMA,
                      signal_api_return_time=clock["api_return_time"],
                      signal_snapshot_time=clock["snapshot_time"],
                      signal_step_seconds=clock["step_seconds"])
        return values

    def contract(self):
        result = super().contract()
        result.update(version="sumo-native-snapshot-rg-v2",
                      reference_time="last completed snapshot time = API getTime minus getDeltaT; not exact sub-step road entry",
                      clock_schema=CLOCK_SCHEMA,
                      validated_domain="SUMO 1.27 semi-implicit Euler; keep API/event/snapshot clocks distinct",
                      actual_event_time_policy="departure/arrival event timestamps are not shifted by this reader")
        result["audit_columns"] += ["signal_clock_schema", "signal_api_return_time",
                                    "signal_snapshot_time", "signal_step_seconds"]
        return result


class NativeStateV2(NativeState):
    """Freeze native snapshots/free clocks at snapshot_time, not API time.

    This class intentionally keeps the original interval labels. The separate
    boundary-reconstruction experiment must not infer restored labels from it.
    Both step(snapshot_time, ...) and step_from_api(api, ...) are available;
    the signals object must carry the same read-only TraCI connection.
    """

    def __init__(self, routes, vehicle_metadata, signals, emit):
        super().__init__(routes, vehicle_metadata, signals, emit)
        if not hasattr(signals, "api"):
            raise ValueError("snapshot-clock state requires a signal reader with its TraCI connection")
        self.snapshot_step_seconds = None

    def _base(self, vehicle, index):
        row = super()._base(vehicle, index)
        row.update(schema=STATE_SCHEMA, snapshot_clock_schema=CLOCK_SCHEMA,
                   snapshot_clock_step_seconds=self.snapshot_step_seconds)
        return row

    def step(self, now, observations, arrivals=(), abnormal=(), internal=()):
        now, clock = _checked_snapshot_reference(self.signals.api, now)
        delta = clock["step_seconds"]
        if self.snapshot_step_seconds is not None and not math.isclose(
                delta, self.snapshot_step_seconds, rel_tol=0, abs_tol=1e-12):
            raise ValueError("changing SUMO step invalidates this snapshot-clock collection contract")
        # Set before emitting first-step missing occurrence rows.
        self.snapshot_step_seconds = delta
        return super().step(now, observations, arrivals, abnormal, internal)

    def step_from_api(self, api, observations, arrivals=(), abnormal=(), internal=()):
        if api is not self.signals.api:
            raise ValueError("state and signals must use the same TraCI connection")
        return self.step(snapshot_clock(api)["snapshot_time"], observations, arrivals, abnormal, internal)

    def censor(self, now):
        now, _ = _checked_snapshot_reference(self.signals.api, now)
        if self.last_time is not None and now < self.last_time:
            raise ValueError("censor snapshot time precedes last observation")
        return super().censor(now)

    def censor_from_api(self, api):
        if api is not self.signals.api:
            raise ValueError("state and signals must use the same TraCI connection")
        return self.censor(snapshot_clock(api)["snapshot_time"])


def sampling_contract_v2():
    result = sampling_contract()
    result.update(schema=STATE_SCHEMA, clock_schema=CLOCK_SCHEMA,
                  clock="first observed snapshot time (API getTime - step) + observed remaining lane distance / entry cap",
                  signal="same observed snapshot clock; live immediate selected link and nextSwitch; no C++ shadow",
                  labels="UNCHANGED v1: right-end speed integration between same-occurrence snapshot times only",
                  boundary="UNCHANGED v1: no inferred sub-step or skipped-road labels; separate reconstruction required",
                  actual_event_time_policy="actual departure/arrival remain SUMO event timestamps, not API return time",
                  legacy_compatibility="separate candidate schema; do not silently mix with native-v1 or shadow data")
    return result
