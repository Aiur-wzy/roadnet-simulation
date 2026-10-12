"""Pure feature-source checks for the streaming collector; no SUMO or files.

These tests deliberately do not credit NativeStateV2 with collector-level
subscription, expected-live-ID, per-step continuity or boundary-label checks.
"""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from collection.native_boundary_v2.sumo_native_signals_v2 import (
    CLOCK_SCHEMA, NativeSignalReaderV2, NativeStateV2, snapshot_clock,
)
from collection.sumo_native_state import FEATURES, LABELS, Observation


class ClockAPI:
    def __init__(self, api_time=1., delta=1.):
        self.api_time, self.delta = api_time, delta
        self.simulation = NS(getTime=lambda: self.api_time,
                             getDeltaT=lambda: self.delta)

    def at_snapshot(self, time):
        self.api_time = time + self.delta


class Signals:
    def __init__(self, api, unknown=()):
        self.api, self.unknown, self.calls = api, set(unknown), []

    def features(self, vehicle, route, index, lane, now):
        self.calls.append((vehicle, index, lane, now))
        if vehicle in self.unknown:
            raise ValueError("local signal feature unavailable")
        clock = snapshot_clock(self.api)
        return dict(entry_signal_type="G", entry_signal_remaining_seconds=8.,
                    next_signal_type="R", next_signal_duration_seconds=7.,
                    signal_clock_schema=CLOCK_SCHEMA,
                    signal_api_return_time=clock["api_return_time"],
                    signal_snapshot_time=clock["snapshot_time"],
                    signal_step_seconds=clock["step_seconds"])


def observation(vehicle="v", *, road="a", index=0, lane="a_0",
                position=0., speed=5., cap=10., length=100.):
    return Observation(vehicle, road, index, lane, position, speed, cap,
                       length, 10.)


def make_state(routes=None, *, unknown=()):
    routes = dict(routes or {"v": ("a",)})
    api = ClockAPI()
    signals = Signals(api, unknown)
    rows = []
    metadata = {v: dict(actual_depart=0., planned_depart=0., speed_factor=1.)
                for v in routes}
    state = NativeStateV2(routes, metadata, signals, rows.append)
    return state, api, signals, rows


def advance(state, api, time, observations, arrivals=()):
    api.at_snapshot(time)
    state.step_from_api(api, observations, arrivals=arrivals)


class NativeStreamingFeatureSourceTests(unittest.TestCase):
    def test_completed_snapshot_clock_and_event_metadata_are_distinct(self):
        state, api, _, rows = make_state()
        advance(state, api, 0., [observation(position=5.1)])
        advance(state, api, 1., [observation(position=10.1)])
        advance(state, api, 2., [], arrivals=("v",))
        row = rows[0]
        self.assertEqual(row["actual_depart"], 0.)
        self.assertEqual(row["observed_enter_time"], 0.)
        self.assertEqual(row["observed_exit_time"], 2.)
        self.assertEqual(row["signal_snapshot_time"], 0.)
        self.assertEqual(row["signal_api_return_time"], 1.)
        self.assertEqual(row["snapshot_clock_schema"], CLOCK_SCHEMA)
        self.assertEqual(row["snapshot_clock_step_seconds"], 1.)

    def test_api_return_time_is_rejected_as_feature_reference(self):
        state, api, _, rows = make_state()
        with self.assertRaisesRegex(ValueError, "snapshot reference"):
            state.step(api.api_time, [observation()])
        self.assertEqual(state.active, {})
        self.assertEqual(rows, [])

    def test_changed_step_is_rejected_before_new_features(self):
        state, api, signals, _ = make_state({"v": ("a",), "w": ("a",)})
        advance(state, api, 0., [observation()])
        api.delta = .5
        api.at_snapshot(.5)
        with self.assertRaisesRegex(ValueError, "changing SUMO step"):
            state.step_from_api(api, [observation(), observation("w")])
        self.assertEqual(len(signals.calls), 1)
        self.assertNotIn("w", state.active)

    def test_connection_identity_is_checked(self):
        state, api, _, _ = make_state()
        with self.assertRaisesRegex(ValueError, "same TraCI connection"):
            state.step_from_api(ClockAPI(), [observation()])
        with self.assertRaisesRegex(ValueError, "same TraCI connection"):
            state.censor_from_api(ClockAPI())

    def test_same_snapshot_flow_is_independent_of_iteration_order(self):
        results = []
        for reverse in (False, True):
            state, api, _, _ = make_state({v: ("a",) for v in ("ego", "front", "other")})
            observed = [observation("ego"), observation("front", position=50.),
                        observation("other", lane="a_1", position=80.)]
            advance(state, api, 0., list(reversed(observed)) if reverse else observed)
            results.append(deepcopy(state.active["ego"].features))
        self.assertEqual(results[0], results[1])
        self.assertEqual((results[0]["road_flow"], results[0]["lane_flow"],
                          results[0]["candidate_count"]), (3, 2, 1))

    def test_lane_change_moves_membership_without_refreshing_entry_features(self):
        state, api, signals, rows = make_state({"front": ("a",), "ego": ("a",)})
        advance(state, api, 0., [observation("front", position=40., cap=2.)])
        frozen = deepcopy(state.active["front"].features)
        free_finish = state.active["front"].free_finish
        advance(state, api, 1., [observation("front", lane="a_1", position=45., cap=10.),
                                 observation("ego", lane="a_1")])
        self.assertEqual(state.active["front"].features, frozen)
        self.assertEqual(state.active["front"].free_finish, free_finish)
        self.assertEqual(state.active["ego"].features["candidate_count"], 1)
        self.assertEqual(state.active["ego"].features["lane_flow"], 2)
        self.assertEqual(sum(v == "front" for v, *_ in signals.calls), 1)
        advance(state, api, 2., [], arrivals=("front", "ego"))
        row = next(r for r in rows if r["vehicle_id"] == "front")
        self.assertEqual(row["entry_lane_id"], "a_0")
        self.assertEqual(row["lane_changes_within_road"], 1)

    def test_local_signal_unknown_does_not_remove_physical_member(self):
        state, api, _, rows = make_state({"ego": ("a",), "front": ("a",)}, unknown=("front",))
        advance(state, api, 0., [observation("ego"), observation("front", position=50.)])
        self.assertEqual(state.active["front"].features["feature_valid"], 0)
        ego = state.active["ego"].features
        self.assertEqual(ego["feature_valid"], 1)
        self.assertEqual((ego["road_flow"], ego["lane_flow"], ego["candidate_count"]), (2, 2, 1))
        advance(state, api, 1., [observation("ego", position=5.), observation("front", position=55.)])
        advance(state, api, 2., [], arrivals=("ego", "front"))
        bad = next(r for r in rows if r["vehicle_id"] == "front")
        self.assertEqual((bad["feature_valid"], bad["trainable"]), (0, 0))
        self.assertIn("local signal", bad["feature_error"])

    def test_single_visible_snapshot_can_have_valid_features_without_v1_label(self):
        state, api, _, rows = make_state()
        advance(state, api, 0., [observation()])
        advance(state, api, 1., [], arrivals=("v",))
        self.assertEqual(rows[0]["feature_valid"], 1)
        self.assertEqual(rows[0]["label_valid"], 0)
        self.assertEqual(rows[0]["trainable"], 0)
        self.assertTrue(all(name in rows[0] for name in FEATURES))
        self.assertTrue(all(name not in rows[0] for name in LABELS))

    def test_zero_remaining_horizon_stays_explicit_local_unknown(self):
        state, api, _, rows = make_state()
        advance(state, api, 0., [observation(position=100.)])
        advance(state, api, 1., [], arrivals=("v",))
        self.assertEqual(rows[0]["feature_valid"], 0)
        self.assertIn("nonpositive observed remaining", rows[0]["feature_error"])

    def test_repeated_road_occurrences_get_distinct_frozen_feature_keys(self):
        state, api, signals, rows = make_state({"v": ("a", "b", "a")})
        for time, road in enumerate(("a", "b", "a")):
            advance(state, api, float(time),
                    [observation(road=road, index=time, lane=road + "_0")])
        advance(state, api, 3., [], arrivals=("v",))
        self.assertEqual([(r["road_id"], r["route_index"]) for r in rows],
                         [("a", 0), ("b", 1), ("a", 2)])
        self.assertEqual([r["signal_snapshot_time"] for r in rows], [0., 1., 2.])
        self.assertEqual(len(signals.calls), 3)

    def test_closed_feature_source_rows_do_not_require_full_run_retention(self):
        state, api, _, rows = make_state()
        advance(state, api, 0., [observation()])
        advance(state, api, 1., [], arrivals=("v",))
        state.discard_arrived(("v",))
        self.assertEqual(len(rows), 1)
        self.assertEqual(state.active, {})
        self.assertEqual(state.routes, {})
        self.assertEqual(state.metadata, {})
        self.assertEqual(dict(state.last_closed), {})


class NativeStreamingSignalClockTests(unittest.TestCase):
    def make_reader(self, snapshot):
        root = ET.fromstring('''<net>
          <edge id="a" to="j"><lane id="a_0" index="0"/></edge>
          <edge id="b" to="k"><lane id="b_0" index="0"/></edge>
          <junction id="j" type="traffic_light"/>
          <junction id="k" type="dead_end"/>
          <connection from="a" to="b" fromLane="0" toLane="0" tl="T" linkIndex="0"/>
        </net>''')
        api = ClockAPI(snapshot + 1.)
        durations, states = (10., 3., 7.), ("G", "y", "r")
        phase = 0 if snapshot < 10 else 1 if snapshot < 13 else 2
        ends = (10., 13., 20.)
        api.trafficlight = NS(getProgram=lambda _: "0", getPhase=lambda _: phase,
            getNextSwitch=lambda _: ends[phase], getRedYellowGreenState=lambda _: states[phase],
            getControlledLinks=lambda _: ((('a_0', 'b_0', ''),),),
            getAllProgramLogics=lambda _: [NS(programID="0", type=0, phases=tuple(
                NS(duration=d, state=s, next=()) for d, s in zip(durations, states)))])
        api.vehicle = NS(getNextTLS=lambda _: [("T", 0, 20., states[phase])])
        with patch("collection.traci_signal_rg.ET.parse", return_value=ET.ElementTree(root)), \
                patch("collection.traci_signal_rg.ET.iterparse", return_value=[]):
            return NativeSignalReaderV2(api, "in-memory.net.xml"), api

    def test_last_green_snapshot_keeps_one_second_at_api_boundary(self):
        reader, api = self.make_reader(9.)
        row = reader.features("v", ("a", "b"), 0, "a_0", 9.)
        self.assertEqual((row["entry_signal_type"], row["entry_signal_remaining_seconds"]), ("G", 1.))
        self.assertEqual((row["signal_snapshot_time"], row["signal_api_return_time"]), (9., 10.))
        with self.assertRaisesRegex(ValueError, "snapshot reference"):
            reader.features("v", ("a", "b"), 0, "a_0", api.api_time)

    def test_first_yellow_snapshot_uses_unchanged_yellow_policy(self):
        reader, _ = self.make_reader(10.)
        row = reader.features("v", ("a", "b"), 0, "a_0", 10.)
        self.assertEqual((row["signal_state_at_reference"], row["signal_yellow_skipped"]), ("Y", 1))
        self.assertEqual((row["entry_signal_type"], row["entry_signal_remaining_seconds"],
                          row["next_signal_type"], row["next_signal_duration_seconds"]), ("R", 7., "G", 10.))


if __name__ == "__main__":
    unittest.main()
