"""Independent pure-core checks: real observed lanes/occurrences, no SUMO."""
import math
import unittest

from collection.sumo_native_state import LABELS, NativeState, Observation


class Signals:
    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    def features(self, vehicle, route, index, lane, now):
        self.calls.append((vehicle, tuple(route), index, lane, now))
        if self.fail:
            raise ValueError("unknown actual signal")
        return {"entry_signal_type": "G", "entry_signal_remaining_seconds": 30.,
                "next_signal_type": "R", "next_signal_duration_seconds": 60.}


def obs(vehicle="v", road="a", index=0, lane=None, position=0., speed=5., cap=10., length=100., limit=10.):
    return Observation(vehicle, road, index, lane or road + "_0", position, speed, cap, length, limit)


def make_state(routes=None, signals=None):
    routes = routes or {"v": ("a",)}
    metadata = {v: {"speed_factor": 1., "actual_depart": 0., "planned_depart": 0.} for v in routes}
    emitted = []
    signals = signals or Signals()
    return NativeState(dict(routes), metadata, signals, emitted.append), emitted, signals


class NativeStateTests(unittest.TestCase):
    def test_flow_includes_ego_but_only_same_actual_lane_for_context(self):
        state, _, _ = make_state({v: ("a",) for v in ("ego", "front", "other")})
        state.step(0., [obs("ego"), obs("front", position=50.), obs("other", lane="a_1", position=90.)])
        features = state.active["ego"].features
        self.assertEqual((features["road_flow"], features["lane_flow"], features["candidate_count"]), (3, 2, 1))
        self.assertEqual(features["entry_lane_id"], "a_0")
        self.assertEqual(features["platoon_context"], 4)

    def test_equal_position_or_behind_not_ahead(self):
        state, _, _ = make_state({v: ("a",) for v in ("ego", "equal", "behind")})
        state.step(0., [obs("ego", position=30.), obs("equal", position=30.), obs("behind", position=5.)])
        self.assertEqual(state.active["ego"].features["candidate_count"], 0)
        self.assertEqual(state.active["ego"].features["context0_subclass"], "A")

    def test_context_slow_front_deficit_has_existing_abc_arithmetic(self):
        state, _, _ = make_state({"ego": ("a",), "front": ("a",)})
        state.step(0., [obs("ego"), obs("front", position=40., cap=2.)])
        features = state.active["ego"].features
        self.assertEqual(features["platoon_context"], 2)
        self.assertEqual((features["P"], features["E"]), (2., 1.))

    def test_overdue_real_front_selects_c_without_predicting_stop(self):
        state, _, _ = make_state({"ego": ("a",), "front": ("a",)})
        state.step(0., [obs("front", position=50.)])
        state.step(6., [obs("front", position=60.), obs("ego")])
        features = state.active["ego"].features
        self.assertEqual((features["platoon_context"], features["context0_subclass"]), (5, "C"))
        self.assertEqual(features["max_overdue_seconds"], 1.)

    def test_actual_lane_change_moves_membership_but_keeps_entry_clock(self):
        state, emitted, _ = make_state({"ego": ("a",), "front": ("a",)})
        state.step(0., [obs("front", lane="a_0", position=40., cap=2.)])
        finish = state.active["front"].free_finish
        state.step(1., [obs("front", lane="a_1", position=45., cap=10.), obs("ego", lane="a_1")])
        self.assertEqual(state.active["front"].free_finish, finish)
        self.assertEqual(state.active["front"].lane_changes, 1)
        self.assertEqual(state.active["ego"].features["lane_flow"], 2)
        self.assertEqual(state.active["ego"].features["candidate_count"], 1)
        self.assertEqual(state.active["front"].features["entry_lane_id"], "a_0")
        state.step(2., [], arrivals=("ego", "front"))
        row = next(r for r in emitted if r["vehicle_id"] == "front")
        self.assertEqual(row["lane_changes_within_road"], 1)

    def test_same_step_feature_freeze_is_independent_of_observation_order(self):
        routes = {v: ("a", "b") for v in ("ego", "front", "leaver")}
        results = []
        for reverse in (False, True):
            state, _, _ = make_state(routes)
            state.step(0., [obs("leaver", position=80.)])
            observations = [obs("ego"), obs("front", position=50., cap=5.), obs("leaver", "b", 1)]
            state.step(1., list(reversed(observations)) if reverse else observations)
            results.append({v: visit.features for v, visit in state.active.items()})
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[0]["ego"]["road_flow"], 2)
        self.assertEqual(results[0]["leaver"]["road_flow"], 1)

    def test_skipped_middle_road_is_missing_not_zero(self):
        state, emitted, _ = make_state({"v": ("a", "b", "c")})
        state.step(0., [obs()])
        state.step(1., [obs(position=10.)])
        state.step(2., [obs(road="c", index=2)])
        state.step(3., [obs(road="c", index=2, position=20.)])
        state.step(4., [], arrivals=("v",))
        self.assertEqual([r["route_index"] for r in emitted], [0, 1, 2])
        missing = emitted[1]
        self.assertEqual(missing["quality"], "unobserved_road_occurrence")
        self.assertEqual(missing["label_valid"], 0)
        self.assertTrue(all(k not in missing for k in LABELS))
        self.assertEqual(state.counts["road_occurrences"], 3)
        self.assertEqual(state.counts["valid_label"], 2)

    def test_initial_skip_and_all_unobserved_arrival_are_explicit(self):
        state, emitted, _ = make_state({"v": ("a", "b", "c")})
        state.step(0., [obs(road="c", index=2)])
        self.assertEqual([r["route_index"] for r in emitted], [0, 1])
        state.step(1., [], arrivals=("v",))
        self.assertEqual(emitted[-1]["quality"], "no_observed_interval")
        state2, rows2, _ = make_state({"v": ("a", "b")})
        state2.step(1., [], arrivals=("v",))
        self.assertEqual([r["quality"] for r in rows2], ["unobserved_road_occurrence"] * 2)

    def test_consecutive_repeated_road_id_keeps_occurrences_separate(self):
        state, emitted, _ = make_state({"v": ("a", "a")})
        state.step(0., [obs(index=0)])
        state.step(1., [obs(index=0, position=90.)])
        state.step(2., [obs(index=1)])
        state.step(3., [obs(index=1, position=50.)])
        state.step(4., [], arrivals=("v",))
        self.assertEqual([r["route_index"] for r in emitted], [0, 1])
        self.assertEqual([r[LABELS[0]] for r in emitted], [1., 1.])

    def test_internal_transition_does_not_add_crossing_time(self):
        state, emitted, _ = make_state({"v": ("a", "b")})
        state.step(0., [obs()])
        state.step(1., [obs(position=90.)])
        state.step(2., [], internal=("v",))  # real internal edge, excluded from external snapshot
        state.step(5., [obs(road="b", index=1)])
        state.step(6., [obs(road="b", index=1, position=20.)])
        state.step(7., [], arrivals=("v",))
        self.assertEqual([r[LABELS[0]] for r in emitted], [1., 1.])
        self.assertEqual([r["exit_boundary_unallocated_seconds"] for r in emitted], [1., 1.])

    def test_legal_observed_zero_is_not_unobserved_interval(self):
        state, emitted, _ = make_state()
        state.step(0., [obs(speed=0.)])
        state.step(1., [obs(speed=0.)])
        state.step(2., [], arrivals=("v",))
        self.assertEqual(emitted[0]["trainable"], 1)
        self.assertEqual([emitted[0][k] for k in LABELS], [0.] * 6)
        state2, rows2, _ = make_state()
        state2.step(0., [obs(speed=20.)])
        state2.step(1., [], arrivals=("v",))
        self.assertEqual(rows2[0]["trainable"], 0)
        self.assertEqual(rows2[0]["quality"], "no_observed_interval")
        self.assertNotIn(LABELS[0], rows2[0])

    def test_strict_speed_thresholds_and_consecutive_interval_time(self):
        state, emitted, _ = make_state()
        state.step(0., [obs()])
        for time, speed in enumerate((0., .1, .5, 1., 1.5, 2., 3., 5.), 1):
            state.step(float(time), [obs(speed=speed)])
        state.step(9., [], arrivals=("v",))
        self.assertEqual([emitted[0][k] for k in LABELS], [6., 5., 4., 3., 2., 1.])
        self.assertEqual(emitted[0]["sampled_seconds"], 8.)

    def test_unknown_signal_keeps_label_but_excludes_training(self):
        state, emitted, _ = make_state(signals=Signals(fail=True))
        state.step(0., [obs()])
        state.step(1., [obs(position=5.)])
        state.step(2., [], arrivals=("v",))
        self.assertEqual((emitted[0]["label_valid"], emitted[0]["feature_valid"], emitted[0]["trainable"]), (1, 0, 0))
        self.assertIn("unknown", emitted[0]["feature_error"])

    def test_censor_and_abnormal_remove_valid_label_status(self):
        for abnormal in (False, True):
            with self.subTest(abnormal=abnormal):
                state, emitted, _ = make_state()
                state.step(0., [obs()])
                state.step(1., [obs(position=10.)])
                if abnormal:
                    state.step(2., [], abnormal=("v",))
                else:
                    state.censor(2.)
                self.assertEqual((emitted[0]["label_valid"], emitted[0]["trainable"]), (0, 0))
                self.assertNotIn(LABELS[0], emitted[0])

    def test_discard_arrived_releases_active_routes_and_metadata(self):
        state, emitted, _ = make_state()
        state.step(0., [obs()])
        state.step(1., [obs(position=90.)])
        state.step(2., [], arrivals=("v",))
        state.discard_arrived(("v",))
        self.assertEqual(state.active, {})
        self.assertEqual(state.routes, {})
        self.assertEqual(state.metadata, {})
        self.assertEqual(dict(state.last_closed), {})
        self.assertEqual(len(emitted), 1)

    def test_reject_duplicate_vehicle_route_mismatch_and_rollback(self):
        cases = ([obs(), obs()], [obs(road="b")], [obs(index=2)])
        for observations in cases:
            with self.subTest(observations=observations):
                state, _, _ = make_state()
                with self.assertRaises(ValueError):
                    state.step(0., observations)
        state, _, _ = make_state({"v": ("a", "b")})
        state.step(0., [obs(road="b", index=1)])
        with self.assertRaises(ValueError):
            state.step(1., [obs()])

    def test_nonmonotone_clock_and_nonfinite_observation_fail(self):
        state, _, _ = make_state()
        state.step(0., [obs()])
        with self.assertRaises(ValueError):
            state.step(0., [obs()])
        for field in ("speed", "position", "cap", "length", "limit"):
            with self.subTest(field=field):
                state, _, _ = make_state()
                with self.assertRaises(ValueError):
                    state.step(0., [obs(**{field: math.nan})])

    def test_final_disappearance_without_arrival_is_not_normal_label(self):
        state, emitted, _ = make_state()
        state.step(0., [obs()])
        state.step(1., [obs(position=90.)])
        state.step(2., [])
        self.assertEqual(emitted[0]["trainable"], 0)
        self.assertNotIn(LABELS[0], emitted[0])

    def test_final_internal_marker_cannot_replace_confirmed_arrival(self):
        state, emitted, _ = make_state()
        state.step(0., [obs()])
        state.step(1., [obs(position=90.)])
        state.step(2., [], internal=("v",))
        self.assertEqual(emitted[0]["quality"], "unexplained_disappearance")
        self.assertEqual(state.counts["unexplained_disappearances"], 1)
        self.assertEqual(emitted[0]["trainable"], 0)

    def test_unknown_geometry_singleton_and_observed_label_are_separated(self):
        for intervals in (0, 1):
            with self.subTest(intervals=intervals):
                state, emitted, _ = make_state()
                state.step(0., [obs(length=1., position=1., speed=0.)])
                if intervals:
                    state.step(1., [obs(length=1., position=1., speed=0.)])
                state.step(2., [], arrivals=("v",))
                self.assertEqual(state.counts["unknown_features"], 1)
                self.assertEqual(state.counts["unusable_feature_rows"], intervals)
                self.assertEqual(emitted[0]["label_valid"], intervals)
                self.assertEqual(emitted[0]["trainable"], 0)

    def test_streaming_completed_visits_not_retained_by_core(self):
        state, _, _ = make_state()
        state.routes.clear()
        state.metadata.clear()
        seen = []
        state.emit = lambda row: seen.append((row["vehicle_id"], row["route_index"]))
        for i in range(30):
            vehicle = f"v{i}"
            state.routes[vehicle] = ("a",)
            state.metadata[vehicle] = {"speed_factor": 1.}
            state.step(i * 3., [obs(vehicle)])
            state.step(i * 3. + 1, [obs(vehicle, position=50.)])
            state.step(i * 3. + 2, [], arrivals=(vehicle,))
            state.discard_arrived((vehicle,))
            self.assertEqual(len(state.active) + len(state.routes) + len(state.metadata) + len(state.last_closed), 0)
        self.assertEqual(len(seen), 30)
        self.assertEqual(state.counts["road_occurrences"], 30)

    def test_negative_entry_speed_and_position_fail(self):
        for change in ({"speed": -1.}, {"position": -1.}):
            with self.subTest(change=change):
                state, _, _ = make_state()
                with self.assertRaises(ValueError):
                    state.step(0., [obs(**change)])

    def test_nonfinite_or_negative_clock_fails_before_any_emit(self):
        for value in (math.nan, math.inf, -1.):
            with self.subTest(time=value):
                state, emitted, _ = make_state()
                with self.assertRaises(ValueError):
                    state.step(value, [obs()])
                self.assertEqual(emitted, [])


if __name__ == "__main__":
    unittest.main()
