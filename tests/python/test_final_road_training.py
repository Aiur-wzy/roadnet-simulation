"""Regression tests for terminal-road labels without SUMO or AutoGluon."""

import contextlib
import csv
import io
import os
from pathlib import Path
import sys
import types
import unittest
from unittest import mock
import uuid


# Match test_missed_green.py: all TraCI operations used below are local mocks.
os.environ.setdefault("SUMO_HOME", "/tmp")
sys.modules.setdefault("traci", types.SimpleNamespace())
import collection._shared.TraCI_Python_Adjusted as cams
import collection._shared.TraCI_Python_Threshold_Sweep as sweep
from legacy.collection import traci_vehicle_features as vehicle_features


def config(outputs=()):
    return cams.CamsConfig(
        "fake.sumocfg", "fake.net.xml", "sumo", "events.csv", "cycles.csv",
        "slim.csv", 80.0, 0.1, 2.0, 2.0, 85.0, "full.csv", 2.0,
        set(outputs),
    )


def network_info():
    return cams.NetworkInfo(
        edge_lanes={"a": ["a_0", "a_1"], "b": ["b_0", "b_1"]},
        lane_lengths={"a_0": 100.0, "a_1": 110.0, "b_0": 120.0, "b_1": 130.0},
        connection_dirs={("a", "b"): "l", ("b", "a"): "s"},
        edge_speed_limits={"a": 13.89, "b": 13.89},
        movement_from_lanes={("tls", 0, "a", "b"): ("a_1",)},
    )


def snapshot(final=True, speed=5.0, route=("a", "b"), route_index=None):
    if route_index is None:
        route_index = len(route) - 1 if final else 0
    edge = route[route_index]
    return cams.VehicleSnapshot(
        vehicle_id="v", route_id="r", route_edges=route, route_index=route_index,
        from_edge=edge, to_edge="" if final else route[route_index + 1],
        lane_id=edge + "_0", tls_id="" if final else "tls",
        link_index=-1 if final else 0, signal_state="" if final else "G",
        tls_distance=float("inf") if final else 10.0, speed=speed,
        downstream_occupancy=0.0, available_storage_meter=0.0 if final else 120.0,
        turn_direction="unknown" if final else "l", turn_type=0 if final else 1,
        vehicle_length=5.0, vehicle_min_gap=2.5, is_final_road=final,
    )


def event(final=True, completed=False, label=2.0):
    s = snapshot(final=final)
    e = cams.VehicleMovementEvent(
        vehicle_id=s.vehicle_id, route_id=s.route_id, route_edges=s.route_edges,
        route_index=s.route_index, from_edge=s.from_edge, to_edge=s.to_edge,
        lane_id=s.lane_id, tls_id=s.tls_id, link_index=s.link_index,
        edge_enter_time=1.0, turn_direction=s.turn_direction,
        vehicle_length=5.0, vehicle_min_gap=2.5, last_update_time=1.0,
        road_length=120.0 if final else 100.0, lane_index=0, lane_num=2,
        speed_limit=13.89, road_flow=3, lane_flow=2, turn_type=s.turn_type,
        sampled_time=1.0, driving_time=label, previous_has_waiting=1,
        previous_waiting_duration=3.5, is_final_road=final,
        speed_factor=1.0123456789012345,
    )
    for name, value in {
        "vehicle_max_speed": 13.89, "vehicle_accel": 2.6,
        "vehicle_decel": 4.5, "vehicle_sigma": 0.5,
    }.items():
        setattr(e, name, value)
    if completed:
        cams.finalize_event(e, 4.0, [], arrived=final)
    return e


def fake_vehicle_api(route=("a", "b"), route_index=1, current_edge=None):
    edge = route[route_index] if current_edge is None else current_edge
    return types.SimpleNamespace(
        getRoadID=mock.Mock(return_value=edge),
        getRoute=mock.Mock(return_value=route),
        getRouteIndex=mock.Mock(return_value=route_index),
        getNextTLS=mock.Mock(return_value=[("tls", 0, 10.0, "G")]),
        getRouteID=mock.Mock(return_value="r"),
        getLaneID=mock.Mock(return_value=edge + "_0"),
        getSpeed=mock.Mock(return_value=5.0),
        getLength=mock.Mock(return_value=5.0),
        getMinGap=mock.Mock(return_value=2.5),
        getSpeedFactor=mock.Mock(return_value=1.0123456789012345),
        getIDList=mock.Mock(return_value=["v"]),
    )


class FinalRoadSnapshotTests(unittest.TestCase):
    def read_snapshot(self, route, route_index, current_edge=None):
        api = fake_vehicle_api(route, route_index, current_edge)
        traci = types.SimpleNamespace(vehicle=api)
        with mock.patch.object(cams, "traci", traci), mock.patch.object(
            cams, "get_downstream_storage", return_value=(0.0, 120.0)
        ) as storage:
            result = cams.get_vehicle_snapshot("v", network_info())
        return result, api, storage

    def test_single_road_requires_no_next_tls_or_downstream(self):
        result, api, storage = self.read_snapshot(("a",), 0)
        self.assertTrue(result.is_final_road)
        self.assertEqual(result.route_index, 0)
        self.assertEqual((result.to_edge, result.tls_id, result.link_index), ("", "", -1))
        self.assertEqual(result.turn_type, 0)
        self.assertEqual(result.tls_distance, float("inf"))
        api.getNextTLS.assert_not_called()
        storage.assert_not_called()

    def test_last_occurrence_not_first_matching_edge_is_terminal(self):
        first, first_api, _ = self.read_snapshot(("a", "b", "a"), 0)
        final, final_api, _ = self.read_snapshot(("a", "b", "a"), 2)
        self.assertFalse(first.is_final_road)
        self.assertEqual(first.to_edge, "b")
        first_api.getNextTLS.assert_called_once_with("v")
        self.assertTrue(final.is_final_road)
        self.assertEqual(final.route_index, 2)
        final_api.getNextTLS.assert_not_called()

    def test_missing_tls_is_allowed_only_on_final_road(self):
        for index, expected in ((0, False), (1, True)):
            api = fake_vehicle_api(route_index=index)
            api.getNextTLS.return_value = []
            with mock.patch.object(cams, "traci", types.SimpleNamespace(vehicle=api)):
                result = cams.get_vehicle_snapshot("v", network_info())
            self.assertEqual(result is not None, expected)

    def test_internal_edge_and_route_mismatch_are_not_terminal_samples(self):
        for current_edge in (":internal", "other", ""):
            with self.subTest(edge=current_edge):
                result, _, _ = self.read_snapshot(("a", "b"), 1, current_edge)
                self.assertIsNone(result)

    def test_final_entry_features_use_actual_lane_and_inherit_previous_wait(self):
        s = snapshot()
        waiting = {"v": (1, 7.25)}
        with mock.patch.object(
            cams, "compute_flow_snapshot",
            return_value=({"b": 4}, {"b_0": 3, "b_1": 1}, {"b_0": 22.5, "b_1": 7.5}),
        ), mock.patch.object(
            cams, "read_vehicle_speed_factor", return_value=1.0123456789012345,
        ):
            e = cams.make_vehicle_event(s, 10.0, network_info(), waiting, config())
        self.assertTrue(e.is_final_road)
        self.assertEqual(e.lane_index, 0)
        self.assertEqual(e.road_length, 120.0)
        self.assertEqual(e.road_flow, 4)
        self.assertEqual(e.lane_flow, 3)  # The sampled vehicle is already in this count.
        self.assertEqual(e.lane_occupied_length, 22.5)
        self.assertEqual((e.previous_has_waiting, e.previous_waiting_duration), (1, 7.25))
        self.assertEqual(e.turn_type, 0)
        self.assertEqual(e.incoming_turn_type, 1)
        self.assertNotIn("v", waiting)

    def test_flow_snapshot_includes_current_vehicle_exactly_once(self):
        api = fake_vehicle_api()
        with mock.patch.object(cams, "traci", types.SimpleNamespace(vehicle=api)):
            road, lane, occupied = cams.compute_flow_snapshot(network_info())
            e = cams.make_vehicle_event(snapshot(), 1.0, network_info(), {}, config())
        self.assertEqual((road, lane, occupied), ({"b": 1}, {"b_0": 1}, {"b_0": 7.5}))
        self.assertEqual((e.road_flow, e.lane_flow, e.previous_has_waiting), (1, 1, 0))

    def test_vehicle_feature_hook_runs_for_terminal_event(self):
        params = {"getLength": 8.0, "getMinGap": 2.0, "getMaxSpeed": 11.0,
                  "getAccel": 1.7, "getDecel": 3.0, "getImperfection": 0.2,
                  "getSpeedFactor": 0.8765432109876543}
        api = types.SimpleNamespace(**{
            getter: mock.Mock(return_value=value) for getter, value in params.items()
        })
        with mock.patch.object(cams, "traci", types.SimpleNamespace(vehicle=api)), mock.patch.object(
            cams, "compute_flow_snapshot", return_value=({"b": 1}, {"b_0": 1}, {"b_0": 10.0})
        ):
            e = vehicle_features.make_vehicle_event(snapshot(), 1.0, network_info(), {}, config())
        self.assertTrue(e.is_final_road)
        self.assertEqual(
            [getattr(e, name) for name in vehicle_features.VEHICLE_FEATURES],
            [8.0, 2.0, 11.0, 1.7, 3.0, 0.2],
        )


class FinalRoadAccountingTests(unittest.TestCase):
    def test_base_strict_speed_boundaries_and_terminal_wait(self):
        e = event(label=0.0)
        for time, speed in enumerate((0.0, 0.1, 0.1001, 2.0, 2.0001), start=2):
            cams.accumulate_waiting_time(e, snapshot(speed=speed), config(), float(time))
        self.assertEqual((e.terminal_wait_time, e.low_speed_time, e.driving_time), (2.0, 2.0, 1.0))
        self.assertEqual(e.red_wait_time + e.green_queue_wait_time + e.downstream_block_wait_time, 0.0)

    def test_sweep_boundaries_are_strictly_greater_than_each_cutoff(self):
        e = event(label=0.0)
        thresholds = sweep.DEFAULT_LABEL_SPEED_THRESHOLDS
        speeds = (0.0,) + thresholds + (4.0,)
        with mock.patch.object(sweep, "_label_speed_thresholds", thresholds):
            for time, speed in enumerate(speeds, start=2):
                sweep.accumulate_waiting_time(e, snapshot(speed=speed), config(), float(time))
            extra = sweep._event_extra_values(e)
        for cutoff in thresholds:
            name = "travel_time_label_gt_" + sweep.threshold_token(cutoff)
            self.assertEqual(extra[name], sum(speed > cutoff for speed in speeds), name)
        self.assertEqual(extra["speed_bin_time_total"], float(len(speeds)))
        self.assertEqual(extra["speed_bin_conservation_residual"], 0.0)

    def test_first_observation_and_removal_step_do_not_add_time(self):
        for accumulate in (cams.accumulate_waiting_time, sweep.accumulate_waiting_time):
            with self.subTest(accumulate=accumulate.__module__):
                e = event(label=0.0)
                accumulate(e, snapshot(speed=10.0), config(), 1.0)
                self.assertEqual(e.driving_time, 0.0)
                accumulate(e, snapshot(speed=10.0), config(), 2.0)
                completed = []
                cams.finalize_event(e, 3.0, completed, arrived=True)
                self.assertEqual(e.driving_time, 1.0)
                self.assertEqual(e.last_update_time, 2.0)
                accumulate(e, snapshot(speed=10.0), config(), 4.0)
                self.assertEqual(e.driving_time, 1.0)
                self.assertEqual(e.destination_arrival_time, 3.0)
                self.assertIsNone(e.pass_stopline_time)
                self.assertIsNone(e.downstream_enter_time)

    def test_first_observation_without_previous_time_also_adds_zero(self):
        e = event(label=0.0)
        e.last_update_time = None
        sweep.accumulate_waiting_time(e, snapshot(), config(), 100.0)
        self.assertEqual(e.driving_time + e.low_speed_time + e.terminal_wait_time, 0.0)
        self.assertEqual(sweep._event_extra_values(e)["speed_bin_time_total"], 0.0)

    def test_terminal_finalize_requires_confirmed_arrival_and_is_not_repeatable(self):
        e, completed = event(), []
        with self.assertRaisesRegex(ValueError, "confirmed SUMO arrival"):
            cams.finalize_event(e, 4.0, completed)
        self.assertEqual(completed, [])
        self.assertIsNone(e.destination_arrival_time)
        cams.finalize_event(e, 4.0, completed, arrived=True)
        self.assertEqual(completed, [e])
        with self.assertRaisesRegex(ValueError, "already completed"):
            cams.finalize_event(e, 5.0, completed, arrived=True)
        self.assertEqual(completed, [e])
        self.assertEqual(e.destination_arrival_time, 4.0)

    def test_nonfinal_stopline_still_ends_accumulation(self):
        e = event(final=False, label=0.0)
        sweep.accumulate_waiting_time(e, snapshot(final=False), config(), 2.0)
        e.pass_stopline_time = 3.0
        sweep.accumulate_waiting_time(e, snapshot(final=False), config(), 3.0)
        self.assertEqual(e.driving_time, 1.0)
        self.assertEqual(sweep._event_extra_values(e)["speed_bin_time_total"], 1.0)


class FinalRoadWriterTests(unittest.TestCase):
    def setUp(self):
        # Inherit the working directory's permissions: Windows sandbox tokens
        # cannot reopen the owner-only directory made by TemporaryDirectory.
        self.directory = Path.cwd() / ("final-road-training-test-" + uuid.uuid4().hex)
        self.directory.mkdir()
        self.addCleanup(self.cleanup_outputs)

    def cleanup_outputs(self):
        # Only our flat, newly created test directory is eligible for cleanup.
        for path in self.directory.iterdir():
            path.unlink()
        self.directory.rmdir()

    def read_rows(self, path):
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            return list(reader), reader.fieldnames

    def test_all_writers_share_filter_and_do_not_fabricate_signal_fields(self):
        normal = event(final=False, completed=True)
        final = event(completed=True)
        final.terminal_wait_time = 1.5
        pending = event()
        zero_label = event(completed=True, label=0.0)
        bad_length = event(completed=True)
        bad_length.road_length = 0.0
        events = [normal, pending, final, zero_label, bad_length]
        self.assertEqual(list(cams.iter_valid_training_events(events)), [normal, final])
        self.assertEqual(list(sweep._valid_training_events(events)), [normal, final])
        self.assertEqual([item[0] for item in vehicle_features._valid_training_events(events)], [normal, final])
        for module in (cams, vehicle_features):
            with self.subTest(module=module.__name__):
                full = self.directory / (module.__name__ + "-full.csv")
                slim = self.directory / (module.__name__ + "-slim.csv")
                module.write_training_rows(str(full), events)
                count = module.write_lightweight_training_rows(str(slim), events)
                rows, fields = self.read_rows(full)
                slim_rows, slim_fields = self.read_rows(slim)
                self.assertEqual(count, 2)
                self.assertEqual(len(rows), 2)
                self.assertEqual(len(slim_rows), 2)
                self.assertEqual([row["roadID"] for row in rows], ["a", "b"])
                terminal = rows[1]
                self.assertEqual(terminal["is_final_road"], "1")
                self.assertEqual(terminal["route_index"], "1")
                self.assertEqual(terminal["road_exit_time"], "4.0")
                self.assertEqual(terminal["destination_arrival_time"], "4.0")
                self.assertEqual(terminal["terminal_wait_time"], "1.5")
                for name in ("movementID", "tls_id", "link_index", "pass_stopline_time",
                             "downstream_enter_time", "total_before_stopline_time"):
                    self.assertEqual(terminal[name], "", name)
                self.assertEqual(rows[0]["movementID"], "tls:0")
                self.assertEqual(rows[0]["pass_stopline_time"], "4.0")
                self.assertEqual(rows[0]["total_before_stopline_time"], "3.0")
                self.assertEqual(rows[0]["destination_arrival_time"], "")
                for index, row in enumerate(slim_rows):
                    self.assertEqual(row["travel_time_label"], rows[index]["travel_time_label"])
                base_columns = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
                extra = [] if module is cams else vehicle_features.VEHICLE_FEATURES
                self.assertEqual(slim_fields, base_columns + extra + ["speed_factor", "travel_time_label"])
                self.assertEqual(fields[-6:-1], ["route_index", "is_final_road", "road_exit_time",
                                             "terminal_wait_time", "destination_arrival_time"])
                self.assertEqual(fields[-1], "speed_factor")

    def test_sweep_appended_rows_match_base_row_order_with_pending_final(self):
        normal = event(final=False, completed=True)
        final = event(completed=True)
        normal.threshold_sweep_speed_bins = [0.0] * 6 + [2.0]
        final.threshold_sweep_speed_bins = [1.0] + [0.0] * 5 + [2.0]
        final.terminal_wait_time = 1.0
        events = [normal, event(), final]
        for writer in (sweep.write_lightweight_training_rows, sweep.write_training_rows):
            path = self.directory / (writer.__name__ + ".csv")
            with contextlib.redirect_stdout(io.StringIO()):
                writer(str(path), events)
            rows, _ = self.read_rows(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual([float(row["speed_bin_time_total"]) for row in rows], [2.0, 3.0])
            self.assertEqual([float(row["travel_time_label_gt_0p1"]) for row in rows], [2.0, 2.0])
            self.assertTrue(all(float(row["speed_bin_conservation_residual"]) == 0.0 for row in rows))

    def test_positive_candidate_survives_zero_default_label_filter(self):
        e = event(label=0.0)
        custom = config()
        custom.stop_speed_threshold = 1.0
        for time in (2.0, 3.0):
            sweep.accumulate_waiting_time(e, snapshot(speed=0.5), custom, time)
        cams.finalize_event(e, 4.0, [], arrived=True)
        self.assertEqual(e.driving_time + e.low_speed_time, 0.0)
        self.assertEqual(e.terminal_wait_time, 2.0)
        self.assertTrue(e.has_positive_candidate_label)
        self.assertEqual(list(cams.iter_valid_training_events([e])), [e])
        self.assertEqual([item[0] for item in vehicle_features._valid_training_events([e])], [e])
        for writer in (sweep.write_lightweight_training_rows, sweep.write_training_rows):
            path = self.directory / (writer.__name__ + "-positive-candidate.csv")
            with contextlib.redirect_stdout(io.StringIO()):
                writer(str(path), [e])
            rows, _ = self.read_rows(path)
            self.assertEqual(len(rows), 1)
            self.assertEqual(float(rows[0]["travel_time_label"]), 0.0)
            self.assertEqual(float(rows[0]["travel_time_label_gt_0p1"]), 2.0)
            self.assertEqual(float(rows[0]["travel_time_label_gt_0p5"]), 0.0)
            self.assertEqual(float(rows[0]["speed_bin_conservation_residual"]), 0.0)

    def test_signal_event_output_excludes_final_roads(self):
        path = self.directory / "events.csv"
        cams.write_vehicle_events(str(path), [event(completed=True), event(final=False, completed=True)], {})
        rows, _ = self.read_rows(path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["from_edge"], "a")

    def test_terminal_snapshots_do_not_create_queues_or_cycles(self):
        terminal = snapshot(speed=0.0)
        # Even a malformed finite distance must not create a signal queue.
        terminal.tls_distance = 0.0
        self.assertEqual(cams.compute_queue_counts({"v": terminal}, config()), {})
        light = mock.Mock(side_effect=AssertionError("final road queried a traffic light"))
        active, completed = {}, []
        with mock.patch.object(cams, "traci", types.SimpleNamespace(
            trafficlight=types.SimpleNamespace(getRedYellowGreenState=light)
        )):
            cams.update_cycle_states({"v": terminal}, {"v": event()}, {}, {}, {}, active,
                                     completed, config(), 2.0)
        self.assertEqual((active, completed), ({}, []))
        light.assert_not_called()


class FinalRoadMainLifecycleTests(unittest.TestCase):
    def run_lifecycle(self, frames):
        position = [-1]

        def frame():
            return frames[position[0]]

        def step():
            position[0] += 1

        simulation = types.SimpleNamespace(
            getMinExpectedNumber=lambda: int(position[0] < len(frames) - 1),
            getTime=lambda: float(position[0] + 1),
            getArrivedIDList=lambda: frame().get("arrived", ()),
            getStartingTeleportIDList=lambda: frame().get("teleport", ()),
        )
        vehicle = types.SimpleNamespace(
            getIDList=lambda: frame().get("ids", ("v",)),
            getRoadID=lambda _vehicle_id: frame().get("edge", "b"),
            getSpeedFactor=lambda _vehicle_id: 1.0123456789012345,
        )
        light = mock.Mock(side_effect=AssertionError("terminal road queried TLS"))
        traci = types.SimpleNamespace(
            simulation=simulation, vehicle=vehicle, simulationStep=step,
            trafficlight=types.SimpleNamespace(getRedYellowGreenState=light),
            start=mock.Mock(), close=mock.Mock(),
        )
        completed, cycles = [], []

        def write_training(_path, events):
            completed.extend(events)

        def get_snapshot(_vehicle_id, _network):
            return snapshot(speed=frame().get("speed", 5.0))

        with mock.patch.object(cams, "traci", traci), mock.patch.object(
            cams, "parse_args", return_value=config({"training", "cycles"})
        ), mock.patch.object(cams, "parse_network_info", return_value=network_info()), mock.patch.object(
            cams, "get_vehicle_snapshot", side_effect=get_snapshot
        ), mock.patch.object(
            cams, "compute_flow_snapshot", return_value=({"b": 1}, {"b_0": 1}, {"b_0": 7.5})
        ), mock.patch.object(cams, "write_training_rows", side_effect=write_training), mock.patch.object(
            cams, "write_cycle_summaries", side_effect=lambda _path, values: cycles.extend(values)
        ), contextlib.redirect_stdout(io.StringIO()):
            cams.main()
        traci.start.assert_called_once()
        traci.close.assert_called_once()
        light.assert_not_called()
        self.assertEqual(cycles, [])
        return completed

    def test_normal_arrival_exports_exactly_one_final_without_removal_extrapolation(self):
        completed = self.run_lifecycle([{}, {}, {"ids": (), "arrived": ("v",)}])
        self.assertEqual(len(completed), 1)
        e = completed[0]
        self.assertTrue(e.is_final_road)
        self.assertEqual(e.driving_time, 1.0)
        self.assertEqual(e.destination_arrival_time, 3.0)
        self.assertIsNone(e.pass_stopline_time)

    def test_teleport_then_reappearance_and_arrival_remain_excluded(self):
        completed = self.run_lifecycle([
            {}, {"ids": (), "teleport": ("v",)}, {}, {"ids": (), "arrived": ("v",)},
        ])
        self.assertEqual(completed, [])

    def test_simultaneous_teleport_and_arrival_is_not_valid_completion(self):
        completed = self.run_lifecycle([{}, {"ids": (), "arrived": ("v",), "teleport": ("v",)}])
        self.assertEqual(completed, [])

    def test_manual_removal_without_arrival_is_excluded(self):
        self.assertEqual(self.run_lifecycle([{}, {}, {"ids": ()}]), [])

    def test_remaining_unfinished_final_at_loop_end_is_excluded(self):
        self.assertEqual(self.run_lifecycle([{}, {}]), [])

    def test_unexpected_road_change_does_not_finalize_terminal(self):
        completed = self.run_lifecycle([{}, {"edge": "other"}, {"ids": (), "arrived": ("v",)}])
        self.assertEqual(completed, [])


if __name__ == "__main__":
    unittest.main()
