"""Read-only TraCI mocks: immediate exits must not inherit a remote signal."""
import contextlib
import io
import os
import sys
import types
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

os.environ.setdefault("SUMO_HOME", "/tmp")
sys.modules.setdefault("traci", types.SimpleNamespace())
from collection._shared import TraCI_Python_Adjusted as cams
from collection._shared import TraCI_Python_Threshold_Sweep as sweep


def cfg():
    return cams.CamsConfig("fake.sumocfg", "fake.net.xml", "sumo", "events.csv", "cycles.csv",
                           "slim.csv", 80., .1, 2., 2., 85., "training.csv", 2., {"training", "cycles"},
                           immediate_roadpair_labels=True)


class ImmediateRoadPairTests(unittest.TestCase):
    def setUp(self):
        self.xml = '<net>' + ''.join(
            f'<edge id="{e}"><lane id="{e}_0" index="0" length="100" speed="10"/>'
            f'<lane id="{e}_1" index="1" length="100" speed="10" allow="bus"/></edge>'
            for e in "abc") + (
            '<connection from="a" to="b" fromLane="0" toLane="0" dir="s"/>'
            '<connection from="a" to="b" fromLane="1" toLane="1" dir="s"/>'
            '<connection from="b" to="c" fromLane="0" toLane="0" dir="r"/>'
            '</net>')
        self.net = self.parse_xml(self.xml)

    def parse_xml(self, text):
        with mock.patch.object(cams.ET, "parse", return_value=ET.ElementTree(ET.fromstring(text))):
            return cams.parse_network_info("fixture.net.xml")

    def vehicle(self, *, edge="a", index=0, cls="passenger", lane=None, next_tls=None):
        return types.SimpleNamespace(
            getRoadID=mock.Mock(return_value=edge), getRouteIndex=mock.Mock(return_value=index),
            getRoute=mock.Mock(return_value=("a", "b", "c")), getRouteID=mock.Mock(return_value="r"),
            getLaneID=mock.Mock(return_value=lane or edge + "_0"), getVehicleClass=mock.Mock(return_value=cls),
            getNextTLS=mock.Mock(return_value=next_tls if next_tls is not None else [("remote", 7, 250., "r")]),
            getSpeed=mock.Mock(return_value=5.), getLength=mock.Mock(return_value=5.), getMinGap=mock.Mock(return_value=2.5),
            getSpeedFactor=mock.Mock(return_value=1.), getIDList=mock.Mock(return_value=["v"]),
        )

    def snapshot(self, vehicle, immediate=True):
        with mock.patch.object(cams, "traci", types.SimpleNamespace(vehicle=vehicle)), mock.patch.object(
                cams, "get_downstream_storage", return_value=(0., 100.)):
            return cams.get_vehicle_snapshot("v", self.net, immediate_roadpair=immediate)

    def signalize(self):
        self.net = self.parse_xml(self.xml.replace('from="a" to="b" fromLane="0" toLane="0"',
                            'from="a" to="b" fromLane="0" toLane="0" tl="near" linkIndex="0"'))

    def test_immediate_unsignalized_ignores_remote_red(self):
        vehicle = self.vehicle()
        snapshot = self.snapshot(vehicle)
        self.assertEqual((snapshot.tls_id, snapshot.link_index, snapshot.signal_state), ("", -1, "G"))
        self.assertEqual(snapshot.exit_signal_status, "unsignalized_connection")
        self.assertEqual(snapshot.legal_exit_lanes, ("a_0",))
        self.assertEqual(snapshot.tls_distance, float("inf"))
        vehicle.getNextTLS.assert_not_called()

    def test_unsignalized_without_any_upcoming_tls_still_has_road_record(self):
        self.assertIsNotNone(self.snapshot(self.vehicle(next_tls=[])))
        self.assertIsNone(self.snapshot(self.vehicle(next_tls=[]), immediate=False))

    def test_legacy_default_retains_remote_tls_semantics(self):
        snapshot = self.snapshot(self.vehicle(), immediate=False)
        self.assertEqual((snapshot.tls_id, snapshot.link_index, snapshot.signal_state), ("remote", 7, "r"))
        self.assertEqual(snapshot.exit_signal_status, "legacy_next_tls")

    def test_immediate_controlled_matches_legacy_when_tls_is_correct(self):
        self.signalize()
        old = self.snapshot(self.vehicle(next_tls=[("near", 0, 10., "G")]), immediate=False)
        new = self.snapshot(self.vehicle(next_tls=[("near", 0, 10., "G")]))
        for key in ("tls_id", "link_index", "tls_distance", "signal_state", "from_edge", "to_edge", "speed", "lane_id"):
            self.assertEqual(getattr(old, key), getattr(new, key))
        self.assertEqual(new.exit_signal_status, "controlled_immediate")

    def test_controlled_missing_or_remote_or_wrong_link_fails_closed(self):
        self.signalize()
        for tls in ([], [("remote", 0, 20., "G")], [("near", 1, 10., "G")]):
            with self.subTest(tls=tls), self.assertRaises(ValueError):
                self.snapshot(self.vehicle(next_tls=tls))

    def test_permissions_use_runtime_class_not_hardcoded_passenger(self):
        bus = self.snapshot(self.vehicle(cls="bus", lane="a_1"))
        self.assertEqual(bus.legal_exit_lanes, ("a_1",))
        passenger = self.snapshot(self.vehicle(cls="passenger", lane="a_1"))
        self.assertEqual(passenger.legal_exit_lanes, ("a_0",))
        self.assertEqual(cams.select_movement_from_lane(passenger, self.net), "a_0")

    def test_connection_target_and_via_permissions_are_all_enforced(self):
        connection = self.net.road_pair_connections[("a", "b")][0]
        for key in ("connection_permissions", "from_permissions", "to_permissions", "via_permissions"):
            with self.subTest(permission=key):
                old = connection[key]
                connection[key] = {"disallow": "passenger"}
                with self.assertRaisesRegex(ValueError, "no legal immediate"):
                    self.snapshot(self.vehicle())
                connection[key] = old
        connection["via_permissions"] = None
        with self.assertRaisesRegex(ValueError, "no legal immediate"):
            self.snapshot(self.vehicle())

    def test_allow_disallow_intersection_and_ignoring_match_cpp(self):
        self.assertFalse(cams._class_allowed({"allow": "all", "disallow": "passenger"}, "passenger"))
        self.assertFalse(cams._class_allowed({"allow": "passenger", "disallow": "all"}, "passenger"))
        self.assertTrue(cams._class_allowed({"allow": "bus passenger", "disallow": "taxi"}, "passenger"))
        self.assertFalse(cams._class_allowed({"allow": "bus", "disallow": ""}, "passenger"))
        self.assertTrue(cams._class_allowed({"allow": "", "disallow": "all"}, "ignoring"))
        self.assertFalse(cams._class_allowed(None, "ignoring"))

    def internal_xml(self):
        return self.xml.replace('<connection from="a" to="b" fromLane="0" toLane="0" dir="s"/>',
            '<connection from="a" to="b" fromLane="0" toLane="0" dir="s" via=":x0_0"/>').replace('</net>',
            '<edge id=":x0" function="internal"><lane id=":x0_0" index="0" length="3" speed="10"/></edge>'
            '<edge id=":x1" function="internal"><lane id=":x1_0" index="0" length="4" speed="10"/></edge>'
            '<connection from=":x0" to="b" fromLane="0" toLane="0" via=":x1_0"/>'
            '<connection from=":x1" to="b" fromLane="0" toLane="0"/></net>')

    def test_complete_internal_chain_checks_later_lane_and_connection(self):
        xml = self.internal_xml()
        self.net = self.parse_xml(xml)
        self.assertEqual(len(self.net.road_pair_connections[("a", "b")][0]["internal_path_permissions"]), 4)
        self.assertIsNotNone(self.snapshot(self.vehicle()))
        for old, new in (
                ('id=":x1_0" index="0"', 'id=":x1_0" index="0" disallow="passenger"'),
                ('from=":x1" to="b"', 'from=":x1" disallow="passenger" to="b"')):
            with self.subTest(restriction=old):
                self.net = self.parse_xml(xml.replace(old, new))
                with self.assertRaisesRegex(ValueError, "no legal immediate"):
                    self.snapshot(self.vehicle())
                self.assertIsNotNone(self.snapshot(self.vehicle(cls="ignoring")))

    def test_incomplete_branching_cyclic_and_wrong_target_via_chain_fail_closed(self):
        xml = self.internal_xml()
        variants = (
            xml.replace('via=":x1_0"', 'via=":missing_0"'),
            xml.replace('<connection from=":x1" to="b" fromLane="0" toLane="0"/>', ''),
            xml.replace('via=":x1_0"', 'via=":x0_0"'),
            xml.replace('from=":x1" to="b"', 'from=":x1" to="c"'),
            xml.replace('</net>', '<connection from=":x1" to="b" fromLane="0" toLane="0"/></net>'),
            xml.replace('</net>', '<edge id=":duplicate" function="internal"><lane id=":x1_0" index="0"/></edge></net>'),
        )
        for index, variant in enumerate(variants):
            with self.subTest(malformed=index):
                self.net = self.parse_xml(variant)
                with self.assertRaisesRegex(ValueError, "no legal immediate"):
                    self.snapshot(self.vehicle())
                # ignoring bypasses permissions, not missing topology. Keep
                # this vehicle on a_0 so the bus-only alternative is not used.
                self.net.road_pair_connections[("a", "b")] = self.net.road_pair_connections[("a", "b")][:1]
                with self.assertRaisesRegex(ValueError, "no legal immediate"):
                    self.snapshot(self.vehicle(cls="ignoring"))

    def test_actual_compatible_lane_controls_signal_resolution(self):
        self.signalize()
        # One pair can have controlled and uncontrolled alternatives; actual
        # compatible lane remains sufficient to disambiguate the live exit.
        snapshot = self.snapshot(self.vehicle(cls="bus", lane="a_1"))
        self.assertEqual(snapshot.exit_signal_status, "unsignalized_connection")
        with self.assertRaisesRegex(ValueError, "ambiguous immediate"):
            self.snapshot(self.vehicle(cls="bus", lane="a_2"))

    def test_cli_is_opt_in_and_legacy_default_remains_false(self):
        with mock.patch.object(sys, "argv", ["collector"]):
            self.assertFalse(cams.parse_args().immediate_roadpair_labels)
        with mock.patch.object(sys, "argv", ["collector", "--immediate-roadpair-labels"]):
            self.assertTrue(cams.parse_args().immediate_roadpair_labels)

    def test_unknown_class_and_missing_pair_fail_closed(self):
        with self.assertRaisesRegex(ValueError, "unknown runtime"):
            self.snapshot(self.vehicle(cls=""))
        self.net.road_pair_connections.clear()
        with self.assertRaisesRegex(ValueError, "no legal immediate"):
            self.snapshot(self.vehicle())

    def test_final_road_does_not_require_next_tls_or_vehicle_class(self):
        vehicle = self.vehicle(edge="c", index=2)
        snapshot = self.snapshot(vehicle)
        self.assertTrue(snapshot.is_final_road)
        self.assertEqual(snapshot.exit_signal_status, "no_outgoing_route")
        vehicle.getNextTLS.assert_not_called()
        vehicle.getVehicleClass.assert_not_called()

    def test_no_fake_signal_queue_or_cycle_on_unsignalized_road(self):
        snapshot = self.snapshot(self.vehicle())
        snapshot.speed = 0.
        snapshot.tls_distance = 0.
        config = cfg()
        self.assertEqual(cams.compute_queue_counts({"v": snapshot}, config), {})
        tls = mock.Mock(side_effect=AssertionError("unsignalized road queried TLS"))
        with mock.patch.object(cams, "traci", types.SimpleNamespace(trafficlight=types.SimpleNamespace(getRedYellowGreenState=tls))):
            cams.update_cycle_states({"v": snapshot}, {}, {}, {}, {}, {}, [], config, 0.)
        tls.assert_not_called()

    def test_endpoint_speed_integration_is_unchanged(self):
        snapshot = self.snapshot(self.vehicle())
        with mock.patch.object(cams, "read_vehicle_speed_factor", return_value=1.), mock.patch.object(cams, "sample_training_features"):
            event = cams.make_vehicle_event(snapshot, 0., self.net, {}, cfg())
        for t, speed in enumerate((0., .1, .5, 1., 1.5, 2., 3., 5.), 1):
            snapshot.speed = speed
            sweep.accumulate_waiting_time(event, snapshot, cfg(), float(t))
        completed = []
        cams.finalize_event(event, 9., completed)
        labels = sweep._event_extra_values(event)
        self.assertEqual([labels[c] for c in sweep.candidate_label_columns(sweep.DEFAULT_LABEL_SPEED_THRESHOLDS)], [6., 5., 4., 3., 2., 1.])
        self.assertEqual(labels['speed_bin_conservation_residual'], 0.)
        self.assertEqual(sum(event.threshold_sweep_speed_bins), 8.)

    def run_mock_main(self, frames, route=("a", "b", "c"), config=None):
        pos = [-1]
        def current():
            return frames[pos[0]]
        vehicle = self.vehicle()
        vehicle.getRoadID = lambda vid: current()[0]
        vehicle.getLaneID = lambda vid: current()[0] + "_0"
        vehicle.getRouteIndex = lambda vid: current()[1]
        vehicle.getRoute = lambda vid: route
        vehicle.getIDList = lambda: ["v"] if current()[0] else []
        remote = vehicle.getNextTLS
        step = lambda: pos.__setitem__(0, pos[0] + 1)
        api = types.SimpleNamespace(vehicle=vehicle, simulationStep=step, start=mock.Mock(), close=mock.Mock(),
            simulation=types.SimpleNamespace(getMinExpectedNumber=lambda: int(pos[0] < len(frames) - 1),
                getTime=lambda: float(pos[0] + 1), getArrivedIDList=lambda: [] if current()[0] else ["v"], getStartingTeleportIDList=lambda: []),
            trafficlight=types.SimpleNamespace(getRedYellowGreenState=mock.Mock(side_effect=AssertionError("fake TLS cycle"))))
        completed, cycles = [], []
        flow = lambda net: ({current()[0]: 1}, {current()[0] + "_0": 1}, {current()[0] + "_0": 7.5})
        with mock.patch.object(cams, "traci", api), mock.patch.object(cams, "parse_args", return_value=config or cfg()), \
                mock.patch.object(cams, "parse_network_info", return_value=self.net), \
                mock.patch.object(cams, "get_downstream_storage", return_value=(0., 100.)), \
                mock.patch.object(cams, "compute_flow_snapshot", side_effect=flow), \
                mock.patch.object(cams, "accumulate_waiting_time", sweep.accumulate_waiting_time), \
                mock.patch.object(cams, "write_training_rows", side_effect=lambda path, rows: completed.extend(rows)), \
                mock.patch.object(cams, "write_cycle_summaries", side_effect=lambda path, rows: cycles.extend(rows)), \
                contextlib.redirect_stdout(io.StringIO()):
            cams.main()
        api.start.assert_called_once()
        api.close.assert_called_once()
        return completed, cycles, remote

    def test_mock_main_collects_unsignalized_middle_and_final_without_tls_cycles(self):
        frames = [("a", 0), ("a", 0), ("b", 1), ("b", 1), ("c", 2), ("c", 2), (None, 2)]
        completed, cycles, remote = self.run_mock_main(frames)
        self.assertEqual([e.route_index for e in completed], [0, 1, 2])
        self.assertEqual([e.driving_time for e in completed], [1., 1., 1.])
        self.assertEqual([e.exit_signal_status for e in completed], ["unsignalized_connection", "unsignalized_connection", "no_outgoing_route"])
        self.assertEqual(cycles, [])
        remote.assert_not_called()
        for event in completed:
            self.assertEqual([sweep._event_extra_values(event)[c] for c in sweep.candidate_label_columns(sweep.DEFAULT_LABEL_SPEED_THRESHOLDS)], [1.] * 6)

    def test_mock_main_consecutive_same_edge_visits_are_distinct(self):
        self.net = self.parse_xml(self.xml.replace('</net>', '<connection from="a" to="a" fromLane="0" toLane="0" dir="s"/></net>'))
        frames = [("a", 0), ("a", 0), ("a", 1), ("a", 1), ("b", 2), ("b", 2), (None, 2)]
        completed, cycles, remote = self.run_mock_main(frames, route=("a", "a", "b"))
        self.assertEqual([e.route_index for e in completed], [0, 1, 2])
        self.assertEqual([e.from_edge for e in completed], ["a", "a", "b"])
        self.assertEqual([e.driving_time for e in completed], [1., 1., 1.])
        self.assertEqual([e.edge_enter_time for e in completed], [1., 3., 5.])
        self.assertEqual([e.pass_stopline_time for e in completed], [3., 5., None])
        self.assertEqual(cycles, [])
        remote.assert_not_called()

    def test_mock_main_same_edge_skipped_occurrence_fails_closed(self):
        self.net = self.parse_xml(self.xml.replace('</net>', '<connection from="a" to="a" fromLane="0" toLane="0" dir="s"/></net>'))
        frames = [("a", 0), ("a", 0), ("a", 2), ("b", 3), (None, 3)]
        with self.assertRaisesRegex(ValueError, "unobserved route occurrence"):
            self.run_mock_main(frames, route=("a", "a", "a", "b"))

    def test_legacy_same_edge_lifecycle_is_not_silently_changed(self):
        self.net = self.parse_xml(self.xml.replace('</net>', '<connection from="a" to="a" fromLane="0" toLane="0" dir="s"/></net>'))
        frames = [("a", 0), ("a", 0), ("a", 1), ("a", 1), ("b", 2), ("b", 2), (None, 2)]
        config = cfg()
        config.immediate_roadpair_labels = False
        original_snapshot = cams.get_vehicle_snapshot
        # Supply the same snapshots to isolate lifecycle behavior from legacy
        # no-TLS snapshot filtering; legacy still calls two positional args.
        with mock.patch.object(cams, "get_vehicle_snapshot", side_effect=lambda vehicle, net:
                original_snapshot(vehicle, net, immediate_roadpair=True)):
            completed, _, _ = self.run_mock_main(frames, route=("a", "a", "b"), config=config)
        self.assertEqual([e.route_index for e in completed], [1, 2])


if __name__ == '__main__':
    unittest.main()
