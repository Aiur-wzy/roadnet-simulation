"""Turn metadata regression tests: no SUMO, pandas, or model is required."""
import csv
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest import mock
from roadnet_paths import ROOT
import uuid

from features.turn_context import (
    TURN_CONTEXT_COLUMNS, TURN_CONTEXT_SCHEMA, TURN_TYPE_MAP,
    encode_turn_direction, event_turn_metadata, resolve_turn_context,
)


class PureTurnContextTests(unittest.TestCase):
    def test_canonical_codes_and_explicit_legacy_aliases(self):
        expected = {"l": 1, "L": 1, "s": 2, "S": 2, "r": 3, "R": 3, "t": 4, "U": 4}
        self.assertEqual(TURN_TYPE_MAP, expected)
        for raw, code in expected.items():
            self.assertEqual(encode_turn_direction(raw), code)
        for raw in ("unknown", "end", "", "u", "T", "left", None, 3, []):
            self.assertEqual(encode_turn_direction(raw), 0)

    def test_first_middle_final_and_single_road(self):
        route = ("a", "b", "c")
        network = {("a", "b"): "l", ("b", "c"): "r"}
        expected = [(0, 1, False, True), (1, 3, True, True), (3, 0, True, False)]
        for index, values in enumerate(expected):
            row = resolve_turn_context(route, index, network)
            self.assertEqual(tuple(row[key] for key in ("incoming_turn_type", "outgoing_turn_type",
                                                       "incoming_turn_present", "outgoing_turn_present")), values)
            self.assertEqual(row["turn_context_schema"], TURN_CONTEXT_SCHEMA)
            self.assertEqual(row["current_edge"], route[index])
        single = resolve_turn_context(("a",), 0, network)
        self.assertEqual((single["incoming_turn_type"], single["outgoing_turn_type"]), (0, 0))
        self.assertFalse(single["incoming_turn_present"])
        self.assertFalse(single["outgoing_turn_present"])
        self.assertEqual((single["incoming_from_edge"], single["outgoing_to_edge"]), ("", ""))

    def test_repeated_edge_uses_occurrence_index_not_first_match(self):
        route = ("a", "b", "a", "c")
        network = {("a", "b"): "l", ("b", "a"): "t", ("a", "c"): "r"}
        first = resolve_turn_context(route, 0, network)
        revisited = resolve_turn_context(route, 2, network)
        self.assertEqual((first["incoming_turn_type"], first["outgoing_turn_type"]), (0, 1))
        self.assertEqual((revisited["incoming_turn_type"], revisited["outgoing_turn_type"]), (4, 3))

    def test_missing_and_unknown_movements_are_present_not_boundary(self):
        for network in ({}, {("a", "b"): "unexpected"}):
            row = resolve_turn_context(("a", "b"), 0, network)
            self.assertEqual(row["outgoing_turn_type"], 0)
            self.assertTrue(row["outgoing_turn_present"])
            self.assertFalse(row["incoming_turn_present"])
        row = resolve_turn_context(("a", "b"), 0, {("a", "b"): "unexpected"})
        self.assertEqual(row["outgoing_turn_direction"], "unexpected")

    def test_slight_and_uturn_raw_codes_are_not_lost(self):
        for raw, code in (("L", 1), ("R", 3), ("t", 4), ("U", 4), ("S", 2)):
            row = resolve_turn_context(("a", "b"), 0, {("a", "b"): raw})
            self.assertEqual((row["outgoing_turn_direction"], row["outgoing_turn_type"]), (raw, code))

    def test_invalid_occurrence_or_preencoded_mapping_rejected(self):
        for route, index in (((), 0), ("abc", 0), (("a",), -1), (("a",), 1),
                             (("a",), True), (("a",), .5), (("a", ""), 0)):
            with self.assertRaises(ValueError):
                resolve_turn_context(route, index, {})
        with self.assertRaisesRegex(ValueError, "raw direction strings"):
            resolve_turn_context(("a", "b"), 0, {("a", "b"): 1})

    def test_cpp_route_occurrence_fixture_matches_python(self):
        root = ROOT
        candidates = [root / directory / name for directory in ("", "build_t04", "build")
                      for name in ("TurnContextTest.exe", "TurnContextTest")]
        if os.environ.get("ROADNET_TEST_BUILD_DIR"):
            candidates = [Path(os.environ["ROADNET_TEST_BUILD_DIR"]) / name
                          for name in ("TurnContextTest.exe", "TurnContextTest")] + candidates
        executable = next((path for path in candidates if path.is_file()), None)
        if executable is None:
            self.skipTest("optional C++ turn-context test executable has not been built")
        completed = subprocess.run([str(executable), "--fixture"], check=True, capture_output=True, text=True)
        fixture = json.loads(completed.stdout)
        self.assertEqual(fixture["turn_context_schema"], TURN_CONTEXT_SCHEMA)
        self.assertEqual(len(fixture["cases"]), 7)
        route = ("a", "b", "c", "b", "d")
        network = {("a", "b"): "l", ("b", "c"): "r", ("c", "b"): "t", ("b", "d"): "s"}
        for row in fixture["cases"]:
            edges, connections = route, network
            if row["name"] == "single":
                edges, connections = ("b",), {}
            elif row["name"] == "missing_connections":
                edges, connections = ("a", "b", "c"), {}
            expected = resolve_turn_context(edges, row["route_index"], connections)
            self.assertTrue(row["route_context_resolved"])
            for key in ("incoming_turn_type", "outgoing_turn_type", "incoming_turn_present", "outgoing_turn_present"):
                self.assertEqual(row[key], expected[key], (row["name"], key))
            self.assertEqual(row["turn_type"], expected["outgoing_turn_type"])


class CollectorTurnContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("SUMO_HOME", "/tmp")
        sys.modules.setdefault("traci", SimpleNamespace())
        import collection._shared.TraCI_Python_Adjusted as base
        from legacy.collection import traci_vehicle_features
        import tests.python.test_final_road_training as fixtures
        cls.base, cls.vehicle, cls.fixtures = base, traci_vehicle_features, fixtures

    def setUp(self):
        self.directory = Path(__file__).resolve().parent / (".test_turn_context_" + uuid.uuid4().hex)
        self.directory.mkdir()

    def tearDown(self):
        target = self.directory.resolve()
        if target.parent != Path(__file__).resolve().parent or not target.name.startswith(".test_turn_context_"):
            raise RuntimeError("unsafe test cleanup target")
        shutil.rmtree(target)

    def sampled_event(self, route, index, network):
        snapshot = self.fixtures.snapshot(final=index == len(route) - 1, route=route, route_index=index)
        with mock.patch.object(self.base, "compute_flow_snapshot", return_value=({}, {}, {})), \
                mock.patch.object(self.base, "read_vehicle_speed_factor", return_value=1.):
            event = self.base.make_vehicle_event(snapshot, 1., network, {}, self.fixtures.config())
        event.driving_time = 3.
        self.base.finalize_event(event, 4., [], arrived=event.is_final_road)
        for field, value in dict(vehicle_max_speed=13.89, vehicle_accel=2.6,
                                 vehicle_decel=4.5, vehicle_sigma=.5).items():
            setattr(event, field, value)
        return event

    def test_sampler_and_both_full_writers_preserve_compact_alias_and_labels(self):
        network = self.fixtures.network_info()
        events = [self.sampled_event(("a", "b", "a"), index, network) for index in range(3)]
        expected = [(0, 1), (1, 2), (2, 0)]
        for event, types in zip(events, expected):
            self.assertEqual((event.incoming_turn_type, event.outgoing_turn_type), types)
            self.assertEqual(event.turn_type, event.outgoing_turn_type)
            self.assertEqual(event.turn_direction, event.outgoing_turn_direction)
        for module in (self.base, self.vehicle):
            full, slim = self.directory / (module.__name__ + "-full.csv"), self.directory / (module.__name__ + "-slim.csv")
            module.write_training_rows(str(full), events)
            module.write_lightweight_training_rows(str(slim), events)
            with full.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                full_rows = list(reader)
                self.assertTrue(set(TURN_CONTEXT_COLUMNS).issubset(reader.fieldnames))
                self.assertEqual(len(reader.fieldnames), len(set(reader.fieldnames)))
            with slim.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                slim_rows = list(reader)
                self.assertFalse(set(TURN_CONTEXT_COLUMNS).intersection(reader.fieldnames))
                extra = [] if module is self.base else module.VEHICLE_FEATURES
                self.assertEqual(reader.fieldnames, ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"] +
                                 extra + ["speed_factor", "travel_time_label"])
            self.assertEqual(len(full_rows), 3)
            for full_row, slim_row, types in zip(full_rows, slim_rows, expected):
                self.assertEqual((int(full_row["incoming_turn_type"]), int(full_row["outgoing_turn_type"])), types)
                self.assertEqual(full_row["turn_type"], full_row["outgoing_turn_type"])
                self.assertEqual(full_row["turn_type"], slim_row["turn_type"])
                self.assertEqual(full_row["travel_time_label"], slim_row["travel_time_label"])
                self.assertEqual(full_row["turn_context_schema"], TURN_CONTEXT_SCHEMA)

    def test_legacy_helper_does_not_use_first_matching_edge(self):
        api = SimpleNamespace(getRoute=lambda vid: ("a", "b", "a", "c"), getRouteIndex=lambda vid: 2)
        with mock.patch.object(self.base, "traci", SimpleNamespace(vehicle=api)):
            self.assertEqual(self.base.get_next_edge_direction("v", "a", {
                "a": [{"to": "b", "dir": "l"}, {"to": "c", "dir": "r"}]}), "r")


if __name__ == "__main__":
    unittest.main()
