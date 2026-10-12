"""Adversarial unit and tiny private-SUMO integration checks for the adapter."""

import json
from contextlib import contextmanager
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import uuid
import xml.etree.ElementTree as ET
from roadnet_paths import ROOT

from collection.discharge_sumo import (Connection, _prepare_isolated_inputs, build_network_registry,
    classify_gate, internal_path, iter_sumo_events, resolve_inputs,
    resolve_passage_connection, stopped_cohort, valid_speed)


def vehicle(name="a", position=95, speed=0, lane="in_0"):
    return {"vehicle_id": name, "vehicle_key": name + "|visit=1", "lane_id": lane,
            "position": position, "speed": speed, "length": 5, "min_gap": 2.5,
            "route": ("in", "out"), "route_index": 0, "type_id": "car"}


@contextmanager
def fixture_directory():
    # Windows' restricted token cannot reopen tempfile's mode=0700 directories
    # on this host. Ordinary workspace mkdir is portable; retain tiny evidence.
    directory = ROOT / "eval" / "discharge_adapter_tests" / uuid.uuid4().hex
    directory.mkdir(parents=True)
    yield directory


class GateTests(unittest.TestCase):
    def setUp(self):
        self.connection = Connection("in", "in_0", ":j_0", "out", "out_0", "j", 0, "s")
        self.registry = {"lanes": {"in_0": {"length": 100, "internal": False},
                                    ":j_0": {"length": 10, "internal": True},
                                    "out_0": {"length": 100, "internal": False}},
                         "internal_successors": {":j_0": ["out_0"]}}
        self.links = [("out_0", True, True, False, ":j_0", "G", "s", 10)]

    def gate(self, lanes, **kwargs):
        return classify_gate(self.connection, self.links, lanes, vehicle(), self.registry, **kwargs)

    def test_zero_is_not_missing_or_negative_speed(self):
        self.assertTrue(valid_speed(0))
        for value in (None, -1, float("nan"), float("inf")):
            self.assertFalse(valid_speed(value))

    def test_initial_rear_zeros_are_cohort(self):
        rows = [vehicle("a", 99), vehicle("b", 91.5), vehicle("c", 84)]
        self.assertEqual(len(stopped_cohort(rows, 100)), 3)

    def test_rolling_head_is_not_a_stationary_start(self):
        self.assertEqual(stopped_cohort([vehicle("a", 99, 1), vehicle("b", 91)], 100), [])

    def test_distant_queue_is_not_joined(self):
        self.assertEqual(len(stopped_cohort([vehicle("a", 99), vehicle("b", 60)], 100)), 1)

    def test_recently_stopped_rear_does_not_veto_qualified_head(self):
        rows = [vehicle("a", 99), vehicle("b", 91.5)]
        cohort = stopped_cohort(rows, 100, stop_since={rows[0]["vehicle_key"]: 1,
            rows[1]["vehicle_key"]: 2.5}, observed_time=3, min_stopped_duration=1)
        self.assertEqual([item["vehicle_id"] for item in cohort], ["a"])

    def test_unqualified_head_cannot_be_skipped_for_qualified_rear(self):
        rows = [vehicle("a", 99), vehicle("b", 91.5)]
        cohort = stopped_cohort(rows, 100, stop_since={rows[0]["vehicle_key"]: 2.5,
            rows[1]["vehicle_key"]: 1}, observed_time=3, min_stopped_duration=1)
        self.assertEqual(cohort, [])

    def test_unqualified_middle_cannot_be_bridged(self):
        rows = [vehicle("a", 99), vehicle("b", 91.5), vehicle("c", 84)]
        cohort = stopped_cohort(rows, 100, stop_since={rows[0]["vehicle_key"]: 1,
            rows[1]["vehicle_key"]: 2.5, rows[2]["vehicle_key"]: 1}, observed_time=3, min_stopped_duration=1)
        self.assertEqual([item["vehicle_id"] for item in cohort], ["a"])

    def test_empty_observed_receiver_is_clear(self):
        self.assertEqual(self.gate({"out_0": [], ":j_0": []})["status"], "clear")

    def test_green_without_receiver_observation_is_unknown(self):
        self.assertEqual(self.gate({":j_0": []})["status"], "unknown")

    def test_planned_stopped_head_is_not_green_start_opportunity(self):
        planned = vehicle()
        planned["stop_state"] = 1
        self.assertEqual(classify_gate(self.connection, self.links, {"out_0": [], ":j_0": []},
                                      planned, self.registry)["reason"], "planned_stop")

    def test_receiver_entry_blocked_despite_long_empty_tail(self):
        row = vehicle(position=5, lane="out_0")
        self.assertEqual(self.gate({"out_0": [row], ":j_0": []})["reason"], "receiver_space")

    def test_own_moving_internal_vehicle_is_intrinsic_following(self):
        row = vehicle(position=2, speed=3, lane=":j_0")
        self.assertEqual(self.gate({"out_0": [], ":j_0": [row]}, own_keys={row["vehicle_key"]})["status"], "clear")

    def test_own_stopped_internal_vehicle_hard_blocks(self):
        row = vehicle(position=2, speed=0, lane=":j_0")
        self.assertEqual(self.gate({"out_0": [], ":j_0": [row]}, own_keys={row["vehicle_key"]})["reason"], "internal_stopped")

    def test_unrelated_moving_internal_vehicle_is_not_certified_clear(self):
        self.assertEqual(self.gate({"out_0": [], ":j_0": [vehicle(speed=3)]})["status"], "unknown")

    def test_crawling_own_leader_with_connected_stopped_queue_is_blocked(self):
        own = vehicle("a", position=6, speed=.2, lane="out_0")
        stopped = vehicle("b", position=13.5, speed=0, lane="out_0")
        result = self.gate({"out_0": [own, stopped], ":j_0": []}, own_keys={own["vehicle_key"]})
        self.assertEqual(result["reason"], "receiver_connected_stopped_queue")

    def test_distant_stopped_car_does_not_block_a_free_entry_queue(self):
        own = vehicle("a", position=6, speed=2, lane="out_0")
        distant = vehicle("b", position=60, speed=0, lane="out_0")
        self.assertEqual(self.gate({"out_0": [own, distant], ":j_0": []}, own_keys={own["vehicle_key"]})["status"], "clear")

    def test_permission_domain_distinguishes_protected_yield_unsignalized(self):
        self.assertEqual(self.gate({"out_0": [], ":j_0": []})["permission_class"], "protected")
        self.links = [("out_0", True, True, False, ":j_0", "g", "s", 10)]
        self.assertEqual(self.gate({"out_0": [], ":j_0": []})["permission_class"], "yield")
        self.connection = Connection("in", "in_0", ":j_0", "out", "out_0", "", None, "s")
        self.assertEqual(self.gate({"out_0": [], ":j_0": []})["permission_class"], "unsignalized")

    def test_actual_via_resolves_shared_route_connections(self):
        alternate = Connection("in", "in_0", ":other", "out", "out_1", "j", 1, "s")
        result, quality = resolve_passage_connection([self.connection, alternate], {":j_0"}, self.registry)
        self.assertEqual(result, self.connection)
        self.assertEqual(quality, "observed_path")

    def test_ambiguous_route_never_picks_first(self):
        alternate = Connection("in", "in_0", ":other", "out", "out_1", "j", 1, "s")
        self.assertIsNone(resolve_passage_connection([self.connection, alternate], set(), self.registry)[0])

    def test_internal_branch_is_unknown(self):
        self.registry["internal_successors"][":j_0"] = ["out_0", "out_1"]
        self.assertIsNone(internal_path(self.connection, self.registry))


def make_fixture(directory, blocked_lane=False):
    """Two independent main lanes, a TLS, and an unused conflicting approach."""
    directory = Path(directory)
    (directory / "nodes.xml").write_text('<nodes><node id="w" x="-200" y="0"/>'
        '<node id="j" x="0" y="0" type="traffic_light"/>'
        '<node id="e" x="200" y="0"/><node id="n" x="0" y="200"/>'
        '<node id="s" x="0" y="-200"/></nodes>', encoding="utf-8")
    (directory / "edges.xml").write_text('<edges><edge id="in" from="w" to="j" numLanes="2" speed="13.89"/>'
        '<edge id="out" from="j" to="e" numLanes="2" speed="13.89"/>'
        '<edge id="north" from="n" to="j" speed="13.89"/>'
        '<edge id="south" from="j" to="s" speed="13.89"/></edges>', encoding="utf-8")
    (directory / "connections.xml").write_text('<connections>'
        '<connection from="in" to="out" fromLane="0" toLane="0"/>'
        '<connection from="in" to="out" fromLane="1" toLane="1"/>'
        '<connection from="north" to="south" fromLane="0" toLane="0"/></connections>', encoding="utf-8")
    subprocess.run([shutil.which("netconvert"), "--node-files", str(directory / "nodes.xml"),
                    "--edge-files", str(directory / "edges.xml"),
                    "--connection-files", str(directory / "connections.xml"),
                    "--output-file", str(directory / "net.xml")], check=True, capture_output=True, text=True)
    net = ET.parse(directory / "net.xml")
    logic = net.getroot().find("tlLogic")
    count = len(logic.find("phase").attrib["state"])
    for item in list(logic):
        logic.remove(item)
    ET.SubElement(logic, "phase", duration="60", state="r" * count)
    ET.SubElement(logic, "phase", duration="60", state="G" * count)
    ET.SubElement(logic, "phase", duration="3", state="y" * count)
    net.write(directory / "net.xml", encoding="utf-8", xml_declaration=True)
    lane_settings = ' lcStrategic="-1" lcCooperative="-1" lcSpeedGain="-1" lcKeepRight="-1"' if blocked_lane else ''
    routes = '<routes><vType id="car" sigma="0" speedFactor="1" speedDev="0"%s/><route id="r" edges="in out"/>' % lane_settings
    for number in range(12):
        body = '<stop lane="out_0" endPos="10" until="85"/>' if blocked_lane and number == 0 else ''
        routes += '<vehicle id="v%d" type="car" route="r" depart="%s" departLane="%d">%s</vehicle>' % (number, number * .2, number % 2, body)
    (directory / "routes.xml").write_text(routes + '</routes>', encoding="utf-8")
    return directory / "net.xml", directory / "routes.xml"


@unittest.skipUnless(shutil.which("sumo") and shutil.which("netconvert"), "SUMO/netconvert unavailable")
class LiveFixtureTests(unittest.TestCase):
    def test_green_block_unblock_splits_only_affected_lane(self):
        self._assert_green_block_unblock(stop_duration=.2, expected_restart_size=2)

    def test_cli_default_stop_duration_keeps_qualified_recovery_prefix(self):
        self._assert_green_block_unblock(stop_duration=None, expected_restart_size=1)

    def _assert_green_block_unblock(self, stop_duration, expected_restart_size):
        with fixture_directory() as temporary:
            directory = Path(temporary)
            net, routes = make_fixture(directory, blocked_lane=True)
            metadata = {}
            debug = []
            config = {"net": str(net), "routes": [str(routes)],
                "output_dir": directory / "observations", "end": 115, "seed": 17,
                "step_length": .1, "run_id": "restart", "metadata": metadata,
                "_debug_gate_sink": lambda row: debug.append(row) if row["lane_id"] == "in_0" and 84 <= row["time"] <= 92 else None}
            if stop_duration is not None:
                config["stop_duration"] = stop_duration
            events = list(iter_sumo_events(config))
            (directory / "debug.json").write_text(json.dumps(debug, indent=2), encoding="utf-8")
            (directory / "events.json").write_text(json.dumps(events, indent=2), encoding="utf-8")
            from features.discharge_sequences import MemoryStore, SequenceExtractor
            store = MemoryStore()
            engine = SequenceExtractor("restart", store=store)
            for item in events:
                engine.feed(item)
            engine.finish()
            sequences = list(store.iter_sequences())
            (directory / "sequences.json").write_text(json.dumps(sequences, indent=2), encoding="utf-8")
            lane_zero = [item for item in sequences if item["lane_id"] == "in_0"]
            lane_one = [item for item in sequences if item["lane_id"] == "in_1"]
            self.assertEqual(len(lane_one), 1, str(sequences))
            self.assertGreaterEqual(len(lane_one[0]["headways"]), 3)
            self.assertGreaterEqual(len(lane_zero), 2, str(sequences))
            self.assertEqual(lane_zero[-1]["start_kind"], "stopped_unblock_start")
            self.assertLess(lane_zero[0]["cut_time"], lane_zero[-1]["anchor_time"])
            self.assertEqual(lane_zero[-1]["cohort_size"], expected_restart_size)
            self.assertEqual(len(lane_zero[-1]["headways"]), expected_restart_size - 1)
            self.assertEqual([item.split("|", 1)[0] for item in lane_zero[-1]["cohort_vehicle_keys"]],
                             ["v6"] if expected_restart_size == 1 else ["v6", "v8"])
            self.assertEqual([item["vehicle_id"] for item in lane_zero[-1]["passages"]],
                             ["v6"] if expected_restart_size == 1 else ["v6", "v8"])
            self.assertIsNotNone(lane_zero[-1]["startup_delay"])
            self.assertEqual(metadata["observation"]["stop_duration"], 1 if stop_duration is None else stop_duration)
            self.assertEqual(metadata["observation"]["cohort_distance"], 100)
            for sequence in sequences:
                for passage in sequence["passages"]:
                    self.assertGreaterEqual(passage["time"], sequence["anchor_time"])
                    self.assertLessEqual(passage["time"], sequence["cut_time"])

    def test_physical_lanes_e1_end_and_green_start(self):
        with fixture_directory() as temporary:
            directory = Path(temporary)
            net, routes = make_fixture(directory)
            metadata = {}
            events = list(iter_sumo_events({"net": str(net), "routes": [str(routes)],
                "output_dir": directory / "observations", "end": 95, "seed": 17,
                "step_length": .1, "run_id": "test", "metadata": metadata}))
            passages = [item for item in events if item["kind"] == "passage"]
            starts = [item for item in events if item["kind"] == "start"]
            self.assertEqual(len(passages), 12)
            self.assertEqual({item["lane_id"] for item in passages}, {"in_0", "in_1"})
            self.assertEqual({item["lane_id"] for item in starts}, {"in_0", "in_1"}, json.dumps(metadata))
            self.assertEqual(metadata["detector_offset"], 0)
            self.assertTrue(all(item["connection_resolution"] != "ambiguous_connection" for item in passages))
            for lane in ("in_0", "in_1"):
                times = [item["time"] for item in events if item["lane_id"] == lane]
                self.assertEqual(times, sorted(times))
            self.assertTrue(metadata["completed"])
            from features.discharge_sequences import MemoryStore, SequenceExtractor
            store = MemoryStore()
            engine = SequenceExtractor("test", store=store)
            for item in events:
                engine.feed(item)
            engine.finish()
            sequences = list(store.iter_sequences())
            self.assertEqual(len(sequences), 2)
            self.assertTrue(all(item["cohort_size"] >= 3 for item in sequences), str(sequences))
            self.assertGreaterEqual(sum(len(item["headways"]) for item in sequences), 4, str(sequences))

    def test_configuration_outputs_are_isolated(self):
        with fixture_directory() as temporary:
            directory = Path(temporary)
            net, routes = make_fixture(directory)
            cfg = directory / "original.sumocfg"
            original_output = directory / "DO_NOT_TOUCH.xml"
            original_output.write_text("sentinel", encoding="utf-8")
            cfg.write_text('<configuration><input><net-file value="net.xml"/><route-files value="routes.xml"/></input>'
                '<output><tripinfo-output value="DO_NOT_TOUCH.xml"/></output></configuration>', encoding="utf-8")
            output = directory / "observations"
            output.mkdir()
            cloned, _ = _prepare_isolated_inputs(resolve_inputs({"sumo_config": str(cfg)}), output)
            value = ET.parse(cloned).getroot().find("output/tripinfo-output").attrib["value"]
            self.assertTrue(Path(value).is_relative_to(output))
            self.assertEqual(original_output.read_text(encoding="utf-8"), "sentinel")

    def test_unsignalized_connections_are_registered_not_discarded(self):
        with fixture_directory() as directory:
            net, _ = make_fixture(directory)
            tree = ET.parse(net)
            for node in tree.getroot().findall("connection"):
                node.attrib.pop("tl", None)
                node.attrib.pop("linkIndex", None)
            tree.write(net, encoding="utf-8", xml_declaration=True)
            registry = build_network_registry(net)
            self.assertEqual(sum(len(items) for items in registry["connections"].values()), 3)
            self.assertTrue(all(not item.tls and item.link_index is None
                                for items in registry["connections"].values() for item in items))


if __name__ == "__main__":
    unittest.main()
