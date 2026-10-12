"""Native signal tests: pure mocks, isolated UUID fixture directories, no SUMO."""
from collections import Counter
from pathlib import Path
from types import SimpleNamespace as NS
import math
import shutil
import unittest
from unittest import mock
import uuid
import xml.etree.ElementTree as ET

from collection.sumo_native_signals import NativeSignalReader
from features.signal_rg_features import SIGNAL_FEATURE_COLS


def fixture_root(states=("GG", "yy", "rr"), durations=(10, 3, 7)):
    root = ET.Element("net")
    for edge_id, count in (("a", 3), ("b", 2), ("c", 1), ("d", 1)):
        edge = ET.SubElement(root, "edge", id=edge_id, **{"from": edge_id + "s", "to": edge_id + "t"})
        ET.SubElement(root, "junction", id=edge_id + "t", type="priority")
        for index in range(count):
            ET.SubElement(edge, "lane", id=f"{edge_id}_{index}", index=str(index), length="100", speed="10")
    for i in (0, 1):
        ET.SubElement(root, "connection", **{"from": "a", "to": "b", "fromLane": str(i),
                      "toLane": str(i), "tl": "T", "linkIndex": str(i)})
    ET.SubElement(root, "connection", **{"from": "b", "to": "c", "fromLane": "0", "toLane": "0"})
    ET.SubElement(root, "connection", **{"from": "c", "to": "d", "fromLane": "0", "toLane": "0", "tl": "U", "linkIndex": "0"})
    for tls, ds, ss in (("T", durations, states), ("U", (20, 3, 7), ("G", "y", "r"))):
        logic = ET.SubElement(root, "tlLogic", id=tls, type="static", programID="0", offset="0")
        for duration, state in zip(ds, ss):
            ET.SubElement(logic, "phase", duration=str(duration), state=state)
    return root


class FakeAPI:
    def __init__(self, root):
        self.time = 2.0
        self.calls = Counter()
        self.selected = ("T", 0)
        self.empty = False
        self.next_override = None
        self.origins = {}
        self.plans = {}
        self.links = {}
        for element in root.findall("tlLogic"):
            tls = element.get("id")
            self.origins[tls] = 0.0
            self.plans[tls] = NS(programID="0", type=0, phases=tuple(
                NS(duration=float(p.get("duration")), state=p.get("state"), next=()) for p in element.findall("phase")))
            self.links[tls] = [[] for _ in self.plans[tls].phases[0].state]
        for connection in root.findall("connection"):
            if connection.get("tl"):
                self.links[connection.get("tl")][int(connection.get("linkIndex"))].append((
                    connection.get("from") + "_" + connection.get("fromLane"),
                    connection.get("to") + "_" + connection.get("toLane"), ""))
        self.simulation = NS(getTime=lambda: self.time)
        self.vehicle = NS(getNextTLS=self.get_next_tls)
        self.trafficlight = NS(getProgram=lambda tls: self.call("getProgram", tls, self.plans[tls].programID),
            getAllProgramLogics=lambda tls: self.call("getAllProgramLogics", tls, [self.plans[tls]]),
            getControlledLinks=lambda tls: self.call("getControlledLinks", tls, self.links[tls]),
            getPhase=lambda tls: self.call("getPhase", tls, self.phase_end(tls)[0]),
            getNextSwitch=lambda tls: self.call("getNextSwitch", tls, self.phase_end(tls)[1]),
            getRedYellowGreenState=lambda tls: self.call("getRedYellowGreenState", tls,
                self.plans[tls].phases[self.phase_end(tls)[0]].state))

    def call(self, name, tls, result):
        self.calls[name, tls] += 1
        return result

    def phase_end(self, tls):
        phases, origin = self.plans[tls].phases, self.origins[tls]
        cycle = sum(p.duration for p in phases)
        turn = math.floor((self.time - origin) / cycle)
        elapsed = origin + turn * cycle
        for index, phase in enumerate(phases):
            elapsed += phase.duration
            if self.time < elapsed:
                return index, elapsed
        raise AssertionError("phase calculation")

    def get_next_tls(self, vehicle):
        self.calls["getNextTLS", vehicle] += 1
        if self.next_override is not None:
            return self.next_override
        if self.empty:
            return []
        tls, index = self.selected
        state = self.plans[tls].phases[self.phase_end(tls)[0]].state[index]
        return [(tls, index, 90.0, state)]


class NativeSignalsTests(unittest.TestCase):
    def setUp(self):
        self.root_dir = Path(__file__).resolve().parent / (".native_signal_" + uuid.uuid4().hex)
        self.root_dir.mkdir()
        self.addCleanup(self.cleanup)
        self.make()

    def cleanup(self):
        target = self.root_dir.resolve()
        if target.parent != Path(__file__).resolve().parent or not target.name.startswith(".native_signal_"):
            raise ValueError("unsafe test cleanup")
        shutil.rmtree(target)

    def make(self, root=None):
        self.net = fixture_root() if root is None else root
        self.path = self.root_dir / "network.net.xml"
        ET.ElementTree(self.net).write(self.path, encoding="utf-8")
        self.api = FakeAPI(self.net)
        self.reader = NativeSignalReader(self.api, self.path)

    def sample(self, route=("a", "b", "c", "d"), index=0, lane="a_0"):
        return self.reader.features("ego", route, index, lane, self.api.time)

    def assert_formal(self, row, values):
        self.assertEqual(tuple(row[column] for column in SIGNAL_FEATURE_COLS), values)

    def test_selected_signal_is_current_time_without_crossing(self):
        row = self.sample()
        self.assert_formal(row, ("G", 8.0, "R", 7.0))
        self.assertEqual(row["signal_projection_seconds"], 0)
        self.assertEqual(row["signal_reference_time"], self.api.time)
        self.assertEqual(row["signal_selection_method"], "sumo_next_tls")

    def test_necessary_lane_change_does_not_force_projected_lane(self):
        self.api.selected = ("T", 1)
        row = self.sample(lane="a_2")  # no immediate connection from actual lane
        self.assertEqual(row["signal_link_indices"], "1")
        self.assertEqual(row["signal_actual_lane_id"], "a_2")

    def test_yellow_is_skipped_without_becoming_a_feature_type(self):
        self.api.time = 11
        row = self.sample()
        self.assert_formal(row, ("R", 7.0, "G", 10.0))
        self.assertEqual((row["signal_state_at_reference"], row["signal_yellow_skipped"]), ("Y", 1))

    def test_same_time_queries_share_clock(self):
        self.sample()
        self.sample(lane="a_1")
        self.assertEqual(self.api.calls["getNextSwitch", "T"], 1)

    def test_unsignalized_uses_cached_static_horizon_no_global_api_scan(self):
        import collection.sumo_native_signals as module
        original = module.network_green_horizon
        with mock.patch.object(module, "network_green_horizon", wraps=original) as compute:
            self.assert_formal(self.sample(index=1, lane="b_0"), ("G", 20.0, "G", 20.0))
            self.api.time = 8
            self.assert_formal(self.sample(index=1, lane="b_0"), ("G", 20.0, "G", 20.0))
            self.assertEqual(compute.call_count, 1)
        self.assertEqual(self.api.calls, Counter())

    def test_terminal_needs_neither_horizon_nor_next_tls(self):
        self.assert_formal(self.sample(index=3, lane="d_0"), ("G", 0.0, "G", 0.0))
        self.assertEqual(self.api.calls, Counter())

    def test_empty_next_tls_can_use_whole_pair_calendar_equivalence(self):
        self.api.empty = True
        row = self.sample()
        self.assertEqual(row["signal_selection_method"], "whole_roadpair_calendar_equivalent")
        self.assertEqual(row["signal_selection_fallback_reason"], "next_tls_empty")
        self.assertEqual(row["signal_selected_link_count"], 2)

    def test_remote_tls_never_supplies_immediate_exit_features(self):
        self.api.selected = ("U", 0)
        row = self.sample()
        self.assert_formal(row, ("G", 8.0, "R", 7.0))  # U would have G18 R7
        self.assertEqual(row["signal_tls_id"], "T")
        self.assertEqual(row["signal_selection_fallback_reason"], "next_tls_not_immediate_pair")

    def test_full_calendar_not_just_current_color_must_match(self):
        self.make(fixture_root(states=("GG", "yr", "ry")))
        self.api.empty = True
        with self.assertRaisesRegex(ValueError, "non-equivalent complete calendars"):
            self.sample()

    def test_exact_selection_handles_non_equivalent_pair(self):
        self.make(fixture_root(states=("GG", "yr", "ry")))
        self.api.selected = ("T", 1)
        row = self.sample(lane="a_0")
        self.assertEqual(row["signal_selection_method"], "sumo_next_tls")
        self.assert_formal(row, ("G", 8.0, "R", 3.0))

    def test_normalized_case_equivalence(self):
        self.make(fixture_root(states=("Gg", "yY", "rR")))
        self.api.empty = True
        self.assertEqual(self.sample()["signal_selection_method"], "whole_roadpair_calendar_equivalent")

    def cross_controller_net(self, rotated=False):
        root = fixture_root()
        root.findall("connection")[1].set("tl", "U")
        root.findall("connection")[1].set("linkIndex", "0")
        logic = root.find("tlLogic[@id='U']")
        for phase in list(logic):
            logic.remove(phase)
        ds, ss = ((3, 7, 10), ("y", "r", "G")) if rotated else ((10, 3, 7), ("G", "y", "r"))
        for duration, state in zip(ds, ss):
            ET.SubElement(logic, "phase", duration=str(duration), state=state)
        return root

    def test_equivalent_rotated_calendars_across_tls(self):
        self.make(self.cross_controller_net(rotated=True))
        self.api.origins["U"] = -10
        self.api.empty = True
        self.assert_formal(self.sample(), ("G", 8.0, "R", 7.0))

    def test_different_absolute_offsets_are_not_equivalent(self):
        self.make(self.cross_controller_net())
        self.api.origins["U"] = 2
        self.api.empty = True
        with self.assertRaisesRegex(ValueError, "non-equivalent complete calendars"):
            self.sample()

    def test_mixed_uncontrolled_pair_without_selection_is_unknown(self):
        root = fixture_root()
        for name in ("tl", "linkIndex"):
            del root.findall("connection")[1].attrib[name]
        self.make(root)
        self.api.empty = True
        with self.assertRaisesRegex(ValueError, "controlled/uncontrolled"):
            self.sample()

    def test_runtime_non_static_is_unknown(self):
        self.api.plans["T"].type = 3
        with self.assertRaisesRegex(ValueError, "STATIC"):
            self.sample()

    def test_now_and_actual_lane_are_checked(self):
        for now in (float("nan"), 1.0, True):
            with self.subTest(now=now), self.assertRaises(ValueError):
                self.reader.features("ego", ("a", "b"), 0, "a_0", now)
        with self.assertRaisesRegex(ValueError, "actual SUMO lane"):
            self.sample(lane="b_0")

    def test_optional_initialization_verifies_all_tls_once(self):
        result = self.reader.verify_all_static(self.api.time)
        self.assertEqual(result, {"verified_tls_count": 2, "green_horizon": 20.0})
        self.api.time = 3
        self.reader.verify_all_static(self.api.time)
        self.assertEqual(self.api.calls["getNextSwitch", "U"], 1)
        self.assertTrue(self.reader.contract()["runtime_all_static_verified"])

    def test_horizon_queries_validate_only_the_tls_that_is_used(self):
        self.sample(index=1, lane="b_0")
        self.sample()
        self.assertEqual(self.api.calls["getNextSwitch", "T"], 1)
        self.assertEqual(self.api.calls["getNextSwitch", "U"], 0)

    def test_live_program_different_from_horizon_definition_is_unknown(self):
        self.sample(index=1, lane="b_0")
        self.api.plans["T"].programID = "other"
        with self.assertRaisesRegex(ValueError, "horizon net/queried live program mismatch"):
            self.sample()

    def test_missing_horizon_fails_repeatedly_without_reparsing(self):
        root = fixture_root()
        for connection in root.findall("connection"):
            connection.attrib.pop("tl", None)
            connection.attrib.pop("linkIndex", None)
        for logic in list(root.findall("tlLogic")):
            root.remove(logic)
        self.make(root)
        import collection.sumo_native_signals as module
        with mock.patch.object(module, "network_green_horizon", wraps=module.network_green_horizon) as compute:
            for _ in range(2):
                with self.assertRaisesRegex(ValueError, "No finite network green"):
                    self.sample()
            self.assertEqual(compute.call_count, 1)

    def test_contract_does_not_claim_planned_crossing_reference(self):
        contract = self.reader.contract()
        self.assertEqual(contract["version"], "sumo-native-observed-rg-v1")
        self.assertNotIn("crossing_policy", contract)
        self.assertIn("no crossing projection", contract["reference_time"])


if __name__ == "__main__":
    unittest.main()
