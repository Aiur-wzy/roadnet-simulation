import bisect
from collections import Counter
from pathlib import Path
import shutil
from types import SimpleNamespace
import unittest
import uuid

from features.signal_rg_features import SIGNAL_FEATURE_COLS
from collection.traci_signal_rg import TraCISignalRG


DURATIONS = (24., 3., 15., 3., 24., 3., 15., 3.)
STATES = ("Gr", "yr", "rG", "ry", "rr", "rr", "rr", "rr")


def logic(program_id="active", durations=DURATIONS, states=STATES, type_id=0):
    return SimpleNamespace(programID=program_id, type=type_id,
                           phases=tuple(SimpleNamespace(duration=d, state=s, next=())
                                        for d, s in zip(durations, states)))


class FakeTrafficLight:
    """Read-only API with a clock independent of the implementation under test."""

    def __init__(self, now, active=None):
        self.now = now
        self.active_id = "active"
        self.logics = [logic("inactive", (1., 1.), ("rG", "Gr")), active or logic()]
        self.links = ((("a_0", "b_0", ":ab_0"),), (("b_0", "c_0", ":bc_0"),))
        self.calls = Counter()
        self.origin = 0.
        self.end_shift = 0.
        self.phase_override = None
        self.state_override = None
        self.expose_just_ended = False

    def _called(self, name, tls):
        self.calls[name, tls] += 1

    def _logic(self):
        return next(p for p in self.logics if p.programID == self.active_id)

    def snapshot(self):
        program = self._logic()
        ends, total = [], 0.
        for phase in program.phases:
            total += phase.duration
            ends.append(total)
        now = self.now()
        local = (now - self.origin) % total
        index = bisect.bisect_right(ends, local)
        if self.expose_just_ended and (local == 0 or local in ends):
            index = (index - 1) % len(ends)
            end = now
        else:
            end = now - local + ends[index]
        if self.phase_override is not None:
            index = self.phase_override
        return index, end + self.end_shift

    def getProgram(self, tls):
        self._called("getProgram", tls)
        return self.active_id

    def getAllProgramLogics(self, tls):
        self._called("getAllProgramLogics", tls)
        return tuple(self.logics)

    def getControlledLinks(self, tls):
        self._called("getControlledLinks", tls)
        return self.links

    def getPhase(self, tls):
        self._called("getPhase", tls)
        return self.snapshot()[0]

    def getNextSwitch(self, tls):
        self._called("getNextSwitch", tls)
        return self.snapshot()[1]

    def getRedYellowGreenState(self, tls):
        self._called("getRedYellowGreenState", tls)
        if self.state_override is not None:
            return self.state_override
        return self._logic().phases[self.snapshot()[0]].state


def net_xml(connections=None, lanes=1):
    edges = ''.join('<edge id="{0}">{1}</edge>'.format(
        road, ''.join(f'<lane id="{road}_{i}" index="{i}" length="100" speed="10"/>'
                      for i in range(lanes))) for road in "abc")
    if connections is None:
        connections = ('<connection from="a" to="b" fromLane="0" toLane="0" tl="T" linkIndex="0"/>'
                       '<connection from="b" to="c" fromLane="0" toLane="0" tl="T" linkIndex="1"/>')
    # This fixture models traversing :ab/:bc; make that topology explicit.
    internal = '<edge id=":ab" function="internal"><lane id=":ab_0" index="0" length="3" speed="10"/></edge>'
    return '<net>' + edges + internal + connections + '</net>'


def cleanup_test_directory(root):
    resolved = root.resolve()
    if resolved.parent != Path(__file__).resolve().parent or not resolved.name.startswith(".test_rg_"):
        raise RuntimeError("unsafe R/G test cleanup target")
    shutil.rmtree(resolved)


class TraCISignalRGTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".test_rg_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(cleanup_test_directory, self.root)
        self.net = self.root / "net.xml"
        self.net.write_text(net_xml(), encoding="utf-8")
        self.api = SimpleNamespace(time=0.)
        self.api.simulation = SimpleNamespace(getTime=lambda: self.api.time)
        self.api.trafficlight = FakeTrafficLight(lambda: self.api.time)
        self.reader = TraCISignalRG(self.api, self.net)

    def values(self, time=0., entry=None, route=("a", "b", "c"), index=0, lane=0, reader=None):
        self.api.time = time
        return (reader or self.reader).features(route, index, lane, time, time if entry is None else entry)

    def assert_four(self, values, expected):
        self.assertEqual(tuple(values[name] for name in SIGNAL_FEATURE_COLS), expected)

    def test_active_program_not_first_program_and_complete_red_run(self):
        result = self.values(28.)
        self.assert_four(result, ("R", 62., "G", 24.))
        self.assertEqual(result["signal_program_id"], "active")
        self.assertEqual(result["signal_anchor_phase"], 2)
        self.assertEqual(result["signal_anchor_next_switch_time"], 42.)

    def test_yellow_skipped_not_added_to_durations(self):
        result = self.values(25.)
        self.assert_four(result, ("R", 63., "G", 24.))
        self.assertEqual(result["signal_state_at_reference"], "Y")
        self.assertEqual(result["signal_yellow_skipped"], 1)

    def test_green_priority_is_intentionally_normalized_to_color(self):
        # This schema retains light color, not protected/permissive right of way.
        self.api.trafficlight.logics[1] = logic(
            durations=(10., 5., 3., 12.), states=("Gr", "gr", "yr", "rG"))
        result = self.values(9.)
        self.assert_four(result, ("G", 6., "R", 12.))
        self.assertEqual(result["signal_state_at_reference"], "G")
        self.assertIn("state_normalization", self.reader.contract())

    def test_decision_and_planned_entry_use_different_signal_times(self):
        self.assert_four(self.values(23., entry=26.), ("R", 63., "G", 24.))
        self.assert_four(self.values(23., entry=23.), ("G", 1., "R", 63.))

    def test_outgoing_not_incoming_movement_is_used(self):
        result = self.values(23., entry=26., index=1)
        self.assert_four(result, ("R", 1., "G", 15.))
        self.assertEqual(result["signal_link_indices"], "1")

    def test_live_anchor_handles_nonzero_origin(self):
        self.api.trafficlight.origin = 40.
        self.assert_four(self.values(0.), ("R", 40., "G", 24.))
        self.assertEqual(self.reader.contract()["runtime_programs"]["T"]["initial_cycle_origin"], -50.)

    def test_exact_boundary_advances_even_if_getter_exposes_old_phase(self):
        self.api.trafficlight.expose_just_ended = True
        result = self.values(24.)
        self.assertEqual(result["signal_anchor_phase"], 0)
        self.assert_four(result, ("R", 63., "G", 24.))
        self.assertEqual(result["signal_state_at_reference"], "Y")

    def test_multiple_cycles_and_cross_cycle_same_red(self):
        self.assert_four(self.values(50., entry=950., index=1), ("R", 67., "G", 15.))

    def test_each_tls_clock_read_once_per_observation_time(self):
        self.values(1.)
        self.values(1., entry=4., index=1)
        self.values(1., entry=181.)
        for name in ("getProgram", "getPhase", "getNextSwitch", "getRedYellowGreenState"):
            self.assertEqual(self.api.trafficlight.calls[name, "T"], 1)
        self.values(2.)
        self.assertEqual(self.api.trafficlight.calls["getNextSwitch", "T"], 2)
        self.assertEqual(self.api.trafficlight.calls["getAllProgramLogics", "T"], 1)
        self.assertEqual(self.api.trafficlight.calls["getControlledLinks", "T"], 1)
        self.assertEqual(self.reader.clock_queries, 2)

    def test_final_road_is_explicit_no_outgoing_placeholder(self):
        result = self.values(2., index=2)
        self.assert_four(result, ("G", 0., "G", 0.))
        self.assertEqual(result["signal_status"], "no_outgoing_route")
        self.assertEqual(self.api.trafficlight.calls, Counter())

    def test_explicit_unsignalized_connection_has_distinct_status(self):
        phases = ''.join(f'<phase duration="{d}" state="{s}"/>' for d, s in zip(DURATIONS, STATES))
        self.net.write_text(net_xml('<connection from="a" to="b" fromLane="0" toLane="0"/>'
            '<connection from="b" to="c" fromLane="0" toLane="0" tl="T" linkIndex="1"/>')
            .replace('</net>', f'<tlLogic id="T" type="static" programID="active">{phases}</tlLogic></net>'), encoding="utf-8")
        reader = TraCISignalRG(self.api, self.net)
        result = self.values(reader=reader)
        # Only movement 1 is controlled/referenced in this network: H=15, not
        # unused movement 0's longer 24-second green.
        self.assert_four(result, ("G", 15., "G", 15.))
        self.assertEqual(result["signal_status"], "unsignalized_connection")
        before = self.api.trafficlight.calls.copy()
        self.assert_four(self.values(time=100., entry=1000., reader=reader), ("G", 15., "G", 15.))
        self.assertEqual(before, self.api.trafficlight.calls)
        self.assertEqual(reader.contract()["runtime_unsignalized_green_seconds"], 15.)

    def test_no_green_horizon_is_error_not_zero_placeholder(self):
        self.net.write_text(net_xml('<connection from="a" to="b" fromLane="0" toLane="0"/>'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "No finite network green"):
            self.values(reader=TraCISignalRG(self.api, self.net))

    def test_missing_connection_is_not_no_tls(self):
        for route, index, lane in ((("b", "a"), 0, 0), (("a", "b"), 0, 9)):
            with self.subTest(route=route, lane=lane), self.assertRaisesRegex(ValueError, "no outgoing connection"):
                self.values(route=route, index=index, lane=lane)

    def test_net_live_connection_mismatch_rejected(self):
        self.api.trafficlight.links = ((("wrong_0", "b_0", ""),), self.api.trafficlight.links[1])
        with self.assertRaisesRegex(ValueError, "net/live"):
            self.values()

    def test_out_of_range_link_rejected(self):
        self.net.write_text(net_xml('<connection from="a" to="b" fromLane="0" toLane="0" tl="T" linkIndex="99"/>'), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.values(reader=TraCISignalRG(self.api, self.net))

    def test_missing_controlled_link_index_rejected(self):
        self.net.write_text(net_xml('<connection from="a" to="b" fromLane="0" toLane="0" tl="T"/>'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "linkIndex"):
            TraCISignalRG(self.api, self.net)

    def test_equal_instant_but_different_full_sequences_are_ambiguous(self):
        connections = ('<connection from="a" to="b" fromLane="0" toLane="0" tl="T" linkIndex="0"/>'
                       '<connection from="a" to="b" fromLane="0" toLane="1" tl="T" linkIndex="1"/>')
        self.net.write_text(net_xml(connections, lanes=2), encoding="utf-8")
        self.api.trafficlight.links = ((("a_0", "b_0", ""),), (("a_0", "b_1", ""),))
        with self.assertRaisesRegex(ValueError, "ambiguous outgoing movement"):
            self.values(50., reader=TraCISignalRG(self.api, self.net))

    def test_multiple_connections_with_equal_programs_are_supported(self):
        connections = ('<connection from="a" to="b" fromLane="0" toLane="0" tl="T" linkIndex="0"/>'
                       '<connection from="a" to="b" fromLane="0" toLane="1" tl="T" linkIndex="1"/>')
        self.net.write_text(net_xml(connections, lanes=2), encoding="utf-8")
        self.api.trafficlight.logics[1] = logic(states=tuple(s[0] * 2 for s in STATES))
        self.api.trafficlight.links = ((("a_0", "b_0", ""),), (("a_0", "b_1", ""),))
        result = self.values(28., reader=TraCISignalRG(self.api, self.net))
        self.assert_four(result, ("R", 62., "G", 24.))
        self.assertEqual(result["signal_link_indices"], "0;1")

    def test_mixed_or_different_controllers_rejected(self):
        for extra in ('', ' tl="OTHER" linkIndex="1"'):
            connections = ('<connection from="a" to="b" fromLane="0" toLane="0" tl="T" linkIndex="0"/>'
                           f'<connection from="a" to="b" fromLane="0" toLane="1"{extra}/>')
            self.net.write_text(net_xml(connections, lanes=2), encoding="utf-8")
            with self.subTest(extra=extra), self.assertRaisesRegex(ValueError, "ambiguous outgoing signal controllers"):
                self.values(reader=TraCISignalRG(self.api, self.net))

    def test_waut_in_net_or_additional_rejected(self):
        additional = self.root / "additional.xml"
        for tag in ("WAUT", "wautJunction", "wautSwitch"):
            additional.write_text(f'<additional><{tag}/></additional>', encoding="utf-8")
            with self.subTest(tag=tag), self.assertRaisesRegex(ValueError, "WAUT"):
                TraCISignalRG(self.api, self.net, [additional])
        self.net.write_text(net_xml().replace('</net>', '<WAUT/></net>'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "WAUT"):
            TraCISignalRG(self.api, self.net)

    def test_actuated_or_duplicate_active_program_rejected(self):
        self.api.trafficlight.logics[1].type = 3
        with self.assertRaisesRegex(ValueError, "STATIC"):
            self.values()
        self.api.trafficlight.logics[1].type = 0
        self.api.trafficlight.logics.append(logic())
        with self.assertRaisesRegex(ValueError, "STATIC"):
            self.values()

    def test_nonsequential_phase_successor_rejected(self):
        self.api.trafficlight.logics[1].phases[0].next = (2,)
        with self.assertRaisesRegex(ValueError, "sequential"):
            self.values()

    def test_unknown_signal_char_and_constant_color_rejected(self):
        for char in ("u", "s", "o", "O", "X"):
            self.api.trafficlight.logics[1] = logic(states=tuple(char + s[1] for s in STATES))
            with self.subTest(char=char), self.assertRaises(ValueError):
                self.values(reader=TraCISignalRG(self.api, self.net))
        for char in ("G", "r", "y"):
            self.api.trafficlight.logics[1] = logic(states=tuple(char * 2 for _ in STATES))
            with self.subTest(constant=char), self.assertRaises(ValueError):
                self.values(reader=TraCISignalRG(self.api, self.net))

    def test_runtime_program_switch_rejected(self):
        self.values(0.)
        self.api.trafficlight.active_id = "inactive"
        with self.assertRaisesRegex(ValueError, "program changed"):
            self.values(1.)

    def test_runtime_extension_shortening_or_state_edit_rejected(self):
        self.values(0.)
        for shift in (1., -1.):
            self.api.trafficlight.end_shift = shift
            with self.subTest(shift=shift), self.assertRaisesRegex(ValueError, "calendar changed"):
                self.values(2.)
        self.api.trafficlight.end_shift = 0.
        self.api.trafficlight.state_override = "rr"
        with self.assertRaisesRegex(ValueError, "state differs"):
            self.values(3.)

    def test_invalid_runtime_phase_and_next_switch_rejected(self):
        self.api.trafficlight.phase_override = 99
        with self.assertRaisesRegex(ValueError, "phase index"):
            self.values()
        self.api.trafficlight.phase_override = None
        for shift in (100., float("nan"), float("inf"), -100.):
            self.api.trafficlight.end_shift = shift
            with self.subTest(shift=shift), self.assertRaises(ValueError):
                self.values(reader=TraCISignalRG(self.api, self.net))

    def test_causal_and_current_anchor_validation(self):
        self.api.time = 3.
        for decision, entry in ((3., 2.), (2., 5.), (float("nan"), 5.), (3., float("inf"))):
            with self.subTest(decision=decision, entry=entry), self.assertRaises(ValueError):
                self.reader.features(("a", "b", "c"), 0, 0, decision, entry)

    def test_contract_keeps_clock_and_program_provenance(self):
        self.values(28.)
        contract = self.reader.contract()
        self.assertEqual(contract["feature_columns"], SIGNAL_FEATURE_COLS)
        self.assertEqual(contract["runtime_programs"]["T"]["durations"], DURATIONS)
        self.assertEqual(contract["runtime_programs"]["T"]["states"], STATES)
        self.assertEqual(contract["clock_queries"], 1)
        self.assertIn("no future", contract["causality"])


if __name__ == "__main__":
    unittest.main()
