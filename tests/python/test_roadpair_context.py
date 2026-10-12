"""Rule parity with unmodified production C++, and observer causal tests."""
import csv
import json
import os
from pathlib import Path
import subprocess
import shutil
import uuid
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from collection.roadpair_context import RoadPairNetwork, ReservationLedger, RoadPairCapContextObserver, SAMPLING
from collection._shared.TraCI_Python_Cap_Context import write_feature_rows
from features.cap_context_features import ShadowRecord, ContextConfig
from tests.python.test_traci_cap_context import FakeTraCI


def network_xml(incoming=((0, 0),), outgoing=((0, 0), (1, 0)), source_lanes=1, receiving_lanes=2):
    root = ET.Element("net")
    for road, count in (("a", source_lanes), ("b", receiving_lanes), ("c", 1)):
        edge = ET.SubElement(root, "edge", id=road, **{"from": road+"s", "to": road+"t"})
        ET.SubElement(root, "junction", id=road+"t", type="priority")
        for lane in range(count):
            ET.SubElement(edge, "lane", id=f"{road}_{lane}", index=str(lane), length="100", speed="10")
    for pair, connections in ((("a", "b"), incoming), (("b", "c"), outgoing)):
        for frm, to in connections:
            ET.SubElement(root, "connection", **{"from": pair[0], "to": pair[1], "fromLane": str(frm), "toLane": str(to)})
    return root


class RoadPairRulesTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".test_roadpair_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        target = self.root.resolve()
        if target.parent != Path(__file__).resolve().parent or not target.name.startswith(".test_roadpair_"):
            raise ValueError("unsafe test cleanup")
        shutil.rmtree(target)

    def network(self, root):
        path = self.root / "net.xml"
        ET.ElementTree(root).write(path)
        return RoadPairNetwork(path, {"length": 5, "minGap": 2.5})

    def test_withdrawn_shadow_opt_in_rejected_and_historical_default_preserved(self):
        from collection import TraCI_Python_Cap_Context_ABC_RG as entry
        with patch.object(entry.shared, "main") as run:
            with self.assertRaisesRegex(ValueError, "shadow admission collection was withdrawn"):
                entry.main(["--context-lane-policy", "road-pair-reselection-v1", "--sumo-config", "fake.xml"])
            run.assert_not_called()
        with patch.object(entry.shared, "main") as run:
            entry.main(["--sumo-config", "fake.xml"])
        args, kwargs = run.call_args
        self.assertNotIn("--immediate-roadpair-labels", args[0])
        self.assertIs(kwargs["observer_factory"].func, entry.ABCRGCapContextObserver)
        self.assertNotIn("roadpair_context", kwargs["extra_source_paths"])

    def test_empty_intersection_not_required_and_entry_capacity(self):
        net = self.network(network_xml())
        ledger = ReservationLedger()
        ledger.reserve(ShadowRecord("other", 0, "b", 0, 0, 100, 10))
        self.assertEqual(net.select(("a", "b", "c"), 1, 0, "passenger", ledger), (0, 0, 1))
        net.capacities["b"] = 1
        with self.assertRaisesRegex(ValueError, "admission disagreement"):
            net.select(("a", "b", "c"), 1, 0, "passenger", ledger)
        self.assertEqual(len(ledger), 1)

    def test_pending_counts_in_flow_not_context(self):
        ledger = ReservationLedger()
        ledger.reserve(ShadowRecord("pending", 0, "b", 1, 10, 100, 2))
        ego = ShadowRecord("ego", 1, "b", 1, 8, 100, 10)
        values = ledger.query(ego, 5)
        self.assertEqual((values["pending_count"], values["candidate_count"], values["platoon_context"]), (1, 0, 0))
        ledger.reserve(ego)
        self.assertEqual(ledger.lane_load("b", 1), (2, 15.0))
        self.assertEqual(ledger.road_counts["b"], 2)
        ledger.release("ego", 1)
        self.assertEqual(ledger.lane_load("b", 1), (1, 7.5))

    def test_edge_means_match_cpp_not_selected_lane_geometry(self):
        root = network_xml()
        root.find("./edge[@id='b']/lane[@index='1']").set("length", "80")
        root.find("./edge[@id='b']/lane[@index='1']").set("speed", "8")
        net = self.network(root)
        self.assertEqual((net.lengths["b"], net.speed_limits["b"]), (90, 9))
        self.assertEqual(net.capacities["b"], 13)

    def test_original_cpp_probe(self):
        exe = os.environ.get("ROADPAIR_CPP_PROBE")
        if not exe:
            self.skipTest("set ROADPAIR_CPP_PROBE to freshly compiled production-kernel probe")
        evidence = json.loads(subprocess.run([exe], capture_output=True, text=True, check=True).stdout)
        for case in evidence["cases"]:
            if not case["enabled"]:
                continue  # Legacy is covered by unchanged historical collector tests.
            with self.subTest(case=case["name"]):
                root = network_xml(case["incoming"], case["outgoing"], case["source_lanes"], case["receiving_lanes"])
                connections = root.findall("connection")
                for i in case["bus_only_incoming"]:
                    connections[i].set("allow", "bus")
                for i in case["bus_only_outgoing"]:
                    connections[len(case["incoming"])+i].set("allow", "bus")
                for i in case["bus_only_receiving"]:
                    root.find(f"./edge[@id='b']/lane[@index='{i}']").set("allow", "bus")
                if case["bus_only_source"]:
                    root.find("./edge[@id='a']/lane").set("allow", "bus")
                if case["missing_internal_path"]:
                    connections[0].set("via", ":missing_0")
                net = self.network(root)
                self.assertEqual([net.capacities["b"]]*case["receiving_lanes"], case["default_capacity"])
                ledger = ReservationLedger()
                ledger.loads = {("b", i): (n, case["occupied_length"][i]) for i, n in enumerate(case["flow"])}
                ledger.road_counts["b"] = sum(case["flow"])
                net.has_capacity = lambda road, lane, state: state.lane_load(road, lane)[0] < case["capacity"][lane]
                route = ("a", "b") if case["final_road"] else ("a", "b", "c")
                if not case["feasible"]:
                    with self.assertRaises(ValueError):
                        net.select(route, 1, case["source_lane"], "passenger", ledger)
                    continue
                selected = net.select(route, 1, case["source_lane"], "passenger", ledger)
                self.assertEqual(list(selected), case["selected"])
                ledger.reserve(ShadowRecord("ego", 1, "b", selected[2], 1, 100, 10))
                self.assertEqual([ledger.lane_load("b", i)[0] for i in range(case["receiving_lanes"])], case["post_reserve_lane_flow"])
                self.assertEqual(ledger.road_counts["b"], case["post_reserve_road_flow"])


class RoadPairObserverTest(RoadPairRulesTest):
    def setUp(self):
        super().setUp()
        ET.ElementTree(network_xml()).write(self.root / "net.xml")
        (self.root / "route.xml").write_text(
            '<routes><vType id="car" length="5" maxSpeed="10"/>'
            '<vehicle id="ego" type="car" depart="0" speedFactor="1"><route edges="a b c"/></vehicle>'
            '<vehicle id="leader" type="car" depart="1" speedFactor="0.5"><route edges="a b c"/></vehicle></routes>')
        (self.root / "config.xml").write_text('<configuration><input><route-files value="route.xml"/></input></configuration>')
        self.api = FakeTraCI()
        self.arrived = []
        self.api.simulation.getArrivedIDList = lambda: self.arrived
        self.observer = RoadPairCapContextObserver(self.api, self.root / "config.xml", self.root / "net.xml")
        # Signal arithmetic is independently covered by the existing R/G suite.
        self.observer.signals.features = lambda *args: dict(entry_signal_type="G", entry_signal_remaining_seconds=90.,
                                                          next_signal_type="G", next_signal_duration_seconds=90.)

    def step(self, time, rows, arrived=()):
        self.arrived = list(arrived)
        self.api.step(time, rows)
        self.observer.on_step()

    def test_selector_and_flow_freeze_before_actual_entry(self):
        self.step(0, {"ego": ("a", 0)})
        self.observer.ledger.reserve(ShadowRecord("synthetic", 0, "b", 0, 0, 100, 10))
        self.step(1, {"ego": ("b", 1), "leader": ("a", 0)})
        row = self.observer.frozen["ego", 1]
        self.assertEqual((row["shadow_entry_lane"], row["logical_lane_index"]), (0, 1))
        self.assertEqual((row["shadow_road_flow"], row["shadow_lane_flow"]), (2, 1))
        self.assertEqual(row["planned_enter_time"], 1)  # no internal: not 4s
        self.assertEqual(self.observer.ledger.lane_load("b", 0)[0], 1)
        self.assertEqual(row["shadow_source_lane"], 0)

    def test_first_queue_is_assigned_at_schedule_not_actual_departure(self):
        self.step(0, {})
        self.assertEqual(self.observer.entry_assignments["ego"], ("a", 0, 0, 0))
        self.step(3, {"ego": ("a", 0)})
        self.assertEqual(self.observer.frozen["ego", 0]["decision_time"], 3)
        self.assertEqual(self.observer.entry_assignments["leader"][2], 1)

    def test_source_ledger_retained_on_admission_failure(self):
        self.step(0, {"ego": ("a", 0)})
        self.observer.network.capacities["b"] = 1
        self.observer.ledger.reserve(ShadowRecord("block", 0, "b", 0, 0, 100, 10))
        with self.assertRaisesRegex(ValueError, "admission disagreement"):
            self.step(.5, {"ego": ("b", 1)})
        self.assertEqual(self.observer.ledger.record_for("ego").road_id, "a")
        self.assertNotIn(("ego", 1), self.observer.frozen)

    def test_gap_does_not_fabricate_intermediate_feature(self):
        self.step(0, {"ego": ("a", 0)})
        with self.assertRaisesRegex(ValueError, "skipped"):
            self.step(.5, {"ego": ("c", 2)})
        self.assertNotIn(("ego", 1), self.observer.frozen)

    def test_unobserved_or_abnormal_disappearance_rejected(self):
        with self.assertRaisesRegex(ValueError, "unseen=2"):
            self.observer.validate_completion()
        self.step(0, {"ego": ("a", 0)})
        with self.assertRaisesRegex(ValueError, "abnormal"):
            self.step(.5, {})

    def test_arrival_cannot_hide_a_skipped_terminal_road(self):
        self.step(0, {"ego": ("a", 0)})
        with self.assertRaisesRegex(ValueError, "unobserved terminal"):
            self.step(.5, {}, arrived=("ego",))

    def test_shadow_completeness_does_not_hide_label_loss(self):
        self.step(0, {"ego": ("a", 0)})
        event = SimpleNamespace(vehicle_id="ego", route_index=0)
        keep = SimpleNamespace(iter_valid_training_events=iter)
        self.observer.validate_training_events([event], keep)
        for events in ([], [event, event]):
            with self.assertRaisesRegex(ValueError, "label occurrences"):
                self.observer.validate_training_events(events, keep)
        reject = SimpleNamespace(iter_valid_training_events=lambda _: ())
        with self.assertRaisesRegex(ValueError, "silent filtering"):
            self.observer.validate_training_events([event], reject)

    def test_simultaneous_insertions_use_plan_order_not_lexical_id(self):
        # leader sorts after ego, but was first in the departure file/queue.
        self.observer.departures = [(0., 0, "leader", ("a", "b")), (0., 1, "ego", ("a", "b"))]
        self.observer.departure_rank = {"leader": (0., 0), "ego": (0., 1)}
        self.step(0, {"ego": ("a", 0), "leader": ("a", 0)})
        self.assertEqual(self.observer.frozen["leader", 0]["shadow_road_flow"], 1)
        self.assertEqual(self.observer.frozen["ego", 0]["shadow_road_flow"], 2)

    def test_formal_shadow_flow_and_real_flow_audit_no_label_change(self):
        self.step(0, {"ego": ("a", 0)})
        event = SimpleNamespace(vehicle_id="ego", route_index=0, from_edge="a", speed_factor=1.,
                                sampled_time=0., edge_enter_time=0., lane_index=0, is_final_road=False,
                                road_length=100., road_flow=9, lane_flow=8, driving_time=8., low_speed_time=1.,
                                route_edges=("a", "b", "c"), turn_type=3, turn_direction="r",
                                incoming_turn_type=0, incoming_turn_direction="unknown")
        self.observer.attach(event)
        self.api.lane = SimpleNamespace(getLastStepVehicleNumber=lambda _: 8)
        self.api.vehicle.getLaneIndex = lambda _: 1
        self.observer.attach_observation(event, SimpleNamespace(lane_id="a_1"),
                                         SimpleNamespace(lane_lengths={"a_1": 99.}))
        labels = ["travel_time_label_gt_"+s for s in ("0p1", "0p5", "1", "1p5", "2", "3")]
        extras = dict(zip(labels, (12., 11., 10., 9., 8., 7.)))
        sweep = SimpleNamespace(DEFAULT_LABEL_SPEED_THRESHOLDS=(.1, .5, 1, 1.5, 2, 3),
                                candidate_label_columns=lambda _: labels, speed_bin_columns=lambda _: [],
                                _event_extra_values=lambda _: extras)
        base = SimpleNamespace(iter_valid_training_events=iter, training_event_end_time=lambda _: 15.)
        path = self.root / "rows.csv"
        write_feature_rows(path, [event], base=base, sweep=sweep, observer=self.observer, source={})
        with path.open() as stream:
            row = next(csv.DictReader(stream))
        with Path(str(path)+".audit.csv").open() as stream:
            audit = next(csv.DictReader(stream))
        self.assertEqual((row["road_flow"], row["lane_flow"]), ("1", "1"))
        self.assertEqual((audit["actual_road_flow"], audit["actual_lane_flow"]), ("9", "8"))
        self.assertEqual((audit["actual_lane_index"], audit["physical_label_road_length"]), ("1", "99.0"))
        self.assertEqual([float(row[label]) for label in labels], list(extras.values()))
        self.assertEqual(json.loads(Path(str(path)+".contract.json").read_text())["sampling"], SAMPLING)


if __name__ == "__main__":
    unittest.main()
