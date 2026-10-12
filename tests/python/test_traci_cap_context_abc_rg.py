import csv
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid
import xml.etree.ElementTree as ET

import collection._shared.TraCI_Python_Cap_Context as shared
import collection._shared.TraCI_Python_Cap_Context_ABC as abc
import collection.TraCI_Python_Cap_Context_ABC_RG as rg
from features.cap_context_features import ContextConfig, FEATURE_COLS, FEATURE_SCHEMA
from features.cap_context_abc_features import ABC_AUDIT_FIELDS, ABC_FEATURE_SCHEMA
from features.signal_rg_features import SIGNAL_FEATURE_COLS, SIGNAL_FEATURE_SCHEMA
from tests.python.test_traci_cap_context import FakeTraCI
from tests.python.test_traci_signal_rg import FakeTrafficLight, cleanup_test_directory, net_xml
from features.turn_context import TURN_CONTEXT_COLUMNS


class ABCRGObserverTest(unittest.TestCase):
    """Independent fixtures: no inherited old-five-feature schema assertions."""

    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".test_rg_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(cleanup_test_directory, self.root)
        (self.root / "route.xml").write_text(
            '<routes><vType id="car" length="5" maxSpeed="10"/>' + ''.join(
                f'<vehicle id="{vid}" type="car" departPos="base" speedFactor="{sf}"/>'
                for vid, sf in (("ego", 1.), ("leader", .5))) + '</routes>', encoding="utf-8")
        (self.root / "config.xml").write_text(
            '<configuration><input><route-files value="route.xml"/></input></configuration>', encoding="utf-8")
        (self.root / "net.xml").write_text(net_xml(), encoding="utf-8")
        self.api = FakeTraCI()
        self.api.trafficlight = FakeTrafficLight(lambda: self.api.time)
        self.observer = self.make_observer()

    def make_observer(self, observer_type=rg.ABCRGCapContextObserver, config=ContextConfig()):
        return observer_type(self.api, self.root / "config.xml", self.root / "net.xml", config)

    def test_formal_columns_are_original_five_plus_only_four_signals(self):
        self.assertEqual(list(self.observer.feature_columns), FEATURE_COLS + SIGNAL_FEATURE_COLS)
        self.assertEqual(len(self.observer.feature_columns), 9)
        self.assertEqual(self.observer.feature_schema, SIGNAL_FEATURE_SCHEMA)
        self.assertNotIn(self.observer.feature_schema, (FEATURE_SCHEMA, ABC_FEATURE_SCHEMA))
        self.assertTrue(set(ABC_AUDIT_FIELDS).isdisjoint(self.observer.feature_columns))
        self.assertEqual(shared.CapContextObserver.feature_columns, tuple(FEATURE_COLS))
        self.assertEqual(abc.ABCCapContextObserver.feature_columns, tuple(FEATURE_COLS))

    def test_signal_snapshot_uses_planned_entry_not_future_actual_entry(self):
        self.api.step(20., {"ego": ("a", 0)})
        self.observer.on_step()
        self.api.step(23., {"ego": (":ab", 0)})
        self.observer.on_step()
        frozen = self.observer.frozen[("ego", 1)].copy()
        self.assertEqual(frozen["decision_time"], 23.)
        self.assertEqual(frozen["planned_enter_time"], 26.)
        self.assertEqual(frozen["signal_reference_time"], 26.)
        self.assertEqual(frozen["signal_projection_seconds"], 3.)
        self.assertEqual(tuple(frozen[c] for c in SIGNAL_FEATURE_COLS), ("R", 1., "G", 15.))
        self.assertEqual(frozen["signal_link_indices"], "1")
        calls = self.observer.signals.clock_queries
        self.api.step(40., {"ego": ("b", 1)})
        self.observer.on_step()
        self.assertEqual(self.observer.frozen[("ego", 1)], frozen)
        self.assertEqual(self.observer.signals.clock_queries, calls)
        event = SimpleNamespace(vehicle_id="ego", route_index=1, from_edge="b", speed_factor=1., edge_enter_time=40.)
        self.observer.attach(event)
        self.assertEqual(event.cap_context_snapshot, frozen)

    def test_custom_crossing_changes_only_plan_projection_contract(self):
        self.observer = self.make_observer(config=ContextConfig(crossing_time_seconds=5.))
        self.api.step(20., {"ego": ("a", 0)})
        self.observer.on_step()
        self.api.step(23., {"ego": (":ab", 0)})
        self.observer.on_step()
        values = self.observer.frozen[("ego", 1)]
        self.assertEqual(values["planned_enter_time"], 28.)
        self.assertEqual(tuple(values[c] for c in SIGNAL_FEATURE_COLS), ("G", 14., "R", 72.))

    def test_unsignalized_incoming_zero_but_outgoing_clock_is_still_controlled(self):
        from tests.python.test_traci_signal_rg import DURATIONS, STATES
        net = ET.fromstring(net_xml())
        net.find("edge[@id='a']").set("to", "junction_ab")
        ET.SubElement(net, "junction", id="junction_ab", type="priority")
        connection = net.find("connection[@from='a']")
        del connection.attrib["tl"]
        del connection.attrib["linkIndex"]
        program = ET.SubElement(net, "tlLogic", id="T", programID="active", type="static", offset="0")
        for duration, state in zip(DURATIONS, STATES):
            ET.SubElement(program, "phase", duration=str(duration), state=state)
        ET.ElementTree(net).write(self.root / "net.xml", encoding="utf-8")
        self.observer = self.make_observer()
        self.api.step(20., {"ego": ("a", 0)})
        self.observer.on_step()
        self.api.step(23., {"ego": ("b", 1)})
        self.observer.on_step()
        values = self.observer.frozen[("ego", 1)]
        self.assertEqual(values["incoming_junction_status"], "unsignalized_node")
        self.assertEqual(values["incoming_crossing_seconds"], 0.)
        self.assertEqual(values["planned_enter_time"], 23.)
        self.assertEqual(values["signal_reference_time"], 23.)
        self.assertEqual(tuple(values[c] for c in SIGNAL_FEATURE_COLS), ("R", 4., "G", 15.))
        self.api.step(30., {"ego": ("c", 2)})
        self.observer.on_step()
        final = self.observer.frozen[("ego", 2)]
        self.assertEqual(final["incoming_crossing_seconds"], 3.)
        self.assertEqual(final["planned_enter_time"], 33.)
        self.assertEqual(final["signal_status"], "no_outgoing_route")

    def test_tls_node_without_connection_tls_is_not_zero_crossing(self):
        net = ET.fromstring(net_xml())
        net.find("edge[@id='a']").set("to", "junction_ab")
        ET.SubElement(net, "junction", id="junction_ab", type="traffic_light")
        connection = net.find("connection[@from='a']")
        del connection.attrib["tl"]
        del connection.attrib["linkIndex"]
        ET.ElementTree(net).write(self.root / "net.xml", encoding="utf-8")
        observer = self.make_observer()
        self.assertEqual(observer.entry_crossing_seconds(("a", "b", "c"), 1), 3.)
        for node_type in ("traffic_light_unregulated", "traffic_light_right_on_red", "custom", ""):
            net.find("junction").set("type", node_type)
            ET.ElementTree(net).write(self.root / "net.xml", encoding="utf-8")
            with self.subTest(node_type=node_type):
                self.assertEqual(self.make_observer().entry_crossing_seconds(("a", "b", "c"), 1), 3.)

    def test_abc_and_old_frozen_math_are_unchanged(self):
        original = self.make_observer(observer_type=abc.ABCCapContextObserver)
        for time, rows in ((0., {"leader": ("a", 0)}),
                           (20., {"leader": ("a", 0), "ego": ("a", 0)})):
            self.api.step(time, rows)
            original.on_step()
            self.observer.on_step()
        for key, values in original.frozen.items():
            self.assertEqual({name: self.observer.frozen[key][name] for name in values}, values)
        self.assertEqual(self.observer.frozen[("ego", 0)]["platoon_context"], 5)
        self.assertEqual(self.observer.ledger.scanned_records, original.ledger.scanned_records)

    def test_same_time_vehicles_share_readonly_tls_anchor(self):
        self.api.step(0., {"leader": ("a", 0), "ego": ("a", 0)})
        self.observer.on_step()
        self.assertEqual(self.observer.signals.clock_queries, 1)
        self.assertEqual(self.api.trafficlight.calls["getNextSwitch", "T"], 1)

    def test_final_road_is_collected_with_no_exit_sentinel(self):
        for time, rows in ((0., {"ego": ("a", 0)}), (1., {"ego": (":ab", 0)}),
                           (5., {"ego": ("b", 1)}), (6., {"ego": (":bc", 1)})):
            self.api.step(time, rows)
            self.observer.on_step()
        values = self.observer.frozen[("ego", 2)]
        self.assertEqual(tuple(values[c] for c in SIGNAL_FEATURE_COLS), ("G", 0., "G", 0.))
        self.assertEqual(values["signal_status"], "no_outgoing_route")

    def event_and_writer_fakes(self):
        self.api.step(20., {"ego": ("a", 0)})
        self.observer.on_step()
        event = SimpleNamespace(
            vehicle_id="ego", route_index=0, from_edge="a", speed_factor=1.,
            sampled_time=20., edge_enter_time=20., lane_index=0, is_final_road=False,
            road_length=100., road_flow=1, lane_flow=1, driving_time=8., low_speed_time=1.,
            route_edges=("a", "b", "c"), turn_type=3, turn_direction="r",
            incoming_turn_type=0, incoming_turn_direction="unknown")
        self.observer.attach(event)
        labels = ["travel_time_label_gt_" + token for token in ("0p1", "0p5", "1", "1p5", "2", "3")]
        extras = dict(zip(labels, (12., 11., 10., 9., 8., 7.)))
        extras.update(speed_time_le_0p1=3., speed_bin_time_total=15., speed_bin_conservation_residual=0.)
        sweep = SimpleNamespace(DEFAULT_LABEL_SPEED_THRESHOLDS=(.1, .5, 1., 1.5, 2., 3.),
                                candidate_label_columns=lambda _: labels,
                                speed_bin_columns=lambda _: ["speed_time_le_0p1"],
                                _event_extra_values=lambda _: extras)
        base = SimpleNamespace(iter_valid_training_events=iter, training_event_end_time=lambda _: 50.)
        return event, labels, base, sweep

    def test_writer_nine_features_separate_audit_and_unchanged_labels(self):
        event, labels, base, sweep = self.event_and_writer_fakes()
        path = self.root / "rg.csv"
        self.assertEqual(shared.write_feature_rows(path, [event], base=base, sweep=sweep,
                                                  observer=self.observer, source={"fixture": True}), 1)
        with path.open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            row = next(reader)
            self.assertEqual(reader.fieldnames, shared.IDENTITY_COLS + FEATURE_COLS + SIGNAL_FEATURE_COLS + ["travel_time_label"] + labels)
        self.assertEqual(float(row["travel_time_label"]), 9.)
        self.assertEqual([float(row[name]) for name in labels], [12., 11., 10., 9., 8., 7.])
        self.assertEqual(tuple(row[name] for name in SIGNAL_FEATURE_COLS), ("G", "4.0", "R", "63.0"))
        for field in (*ABC_AUDIT_FIELDS, *TURN_CONTEXT_COLUMNS, "P", "E", "decision_time",
                      "signal_status", "signal_yellow_skipped", "speed_factor", "speed_time_le_0p1"):
            self.assertNotIn(field, row)
        with Path(str(path) + ".audit.csv").open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            audit = next(reader)
            self.assertEqual(len(reader.fieldnames), len(set(reader.fieldnames)))
        for name in (*ABC_AUDIT_FIELDS, *SIGNAL_FEATURE_COLS, "signal_status", "signal_anchor_time"):
            self.assertNotEqual(audit[name], "", name)
        self.assertEqual(float(audit["speed_time_le_0p1"]), 3.)
        self.assertEqual(audit["signal_status"], "controlled_static")
        contract = json.loads(Path(str(path) + ".contract.json").read_text(encoding="utf-8"))
        self.assertEqual(contract["schema"], SIGNAL_FEATURE_SCHEMA)
        self.assertEqual(contract["raw_features"], FEATURE_COLS + SIGNAL_FEATURE_COLS)
        self.assertEqual(contract["context_contract"], self.observer.feature_context_contract())
        self.assertEqual(contract["signal_contract"]["feature_columns"], SIGNAL_FEATURE_COLS)
        self.assertEqual(contract["signal_contract"]["runtime_programs"]["T"]["program_id"], "active")
        self.assertEqual(contract["data_sha256"], shared.file_sha256(path))
        self.assertEqual(contract["audit"]["sha256"], shared.file_sha256(str(path) + ".audit.csv"))

    def test_writer_requires_every_extension_feature(self):
        event, _, base, sweep = self.event_and_writer_fakes()
        del event.cap_context_snapshot[SIGNAL_FEATURE_COLS[0]]
        with self.assertRaises(KeyError):
            shared.write_feature_rows(self.root / "missing.csv", [event], base=base, sweep=sweep,
                                      observer=self.observer, source={})

    def test_writer_rejects_duplicate_feature_and_identity_collision(self):
        event, _, base, sweep = self.event_and_writer_fakes()
        for extra in (FEATURE_COLS[0], "vehicleID", "travel_time_label"):
            self.observer.feature_columns = tuple(FEATURE_COLS) + (extra,)
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                shared.write_feature_rows(self.root / "bad.csv", [event], base=base, sweep=sweep,
                                          observer=self.observer, source={})

    def test_cli_delegates_without_running_sumo_or_mutating_old_schema(self):
        old_class = shared.CapContextObserver
        with patch.object(shared, "main") as run:
            rg.main(["--context0-overdue-threshold", "2.5", "--sumo-config", "fake.xml"])
        args, kwargs = run.call_args
        self.assertEqual(args, (["--sumo-config", "fake.xml"],))
        self.assertIs(kwargs["observer_factory"].func, rg.ABCRGCapContextObserver)
        self.assertEqual(kwargs["observer_factory"].keywords["overdue_threshold_seconds"], 2.5)
        self.assertEqual(kwargs["default_output_prefix"], "cap_context_abc_rg")
        self.assertEqual(Path(kwargs["collector_path"]).name, "TraCI_Python_Cap_Context_ABC_RG.py")
        self.assertEqual(Path(kwargs["extra_source_paths"]["signal_features"]).name, "signal_rg_features.py")
        self.assertIs(shared.CapContextObserver, old_class)
        self.assertEqual(shared.CapContextObserver.feature_schema, FEATURE_SCHEMA)

    def test_invalid_cli_threshold_rejected_before_delegating(self):
        for threshold in ("-1", "nan", "inf"):
            with self.subTest(threshold=threshold), patch.object(shared, "main") as run:
                with self.assertRaises(ValueError):
                    rg.main(["--context0-overdue-threshold", threshold])
                run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
