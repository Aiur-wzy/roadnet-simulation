import csv
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import tests.python.test_traci_cap_context as old_tests
import collection._shared.TraCI_Python_Cap_Context as shared
import collection._shared.TraCI_Python_Cap_Context_ABC as abc
from features.cap_context_features import FEATURE_SCHEMA, validate_context_contract
from features.cap_context_abc_features import ABC_AUDIT_FIELDS, ABC_FEATURE_SCHEMA


class ABCObserverTest(old_tests.TraCICapContextTest):
    """Apply the existing observer/label/metadata contract tests to the new ledger."""

    def setUp(self):
        super().setUp()
        self.observer = abc.ABCCapContextObserver(
            self.api, self.root / "config.xml", self.root / "net.xml")

    def test_a_b_c_from_causal_observer_snapshots(self):
        for decision, expected, subclass in ((10., 4, "B"), (20., 5, "C")):
            with self.subTest(decision=decision):
                observer = abc.ABCCapContextObserver(
                    self.api, self.root / "config.xml", self.root / "net.xml")
                self.api.step(0., {"leader": ("a", 0)})
                observer.on_step()
                first = observer.frozen[("leader", 0)]
                self.assertEqual((first["platoon_context"], first["context0_subclass"]), (0, "A"))
                self.api.step(decision, {"leader": ("a", 0), "ego": ("a", 0)})
                observer.on_step()
                values = observer.frozen[("ego", 0)]
                self.assertEqual((values["platoon_context"], values["context0_subclass"]), (expected, subclass))
                self.assertEqual((values["legacy_platoon_context"], values["P"], values["E"]), (0, 0., 0.))
                self.assertEqual(values["max_overdue_seconds"], max(0., decision - 94.9 / 5.))

    def test_audit_and_version_are_separate_from_formal_features(self):
        self.api.step(0, {"leader": ("a", 0)})
        self.observer.on_step()
        self.api.step(20, {"leader": ("a", 0), "ego": ("a", 0)})
        self.observer.on_step()
        event = SimpleNamespace(
            vehicle_id="ego", route_index=0, from_edge="a", speed_factor=1.,
            sampled_time=20., edge_enter_time=20., lane_index=0, is_final_road=False,
            road_length=100., road_flow=2, lane_flow=2, driving_time=8., low_speed_time=1.,
            route_edges=("a", "b", "c"), turn_type=3, turn_direction="r",
            incoming_turn_type=0, incoming_turn_direction="unknown")
        self.observer.attach(event)
        labels = ["travel_time_label_gt_" + t for t in ("0p1", "0p5", "1", "1p5", "2", "3")]
        extras = dict(zip(labels, (12., 11., 10., 9., 8., 7.)))
        extras.update(speed_bin_time_total=12., speed_bin_conservation_residual=0.)
        sweep = SimpleNamespace(
            DEFAULT_LABEL_SPEED_THRESHOLDS=(.1, .5, 1, 1.5, 2, 3),
            candidate_label_columns=lambda _: labels, speed_bin_columns=lambda _: [],
            _event_extra_values=lambda _: extras)
        base = SimpleNamespace(iter_valid_training_events=iter, training_event_end_time=lambda _: 40.)
        output = self.root / "abc.csv"
        shared.write_feature_rows(output, [event], base=base, sweep=sweep, observer=self.observer, source={})
        with output.open(newline="", encoding="utf-8") as stream:
            formal = next(csv.DictReader(stream))
        self.assertEqual(formal["platoon_context"], "5")
        self.assertEqual(float(formal["travel_time_label"]), 9.)
        self.assertEqual([float(formal[name]) for name in labels], [12., 11., 10., 9., 8., 7.])
        self.assertTrue(set(ABC_AUDIT_FIELDS).isdisjoint(formal))
        with Path(str(output) + ".audit.csv").open(newline="", encoding="utf-8") as stream:
            audit = next(csv.DictReader(stream))
        self.assertTrue(set(ABC_AUDIT_FIELDS).issubset(audit))
        self.assertTrue(all(audit[name] != "" for name in ABC_AUDIT_FIELDS))
        self.assertEqual(audit["legacy_platoon_context"], "0")
        contract = json.loads(Path(str(output) + ".contract.json").read_text(encoding="utf-8"))
        self.assertEqual(contract["schema"], ABC_FEATURE_SCHEMA)
        self.assertNotEqual(contract["schema"], FEATURE_SCHEMA)
        self.assertEqual(contract["context_counts"], {"0": 0, "1": 0, "2": 0, "3": 0, "4": 0, "5": 1})
        self.assertEqual(contract["data_sha256"], shared.file_sha256(output))
        # An ABC file must never masquerade as a valid old-model data contract.
        with self.assertRaises(ValueError):
            validate_context_contract(contract["context_contract"])

    def test_cli_routes_new_schema_and_threshold_without_mutating_old_globals(self):
        original_observer = shared.CapContextObserver
        with patch.object(shared, "main") as run:
            abc.main(["--context0-overdue-threshold", "2.5", "--sumo-config", "config.xml"])
        args, kwargs = run.call_args
        self.assertEqual(args, (["--sumo-config", "config.xml"],))
        self.assertEqual(kwargs["observer_factory"].keywords["overdue_threshold_seconds"], 2.5)
        self.assertEqual(kwargs["default_output_prefix"], "cap_context_abc")
        self.assertEqual(Path(kwargs["collector_path"]).name, "TraCI_Python_Cap_Context_ABC.py")
        self.assertIs(shared.CapContextObserver, original_observer)
        self.assertEqual(shared.FEATURE_SCHEMA, FEATURE_SCHEMA)

    def test_invalid_cli_threshold_rejected_before_sumo(self):
        for threshold in ("-1", "nan", "inf"):
            with self.subTest(threshold=threshold), patch.object(shared, "main") as run:
                with self.assertRaises(ValueError):
                    abc.main(["--context0-overdue-threshold", threshold])
                run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
