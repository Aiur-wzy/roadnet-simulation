"""Constant G,H,G,H contract, training and mocked export regression tests."""
import copy
import json
from pathlib import Path
import shutil
import unittest
import uuid

from features.signal_rg_features import always_green_features, longest_finite_green_seconds, network_green_horizon
import catching.model_catching_sumo_cap_context_abc_rg as catch
import training.model_training_sumo_cap_context_abc_rg as training
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, fixture_args, mock_predictor
from tests.python.test_model_training_sumo_cap_context_abc_rg import ABCRGTrainingTests


class UnsignalizedPipelineTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".test_constant_green_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        resolved = self.root.resolve()
        if resolved.parent != Path(__file__).resolve().parent or not resolved.name.startswith(".test_constant_green_"):
            raise RuntimeError("unsafe cleanup target")
        shutil.rmtree(resolved)

    def test_green_run_not_tls_phase_or_cycle(self):
        self.assertEqual(longest_finite_green_seconds([5., 3., 7., 11.], "GrgG"), 23.)
        self.assertEqual(longest_finite_green_seconds([5., 10., 3.], "Ggr"), 15.)
        self.assertEqual(longest_finite_green_seconds([5., 3., 10., 7.], "GyGr"), 10.)
        self.assertEqual(longest_finite_green_seconds([5., 10.], "GG"), 0.)
        self.assertEqual(longest_finite_green_seconds([5., 10.], "rr"), 0.)
        self.assertEqual(always_green_features(23.)["next_signal_duration_seconds"], 23.)
        for invalid in (0., -1., float("inf"), True):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                always_green_features(invalid)

    def unsignalized_network(self):
        _, net, _ = prepare_fixture(self.root)
        net.write_text(net.read_text().replace('</net>', '<connection from="b" to="a" fromLane="0" toLane="0"/></net>'))
        return net

    def test_catalog_includes_constant_gg_and_exact_nonstep_endpoint(self):
        net = self.unsignalized_network()
        horizon, _ = network_green_horizon(net)
        self.assertEqual(horizon, 6.)
        for step in (1., 5., 20.):
            catalog = catch.signal_catalog(net, step)
            _, values = catch.quantize_signal(catalog, "G", 6., "G", 6.)
            self.assertEqual(values, ("G", 6., "G", 6.))
            _, terminal = catch.quantize_signal(catalog, "G", 0., "G", 0.)
            self.assertEqual(terminal, ("G", 0., "G", 0.))
        with self.assertRaisesRegex(ValueError, "Legacy model"):
            catch.signal_catalog(net, 5., legacy=True)

    def test_old_contract_remains_loadable_not_silently_upgraded(self):
        old = training.expected_signal_contract(legacy=True)
        self.assertEqual(catch.validated_signal_contract(old), old)
        self.assertNotEqual(old, training.expected_signal_contract())
        self.unsignalized_network()
        contract_path = self.root / "model/training_contract.json"
        contract = json.loads(contract_path.read_text())
        contract["signal_contract"] = old
        contract_path.write_text(json.dumps(contract))
        with self.assertRaisesRegex(ValueError, "Legacy model"):
            catch.build_plan(fixture_args(self.root))

    def test_v2_is_readable_but_rejected_for_new_uncontrolled_clock(self):
        old = training.expected_signal_contract(version="static-movement-rg-segments-v2")
        self.assertEqual(catch.validated_signal_contract(old), old)
        self.unsignalized_network()
        contract_path = self.root / "model/training_contract.json"
        contract = json.loads(contract_path.read_text())
        contract["signal_contract"] = old
        contract_path.write_text(json.dumps(contract))
        with self.assertRaisesRegex(ValueError, "Legacy model"):
            catch.build_plan(fixture_args(self.root))

    def test_training_zero_incoming_is_not_outgoing_unsignalized_flag(self):
        helper = ABCRGTrainingTests()
        helper.root = self.root
        def update(audit):
            # Row 1 is terminal, but was reached over an uncontrolled entrance.
            audit.loc[1, "incoming_junction_status"] = "unsignalized_node"
            audit.loc[1, "incoming_crossing_seconds"] = 0.
            audit.loc[1, "planned_enter_time"] = audit.loc[1, "decision_time"]
            audit.loc[1, "signal_reference_time"] = audit.loc[1, "decision_time"]
            audit.loc[1, "signal_projection_seconds"] = 0.
        self.assertEqual(len(helper.load(helper.write("zero.csv", audit_update=update))), 6)
        def bad(audit):
            update(audit)
            audit.loc[1, "incoming_crossing_seconds"] = 3.
        with self.assertRaisesRegex(ValueError, "incoming crossing"):
            helper.load(helper.write("badzero.csv", audit_update=bad))

    def test_mocked_export_roundtrips_gg_queries(self):
        self.unsignalized_network()
        args = fixture_args(self.root, "--signal-remaining-step", "5", "--max-road-flow", "2",
                            "--max-lane-flow", "1", "--cap-min", "8", "--cap-max", "8")
        plan = catch.build_plan(args)
        catch.export_table(args, plan, mock_predictor())
        header, values = catch.read_table(Path(args.output_dir) / "table.rle")
        self.assertEqual(header["signal_contract"]["version"], "static-movement-rg-segments-v3")
        self.assertEqual(len(values), plan["expanded_rows"])
        self.assertEqual(catch.quantize_signal(header["signal_catalog"], "G", 6., "G", 6.)[1], ("G", 6., "G", 6.))

    def training_fixture(self):
        helper = ABCRGTrainingTests()
        helper.root = self.root
        raw = helper.sample()
        raw.loc[0, training.SIGNAL_FEATURE_COLS] = ["G", 24., "G", 24.]
        signal = helper.signal_contract()
        signal["runtime_unsignalized_green_seconds"] = 24.
        signal["runtime_green_horizon_movements"] = {"T": [0]}
        def audit_update(frame):
            frame.loc[0, ["signal_status", "signal_tls_id", "signal_program_id"]] = ["unsignalized_connection", "", ""]
        return helper, raw, signal, audit_update

    def test_training_accepts_audited_constant_gg_without_new_feature(self):
        helper, raw, signal, audit_update = self.training_fixture()
        frame = helper.load(helper.write("train.csv", raw, signal=signal, audit_update=audit_update))
        self.assertEqual(frame.loc[0, "entry_signal_remaining_seconds"], 24.)
        captured = helper.fake_fit(frame, frame.copy())
        self.assertEqual(captured["contract"]["signal_contract"]["version"], "static-movement-rg-segments-v3")
        self.assertEqual(len(captured["contract"]["raw_features"]), 9)

    def test_training_rejects_wrong_or_missing_horizon(self):
        helper, raw, signal, audit_update = self.training_fixture()
        for index, value in enumerate((23., 0.)):
            invalid = raw.copy()
            invalid.loc[0, ["entry_signal_remaining_seconds", "next_signal_duration_seconds"]] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                helper.load(helper.write(f"bad{index}.csv", invalid, signal=signal, audit_update=audit_update))
        broken = copy.deepcopy(signal)
        broken["runtime_unsignalized_green_seconds"] = 25.
        with self.assertRaisesRegex(ValueError, "green horizon disagrees"):
            training.validate_signal_contract(broken)


if __name__ == "__main__":
    unittest.main()
