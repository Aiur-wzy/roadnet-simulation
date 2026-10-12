"""Low-flow sample-weight regressions; no real AutoGluon training is run."""

import contextlib
import io
import json
from pathlib import Path
import sys
import types
import unittest
from unittest import mock

import numpy as np
import pandas as pd

import training._shared.model_training_sumo_sf as training
import legacy.training.model_training_sumo_sf_lowflow as lowflow
import catching._shared.model_catching_sumo_sf as catching
import tests.python.test_model_training_sumo_sf as base_tests


class LowFlowWeightTests(unittest.TestCase):
    # Reuse only fixture helpers, not the base regression test methods.
    setUp = base_tests.TrainingContractTests.setUp
    cleanup_fixture = base_tests.TrainingContractTests.cleanup_fixture
    sample = base_tests.TrainingContractTests.sample
    write_csv = base_tests.TrainingContractTests.write_csv
    load = base_tests.TrainingContractTests.load
    args = base_tests.TrainingContractTests.args

    def weighted_args(self, *arguments):
        return lowflow.build_parser().parse_args([
            "--save-path", str(self.directory / "weighted-model-output"),
            *map(str, arguments),
        ])

    def weighted_frame(self, name, flows, base=10.0):
        raw = self.sample(base=base)
        raw["road_flow"] = flows
        raw["lane_flow"] = 1
        return self.load(self.write_csv(name, raw))

    def mock_fit(self, train_frame, validation_frame, args, predictions, framework_features=None):
        captured = {}

        class FakePredictor:
            def __init__(self, **kwargs):
                captured["constructor"] = kwargs
                self.feature_metadata = types.SimpleNamespace(
                    get_features=lambda: training.FEATURE_COLS.copy() if framework_features is None else framework_features)

            def fit(self, **kwargs):
                captured["fit"] = kwargs
                return self

            def predict(self, frame):
                captured["prediction_input"] = frame.copy()
                return np.asarray(predictions, dtype=float)

        package = types.ModuleType("autogluon")
        tabular = types.ModuleType("autogluon.tabular")
        tabular.TabularPredictor = FakePredictor
        with mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}), \
                mock.patch.object(training, "version", side_effect=lambda package: "1.6.1" if package == "autogluon.tabular" else "fixture"), \
                contextlib.redirect_stdout(io.StringIO()):
            training.fit_model(train_frame, validation_frame, args, {"split_kind": "fixture"})
        output = Path(args.save_path)
        captured["contract"] = json.loads((output / "training_contract.json").read_text(encoding="utf-8"))
        captured["metrics"] = json.loads((output / "validation_metrics.json").read_text(encoding="utf-8"))
        return captured

    def test_entrypoint_defaults_are_opt_in_and_five_features_stay_unchanged(self):
        self.assertEqual(self.args().sample_weighting, "none")
        weighted = self.weighted_args()
        self.assertEqual(weighted.sample_weighting, "low-flow")
        self.assertEqual(weighted.low_flow_cutoff, 5)
        self.assertEqual(weighted.low_flow_weight, 3.0)
        self.assertEqual(training.FEATURE_COLS, [
            "road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit",
        ])
        self.assertEqual(training.WEIGHT_COL, "__sample_weight")
        self.assertNotIn(training.WEIGHT_COL, training.FEATURE_COLS)

    def test_strict_four_five_six_boundary_ratio_and_mean_one(self):
        frame = pd.DataFrame({"road_flow": [4, 5, 6]}, index=[9, 3, 7])
        original = frame.copy(deep=True)
        weights, audit = training.make_sample_weights(frame, self.weighted_args())
        values = np.asarray(weights, dtype=float)
        np.testing.assert_allclose(values, [1.8, 0.6, 0.6], rtol=1e-12)
        self.assertAlmostEqual(values.mean(), 1.0)
        self.assertAlmostEqual(values[0] / values[1], 3.0)
        if isinstance(weights, pd.Series):
            self.assertEqual(weights.index.tolist(), frame.index.tolist())
        self.assertIsInstance(audit, dict)
        self.assertEqual(audit["operator"], "<")
        self.assertEqual(audit["rows"], 3)
        self.assertEqual(audit["low_flow_rows"], 1)
        self.assertEqual(audit["other_rows"], 2)
        self.assertAlmostEqual(audit["raw_weight_mean"], 5.0 / 3.0)
        self.assertAlmostEqual(audit["normalized_weight_sum"], 3.0)
        self.assertAlmostEqual(audit["low_flow_weight_fraction"], 3.0 / 5.0)
        self.assertFalse(audit["adds_predictive_feature"])
        self.assertFalse(audit["resampling"])
        pd.testing.assert_frame_equal(frame, original)

    def test_zero_flow_is_low_flow_and_current_vehicle_count_is_not_subtracted(self):
        frame = pd.DataFrame({"road_flow": [0, 1, 4, 5]})
        weights, _ = training.make_sample_weights(frame, self.weighted_args())
        np.testing.assert_allclose(weights, [1.2, 1.2, 1.2, 0.4])

    def test_all_low_or_all_other_rows_normalize_to_one(self):
        for flows in ([0, 3, 4], [5, 6, 20]):
            with self.subTest(flows=flows):
                weights, _ = training.make_sample_weights(
                    pd.DataFrame({"road_flow": flows}), self.weighted_args())
                np.testing.assert_allclose(weights, np.ones(3))

    def test_custom_cutoff_and_multiplier_are_applied(self):
        args = self.weighted_args("--low-flow-cutoff", 7, "--low-flow-weight", 2.5)
        weights, _ = training.make_sample_weights(pd.DataFrame({"road_flow": [6, 7]}), args)
        np.testing.assert_allclose(weights, np.array([2.5, 1.0]) / 1.75)

    def test_none_returns_no_weight_column_or_mutation(self):
        frame = pd.DataFrame({"road_flow": [4, 5, 6]})
        original = frame.copy(deep=True)
        weights, audit = training.make_sample_weights(frame, self.args())
        self.assertIsNone(weights)
        self.assertIsInstance(audit, dict)
        pd.testing.assert_frame_equal(frame, original)

    def test_invalid_weight_options_fail_before_data_loading(self):
        invalid = [
            ("--low-flow-cutoff", "-1"),
            ("--low-flow-cutoff", "0"),
            ("--low-flow-weight", "0"),
            ("--low-flow-weight", "0.5"),
            ("--low-flow-weight", "-1"),
            ("--low-flow-weight", "nan"),
            ("--low-flow-weight", "inf"),
            ("--low-flow-weight", "-inf"),
        ]
        for options in invalid:
            with self.subTest(options=options):
                # Equals form makes argparse treat negative infinity as a value.
                option = "{}={}".format(*options)
                with self.assertRaises(ValueError):
                    training.validate_options(self.weighted_args(option))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.weighted_args("--low-flow-cutoff", "4.5")
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            self.weighted_args("--sample-weighting", "unsupported")

    def test_weighting_does_not_depend_on_labels_sf_or_metadata(self):
        frame = pd.DataFrame({"road_flow": [4, 5, 6], "speed_factor": [0.7, 1.0, 1.3],
                              training.LABEL_COL: [1.0, 10.0, 100.0]})
        first, _ = training.make_sample_weights(frame, self.weighted_args())
        modified = frame.copy()
        modified[training.LABEL_COL] = [1000.0, 0.0, -1.0]
        modified["speed_factor"] = [1.3, 0.7, 1.0]
        second, _ = training.make_sample_weights(modified, self.weighted_args())
        np.testing.assert_array_equal(first, second)

    def test_mock_fit_receives_normalized_weights_and_weighted_internal_evaluation(self):
        train_frame = self.weighted_frame("weighted-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("weighted-validation.csv", [4, 5], base=100)
        captured = self.mock_fit(train_frame, validation_frame, self.weighted_args(), [110, 101])
        constructor = captured["constructor"]
        self.assertEqual(constructor["sample_weight"], training.WEIGHT_COL)
        self.assertIs(constructor["weight_evaluation"], True)
        self.assertEqual(constructor["eval_metric"], "mae")
        fit_data = captured["fit"]["train_data"]
        self.assertEqual(set(fit_data.columns), set(training.FEATURE_COLS + [training.LABEL_COL, training.WEIGHT_COL]))
        np.testing.assert_allclose(fit_data[training.WEIGHT_COL], [1.5, 0.5])
        self.assertEqual(fit_data[training.LABEL_COL].tolist(), [10.0, 11.0])
        self.assertNotIn("tuning_data", captured["fit"])
        self.assertNotIn("validation_data", captured["fit"])
        self.assertEqual(list(captured["prediction_input"].columns), training.FEATURE_COLS)
        self.assertNotIn(training.WEIGHT_COL, train_frame.columns)
        self.assertNotIn(training.WEIGHT_COL, validation_frame.columns)
        self.assertNotIn(training.WEIGHT_COL, captured["contract"]["raw_features"])
        self.assertNotIn(training.WEIGHT_COL, captured["contract"]["framework_features"])
        self.assertIn("NN_TORCH", captured["fit"]["excluded_model_types"])
        self.assertIn("FASTAI", captured["fit"]["excluded_model_types"])
        self.assertEqual(captured["contract"]["effective_excluded_model_types"], captured["fit"]["excluded_model_types"])

    def test_external_mae_and_bias_stay_unweighted(self):
        train_frame = self.weighted_frame("metric-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("metric-validation.csv", [4, 5], base=100)
        captured = self.mock_fit(train_frame, validation_frame, self.weighted_args(), [110, 101])
        # Absolute errors [10, 0]: ordinary MAE=5; weighted MAE would be 7.5.
        self.assertEqual(captured["metrics"]["overall"]["mae"], 5.0)
        self.assertEqual(captured["metrics"]["overall"]["bias"], 5.0)
        self.assertEqual(captured["metrics"]["overall"]["n"], 2)
        self.assertEqual(captured["metrics"]["by_low_flow"]["lt_5"]["mae"], 10.0)
        self.assertEqual(captured["metrics"]["by_low_flow"]["ge_5"]["mae"], 0.0)
        self.assertIn("unweighted", captured["metrics"]["metric_weighting"])

    def test_weighted_contract_stays_compatible_with_existing_sf_catch(self):
        train_frame = self.weighted_frame("catch-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("catch-validation.csv", [4, 5], base=100)
        args = self.weighted_args()
        self.mock_fit(train_frame, validation_frame, args, [100, 101])
        (Path(args.save_path) / "predictor").mkdir()
        (Path(args.save_path) / "predictor" / "predictor.pkl").write_bytes(b"mock contract test only")
        predictor_path, _, contract = catching.resolve_model(args.save_path, "0p1")
        self.assertEqual(predictor_path, Path(args.save_path).resolve() / "predictor")
        self.assertEqual(contract["raw_features"], training.FEATURE_COLS)
        self.assertEqual(contract["sample_weighting"]["operator"], "<")

    def test_weight_statistics_are_derived_from_training_only(self):
        train_frame = self.weighted_frame("audit-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("audit-validation.csv", [6, 7], base=100)
        original = training.make_sample_weights
        calls = []

        def record(frame, args):
            calls.append(frame["road_flow"].tolist())
            return original(frame, args)

        with mock.patch.object(training, "make_sample_weights", side_effect=record):
            captured = self.mock_fit(train_frame, validation_frame, self.weighted_args(), [100, 101])
        self.assertTrue(calls)
        self.assertTrue(all(flows == [4, 5] for flows in calls))
        np.testing.assert_allclose(captured["fit"]["train_data"][training.WEIGHT_COL], [1.5, 0.5])
        audit = captured["contract"]["sample_weighting"]
        self.assertEqual(audit["normalization_source"], "training split only")
        self.assertEqual(audit["rows"], 2)
        self.assertEqual(audit["low_flow_rows"], 1)
        self.assertEqual(audit["other_rows"], 1)
        self.assertEqual(audit["raw_weight_mean"], 2.0)
        self.assertEqual(audit["normalized_low_flow_weight"], 1.5)
        self.assertEqual(audit["normalized_other_weight"], 0.5)

    def test_original_entrypoint_none_does_not_pass_sample_weight(self):
        train_frame = self.weighted_frame("none-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("none-validation.csv", [4, 5], base=100)
        captured = self.mock_fit(train_frame, validation_frame, self.args(), [100, 101])
        self.assertIsNone(captured["constructor"].get("sample_weight"))
        self.assertFalse(captured["constructor"].get("weight_evaluation", False))
        self.assertEqual(list(captured["fit"]["train_data"].columns), training.FEATURE_COLS + [training.LABEL_COL])
        self.assertEqual(captured["fit"]["excluded_model_types"], ["NN_TORCH"])
        self.assertEqual(captured["contract"]["sample_weighting"]["mode"], "none")

    def test_weights_are_computed_after_selected_label_filtering(self):
        raw = self.sample(base=10)
        raw["road_flow"] = [4, 5]
        raw[training.LABEL_COLUMNS[0]] = [0, 11]
        frame = self.load(self.write_csv("post-filter.csv", raw))
        weights, audit = training.make_sample_weights(frame, self.weighted_args())
        np.testing.assert_allclose(weights, [1.0])
        self.assertEqual(audit["rows"], 1)
        self.assertEqual(audit["low_flow_rows"], 0)

    def test_empty_and_invalid_flow_weight_inputs_are_rejected(self):
        for flows in ([], [-1], [1.5], [float("nan")], [float("inf")]):
            with self.subTest(flows=flows), self.assertRaises(ValueError):
                training.make_sample_weights(pd.DataFrame({"road_flow": flows}), self.weighted_args())

    def test_validation_flow_groups_respect_all_boundaries(self):
        frame = pd.DataFrame({
            "road_flow": [4, 5, 10, 11, 20, 21],
            training.LABEL_COL: [10] * 6,
            "is_final_road": [0, 0, 0, 1, 1, 1],
            "__source_file": ["one"] * 6,
        })
        result = training.validation_metrics(frame, [11, 12, 13, 14, 15, 16])
        self.assertEqual({key: value["n"] for key, value in result["by_road_flow"].items()},
                         {"lt_5": 1, "5_to_10": 2, "11_to_20": 2, "gt_20": 1})
        self.assertEqual(result["by_low_flow"]["lt_5"]["n"], 1)
        self.assertEqual(result["by_low_flow"]["ge_5"]["n"], 5)
        self.assertEqual(result["overall"]["mae"], 3.5)

    def test_non_audited_autogluon_versions_fail_without_output_or_model_creation(self):
        train_frame = self.weighted_frame("version-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("version-validation.csv", [4, 5], base=100)
        package = types.ModuleType("autogluon")
        tabular = types.ModuleType("autogluon.tabular")
        tabular.TabularPredictor = mock.Mock(side_effect=AssertionError("must not construct model"))
        for installed in ("1.5.0", "1.7.0", None):
            with self.subTest(installed=installed):
                args = self.weighted_args()
                with mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}), \
                        mock.patch.object(training, "version", return_value=installed), \
                        self.assertRaisesRegex(ValueError, "Weighted training is audited"):
                    training.fit_model(train_frame, validation_frame, args, {})
                self.assertFalse(Path(args.save_path).exists())
        tabular.TabularPredictor.assert_not_called()

    def test_weight_leaking_into_framework_features_fails_loudly(self):
        train_frame = self.weighted_frame("leak-train.csv", [4, 5], base=10)
        validation_frame = self.weighted_frame("leak-validation.csv", [4, 5], base=100)
        with self.assertRaisesRegex(ValueError, "Unexpected model inputs"):
            self.mock_fit(train_frame, validation_frame, self.weighted_args(), [100, 101],
                          framework_features=training.FEATURE_COLS + [training.WEIGHT_COL])

    def test_weighted_dry_run_never_trains_imports_autogluon_or_creates_outputs(self):
        train_frame = self.sample(base=10)
        train_frame["road_flow"] = [4, 5]
        validation_frame = self.sample(base=100)
        validation_frame["road_flow"] = [6, 7]
        one = self.write_csv("dry-weighted-train.csv", train_frame)
        two = self.write_csv("dry-weighted-validation.csv", validation_frame)
        output = self.directory / "never-created" / "model"
        before = {str(path.relative_to(self.directory)) for path in self.directory.rglob("*")}
        with mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}), \
                mock.patch.object(training, "fit_model", side_effect=AssertionError("must not train")) as fit, \
                mock.patch.object(training, "make_sample_weights", wraps=training.make_sample_weights) as weighting, \
                contextlib.redirect_stdout(io.StringIO()) as stdout:
            lowflow.main(["--csv", str(one), "--validation-csv", str(two),
                          "--save-path", str(output), "--dry-run"])
        self.assertGreaterEqual(weighting.call_count, 1)
        for call in weighting.call_args_list:
            self.assertEqual(call.args[0]["road_flow"].tolist(), [4, 5])
        fit.assert_not_called()
        self.assertIn("[DRY-RUN] Passed", stdout.getvalue())
        self.assertFalse(output.exists())
        after = {str(path.relative_to(self.directory)) for path in self.directory.rglob("*")}
        self.assertEqual(before, after)


if __name__ == "__main__":
    unittest.main()
