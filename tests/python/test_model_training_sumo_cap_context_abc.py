"""New cap/context trainer tests; no real AutoGluon, GPU or production data writes."""

import contextlib
import copy
import csv
import io
import json
from pathlib import Path
import shutil
import sys
import types
import unittest
from unittest import mock
import uuid

import numpy as np
import pandas as pd

import features.cap_context_abc_features as features
import features.cap_context_features as base_features
import training._shared.model_training_sumo_cap_context_abc as training
import training._shared.model_training_sumo_sf as legacy


DEFAULT_CONTEXT = features.abc_context_contract()


def context_with_base(**changes):
    result = copy.deepcopy(DEFAULT_CONTEXT)
    result["base_context_contract"].update(changes)
    return result


class ABCCapContextTrainingTests(unittest.TestCase):
    def setUp(self):
        self.root = Path.cwd() / ("abc-cap-context-training-test-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup_fixture)

    def cleanup_fixture(self):
        target = self.root.resolve()
        if target.parent != Path.cwd().resolve() or not target.name.startswith("abc-cap-context-training-test-"):
            raise RuntimeError("Refusing cleanup outside the test workspace")
        shutil.rmtree(target)

    def sample(self, base=10.0):
        frame = pd.DataFrame({
            "road_length": [100.0, 200.0, 100.0, 200.0],
            "road_flow": [1, 4, 5, 6], "lane_flow": [1, 2, 2, 3],
            "free_speed_cap": [10.123456789012345, 11.0, 12.0, 13.0],
            "platoon_context": [0, 1, 2, 3],
            "vehicleID": ["v0", "v0", "v1", "v1"],
            "route_index": [0, 1, 0, 1], "is_final_road": [0, 1, 0, 1],
            "time": [0, 100, 1, 101], "roadID": ["r0", "r1", "r0", "r1"],
            "laneIndex": [0, 0, 0, 0],
        })
        for index, label in enumerate(training.LABEL_COLUMNS):
            frame[label] = np.arange(base + index, base + index + 4)
        return frame

    def write(self, name, frame=None, context=None, metadata_updates=None, sidecar=True):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        (self.sample() if frame is None else frame).to_csv(path, index=False)
        if sidecar:
            contract = {
                "schema": training.SCHEMA, "raw_features": list(training.FEATURE_COLS),
                "context_contract": copy.deepcopy(DEFAULT_CONTEXT if context is None else context),
                "data_sha256": training.file_sha256(path),
                "source": {"path": "fixture-only", "note": "Not a SUMO or real predictor result"},
            }
            contract.update(metadata_updates or {})
            Path(str(path) + ".contract.json").write_text(json.dumps(contract), encoding="utf-8")
        return path

    def args(self, *arguments):
        return training.build_parser().parse_args([
            "--save-path", str(self.root / "model"), "--sample-weighting", "none",
            "--label-column", training.LABEL_COLUMNS[0], *map(str, arguments),
        ])

    def load(self, path, label=None):
        with contextlib.redirect_stdout(io.StringIO()):
            return training.load_frame(path, label or training.LABEL_COLUMNS[0])

    def loaded_pair(self):
        return self.load(self.write("train.csv")), self.load(self.write("validation.csv", self.sample(100)))

    def fake_fit(self, train, validation, args, used=None, predictions=None, fail_fit=False, version="1.6.1"):
        captured = {}

        class FakePredictor:
            def __init__(self, **kwargs):
                captured["constructor"] = kwargs
                self.feature_metadata = types.SimpleNamespace(get_features=lambda: list(training.FEATURE_COLS) if used is None else used)

            def fit(self, **kwargs):
                captured["fit"] = kwargs
                if fail_fit:
                    raise RuntimeError("fixture fit failure")
                return self

            def predict(self, frame):
                captured["predict"] = frame.copy()
                return np.asarray(predictions if predictions is not None else validation[training.LABEL_COL], dtype=float)

        package = types.ModuleType("autogluon")
        tabular = types.ModuleType("autogluon.tabular")
        tabular.TabularPredictor = FakePredictor
        with mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}), \
                mock.patch.object(training, "version", return_value=version), contextlib.redirect_stdout(io.StringIO()):
            training.fit_model(train, validation, args, {"split_kind": "fixture", "collection_source": "test"})
        captured["contract"] = json.loads((Path(args.save_path) / "training_contract.json").read_text(encoding="utf-8"))
        captured["metrics"] = json.loads((Path(args.save_path) / "validation_metrics.json").read_text(encoding="utf-8"))
        return captured

    def test_raw_five_and_legacy_globals_unchanged(self):
        self.assertEqual(training.SCHEMA, "road-time-cap-context-abc-v1")
        self.assertEqual(training.FEATURE_COLS, ["road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context"])
        self.assertEqual(legacy.FEATURE_COLS, ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"])
        frame = self.load(self.write("one.csv"))
        self.assertEqual(list(frame.columns[:5]), training.FEATURE_COLS)
        self.assertEqual(frame["free_speed_cap"].iloc[0], 10.123456789012345)
        self.assertEqual(frame.attrs["context_contract"], DEFAULT_CONTEXT)

    def test_category_type_and_all_levels_remain_when_sample_has_only_one(self):
        raw = self.sample()
        raw["platoon_context"] = 2
        frame = self.load(self.write("one-category.csv", raw))
        self.assertIsInstance(frame["platoon_context"].dtype, pd.CategoricalDtype)
        self.assertEqual(frame["platoon_context"].cat.categories.tolist(), [0, 1, 2, 3, 4, 5])
        self.assertFalse(frame["platoon_context"].cat.ordered)
        summary = training.feature_summary(frame)["platoon_context"]
        self.assertEqual(summary["observed_categories"], [2])
        self.assertEqual(summary["counts"], {"0": 0, "1": 0, "2": 4, "3": 0, "4": 0, "5": 0})

    def test_formal_encoder_drops_every_audit_and_source_feature(self):
        raw = self.sample()
        for column in ("speed_factor", "speed_limit", "P", "E", "dmax", "decision_time", "candidate_count", "__sample_weight",
                       "legacy_platoon_context", "context0_subclass", "max_overdue_seconds", "sum_overdue_seconds",
                       "overdue_count", "overdue_fraction", "mean_overdue_seconds", "context0_overdue_threshold_seconds"):
            raw[column] = 123.0
        encoded = training.encode_feature_frame(raw)
        self.assertEqual(list(encoded.columns), training.FEATURE_COLS)
        self.assertNotIn(training.LABEL_COL, encoded.columns)

    def test_each_six_label_and_terminal_row_survive(self):
        path = self.write("all-labels.csv")
        for index, label in enumerate(training.LABEL_COLUMNS):
            with self.subTest(label=label):
                frame = self.load(path, label)
                self.assertEqual(frame[training.LABEL_COL].tolist(), [10 + index, 11 + index, 12 + index, 13 + index])
                self.assertEqual(frame.attrs["audit"]["final_rows_retained"], 2)

    def test_only_selected_nonpositive_labels_are_removed_before_weighting(self):
        raw = self.sample()
        raw[training.LABEL_COLUMNS[0]] = [0, 11, 12, 13]
        raw[training.LABEL_COLUMNS[1]] = [-1, -1, -1, -1]
        frame = self.load(self.write("filter.csv", raw))
        self.assertEqual(len(frame), 3)
        self.assertEqual(frame.attrs["audit"]["nonpositive_label_rows_removed"], 1)
        weights, audit = training.make_sample_weights(frame, self.args("--sample-weighting", "low-flow"))
        np.testing.assert_allclose(weights, [1.8, .6, .6])
        self.assertEqual(audit["low_flow_rows"], 1)

    def test_missing_new_feature_or_identity_fails_instead_of_guessing(self):
        for column in training.FEATURE_COLS + training.META_COLS + [training.LABEL_COLUMNS[0]]:
            with self.subTest(column=column):
                raw = self.sample().drop(columns=column)
                raw["speed_factor"], raw["speed_limit"] = 1.0, 13.89
                with self.assertRaisesRegex(ValueError, "missing cap/context full-training columns"):
                    self.load(self.write("missing.csv", raw))

    def test_missing_contract_rejected_even_when_columns_exist(self):
        with self.assertRaisesRegex(ValueError, "Missing collection contract"):
            self.load(self.write("no-contract.csv", sidecar=False))

    def test_stale_hash_wrong_schema_and_wrong_raw_order_rejected(self):
        for changes, message in (({"data_sha256": "stale"}, "data_sha256"),
                                 ({"schema": "old-sf"}, "schema/raw_features"),
                                 ({"raw_features": list(reversed(training.FEATURE_COLS))}, "schema/raw_features")):
            with self.subTest(changes=changes), self.assertRaisesRegex(ValueError, message):
                self.load(self.write("wrong-sidecar.csv", metadata_updates=changes))

    def test_bad_geometry_event_version_and_invalid_thresholds_rejected(self):
        for changes in ({"geometry": "full-road"}, {"sampling": "real-edge-entry"},
                        {"p_threshold": 0}, {"e_threshold": .5}, {"crossing_time_seconds": -1},
                        {"epsilon_seconds": float("nan")}, {"unexpected": "key"}):
            with self.subTest(changes=changes), self.assertRaises((ValueError, TypeError)):
                context = context_with_base(**changes)
                self.load(self.write("wrong-context.csv", context=context))

    def test_valid_custom_crossing_is_preserved_not_replaced_with_default(self):
        context = context_with_base(crossing_time_seconds=2.5)
        frame = self.load(self.write("custom.csv", context=context))
        self.assertEqual(frame.attrs["context_contract"], context)

    def test_v1_threshold_change_requires_new_version(self):
        for key, value in (("p_threshold", .1), ("e_threshold", 3), ("epsilon_seconds", 0)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.load(self.write("changed-threshold.csv", context=context_with_base(**{key: value})))

    def test_mixed_contract_within_and_between_splits_fails(self):
        first = self.write("a.csv")
        second = self.write("b.csv", self.sample(100), context=context_with_base(crossing_time_seconds=4.0))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "Mixed collection"):
            training.load_split([first, second], training.LABEL_COLUMNS[0])
        with self.assertRaisesRegex(ValueError, "Training/validation context_contract mismatch"):
            training.shared_context_contract(self.load(first), self.load(second))

    def test_cap_can_change_along_one_vehicle_route(self):
        frame = self.load(self.write("different-road-cap.csv"))
        self.assertNotEqual(frame.loc[0, "free_speed_cap"], frame.loc[1, "free_speed_cap"])

    def test_invalid_feature_values_fail_without_fallback(self):
        for column, values in (
                ("free_speed_cap", [0, -1, float("inf"), float("nan"), "bad"]),
                ("road_length", [0, -1]),
                ("platoon_context", [-1, 6, .5, float("nan"), float("inf"), "bad"]),
                ("road_flow", [-1, .5, float("nan")]),
                ("lane_flow", [-1, .5, float("inf")])):
            for value in values:
                with self.subTest(column=column, value=value), self.assertRaises((ValueError, TypeError)):
                    raw = self.sample()
                    raw[column] = value
                    self.load(self.write("invalid.csv", raw))

    def test_boolean_is_not_a_numeric_feature_or_label(self):
        for column in training.FEATURE_COLS + [training.LABEL_COLUMNS[0], "route_index", "is_final_road"]:
            with self.subTest(column=column), self.assertRaisesRegex(ValueError, "not boolean"):
                raw = self.sample()
                raw[column] = True
                self.load(self.write("boolean.csv", raw))

    def test_identity_and_lane_occupancy_validation(self):
        for column, values in (("vehicleID", ["", "v0", "v1", "v1"]),
                               ("route_index", [0, .5, 0, 1]), ("is_final_road", [0, 2, 0, 1]),
                               ("lane_flow", [2, 2, 2, 3]), ("route_index", [0, 0, 0, 1])):
            with self.subTest(column=column, values=values), self.assertRaises(ValueError):
                raw = self.sample()
                raw[column] = values
                self.load(self.write("bad-identity.csv", raw))

    def test_split_concat_preserves_categories_and_episode_local_vehicle_ids(self):
        first, second = self.write("episode-a.csv"), self.write("episode-b.csv", self.sample(30))
        with contextlib.redirect_stdout(io.StringIO()):
            frame, audits = training.load_split([first, second], training.LABEL_COLUMNS[0])
        self.assertEqual(len(frame), 8)
        self.assertEqual(len(audits), 2)
        self.assertEqual(frame["platoon_context"].cat.categories.tolist(), [0, 1, 2, 3, 4, 5])

    def test_mock_fit_predict_receive_same_five_dtypes_without_audit(self):
        train, validation = self.loaded_pair()
        for frame in (train, validation):
            for column in ("speed_factor", "speed_limit", "P", "E", "decision_time", "candidate_count"):
                frame[column] = 999
        captured = self.fake_fit(train, validation, self.args())
        self.assertEqual(list(captured["fit"]["train_data"].columns), training.FEATURE_COLS + [training.LABEL_COL])
        self.assertEqual(list(captured["predict"].columns), training.FEATURE_COLS)
        self.assertEqual(captured["fit"]["train_data"]["platoon_context"].dtype, captured["predict"]["platoon_context"].dtype)
        self.assertEqual(captured["predict"]["platoon_context"].cat.categories.tolist(), [0, 1, 2, 3, 4, 5])
        self.assertNotIn("tuning_data", captured["fit"])
        self.assertNotIn("validation_data", captured["fit"])
        self.assertEqual(captured["fit"]["train_data"][training.LABEL_COL].tolist(), [10, 11, 12, 13])
        contract = captured["contract"]
        self.assertEqual(contract["status"], "complete")
        self.assertEqual(contract["raw_features"], training.FEATURE_COLS)
        self.assertEqual(contract["context_contract"], DEFAULT_CONTEXT)
        self.assertEqual(contract["categorical_features"], {"platoon_context": [0, 1, 2, 3, 4, 5]})
        self.assertEqual(contract["schema"], "road-time-cap-context-abc-v1")
        self.assertEqual(captured["metrics"]["overall"]["mae"], 0)

    def test_missing_training_classes_keep_six_category_dtype_in_fit_and_predict(self):
        raw_train, raw_validation = self.sample(), self.sample(100)
        raw_train["platoon_context"] = 0
        raw_validation["platoon_context"] = 3
        raw_train["road_length"] = 100  # Infer int from CSV, then normalize to float64.
        train = self.load(self.write("missing-class-train.csv", raw_train))
        validation = self.load(self.write("missing-class-val.csv", raw_validation))
        captured = self.fake_fit(train, validation, self.args())
        train_input = captured["fit"]["train_data"]
        val_input = captured["predict"]
        for frame in (train_input, val_input):
            self.assertEqual(frame["platoon_context"].cat.categories.tolist(), [0, 1, 2, 3, 4, 5])
            self.assertEqual(str(frame["road_length"].dtype), "float64")
            self.assertEqual(str(frame["free_speed_cap"].dtype), "float64")
            self.assertEqual(str(frame["road_flow"].dtype), "int64")
            self.assertEqual(str(frame["lane_flow"].dtype), "int64")
        self.assertEqual(captured["contract"]["training_features"]["platoon_context"]["observed_categories"], [0])
        self.assertEqual(captured["contract"]["validation_features"]["platoon_context"]["observed_categories"], [3])

    def test_custom_crossing_agrees_between_collection_model_contract(self):
        context = context_with_base(crossing_time_seconds=2.5)
        train = self.load(self.write("crossing-train.csv", context=context))
        validation = self.load(self.write("crossing-val.csv", self.sample(100), context=context))
        captured = self.fake_fit(train, validation, self.args())
        self.assertEqual(captured["contract"]["context_contract"]["base_context_contract"]["crossing_time_seconds"], 2.5)

    def test_framework_can_prune_constants_but_cannot_add_source_features(self):
        train, validation = self.loaded_pair()
        captured = self.fake_fit(train, validation, self.args(), used=["road_flow", "lane_flow", "free_speed_cap", "platoon_context"])
        self.assertEqual(captured["contract"]["raw_features"], training.FEATURE_COLS)
        self.assertNotIn("road_length", captured["contract"]["framework_features"])
        with self.assertRaisesRegex(ValueError, "Unexpected model inputs"):
            self.fake_fit(train, validation, self.args("--save-path", self.root / "leak"), used=training.FEATURE_COLS + ["speed_factor"])
        failed = json.loads((self.root / "leak" / "training_contract.json").read_text())
        self.assertEqual(failed["status"], "failed")

    def test_completed_abc_contract_is_rejected_by_current_four_state_catch(self):
        import legacy.catching.model_catching_sumo_cap_context as catching

        train, validation = self.loaded_pair()
        args = self.args()
        self.fake_fit(train, validation, args)
        predictor = Path(args.save_path) / "predictor"
        predictor.mkdir()
        (predictor / "predictor.pkl").write_bytes(b"FAKE fixture only: never load this as a model")
        with self.assertRaisesRegex(ValueError, "schema"):
            catching.resolve_model(args.save_path, "0p1")

    def test_fit_failure_is_not_marked_complete(self):
        train, validation = self.loaded_pair()
        with self.assertRaisesRegex(RuntimeError, "fixture fit failure"):
            self.fake_fit(train, validation, self.args(), fail_fit=True)
        contract = json.loads((self.root / "model" / "training_contract.json").read_text())
        self.assertEqual(contract["status"], "failed")
        self.assertNotIn("framework_features", contract)

    def test_weighted_fit_is_strict_three_to_one_and_external_metrics_unweighted(self):
        train, validation = self.loaded_pair()
        captured = self.fake_fit(train, validation, self.args("--sample-weighting", "low-flow"), predictions=[110, 101, 102, 103])
        np.testing.assert_allclose(captured["fit"]["train_data"][training.WEIGHT_COL], [1.5, 1.5, .5, .5])
        self.assertEqual(captured["constructor"]["sample_weight"], training.WEIGHT_COL)
        self.assertTrue(captured["constructor"]["weight_evaluation"])
        self.assertIn("NN_TORCH", captured["fit"]["excluded_model_types"])
        self.assertIn("FASTAI", captured["fit"]["excluded_model_types"])
        self.assertEqual(captured["metrics"]["overall"]["mae"], 2.5)
        self.assertEqual(captured["metrics"]["overall"]["bias"], 2.5)
        self.assertEqual(captured["metrics"]["by_platoon_context"]["0"]["mae"], 10.0)
        self.assertEqual(captured["contract"]["sample_weighting"]["operator"], "<")
        self.assertNotIn(training.WEIGHT_COL, captured["predict"].columns)

    def test_equal_weight_mode_has_no_weight_column(self):
        train, validation = self.loaded_pair()
        captured = self.fake_fit(train, validation, self.args())
        self.assertNotIn("sample_weight", captured["constructor"])
        self.assertNotIn(training.WEIGHT_COL, captured["fit"]["train_data"].columns)
        self.assertEqual(captured["contract"]["sample_weighting"]["mode"], "none")

    def test_weighted_unknown_framework_version_is_rejected_before_directory_creation(self):
        train, validation = self.loaded_pair()
        with self.assertRaisesRegex(ValueError, "Weighted training is audited"):
            self.fake_fit(train, validation, self.args("--sample-weighting", "low-flow"), version="1.7.0")
        self.assertFalse((self.root / "model").exists())

    def test_weighting_choice_required_and_original_parser_defaults_unchanged(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            training.build_parser().parse_args(["--save-path", "unused"])
        self.assertEqual(legacy.build_parser().parse_args(["--save-path", "unused"]).sample_weighting, "none")

    def test_existing_output_rejected_before_collection_loading(self):
        output = self.root / "exists"
        output.mkdir()
        with mock.patch.object(training, "load_split", side_effect=AssertionError("must not load")):
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                training.main(["--save-path", str(output), "--sample-weighting", "none"])

    def test_six_threshold_dry_run_order_has_no_autogluon_or_writes(self):
        first, second = self.write("train.csv"), self.write("val.csv", self.sample(100))
        before = {str(path.relative_to(self.root)) for path in self.root.rglob("*")}
        with mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}), \
                mock.patch.object(training, "fit_model", side_effect=AssertionError("no fit")) as fit, \
                contextlib.redirect_stdout(io.StringIO()) as stdout:
            results = training.main(["--csv", str(first), "--validation-csv", str(second),
                                     "--save-path", str(self.root / "never-created"), "--sample-weighting", "low-flow",
                                     "--all-thresholds", "--dry-run"])
        fit.assert_not_called()
        self.assertEqual([result["source_label"] for result in results],
                         ["travel_time_label_gt_" + value for value in ["0p1", "3", "1", "0p5", "1p5", "2"]])
        self.assertEqual(stdout.getvalue().count("[DRY-RUN] Passed"), 6)
        self.assertEqual(before, {str(path.relative_to(self.root)) for path in self.root.rglob("*")})

    def test_six_thresholds_preflight_missing_later_label_before_fit(self):
        first = self.write("train.csv", self.sample().drop(columns="travel_time_label_gt_2"))
        second = self.write("val.csv", self.sample(100))
        with mock.patch.object(training, "fit_model", side_effect=AssertionError("no fit")) as fit:
            with self.assertRaisesRegex(ValueError, "six-threshold input missing labels"):
                training.main(["--csv", str(first), "--validation-csv", str(second), "--save-path", str(self.root / "no-fit"),
                               "--sample-weighting", "none", "--all-thresholds"])
        fit.assert_not_called()

    def test_all_thresholds_cannot_silently_ignore_explicit_label(self):
        with self.assertRaisesRegex(ValueError, "Do not combine"):
            training.validate_options(self.args("--all-thresholds"))

    def test_manifest_preserves_exact_eighty_twenty_whole_episode_split(self):
        dataset = self.root / "dataset"
        commands = dataset / "commands"
        commands.mkdir(parents=True)
        run = self.root / "run"
        with (commands / "traci_jobs.tsv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=["episode_id", "split", "full_training_output"], delimiter="\t")
            writer.writeheader()
            for index in range(100):
                episode = f"ep_{index:03}"
                name = f"cams_model_training_data_{episode}.csv"
                writer.writerow({"episode_id": episode, "split": "train" if index < 80 else "validation", "full_training_output": name})
                self.write("run/training/" + name, self.sample(10 + index))
        train, validation, split = training.resolve_splits(self.args(
            "--dataset-dir", dataset, "--run-dir", run, "--expected-train-files", 80, "--expected-validation-files", 20))
        self.assertEqual((len(train), len(validation)), (80, 20))
        self.assertEqual(split, "manifest_external_validation")
        self.assertFalse(set(train).intersection(validation))

    def test_validation_metrics_handles_absent_categories_without_filling_mae(self):
        raw = self.sample()
        raw["platoon_context"] = 0
        frame = self.load(self.write("only-zero.csv", raw))
        result = training.validation_metrics(frame, [10, 11, 12, 13])
        self.assertEqual(result["by_platoon_context"]["0"]["n"], 4)
        self.assertEqual(result["by_platoon_context"]["3"], {"n": 0})
        self.assertEqual(result["by_platoon_context"]["4"], {"n": 0})
        self.assertEqual(result["by_platoon_context"]["5"], {"n": 0})

    def sample_all_six(self, base=10.0):
        first = self.sample(base)
        extra = self.sample(base + 10).iloc[:2].copy()
        extra["vehicleID"] = "v2"
        extra["platoon_context"] = [4, 5]
        return pd.concat([first, extra], ignore_index=True)

    def test_six_categories_pass_fit_unchanged_and_get_separate_metrics(self):
        train = self.load(self.write("six-train.csv", self.sample_all_six()))
        validation = self.load(self.write("six-validation.csv", self.sample_all_six(100)))
        deltas = np.arange(6, dtype=float)
        captured = self.fake_fit(train, validation, self.args(),
                                 predictions=validation[training.LABEL_COL].to_numpy() + deltas)
        self.assertEqual(captured["fit"]["train_data"]["platoon_context"].tolist(), [0, 1, 2, 3, 4, 5])
        self.assertEqual(captured["predict"]["platoon_context"].tolist(), [0, 1, 2, 3, 4, 5])
        self.assertEqual(set(captured["metrics"]["by_platoon_context"]), set("012345"))
        names = {"0": "A", "1": "legacy_1", "2": "legacy_2", "3": "legacy_3", "4": "B", "5": "C"}
        self.assertEqual(captured["metrics"]["platoon_context_names"], names)
        self.assertEqual(captured["contract"]["training_features"]["platoon_context"]["category_names"], names)
        self.assertEqual(captured["contract"]["training_features"]["platoon_context"]["counts"], {str(i): 1 for i in range(6)})
        for context in range(6):
            self.assertEqual(captured["metrics"]["by_platoon_context"][str(context)]["n"], 1)
            self.assertEqual(captured["metrics"]["by_platoon_context"][str(context)]["mae"], float(context))
        self.assertFalse(captured["contract"]["categorical_ordered"])

    def test_prior_four_state_schema_and_old_contract_are_not_relabelled(self):
        for updates in ({"schema": "road-time-cap-context-v1"},
                        {"context_contract": base_features.CONTEXT_CONTRACT}):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                self.load(self.write("old-four.csv", metadata_updates=updates))

    def test_changed_abc_mapping_ids_and_semantics_are_rejected(self):
        changes = []
        mapping = copy.deepcopy(DEFAULT_CONTEXT)
        mapping["context_mapping"]["4"] = "C"
        changes.append(mapping)
        for ids in ([0, 1, 2, 3], [0, 1, 2, 3, 5, 4], [0, 1, 2, 3, 4, 6], [0, 1, 2, 3, 4, 4]):
            context = copy.deepcopy(DEFAULT_CONTEXT)
            context["context_ids"] = ids
            changes.append(context)
        for key, value in (("overdue_formula", "physical waiting time"),
                           ("overdue_epsilon_seconds", 0), ("unexpected_key", "bad")):
            context = copy.deepcopy(DEFAULT_CONTEXT)
            context[key] = value
            changes.append(context)
        for context in changes:
            with self.subTest(context=context), self.assertRaises(ValueError):
                self.load(self.write("wrong-abc-semantics.csv", context=context))

    def test_abc_overdue_threshold_is_validated_and_preserved(self):
        for value in (-1, True, "0", float("nan"), float("inf")):
            context = copy.deepcopy(DEFAULT_CONTEXT)
            context["context0_overdue_threshold_seconds"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(self.write("wrong-overdue.csv", context=context))
        context = features.abc_context_contract(threshold=1.5)
        train = self.load(self.write("threshold-train.csv", context=context))
        validation = self.load(self.write("threshold-validation.csv", self.sample(100), context=context))
        captured = self.fake_fit(train, validation, self.args())
        self.assertEqual(captured["contract"]["context_contract"], context)
        self.assertEqual(captured["contract"]["context_contract"]["context0_overdue_threshold_seconds"], 1.5)

    def test_different_abc_overdue_thresholds_cannot_mix(self):
        first = self.write("abc-a.csv")
        second = self.write("abc-b.csv", self.sample(100), context=features.abc_context_contract(threshold=1.5))
        with self.assertRaisesRegex(ValueError, "Mixed collection"), contextlib.redirect_stdout(io.StringIO()):
            training.load_split([first, second], training.LABEL_COLUMNS[0])
        with self.assertRaisesRegex(ValueError, "context_contract mismatch"):
            training.shared_context_contract(self.load(first), self.load(second))

    def test_missing_classes_warn_without_synthetic_rows(self):
        train, validation = self.sample(), self.sample(100)
        train["platoon_context"] = 0
        validation["platoon_context"] = 5
        first, second = self.write("only-a-train.csv", train), self.write("only-c-validation.csv", validation)
        with contextlib.redirect_stdout(io.StringIO()) as stdout, \
                mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}):
            results = training.main(["--csv", str(first), "--validation-csv", str(second),
                                     "--save-path", str(self.root / "no-synthetic"),
                                     "--sample-weighting", "none", "--label-column", training.LABEL_COLUMNS[0], "--dry-run"])
        self.assertIn("training lacks platoon_context categories [1, 2, 3, 4, 5]", stdout.getvalue())
        self.assertIn("unseen in training: [5]", stdout.getvalue())
        self.assertEqual((results[0]["train_rows"], results[0]["validation_rows"]), (4, 4))
        self.assertFalse((self.root / "no-synthetic").exists())

    def test_training_dependency_provenance_points_to_exact_shared_sources(self):
        train, validation = self.loaded_pair()
        captured = self.fake_fit(train, validation, self.args())
        expected = {"model_training_sumo_sf.py", "cap_context_features.py", "cap_context_abc_features.py"}
        self.assertEqual(set(captured["contract"]["trainer_dependencies"]), expected)
        for name, item in captured["contract"]["trainer_dependencies"].items():
            path = Path(item["path"])
            self.assertEqual(path.name, name)
            self.assertEqual(item["sha256"], training.file_sha256(path))
        self.assertEqual(captured["contract"]["trainer_sha256"], training.file_sha256(training.__file__))

    def test_bad_predictions_cannot_produce_complete_contract_or_metrics(self):
        train, validation = self.loaded_pair()
        for index, predictions in enumerate(([1, 2], [float("nan")] * 4, [float("inf")] * 4)):
            output = self.root / f"prediction-failed-{index}"
            with self.subTest(predictions=predictions), self.assertRaises(ValueError):
                self.fake_fit(train, validation, self.args("--save-path", output), predictions=predictions)
            contract = json.loads((output / "training_contract.json").read_text(encoding="utf-8"))
            self.assertEqual(contract["status"], "failed")
            self.assertFalse((output / "validation_metrics.json").exists())

    def test_six_real_entry_dispatch_order_with_mock_fit_and_summary(self):
        first, second = self.write("six-dispatch-train.csv"), self.write("six-dispatch-val.csv", self.sample(100))
        dispatched = []
        def fit(train, validation, args, provenance):
            dispatched.append((Path(args.save_path).name, args.label_column, len(train), len(validation)))
            self.assertEqual(provenance["split_kind"], "explicit_external_validation")
            Path(args.save_path).mkdir(parents=True)
        output = self.root / "six-models"
        with mock.patch.object(training, "fit_model", side_effect=fit), contextlib.redirect_stdout(io.StringIO()), \
                mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}):
            training.main(["--csv", str(first), "--validation-csv", str(second), "--save-path", str(output),
                           "--sample-weighting", "none", "--all-thresholds"])
        order = ["0p1", "3", "1", "0p5", "1p5", "2"]
        self.assertEqual(dispatched, [("gt_" + t, "travel_time_label_gt_" + t, 4, 4) for t in order])
        summary = json.loads((output / "six_threshold_training.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["status"], "complete")
        self.assertEqual(summary["schema"], "road-time-cap-context-abc-v1")

    def test_interrupted_six_dispatch_does_not_mark_batch_complete(self):
        first, second = self.write("failure-train.csv"), self.write("failure-val.csv", self.sample(100))
        output = self.root / "failed-batch"
        calls = []
        def fit(train, validation, args, provenance):
            calls.append(args.label_column)
            if len(calls) == 2:
                raise RuntimeError("fixture batch interruption")
            Path(args.save_path).mkdir(parents=True)
        with mock.patch.object(training, "fit_model", side_effect=fit), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "batch interruption"):
                training.main(["--csv", str(first), "--validation-csv", str(second), "--save-path", str(output),
                               "--sample-weighting", "none", "--all-thresholds"])
        self.assertEqual(calls, ["travel_time_label_gt_0p1", "travel_time_label_gt_3"])
        self.assertFalse((output / "six_threshold_training.json").exists())

    def test_weighting_depends_on_flow_not_abc_context(self):
        raw = self.sample()
        raw["platoon_context"] = [5, 4, 0, 3]
        frame = self.load(self.write("abc-weighting.csv", raw))
        weights, audit = training.make_sample_weights(frame, self.args("--sample-weighting", "low-flow"))
        np.testing.assert_array_equal(weights, [1.5, 1.5, .5, .5])
        self.assertEqual(audit["cutoff"], 5)
        self.assertEqual(audit["low_flow_raw_weight"], 3)

    def test_identical_contents_across_split_rejected_before_any_fit(self):
        first, second = self.write("same-train.csv"), self.write("same-val.csv")
        with mock.patch.object(training, "fit_model", side_effect=AssertionError("must not fit")) as fit, \
                contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "identical|[Dd]uplicate|same"):
            training.main(["--csv", str(first), "--validation-csv", str(second), "--save-path", str(self.root / "leak"),
                           "--sample-weighting", "none", "--label-column", training.LABEL_COLUMNS[0]])
        fit.assert_not_called()


if __name__ == "__main__":
    unittest.main()
