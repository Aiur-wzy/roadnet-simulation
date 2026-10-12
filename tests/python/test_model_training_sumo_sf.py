"""Five-feature training-contract regressions; AutoGluon is always mocked."""

import contextlib
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

import training._shared.model_training_sumo_sf as training


class TrainingContractTests(unittest.TestCase):
    def setUp(self):
        self.directory = Path.cwd() / ("sf-trainer-test-" + uuid.uuid4().hex)
        self.directory.mkdir()
        self.addCleanup(self.cleanup_fixture)

    def cleanup_fixture(self):
        target = self.directory.resolve()
        if target.parent != Path.cwd().resolve() or not target.name.startswith("sf-trainer-test-"):
            raise RuntimeError("Refusing to clean a directory outside this test workspace")
        shutil.rmtree(target)

    def sample(self, vehicle="v0", base=10.0):
        frame = pd.DataFrame({
            "road_length": [100.0, 200.0],
            "road_flow": [2, 3],
            "lane_flow": [1, 2],
            "speed_factor": [0.8123456789012345, 0.8123456789012345],
            "speed_limit": [13.89, 13.89],
            "vehicleID": [vehicle, vehicle],
            "route_index": [0, 1],
            "is_final_road": [0, 1],
            "turn_type": [2, 0],
        })
        for index, label in enumerate(training.LABEL_COLUMNS):
            frame[label] = [base + index, base + index + 1]
        return frame

    def write_csv(self, name, frame=None):
        path = self.directory / name
        path.parent.mkdir(parents=True, exist_ok=True)
        (self.sample() if frame is None else frame).to_csv(path, index=False)
        return path

    def args(self, *arguments):
        return training.build_parser().parse_args([
            "--save-path", str(self.directory / "model-output"), *map(str, arguments),
        ])

    def load(self, path, label=None):
        with contextlib.redirect_stdout(io.StringIO()):
            return training.load_frame(str(path), label or training.LABEL_COLUMNS[0])

    def test_exact_five_feature_contract_and_constant_limit_are_preserved(self):
        self.assertEqual(training.FEATURE_COLS, [
            "road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit",
        ])
        frame = self.load(self.write_csv("one.csv"))
        self.assertEqual(list(frame.columns[:5]), training.FEATURE_COLS)
        summary = frame.attrs["audit"]["features"]["speed_limit"]
        self.assertEqual(summary, {"min": 13.89, "max": 13.89, "unique": 1, "constant": True})
        self.assertEqual(frame["speed_factor"].iloc[0], 0.8123456789012345)

    def test_all_six_labels_and_terminal_rows_without_old_base_columns(self):
        raw = self.sample().drop(columns="turn_type")
        for index, label in enumerate(training.LABEL_COLUMNS):
            with self.subTest(label=label):
                columns = training.FEATURE_COLS + training.META_COLS + [label]
                frame = self.load(self.write_csv("label.csv", raw[columns]), label)
                self.assertEqual(frame[training.LABEL_COL].tolist(), [10.0 + index, 11.0 + index])
                self.assertEqual(frame["is_final_road"].tolist(), [0, 1])
                self.assertEqual(frame.attrs["audit"]["final_rows_retained"], 1)
                self.assertNotIn("turn_type", frame.columns)
                self.assertNotIn("has_waiting", frame.columns)

    def test_only_selected_label_controls_nonpositive_filtering(self):
        raw = self.sample()
        raw["travel_time_label"] = 0
        raw[training.LABEL_COLUMNS[1]] = [0, 11]
        path = self.write_csv("zero.csv", raw)
        self.assertEqual(len(self.load(path, training.LABEL_COLUMNS[0])), 2)
        frame = self.load(path, training.LABEL_COLUMNS[1])
        self.assertEqual(len(frame), 1)
        self.assertEqual(frame.attrs["audit"]["nonpositive_label_rows_removed"], 1)
        self.assertEqual(frame["is_final_road"].tolist(), [1])

    def test_missing_feature_metadata_or_label_is_not_silently_filled(self):
        for column in training.FEATURE_COLS + training.META_COLS + [training.LABEL_COLUMNS[0]]:
            with self.subTest(column=column):
                path = self.write_csv("missing.csv", self.sample().drop(columns=column))
                with self.assertRaisesRegex(ValueError, "missing full-training columns"):
                    self.load(path)

    def test_invalid_sf_and_limit_fail_without_fallback(self):
        for column in ("speed_factor", "speed_limit"):
            for value in (0, -1, float("nan"), float("inf"), -float("inf"), "invalid"):
                with self.subTest(column=column, value=value):
                    raw = self.sample()
                    raw[column] = value
                    with self.assertRaises(ValueError):
                        self.load(self.write_csv("invalid.csv", raw))

    def test_sf_above_one_is_not_capped(self):
        raw = self.sample()
        raw["speed_factor"] = 1.3
        frame = self.load(self.write_csv("sf-high.csv", raw))
        self.assertEqual(frame["speed_factor"].tolist(), [1.3, 1.3])

    def test_same_vehicle_sf_must_be_constant(self):
        raw = self.sample()
        raw["speed_factor"] = [0.8, 0.9]
        with self.assertRaisesRegex(ValueError, "speed_factor changes within a vehicle"):
            self.load(self.write_csv("changing.csv", raw))

    def test_occupancy_and_metadata_validation(self):
        for column, values in (
            ("road_flow", [-1, 3]), ("lane_flow", [0.5, 2]),
            ("lane_flow", [3, 2]), ("route_index", [0, 1.5]),
            ("is_final_road", [0, 2]), ("vehicleID", ["", "v0"]),
        ):
            with self.subTest(column=column, values=values):
                raw = self.sample()
                raw[column] = values
                with self.assertRaises(ValueError):
                    self.load(self.write_csv("invalid-meta.csv", raw))

    def test_duplicate_vehicle_road_index_event_rejected(self):
        raw = self.sample()
        raw["route_index"] = [0, 0]
        with self.assertRaisesRegex(ValueError, "duplicate vehicleID x route_index"):
            self.load(self.write_csv("duplicate-event.csv", raw))

    def test_same_vehicle_id_in_different_episodes_is_allowed(self):
        one = self.write_csv("episode-one.csv", self.sample(base=10))
        two = self.write_csv("episode-two.csv", self.sample(base=20))
        with contextlib.redirect_stdout(io.StringIO()):
            frame, audits = training.load_split([str(one), str(two)], training.LABEL_COLUMNS[0])
        self.assertEqual(len(frame), 4)
        self.assertEqual(len(audits), 2)
        self.assertEqual(frame["__source_file"].nunique(), 2)

    def test_external_validation_files_are_separate(self):
        one = self.write_csv("train.csv")
        two = self.write_csv("validation.csv", self.sample(base=100))
        with contextlib.redirect_stdout(io.StringIO()):
            train_paths, validation_paths, kind = training.resolve_splits(self.args(
                "--csv", one, "--validation-csv", two,
                "--expected-train-files", 1, "--expected-validation-files", 1,
            ))
        self.assertEqual(train_paths, [str(one.resolve())])
        self.assertEqual(validation_paths, [str(two.resolve())])
        self.assertEqual(kind, "explicit_external_validation")

    def test_no_external_validation_splits_files_not_rows(self):
        paths = [self.write_csv(f"episode-{index}.csv", self.sample(base=10 + index)) for index in range(5)]
        args = self.args(*[argument for path in paths for argument in ("--csv", path)])
        with contextlib.redirect_stdout(io.StringIO()):
            train_paths, validation_paths, kind = training.resolve_splits(args)
            repeat = training.resolve_splits(args)
        self.assertEqual((len(train_paths), len(validation_paths)), (4, 1))
        self.assertEqual(kind, "whole_file_holdout")
        self.assertFalse(set(train_paths) & set(validation_paths))
        self.assertEqual(set(train_paths + validation_paths), {str(path.resolve()) for path in paths})
        self.assertEqual((train_paths, validation_paths, kind), repeat)

    def test_single_file_without_validation_refuses_row_split(self):
        path = self.write_csv("single.csv")
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "row splitting is disabled"):
            training.resolve_splits(self.args("--csv", path))

    def test_train_validation_overlap_is_rejected(self):
        path = self.write_csv("same.csv")
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "overlap"):
            training.resolve_splits(self.args("--csv", path, "--validation-csv", path))

    def test_wrong_expected_file_count_is_rejected(self):
        one, two = self.write_csv("a.csv"), self.write_csv("b.csv", self.sample(base=20))
        with contextlib.redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "Expected 2 train files"):
            training.resolve_splits(self.args("--csv", one, "--validation-csv", two, "--expected-train-files", 2))

    def test_relative_csv_list_and_glob_inputs(self):
        one, two = self.write_csv("a.csv"), self.write_csv("b.csv", self.sample(base=20))
        listing = self.directory / "train list.txt"
        listing.write_text("# comment\n\na.csv\n", encoding="utf-8-sig")
        with contextlib.redirect_stdout(io.StringIO()):
            train_paths, validation_paths, _ = training.resolve_splits(self.args(
                "--csv-list", listing, "--validation-csv-glob", self.directory / "b*.csv",
            ))
        self.assertEqual(train_paths, [str(one.resolve())])
        self.assertEqual(validation_paths, [str(two.resolve())])

    def test_duplicate_explicit_csv_and_glob_or_list_overlap_rejected(self):
        one = self.write_csv("dup-train.csv")
        two = self.write_csv("dup-validation.csv", self.sample(base=20))
        listing = self.directory / "duplicate-list.txt"
        listing.write_text("dup-train.csv\n", encoding="utf-8")
        for duplicates in (("--csv", one), ("--csv-glob", self.directory / "dup-tr*.csv"),
                           ("--csv-list", listing)):
            with self.subTest(duplicates=duplicates), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(ValueError, "Duplicate|duplicate"):
                    training.resolve_splits(self.args("--csv", one, *duplicates, "--validation-csv", two))

    def test_duplicate_content_across_splits_is_rejected(self):
        audits = [{"sha256": "same", "path": "a.csv"}]
        with self.assertRaisesRegex(ValueError, "Duplicate CSV content"):
            training.reject_duplicate_contents(audits, [{"sha256": "same", "path": "b.csv"}])

    def manifest(self, rows):
        dataset = self.directory / "dataset"
        run = self.directory / "run with spaces"
        manifest = dataset / "commands" / "traci_jobs.tsv"
        manifest.parent.mkdir(parents=True, exist_ok=True)
        with manifest.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=["episode_id", "split", "full_training_output"], delimiter="\t")
            writer.writeheader()
            writer.writerows(rows)
        for index, row in enumerate(rows):
            episode = row["episode_id"]
            self.write_csv(f"run with spaces/training/cams_model_training_data_{episode}.csv", self.sample(base=10 + index))
        return dataset, run

    def test_manifest_split_uses_names_and_full_output_basename(self):
        dataset, run = self.manifest([
            {"episode_id": "ep-val", "split": "validation", "full_training_output": "outputs/traci/cams_model_training_data_ep-val.csv"},
            {"episode_id": "ep-train", "split": "train", "full_training_output": "outputs\\traci\\cams_model_training_data_ep-train.csv"},
        ])
        train_paths, validation_paths, kind = training.resolve_splits(self.args(
            "--dataset-dir", dataset, "--run-dir", run,
            "--expected-train-files", 1, "--expected-validation-files", 1,
        ))
        self.assertEqual(kind, "manifest_external_validation")
        self.assertTrue(train_paths[0].endswith("cams_model_training_data_ep-train.csv"))
        self.assertTrue(validation_paths[0].endswith("cams_model_training_data_ep-val.csv"))

    def test_manifest_unknown_split_duplicate_episode_bad_basename(self):
        base = {"episode_id": "ep", "split": "train", "full_training_output": "cams_model_training_data_ep.csv"}
        for rows, match in (
            ([dict(base, split="testing")], "Unknown manifest split"),
            ([base, base], "Duplicate episode_id"),
            ([dict(base, full_training_output="wrong.csv")], "Unexpected full CSV basename"),
        ):
            with self.subTest(match=match):
                dataset, run = self.manifest(rows)
                with self.assertRaisesRegex(ValueError, match):
                    training.manifest_splits(dataset, run)

    def test_manifest_and_manual_inputs_cannot_mix(self):
        with self.assertRaisesRegex(ValueError, "Do not mix"):
            training.resolve_splits(self.args("--dataset-dir", "dataset", "--run-dir", "run", "--csv", "one.csv"))

    def test_missing_data_input_and_partial_manifest_pair_fail(self):
        for arguments in ((), ("--dataset-dir", "dataset"), ("--run-dir", "run")):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                training.resolve_splits(self.args(*arguments))

    def test_invalid_training_options_fail(self):
        for arguments in (("--bag-folds", "1"), ("--stack-levels", "-1"),
                          ("--bag-folds", "0", "--stack-levels", "1"),
                          ("--time-limit", "0"), ("--num-gpus", "-1"),
                          ("--validation-fraction", "1"), ("--expected-train-files", "-1")):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                training.validate_options(self.args(*arguments))

    def test_validation_mae_bias_groups_and_prediction_validation(self):
        frame = self.load(self.write_csv("metrics.csv"))
        metrics = training.validation_metrics(frame, [12, 10])
        self.assertEqual(metrics["overall"]["n"], 2)
        self.assertEqual(metrics["overall"]["mae"], 1.5)
        self.assertEqual(metrics["overall"]["bias"], 0.5)
        self.assertEqual(metrics["by_final_road"]["1"]["bias"], -1.0)
        self.assertEqual(len(metrics["by_file"]), 1)
        for prediction in ([1], [float("nan"), 1], [[1], [2]]):
            with self.subTest(prediction=prediction), self.assertRaises(ValueError):
                training.validation_metrics(frame, prediction)

    def test_mock_fit_gets_five_inputs_and_no_external_validation(self):
        train_frame = self.load(self.write_csv("fit.csv", self.sample(base=10)))
        validation_frame = self.load(self.write_csv("held-out.csv", self.sample(base=100)))
        captured = {}

        class FakePredictor:
            def __init__(self, **kwargs):
                captured["constructor"] = kwargs
                self.feature_metadata = types.SimpleNamespace(
                    get_features=lambda: [c for c in training.FEATURE_COLS if c != "speed_limit"])

            def fit(self, **kwargs):
                captured["fit"] = kwargs
                return self

            def predict(self, frame):
                captured["prediction_input"] = frame.copy()
                return np.array([102.0, 100.0])

        package = types.ModuleType("autogluon")
        tabular = types.ModuleType("autogluon.tabular")
        tabular.TabularPredictor = FakePredictor
        args = self.args("--time-limit", 42, "--bag-folds", 3, "--stack-levels", 1, "--num-gpus", 0)
        with mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}), contextlib.redirect_stdout(io.StringIO()):
            predictor = training.fit_model(train_frame, validation_frame, args, {"split_kind": "fixture"})
        self.assertIsInstance(predictor, FakePredictor)
        self.assertEqual(captured["constructor"]["eval_metric"], "mae")
        self.assertEqual(list(captured["fit"]["train_data"].columns), training.FEATURE_COLS + [training.LABEL_COL])
        self.assertEqual(captured["fit"]["train_data"][training.LABEL_COL].tolist(), [10, 11])
        self.assertNotIn("tuning_data", captured["fit"])
        self.assertNotIn("validation_data", captured["fit"])
        self.assertEqual(captured["fit"]["num_bag_folds"], 3)
        self.assertEqual(captured["fit"]["num_stack_levels"], 1)
        self.assertEqual(captured["fit"]["time_limit"], 42)
        self.assertEqual(list(captured["prediction_input"].columns), training.FEATURE_COLS)
        contract = json.loads((Path(args.save_path) / "training_contract.json").read_text(encoding="utf-8"))
        self.assertEqual(contract["raw_features"], training.FEATURE_COLS)
        self.assertIn("speed_limit", contract["constant_training_features"])
        self.assertNotIn("speed_limit", contract["framework_features"])
        self.assertTrue(contract["raw_schema_preserved_even_if_framework_prunes_constants"])
        self.assertEqual(contract["source_label"], training.LABEL_COLUMNS[0])
        metrics = json.loads((Path(args.save_path) / "validation_metrics.json").read_text(encoding="utf-8"))
        self.assertEqual(metrics["overall"]["mae"], 1.5)
        self.assertEqual(metrics["overall"]["bias"], 0.5)

    def test_existing_model_directory_refused_before_autogluon_import(self):
        args = self.args()
        Path(args.save_path).mkdir()
        with mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}):
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                training.fit_model(None, None, args, {})

    def test_existing_main_output_refused_before_loading_any_csv(self):
        output = self.directory / "existing-output"
        output.mkdir()
        with mock.patch.object(training, "load_split", side_effect=AssertionError("must fail before loading")) as loader:
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                training.main(["--csv", "unused.csv", "--save-path", str(output)])
        loader.assert_not_called()

    def test_full_100_episode_manifest_dry_run_keeps_80_20_and_writes_nothing(self):
        rows = [
            {"episode_id": f"ep-{index:03d}",
             "split": "validation" if index % 5 == 0 else "train",
             "full_training_output": f"outputs/traci/cams_model_training_data_ep-{index:03d}.csv"}
            for index in range(100)
        ]
        dataset, run = self.manifest(rows)
        output = self.directory / "not-created-100" / "model"
        before = {str(path.relative_to(self.directory)) for path in self.directory.rglob("*")}
        arguments = ["--dataset-dir", str(dataset), "--run-dir", str(run),
                     "--expected-train-files", "80", "--expected-validation-files", "20",
                     "--save-path", str(output), "--dry-run"]
        with mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}), \
                mock.patch.object(training, "fit_model", side_effect=AssertionError("must not train")), \
                contextlib.redirect_stdout(io.StringIO()) as stdout:
            training.main(arguments)
        self.assertIn("train_files=80 validation_files=20", stdout.getvalue())
        self.assertIn("[DRY-RUN] Passed", stdout.getvalue())
        self.assertEqual(stdout.getvalue().count("[INPUT]"), 100)
        after = {str(path.relative_to(self.directory)) for path in self.directory.rglob("*")}
        self.assertEqual(before, after)
        self.assertFalse(output.exists())
        invalid = arguments.copy()
        invalid[invalid.index("--expected-train-files") + 1] = "79"
        with self.assertRaisesRegex(ValueError, "Expected 79 train files"):
            training.main(invalid)

    def test_cli_dry_run_never_trains_imports_autogluon_or_writes_outputs(self):
        one = self.write_csv("dry-train.csv", self.sample(base=10))
        two = self.write_csv("dry-validation.csv", self.sample(base=100))
        output = self.directory / "not-created" / "model"
        before = {str(path.relative_to(self.directory)) for path in self.directory.rglob("*")}
        with mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}), \
                mock.patch.object(training, "fit_model", side_effect=AssertionError("must not train")), \
                contextlib.redirect_stdout(io.StringIO()) as stdout:
            training.main(["--csv", str(one), "--validation-csv", str(two),
                           "--save-path", str(output), "--dry-run"])
        after = {str(path.relative_to(self.directory)) for path in self.directory.rglob("*")}
        self.assertEqual(before, after)
        self.assertFalse(output.exists())
        self.assertIn("[DRY-RUN] Passed", stdout.getvalue())


if __name__ == "__main__":
    unittest.main()
