"""ABC/RG training contract and mocked fit tests; never train a real model."""
import contextlib
import copy
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

from features.cap_context_abc_features import abc_context_contract
import training._shared.model_training_sumo_cap_context_abc as old
import training.model_training_sumo_cap_context_abc_rg as training
import training._shared.model_training_sumo_sf as legacy


class ABCRGTrainingTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".rg_training_test_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        target = self.root.resolve()
        if target.parent != Path(__file__).resolve().parent or not target.name.startswith(".rg_training_test_"):
            raise RuntimeError("Refusing unscoped test cleanup")
        shutil.rmtree(target)

    def sample(self, base=10.):
        frame = pd.DataFrame({
            "road_length": [100., 200., 100., 200., 100., 200.],
            "road_flow": [1, 4, 5, 6, 7, 8], "lane_flow": [1, 2, 2, 3, 3, 4],
            "free_speed_cap": [10.123456789012345, 11., 12., 13., 10., 12.],
            "platoon_context": [0, 1, 2, 3, 4, 5],
            "entry_signal_type": ["G", "G", "R", "G", "R", "G"],
            "entry_signal_remaining_seconds": [4., 0., 45., 0., 20., 0.],
            "next_signal_type": ["R", "G", "G", "G", "G", "G"],
            "next_signal_duration_seconds": [63., 0., 24., 0., 24., 0.],
            "vehicleID": ["v0", "v0", "v1", "v1", "v2", "v2"],
            "route_index": [0, 1, 0, 1, 0, 1], "is_final_road": [0, 1, 0, 1, 0, 1],
        })
        for index, label in enumerate(training.LABEL_COLUMNS):
            frame[label] = np.arange(base + index, base + index + 6)
        return frame

    def signal_contract(self, origin=0., clock_queries=3):
        signal = training.expected_signal_contract()
        signal.update(runtime_programs={"T": {"program_id": "0", "durations": [24., 3., 63.],
                                              "states": ["G", "y", "r"], "initial_cycle_origin": origin}},
                      clock_queries=clock_queries)
        return signal

    def write(self, name, frame=None, context=None, signal=None, metadata=None,
              audit_update=None, audit=True):
        path = self.root / name
        frame = self.sample() if frame is None else frame.copy()
        context = copy.deepcopy(context or abc_context_contract())
        frame.to_csv(path, index=False)
        paired = frame[training.META_COLS + training.SIGNAL_FEATURE_COLS].copy()
        paired["decision_time"] = [20., 30., 45., 50., 70., 90.]
        delta = np.where(frame["route_index"] == 0, 0., context["base_context_contract"]["crossing_time_seconds"])
        paired["planned_enter_time"] = paired["decision_time"] + delta
        paired["signal_anchor_time"] = paired["decision_time"]
        paired["signal_reference_time"] = paired["planned_enter_time"]
        paired["signal_projection_seconds"] = delta
        paired["incoming_junction_status"] = np.where(frame["route_index"] == 0, "route_start", "signalized_node")
        paired["incoming_crossing_seconds"] = delta
        paired["signal_status"] = np.where(frame["is_final_road"] == 1, "no_outgoing_route", "controlled_static")
        paired["signal_tls_id"] = np.where(frame["is_final_road"] == 1, "", "T")
        paired["signal_program_id"] = np.where(frame["is_final_road"] == 1, "", "0")
        if audit_update:
            audit_update(paired)
        audit_path = Path(str(path) + ".audit.csv")
        if audit:
            paired.to_csv(audit_path, index=False)
        contract = {
            "schema": training.SCHEMA, "raw_features": list(training.FEATURE_COLS),
            "context_contract": context, "signal_contract": signal or self.signal_contract(),
            "data_sha256": training.file_sha256(path),
            "audit": {"path": str(audit_path), "sha256": training.file_sha256(audit_path) if audit else "missing"},
            "sampling": dict(training.COLLECTION_SAMPLING),
            "source": {"fixture": True},
        }
        contract.update(metadata or {})
        Path(str(path) + ".contract.json").write_text(json.dumps(contract), encoding="utf-8")
        return path

    def load(self, path, label=None):
        with contextlib.redirect_stdout(io.StringIO()):
            return training.load_frame(path, label or training.DEFAULT_LABEL_COLUMN)

    def args(self, *extra):
        return training.build_parser().parse_args([
            "--save-path", str(self.root / "model"), "--sample-weighting", "none",
            "--label-column", training.DEFAULT_LABEL_COLUMN, *map(str, extra)])

    def pair(self):
        return self.load(self.write("train.csv")), self.load(self.write("validation.csv", self.sample(100)))

    def fake_fit(self, train, validation, args=None, used=None, predictions=None,
                 fail=False, framework_version="1.6.1"):
        captured = {}
        class Predictor:
            def __init__(self, **kwargs):
                captured["constructor"] = kwargs
                self.feature_metadata = types.SimpleNamespace(get_features=lambda: training.FEATURE_COLS if used is None else used)
            def fit(self, **kwargs):
                captured["fit"] = kwargs
                if fail:
                    raise RuntimeError("mock fit failure")
                return self
            def predict(self, frame):
                captured["predict"] = frame.copy()
                return validation[training.LABEL_COL].to_numpy() if predictions is None else np.asarray(predictions)
        args = args or self.args()
        package, tabular = types.ModuleType("autogluon"), types.ModuleType("autogluon.tabular")
        tabular.TabularPredictor = Predictor
        provenance = {"train_files": [train.attrs["audit"]], "validation_files": [validation.attrs["audit"]]}
        with mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}), \
                mock.patch.object(training, "version", return_value=framework_version), contextlib.redirect_stdout(io.StringIO()):
            training.fit_model(train, validation, args, provenance)
        captured["contract"] = json.loads((Path(args.save_path) / "training_contract.json").read_text())
        captured["metrics"] = json.loads((Path(args.save_path) / "validation_metrics.json").read_text())
        return captured

    def test_nine_features_do_not_mutate_old_globals(self):
        self.assertEqual(training.FEATURE_COLS, old.FEATURE_COLS + training.SIGNAL_FEATURE_COLS)
        self.assertEqual(len(training.FEATURE_COLS), 9)
        self.assertEqual(len(old.FEATURE_COLS), 5)
        self.assertEqual(old.SCHEMA, "road-time-cap-context-abc-v1")
        self.assertEqual(legacy.FEATURE_COLS[-2:], ["speed_factor", "speed_limit"])

    def test_all_three_categories_are_unordered_and_complete(self):
        raw = self.sample()
        raw["platoon_context"] = 0
        raw["entry_signal_type"] = "G"
        frame = training.encode_feature_frame(raw)
        for name, categories in training.CATEGORICAL_FEATURES.items():
            self.assertIsInstance(frame[name].dtype, pd.CategoricalDtype)
            self.assertEqual(frame[name].cat.categories.tolist(), categories)
            self.assertFalse(frame[name].cat.ordered)
        self.assertEqual(training.feature_summary(frame)["entry_signal_type"]["observed_categories"], ["G"])

    def test_duration_fraction_preserved_and_audit_not_selected(self):
        raw = self.sample()
        precise = 4.123456789012345
        raw.loc[0, "entry_signal_remaining_seconds"] = precise
        for name in ("Z", "M", "P", "E", "actual_edge_enter_time", "signal_projection_seconds", "speed_factor", "speed_limit"):
            raw[name] = 123
        frame = training.encode_feature_frame(raw)
        self.assertEqual(list(frame), training.FEATURE_COLS)
        self.assertEqual(frame.loc[0, "entry_signal_remaining_seconds"], precise)
        self.assertEqual(frame["entry_signal_remaining_seconds"].dtype, np.dtype("float64"))

    def test_zero_durations_only_g0g0_sentinel(self):
        training.encode_feature_frame(self.sample())
        for first, seconds, second, duration in (("R", 0, "G", 0), ("G", 0, "G", 5), ("G", 4, "R", 0), ("R", -1, "G", 24)):
            raw = self.sample()
            raw.loc[0, training.SIGNAL_FEATURE_COLS] = [first, seconds, second, duration]
            with self.subTest(values=(first, seconds, second, duration)), self.assertRaises(ValueError):
                training.encode_feature_frame(raw)

    def test_signal_categories_reject_numeric_yellow_lowercase_and_missing(self):
        for column in ("entry_signal_type", "next_signal_type"):
            for value in (0, 1, True, "Y", "g", "unknown", None):
                raw = self.sample()
                raw[column] = raw[column].astype(object)
                raw.loc[0, column] = value
                with self.subTest(column=column, value=value), self.assertRaises(ValueError):
                    training.encode_feature_frame(raw)

    def test_signal_durations_reject_nonfinite_and_boolean(self):
        for name in ("entry_signal_remaining_seconds", "next_signal_duration_seconds"):
            for value in (float("nan"), float("inf"), -1, True):
                raw = self.sample()
                raw[name] = value
                with self.subTest(column=name, value=value), self.assertRaises(ValueError):
                    training.encode_feature_frame(raw)

    def test_old_csv_missing_signals_and_duplicate_columns_rejected(self):
        with self.assertRaisesRegex(ValueError, "Missing ABC/RG"):
            training.encode_feature_frame(self.sample().drop(columns=training.SIGNAL_FEATURE_COLS))
        raw = self.sample()
        raw = pd.concat([raw, raw[["entry_signal_type"]]], axis=1)
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            training.encode_feature_frame(raw)

    def test_valid_runtime_differences_do_not_change_stable_contract(self):
        first, second = self.signal_contract(), self.signal_contract(origin=-50., clock_queries=999)
        second["runtime_programs"]["T"]["durations"] = [15., 3., 72.]
        second["runtime_programs"]["T"]["program_id"] = "alternate"
        expected = training.expected_signal_contract()
        self.assertEqual(training.validate_signal_contract(first), expected)
        self.assertEqual(training.validate_signal_contract(second), expected)
        self.assertEqual(training.validate_signal_contract(expected), expected)
        self.assertIn("runtime_programs", first)

    def test_changed_missing_extra_signal_rules_are_rejected(self):
        for key in training.expected_signal_contract():
            with self.subTest(key=key):
                signal = self.signal_contract()
                signal[key] = "wrong"
                with self.assertRaisesRegex(ValueError, "signal_contract"):
                    training.validate_signal_contract(signal)
                signal = self.signal_contract()
                del signal[key]
                with self.assertRaises(ValueError):
                    training.validate_signal_contract(signal)
        signal = self.signal_contract()
        signal["future_actual_entry"] = True
        with self.assertRaises(ValueError):
            training.validate_signal_contract(signal)

    def test_invalid_runtime_provenance_rejected(self):
        mutations = [lambda s: s.update(clock_queries=True), lambda s: s.update(clock_queries=-1),
                     lambda s: s.update(runtime_programs=[]), lambda s: s.pop("clock_queries"),
                     lambda s: s["runtime_programs"]["T"].update(durations=[True, 3, 63]),
                     lambda s: s["runtime_programs"]["T"].update(durations=[0, 3, 63]),
                     lambda s: s["runtime_programs"]["T"].update(initial_cycle_origin=float("nan")),
                     lambda s: s["runtime_programs"]["T"].update(states=["G", "y", "off"]),
                     lambda s: s["runtime_programs"]["T"].update(program_id="")]
        for mutate in mutations:
            signal = self.signal_contract()
            mutate(signal)
            with self.subTest(signal=signal), self.assertRaises(ValueError):
                training.validate_signal_contract(signal)

    def test_signal_placeholder_rule_rejects_boolean_numeric_alias(self):
        signal = self.signal_contract()
        signal["no_tls_placeholder"]["entry_signal_remaining_seconds"] = False
        with self.assertRaises(ValueError):
            training.validate_signal_contract(signal)

    def test_collection_sampling_rules_cannot_change(self):
        sampling = dict(training.COLLECTION_SAMPLING)
        sampling["platoon_context"] = "future actual entry"
        with self.assertRaisesRegex(ValueError, "sampling"):
            self.load(self.write("changed-sampling.csv", metadata={"sampling": sampling}))

    def test_csv_audit_sha_and_schema_required(self):
        for changes in ({"schema": old.SCHEMA}, {"raw_features": old.FEATURE_COLS},
                        {"data_sha256": "stale"}, {"audit": {"sha256": "stale"}}, {"signal_contract": None}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.load(self.write("bad.csv", metadata=changes))
        with self.assertRaisesRegex(ValueError, "missing paired audit"):
            self.load(self.write("no-audit.csv", audit=False))
        with self.assertRaisesRegex(ValueError, "runtime provenance"):
            self.load(self.write("no-runtime.csv", signal=training.expected_signal_contract()))

    def test_six_labels_and_all_final_rows_preserved(self):
        path = self.write("six.csv")
        for index, label in enumerate(training.LABEL_COLUMNS):
            frame = self.load(path, label)
            np.testing.assert_array_equal(frame[training.LABEL_COL], np.arange(10 + index, 16 + index))
            self.assertEqual(frame.attrs["audit"]["final_rows_retained"], 3)
            self.assertEqual(frame.loc[frame["is_final_road"] == 1, "entry_signal_remaining_seconds"].tolist(), [0., 0., 0.])

    def test_only_selected_nonpositive_label_is_removed(self):
        raw = self.sample()
        raw.loc[0, training.DEFAULT_LABEL_COLUMN] = 0
        raw[training.LABEL_COLUMNS[0]] = -1
        frame = self.load(self.write("selected.csv", raw))
        self.assertEqual(len(frame), 5)
        self.assertEqual(frame.attrs["audit"]["nonpositive_label_rows_removed"], 1)

    def test_stable_contract_shared_runtime_kept_per_source(self):
        first = self.write("a.csv")
        second = self.write("b.csv", self.sample(100), signal=self.signal_contract(origin=-50, clock_queries=900))
        with contextlib.redirect_stdout(io.StringIO()):
            frame, audits = training.load_split([first, second], training.DEFAULT_LABEL_COLUMN)
        self.assertEqual(frame.attrs["signal_contract"], training.expected_signal_contract())
        self.assertEqual([a["collection_contract_file"]["signal_contract"]["clock_queries"] for a in audits], [3, 900])

    def test_audit_timestamps_reject_actual_entry_or_wrong_crossing(self):
        for name in ("planned_enter_time", "signal_anchor_time", "signal_reference_time", "signal_projection_seconds"):
            with self.subTest(column=name), self.assertRaisesRegex(ValueError, "audit"):
                self.load(self.write("bad-time.csv", audit_update=lambda frame, n=name: frame.loc.__setitem__((1, n), 500.)))

    def test_audit_frozen_values_identity_status_and_tls_checked(self):
        cases = [("entry_signal_remaining_seconds", 0, 5.), ("vehicleID", 0, "wrong"),
                 ("signal_status", 1, "controlled_static"), ("signal_tls_id", 0, "MISSING"),
                 ("signal_program_id", 0, "UNKNOWN")]
        for name, row, value in cases:
            with self.subTest(column=name), self.assertRaises(ValueError):
                self.load(self.write("bad-audit.csv", audit_update=lambda f, n=name, r=row, v=value: f.loc.__setitem__((r, n), v)))

    def test_nondefault_crossing_preserved_and_mixed_contract_rejected(self):
        context = abc_context_contract()
        context["base_context_contract"]["crossing_time_seconds"] = 2.5
        first, second = self.write("a.csv"), self.write("b.csv", self.sample(100), context=context)
        frame = self.load(second)
        self.assertEqual(frame.attrs["context_contract"], context)
        with self.assertRaisesRegex(ValueError, "Mixed collection"), contextlib.redirect_stdout(io.StringIO()):
            training.load_split([first, second], training.DEFAULT_LABEL_COLUMN)

    def test_default_gt1_dryrun_no_training_or_output(self):
        first, second = self.write("train.csv"), self.write("val.csv", self.sample(100))
        with mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}), \
                mock.patch.object(training, "fit_model", side_effect=AssertionError("must not fit")), contextlib.redirect_stdout(io.StringIO()):
            results = training.main(["--csv", str(first), "--validation-csv", str(second), "--save-path", str(self.root / "no-model"),
                                     "--sample-weighting", "none", "--dry-run"])
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]["source_label"], "travel_time_label_gt_1")
        self.assertEqual(results[0]["sample_weighting"]["mode"], "none")
        self.assertFalse((self.root / "no-model").exists())

    def test_all_thresholds_remains_explicit_and_exact_order(self):
        first, second = self.write("train.csv"), self.write("val.csv", self.sample(100))
        with contextlib.redirect_stdout(io.StringIO()):
            results = training.main(["--csv", str(first), "--validation-csv", str(second), "--save-path", str(self.root / "no-model"),
                                     "--sample-weighting", "none", "--all-thresholds", "--dry-run"])
        self.assertEqual([r["source_label"] for r in results], ["travel_time_label_gt_" + t for t in training.THRESHOLD_ORDER])
        with self.assertRaises(ValueError):
            training.validate_options(self.args("--all-thresholds"))

    def test_explicit_weighting_choice_required(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            training.build_parser().parse_args(["--save-path", "unused"])
        self.assertEqual(legacy.build_parser().parse_args(["--save-path", "unused"]).sample_weighting, "none")

    def test_mock_fit_predict_exact_nine_categories_contract_and_runtime_sources(self):
        train, validation = self.pair()
        captured = self.fake_fit(train, validation)
        self.assertEqual(list(captured["fit"]["train_data"]), training.FEATURE_COLS + [training.LABEL_COL])
        self.assertEqual(list(captured["predict"]), training.FEATURE_COLS)
        for name, categories in training.CATEGORICAL_FEATURES.items():
            for frame in (captured["fit"]["train_data"], captured["predict"]):
                self.assertEqual(frame[name].cat.categories.tolist(), categories)
                self.assertFalse(frame[name].cat.ordered)
        contract = captured["contract"]
        self.assertEqual(contract["schema"], training.SCHEMA)
        self.assertEqual(contract["raw_features"], training.FEATURE_COLS)
        self.assertEqual(contract["signal_contract"], training.expected_signal_contract())
        self.assertIn("runtime_programs", contract["train_files"][0]["collection_contract_file"]["signal_contract"])
        self.assertEqual(contract["status"], "complete")
        self.assertEqual(captured["metrics"]["overall"]["mae"], 0.)
        self.assertEqual(captured["metrics"]["by_signal_exit"]["no_exit"]["n"], 3)
        self.assertNotIn("tuning_data", captured["fit"])

    def test_declared_constant_signal_categories_survive_framework_pruning(self):
        raw = self.sample()
        raw["entry_signal_type"] = "G"
        train = self.load(self.write("train.csv", raw))
        validation = self.load(self.write("val.csv", self.sample(100)))
        used = [name for name in training.FEATURE_COLS if name != "entry_signal_type"]
        captured = self.fake_fit(train, validation, used=used)
        self.assertIn("entry_signal_type", captured["contract"]["raw_features"])
        self.assertIn("entry_signal_type", captured["contract"]["constant_training_features"])
        self.assertNotIn("entry_signal_type", captured["contract"]["framework_features"])

    def test_weighting_only_flow_and_validation_metrics_unweighted(self):
        train, validation = self.pair()
        predicted = validation[training.LABEL_COL].to_numpy() + np.arange(6)
        captured = self.fake_fit(train, validation, self.args("--sample-weighting", "low-flow"), predictions=predicted)
        np.testing.assert_allclose(captured["fit"]["train_data"][training.WEIGHT_COL], [1.8, 1.8, .6, .6, .6, .6])
        self.assertEqual(captured["metrics"]["overall"]["mae"], 2.5)
        self.assertEqual(captured["metrics"]["overall"]["bias"], 2.5)
        self.assertNotIn(training.WEIGHT_COL, captured["predict"])
        self.assertIn("NN_TORCH", captured["fit"]["excluded_model_types"])
        self.assertNotIn(training.WEIGHT_COL, captured["contract"]["raw_features"])

    def test_unknown_weighted_framework_version_rejected_before_output(self):
        train, validation = self.pair()
        with self.assertRaisesRegex(ValueError, "Weighted training is audited"):
            self.fake_fit(train, validation, self.args("--sample-weighting", "low-flow"), framework_version="unknown")
        self.assertFalse((self.root / "model").exists())

    def test_bad_metadata_failed_fit_and_bad_prediction_never_mark_complete(self):
        train, validation = self.pair()
        for name, options in (("bad-input", {"used": training.FEATURE_COLS + ["signal_reference_time"]}),
                              ("failed", {"fail": True}), ("nonfinite", {"predictions": [float("nan")] * 6})):
            output = self.root / name
            with self.subTest(name=name), self.assertRaises((ValueError, RuntimeError)):
                self.fake_fit(train, validation, self.args("--save-path", output), **options)
            self.assertEqual(json.loads((output / "training_contract.json").read_text())["status"], "failed")

    def test_metrics_separate_all_contexts_and_signal_pairs(self):
        train, validation = self.pair()
        result = training.validation_metrics(validation, validation[training.LABEL_COL] + np.arange(6))
        self.assertEqual(set(result["by_platoon_context"]), set("012345"))
        self.assertEqual(result["by_signal_pair"]["RR"], {"n": 0})
        self.assertEqual(result["by_signal_pair"]["GG"]["n"], 3)
        for context in range(6):
            self.assertEqual(result["by_platoon_context"][str(context)]["mae"], context)


if __name__ == "__main__":
    unittest.main()
