"""Synthetic current-native releases; no SUMO, GPU, or AutoGluon needed."""
from __future__ import annotations

import copy
import csv
import gzip
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import pandas as pd

from training import native_contract as contract
from training import native_data as data
from training import train_road_time as train


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True), encoding="utf-8")


def write_rows(path, rows):
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=data.READ_COLUMNS)
    writer.writeheader()
    writer.writerows(rows)
    with Path(path).open("wb") as output:
        with gzip.GzipFile(filename="", mode="wb", fileobj=output, mtime=0) as compressed:
            compressed.write(stream.getvalue().encode())


def make_rows(sid, road_length=10):
    result = []
    for index, label in enumerate((2.0, 0.0, 0.5)):
        final = index == 2
        observed = 5.0 * index
        row = {key: "1" for key in data.FLAGS}
        row.update(schema=contract.SOURCE_SCHEMA, vehicle_id="shared_vehicle", route_index=str(index),
                   is_final_road=str(int(final)), training_release_scope="provisional_until_run_audit_passes",
                   road_length=str(road_length), road_flow="2", lane_flow="1", free_speed_cap="10",
                   platoon_context="0", entry_signal_type="G", next_signal_type="G" if final else "R",
                   entry_signal_remaining_seconds="0" if final else "3",
                   next_signal_duration_seconds="0" if final else "2",
                   snapshot_clock_schema=contract.CLOCK_SCHEMA, signal_clock_schema=contract.CLOCK_SCHEMA,
                   snapshot_clock_step_seconds="1", signal_step_seconds="1",
                   signal_api_return_time=str(observed + 1), signal_snapshot_time=str(observed),
                   observed_enter_time=str(observed), signal_reference_time=str(observed),
                   signal_projection_seconds="0", v2_enter=str(observed), v2_exit=str(observed + 3),
                   feature_snapshot_lag_seconds="0")
        row.update({key: str(label) for key in contract.LABEL_COLUMNS})
        result.append(row)
    return result


class ReleaseFixture:
    def __init__(self, root, schema="native-collection-batch-v2", train_count=2, validation_count=1):
        self.root = Path(root)
        self.root.mkdir()
        self.schema = schema
        source_hash = "a" * 64
        sources = {"collection/native_boundary_v2/collect_native_v2.py": source_hash,
                   "collection/run_manhattan_training_batch_v2.py": "b" * 64}
        self.scenarios = []
        for split, count in (("train", train_count), ("validation", validation_count)):
            for index in range(count):
                sid = split + "_" + str(index)
                self.scenarios.append({"scenario_id": sid, "family_id": split + "_family", "split": split,
                    "vehicle_count": 1, "network": "/original/net.xml", "network_sha256": "c" * 64,
                    "paths": {"route": "/original/" + sid + ".rou.xml", "config": "/original/" + sid + ".sumocfg",
                              "manifest": "/original/" + sid + ".json"},
                    "hashes": {"route": "d" * 64, "config": "e" * 64, "manifest": "f" * 64}})
        self.contract = {"pilot": False, "dataset_sha256": "1" * 64, "collector_sources_sha256": sources,
            "collector_sha256": source_hash, "runner_sha256": "b" * 64,
            "scenarios": self.scenarios, "scenario_ids": [row["scenario_id"] for row in self.scenarios]}
        write_json(self.root / "batch_manifest.json", {"schema": schema, "contract": self.contract})
        write_json(self.root / "status.json", {"schema": schema, "status": "COMPLETE", "pilot": False,
            "active": [], "pending": [], "failures": [],
            "scenarios": {row["scenario_id"]: "COMPLETE" for row in self.scenarios}})
        for index, row in enumerate(self.scenarios):
            sid = row["scenario_id"]
            job = self.root / "jobs" / sid
            result = job / "result"
            result.mkdir(parents=True)
            prepared = {"/original/net.xml": "c" * 64, row["paths"]["route"]: "d" * 64,
                        "/prepared/" + sid + ".sumocfg": "2" * 64}
            spec = {"schema": schema, "scenario_id": sid, "contract_sha256": contract.canonical_digest(self.contract),
                    "source_scenario": row, "expected_vehicles": 1, "pilot": False, "pilot_vehicle_ids": None,
                    "collector_sources_sha256": sources, "collector_sha256": source_hash, "runner_sha256": "b" * 64,
                    "prepared_sha256": prepared, "network": "/original/net.xml", "route": row["paths"]["route"],
                    "config": "/prepared/" + sid + ".sumocfg"}
            write_json(job / "job.json", spec)
            run = {"schema": "sumo-native-collection-run-v2", "contract": copy.deepcopy(contract.COLLECTION_SEMANTICS),
                   "inputs": prepared, "sources": sources, "expected_vehicles": 1, "expected_road_occurrences": 3,
                   "no_hard_end": True, "behavior_setters_used": False, "step_seconds": 1., "sumo_version": [22, "SUMO 1.27.1"]}
            write_json(result / "manifest.json", run)
            write_rows(result / "training.csv.gz", make_rows(sid, 10 + index))
            for name in ("road_visits.csv.gz", "tripinfo.xml", "vehroute.xml.gz"):
                (result / name).write_bytes(b"closed-test-evidence")
            counts = {"expected_vehicles": 1, "arrived": 1, "tripinfo_count": 1, "normal_arrived": 1,
                      "collisions": 0, "teleports": 0, "missing_ids": 0, "missing_runtime_sf": 0,
                      "trainable": 3, "excluded": 0, "road_occurrences": 3}
            outputs = {name: contract.sha256_file(result / name) for name in
                       ("training.csv.gz", "road_visits.csv.gz", "tripinfo.xml", "vehroute.xml.gz")}
            audit = {"status": "COMPLETE", "error": None, "native_outputs_closed": True,
                     "training_released": True, "explicit_speed_factor_audited": True, "counts": counts,
                     "contract": run["contract"], "manifest_sha256": contract.sha256_file(result / "manifest.json"),
                     "closed_outputs_sha256": outputs, "quality": {"pending_terminal_vehicles": 0}}
            write_json(result / "audit.json", audit)
            status = {"state": "COMPLETE", "returncode": 0, "audit_status": "COMPLETE", "gate_problems": [],
                      "scenario_id": sid, "contract_sha256": spec["contract_sha256"],
                      "job_spec_sha256": contract.sha256_file(job / "job.json"), "counts": counts,
                      "audit_sha256": contract.sha256_file(result / "audit.json"),
                      "verified_bindings": {"manifest_sha256": audit["manifest_sha256"], "closed_outputs_sha256": outputs}}
            write_json(job / "status.json", status)

    def edit(self, relative, callback):
        path = self.root / relative
        value = data.read_json(path)
        callback(value)
        write_json(path, value)

    def rebind(self, sid):
        job = self.root / "jobs" / sid
        result = job / "result"
        audit = data.read_json(result / "audit.json")
        audit["manifest_sha256"] = contract.sha256_file(result / "manifest.json")
        for name in audit["closed_outputs_sha256"]:
            audit["closed_outputs_sha256"][name] = contract.sha256_file(result / name)
        write_json(result / "audit.json", audit)
        status = data.read_json(job / "status.json")
        status.update(job_spec_sha256=contract.sha256_file(job / "job.json"), counts=audit["counts"],
                      audit_sha256=contract.sha256_file(result / "audit.json"),
                      verified_bindings={"manifest_sha256": audit["manifest_sha256"],
                                         "closed_outputs_sha256": audit["closed_outputs_sha256"]})
        write_json(job / "status.json", status)


class NativeTrainingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="native_training_test_")
        self.base = Path(self.temp.name)
        self.fixture = ReleaseFixture(self.base / "release")

    def tearDown(self):
        self.temp.cleanup()

    def load(self):
        return data.load_batch(self.fixture.root)

    def args(self, *extra):
        return train.build_parser().parse_args(["--batch-root", str(self.fixture.root), "--output-dir",
            str(self.base / "model"), "--sample-weighting", "none", "--memory-budget-gb", "4", "--chunk-size", "2", *extra])

    def test_generic_release_small_count_fixed_families_and_six_labels(self):
        batch = self.load()
        self.assertEqual(batch.provenance["split_counts"], {"train": 2, "validation": 1})
        report = data.preflight(batch, chunk_size=1)
        for label in contract.LABEL_COLUMNS:
            self.assertEqual(report["rows_by_split_and_label"]["train"][label],
                             {"positive": 4, "zero_policy_excluded": 2, "positive_subsecond": 2})
        self.assertEqual(len(report["scenarios"]), 3)

    def test_current_manhattan_tag_same_structure(self):
        other = ReleaseFixture(self.base / "named", schema="manhattan-native-collection-batch-v2", train_count=1)
        self.assertEqual(len(data.load_batch(other.root).scenarios), 2)

    def test_expected_split_count(self):
        with self.assertRaisesRegex(ValueError, "scenario count"):
            data.load_batch(self.fixture.root, expected_train=36)

    def test_pilot_and_incomplete_rejected(self):
        self.fixture.edit("batch_manifest.json", lambda value: value["contract"].update(pilot=True))
        with self.assertRaisesRegex(ValueError, "Pilot"):
            self.load()
        self.fixture.edit("batch_manifest.json", lambda value: value["contract"].update(pilot=False))
        self.fixture.edit("status.json", lambda value: value.update(status="RUNNING", active=["train_0"]))
        with self.assertRaisesRegex(ValueError, "Entire batch"):
            self.load()

    def test_foreign_batch_schema_rejected(self):
        self.fixture.edit("batch_manifest.json", lambda value: value.update(schema="native-v1"))
        with self.assertRaisesRegex(ValueError, "current native"):
            self.load()

    def test_digest_and_closed_output_tampering(self):
        (self.fixture.root / "jobs/train_0/result/tripinfo.xml").write_bytes(b"tamper")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.load()

    def test_missing_release_rejected_even_with_rebound_hashes(self):
        self.fixture.edit("jobs/train_0/result/audit.json", lambda value: value.update(training_released=False))
        self.fixture.rebind("train_0")
        with self.assertRaisesRegex(ValueError, "quality-released"):
            self.load()

    def test_missing_error_null_rejected(self):
        self.fixture.edit("jobs/train_0/result/audit.json", lambda value: value.pop("error"))
        self.fixture.rebind("train_0")
        with self.assertRaisesRegex(ValueError, "quality-released"):
            self.load()

    def test_mismatched_source_contract_rejected(self):
        self.fixture.edit("jobs/train_0/result/manifest.json", lambda value: value["contract"].update(flow="shadow"))
        self.fixture.rebind("train_0")
        with self.assertRaisesRegex(ValueError, "collection semantics"):
            self.load()

    def test_same_family_across_split_rejected(self):
        batch = data.read_json(self.fixture.root / "batch_manifest.json")
        batch["contract"]["scenarios"][-1]["family_id"] = "train_family"
        write_json(self.fixture.root / "batch_manifest.json", batch)
        digest = contract.canonical_digest(batch["contract"])
        for row in batch["contract"]["scenarios"]:
            sid = row["scenario_id"]
            self.fixture.edit("jobs/" + sid + "/job.json", lambda value: value.update(contract_sha256=digest, source_scenario=row))
            self.fixture.edit("jobs/" + sid + "/status.json", lambda value: value.update(contract_sha256=digest))
            self.fixture.rebind(sid)
        with self.assertRaisesRegex(ValueError, "families"):
            self.load()

    def test_invalid_quality_and_schema(self):
        scenario = self.load().scenarios[0]
        for key, value in (("trainable", "0"), ("v2_label_valid", "0"), ("schema", "old-schema")):
            with self.subTest(key=key):
                raw = pd.DataFrame(make_rows("x"))
                raw.loc[0, key] = value
                with self.assertRaisesRegex(ValueError, "low-quality"):
                    data.validate_chunk(raw, scenario, {})

    def test_unknown_negative_nonmonotone_targets(self):
        scenario = self.load().scenarios[0]
        for value in ("", "NaN", "-1", "100"):
            raw = pd.DataFrame(make_rows("x"))
            raw.loc[0, contract.LABEL_COLUMNS[0]] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                data.validate_chunk(raw, scenario, {})

    def test_cross_chunk_duplicate_and_post_final(self):
        scenario = self.load().scenarios[0]
        raw = pd.DataFrame(make_rows("x"))
        seen = {}
        data.validate_chunk(raw.iloc[:1], scenario, seen)
        with self.assertRaisesRegex(ValueError, "across chunks"):
            data.validate_chunk(raw.iloc[:1], scenario, seen)
        seen = {}
        data.validate_chunk(raw, scenario, seen)
        extra = raw.iloc[:1].copy()
        extra["route_index"] = "3"
        with self.assertRaisesRegex(ValueError, "across chunks"):
            data.validate_chunk(extra, scenario, seen)

    def test_nonfinal_sentinel_and_clock_mismatch(self):
        scenario = self.load().scenarios[0]
        raw = pd.DataFrame(make_rows("x"))
        raw.loc[0, "signal_api_return_time"] = "2"
        with self.assertRaisesRegex(ValueError, "clock"):
            data.validate_chunk(raw, scenario, {})
        raw = pd.DataFrame(make_rows("x"))
        raw.loc[0, "entry_signal_remaining_seconds"] = "0"
        raw.loc[0, "next_signal_duration_seconds"] = "0"
        raw.loc[0, "next_signal_type"] = "G"
        with self.assertRaisesRegex(ValueError, "final roads"):
            data.validate_chunk(raw, scenario, {})

    def test_training_bytes_rechecked_after_preflight(self):
        batch = self.load()
        with (batch.scenarios[0].training_path).open("ab") as stream:
            stream.write(b"changed")
        memory = train.memory_plan(4, 4, 2, 0, 0)
        with self.assertRaisesRegex(ValueError, "SHA256"):
            train.materialize_training(batch, contract.LABEL_COLUMNS[0], 4, 2, memory)

    def test_budget_fails_before_allocation(self):
        batch = self.load()
        memory = train.memory_plan(4, .001, 2, 0, 0)
        with patch("training.train_road_time.np.empty", side_effect=AssertionError("must not allocate")):
            with self.assertRaisesRegex(ValueError, "before materialization"):
                train.materialize_training(batch, contract.LABEL_COLUMNS[0], 4, 2, memory)

    def test_chunk_budget_checked_before_batch_read(self):
        args = self.args("--chunk-size", "100000000")
        with patch("training.train_road_time.load_batch", side_effect=AssertionError("must not read")):
            with self.assertRaisesRegex(ValueError, "preflight chunk"):
                train.run(args)

    def test_invalid_random_state_rejected_before_batch_read(self):
        for value in (-1, 2**32, True, 1.5, None):
            with self.subTest(value=value):
                args = self.args()
                args.random_state = value
                with patch("training.train_road_time.load_batch", side_effect=AssertionError("must not read")):
                    with self.assertRaisesRegex(ValueError, "random-state"):
                        train.run(args)

    def test_invalid_random_state_rejected_before_fit_materialization(self):
        args = self.args()
        args.random_state = -1
        with patch("training.train_road_time.materialize_training", side_effect=AssertionError("must not allocate")):
            with self.assertRaisesRegex(ValueError, "random-state"):
                train.fit_one(None, None, "1", self.base / "model", args, None, predictor_class=object)
        self.assertFalse((self.base / "model").exists())

    def test_random_state_domain_includes_uint32_endpoints(self):
        for value in (0, 42, 2**32 - 1):
            with self.subTest(value=value):
                train._validate_random_state(value)
                # A local generator checks NumPy's seed domain without changing
                # the test process's global state or fitting any model.
                np.random.RandomState(value)

    def test_materialize_no_metadata_leak_and_positive_subsecond_retained(self):
        batch = self.load()
        memory = train.memory_plan(4, 4, 2, 0, 0)
        frame = train.materialize_training(batch, contract.LABEL_COLUMNS[0], 4, 2, memory)
        self.assertEqual(list(frame), contract.FEATURE_COLUMNS + [contract.MODEL_LABEL])
        self.assertEqual(frame[contract.MODEL_LABEL].tolist(), [2., .5, 2., .5])
        for key, categories in contract.CATEGORICAL_FEATURES.items():
            self.assertEqual(frame[key].cat.categories.tolist(), categories)
            self.assertFalse(frame[key].cat.ordered)

    def test_weighting_normalization_no_additional_input(self):
        frame = pd.DataFrame({"road_flow": [1, 4, 5, 9]})
        weight, audit = train.sample_weights(frame, "low-flow")
        np.testing.assert_allclose(weight, [1.5, 1.5, .5, .5])
        self.assertEqual(audit["external_metrics"], "unweighted")
        self.assertEqual(list(frame), ["road_flow"])
        self.assertEqual(train.sample_weights(frame, "none")[0], None)

    def test_stream_metrics_match_whole_vector_and_no_fake_p95(self):
        batch = self.load()
        class Predictor:
            def predict(self, frame):
                self.columns = list(frame)
                return np.ones(len(frame))
        predictor = Predictor()
        metrics = train.evaluate_streaming(batch, predictor, contract.LABEL_COLUMNS[0], 1)
        expected = np.array([1. - 2., 1. - .5])
        self.assertAlmostEqual(metrics["overall"]["mae"], np.abs(expected).mean())
        self.assertAlmostEqual(metrics["overall"]["bias"], expected.mean())
        self.assertAlmostEqual(metrics["overall"]["rmse"], np.sqrt(np.mean(expected**2)))
        self.assertNotIn("p95_absolute_error", metrics["overall"])
        self.assertEqual(predictor.columns, contract.FEATURE_COLUMNS)

    def test_dry_run_no_autogluon_no_writes_and_fixed_order(self):
        args = self.args("--dry-run", "--all-thresholds")
        result = train.run(args)
        self.assertEqual(result["threshold_order"], ["1", "3", "0p1", "0p5", "1p5", "2"])
        self.assertFalse(Path(args.output_dir).exists())

    def test_explicit_weighting_and_budget_required(self):
        with self.assertRaises(SystemExit):
            train.build_parser().parse_args(["--batch-root", "x", "--output-dir", "y"])

    def test_fake_fit_complete_contract_and_validation_is_not_fit_input(self):
        class FakePredictor:
            last_kwargs = None
            last_constructor_kwargs = None
            last_columns = None
            def __init__(self, **kwargs):
                FakePredictor.last_constructor_kwargs = kwargs
                self.path = Path(kwargs["path"])
                self.label = kwargs["label"]
                self.feature_metadata = type("Metadata", (), {"get_features": lambda self: contract.FEATURE_COLUMNS})()
            def fit(self, **kwargs):
                FakePredictor.last_kwargs = kwargs
                FakePredictor.last_columns = list(kwargs["train_data"])
                self.path.mkdir()
                (self.path / "mock_model").write_bytes(b"not-a-real-model")
                return self
            def predict(self, frame):
                return np.ones(len(frame))
        batch = self.load()
        report = data.preflight(batch, 2)
        args = self.args("--random-state", "42")
        memory = train.memory_plan(4, 4, 2, 10, 2)
        output = self.base / "model"
        with patch("training.train_road_time.random.seed") as python_seed, patch("training.train_road_time.np.random.seed") as numpy_seed:
            result = train.fit_one(batch, report, "1", output, args, memory, predictor_class=FakePredictor)
        python_seed.assert_called_once_with(42)
        numpy_seed.assert_called_once_with(42)
        self.assertEqual(FakePredictor.last_constructor_kwargs["learner_kwargs"], {"random_state": 42})
        value = data.read_json(output / "model_contract.json")
        self.assertEqual(contract.validate_model_contract(value)["status"], "COMPLETE")
        self.assertEqual(result["validation"]["n"], 2)
        self.assertEqual(FakePredictor.last_columns, contract.FEATURE_COLUMNS + [contract.MODEL_LABEL])
        self.assertNotIn("tuning_data", FakePredictor.last_kwargs)
        self.assertEqual(FakePredictor.last_kwargs["num_gpus"], 0)
        self.assertEqual(FakePredictor.last_kwargs["memory_limit"], 4)
        self.assertIn("mock_model", value["predictor_files_sha256"])
        self.assertEqual(set(value["training_sources_sha256"]),
                         {"train_road_time.py", "native_contract.py", "native_data.py", "native_release.py"})
        self.assertEqual(value["training_sources_sha256"]["native_release.py"],
                         contract.sha256_file(Path(train.__file__).with_name("native_release.py")))
        self.assertEqual(value["training_options"]["random_state"], 42)

    def test_full_36_6_size_is_optional_not_hardcoded(self):
        fixture = ReleaseFixture(self.base / "full_count", train_count=36, validation_count=6)
        batch = data.load_batch(fixture.root, expected_train=36, expected_validation=6)
        self.assertEqual(len(batch.scenarios), 42)
        report = data.preflight(batch, 2)
        self.assertEqual(report["rows_by_split_and_label"]["train"][contract.LABEL_COLUMNS[0]]["positive"], 72)

    def test_truncated_gzip_fails_even_if_audit_hash_was_rebound(self):
        path = self.fixture.root / "jobs/train_0/result/training.csv.gz"
        payload = path.read_bytes()
        path.write_bytes(payload[:-12])
        self.fixture.rebind("train_0")
        batch = self.load()
        with self.assertRaises((EOFError, OSError)):
            data.preflight(batch, 1)

    def test_audit_row_count_mismatch_rejected(self):
        self.fixture.edit("jobs/train_0/result/audit.json", lambda value: value["counts"].update(trainable=2, excluded=1))
        self.fixture.rebind("train_0")
        with self.assertRaisesRegex(ValueError, "row count"):
            data.preflight(self.load(), 2)

    def test_false_complete_model_contract_rejected(self):
        with self.assertRaisesRegex(ValueError, "status"):
            contract.validate_model_contract({"schema": contract.MODEL_SCHEMA, "status": "FIT_STARTED"})

    def test_constant_pruning_is_recorded_without_changing_raw_nine(self):
        class FakePredictor:
            def __init__(self, **kwargs):
                self.path, self.label = Path(kwargs["path"]), kwargs["label"]
                self.feature_metadata = type("Metadata", (), {"get_features": lambda self: ["road_length"]})()
            def fit(self, **kwargs):
                self.path.mkdir()
                (self.path / "mock_model").write_bytes(b"fake")
                return self
            def predict(self, frame):
                return np.ones(len(frame))
        batch = self.load()
        report = data.preflight(batch, 2)
        train.fit_one(batch, report, "1", self.base / "model", self.args(),
                      train.memory_plan(4, 4, 2, 10, 2), predictor_class=FakePredictor)
        value = data.read_json(self.base / "model/model_contract.json")
        self.assertEqual(value["feature_columns"], contract.FEATURE_COLUMNS)
        self.assertEqual(value["framework_raw_features"], ["road_length"])
        self.assertEqual(len(value["framework_removed_raw_features"]), 8)
        contract.validate_model_contract(value)

    def test_unapproved_framework_feature_marks_failed_model(self):
        class FakePredictor:
            def __init__(self, **kwargs):
                self.path, self.label = Path(kwargs["path"]), kwargs["label"]
                self.feature_metadata = type("Metadata", (), {"get_features": lambda self: ["vehicle_id"]})()
            def fit(self, **kwargs):
                return self
        batch = self.load()
        report = data.preflight(batch, 2)
        with self.assertRaisesRegex(ValueError, "unapproved"):
            train.fit_one(batch, report, "1", self.base / "model", self.args(),
                          train.memory_plan(4, 4, 2, 10, 2), predictor_class=FakePredictor)
        value = data.read_json(self.base / "model/model_contract.json")
        self.assertEqual(value["status"], "FAILED")

    def test_shared_threshold_directory(self):
        self.assertEqual([contract.model_directory_name(value) for value in contract.THRESHOLD_ORDER],
                         ["gt_1", "gt_3", "gt_0p1", "gt_0p5", "gt_1p5", "gt_2"])

    def test_duplicate_json_keys_rejected(self):
        path = self.base / "duplicate.json"
        path.write_text('{"status":"COMPLETE","status":"FAILED"}', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Duplicate JSON"):
            data.read_json(path)

    def test_unsafe_closed_relative_path_rejected(self):
        for path in ("../training.csv.gz", "/training.csv.gz", "C:/training.csv.gz", "x\\y"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                data.safe_file(self.fixture.root, path)


if __name__ == "__main__":
    unittest.main()
