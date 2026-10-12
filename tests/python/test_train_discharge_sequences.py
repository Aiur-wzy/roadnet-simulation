"""Deterministic training-contract adversarial tests; no SUMO or large data."""

import copy
import json
from pathlib import Path
import unittest

import training.train_discharge_sequences as training


def sequence(run="r0", seq="s0", lane="a_0", anchor=0.0, times=(1.0, 3.0, 4.5),
             ordinal=1, cohort_size=None, turns=None):
    passages = []
    turns = turns or ["s"] * len(times)
    for j, (time, turn) in enumerate(zip(times, turns), 1):
        passages.append({"run_id": run, "lane_id": lane, "section_id": "exit",
                         "vehicle_key": "{}:{}".format(seq, j), "vehicle_id": "v" + str(j),
                         "time": time, "lane_passage_ordinal": ordinal + j - 1, "local_j": j,
                         "movement_id": "m_" + turn, "turn": turn, "type_id": "car"})
    headways = []
    for previous, current in zip(passages, passages[1:]):
        headways.append({"local_j": current["local_j"], "headway": current["time"] - previous["time"],
                         "prev_vehicle_key": previous["vehicle_key"], "vehicle_key": current["vehicle_key"],
                         "prev_time": previous["time"], "time": current["time"],
                         "prev_ordinal": previous["lane_passage_ordinal"], "ordinal": current["lane_passage_ordinal"],
                         "prev_turn": previous["turn"], "turn": current["turn"],
                         "prev_type_id": previous["type_id"], "type_id": current["type_id"]})
    return {"schema": training.SEQUENCE_SCHEMA, "rule_version": training.RULE_VERSION,
            "run_id": run, "lane_id": lane,
            "section_id": "exit", "sequence_id": seq, "start_kind": "stopped_green_start",
            "anchor_time": anchor, "cut_time": (times[-1] if times else anchor) + 0.5,
            "cut_reason": "observation_end", "cohort_size": cohort_size if cohort_size is not None else len(times),
            "valid_start": True, "closed": True, "passages": passages, "headways": headways,
            "startup_delay": times[0] - anchor if times else None}


def write_lines(path, rows):
    path.write_text("".join(json.dumps(row, allow_nan=False) + "\n" for row in rows), encoding="utf-8")


def dataset(root, run="r0", sequences=None, audit=None, domain=None):
    target = root / run
    target.mkdir()
    sequences = sequences if sequences is not None else [sequence(run=run)]
    audit = audit if audit is not None else [copy.deepcopy(p) for row in sequences for p in row["passages"]]
    manifest = {"schema": training.DATASET_SCHEMA, "complete": True, "run_id": run,
                "config": {"epsilon_zero": 1e-7}, "mode": "replay", "rule_version": training.RULE_VERSION}
    if domain is not None:
        manifest["domain"] = domain
    training.write_json(target / "manifest.json", manifest)
    write_lines(target / "sequences.jsonl", sequences)
    write_lines(target / "passages.jsonl", audit)
    raw = [{"run_id": run, "event": "start", "sequence_id": row["sequence_id"],
            "lane_id": row["lane_id"], "time": row["anchor_time"]} for row in sequences]
    raw += [dict(p, event="passage") for p in audit]
    write_lines(target / "raw_events.jsonl", raw)
    return target


class SequenceValidationTests(unittest.TestCase):
    def test_valid_mixed_turn_lane_prefix(self):
        row = sequence(turns=["s", "l", "s"])
        self.assertIs(training.validate_sequence(row, "r0"), row)

    def test_zero_car_failed_start_allowed(self):
        row = sequence(times=(), cohort_size=4)
        training.validate_sequence(row, "r0")
        row.update(valid_start=False, cut_time=0.0)
        training.validate_sequence(row, "r0")

    def test_invalid_start_cannot_retain_labels(self):
        row = sequence()
        row["valid_start"] = False
        with self.assertRaisesRegex(training.DatasetError, "empty audit"):
            training.validate_sequence(row, "r0")

    def test_cohort_order_cannot_be_rewritten(self):
        row = sequence()
        row["cohort_vehicle_keys"] = ["s0:1", "s0:3", "s0:2"]
        with self.assertRaisesRegex(training.DatasetError, "original cohort"):
            training.validate_sequence(row, "r0")

    def test_corruptions_rejected(self):
        mutations = [
            lambda r: r.update(closed=False),
            lambda r: r.update(run_id="other"),
            lambda r: r.update(cohort_size=1),
            lambda r: r.update(startup_delay=2),
            lambda r: r.update(startup_delay=float("nan")),
            lambda r: r.update(valid_start="true"),
            lambda r: r["passages"][1].update(local_j=3),
            lambda r: r["passages"][1].update(lane_passage_ordinal=4),
            lambda r: r["passages"][1].update(time=1),
            lambda r: r["passages"][1].update(lane_id="other"),
            lambda r: r["passages"][1].update(vehicle_key="s0:1"),
            lambda r: r["headways"][0].update(headway=0),
            lambda r: r["headways"][0].update(headway=10),
            lambda r: r["headways"][0].update(prev_vehicle_key="different"),
            lambda r: r["headways"][0].update(local_j=True),
            lambda r: r["headways"].pop(),
        ]
        for mutate in mutations:
            row = sequence()
            mutate(row)
            with self.subTest(mutate=mutate), self.assertRaises(training.DatasetError):
                training.validate_sequence(row, "r0")

    def test_hard_boundary_is_exclusive_natural_end_inclusive(self):
        row = sequence()
        row["cut_time"] = row["passages"][-1]["time"]
        for reason in ("cohort_end", "observation_end"):
            row["cut_reason"] = reason
            training.validate_sequence(row, "r0")
        for reason in ("restop", "unknown", "run_end"):
            row["cut_reason"] = reason
            with self.assertRaisesRegex(training.DatasetError, "interference boundary"):
                training.validate_sequence(row, "r0")

    def test_zero_startup_is_valid_distinct_from_missing(self):
        row = sequence(times=(0, 2, 3))
        training.validate_sequence(row, "r0")
        self.assertEqual(row["startup_delay"], 0)


class TrainingTests(unittest.TestCase):
    def setUp(self):
        self.tmp = training.temporary_workspace(Path.cwd(), ".discharge_train_tests_")
        self.root = self.tmp.__enter__()

    def tearDown(self):
        self.tmp.__exit__(None, None, None)

    def train(self, paths, **kwargs):
        output = self.root / ("model" + str(len(list(self.root.glob("model*")))))
        defaults = {"prefix_length": 3, "min_samples": 1, "min_sequences": 1}
        defaults.update(kwargs)
        result = training.train_datasets(paths, output, **defaults)
        return output, result

    def test_single_run_export_is_explicitly_unvalidated(self):
        path = dataset(self.root)
        output, manifest = self.train([path])
        self.assertEqual(manifest["validation_status"], "unavailable_insufficient_runs")
        model = training.read_json(output / "model.json")
        predict = training.Predictor(model)
        context = dict(start_kind="stopped_green_start", turn="s", type_id="car")
        self.assertEqual(predict.predict("headway", 2, context)[0], 2)
        self.assertEqual(predict.predict("headway", 3, context)[0], 1.5)
        self.assertEqual(predict.predict("headway", 4, context), (2, "fixed"))
        self.assertEqual(predict.predict("startup", 1, context)[0], 1)
        self.assertEqual(model["feature_order"], ["permission_class", "start_kind", "turn", "type_id"])
        self.assertEqual(model["semantics"]["lane_count_scaling"], "none")

    def test_whole_holdout_is_not_used_for_fit(self):
        train = dataset(self.root, "train", [sequence(run="train", times=(1, 3, 4.5))])
        held = dataset(self.root, "held", [sequence(run="held", times=(1, 13, 24.5))])
        output, manifest = self.train([held, train], holdout_runs=["held"])
        self.assertEqual(manifest["split"]["assignments"], {"held": "validation", "train": "train"})
        metrics = training.read_json(output / "metrics.json")["validation"]
        h_all = next(row for row in metrics["strata"] if row["target"] == "headway" and row["dimension"] == "all")
        self.assertEqual(h_all["mae"], 10)
        self.assertEqual(h_all["bias"], -10)
        self.assertEqual(metrics["cumulative_headway_from_first_passage"]["3"]["bias"], -20)

    def test_tail_is_only_fit_from_j_beyond_prefix(self):
        path = dataset(self.root, sequences=[sequence(times=(1, 5, 8, 10, 11.5))])
        output, _ = self.train([path], prefix_length=3)
        model = training.read_json(output / "model.json")
        context = dict(start_kind="stopped_green_start", turn="s", type_id="car")
        predictor = training.Predictor(model)
        self.assertEqual(predictor.predict("headway", 3, context)[0], 3)
        self.assertEqual(predictor.predict("headway", 4, context)[0], 1.75)
        self.assertEqual(predictor.predict("headway", 100, context)[0], 1.75)

    def test_prefix_fit_does_not_downweight_long_episodes(self):
        short = sequence(seq="short", lane="a", times=(1, 11))
        long = sequence(seq="long", lane="b", times=(1, 2, 3, 4, 5, 6))
        path = dataset(self.root, sequences=[short, long])
        output, _ = self.train([path])
        model = training.read_json(output / "model.json")
        global_j2 = next(row for row in model["rows"] if row["target"] == "headway" and row["bucket"] == "2" and row["fields"] == ["permission_class"])
        self.assertAlmostEqual(global_j2["value"], 5.5)

    def test_tail_fit_weights_each_episode_once(self):
        short = sequence(seq="short", lane="a", times=(0, 1, 2))
        long = sequence(seq="long", lane="b", times=(0, 1, 11, 21, 31, 41))
        path = dataset(self.root, sequences=[short, long])
        output, _ = self.train([path], prefix_length=2)
        model = training.read_json(output / "model.json")
        global_tail = next(row for row in model["rows"] if row["target"] == "headway" and row["bucket"] == "tail" and row["fields"] == ["permission_class"])
        self.assertAlmostEqual(global_tail["value"], 5.5)
        self.assertAlmostEqual(global_tail["effective_episode_weight"], 2)

    def test_sparse_startup_is_missing_and_headway_fixed(self):
        path = dataset(self.root)
        output, _ = self.train([path], min_sequences=2)
        model = training.read_json(output / "model.json")
        predictor = training.Predictor(model)
        context = dict(start_kind="stopped_green_start", turn="s", type_id="car")
        self.assertEqual(predictor.predict("startup", 1, context), (None, "unavailable"))
        self.assertEqual(predictor.predict("headway", 2, context), (2, "fixed"))

    def test_startup_only_retained_and_zero_starts_in_denominator(self):
        first = sequence(times=(1,), cohort_size=3)
        failed = sequence(seq="failed", lane="other", times=(), cohort_size=5)
        path = dataset(self.root, sequences=[first, failed])
        output, _ = self.train([path])
        report = training.read_json(output / "coverage.json")
        self.assertEqual((report["startup_targets"], report["headway_targets"], report["zero_passage_count"]), (1, 0, 1))
        rows = list(training.json_lines(output / "coverage_by_lane.jsonl"))
        failed_coverage = next(row for _, row in rows if row["lane_id"] == "other")
        self.assertEqual(failed_coverage["prefix_coverage"][0]["risk_sequences"], 1)
        self.assertEqual(failed_coverage["prefix_coverage"][0]["observed_prefix_sequences"], 0)

    def test_empty_target_set_is_rejected(self):
        path = dataset(self.root, sequences=[sequence(times=(), cohort_size=3)])
        with self.assertRaisesRegex(training.DatasetError, "no completed"):
            self.train([path])

    def test_duplicate_run_rejected(self):
        path = dataset(self.root)
        with self.assertRaisesRegex(training.DatasetError, "duplicate run_id"):
            self.train([path, path])

    def test_renamed_same_raw_run_rejected(self):
        a = dataset(self.root, "a", [sequence(run="a")])
        b = dataset(self.root, "b", [sequence(run="b")])
        with self.assertRaisesRegex(training.DatasetError, "different run IDs"):
            self.train([a, b])

    def test_domain_mismatch_rejected(self):
        a = dataset(self.root, "a", [sequence(run="a")], domain={"step": 0.1})
        b = dataset(self.root, "b", [sequence(run="b", times=(1, 4, 6))], domain={"step": 1})
        with self.assertRaisesRegex(training.DatasetError, "incompatible collection domains"):
            self.train([a, b])

    def test_same_simulation_identity_rejected(self):
        paths = [dataset(self.root, run, [sequence(run=run, times=times)])
                 for run, times in (("a", (1, 3, 5)), ("b", (1, 4, 7)))]
        for path in paths:
            manifest = training.read_json(path / "manifest.json")
            manifest["simulation_identity"] = "same_input_seed"
            training.write_json(path / "manifest.json", manifest)
        with self.assertRaisesRegex(training.DatasetError, "same physical simulation"):
            self.train(paths)

    def test_dataset_hash_tampering_rejected(self):
        path = dataset(self.root)
        manifest = training.read_json(path / "manifest.json")
        manifest["file_sha256"] = {"sequences.jsonl": "0" * 64}
        training.write_json(path / "manifest.json", manifest)
        with self.assertRaisesRegex(training.DatasetError, "declared SHA256"):
            self.train([path])

    def test_raw_passage_mismatch_rejected(self):
        row = sequence()
        audit = copy.deepcopy(row["passages"])
        audit[1]["time"] += 0.2
        path = dataset(self.root, sequences=[row], audit=audit)
        with self.assertRaisesRegex(training.DatasetError, "original audit stream"):
            self.train([path])

    def test_raw_ordinal_hole_not_repaired(self):
        row = sequence(times=(1,))
        audit = copy.deepcopy(row["passages"])
        extra = dict(audit[0], vehicle_key="later", lane_passage_ordinal=3, time=5)
        path = dataset(self.root, sequences=[row], audit=audit + [extra])
        with self.assertRaisesRegex(training.DatasetError, "missing ordinal"):
            self.train([path])

    def test_overlapping_segments_rejected(self):
        a = sequence(times=(1, 2), cohort_size=2)
        b = sequence(seq="b", anchor=1.5, times=(3, 4), ordinal=3)
        path = dataset(self.root, sequences=[a, b])
        with self.assertRaisesRegex(training.DatasetError, "overlapping"):
            self.train([path])

    def test_duplicate_sequence_rejected(self):
        row = sequence()
        path = dataset(self.root, sequences=[row, copy.deepcopy(row)], audit=row["passages"])
        with self.assertRaisesRegex(training.DatasetError, "UNIQUE"):
            self.train([path])

    def test_incomplete_manifest_rejected(self):
        path = dataset(self.root)
        manifest = training.read_json(path / "manifest.json")
        manifest["complete"] = False
        training.write_json(path / "manifest.json", manifest)
        with self.assertRaisesRegex(training.DatasetError, "complete=true"):
            self.train([path])

    def test_existing_output_preserved(self):
        path = dataset(self.root)
        output, _ = self.train([path])
        before = training.sha256_file(output / "model.json")
        with self.assertRaisesRegex(training.DatasetError, "never overwritten"):
            training.train_datasets([path], output)
        self.assertEqual(before, training.sha256_file(output / "model.json"))

    def test_weighted_median_supported(self):
        path = dataset(self.root, sequences=[sequence(times=(1, 5, 8, 10, 11))])
        output, _ = self.train([path], estimator="median")
        model = training.read_json(output / "model.json")
        self.assertEqual(model["estimator"], "median")
        self.assertTrue(all(row["value"] >= 0 for row in model["rows"]))

    def test_rolling_lookup_cannot_borrow_stopped_prefix(self):
        path = dataset(self.root, sequences=[sequence(times=(1, 10, 12))])
        output, _ = self.train([path])
        predictor = training.Predictor(training.read_json(output / "model.json"))
        context = dict(start_kind="rolling", turn="s", type_id="car")
        self.assertEqual(predictor.predict("headway", 2, context), (2, "fixed_nonstartup"))
        self.assertEqual(predictor.predict("startup", 1, context), (None, "unavailable_nonstartup"))

    def test_model_schema_and_values_are_checked(self):
        path = dataset(self.root)
        output, _ = self.train([path])
        model = training.read_json(output / "model.json")
        mutations = [lambda m: m.update(schema="unexpected"),
                     lambda m: m.update(fallback_headway=float("inf")),
                     lambda m: m["rows"].append(copy.deepcopy(m["rows"][0])),
                     lambda m: m["rows"][0].update(value=-1),
                     lambda m: m["rows"][0].update(bucket="invalid"),
                     lambda m: m.update(feature_order=["future_sequence_length"])]
        for mutate in mutations:
            changed = copy.deepcopy(model)
            mutate(changed)
            with self.subTest(mutate=mutate), self.assertRaises(training.DatasetError):
                training.Predictor(changed)

    def test_same_time_raw_anomaly_is_kept_outside_prefix(self):
        row = sequence(times=(1,))
        audit = copy.deepcopy(row["passages"])
        audit.append(dict(audit[0], vehicle_key="uncertain", lane_passage_ordinal=2))
        path = dataset(self.root, sequences=[row], audit=audit)
        output, _ = self.train([path])
        report = training.read_json(output / "coverage.json")
        self.assertEqual(report["startup_targets"], 1)
        self.assertEqual(report["headway_targets"], 0)

    def test_duplicate_json_keys_rejected(self):
        path = dataset(self.root)
        (path / "manifest.json").write_text('{"complete":true,"complete":false}', encoding="utf-8")
        with self.assertRaisesRegex(training.DatasetError, "duplicate JSON"):
            self.train([path])

    def test_permission_domains_never_share_prefix_or_tail(self):
        protected = sequence(lane="p", seq="p", times=(1, 3, 4, 5))
        protected["permission_class"] = "protected"
        yielding = sequence(lane="y", seq="y", times=(1, 9, 17, 25))
        yielding["permission_class"] = "yield"
        for row in (protected, yielding):
            for passage in row["passages"]:
                passage["permission_class"] = row["permission_class"]
        path = dataset(self.root, sequences=[protected, yielding])
        output, _ = self.train([path], prefix_length=2)
        predictor = training.Predictor(training.read_json(output / "model.json"))
        base = dict(start_kind="stopped_green_start", turn="s", type_id="car")
        self.assertEqual(predictor.predict("headway", 2, dict(base, permission_class="protected"))[0], 2)
        self.assertEqual(predictor.predict("headway", 2, dict(base, permission_class="yield"))[0], 8)
        self.assertEqual(predictor.predict("headway", 8, dict(base, permission_class="protected"))[0], 1)
        self.assertEqual(predictor.predict("headway", 8, dict(base, permission_class="yield"))[0], 8)
        self.assertEqual(predictor.predict("headway", 8, dict(base, permission_class="unsignalized")), (2, "fixed"))

    def test_in_sequence_permission_change_is_rejected(self):
        row = sequence()
        row["permission_class"] = "protected"
        for passage in row["passages"]:
            passage["permission_class"] = "protected"
        row["passages"][1]["permission_class"] = "yield"
        with self.assertRaisesRegex(training.DatasetError, "permission class"):
            training.validate_sequence(row, "r0")

    def test_missing_permission_cannot_inherit_explicit_sequence_class(self):
        row = sequence()
        row["permission_class"] = "protected"
        with self.assertRaisesRegex(training.DatasetError, "permission class"):
            training.validate_sequence(row, "r0")

    def test_audit_permission_must_match_safe_prefix(self):
        row = sequence()
        row["permission_class"] = "protected"
        for passage in row["passages"]:
            passage["permission_class"] = "protected"
        audit = copy.deepcopy(row["passages"])
        audit[1]["permission_class"] = "yield"
        path = dataset(self.root, sequences=[row], audit=audit)
        with self.assertRaisesRegex(training.DatasetError, "original audit stream"):
            self.train([path])

    def test_live_collection_requires_permission_class(self):
        path = dataset(self.root)
        manifest = training.read_json(path / "manifest.json")
        manifest["mode"] = "collect"
        training.write_json(path / "manifest.json", manifest)
        with self.assertRaisesRegex(training.DatasetError, "explicit permission_class"):
            self.train([path])

    def test_validation_with_only_fixed_fallback_is_not_learned_validation(self):
        a = dataset(self.root, "a", [sequence(run="a")])
        b = dataset(self.root, "b", [sequence(run="b", times=(1, 5, 7))])
        output, manifest = self.train([a, b], min_sequences=3, holdout_runs=["b"])
        metrics = training.read_json(output / "metrics.json")
        self.assertEqual(manifest["validation_status"], "fallback_only_insufficient_training_support")
        self.assertEqual(metrics["validation"]["targets"]["headway"]["learned_predictions"], 0)


class SplitTests(unittest.TestCase):
    def test_split_is_input_order_independent(self):
        ids = ["r" + str(i) for i in range(10)]
        self.assertEqual(training.choose_split(ids, seed=10), training.choose_split(list(reversed(ids)), seed=10))
        self.assertEqual(list(training.choose_split(ids).values()).count("validation"), 2)

    def test_invalid_holdouts_rejected(self):
        for holds in (["r"], ["unknown"], ["r", "r"]):
            with self.subTest(holdouts=holds), self.assertRaises(training.DatasetError):
                training.choose_split(["r"], holdout_runs=holds)


if __name__ == "__main__":
    unittest.main()
