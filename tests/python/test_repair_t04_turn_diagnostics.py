"""Small deterministic tests; no table, predictor, or SUMO process is loaded."""
from __future__ import annotations

import importlib.util
import json
import shutil
import unittest
import uuid
from pathlib import Path
from unittest.mock import patch
from roadnet_paths import ROOT
from features import turn_context

import numpy as np
import pandas as pd

from tools.maintenance import repair_t04_turn_diagnostics as repair


def load_analyzer():
    root = ROOT
    path = root / "archived_fit/analyze_fit.py"
    if not path.is_file():
        path = root / "eval/sf_training_diagnosis_20260910/fit/analyze_fit.py"
    if not path.is_file():
        return None  # Private historical analysis is not part of the public checkout.
    spec = importlib.util.spec_from_file_location("t04_archived_analyze_fit", path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", {"turn_context": turn_context}):
        spec.loader.exec_module(module)
    return module


analyzer = load_analyzer()


def metric(population="benchmark", dimension="incoming_turn_type", group="1", **extra):
    return {"population": population, "dimension": dimension, "group": group,
            "split": "free", "threshold": "0p1", "n": 7, "mae": 1.23,
            "bias": -0.45, **extra}


def fixture_summary():
    return {"metrics": [metric(), metric(group="3"), metric(group="0"), metric(group="2"),
                        metric(population="new4x_sf"), metric(dimension="outgoing_turn_type"),
                        metric(dimension="overall", group="all")],
            "threshold_decline": [metric(), metric(population="new4x_sf"), metric(dimension="overall", group="all")],
            "matched_free_cells": [{"free_hidden_groups": [metric(), metric(dimension="outgoing_turn_type")],
                                     "train_hidden_groups": [metric(), metric(group="3")]}],
            "provenance": {"unrelated_group": "1", "turn_type": 3, "中文": "保留原样"}}


class SurgicalRepairTests(unittest.TestCase):
    def test_exact_scoped_swap_and_numeric_spelling(self):
        original = fixture_summary()
        raw = json.dumps(original, ensure_ascii=False, indent=3).replace('"mae": 1.23', '"mae": 1.230000e+00').encode("utf-8")
        corrected, evidence = repair.repair_document(raw, "fit_summary.json")
        actual = json.loads(corrected)
        self.assertEqual(evidence["changed_group_count"], 4)
        self.assertEqual(evidence["changed_sections"], {"metrics": 2, "threshold_decline": 1, "matched_free_cells": 1})
        self.assertEqual([row["group"] for row in actual["metrics"]], ["3", "1", "0", "2", "1", "1", "all"])
        self.assertEqual(actual["matched_free_cells"][0]["train_hidden_groups"], original["matched_free_cells"][0]["train_hidden_groups"])
        self.assertEqual(actual["provenance"], original["provenance"])
        self.assertEqual(corrected.count(b'1.230000e+00'), raw.count(b'1.230000e+00'))
        self.assertEqual(len(raw), len(corrected))
        self.assertTrue(evidence["all_other_bytes_unchanged"])
        self.assertTrue(evidence["all_numeric_tokens_unchanged"])
        self.assertTrue(evidence["byte_exact_self_inverse_verified"])
        self.assertEqual(repair.repair_document(corrected, "fit_summary.json")[0], raw)

    def test_early_does_not_touch_nested_free_or_training_groups(self):
        original = fixture_summary()
        corrected, evidence = repair.repair_document(json.dumps(original).encode(), "benchmark_early.json")
        self.assertEqual(evidence["changed_group_count"], 3)
        self.assertEqual(json.loads(corrected)["matched_free_cells"], original["matched_free_cells"])

    def test_closed_only_metrics_no_population_needed(self):
        document = {"metrics": [{"dimension": "incoming_turn_type", "group": "1", "n": 3, "dataset": "free"},
                                 {"dimension": "incoming_turn_type", "group": "3", "n": 2, "dataset": "free"},
                                 {"dimension": "outgoing_turn_type", "group": "1", "n": 1}],
                    "per_vehicle_road_sums": [{"group": "1", "bias": 2.2}], "audits": [{"group": "3"}]}
        corrected, evidence = repair.repair_document(json.dumps(document).encode(), "closed_loop_input_comparison.json")
        actual = json.loads(corrected)
        self.assertEqual(evidence["changed_group_count"], 2)
        self.assertEqual(actual["per_vehicle_road_sums"], document["per_vehicle_road_sums"])
        self.assertEqual(actual["audits"], document["audits"])

    def test_bom_crlf_escapes_retained(self):
        text = json.dumps(fixture_summary(), indent=2).replace("\n", "\r\n")
        raw = b"\xef\xbb\xbf" + text.encode()
        fixed, _ = repair.repair_document(raw, "fit_summary.json")
        self.assertEqual(fixed[:3], raw[:3])
        self.assertEqual(fixed.count(b"\r\n"), raw.count(b"\r\n"))
        self.assertEqual(repair.repair_document(fixed, "fit_summary.json")[0], raw)

    def test_parser_tracks_escaped_keys_and_strings(self):
        text = '{"key\\u0031": ["quoted \\\"1\\\"", 1e-5, true, null, {}, []]}'
        document, spans = repair.json_paths_and_spans(text)
        self.assertEqual(document, json.loads(text))
        self.assertEqual(text[slice(*spans[("key1", 1)])], "1e-5")

    def test_parser_refuses_duplicate_and_nonfinite_or_trailing(self):
        for text in ('{"a":1,"a":2}', '{"a":NaN}', '{"a":Infinity}', '{} {}'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                repair.json_paths_and_spans(text)

    def test_unknown_document_refused(self):
        with self.assertRaisesRegex(ValueError, "Unsupported"):
            repair.repair_document(b"{}", "other.json")

    def test_noncanonical_group_token_refused(self):
        raw = json.dumps(fixture_summary()).replace('"group": "1"', '"group": "\\u0031"').encode()
        with self.assertRaisesRegex(ValueError, "plain single-digit"):
            repair.repair_document(raw, "fit_summary.json")

    def test_independent_count_validator_checks_all_closed_components(self):
        base = {"population": "benchmark", "dimension": "incoming_turn_type", "group": "1", "split": "free", "n": 3}
        docs = {name: {"metrics": [base], "threshold_decline": [base]} for name in ("fit_summary.json", "benchmark_early.json")}
        closed = {"dimension": "incoming_turn_type", "group": "1", "dataset": "free",
                  **{key: {"n": 3} for key in ("teacher_minus_truth", "closed_minus_truth", "closed_minus_teacher")}}
        docs["closed_loop_input_comparison.json"] = {"metrics": [closed]}
        audit = {"datasets": [{"dataset": "free", "canonical_incoming": {"1": 3}}]}
        self.assertEqual(repair.validate_corrected_group_counts(docs, audit), 7)
        closed["closed_minus_teacher"]["n"] = 4
        with self.assertRaisesRegex(AssertionError, "Closed-loop"):
            repair.validate_corrected_group_counts(docs, audit)


class FilesystemFixtureTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path.cwd().resolve()
        self.directory = self.parent / ("t04-diagnostic-test-" + uuid.uuid4().hex)
        self.directory.mkdir()

    def tearDown(self):
        resolved = self.directory.resolve()
        if resolved.parent != self.parent or not resolved.name.startswith("t04-diagnostic-test-"):
            raise AssertionError("Unsafe test cleanup path")
        shutil.rmtree(resolved)

    def create_benchmark_fixture(self):
        work, repo = self.directory / "work", self.directory / "repo"
        net = repo / "data/variants/speed_factor/inputs/dfbf38f23154_test_new.net.xml"
        net.parent.mkdir(parents=True)
        net.write_text('<net><connection from="a" to="b" dir="l"/><connection from="b" to="a" dir="r"/><connection from="a" to="c" dir="s"/></net>', encoding="utf-8")
        route_text = '<routes><route id="long" edges="a b a c"/><route id="single" edges="b"/><vehicle id="v" route="long" speedFactor="1.0"/><vehicle id="single_v" route="single" speedFactor="1.1"/></routes>'
        for dataset in ("free", "tight", "congested"):
            route = repo / f"data/variants/speed_factor/routes/{dataset}.rou.xml"
            route.parent.mkdir(parents=True, exist_ok=True)
            route.write_text(route_text, encoding="utf-8")
            feature_path = work / f"outputs/fit_audit_20260908/features/{dataset}/entry_features.csv"
            label_path = work / f"outputs/oracle_20260907/labels/{dataset}/oracle_labels.csv"
            feature_path.parent.mkdir(parents=True)
            label_path.parent.mkdir(parents=True)
            feature_rows, label_rows = [], []
            for vehicle, edges, turns in (("v", ["a", "b", "a", "c"], [1, 3, 2, 0]), ("single_v", ["b"], [0])):
                for index, (edge, turn) in enumerate(zip(edges, turns)):
                    identity = {"vehicle_id": vehicle, "route_index": index, "edge_id": edge, "is_final": int(index == len(edges) - 1)}
                    feature_rows.append({**identity, "entry_time": index, "turn_type": turn, "road_length": 100,
                                         "road_flow": 1, "lane_flow": 1, "speed_limit": 13.89})
                    labels = ("travel_time_label_gt_" + tag
                              for tag in ("0p1", "0p5", "1", "1p5", "2", "3"))
                    label_rows.append({**identity, "edge_enter_time": index,
                                       **{label: 10.0 for label in labels}})
            pd.DataFrame(feature_rows).to_csv(feature_path, index=False)
            pd.DataFrame(label_rows).to_csv(label_path, index=False)
            feature_path.with_name("validation.json").write_text('{"passed":true,"labels_match_prior_oracle":true}', encoding="utf-8")
        return work, repo

    def test_sha_pinning_rejects_unknown_source_before_writes(self):
        source = self.directory / "source"
        source.mkdir()
        (source / "fit_summary.json").write_text("{}", encoding="utf-8")
        output = self.directory / "new"
        with self.assertRaisesRegex(ValueError, "SHA mismatch"):
            repair.repair_directory(source, output)
        self.assertFalse(output.exists())

    def test_existing_output_refused_without_reading_source(self):
        with self.assertRaises(FileExistsError):
            repair.repair_directory(self.directory / "nonexistent", self.directory)

    @unittest.skipIf(analyzer is None, "Private historical analyzer unavailable")
    def test_benchmark_repeated_edge_single_road_and_outgoing_alias(self):
        work, repo = self.create_benchmark_fixture()
        with patch.object(analyzer, "WORK", work), patch.object(analyzer, "REPO", repo):
            frame, provenance = analyzer.benchmark_data()
        sample = frame[frame.dataset.eq("free")]
        self.assertEqual(sample.incoming_turn_type.tolist(), [0, 1, 3, 2, 0])
        self.assertEqual(sample.outgoing_turn_type.tolist(), [1, 3, 2, 0, 0])
        self.assertEqual(sample.incoming_turn_present.tolist(), [False, True, True, True, False])
        self.assertEqual(sample.outgoing_turn_present.tolist(), [True, True, True, False, False])
        self.assertEqual(sample.incoming_turn_direction.tolist(), ["unknown", "l", "r", "s", "unknown"])
        self.assertEqual(len(provenance), 3)
        self.assertTrue(all(item["unknown_present_incoming"] == 0 for item in provenance))
        self.assertEqual(analyzer.FEATURES, ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"])

    @unittest.skipIf(analyzer is None, "Private historical analyzer unavailable")
    def test_wrong_historical_outgoing_is_not_silently_reinterpreted(self):
        work, repo = self.create_benchmark_fixture()
        path = work / "outputs/fit_audit_20260908/features/free/entry_features.csv"
        frame = pd.read_csv(path)
        frame.loc[0, "turn_type"] = 3
        frame.to_csv(path, index=False)
        with patch.object(analyzer, "WORK", work), patch.object(analyzer, "REPO", repo):
            with self.assertRaisesRegex(AssertionError, "Legacy turn_type"):
                analyzer.benchmark_data()

    def test_independent_static_route_rebuild(self):
        work, repo = self.create_benchmark_fixture()
        result = repair.audit_benchmark_turns(work, repo)
        self.assertEqual(result["rows"], 15)
        self.assertEqual(result["incoming_changed_rows"], 6)
        self.assertTrue(result["all_historical_outgoing_correct"])
        for item in result["datasets"]:
            self.assertEqual(item["canonical_incoming"], {"0": 2, "1": 1, "2": 1, "3": 1})
            self.assertEqual(item["first_rows"], 2)
            self.assertEqual(item["final_rows"], 2)


@unittest.skipIf(analyzer is None, "Private historical analyzer unavailable")
class DimensionTests(unittest.TestCase):
    def base_frame(self):
        return pd.DataFrame({"route_index": [0, 1], "is_final_road": [0, 1], "road_length": [100, 100],
                             "road_flow": [1, 2], "lane_flow": [1, 2], "turn_type": [1, 0],
                             "incoming_turn_type": [0, 3]})

    def test_explicit_outgoing_preferred_and_legacy_fallback(self):
        frame = self.base_frame()
        self.assertEqual(analyzer.dimensions(frame)["outgoing_turn_type"].tolist(), ["1", "0"])
        frame["outgoing_turn_type"] = [3, 0]
        result = analyzer.dimensions(frame)
        self.assertEqual(result["outgoing_turn_type"].tolist(), ["3", "0"])
        self.assertEqual(result["incoming_turn_type"].tolist(), ["0", "3"])

    def test_nonturn_metric_definition_unchanged(self):
        frame = self.base_frame()
        for label in analyzer.LABELS:
            frame[label] = [10.0, 20.0]
        prediction = np.array([[12.] * 6, [19.] * 6])
        old_metrics, old_decline = analyzer.Metrics(), analyzer.Metrics()
        analyzer.accumulate(old_metrics, old_decline, "benchmark", "free", frame, prediction)
        frame["incoming_turn_type"] = [0, 1]
        frame["outgoing_turn_type"] = frame["turn_type"]
        new_metrics, new_decline = analyzer.Metrics(), analyzer.Metrics()
        analyzer.accumulate(new_metrics, new_decline, "benchmark", "free", frame, prediction)
        select = lambda rows: [row for row in rows if row["dimension"] != "incoming_turn_type"]
        self.assertEqual(select(old_metrics.rows()), select(new_metrics.rows()))
        self.assertEqual(select(old_decline.rows()), select(new_decline.rows()))
        overall = next(row for row in new_metrics.rows() if row["dimension"] == "overall")
        self.assertEqual(overall["mae"], 1.5)
        self.assertEqual(overall["bias"], 0.5)


if __name__ == "__main__":
    unittest.main()
