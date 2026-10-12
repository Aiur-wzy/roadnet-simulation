"""ABC/RG exporter tests with mock predictors only; no AutoGluon or SUMO run."""
import copy
import csv
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import struct
from types import SimpleNamespace
import unittest
from unittest import mock
import uuid

import numpy as np
import pandas as pd

import catching.model_catching_sumo_cap_context_abc_rg as catch
import catching._shared.model_catching_sumo_cap_context_abc as old
from features.cap_context_abc_features import abc_context_contract


def mock_formula(frame):
    assert list(frame) == catch.FEATURE_COLS
    for column, categories in catch.CATEGORICAL_FEATURES.items():
        assert isinstance(frame[column].dtype, pd.CategoricalDtype)
        assert frame[column].cat.categories.tolist() == categories
        assert not frame[column].cat.ordered
    return (frame.road_length.to_numpy() / frame.free_speed_cap.to_numpy()
            + frame.platoon_context.astype(int).to_numpy() * .75
            + np.minimum(frame.road_flow.to_numpy(), 2) * .25
            + frame.entry_signal_remaining_seconds.to_numpy() * .125
            + frame.next_signal_duration_seconds.to_numpy() * .0625
            + (frame.entry_signal_type == "R").astype(float).to_numpy() - .5)


def mock_predictor(function=mock_formula):
    metadata = SimpleNamespace(get_features=lambda: list(catch.FEATURE_COLS),
                               get_feature_type_raw=lambda name: "category" if name in catch.CATEGORICAL_FEATURES else "float")
    return SimpleNamespace(label=catch.LABEL_COL, problem_type="regression",
                           feature_metadata=metadata, feature_metadata_in=metadata,
                           predict=function)


def prepare_fixture(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    run = root / "model"
    (run / "predictor").mkdir(parents=True)
    (run / "predictor" / "predictor.pkl").write_bytes(b"MOCK-NOT-A-PICKLE")
    ranges = {"road_length": {"min": 10., "max": 20.}, "road_flow": {"min": 0., "max": 6.},
              "lane_flow": {"min": 0., "max": 3.}, "free_speed_cap": {"min": 8., "max": 10.},
              "entry_signal_remaining_seconds": {"min": 0., "max": 9.},
              "next_signal_duration_seconds": {"min": 0., "max": 9.}}
    for name, categories in catch.CATEGORICAL_FEATURES.items():
        ranges[name] = {"type": "categorical", "categories": categories.copy(),
                        "observed_categories": categories.copy(), "ordered": False}
    ranges["platoon_context"]["category_names"] = dict(catch.trainer.CONTEXT_NAMES)
    contract = {"schema": catch.FEATURE_SCHEMA, "raw_features": list(catch.FEATURE_COLS),
                "status": "complete", "source_label": "travel_time_label_gt_1", "model_label": catch.LABEL_COL,
                "predictor_directory": "predictor", "framework_features": list(catch.FEATURE_COLS),
                "context_contract": abc_context_contract(), "signal_contract": catch.trainer.expected_signal_contract(),
                "categorical_features": copy.deepcopy(catch.CATEGORICAL_FEATURES), "categorical_ordered": False,
                "training_features": ranges, "artifact_role": "synthetic-test-only"}
    (run / "training_contract.json").write_text(json.dumps(contract), encoding="utf-8")
    net = root / "net.xml"
    net.write_text('<net><edge id="a"><lane length="10" speed="10"/></edge>'
                   '<edge id=":ab" function="internal"><lane length="3" speed="10"/></edge>'
                   '<edge id="b"><lane length="20" speed="10"/></edge>'
                   '<tlLogic id="J" type="static" programID="0" offset="0">'
                   '<phase duration="6" state="G"/><phase duration="2" state="y"/>'
                   '<phase duration="9" state="r"/></tlLogic>'
                   '<connection from="a" to="b" fromLane="0" toLane="0" tl="J" linkIndex="0"/>'
                   '</net>', encoding="utf-8")
    return run, net, contract


def fixture_args(root, *extra):
    return catch.build_parser().parse_args([
        "--model-dir", str(Path(root) / "model"), "--net-file", str(Path(root) / "net.xml"),
        "--output-dir", str(Path(root) / "export"), "--signal-remaining-step", "1",
        "--cap-step", "1", "--max-road-flow", "6", "--max-lane-flow", "3",
        "--chunk-size", "7", "--dense-reference", *extra])


def write_golden_fixture(destination):
    """Explicit tiny retained cross-language fixture; refuses an existing target."""
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    prepare_fixture(destination)
    args = fixture_args(destination, "--max-road-flow", "2", "--max-lane-flow", "1",
                        "--cap-min", "8", "--cap-max", "8", "--text-format", "both")
    plan = catch.build_plan(args)
    report = catch.export_table(args, plan, mock_predictor())
    (destination / "header.json").write_text(json.dumps(report["plan"], indent=2), encoding="utf-8")
    return report


def write_network_mock_fixture(destination, net_file, road_max=12, lane_max=8):
    """Real static network, synthetic predictor, small explicit flow coverage.

    Does not run SUMO or access a trained predictor. Caller must select flow
    bounds covering the separate smoke fixture; a real fleet is NOT certified.
    """
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=False)
    run, net, contract = prepare_fixture(destination)
    net.write_bytes(Path(net_file).read_bytes())
    pairs, _ = catch.abc.network_pairs(net)
    lengths = sorted({length for length, _ in pairs})
    ranges = contract["training_features"]
    ranges["road_length"] = {"min": min(lengths), "max": max(lengths)}
    ranges["free_speed_cap"] = {"min": 7.5006, "max": 13.89}
    ranges["road_flow"] = {"min": 0, "max": road_max}
    ranges["lane_flow"] = {"min": 0, "max": lane_max}
    catalog = catch.signal_catalog(net, 1.)
    ranges["entry_signal_remaining_seconds"] = {"min": 0., "max": max(x["entry_max_seconds"] for x in catalog)}
    ranges["next_signal_duration_seconds"] = {"min": 0., "max": max(x["next_duration_seconds"] for x in catalog)}
    (run / "training_contract.json").write_text(json.dumps(contract), encoding="utf-8")
    args = fixture_args(destination, "--cap-step", ".1389", "--max-road-flow", str(road_max),
                        "--max-lane-flow", str(lane_max), "--chunk-size", "50000")
    args.dense_reference = False
    plan = catch.build_plan(args)
    report = catch.export_table(args, plan, mock_predictor())
    report["artifact_role"] = "synthetic-test-only; not trained model or accuracy experiment"
    (destination / "MOCK_ONLY.json").write_text(json.dumps({"artifact_role": report["artifact_role"],
                                                           "rows": report["rows"]}, indent=2), encoding="utf-8")
    return report


class RGTableTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path.cwd().resolve()
        self.root = self.parent / ("rg-catch-test-" + uuid.uuid4().hex)
        self.run, self.net, self.contract = prepare_fixture(self.root)

    def tearDown(self):
        target = self.root.resolve()
        if target.parent != self.parent or not target.name.startswith("rg-catch-test-"):
            raise RuntimeError("Unsafe test cleanup target")
        shutil.rmtree(target)

    def save_contract(self):
        (self.run / "training_contract.json").write_text(json.dumps(self.contract), encoding="utf-8")

    def export(self, *extra, function=mock_formula):
        args = fixture_args(self.root, *extra)
        plan = catch.build_plan(args)
        return args, catch.export_table(args, plan, mock_predictor(function))

    def mutate(self, path, header_edit=None, payload_edit=None, rehash=True):
        with Path(path).open("rb") as stream:
            magic, header = stream.readline(), json.loads(stream.readline())
            payload = bytearray(stream.read())
        if header_edit:
            header_edit(header)
        if payload_edit:
            payload_edit(header, payload)
        if rehash:
            header["payload_sha256"] = hashlib.sha256(payload).hexdigest()
        target = self.root / "mutated.rle"
        target.write_bytes(magic + json.dumps(header).encode() + b"\n" + payload)
        return target

    def test_complete_all_values_binary64_and_three_categories(self):
        args, report = self.export()
        header, values = catch.read_table(Path(args.output_dir) / "table.rle")
        expected = np.maximum(mock_formula(catch.feature_frame(list(catch.iter_cases(header)))), 1.)
        self.assertTrue(np.array_equal(values.view(np.uint64), expected.view(np.uint64)))
        self.assertEqual(values.tobytes(), (Path(args.output_dir) / "reference.f64").read_bytes())
        self.assertEqual(report["dense_sha256"], hashlib.sha256(values.tobytes()).hexdigest())
        self.assertLess(report["run_count"], report["rows"])
        self.assertEqual(report["plan"]["signal_point_count"], 18)

    def test_piecewise_production_grid_and_prediction_budget(self):
        nodes = catch.road_flow_segment_nodes(["10:1", "60:5", "100:20", "222:50"], 222)
        self.assertEqual(nodes, [*range(11), *range(15, 61, 5), 80, 100, 150, 200, 222])
        pairs = catch.prediction_flow_count(222, 80, nodes=nodes)
        self.assertEqual(pairs, 915)
        self.assertEqual(2 * 47 * 6 * pairs * 179, 92_374_740)
        self.assertEqual(catch.road_flow_nodes(61, 222, nodes=nodes), [61, 80, 100, 150, 200, 222])
        self.assertEqual(catch.road_flow_nodes(222, 222, nodes=nodes), [222])

    def test_piecewise_all_legal_queries_nearest_ties_high(self):
        for maximum, segments in [(0, ["0:1"]), (6, ["2:1", "6:4"]), (19, ["3:2", "19:5"])]:
            nodes = catch.road_flow_segment_nodes(segments, maximum)
            counted = 0
            for lane in range(maximum + 1):
                legal = catch.road_flow_nodes(lane, maximum, nodes=nodes)
                counted += len(legal)
                actual = {}
                for representative, low, high in catch.road_flow_spans(lane, maximum, nodes=nodes):
                    self.assertLessEqual(low, high)
                    for road in range(low, high + 1):
                        self.assertNotIn(road, actual)
                        actual[road] = representative
                self.assertEqual(set(actual), set(range(lane, maximum + 1)))
                for road, value in actual.items():
                    self.assertEqual(value, min(legal, key=lambda node: (abs(node - road), -node)))
            self.assertEqual(counted, catch.prediction_flow_count(maximum, maximum, nodes=nodes))

    def test_piecewise_roundtrip_matches_direct_representative_predictions(self):
        function = lambda frame: mock_formula(frame) + frame.road_flow.to_numpy() * .375
        args, report = self.export("--road-flow-segments", "2:1", "6:4", function=function)
        header, values = catch.read_table(Path(args.output_dir) / "table.rle")
        nodes = header["road_flow_quantization"]["nodes"]
        cases = []
        for case in catch.iter_cases(header):
            case = list(case)
            legal = catch.road_flow_nodes(case[2], 6, nodes=nodes)
            case[1] = min(legal, key=lambda node: (abs(node - case[1]), -node))
            cases.append(case)
        expected = np.maximum(function(catch.feature_frame(cases)), 1.)
        self.assertEqual(values.tobytes(), expected.tobytes())
        self.assertEqual(values.tobytes(), (Path(args.output_dir) / "reference.f64").read_bytes())
        self.assertLess(report["prediction_rows"], report["rows"])
        self.assertEqual(len(list(catch.iter_prediction_cases(header))), report["prediction_rows"])

    def test_piecewise_dense_grid_preserves_original_inputs(self):
        args = fixture_args(self.root, "--road-flow-segments", "6:1")
        piecewise = catch.build_plan(args)
        legacy = catch.build_plan(fixture_args(self.root))
        self.assertEqual(list(catch.iter_prediction_cases(piecewise)), list(catch.iter_prediction_cases(legacy)))
        self.assertEqual(piecewise["prediction_rows"], piecewise["expanded_rows"])

    def test_piecewise_rejects_bad_segments_and_conflicting_options(self):
        for segments in [[], ["2"], ["2:1:1"], ["2.0:1", "6:1"], ["-1:1", "6:1"],
                         ["2:0", "6:1"], ["2:1", "2:2", "6:1"], ["6:1", "2:1"],
                         ["5:1"], ["7:1"], ["6:true"], ["６:1"]]:
            with self.subTest(segments=segments), self.assertRaises(ValueError):
                catch.road_flow_segment_nodes(segments, 6)
        for old_options in [("--road-flow-step", "5"), ("--road-flow-dense-until", "4")]:
            args = fixture_args(self.root, "--road-flow-segments", "6:2", *old_options)
            with self.assertRaisesRegex(ValueError, "conflict"):
                catch.build_plan(args)

    def test_piecewise_metadata_is_strict_and_counted(self):
        plan = catch.build_plan(fixture_args(self.root, "--road-flow-segments", "2:1", "6:4"))
        self.assertEqual(catch.validate_road_flow_metadata(plan)["nodes"], [0, 1, 2, 4, 6])
        for nodes in [None, [], [1, 6], [0, 5], [0, 7], [0, 2, 2, 6], [0, 4, 2, 6],
                      [False, 6], [0, 2.0, 6], [0, "2", 6]]:
            bad = copy.deepcopy(plan)
            bad["road_flow_quantization"]["nodes"] = nodes
            with self.subTest(nodes=nodes), self.assertRaises(ValueError):
                catch.validate_road_flow_metadata(bad)
        for edit in [lambda value: value["road_flow_quantization"].update(step=1),
                     lambda value: value["road_flow_quantization"].update(tie="lower"),
                     lambda value: value.update(prediction_rows=value["prediction_rows"] + 1)]:
            bad = copy.deepcopy(plan)
            edit(bad)
            with self.assertRaises(ValueError):
                catch.validate_road_flow_metadata(bad)

    def test_piecewise_dryrun_reports_counts_without_writing(self):
        args = fixture_args(self.root, "--road-flow-segments", "2:1", "6:4")
        with mock.patch.object(catch, "export_table") as export, redirect_stdout(io.StringIO()) as output:
            plan = catch.main(["--model-dir", str(self.run), "--net-file", str(self.net),
                               "--output-dir", args.output_dir, "--signal-remaining-step", "1",
                               "--cap-step", "1", "--max-road-flow", "6", "--max-lane-flow", "3",
                               "--road-flow-segments", "2:1", "6:4", "--dry-run"])
        export.assert_not_called()
        self.assertIn("nearest-explicit-integer-road-flow-v1", output.getvalue())
        self.assertIn(f"prediction_rows={plan['prediction_rows']}", output.getvalue())
        self.assertFalse(Path(args.output_dir).exists())

    def test_text_both_exact_and_header(self):
        args, report = self.export("--text-format", "both")
        _, expected = catch.read_table(Path(args.output_dir) / "table.rle")
        for kind, separator in (("txt", " "), ("csv", ",")):
            with (Path(args.output_dir) / ("table." + kind)).open(newline="", encoding="utf-8") as stream:
                rows = list(csv.reader(stream, delimiter=separator))
            self.assertEqual(rows[0], catch.FEATURE_COLS + [catch.LABEL_COL])
            values = np.array([float(row[-1]) for row in rows[1:]], dtype="<f8")
            self.assertEqual(values.tobytes(), expected.tobytes())
            self.assertEqual(report["text_outputs"][kind]["rows"], len(expected))

    def test_floor_and_wrong_output_do_not_claim_complete(self):
        args, report = self.export(function=lambda frame: np.full(len(frame), -.1))
        self.assertEqual(report["predictions_floored_to_one_second"], report["rows"])
        bad_args = fixture_args(self.root, "--output-dir", str(self.root / "bad"))
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            catch.export_table(bad_args, catch.build_plan(bad_args), mock_predictor(lambda f: np.full(len(f), np.nan)))
        self.assertFalse((Path(bad_args.output_dir) / "manifest.json").exists())

    def test_missing_step_and_no_overwrite(self):
        with redirect_stdout(io.StringIO()), self.assertRaises(SystemExit):
            catch.build_parser().parse_args(["--model-dir", "x", "--net-file", "x", "--output-dir", "x"])
        args, _ = self.export()
        with self.assertRaisesRegex(ValueError, "exists"):
            catch.build_plan(args)

    def test_budget_prints_before_rejection_and_dryrun_no_files(self):
        argv = ["--model-dir", str(self.run), "--net-file", str(self.net),
                "--output-dir", str(self.root / "dry"), "--signal-remaining-step", "5",
                "--cap-step", "1", "--max-road-flow", "6", "--max-lane-flow", "3", "--dry-run"]
        capture = io.StringIO()
        with redirect_stdout(capture), self.assertRaisesRegex(ValueError, "Planned"):
            catch.main(argv + ["--max-rows", "1"])
        self.assertIn("[PLAN]", capture.getvalue())
        with redirect_stdout(io.StringIO()):
            catch.main(argv)
        self.assertFalse((self.root / "dry").exists())

    def test_adjustable_steps_catalog_and_export_roundtrip(self):
        expected = {2.: ([.001, 2., 4., 6.], [.001, 2., 4., 6., 8., 9.]),
                    2.5: ([.001, 2.5, 5., 6.], [.001, 2.5, 5., 7.5, 9.]),
                    3.: ([.001, 3., 6.], [.001, 3., 6., 9.]),
                    5.: ([.001, 5., 6.], [.001, 5., 9.]),
                    10.: ([.001, 6.], [.001, 9.]),
                    100.: ([.001, 6.], [.001, 9.])}
        for step, (green, red) in expected.items():
            with self.subTest(step=step):
                args, report = self.export("--signal-remaining-step", str(step), "--chunk-size", "50000",
                                           "--output-dir", str(self.root / f"step-{step}"))
                header, values = catch.read_table(Path(args.output_dir) / "table.rle")
                self.assertEqual(header["signal_remaining_step"], step)
                self.assertEqual([item["nodes"] for item in header["signal_catalog"]], [[0.], green, red])
                self.assertEqual(header["signal_point_count"], 1 + len(green) + len(red))
                self.assertEqual(header["expanded_rows"], 792 * (1 + len(green) + len(red)))
                expected_values = np.maximum(mock_formula(catch.feature_frame(list(catch.iter_cases(header)))), 1.)
                self.assertEqual(values.tobytes(), expected_values.tobytes())
                self.assertEqual(report["dense_sha256"], hashlib.sha256(values.tobytes()).hexdigest())
                self.assertEqual(catch.quantize_signal(header["signal_catalog"], "G", 6., "R", 9.)[1],
                                 ("G", 6., "R", 9.))
                self.assertEqual(catch.quantize_signal(header["signal_catalog"], "R", 9., "G", 6.)[1],
                                 ("R", 9., "G", 6.))

    def test_one_second_golden_payload_and_case_order_unchanged(self):
        args, report = self.export("--chunk-size", "50000")
        # Captured from the original explicit-1s exporter before comparison support.
        self.assertEqual(report["dense_sha256"], "500c8ad1a2fd386fb73d77127f6e32fc27e22ecf0a3e3fbc243d6c0f4d573912")
        with (Path(args.output_dir) / "table.rle").open("rb") as stream:
            self.assertEqual(stream.readline().decode().strip(), catch.TABLE_MAGIC)
            header = json.loads(stream.readline())
            self.assertEqual(hashlib.sha256(stream.read()).hexdigest(),
                             "88b46ae588fb3f4d210f8d277de987b582d0e5bf049c6cbd8df037eac57d8723")
        signals = [("G", 0., "G", 0.)]
        signals += [("G", node, "R", 9.) for node in [.001, 1., 2., 3., 4., 5., 6.]]
        signals += [("R", node, "G", 6.) for node in [.001, 1., 2., 3., 4., 5., 6., 7., 8., 9.]]
        expected_cases = [(length, road, lane, cap, context, *signal)
                          for signal in signals for length in (10., 20.) for cap in (8., 9., 10.)
                          for context in range(6) for lane in range(4) for road in range(lane, 7)]
        self.assertEqual(list(catch.iter_cases(header)), expected_cases)

    def test_compare_steps_same_grid_ordered_deduplication_and_no_plan_changes(self):
        plan = catch.build_plan(fixture_args(self.root, "--signal-remaining-step", "2.5", "--max-rows", "5000"))
        original = copy.deepcopy(plan)
        with mock.patch.object(catch, "MAX_PAYLOAD", 20000):
            comparison = catch.compare_signal_steps(plan, [5., 2., 2.5, 3., 5., 10., 100., 1.])
        self.assertEqual(plan, original)
        self.assertEqual([item["signal_remaining_step"] for item in comparison], [5., 2., 2.5, 3., 10., 100., 1.])
        self.assertEqual([item["signal_point_count"] for item in comparison], [7, 11, 10, 8, 5, 5, 18])
        for item in comparison:
            self.assertEqual(item["expanded_rows"], 792 * item["signal_point_count"])
            self.assertEqual(item["group_count"], 144 * item["signal_point_count"])
            self.assertEqual(item["relative_to_1s"], item["signal_point_count"] / 18)
            self.assertEqual(item["min_rle_bytes"], 16 * item["group_count"] + 4)
            self.assertEqual(item["explicit_row_budget_pass"], item["expanded_rows"] <= 5000)
            self.assertEqual(item["minimum_payload_budget_pass"], item["min_rle_bytes"] <= 20000)

    def dry_run_argv(self, *extra):
        return ["--model-dir", str(self.run), "--net-file", str(self.net),
                "--output-dir", str(self.root / "dry"), "--signal-remaining-step", "2.5",
                "--cap-step", "1", "--max-road-flow", "6", "--max-lane-flow", "3", *extra]

    def test_comparison_cli_is_readonly_and_preserves_selected_step(self):
        capture = io.StringIO()
        with redirect_stdout(capture), mock.patch.object(catch, "export_table") as export:
            plan = catch.main(self.dry_run_argv("--dry-run", "--compare-signal-steps", "1", "5", "2.5", "5", "10"))
        export.assert_not_called()
        self.assertEqual(plan["signal_remaining_step"], 2.5)
        self.assertEqual(plan["signal_point_count"], 10)
        self.assertNotIn("comparison", plan)
        output = capture.getvalue()
        self.assertIn("signal_remaining_step_seconds=2.5", output)
        self.assertIn("relative_to_1s=1", output)
        self.assertIn("explicit_row_budget=PASS minimum_payload_budget=PASS", output)
        self.assertEqual(output.count("[SCALE]"), 4)
        self.assertIn("[DRY_RUN]", output)
        self.assertFalse((self.root / "dry").exists())

    def test_comparison_requires_dryrun_and_validates_all_values_before_planning(self):
        with redirect_stderr(io.StringIO()), mock.patch.object(catch, "build_plan") as build:
            with self.assertRaises(SystemExit):
                catch.main(self.dry_run_argv("--compare-signal-steps", "2", "5"))
        build.assert_not_called()
        for value in ("0", "-1", "nan", "inf"):
            with self.subTest(value=value), mock.patch.object(catch, "build_plan") as build:
                with self.assertRaisesRegex(ValueError, "comparison signal step"):
                    catch.main(self.dry_run_argv("--dry-run", "--compare-signal-steps", "2.5", value))
            build.assert_not_called()
        self.assertFalse((self.root / "dry").exists())

    def test_selected_budget_failure_still_fatal_after_comparison(self):
        capture = io.StringIO()
        with redirect_stdout(capture), mock.patch.object(catch, "export_table") as export:
            with self.assertRaisesRegex(ValueError, "Planned"):
                catch.main(self.dry_run_argv("--dry-run", "--max-rows", "5000", "--compare-signal-steps", "2.5", "10"))
        export.assert_not_called()
        self.assertIn("explicit_row_budget=FAIL", capture.getvalue())
        self.assertIn("explicit_row_budget=PASS", capture.getvalue())
        with redirect_stdout(io.StringIO()), mock.patch.object(catch, "MAX_PAYLOAD", 20000):
            with self.assertRaisesRegex(ValueError, "one run per group"):
                catch.main(self.dry_run_argv("--dry-run", "--compare-signal-steps", "2.5", "10"))
        self.assertFalse((self.root / "dry").exists())

    def test_invalid_selected_steps_fail_before_export(self):
        for value in ("0", "-1", "nan", "inf", "-inf", "1e-100"):
            with self.subTest(value=value), redirect_stdout(io.StringIO()):
                with mock.patch.object(catch, "export_table") as export, self.assertRaises(ValueError):
                    catch.main(self.dry_run_argv("--signal-remaining-step=" + value, "--dry-run"))
                export.assert_not_called()
        plan = catch.build_plan(fixture_args(self.root))
        for value in (0., -1., math.nan, math.inf, -math.inf, True, 1e-100):
            with self.subTest(comparison_value=value), self.assertRaises(ValueError):
                catch.compare_signal_steps(plan, [2.5, value])
        self.assertFalse((self.root / "dry").exists())

    def test_nearest_ties_endpoint_and_sentinel(self):
        cat = catch.signal_catalog(self.net, 5.)
        self.assertEqual(catch.quantize_signal(cat, "G", 2.5005, "R", 9.)[1][1], 5.)
        self.assertEqual(catch.quantize_signal(cat, "G", 1e-15, "R", 9.)[1][1], .001)
        self.assertEqual(catch.quantize_signal(cat, "G", 6., "R", 9.)[1][1], 6.)
        self.assertEqual(catch.quantize_signal(cat, "G", 0., "G", 0.)[0], 0)
        for values in [("G", 0., "R", 9.), ("G", 6.00000001, "R", 9.),
                       ("G", 1., "R", 9.1), ("G", .001, "G", 0.), ("Y", 1., "R", 9.)]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                catch.quantize_signal(cat, *values)

    def test_tiny_maximum_and_step_nodes(self):
        self.assertEqual(catch.remaining_nodes(.0005, 5), [.0005])
        self.assertEqual(catch.remaining_nodes(.002, .0005), [.001, .0015, .002])
        for step in (0, -1, math.nan, math.inf, True, 1e-100):
            with self.subTest(step=step), self.assertRaises(ValueError):
                catch.remaining_nodes(6, step)

    def test_catalog_duplicate_nodes_offsets_unknown_types(self):
        cat = catch.signal_catalog(self.net, 5.)
        changes = [lambda c: c[1].update(nodes=[.001, 6.]),
                   lambda c: c[1].update(point_start=99), lambda c: c[1].update(duration_key=2),
                   lambda c: c[1].update(entry_type="Y"), lambda c: c[0].update(sentinel=False),
                   lambda c: c.reverse()]
        for change in changes:
            bad = copy.deepcopy(cat)
            change(bad)
            with self.subTest(change=change), self.assertRaises(ValueError):
                catch.validate_catalog(bad, 5.)

    def test_same_color_across_yellow_is_legal(self):
        self.net.write_text('<net><edge id="a"><lane index="0"/></edge><edge id="b"><lane index="0"/></edge>'
                            '<tlLogic id="J" type="static"><phase duration="4" state="G"/>'
                            '<phase duration="2" state="y"/><phase duration="7" state="G"/>'
                            '<phase duration="1" state="y"/></tlLogic>'
                            '<connection from="a" to="b" tl="J" linkIndex="0"/></net>', encoding="utf-8")
        cat = catch.signal_catalog(self.net, 5.)
        self.assertEqual(len(cat), 3)
        self.assertTrue(all(item["entry_type"] == item["next_type"] == "G" for item in cat))

    def test_program_invalid_or_ambiguous(self):
        text = self.net.read_text(encoding="utf-8")
        for changed in (text.replace('type="static"', 'type="actuated"'),
                        text.replace('linkIndex="0"', 'linkIndex="9"'),
                        text.replace('duration="6"', 'duration="6" next="2"'),
                        text.replace('</net>', '<WAUT/></net>'),
                        text.replace('</net>', '<wautJunction/></net>'),
                        text.replace('from="a"', 'from="unknown"')):
            self.net.write_text(changed, encoding="utf-8")
            with self.assertRaises(ValueError):
                catch.signal_catalog(self.net, 5.)

    def test_old_schema_and_changed_contract_rejected(self):
        original = copy.deepcopy(self.contract)
        for change in [lambda c: c.update(schema="road-time-cap-context-abc-v1"),
                       lambda c: c["raw_features"].pop(), lambda c: c.update(categorical_ordered=True),
                       lambda c: c["signal_contract"].update(yellow_duration_policy="add yellow"),
                       lambda c: c["categorical_features"].update(entry_signal_type=["G", "R"]),
                       lambda c: c["categorical_features"]["platoon_context"].__setitem__(0, False),
                       lambda c: c["signal_contract"]["no_tls_placeholder"].update(entry_signal_remaining_seconds=False),
                       lambda c: c["context_contract"].update(context_ids=[0, 1, 2, 3]),
                       lambda c: c.update(status="fit_started")]:
            self.contract = copy.deepcopy(original)
            change(self.contract)
            self.save_contract()
            with self.assertRaises(ValueError):
                catch.resolve_model(self.run)

    def test_changed_threshold_and_framework_leak_rejected(self):
        with self.assertRaises(ValueError):
            catch.resolve_model(self.run, "3")
        self.contract["framework_features"].append("vehicleID")
        self.save_contract()
        with self.assertRaises(ValueError):
            catch.resolve_model(self.run)

    def test_pruned_categories_allowed_raw_numeric_rejected(self):
        plan = catch.build_plan(fixture_args(self.root))
        predictor = mock_predictor()
        predictor.feature_metadata = SimpleNamespace(get_features=lambda: ["road_length"],
                                                     get_feature_type_raw=lambda n: "float")
        plan["model"]["framework_features"] = ["road_length"]
        catch.validate_predictor(predictor, plan)
        predictor.feature_metadata_in = SimpleNamespace(get_features=lambda: catch.FEATURE_COLS,
                                                        get_feature_type_raw=lambda n: "float")
        with self.assertRaisesRegex(ValueError, "categorical"):
            catch.validate_predictor(predictor, plan)
        del predictor.feature_metadata_in
        catch.validate_predictor(predictor, plan)

    def test_all_header_corruption_and_old_reader_rejects(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        with self.assertRaises(ValueError):
            old.read_table(path)
        edits = [lambda h: h.update(format=old.TABLE_MAGIC),
                 lambda h: h.update(expanded_rows=h["expanded_rows"] + 1),
                 lambda h: h.update(signal_point_count=6),
                 lambda h: h.update(group_count=h["group_count"] + 1),
                 lambda h: h.update(context_count=6.), lambda h: h.update(run_count=True),
                 lambda h: h.update(prediction_floor_seconds=True),
                 lambda h: h["categorical_features"]["platoon_context"].__setitem__(0, False),
                 lambda h: h["signal_quantization"].update(tie="lower"),
                 lambda h: h["signal_contract"].update(same_color_merge="never"),
                 lambda h: h["context_contract"]["base_context_contract"].update(p_threshold=.06),
                 lambda h: h["free_speed_cap_grid"].update(min=9.),
                 lambda h: h.update(road_lengths=[20., 10.]),
                 lambda h: h["resource_limits"].update(payload_bytes=2**40)]
        for edit in edits:
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                catch.scan_table(self.mutate(path, header_edit=edit))

    def test_payload_corruption_gap_nan_offsets_and_eof(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        def run_offset(h):
            return 4 * (h["group_count"] + 1)
        changes = [lambda h, p: struct.pack_into("<I", p, 0, 1),
                   lambda h, p: struct.pack_into("<I", p, 4, 0),
                   lambda h, p: struct.pack_into("<H", p, run_offset(h), 1),
                   lambda h, p: struct.pack_into("<H", p, run_offset(h) + 2, 65535),
                   lambda h, p: struct.pack_into("<d", p, run_offset(h) + 4, math.nan),
                   lambda h, p: struct.pack_into("<d", p, run_offset(h) + 4, .5),
                   lambda h, p: p.extend(b"x"), lambda h, p: p.pop()]
        for edit in changes:
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                catch.scan_table(self.mutate(path, payload_edit=edit))
        with self.assertRaisesRegex(ValueError, "SHA"):
            catch.scan_table(self.mutate(path, payload_edit=lambda h, p: p.__setitem__(-1, p[-1] ^ 1), rehash=False))

    def test_input_whitelist_and_illegal_values(self):
        frame = pd.DataFrame([[10., 1, 0, 8., 4, "G", 2.25, "R", 9.]], columns=catch.FEATURE_COLS)
        frame["vehicleID"] = "secret"
        self.assertEqual(list(catch.trainer.encode_feature_frame(frame)), catch.FEATURE_COLS)
        for column, value in [("platoon_context", 6), ("entry_signal_type", 0),
                              ("next_signal_type", "Y"), ("entry_signal_remaining_seconds", -1),
                              ("next_signal_duration_seconds", 0)]:
            changed = frame.copy()
            changed[column] = value
            with self.subTest(column=column), self.assertRaises(ValueError):
                catch.trainer.encode_feature_frame(changed)

    def test_actual_compressed_budget_fail_keeps_only_partial(self):
        args = fixture_args(self.root)
        plan = catch.build_plan(args)
        minimum = 16 * plan["group_count"] + 4
        with mock.patch.object(catch, "MAX_PAYLOAD", minimum):
            with self.assertRaisesRegex(ValueError, "Actual RLE"):
                catch.export_table(args, plan, mock_predictor())
        self.assertFalse((Path(args.output_dir) / "manifest.json").exists())
        self.assertFalse((Path(args.output_dir) / "table.rle").exists())

    def test_no_real_dense_allocation_above_50m_inspection_limit(self):
        plan = catch.build_plan(fixture_args(self.root))
        self.assertEqual(catch.MAX_CPP_ROWS, 4_000_000_000)
        plan["expanded_rows"] = 4_000_000_001
        plan["resource_limits"]["requested_max_rows"] = 4_000_000_000
        with self.assertRaises(ValueError):
            catch.enforce_budget(plan)

    def test_duration_key_collision_and_ambiguous_nearby_values(self):
        catalog = catch.signal_catalog(self.net, 1.)
        new = copy.deepcopy(catalog[1])
        new["next_duration_seconds"] += 1e-9
        new["duration_key"] = catch.duration_key(new["next_duration_seconds"])
        catalog.insert(2, new)
        cursor = 0
        for item in catalog:
            item["point_start"] = cursor
            cursor += len(item["nodes"])
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            catch.validate_catalog(catalog, 1.)

    def test_floating_endpoint_normalization_is_only_eight_ulps(self):
        self.net.write_text('<net><edge id="a"><lane index="0"/></edge><edge id="b"><lane index="0"/></edge>'
                            '<tlLogic id="J" type="static"><phase duration=".1" state="G"/>'
                            '<phase duration=".1" state="G"/><phase duration="1.1" state="G"/>'
                            '<phase duration="1" state="y"/><phase duration="2" state="r"/></tlLogic>'
                            '<connection from="a" to="b" tl="J" linkIndex="0"/></net>', encoding="utf-8")
        cat = catch.signal_catalog(self.net, 1.)
        program = catch.RGSignalProgram([.1, .1, 1.1, 1, 2], ['G', 'G', 'G', 'y', 'r'])
        raw = program.query(0, .1)
        maximum = cat[1]["entry_max_seconds"]
        self.assertGreater(raw["entry_signal_remaining_seconds"], maximum)
        self.assertEqual(catch.quantize_signal(cat, 'G', raw['entry_signal_remaining_seconds'], 'R', 2.)[1][1], maximum)
        self.assertEqual(catch.quantize_signal(cat, 'G', maximum + 8 * math.ulp(maximum), 'R', 2.)[1][1], maximum)
        with self.assertRaises(ValueError):
            catch.quantize_signal(cat, 'G', maximum + 9 * math.ulp(maximum), 'R', 2.)

    def test_internal_constant_signal_skipped_and_empty_tls_is_no_tls(self):
        self.net.write_text('<net><edge id="a"><lane index="0"/></edge><edge id="b"><lane index="0"/></edge>'
                            '<edge id=":internal" function="internal"><lane index="0"/></edge>'
                            '<tlLogic id="J" type="static"><phase duration="6" state="GG"/>'
                            '<phase duration="2" state="yG"/><phase duration="9" state="rG"/></tlLogic>'
                            '<connection from="a" to="b" tl="J" linkIndex="0"/>'
                            '<connection from=":internal" to="b" tl="J" linkIndex="1"/>'
                            '<connection from="a" to="b" tl=""/></net>', encoding="utf-8")
        catalog = catch.signal_catalog(self.net, 1.)
        self.assertEqual(len(catalog), 4)
        self.assertEqual(catch.quantize_signal(catalog, "G", 6., "G", 6.)[1], ("G", 6., "G", 6.))

    def test_actual_net_readonly_budget_if_available(self):
        from roadnet_paths import ROOT
        actual = Path(os.environ.get("ABC_REFERENCE_REPO", ROOT))
        net = actual / "data/net/test_new.net.xml"
        if not net.is_file():
            self.skipTest("Actual network not available")
        old_sha = hashlib.sha256(net.read_bytes()).hexdigest()
        for step, expected_points in [(1., 179), (2., 93), (2.5, 76), (3., 63), (5., 41), (10., 25), (100., 9)]:
            cat = catch.signal_catalog(net, step)
            self.assertEqual(len(cat), 5)
            self.assertEqual(sum(len(x["nodes"]) for x in cat), expected_points)
        self.assertEqual(old_sha, hashlib.sha256(net.read_bytes()).hexdigest())


if __name__ == "__main__":
    unittest.main()
