"""Sparse road-flow prediction; complete V1 query coverage. No real model fit."""
import copy
import csv
import hashlib
import io
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import shutil
import unittest
import uuid

import numpy as np

import catching.model_catching_sumo_cap_context_abc_rg as catch
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, fixture_args, mock_predictor


def independent_nodes(lane, maximum, step, dense):
    return [x for x in range(lane, maximum + 1)
            if x <= dense or x % step == 0 or x in (lane, maximum)]


def independent_map(road, lane, maximum, step, dense):
    return min(independent_nodes(lane, maximum, step, dense), key=lambda node: (abs(node - road), -node))


def formula(frame):
    return (frame.road_flow.to_numpy() * .25 - 1 + frame.lane_flow.to_numpy() * .125
            + frame.entry_signal_remaining_seconds.to_numpy() * .03125
            + frame.platoon_context.astype(int).to_numpy() * .125)


class RoadFlowCatchTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path.cwd().resolve()
        self.root = self.parent / ("roadflow-test-" + uuid.uuid4().hex)
        prepare_fixture(self.root)

    def tearDown(self):
        if self.root.parent != self.parent or not self.root.name.startswith("roadflow-test-"):
            raise RuntimeError("Unexpected cleanup target")
        shutil.rmtree(self.root)

    def args(self, *extra):
        return fixture_args(self.root, "--signal-remaining-step", "5", "--road-flow-step", "5",
                            "--road-flow-dense-until", "10", "--max-road-flow", "27",
                            "--max-lane-flow", "16", "--cap-min", "8", "--cap-max", "8",
                            "--chunk-size", "5000", *extra)

    def run_export(self, *extra, function=formula):
        args = self.args(*extra)
        plan = catch.build_plan(args)
        seen = []

        def predict(frame):
            seen.extend(tuple(row) for row in frame.itertuples(index=False, name=None))
            return function(frame)

        report = catch.export_table(args, plan, mock_predictor(predict))
        return args, report, seen

    def test_nodes_prefix_endpoints_and_lane_above_prefix(self):
        self.assertEqual(catch.road_flow_nodes(0, 27, 5, 10), list(range(11)) + [15, 20, 25, 27])
        self.assertEqual(catch.road_flow_nodes(12, 27, 5, 10), [12, 15, 20, 25, 27])
        self.assertEqual(catch.road_flow_nodes(27, 27, 5, 10), [27])
        self.assertEqual(catch.road_flow_nodes(12, 27, 5, 40), list(range(12, 28)))
        self.assertEqual(catch.road_flow_nodes(0, 0, 5, 10), [0])

    def test_bruteforce_spans_counts_and_ties(self):
        for maximum in range(31):
            for step in (1, 2, 5, 7, 37):
                for dense in (0, 10, 40):
                    count = 0
                    for lane in range(maximum + 1):
                        nodes = independent_nodes(lane, maximum, step, dense)
                        spans = list(catch.road_flow_spans(lane, maximum, step, dense))
                        self.assertEqual([item[0] for item in spans], nodes)
                        actual = [node for node, low, high in spans for _ in range(low, high + 1)]
                        expected = [independent_map(road, lane, maximum, step, dense)
                                    for road in range(lane, maximum + 1)]
                        self.assertEqual(actual, expected)
                        self.assertEqual(spans[0][1], lane)
                        self.assertEqual(spans[-1][2], maximum)
                        count += len(nodes)
                    self.assertEqual(catch.prediction_flow_count(maximum, maximum, step, dense), count)
        # 13 is exactly midway between 12 and 14, so the representative is 14.
        self.assertIn((14, 13, 14), list(catch.road_flow_spans(11, 27, 2, 10)))

    def test_invalid_settings_and_noninteger_cli(self):
        for step, dense in [(0, 10), (-1, 10), (True, 10), (1.5, 10), (5, -1), (5, True), (5, 1.5)]:
            with self.subTest(step=step, dense=dense), self.assertRaises(ValueError):
                catch.road_flow_quantization(step, dense)
        for field in ("--road-flow-step", "--road-flow-dense-until"):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.args(field, "2.5")
        for extra in [("--road-flow-step", "0"), ("--road-flow-step", "-1"),
                      ("--road-flow-dense-until", "-1")]:
            with self.assertRaises(ValueError):
                catch.build_plan(self.args(*extra))

    def test_sparse_predict_only_nodes_and_full_bitwise_coverage(self):
        args, report, seen = self.run_export("--text-format", "both")
        header, values = catch.read_table(Path(args.output_dir) / "table.rle")
        self.assertEqual(len(seen), header["prediction_rows"])
        self.assertEqual(len(set(seen)), len(seen))
        self.assertLess(len(seen), header["expanded_rows"])
        cases = list(catch.iter_cases(header))
        mapped = [(row[0], independent_map(row[1], row[2], 27, 5, 10), *row[2:]) for row in cases]
        expected = np.maximum(formula(catch.feature_frame(mapped)), 1.).astype("<f8")
        self.assertEqual(values.tobytes(), expected.tobytes())
        self.assertEqual((Path(args.output_dir) / "reference.f64").read_bytes(), expected.tobytes())
        self.assertEqual(report["dense_sha256"], hashlib.sha256(expected.tobytes()).hexdigest())
        self.assertEqual(report["prediction_rows"], len(seen))
        for row in seen:
            self.assertIn(row[1], independent_nodes(row[2], 27, 5, 10))
        for kind, separator in (("txt", " "), ("csv", ",")):
            with (Path(args.output_dir) / ("table." + kind)).open(encoding="utf-8", newline="") as stream:
                rows = list(csv.reader(stream, delimiter=separator))
            self.assertEqual(len(rows) - 1, len(cases))
            self.assertEqual([int(row[1]) for row in rows[1:]], [row[1] for row in cases])
            self.assertEqual(np.array([float(row[-1]) for row in rows[1:]], dtype="<f8").tobytes(), expected.tobytes())

    def test_floor_counts_separate_nodes_and_expanded_queries(self):
        _, report, seen = self.run_export(function=lambda frame: np.full(len(frame), -.1))
        self.assertEqual(report["prediction_nodes_floored_to_one_second"], len(seen))
        self.assertEqual(report["predictions_floored_to_one_second"], report["rows"])
        self.assertGreater(report["predictions_floored_to_one_second"], len(seen))

    def test_equal_values_merge_across_nodes_and_chunk_boundaries(self):
        _, report, _ = self.run_export("--max-road-flow", "11", "--max-lane-flow", "0",
                                      "--road-flow-dense-until", "0", "--chunk-size", "1",
                                      function=lambda frame: np.full(len(frame), 3.25))
        self.assertEqual(report["run_count"], report["plan"]["group_count"])
        for _, _, low, high, value in catch.iter_runs(self.root / "export" / "table.rle"):
            self.assertEqual((low, high, value), (0, 11, 3.25))

    def test_dense_prefix_larger_than_domain_and_default_one_are_exact(self):
        for name, extra in [("dense", ("--road-flow-dense-until", "40")),
                            ("step1", ("--road-flow-step", "1"))]:
            args, report, _ = self.run_export("--output-dir", str(self.root / name), *extra)
            header, values = catch.read_table(Path(args.output_dir) / "table.rle")
            self.assertEqual(header["prediction_rows"], header["expanded_rows"])
            expected = np.maximum(formula(catch.feature_frame(list(catch.iter_cases(header)))), 1.).astype("<f8")
            self.assertEqual(values.tobytes(), expected.tobytes())

    def test_optional_metadata_legacy_and_strict_corruption(self):
        _, report, _ = self.run_export()
        header = report["plan"]
        legacy = copy.deepcopy(header)
        legacy.pop("prediction_rows")
        legacy.pop("road_flow_quantization")
        catch.validate_header(legacy)
        changes = [lambda h: h.pop("prediction_rows"), lambda h: h.pop("road_flow_quantization"),
                   lambda h: h.update(prediction_rows=True),
                   lambda h: h.update(prediction_rows=h["prediction_rows"] + 1),
                   lambda h: h["road_flow_quantization"].update(step=0),
                   lambda h: h["road_flow_quantization"].update(step=5.),
                   lambda h: h["road_flow_quantization"].update(dense_until=True),
                   lambda h: h["road_flow_quantization"].update(tie="lower"),
                   lambda h: h["road_flow_quantization"].update(grid_origin=False),
                   lambda h: h["road_flow_quantization"].update(preserve_lane_lower_bound=1),
                   lambda h: h["road_flow_quantization"].update(unknown=1),
                   lambda h: h["road_flow_quantization"].pop("unit")]
        for change in changes:
            bad = copy.deepcopy(header)
            change(bad)
            with self.subTest(change=change), self.assertRaises(ValueError):
                catch.validate_header(bad)

    def test_max_rows_limits_coverage_not_predictor_rows(self):
        plan = catch.build_plan(self.args())
        self.assertLess(plan["prediction_rows"], plan["expanded_rows"])
        plan["resource_limits"]["requested_max_rows"] = plan["prediction_rows"]
        with self.assertRaisesRegex(ValueError, "Planned"):
            catch.enforce_budget(plan)
        plan["resource_limits"]["requested_max_rows"] = catch.MAX_CPP_ROWS
        plan["expanded_rows"] = catch.MAX_CPP_ROWS + 1
        with self.assertRaisesRegex(ValueError, "Planned"):
            catch.enforce_budget(plan)

    def test_comparison_reports_prediction_rows_and_preserves_coverage(self):
        plan = catch.build_plan(self.args())
        for item in catch.compare_signal_steps(plan, [1., 5.]):
            self.assertEqual(item["prediction_rows"] // item["signal_point_count"],
                             plan["prediction_rows"] // plan["signal_point_count"])
            self.assertEqual(item["expanded_rows"] // item["signal_point_count"],
                             plan["expanded_rows"] // plan["signal_point_count"])
            self.assertLess(item["prediction_rows"], item["expanded_rows"])
        argv = ["--model-dir", str(self.root / "model"), "--net-file", str(self.root / "net.xml"),
                "--output-dir", str(self.root / "dry"), "--signal-remaining-step", "5",
                "--road-flow-step", "5", "--road-flow-dense-until", "10", "--max-road-flow", "27",
                "--max-lane-flow", "16", "--cap-step", "1", "--dry-run", "--compare-signal-steps", "1", "5"]
        capture = io.StringIO()
        with redirect_stdout(capture):
            catch.main(argv)
        self.assertIn("road_flow_step_vehicles=5", capture.getvalue())
        self.assertIn("road_flow_dense_until_inclusive=10", capture.getvalue())
        self.assertIn("prediction_rows=", capture.getvalue())
        self.assertIn("expanded_rows=", capture.getvalue())
        self.assertFalse((self.root / "dry").exists())

    def test_large_span_hash_blocks_are_bounded_and_bitwise(self):
        values = np.array([1., 2.5, 9.], dtype="<f8")
        widths = np.array([700_000, 600_000, 400_000], dtype=np.int64)
        blocks = list(catch.expanded_prediction_blocks(values, widths))
        self.assertTrue(all(len(block) <= 131_072 * 8 for block in blocks))
        self.assertEqual(b"".join(blocks), np.repeat(values, widths).tobytes())


if __name__ == "__main__":
    unittest.main()
