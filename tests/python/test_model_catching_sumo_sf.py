"""Deterministic exporter contract tests; no AutoGluon models or SUMO required."""
import builtins
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import shutil
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock
import uuid

import numpy as np

import catching._shared.model_catching_sumo_sf as catching


class CatchingSFTests(unittest.TestCase):
    def setUp(self):
        self.test_parent = Path.cwd().resolve()
        self.root = self.test_parent / ("sf-catch-test-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.run = self.make_model("run")
        self.net = self.write_net()

    def tearDown(self):
        target = self.root.resolve()
        if target.parent != self.test_parent or not target.name.startswith("sf-catch-test-"):
            raise RuntimeError("Refusing unsafe fixture cleanup")
        shutil.rmtree(target)

    def contract(self):
        ranges = {
            "road_length": {"min": 10, "max": 30},
            "road_flow": {"min": 0, "max": 3},
            "lane_flow": {"min": 0, "max": 2},
            "speed_factor": {"min": 0.95, "max": 1.05},
            "speed_limit": {"min": 10, "max": 20},
        }
        return {
            "schema": catching.TRAINING_SCHEMA,
            "raw_features": catching.FEATURE_COLS.copy(),
            "model_label": catching.LABEL_COL,
            "source_label": "travel_time_label_gt_0p1",
            "predictor_directory": "predictor",
            "framework_features": catching.FEATURE_COLS.copy(),
            "training_features": ranges,
            "dependency_versions": {"autogluon.tabular": "test-only"},
        }

    def make_model(self, name, contract=None, pickle_present=True):
        run = self.root / name
        (run / "predictor").mkdir(parents=True)
        (run / "training_contract.json").write_text(
            json.dumps(self.contract() if contract is None else contract), encoding="utf-8")
        if pickle_present:
            (run / "predictor" / "predictor.pkl").write_bytes(b"mock predictor; not executable")
        return run

    def write_net(self, xml=None, name="net.xml"):
        path = self.root / name
        path.write_text(xml or (
            '<net><edge id=":ignored" function="internal"><lane length="bad"/></edge>'
            '<edge id="e1"><lane length="10" speed="10"/></edge>'
            '<edge id="e2"><lane length="20" speed="10"/>'
            '<lane length="20" speed="20"/></edge>'
            '<edge id="e3"><lane length="10" speed="10"/></edge></net>'
        ), encoding="utf-8")
        return path

    def argv(self, *extra):
        return [
            "--model-dir", str(self.run), "--net-file", str(self.net),
            "--output-dir", str(self.root / "export"), "--expected-threshold", "0p1",
            "--sf-step", "0.05", "--max-road-flow", "2", "--max-lane-flow", "1",
            "--chunk-size", "7", "--max-rows", "100", *extra,
        ]

    def args(self, *extra):
        return catching.build_parser().parse_args(self.argv(*extra))

    def predictor(self, feature_names=None, predict=None):
        names = catching.FEATURE_COLS if feature_names is None else feature_names
        return SimpleNamespace(
            label=catching.LABEL_COL, problem_type="regression",
            feature_metadata=SimpleNamespace(get_features=lambda: names.copy()),
            predict=mock.Mock(side_effect=predict) if predict is not None else
                mock.Mock(side_effect=lambda frame: np.full(len(frame), 12.0)),
        )

    def mocked_ag(self, predictor):
        package = ModuleType("autogluon")
        tabular = ModuleType("autogluon.tabular")
        tabular.TabularPredictor = SimpleNamespace(load=mock.Mock(return_value=predictor))
        package.tabular = tabular
        return mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}), tabular

    def run_export(self, args=None, predictor=None, plan=None):
        args = args or self.args()
        plan = plan or catching.build_plan(args)
        predictor = predictor or self.predictor()
        modules, tabular = self.mocked_ag(predictor)
        with modules, redirect_stdout(io.StringIO()):
            report = catching.export_table(args, plan)
        return report, predictor, tabular

    def test_model_run_and_predictor_directory_resolve_identically(self):
        direct = catching.resolve_model(self.run, "0p1")
        nested = catching.resolve_model(self.run / "predictor", "0p1")
        self.assertEqual(direct, nested)
        self.assertEqual(direct[0], self.run / "predictor")

    def test_model_directory_and_required_artifacts_rejected(self):
        folder = self.root / "not-a-model"
        folder.mkdir()
        with self.assertRaises(ValueError):
            catching.resolve_model(folder)
        run = self.make_model("no-pickle", pickle_present=False)
        with self.assertRaisesRegex(ValueError, "predictor.pkl"):
            catching.resolve_model(run)

    def test_schema_raw_features_label_and_predictor_contract_errors(self):
        cases = [
            ("schema", "legacy"), ("raw_features", catching.FEATURE_COLS[:-1]),
            ("raw_features", list(reversed(catching.FEATURE_COLS))),
            ("model_label", "travel_time"), ("predictor_directory", "../other"),
            ("source_label", "travel_time_label_gt_99"),
            ("framework_features", None), ("framework_features", ["turn_type"]),
        ]
        for number, (key, value) in enumerate(cases):
            with self.subTest(key=key, value=value):
                contract = self.contract()
                contract[key] = value
                with self.assertRaises(ValueError):
                    catching.resolve_model(self.make_model(f"bad-{number}", contract))

    def test_all_six_threshold_labels_and_explicit_mismatch(self):
        for threshold in catching.THRESHOLDS:
            with self.subTest(threshold=threshold):
                contract = self.contract()
                contract["source_label"] = "travel_time_label_gt_" + threshold
                run = self.make_model("threshold-" + threshold, contract)
                self.assertEqual(catching.resolve_model(run, threshold)[2]["source_label"],
                                 contract["source_label"])
        with self.assertRaisesRegex(ValueError, "Threshold mismatch"):
            catching.resolve_model(self.run, "3")

    def test_invalid_and_missing_training_ranges_fail(self):
        changes = [
            ("road_length", {}), ("road_flow", {"min": -1, "max": 2}),
            ("lane_flow", {"min": 2, "max": 1}),
            ("speed_factor", {"min": 0, "max": 1}),
            ("speed_limit", {"min": float("nan"), "max": 20}),
            ("speed_limit", {"min": 10, "max": float("inf")}),
        ]
        for number, (name, limits) in enumerate(changes):
            with self.subTest(name=name, limits=limits):
                contract = self.contract()
                contract["training_features"][name] = limits
                with self.assertRaises(ValueError):
                    catching.resolve_model(self.make_model(f"range-{number}", contract))

    def test_constant_feature_pruning_does_not_change_raw_schema(self):
        contract = self.contract()
        contract["framework_features"].remove("speed_limit")
        run = self.make_model("pruned", contract)
        _, _, actual = catching.resolve_model(run)
        self.assertEqual(actual["raw_features"], catching.FEATURE_COLS)
        self.assertNotIn("speed_limit", actual["framework_features"])

    def test_network_filters_internal_deduplicates_and_uses_mean_lane_limit(self):
        pairs, edge_count = catching.network_pairs(self.net)
        self.assertEqual(pairs, [(10.0, 10.0), (20.0, 15.0)])
        self.assertEqual(edge_count, 3)

    def test_network_keeps_distinct_length_limit_pairs(self):
        net = self.write_net('<net><edge id="a"><lane length="10" speed="10"/></edge>'
                             '<edge id="b"><lane length="10" speed="20"/></edge></net>')
        self.assertEqual(catching.network_pairs(net)[0], [(10.0, 10.0), (10.0, 20.0)])

    def test_bad_network_values_edge_ids_and_missing_roads_rejected(self):
        cases = [
            '<net/>', '<net><edge id="a"/></net>',
            '<net><edge><lane length="10" speed="10"/></edge></net>',
            '<net><edge id="a"><lane length="10" speed="10"/></edge>'
            '<edge id="a"><lane length="10" speed="10"/></edge></net>',
            '<net><edge id="a"><lane length="0" speed="10"/></edge></net>',
            '<net><edge id="a"><lane length="10" speed="nan"/></edge></net>',
            '<net><edge id="a"><lane length="10"/></edge></net>',
            '<net><edge id="a"><lane length="10" speed="10"/>'
            '<lane length="11" speed="10"/></edge></net>',
        ]
        for number, xml in enumerate(cases):
            with self.subTest(number=number):
                with self.assertRaises(ValueError):
                    catching.network_pairs(self.write_net(xml, f"invalid-{number}.xml"))

    def test_network_static_integer_key_collision_rejected(self):
        net = self.write_net('<net><edge id="a"><lane length="10.00001" speed="10"/></edge>'
                             '<edge id="b"><lane length="10.00002" speed="10"/></edge></net>')
        with self.assertRaisesRegex(ValueError, "collision"):
            catching.network_pairs(net)

    def test_network_static_precision_must_match_collector(self):
        net = self.write_net('<net><edge id="a"><lane length="10" speed="10"/>'
                             '<lane length="10" speed="10"/>'
                             '<lane length="10" speed="11"/></edge></net>')
        with self.assertRaisesRegex(ValueError, "four decimals"):
            catching.network_pairs(net)

    def test_sf_grid_integer_scale_outward_bounds_and_half_up(self):
        grid = catching.sf_grid(0.01, 0.947, 1.053)
        self.assertEqual((grid["scale"], grid["index_min"], grid["index_max"]), (100, 94, 106))
        self.assertEqual((grid["min"], grid["max"]), (0.94, 1.06))
        self.assertEqual(catching.sf_index(1.005, grid), 101)
        self.assertEqual(catching.sf_index(1.015, grid), 102)
        self.assertEqual(catching.sf_index(1.0049, grid), 100)
        self.assertEqual(catching.sf_index(0.94, grid), 94)
        self.assertEqual(catching.sf_index(1.06, grid), 106)
        self.assertEqual(catching.quantize_positive(10.00005), 100001)

    def test_sf_grid_invalid_step_and_bounds_rejected(self):
        for step, low, high in [(0, 0.9, 1.1), (-1, 0.9, 1.1), (0.03, 0.9, 1.1),
                               (1e-7, 0.9, 1.1), (0.1, 1.1, 0.9), (0.1, 0, 1.1),
                               (0.1, 0.05, 1.1), (float("nan"), 0.9, 1.1)]:
            with self.subTest(step=step, low=low, high=high):
                with self.assertRaises(ValueError):
                    catching.sf_grid(step, low, high)

    def test_sf_lookup_outside_domain_does_not_clamp(self):
        grid = catching.sf_grid(0.01, 0.9, 1.1)
        for value in [0.899, 1.101, 0, -1, float("nan"), float("inf")]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    catching.sf_index(value, grid)

    def test_count_rows_matches_stream_and_flow_constraint(self):
        for road_max in (0, 1, 2, 4):
            for lane_max in (0, 1, 3, 5):
                args = self.args("--max-road-flow", str(road_max), "--max-lane-flow", str(lane_max),
                                 "--max-rows", "1000")
                plan = catching.build_plan(args)
                records = list(catching.iter_cases(plan))
                self.assertEqual(len(records), catching.count_rows(
                    2, plan["speed_factor_grid"], road_max, lane_max))
                self.assertEqual(len(records), plan["planned_rows"])
                self.assertEqual(len(records), len(set(records)))
                self.assertTrue(all(0 <= row[2] <= row[1] <= road_max for row in records))
                self.assertTrue(all(row[2] <= lane_max for row in records))

    def test_build_plan_static_extrapolation_requires_explicit_switch(self):
        self.write_net('<net><edge id="a"><lane length="99" speed="25"/></edge></net>')
        with self.assertRaisesRegex(ValueError, "outside training range"):
            catching.build_plan(self.args())
        plan = catching.build_plan(self.args("--allow-static-extrapolation"))
        self.assertTrue(any("Network road_length" in item for item in plan["warnings"]))
        self.assertTrue(any("Network speed_limit" in item for item in plan["warnings"]))

    def test_plan_records_expansion_and_incomplete_domain_warnings(self):
        plan = catching.build_plan(self.args("--sf-min", "0.9", "--sf-max", "1.1"))
        self.assertTrue(any("Grid speed_factor" in item and "extends" in item for item in plan["warnings"]))
        self.assertTrue(any("Grid road_flow" in item and "does not cover" in item for item in plan["warnings"]))

    def test_size_guard_runs_before_case_enumeration(self):
        with mock.patch.object(catching, "iter_cases", side_effect=AssertionError("must not allocate")):
            with self.assertRaisesRegex(ValueError, "Planned 30 rows exceeds"):
                catching.build_plan(self.args("--max-rows", "29"))

    def test_invalid_numeric_options_rejected(self):
        for option, value in [("--max-road-flow", "-1"), ("--max-lane-flow", "-1"),
                              ("--chunk-size", "0"), ("--max-rows", "0")]:
            with self.subTest(option=option):
                with self.assertRaises(ValueError):
                    catching.build_plan(self.args(option, value))

    def test_existing_output_rejected_before_resolving_model(self):
        output = self.root / "export"
        output.mkdir()
        sentinel = output / "keep.txt"
        sentinel.write_text("original", encoding="utf-8")
        with mock.patch.object(catching, "resolve_model", side_effect=AssertionError("too late")):
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                catching.build_plan(self.args())
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "original")

    def test_batches_are_bounded_and_preserve_order(self):
        self.assertEqual(list(catching.batches(iter(range(8)), 3)), [[0, 1, 2], [3, 4, 5], [6, 7]])
        self.assertEqual(list(catching.batches(iter([]), 3)), [])

    def test_validate_predictor_label_type_and_feature_mismatches(self):
        plan = catching.build_plan(self.args())
        for attribute, value in [("label", "wrong"), ("problem_type", "binary")]:
            predictor = self.predictor()
            setattr(predictor, attribute, value)
            with self.subTest(attribute=attribute):
                with self.assertRaises(ValueError):
                    catching.validate_predictor(predictor, plan)
        with self.assertRaises(ValueError):
            catching.validate_predictor(self.predictor(["turn_type"]), plan)
        catching.validate_predictor(self.predictor(), plan)

    def test_mock_export_versioned_txt_manifest_hashes_columns_and_batches(self):
        args = self.args()
        report, predictor, tabular = self.run_export(args)
        output = self.root / "export"
        lines = (output / "table.txt").read_text(encoding="utf-8").splitlines()
        manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(lines[0], catching.TABLE_MAGIC)
        self.assertEqual(lines[2].split(), catching.FEATURE_COLS + [catching.LABEL_COL])
        self.assertEqual(len(lines) - 3, 30)
        self.assertEqual(report, manifest)
        self.assertEqual(manifest["status"], "complete")
        self.assertEqual(manifest["rows"], 30)
        self.assertEqual(manifest["table_sha256"], catching.sha256_file(output / "table.txt"))
        self.assertEqual(manifest["plan"], json.loads(lines[1]))
        self.assertEqual(manifest["plan"]["network"]["sha256"], catching.sha256_file(self.net))
        self.assertFalse(manifest["plan"]["legacy_cpp_compatible"])
        self.assertFalse((output / "table.txt.partial").exists())
        tabular.TabularPredictor.load.assert_called_once_with(str(self.run / "predictor"))
        self.assertEqual([len(call.args[0]) for call in predictor.predict.call_args_list], [7, 7, 7, 7, 2])
        for call in predictor.predict.call_args_list:
            self.assertEqual(list(call.args[0].columns), catching.FEATURE_COLS)
        records = [list(map(float, line.split())) for line in lines[3:]]
        self.assertTrue(all(row[-1] == 12.0 for row in records), "Never divide prediction by SF")
        self.assertEqual({row[3] for row in records}, {0.95, 1.0, 1.05})
        self.assertEqual({(row[0], row[4]) for row in records}, {(10.0, 10.0), (20.0, 15.0)})

    def test_constant_speed_limit_is_still_supplied_to_pruned_predictor(self):
        contract = self.contract()
        contract["framework_features"].remove("speed_limit")
        contract["training_features"]["speed_limit"] = {"min": 10, "max": 10}
        run = self.make_model("constant-run", contract)
        self.write_net('<net><edge id="a"><lane length="10" speed="10"/></edge></net>')
        predictor = self.predictor(contract["framework_features"])
        _, predictor, _ = self.run_export(self.args("--model-dir", str(run)), predictor)
        for call in predictor.predict.call_args_list:
            frame = call.args[0]
            self.assertEqual(list(frame.columns), catching.FEATURE_COLS)
            self.assertTrue((frame["speed_limit"] == 10).all())

    def test_prediction_floor_one_and_raw_statistics(self):
        values = [-2.0, 0.5, 1.0, 2.0, 3.0]
        args = self.args("--chunk-size", "100")
        report, _, _ = self.run_export(args, self.predictor(
            predict=lambda frame: np.resize(np.asarray(values), len(frame))))
        self.assertEqual(report["predictions_floored_to_one_second"], 12)
        self.assertEqual((report["raw_prediction_min"], report["raw_prediction_max"]), (-2.0, 3.0))
        rows = (self.root / "export" / "table.txt").read_text(encoding="utf-8").splitlines()[3:]
        self.assertTrue(all(float(line.split()[-1]) >= 1 for line in rows))

    def test_nan_infinite_and_wrong_shape_predictions_leave_only_partial(self):
        for number, output_value in enumerate((float("nan"), float("inf"), "wrong-shape")):
            with self.subTest(value=output_value):
                output = self.root / f"failed-{number}"
                args = self.args("--output-dir", str(output))
                plan = catching.build_plan(args)
                if output_value == "wrong-shape":
                    predictor = self.predictor(predict=lambda frame: np.ones((len(frame), 1)))
                else:
                    predictor = self.predictor(predict=lambda frame, value=output_value:
                                               np.full(len(frame), value))
                modules, _ = self.mocked_ag(predictor)
                with modules, redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(ValueError, "Non-finite/wrong-shape"):
                        catching.export_table(args, plan)
                self.assertTrue((output / "table.txt.partial").exists())
                self.assertFalse((output / "table.txt").exists())
                self.assertFalse((output / "manifest.json").exists())
                with self.assertRaisesRegex(ValueError, "Output already exists"):
                    catching.build_plan(args)

    def test_predictor_failure_after_a_batch_retains_partial_and_no_complete_manifest(self):
        calls = []
        def fail_second(frame):
            calls.append(len(frame))
            if len(calls) > 1:
                raise RuntimeError("mock inference failed")
            return np.full(len(frame), 12.0)
        args = self.args()
        plan = catching.build_plan(args)
        modules, _ = self.mocked_ag(self.predictor(predict=fail_second))
        with modules, redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "mock inference failed"):
                catching.export_table(args, plan)
        output = self.root / "export"
        self.assertEqual(len((output / "table.txt.partial").read_text(encoding="utf-8").splitlines()), 10)
        self.assertFalse((output / "manifest.json").exists())
        self.assertFalse((output / "table.txt").exists())

    def test_predictor_metadata_failure_creates_no_output(self):
        args = self.args()
        plan = catching.build_plan(args)
        modules, _ = self.mocked_ag(self.predictor(["turn_type"]))
        with modules:
            with self.assertRaises(ValueError):
                catching.export_table(args, plan)
        self.assertFalse((self.root / "export").exists())

    def test_row_count_mismatch_retains_partial(self):
        args = self.args()
        plan = catching.build_plan(args)
        plan["planned_rows"] += 1
        modules, _ = self.mocked_ag(self.predictor())
        with modules, redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(ValueError, "row count mismatch"):
                catching.export_table(args, plan)
        self.assertTrue((self.root / "export" / "table.txt.partial").exists())
        self.assertFalse((self.root / "export" / "manifest.json").exists())

    def test_no_resume_or_overwrite_after_success(self):
        args = self.args()
        plan = catching.build_plan(args)
        self.run_export(args, plan=plan)
        table = self.root / "export" / "table.txt"
        before = catching.sha256_file(table)
        with mock.patch.object(builtins, "__import__", side_effect=AssertionError("must not import")):
            with self.assertRaisesRegex(ValueError, "Output already exists"):
                catching.export_table(args, plan)
        self.assertEqual(catching.sha256_file(table), before)

    def test_dry_run_no_autogluon_import_or_output_writes(self):
        original_import = builtins.__import__
        def reject_ag(name, *args, **kwargs):
            if name == "autogluon" or name.startswith("autogluon."):
                raise AssertionError("dry-run must not load AutoGluon")
            return original_import(name, *args, **kwargs)
        before = sorted(str(path.relative_to(self.root)) for path in self.root.rglob("*"))
        with mock.patch.object(builtins, "__import__", side_effect=reject_ag), redirect_stdout(io.StringIO()) as log:
            plan = catching.main(self.argv("--dry-run"))
        self.assertEqual(plan["planned_rows"], 30)
        self.assertIn("[DRY-RUN] Passed", log.getvalue())
        self.assertEqual(before, sorted(str(path.relative_to(self.root)) for path in self.root.rglob("*")))


if __name__ == "__main__":
    unittest.main()
