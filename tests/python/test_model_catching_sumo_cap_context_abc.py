"""Real exporter/decoder with injected mock predictions, never a model fit."""
import copy
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import struct
import shutil
import uuid
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import pandas as pd
from roadnet_paths import ROOT

import catching._shared.model_catching_sumo_cap_context_abc as catch
import legacy.catching.model_catching_sumo_cap_context as old_catch
import training._shared.model_training_sumo_cap_context_abc as trainer
from features.cap_context_abc_features import abc_context_contract


class ABCCapContextCatchTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path.cwd().resolve()
        self.root = self.parent / ("f13-abc-catch-test-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.run = self.root / "model"
        (self.run / "predictor").mkdir(parents=True)
        (self.run / "predictor" / "predictor.pkl").write_bytes(b"MOCK-NOT-AN-AUTOGLUON-MODEL")
        self.contract = {
            "schema": catch.FEATURE_SCHEMA, "raw_features": catch.FEATURE_COLS, "status": "complete",
            "model_label": catch.LABEL_COL, "source_label": "travel_time_label_gt_0p1",
            "predictor_directory": "predictor", "framework_features": catch.FEATURE_COLS,
            "context_contract": abc_context_contract(),
            "categorical_features": {"platoon_context": [0, 1, 2, 3, 4, 5]},
            "categorical_ordered": False,
            "training_features": {"road_length": {"min": 10, "max": 20},
                                  "road_flow": {"min": 0, "max": 6},
                                  "lane_flow": {"min": 0, "max": 3},
                                  "free_speed_cap": {"min": 8, "max": 10},
                                  "platoon_context": {
                                      "min": 0, "max": 5, "unique": 6, "constant": False,
                                      "type": "categorical", "categories": [0, 1, 2, 3, 4, 5],
                                      "ordered": False, "category_names": dict(trainer.CONTEXT_NAMES),
                                      "observed_categories": [0, 1, 2, 3, 4, 5],
                                      "counts": {str(value): 1 for value in range(6)},
                                  }},
        }
        self.save_contract()
        self.net = self.root / "net.xml"
        self.net.write_text('<net><edge id="a"><lane length="10" speed="10"/></edge>'
                            '<edge id="b"><lane length="20" speed="10"/></edge></net>', encoding="utf-8")

    def tearDown(self):
        target = self.root.resolve()
        if target.parent != self.parent or not target.name.startswith("f13-abc-catch-test-"):
            raise RuntimeError("Unsafe test cleanup target")
        shutil.rmtree(target)

    def save_contract(self):
        (self.run / "training_contract.json").write_text(json.dumps(self.contract), encoding="utf-8")

    def args(self, *extra):
        return catch.build_parser().parse_args([
            "--model-dir", str(self.run), "--net-file", str(self.net),
            "--output-dir", str(self.root / "export"), "--cap-step", "1",
            "--max-road-flow", "6", "--max-lane-flow", "3", "--chunk-size", "7",
            "--dense-reference", *extra])

    @staticmethod
    def formula(frame):
        assert list(frame) == catch.FEATURE_COLS
        assert isinstance(frame["platoon_context"].dtype, pd.CategoricalDtype)
        assert frame["platoon_context"].cat.categories.tolist() == [0, 1, 2, 3, 4, 5]
        assert not frame["platoon_context"].cat.ordered
        # Equal road-flow intervals, context dependence, and a case below floor.
        return (frame["road_length"].to_numpy() / frame["free_speed_cap"].to_numpy()
                + frame["platoon_context"].astype(int).to_numpy() * .75
                + np.minimum(frame["road_flow"].to_numpy(), 2) * .25 - .5)

    def predictor(self, function=None):
        return SimpleNamespace(label=catch.LABEL_COL, problem_type="regression",
                               feature_metadata=SimpleNamespace(
                                   get_features=lambda: catch.FEATURE_COLS.copy(),
                                   get_feature_type_raw=lambda column: "category" if column == "platoon_context" else "float",
                                   type_map_raw={column: "category" if column == "platoon_context" else "float" for column in catch.FEATURE_COLS}),
                               predict=function or self.formula)

    def export(self, function=None, args=None):
        args = args or self.args()
        plan = catch.build_plan(args)
        with redirect_stdout(io.StringIO()):
            report = catch.export_table(args, plan, self.predictor(function))
        return args, report

    def mutated(self, path, edit_header=None, edit_payload=None, rehash=True):
        with path.open("rb") as stream:
            magic = stream.readline()
            header = json.loads(stream.readline())
            payload = bytearray(stream.read())
        if edit_header:
            edit_header(header)
        if edit_payload:
            edit_payload(header, payload)
        if rehash:
            header["payload_sha256"] = hashlib.sha256(payload).hexdigest()
        out = self.root / "mutated.rle"
        out.write_bytes(magic + json.dumps(header).encode() + b"\n" + payload)
        return out

    def test_current_grid_budget_and_rounding(self):
        grid = catch.cap_grid(.1389, .5464 * 13.89, 13.89)
        self.assertEqual((grid["index_min"], grid["index_max"]), (54, 100))
        self.assertEqual(catch.count_rows(2, grid, 222, 80), 8360172)
        self.assertEqual(catch.cap_index(13.89, grid), 100)
        self.assertEqual(catch.cap_index(54.5 * .1389, grid), 55)
        with self.assertRaises(ValueError):
            catch.cap_index(grid["min"] - 1e-6, grid)

    def test_real_project_network_six_threshold_dryrun_with_fixture_contract(self):
        # Real network, synthetic completed contract; not the server predictor.
        network = ROOT / "data/net/test_new.net.xml"
        self.assertTrue(network.is_file())
        self.contract["training_features"]["road_length"].update(min=320., max=545.6)
        self.contract["training_features"]["free_speed_cap"].update(min=.5464 * 13.89, max=13.89)
        for token in ("0p1", "3", "1", "0p5", "1p5", "2"):
            self.contract["source_label"] = "travel_time_label_gt_" + token
            self.save_contract()
            args = self.args("--net-file", str(network), "--cap-step", "0.1389",
                             "--max-road-flow", "222", "--max-lane-flow", "80",
                             "--expected-threshold", token, "--dry-run")
            with mock.patch.dict("sys.modules", {"autogluon": None, "autogluon.tabular": None}):
                plan = catch.build_plan(args)
            self.assertEqual(plan["road_lengths"], [320., 545.6])
            self.assertEqual(plan["expanded_rows"], 8360172)
            self.assertEqual(plan["group_count"], 45684)
            self.assertEqual(plan["flow_count"], 14823)
            self.assertFalse(Path(args.output_dir).exists())

    def test_exact_roundtrip_and_all_contexts(self):
        args, report = self.export()
        header, dense = catch.read_table(Path(args.output_dir) / "table.rle")
        expected = np.maximum(self.formula(catch.feature_frame(list(catch.iter_cases(header)))), 1.0)
        self.assertEqual(dense.tobytes(), np.asarray(expected, dtype="<f8").tobytes())
        self.assertEqual(dense.tobytes(), (Path(args.output_dir) / "reference.f64").read_bytes())
        self.assertTrue(report["roundtrip_exact"])
        self.assertLess(report["run_count"], report["rows"])
        self.assertGreater(report["predictions_floored_to_one_second"], 0)
        self.assertEqual(len(dense), 792)

    def test_chunk_size_does_not_change_payload(self):
        args, first = self.export()
        second_args = self.args("--output-dir", str(self.root / "other"), "--chunk-size", "1000")
        _, second = self.export(args=second_args)
        self.assertEqual(first["plan"]["payload_sha256"], second["plan"]["payload_sha256"])
        self.assertEqual(first["dense_sha256"], second["dense_sha256"])

    def test_dryrun_does_not_load_model_or_write(self):
        args = self.args("--dry-run")
        with mock.patch.dict("sys.modules", {"autogluon": None, "autogluon.tabular": None}):
            plan = catch.build_plan(args)
        self.assertEqual(plan["expanded_rows"], 792)
        self.assertFalse(Path(args.output_dir).exists())

    def test_original_sf_contract_rejected(self):
        self.contract["raw_features"] = ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"]
        self.save_contract()
        with self.assertRaises(ValueError):
            catch.build_plan(self.args())

    def test_invalid_contract_threshold_and_budget(self):
        with self.assertRaises(ValueError):
            catch.build_plan(self.args("--expected-threshold", "3"))
        with self.assertRaises(ValueError):
            catch.build_plan(self.args("--max-rows", "100"))
        with self.assertRaises(ValueError):
            catch.build_plan(self.args("--max-lane-flow", "7"))
        self.contract["context_contract"]["base_context_contract"]["p_threshold"] = .1
        self.save_contract()
        with self.assertRaises(ValueError):
            catch.build_plan(self.args())

    def test_nonfinite_prediction_never_completes(self):
        with self.assertRaises(ValueError):
            self.export(lambda frame: np.full(len(frame), np.nan))
        self.assertFalse((self.root / "export" / "manifest.json").exists())
        self.assertFalse((self.root / "export" / "table.rle").exists())

    def test_zero_crossing_is_supported_and_failed_fit_is_rejected(self):
        self.contract["context_contract"]["base_context_contract"]["crossing_time_seconds"] = 0.0
        self.save_contract()
        args, report = self.export()
        self.assertEqual(report["plan"]["context_contract"]["base_context_contract"]["crossing_time_seconds"], 0.0)
        for status in (None, "fit_started", "failed"):
            self.contract["status"] = status
            self.save_contract()
            with self.assertRaises(ValueError):
                catch.resolve_model(self.run)

    def test_existing_output_never_overwritten(self):
        args, _ = self.export()
        with self.assertRaises(ValueError):
            catch.build_plan(args)

    def test_bad_sha_trailing_and_truncation_rejected(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        changed = self.mutated(path, edit_payload=lambda h, b: b.__setitem__(-1, b[-1] ^ 1), rehash=False)
        with self.assertRaises(ValueError):
            catch.read_table(changed)
        for payload in (path.read_bytes()[:-1], path.read_bytes() + b"x"):
            changed.write_bytes(payload)
            with self.assertRaises(ValueError):
                catch.read_table(changed)

    def test_bad_coverage_offsets_values_and_dimensions(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        mutations = [
            lambda h, b: struct.pack_into("<H", b, 4 * (h["group_count"] + 1), 1),
            lambda h, b: struct.pack_into("<I", b, 0, 1),
            lambda h, b: struct.pack_into("<d", b, 4 * (h["group_count"] + 1) + 4, float("nan")),
        ]
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                changed = self.mutated(path, edit_payload=mutation)
                with self.assertRaises(ValueError):
                    catch.read_table(changed)
        changed = self.mutated(path, edit_header=lambda h: h.__setitem__("expanded_rows", 50_000_001))
        with self.assertRaises(ValueError):
            catch.read_table(changed)

    def test_one_node_zero_flow_valid(self):
        args, report = self.export(args=self.args("--max-road-flow", "0", "--max-lane-flow", "0", "--cap-min", "10", "--cap-max", "10"))
        _, dense = catch.read_table(Path(args.output_dir) / "table.rle")
        self.assertEqual(len(dense), 12)
        self.assertEqual(report["run_count"], 12)

    def test_json_numeric_types_and_static_precision_rejected(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        edits = [lambda h: h.__setitem__("prediction_floor_seconds", True),
                 lambda h: h["free_speed_cap_grid"].__setitem__("step", "1"),
                 lambda h: h["free_speed_cap_grid"].__setitem__("step", True),
                 lambda h: h["free_speed_cap_grid"].__setitem__("min", "8"),
                 lambda h: h["free_speed_cap_grid"].__setitem__("raw_boundary_tolerance", "1e-12"),
                 lambda h: h["road_lengths"].__setitem__(0, 10.00001)]
        for edit in edits:
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                catch.read_table(self.mutated(path, edit_header=edit))

    def test_export_rejects_heterogeneous_lane_network_like_runtime(self):
        for second in ('length="10" speed="9"', 'length="10.0000001" speed="10"'):
            self.net.write_text('<net><edge id="a"><lane length="10" speed="10"/>'
                                f'<lane {second}/></edge></net>', encoding="utf-8")
            with self.assertRaises(ValueError):
                catch.build_plan(self.args())

    def test_feature_frame_exactly_matches_training_dtype_and_order(self):
        rows = [(10.0, 4, 2, 8.0, value) for value in range(6)]
        source = pd.DataFrame(rows, columns=catch.FEATURE_COLS)
        pd.testing.assert_frame_equal(catch.feature_frame(rows), trainer.encode_feature_frame(source))

    def test_invalid_categories_are_not_silently_converted(self):
        for value in (-1, 6, 1.5, True, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                catch.feature_frame([(10, 2, 1, 8, value)])

    def test_all_six_thresholds_dryrun_no_autogluon_or_outputs(self):
        for token in ("0p1", "3", "1", "0p5", "1p5", "2"):
            self.contract["source_label"] = "travel_time_label_gt_" + token
            self.save_contract()
            args = self.args("--expected-threshold", token, "--dry-run")
            with self.subTest(token=token), mock.patch.dict("sys.modules", {"autogluon": None, "autogluon.tabular": None}):
                plan = catch.build_plan(args)
            self.assertEqual(plan["context_count"], 6)
            self.assertEqual(plan["training_schema"], trainer.SCHEMA)
            self.assertEqual(plan["expanded_rows"], 792)
            self.assertFalse(Path(args.output_dir).exists())

    def test_direct_predictor_directory_resolves_same_contract(self):
        direct = catch.resolve_model(self.run / "predictor", "0p1")
        parent = catch.resolve_model(self.run, "0p1")
        self.assertEqual(direct, parent)

    def test_original_four_state_model_is_rejected_and_reverse(self):
        with self.assertRaises(ValueError):
            old_catch.resolve_model(self.run)
        self.contract["schema"] = old_catch.FEATURE_SCHEMA
        self.contract["context_contract"] = old_catch.CONTEXT_CONTRACT.copy()
        self.save_contract()
        with self.assertRaises(ValueError):
            catch.resolve_model(self.run)

    def test_category_mapping_requires_all_six_in_correct_order(self):
        baseline = copy.deepcopy(self.contract)
        changes = [
            lambda c: c.__setitem__("categorical_features", {"platoon_context": [0, 1, 2, 3, 4]}),
            lambda c: c.__setitem__("categorical_features", {"platoon_context": [0, 1, 2, 3, 5, 4]}),
            lambda c: c.__setitem__("categorical_ordered", True),
            lambda c: c.__setitem__("categorical_ordered", 0),
            lambda c: c["training_features"]["platoon_context"].__setitem__("ordered", True),
            lambda c: c["training_features"]["platoon_context"].__setitem__("categories", [0, 1, 2, 3, 4]),
            lambda c: c["training_features"]["platoon_context"]["category_names"].__setitem__("4", "C"),
        ]
        for change in changes:
            self.contract = copy.deepcopy(baseline)
            change(self.contract)
            self.save_contract()
            with self.subTest(change=change), self.assertRaises(ValueError):
                catch.resolve_model(self.run)

    def test_abc_contract_tampering_is_rejected(self):
        baseline = copy.deepcopy(self.contract)
        changes = [
            lambda c: c["context_contract"]["context_mapping"].__setitem__("5", "B"),
            lambda c: c["context_contract"].__setitem__("overdue_formula", "wrong"),
            lambda c: c["context_contract"].__setitem__("context0_overdue_threshold_seconds", True),
            lambda c: c["context_contract"].__setitem__("context0_overdue_threshold_seconds", -1),
            lambda c: c["context_contract"].__setitem__("context0_overdue_threshold_seconds", float("nan")),
            lambda c: c["context_contract"]["base_context_contract"].__setitem__("crossing_time_seconds", -1),
        ]
        for change in changes:
            self.contract = copy.deepcopy(baseline)
            change(self.contract)
            self.save_contract()
            with self.subTest(change=change), self.assertRaises(ValueError):
                catch.resolve_model(self.run)

    def test_nondefault_abc_threshold_is_preserved_not_recomputed(self):
        self.contract["context_contract"] = abc_context_contract(threshold=2.5)
        self.save_contract()
        args, _ = self.export()
        header, _ = catch.read_table(Path(args.output_dir) / "table.rle")
        self.assertEqual(header["context_contract"], self.contract["context_contract"])

    def test_unused_category_still_has_full_export_grid(self):
        info = self.contract["training_features"]["platoon_context"]
        info.update(max=4, unique=5, observed_categories=[0, 1, 2, 3, 4])
        info["counts"]["5"] = 0
        self.save_contract()
        plan = catch.build_plan(self.args())
        self.assertEqual(plan["expanded_rows"], 792)
        self.assertEqual({case[-1] for case in catch.iter_cases(plan)}, set(range(6)))
        self.assertTrue(any("platoon_context" in item for item in plan["warnings"]))

    def test_constant_pruned_feature_is_accepted_without_adding_metadata(self):
        self.contract["framework_features"] = [column for column in catch.FEATURE_COLS if column != "road_length"]
        self.save_contract()
        plan = catch.build_plan(self.args())
        predictor = self.predictor()
        predictor.feature_metadata.get_features = lambda: self.contract["framework_features"]
        catch.validate_predictor(predictor, plan)

    def test_mock_predictions_do_not_merge_new_categories(self):
        def unique(frame):
            self.formula(frame)
            return 10.0 + frame["platoon_context"].astype(int).to_numpy()
        args, _ = self.export(unique, self.args("--text-format", "both"))
        header, dense = catch.read_table(Path(args.output_dir) / "table.rle")
        self.assertEqual(set(dense.tolist()), {10.0, 11.0, 12.0, 13.0, 14.0, 15.0})
        for fmt in ("txt", "csv"):
            path = Path(args.output_dir) / ("table." + fmt)
            verified = catch.validate_readable_table(path, Path(args.output_dir) / "table.rle", fmt)
            self.assertEqual(verified["rows"], 792)
            lines = path.read_text(encoding="utf-8").splitlines()[1:]
            delimiter = "," if fmt == "csv" else " "
            rows = [line.split(delimiter) for line in lines]
            self.assertEqual({int(row[4]) for row in rows}, set(range(6)))
            for row in rows:
                self.assertEqual(float(row[5]), 10 + int(row[4]))

    def test_independent_grid_and_rle_text_roundtrip(self):
        args, report = self.export(args=self.args("--text-format", "both"))
        header, dense = catch.read_table(Path(args.output_dir) / "table.rle")
        expected = []
        for length in (10.0, 20.0):
            for cap in (8.0, 9.0, 10.0):
                for context in range(6):
                    for lane in range(4):
                        for road in range(lane, 7):
                            expected.append(max(length / cap + context * .75 + min(road, 2) * .25 - .5, 1.0))
        self.assertEqual(dense.tobytes(), np.asarray(expected, dtype="<f8").tobytes())
        self.assertEqual(header["context_count"], 6)
        self.assertEqual(header["format"], "CAMS_CAP_CONTEXT_ABC_RLE_V1")
        self.assertEqual(header["training_schema"], "road-time-cap-context-abc-v1")
        self.assertEqual(report["dense_sha256"], hashlib.sha256(np.asarray(expected, dtype="<f8").tobytes()).hexdigest())

    def test_decoder_rejects_changed_context_count_schema_and_abc_rules(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        edits = [
            lambda h: h.__setitem__("context_count", 4),
            lambda h: h.__setitem__("context_count", 5),
            lambda h: h.__setitem__("context_count", 6.0),
            lambda h: h.__setitem__("training_schema", old_catch.FEATURE_SCHEMA),
            lambda h: h["context_contract"]["context_mapping"].__setitem__("4", "C"),
        ]
        for edit in edits:
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                catch.read_table(self.mutated(path, edit_header=edit))
        with self.assertRaises(ValueError):
            old_catch.read_table(path)

    def test_readable_corruption_of_category_is_rejected(self):
        args, _ = self.export(args=self.args("--text-format", "csv"))
        path = Path(args.output_dir) / "table.csv"
        lines = path.read_text(encoding="utf-8").splitlines()
        fields = lines[1].split(",")
        fields[4] = "5"
        lines[1] = ",".join(fields)
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            catch.validate_readable_table(path, Path(args.output_dir) / "table.rle", "csv")

    def test_text_output_is_opt_in(self):
        args, _ = self.export()
        self.assertFalse((Path(args.output_dir) / "table.txt").exists())
        self.assertFalse((Path(args.output_dir) / "table.csv").exists())

    def test_source_weights_are_not_reapplied_at_catch(self):
        self.contract["sample_weighting"] = {"mode": "low-flow", "weight": 3}
        self.save_contract()
        args, _ = self.export(lambda frame: np.full(len(frame), 2.25))
        _, dense = catch.read_table(Path(args.output_dir) / "table.rle")
        self.assertTrue(np.equal(dense, 2.25).all())

    def test_predictor_numeric_context_metadata_is_rejected(self):
        plan = catch.build_plan(self.args())
        predictor = self.predictor()
        predictor.feature_metadata.get_feature_type_raw = lambda column: "int"
        predictor.feature_metadata.type_map_raw["platoon_context"] = "int"
        with self.assertRaises(ValueError):
            catch.validate_predictor(predictor, plan)

    def test_raw_category_with_valid_framework_binary_transform_is_accepted(self):
        plan = catch.build_plan(self.args())
        predictor = self.predictor()
        predictor.feature_metadata_in = self.predictor().feature_metadata
        predictor.feature_metadata.get_feature_type_raw = lambda column: "int"
        predictor.feature_metadata.type_map_raw["platoon_context"] = "int"
        catch.validate_predictor(predictor, plan)

    def test_missing_interior_training_class_is_warned_without_dropping_it(self):
        info = self.contract["training_features"]["platoon_context"]
        info.update(unique=5, observed_categories=[0, 1, 2, 4, 5])
        info["counts"]["3"] = 0
        self.save_contract()
        plan = catch.build_plan(self.args())
        self.assertEqual({case[-1] for case in catch.iter_cases(plan)}, set(range(6)))
        self.assertTrue(any("3" in warning and ("observ" in warning or "missing" in warning)
                            for warning in plan["warnings"]))

    def test_training_range_boolean_or_string_rejected(self):
        for value in (True, "1"):
            with self.subTest(value=value):
                self.contract["training_features"]["road_flow"]["min"] = value
                self.save_contract()
                with self.assertRaises(ValueError):
                    catch.resolve_model(self.run)

    def test_each_readable_format_predicts_each_node_only_once(self):
        for fmt, expected in (("txt", {"txt"}), ("csv", {"csv"}), ("both", {"txt", "csv"})):
            with self.subTest(fmt=fmt):
                seen = []
                def predict(frame):
                    seen.append(len(frame))
                    return self.formula(frame)
                args, report = self.export(predict, self.args("--output-dir", str(self.root / fmt), "--text-format", fmt))
                self.assertEqual(sum(seen), 792)
                self.assertEqual(set(report["readable_tables"]), expected)

    def test_stream_scan_needs_no_dense_allocation(self):
        args, report = self.export()
        with mock.patch.object(catch.np, "empty", side_effect=AssertionError("dense allocation")):
            header, digest = catch.scan_table(Path(args.output_dir) / "table.rle")
        self.assertEqual(digest, report["dense_sha256"])
        self.assertEqual(header["expanded_rows"], 792)

    def test_actual_trainer_generated_contract_to_catch_all_thresholds_with_mock_backend(self):
        # Exercise the real contract writer, never AutoGluon training or pickle loading.
        raw = pd.DataFrame({
            "road_length": [10., 20., 10., 20., 10., 20.],
            "free_speed_cap": [8., 10., 8., 10., 8., 10.],
            "road_flow": [1, 2, 3, 4, 5, 6], "lane_flow": [0, 1, 1, 2, 2, 3],
            "platoon_context": list(range(6)),
        })
        frame = trainer.encode_feature_frame(raw)
        frame[trainer.LABEL_COL] = [2., 3., 4., 5., 6., 7.]
        frame["vehicleID"] = [f"v{index}" for index in range(6)]
        frame["route_index"] = [0, 1, 0, 1, 0, 1]
        frame["is_final_road"] = [0, 1, 0, 1, 0, 1]
        frame["__source_file"] = "synthetic-abc-integration-only"
        frame.attrs["context_contract"] = abc_context_contract()
        fixture = self

        class MockBackend:
            def __init__(self, **options):
                self.path = Path(options["path"])
                self.label, self.problem_type = options["label"], options["problem_type"]
                self.feature_metadata = fixture.predictor().feature_metadata

            def fit(self, **options):
                fixture.assertEqual(list(options["train_data"]), catch.FEATURE_COLS + [trainer.LABEL_COL])
                self.path.mkdir(parents=True)
                (self.path / "predictor.pkl").write_bytes(b"SYNTHETIC-CONTRACT-TEST-NOT-A-PICKLE")
                return self

            def predict(self, source):
                return fixture.formula(source)

        for token in ("0p1", "3", "1", "0p5", "1p5", "2"):
            run = self.root / ("trained_" + token)
            options = trainer.build_parser().parse_args([
                "--save-path", str(run), "--sample-weighting", "none",
                "--label-column", "travel_time_label_gt_" + token,
                "--time-limit", "1", "--bag-folds", "2", "--stack-levels", "0",
            ])
            with self.subTest(token=token), redirect_stdout(io.StringIO()), \
                    mock.patch.dict("sys.modules", {"autogluon.tabular": SimpleNamespace(TabularPredictor=MockBackend)}), \
                    mock.patch.object(trainer, "version", return_value="1.6.1"):
                predictor = trainer.fit_model(frame, frame.copy(), options, {"evidence": "mock-not-training"})
                args = self.args("--model-dir", str(run), "--output-dir", str(self.root / ("caught_" + token)),
                                 "--expected-threshold", token, "--text-format", "both")
                plan = catch.build_plan(args)
                report = catch.export_table(args, plan, predictor)
            contract = json.loads((run / "training_contract.json").read_text(encoding="utf-8"))
            self.assertEqual(contract["status"], "complete")
            self.assertEqual(plan["context_contract"], contract["context_contract"])
            self.assertEqual(plan["context_count"], 6)
            self.assertEqual(report["status"], "complete")
            self.assertTrue(report["roundtrip_exact"])
            self.assertEqual(set(report["readable_tables"]), {"txt", "csv"})


if __name__ == "__main__":
    unittest.main()
