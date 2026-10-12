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

import legacy.catching.model_catching_sumo_cap_context as catch


class CapContextCatchTests(unittest.TestCase):
    def setUp(self):
        self.parent = Path.cwd().resolve()
        self.root = self.parent / ("f13-catch-test-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.run = self.root / "model"
        (self.run / "predictor").mkdir(parents=True)
        (self.run / "predictor" / "predictor.pkl").write_bytes(b"MOCK-NOT-AN-AUTOGLUON-MODEL")
        self.contract = {
            "schema": catch.FEATURE_SCHEMA, "raw_features": catch.FEATURE_COLS, "status": "complete",
            "model_label": catch.LABEL_COL, "source_label": "travel_time_label_gt_0p1",
            "predictor_directory": "predictor", "framework_features": catch.FEATURE_COLS,
            "context_contract": catch.CONTEXT_CONTRACT.copy(),
            "training_features": {"road_length": {"min": 10, "max": 20},
                                  "road_flow": {"min": 0, "max": 6},
                                  "lane_flow": {"min": 0, "max": 3},
                                  "free_speed_cap": {"min": 8, "max": 10},
                                  "platoon_context": {"min": 0, "max": 3}},
        }
        self.save_contract()
        self.net = self.root / "net.xml"
        self.net.write_text('<net><edge id="a"><lane length="10" speed="10"/></edge>'
                            '<edge id="b"><lane length="20" speed="10"/></edge></net>', encoding="utf-8")

    def tearDown(self):
        target = self.root.resolve()
        if target.parent != self.parent or not target.name.startswith("f13-catch-test-"):
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
        assert frame["platoon_context"].cat.categories.tolist() == [0, 1, 2, 3]
        # Equal road-flow intervals, context dependence, and a case below floor.
        return (frame["road_length"].to_numpy() / frame["free_speed_cap"].to_numpy()
                + frame["platoon_context"].astype(int).to_numpy() * .75
                + np.minimum(frame["road_flow"].to_numpy(), 2) * .25 - .5)

    def predictor(self, function=None):
        return SimpleNamespace(label=catch.LABEL_COL, problem_type="regression",
                               feature_metadata=SimpleNamespace(get_features=lambda: catch.FEATURE_COLS.copy()),
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
        self.assertEqual(catch.count_rows(2, grid, 222, 80), 5573448)
        self.assertEqual(catch.cap_index(13.89, grid), 100)
        self.assertEqual(catch.cap_index(54.5 * .1389, grid), 55)
        with self.assertRaises(ValueError):
            catch.cap_index(grid["min"] - 1e-6, grid)

    def test_exact_roundtrip_and_all_contexts(self):
        args, report = self.export()
        header, dense = catch.read_table(Path(args.output_dir) / "table.rle")
        expected = np.maximum(self.formula(catch.feature_frame(list(catch.iter_cases(header)))), 1.0)
        self.assertEqual(dense.tobytes(), np.asarray(expected, dtype="<f8").tobytes())
        self.assertEqual(dense.tobytes(), (Path(args.output_dir) / "reference.f64").read_bytes())
        self.assertTrue(report["roundtrip_exact"])
        self.assertLess(report["run_count"], report["rows"])
        self.assertGreater(report["predictions_floored_to_one_second"], 0)
        self.assertEqual(len(dense), 528)

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
        self.assertEqual(plan["expanded_rows"], 528)
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
        self.contract["context_contract"]["p_threshold"] = .1
        self.save_contract()
        with self.assertRaises(ValueError):
            catch.build_plan(self.args())

    def test_nonfinite_prediction_never_completes(self):
        with self.assertRaises(ValueError):
            self.export(lambda frame: np.full(len(frame), np.nan))
        self.assertFalse((self.root / "export" / "manifest.json").exists())
        self.assertFalse((self.root / "export" / "table.rle").exists())

    def test_zero_crossing_is_supported_and_failed_fit_is_rejected(self):
        self.contract["context_contract"]["crossing_time_seconds"] = 0.0
        self.save_contract()
        args, report = self.export()
        self.assertEqual(report["plan"]["context_contract"]["crossing_time_seconds"], 0.0)
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
        self.assertEqual(len(dense), 8)
        self.assertEqual(report["run_count"], 8)

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


if __name__ == "__main__":
    unittest.main()
