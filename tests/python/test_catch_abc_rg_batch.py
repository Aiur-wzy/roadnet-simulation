"""Small real-format/mock-predictor tests; never load a real AutoGluon model."""
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import unittest
import uuid
from unittest.mock import patch

from roadnet_paths import ROOT, source_path
sys.path.insert(0, os.environ.get("ABC_REFERENCE_REPO", str(ROOT)))
import catching.model_catching_sumo_cap_context_abc_rg as catch
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, mock_predictor

spec = importlib.util.spec_from_file_location("catch_batch", source_path("scripts/catch_abc_rg_batch.py"))
batch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(batch)


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.root = ROOT / (".test_rg_batch_" + uuid.uuid4().hex)
        self.root.mkdir()
        assert self.root.resolve().parent == ROOT.resolve()
        self.addCleanup(shutil.rmtree, self.root)
        model, self.net, _ = prepare_fixture(self.root / "fixture")
        self.models = self.root / "models"
        for token in batch.DEFAULT_THRESHOLDS:
            target = self.models / ("gt_" + token)
            shutil.copytree(model, target)
            contract = json.loads((target / "training_contract.json").read_text())
            contract["source_label"] = "travel_time_label_gt_" + token
            (target / "training_contract.json").write_text(json.dumps(contract), encoding="utf-8")
        self.out = self.root / "output"
        self.called = []

    def args(self, *extra):
        return batch.parser().parse_args([
            "--model-root", str(self.models), "--net-file", str(self.net),
            "--output-root", str(self.out), "--cap-step", "1", "--max-road-flow", "3",
            "--max-lane-flow", "2", *extra])

    def mock_child(self, command, stream):
        self.assertIn("-m", command)
        module_index = command.index("-m") + 1
        self.assertEqual(command[module_index], "catching.model_catching_sumo_cap_context_abc_rg")
        args = catch.build_parser().parse_args(command[module_index + 1:])
        self.called.append(args.expected_threshold)
        plan = catch.build_plan(args)
        catch.export_table(args, plan, mock_predictor())
        stream.write("MOCK_EXPORT_COMPLETE\n")
        return 0

    def run_batch(self, *extra):
        with patch.object(batch, "run_child", side_effect=self.mock_child), redirect_stdout(io.StringIO()):
            batch.run(self.args(*extra), catch)

    def test_dry_run_all_five_no_output_no_prediction(self):
        with patch.object(batch, "run_child", side_effect=AssertionError("no child allowed")), redirect_stdout(io.StringIO()):
            batch.run(self.args("--dry-run"), catch)
        self.assertFalse(self.out.exists())

    def test_serial_five_order_step_and_resume(self):
        self.run_batch()
        self.assertEqual(self.called, list(batch.DEFAULT_THRESHOLDS))
        self.assertFalse((self.out / "gt_0p1").exists())
        for token in self.called:
            manifest = json.loads((self.out / ("gt_" + token) / "manifest.json").read_text())
            self.assertEqual(manifest["plan"]["signal_remaining_step"], 5.)
            self.assertEqual(manifest["plan"]["prediction_rows"], manifest["plan"]["expanded_rows"])
        hashes = {p: catch.abc.sha256_file(p) for p in self.out.rglob("table.rle")}
        self.called.clear()
        self.run_batch("--resume")
        self.assertEqual(self.called, [])
        self.assertEqual(hashes, {p: catch.abc.sha256_file(p) for p in hashes})
        self.assertFalse((self.out / ".catch_batch.lock").exists())

    def test_existing_requires_resume(self):
        self.run_batch("--thresholds", "1")
        with self.assertRaisesRegex(ValueError, "use --resume"):
            self.run_batch("--thresholds", "1")

    def test_partial_directory_preserved(self):
        partial = self.out / "gt_1"
        partial.mkdir(parents=True)
        (partial / "table.rle.partial").write_bytes(b"incomplete")
        with self.assertRaisesRegex(ValueError, "Incomplete output"):
            self.run_batch("--resume")
        self.assertEqual((partial / "table.rle.partial").read_bytes(), b"incomplete")
        self.assertEqual(self.called, [])

    def test_late_missing_model_blocks_all_exports(self):
        (self.models / "gt_2" / "training_contract.json").rename(self.models / "gt_2" / "saved_contract.json")
        with self.assertRaises(FileNotFoundError):
            self.run_batch()
        self.assertEqual(self.called, [])

    def test_child_failure_stops_and_keeps_previous_completion(self):
        real_mock = self.mock_child
        def fail_second(command, stream):
            if command[command.index("--expected-threshold") + 1] == "3":
                return 7
            return real_mock(command, stream)
        with patch.object(batch, "run_child", side_effect=fail_second), redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "gt_3 exited 7"):
                batch.run(self.args(), catch)
        self.assertEqual(self.called, ["1"])
        self.assertTrue((self.out / "gt_1" / "manifest.json").is_file())
        self.assertFalse((self.out / "gt_0p5").exists())
        self.assertFalse((self.out / ".catch_batch.lock").exists())
        self.called.clear()
        self.run_batch("--resume")
        self.assertEqual(self.called, ["3", "0p5", "1p5", "2"])

    def test_step_mismatch_refused(self):
        self.run_batch("--thresholds", "1")
        with self.assertRaisesRegex(ValueError, "differs from requested"):
            self.run_batch("--thresholds", "1", "--resume", "--signal-remaining-step", "2")

    def test_road_flow_arguments_forwarded_and_logged(self):
        args = self.args("--thresholds", "1", "--road-flow-step", "5",
                         "--road-flow-dense-until", "10", "--max-road-flow", "16")
        argv = batch.job_arguments(args, "1")
        options = catch.build_parser().parse_args(argv)
        self.assertEqual(options.road_flow_step, 5)
        self.assertEqual(options.road_flow_dense_until, 10)
        stream = io.StringIO()
        with redirect_stdout(stream):
            plan = batch.preflight(args, catch)[0][3]
        self.assertLess(plan["prediction_rows"], plan["expanded_rows"])
        self.assertIn("signal_step=5s", stream.getvalue())
        self.assertIn("road_flow_step=5veh road_flow_dense_until=10veh", stream.getvalue())
        self.assertIn(f"prediction_rows={plan['prediction_rows']}", stream.getvalue())
        self.assertIn(f"expanded_rows={plan['expanded_rows']}", stream.getvalue())
        self.assertFalse(self.out.exists())

    def test_road_flow_matching_resume_skips_complete_export(self):
        extra = ("--thresholds", "1", "--road-flow-step", "5",
                 "--road-flow-dense-until", "10", "--max-road-flow", "16")
        self.run_batch(*extra)
        path = self.out / "gt_1" / "table.rle"
        original = catch.abc.sha256_file(path)
        self.called.clear()
        self.run_batch(*extra, "--resume")
        self.assertEqual(self.called, [])
        self.assertEqual(catch.abc.sha256_file(path), original)

    def test_piecewise_arguments_forwarded_and_logged(self):
        args = self.args("--thresholds", "1", "--max-road-flow", "16",
                         "--road-flow-segments", "2:1", "10:5", "16:8")
        options = catch.build_parser().parse_args(batch.job_arguments(args, "1"))
        self.assertEqual(options.road_flow_segments, ["2:1", "10:5", "16:8"])
        stream = io.StringIO()
        with redirect_stdout(stream):
            plan = batch.preflight(args, catch)[0][3]
        self.assertEqual(plan["road_flow_quantization"]["nodes"], [0, 1, 2, 5, 10, 16])
        self.assertLess(plan["prediction_rows"], plan["expanded_rows"])
        self.assertIn("road_flow_grid=explicit road_flow_nodes=6", stream.getvalue())
        self.assertIn(f"prediction_rows={plan['prediction_rows']}", stream.getvalue())
        self.assertFalse(self.out.exists())

    def test_piecewise_resume_preserves_complete_table(self):
        extra = ("--thresholds", "1", "--max-road-flow", "16",
                 "--road-flow-segments", "2:1", "10:5", "16:8")
        self.run_batch(*extra)
        path = self.out / "gt_1" / "table.rle"
        original = catch.abc.sha256_file(path)
        self.called.clear()
        self.run_batch(*extra, "--resume")
        self.assertEqual(self.called, [])
        self.assertEqual(catch.abc.sha256_file(path), original)

    def test_piecewise_resume_rejects_different_nodes(self):
        self.run_batch("--thresholds", "1", "--max-road-flow", "16",
                       "--road-flow-segments", "2:1", "10:5", "16:8")
        with self.assertRaisesRegex(ValueError, "differs from requested"):
            self.run_batch("--thresholds", "1", "--max-road-flow", "16", "--resume",
                           "--road-flow-segments", "2:1", "10:4", "16:8")

    def test_piecewise_conflicting_and_invalid_grid_fails_before_export(self):
        for extra in (("--road-flow-step", "5", "--road-flow-segments", "16:2"),
                      ("--road-flow-dense-until", "4", "--road-flow-segments", "16:2"),
                      ("--road-flow-segments", "16:0")):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                self.run_batch("--thresholds", "1", "--max-road-flow", "16", *extra)
        self.assertEqual(self.called, [])

    def test_road_flow_step_mismatch_refused(self):
        self.run_batch("--thresholds", "1", "--max-road-flow", "16",
                       "--road-flow-step", "5")
        with self.assertRaisesRegex(ValueError, "differs from requested"):
            self.run_batch("--thresholds", "1", "--max-road-flow", "16",
                           "--road-flow-step", "2", "--resume")

    def test_road_flow_dense_range_mismatch_refused(self):
        self.run_batch("--thresholds", "1", "--max-road-flow", "16",
                       "--road-flow-step", "5", "--road-flow-dense-until", "10")
        with self.assertRaisesRegex(ValueError, "differs from requested"):
            self.run_batch("--thresholds", "1", "--max-road-flow", "16",
                           "--road-flow-step", "5", "--road-flow-dense-until", "4", "--resume")

    def test_manifest_road_flow_metadata_mismatch_refused(self):
        self.run_batch("--thresholds", "1", "--road-flow-step", "5")
        path = self.out / "gt_1" / "manifest.json"
        data = json.loads(path.read_text())
        data["plan"]["road_flow_quantization"] = {"tampered": True}
        path.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "header disagree"):
            self.run_batch("--thresholds", "1", "--road-flow-step", "5", "--resume")

    def test_manifest_prediction_count_mismatch_refused(self):
        self.run_batch("--thresholds", "1", "--road-flow-step", "5")
        path = self.out / "gt_1" / "manifest.json"
        data = json.loads(path.read_text())
        data["prediction_rows"] += 1
        path.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "prediction-row count mismatch"):
            self.run_batch("--thresholds", "1", "--road-flow-step", "5", "--resume")

    def test_contract_change_refused(self):
        self.run_batch("--thresholds", "1")
        path = self.models / "gt_1" / "training_contract.json"
        path.write_text(path.read_text() + "\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "differs from requested"):
            self.run_batch("--thresholds", "1", "--resume")

    def test_corrupt_table_refused(self):
        self.run_batch("--thresholds", "1")
        path = self.out / "gt_1" / "table.rle"
        data = path.read_bytes()
        path.write_bytes(data[:-1] + bytes([data[-1] ^ 1]))
        with self.assertRaisesRegex(ValueError, "SHA mismatch"):
            self.run_batch("--thresholds", "1", "--resume")

    def test_manifest_header_mismatch_refused(self):
        self.run_batch("--thresholds", "1")
        path = self.out / "gt_1" / "manifest.json"
        data = json.loads(path.read_text())
        data["plan"]["signal_remaining_step"] = 2
        path.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "header disagree"):
            self.run_batch("--thresholds", "1", "--resume")

    def test_lock_conflict_preserves_existing_lock(self):
        self.out.mkdir()
        lock = self.out / ".catch_batch.lock"
        lock.write_text("another job")
        with self.assertRaisesRegex(ValueError, "Batch lock exists"):
            self.run_batch()
        self.assertEqual(lock.read_text(), "another job")

    def test_duplicate_and_invalid_step(self):
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            self.run_batch("--thresholds", "1", "1")
        for value in ("0", "-1", "nan", "inf"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.run_batch("--signal-remaining-step", value)

    def test_invalid_road_flow_grid_refused_before_export(self):
        for flag, value in (("--road-flow-step", "0"), ("--road-flow-step", "-1"),
                            ("--road-flow-dense-until", "-1")):
            with self.subTest(flag=flag, value=value), self.assertRaises(ValueError):
                self.run_batch("--thresholds", "1", flag, value)
        self.assertEqual(self.called, [])

    def test_fractional_road_flow_grid_rejected_by_parser(self):
        for flag in ("--road-flow-step", "--road-flow-dense-until"):
            with self.subTest(flag=flag), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                self.args(flag, "2.5")

    def test_real_child_return_code_and_log(self):
        with (self.root / "child.log").open("w", encoding="utf-8") as stream:
            code = batch.run_child([sys.executable, "-c", "print('fixture'); raise SystemExit(7)"], stream)
        self.assertEqual(code, 7)
        self.assertIn("fixture", (self.root / "child.log").read_text())


if __name__ == "__main__":
    unittest.main()
