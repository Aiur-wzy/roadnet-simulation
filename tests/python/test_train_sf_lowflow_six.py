"""Small Bash integration tests using a stub; no CSV reads or real training."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import uuid
from roadnet_paths import source_path


HERE = Path(__file__).resolve().parent
DEFAULT_BASH = r"C:\Program Files\Git\bin\bash.exe" if os.name == "nt" else "bash"
BASH = os.environ.get("BASH_BIN", DEFAULT_BASH)
RUN_ROOT = HERE.parent / "batch_script_tests" / f"run_{uuid.uuid4().hex}"
ORDER = ["0p1", "3", "1", "0p5", "1p5", "2"]


class BatchScriptTests(unittest.TestCase):
    def setUp(self):
        # Retain these small generated fixtures for inspection; never delete source data.
        self.repo = RUN_ROOT / self._testMethodName / "repository with spaces"
        (self.repo / "scripts/training").mkdir(parents=True)
        trainer = source_path("model_training_sumo_sf_lowflow.py", self.repo)
        trainer.parent.mkdir(parents=True)
        shutil.copy2(source_path("scripts/train_sf_lowflow_six.sh"),
                     source_path("scripts/train_sf_lowflow_six.sh", self.repo))
        shutil.copy2(source_path("scripts/fixtures/recording_trainer.py"), trainer)
        self.capture = self.repo / "captured commands.jsonl"
        self.output = self.repo / "new model output"
        self.dataset = self.repo / "input data with spaces"
        self.run_dir = self.repo / "collection with spaces"
        self.env = os.environ.copy()
        self.env.update({"PYTHON_BIN": Path(sys.executable).as_posix(),
                         "DATASET": self.dataset.as_posix(), "RUN_DIR": self.run_dir.as_posix(),
                         "MODEL_ROOT": self.output.as_posix(), "TRAIN_GPUS": "0",
                         "STUB_CAPTURE_PATH": self.capture.as_posix()})
        self.env.pop("STUB_FAIL_LABEL", None)

    def run_batch(self, *args):
        return subprocess.run([BASH, str(source_path("scripts/train_sf_lowflow_six.sh", self.repo)), *args],
                              cwd=self.repo.parent, env=self.env, capture_output=True, text=True,
                              encoding="utf-8", errors="replace", timeout=30)

    def records(self):
        if not self.capture.exists():
            return []
        return [json.loads(line) for line in self.capture.read_text(encoding="utf-8").splitlines()]

    def test_six_commands_order_options_and_space_paths(self):
        result = self.run_batch()
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = self.records()
        self.assertEqual(len(rows), 6)
        expected = {"--expected-train-files": "80", "--expected-validation-files": "20",
                    "--sample-weighting": "low-flow", "--low-flow-cutoff": "5", "--low-flow-weight": "3",
                    "--time-limit": "5400", "--random-state": "42", "--num-gpus": "0",
                    "--bag-folds": "10", "--stack-levels": "2"}
        for row, cutoff in zip(rows, ORDER):
            args = row["args"]
            for flag, value in expected.items():
                self.assertEqual(args[args.index(flag) + 1], value)
            self.assertEqual(args[args.index("--label-column") + 1], f"travel_time_label_gt_{cutoff}")
            self.assertEqual(Path(args[args.index("--dataset-dir") + 1]), self.dataset)
            self.assertEqual(Path(args[args.index("--run-dir") + 1]), self.run_dir)
            self.assertEqual(Path(args[args.index("--save-path") + 1]), self.output / f"gt_{cutoff}")
            self.assertEqual(Path(row["cwd"]), self.repo)
            self.assertNotIn("--dry-run", args)

    def test_dry_run_reaches_all_six_without_model_output(self):
        result = self.run_batch("--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.records()), 6)
        self.assertTrue(all("--dry-run" in row["args"] for row in self.records()))
        self.assertFalse(self.output.exists())

    def test_failure_stops_before_third_threshold(self):
        self.env["STUB_FAIL_LABEL"] = "travel_time_label_gt_3"
        result = self.run_batch()
        self.assertEqual(result.returncode, 23)
        rows = self.records()
        self.assertEqual(len(rows), 2)
        self.assertIn("remaining thresholds were not started", result.stderr)

    def test_existing_output_is_rejected_before_python(self):
        self.output.mkdir()
        result = self.run_batch()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.records(), [])
        self.assertTrue(self.output.is_dir())

    def test_dry_run_allows_existing_output_without_modifying_it(self):
        self.output.mkdir()
        result = self.run_batch("--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(self.records()), 6)
        self.assertEqual(list(self.output.iterdir()), [])

    def test_help_and_unknown_argument_never_call_python(self):
        self.assertEqual(self.run_batch("--help").returncode, 0)
        self.assertNotEqual(self.run_batch("--unsupported").returncode, 0)
        self.assertEqual(self.records(), [])

    def test_default_paths_and_gpu_override(self):
        for key in ("DATASET", "RUN_DIR", "MODEL_ROOT"):
            self.env.pop(key)
        self.env["TRAIN_GPUS"] = "1"
        result = self.run_batch("--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.records()[0]["args"]
        self.assertEqual(Path(args[args.index("--dataset-dir") + 1]), self.repo / "train_data/variants/new4x_sf")
        self.assertEqual(Path(args[args.index("--run-dir") + 1]), self.repo / "train_data/variants/new4x_sf/traci_runs/sf_threshold_sweep")
        self.assertEqual(Path(args[args.index("--save-path") + 1]), self.repo / "models/sf_lowflow_lt5_x3_20260910/gt_0p1")
        self.assertEqual(args[args.index("--num-gpus") + 1], "1")

    def test_invalid_gpu_override_never_calls_python(self):
        self.env["TRAIN_GPUS"] = "-1"
        result = self.run_batch()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.records(), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
