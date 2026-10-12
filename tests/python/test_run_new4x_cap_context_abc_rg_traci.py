"""Isolated wrapper/driver tests. Fake files never invoke a real collector/SUMO."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import unittest
import uuid
from roadnet_paths import ROOT, source_path


HERE = Path(__file__).resolve().parent
WRAPPER = source_path("scripts/run_new4x_cap_context_abc_rg_traci.sh")
ACTUAL = Path(os.environ.get("ABC_REFERENCE_REPO", ROOT)).resolve()
GIT_BASH = Path(r"C:\Program Files\Git\bin\bash.exe")
BASH = str(GIT_BASH) if os.name == "nt" and GIT_BASH.is_file() else shutil.which("bash")
DEPS = [source_path(name).relative_to(ROOT).as_posix() for name in
        ("TraCI_Python_Cap_Context_ABC_RG.py", "signal_rg_features.py", "traci_signal_rg.py",
         "TraCI_Python_Cap_Context_ABC.py", "cap_context_abc_features.py",
         "TraCI_Python_Cap_Context.py", "cap_context_features.py", "TraCI_Python_Adjusted.py",
         "TraCI_Python_Threshold_Sweep.py", "turn_context.py")]
HEADER = "episode_id\tsplit\tsumocfg_file\tlegacy_training_output\tfull_training_output\tvehicle_event_output\tcycle_summary_output\trun_log\n"


@unittest.skipUnless(BASH and Path(BASH).exists(), "Bash is required")
class ABCRGWrapperTests(unittest.TestCase):
    def setUp(self):
        self.parent = HERE.resolve()
        self.root = self.parent / ("abc-rg-wrapper-test-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.repo = self.root / "repo"
        (self.repo / "scripts/collection").mkdir(parents=True)
        self.wrapper = self.repo / "scripts/collection" / WRAPPER.name
        self.wrapper.write_bytes(WRAPPER.read_bytes())
        for dependency in DEPS:
            (self.repo / dependency).parent.mkdir(parents=True, exist_ok=True)
            (self.repo / dependency).write_text("# test placeholder; never executed\n", encoding="utf-8")
        self.driver = source_path("run_all_training_traci.sh", self.repo)
        self.driver.write_text('#!/usr/bin/env bash\nset -eu\nprintf "[FAKE_DRIVER] collector=%s\\n" "$TRACI_SCRIPT"\n'
                               'printf "resume=%s thresholds=%s stop=%s low=%s queue=%s\\n" "$RESUME" "$LABEL_SPEED_THRESHOLDS" "$STOP_SPEED_THRESHOLD" "$LOW_SPEED_THRESHOLD" "$QUEUE_SPEED_THRESHOLD"\n'
                               'printf "args="; printf " %q" "$@"; printf "\\n"\n', encoding="utf-8")
        sumo_wrapper = source_path("scripts/sumo_sf_collection.sh", self.repo)
        sumo_wrapper.write_text("#!/usr/bin/env bash\nexit 97 # must never run\n", encoding="utf-8")
        sumo_wrapper.chmod(0o755)
        self.dataset = self.repo / "train_data/variants/new4x_sf"
        for folder in ("inputs", "commands", "manifests", "sumocfg", "routes"):
            (self.dataset / folder).mkdir(parents=True)
        (self.dataset / "inputs/dfbf38f23154_test_new.net.xml").write_text("<net/>\n", encoding="utf-8")
        (self.dataset / "manifests/materialization_run.json").write_text('{"schema":"explicit-speed-factor-v1"}\n', encoding="utf-8")
        self.rows = []
        for i in range(100):
            episode = f"episode_{i:03d}"
            split = "train" if i < 80 else "validation"
            self.rows.append([episode, split, f"sumocfg/{episode}.sumocfg",
                              f"outputs/traci/TraCI_output_adjusted_{episode}.csv",
                              f"outputs/traci/cams_model_training_data_{episode}.csv",
                              f"outputs/traci/cams_vehicle_events_{episode}.csv",
                              f"outputs/traci/cams_cycle_summary_{episode}.csv", f"outputs/logs/traci_{episode}.log"])
            (self.dataset / f"sumocfg/{episode}.sumocfg").write_text(
                '<configuration><input><net-file value="../inputs/dfbf38f23154_test_new.net.xml"/>'
                f'<route-files value="../routes/{episode}.rou.xml"/></input></configuration>\n', encoding="utf-8")
            (self.dataset / f"routes/{episode}.rou.xml").write_text("<routes/>\n", encoding="utf-8")
        self.save_jobs()

    def tearDown(self):
        target = self.root.resolve()
        if target.parent != self.parent or not target.name.startswith("abc-rg-wrapper-test-"):
            raise RuntimeError("Unsafe test cleanup target")
        shutil.rmtree(target)

    def save_jobs(self):
        (self.dataset / "commands/traci_jobs.tsv").write_text(
            HEADER + "".join("\t".join(row) + "\n" for row in self.rows), encoding="utf-8", newline="\n")

    def invoke(self, *args, env=None):
        variables = os.environ.copy()
        for name in ("DATASET_DIR", "DRY_RUN", "OUTPUT_ROOT", "RUN_TAG", "SUMO_BINARY", "SF_REAL_SUMO_BINARY", "START_INDEX"):
            variables.pop(name, None)
        if env:
            variables.update(env)
        return subprocess.run([BASH, self.wrapper.as_posix(), *args], cwd=self.repo, env=variables,
                              text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=90)

    def test_bash_syntax_and_help(self):
        result = subprocess.run([BASH, "-n", self.wrapper.as_posix()], capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = self.invoke("--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("A=0/B=4/C=5", result.stdout)
        self.assertIn("cap_context_abc_rg_v1", result.stdout)
        self.assertIn("9 formal features", result.stdout)
        self.assertIn("signal_rg_features", result.stdout)

    def test_fake_driver_100_dryrun_preserves_config_and_six_labels(self):
        result = self.invoke("new_abc_rg", "--jobs", "8", "--dry-run",
                             env={"STOP_SPEED_THRESHOLD": "99", "LOW_SPEED_THRESHOLD": "99", "QUEUE_SPEED_THRESHOLD": "99", "RESUME": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("episodes=100 train=80 validation=20", result.stdout)
        self.assertIn("collection/TraCI_Python_Cap_Context_ABC_RG.py", result.stdout)
        self.assertIn("schema=road-time-cap-context-abc-rg-v1 formal_features=9", result.stdout)
        self.assertIn("yellow skipped: no duration added", result.stdout)
        self.assertIn("continuous same-color duration spans phase boundaries", result.stdout)
        self.assertIn("resume=0 thresholds=0.1,0.5,1.0,1.5,2.0,3.0 stop=1.0 low=3.0 queue=2.0", result.stdout)
        self.assertIn("--jobs 8 --dry-run --no-resume", result.stdout)
        self.assertFalse((self.dataset / "traci_runs").exists())

    def test_env_dryrun_also_does_not_reserve_directory(self):
        result = self.invoke("new_abc_rg", "--jobs", "8", env={"DRY_RUN": "1"})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse((self.dataset / "traci_runs").exists())

    def test_real_branch_only_fake_driver_atomically_reserves_then_refuses(self):
        result = self.invoke("new_abc_rg", "--jobs", "8")
        self.assertEqual(result.returncode, 0, result.stderr)
        output = self.dataset / "traci_runs/new_abc_rg"
        self.assertTrue(output.is_dir())
        sentinel = output / "KEEP"
        sentinel.write_bytes(b"unchanged")
        result = self.invoke("new_abc_rg", "--dry-run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("already exists", result.stderr)
        self.assertEqual(sentinel.read_bytes(), b"unchanged")

    def test_reserved_options_and_unsafe_tag_rejected(self):
        for option in ("--resume", "--resume=1", "--output-root=x", "--run-tag=x",
                       "--context-crossing-time=3", "--context0-overdue-threshold=0",
                       "--stop-speed-threshold=1", "--low-speed-threshold=3", "--queue-speed-threshold=2"):
            with self.subTest(option=option):
                result = self.invoke("new_abc_rg", option, "--dry-run")
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("[FAKE_DRIVER]", result.stdout)
        for tag in ("../escape", "..", "/tmp/escape", "a/b"):
            with self.subTest(tag=tag):
                self.assertNotEqual(self.invoke(tag, "--dry-run").returncode, 0)

    def test_all_ten_dependencies_required(self):
        self.assertEqual(len(DEPS), 10)
        for dependency in DEPS:
            path = self.repo / dependency
            path.unlink()
            result = self.invoke("new_abc_rg", "--dry-run")
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(dependency, result.stderr)
            path.write_text("# not executed\n", encoding="utf-8")

    def test_wrong_80_20_split_rejected(self):
        self.rows[-1][1] = "train"
        self.save_jobs()
        result = self.invoke("new_abc_rg", "--dry-run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("train80/validation20", result.stderr)

    def test_duplicate_episode_rejected(self):
        self.rows[-1] = self.rows[0].copy()
        self.save_jobs()
        result = self.invoke("new_abc_rg", "--dry-run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("duplicate episode", result.stderr)

    def test_wrong_output_basename_and_missing_route_rejected(self):
        self.rows[0][4] = "outputs/other.csv"
        self.save_jobs()
        result = self.invoke("new_abc_rg", "--dry-run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("feature basename", result.stderr)
        self.rows[0][4] = "outputs/traci/cams_model_training_data_episode_000.csv"
        self.save_jobs()
        (self.dataset / "routes/episode_000.rou.xml").unlink()
        result = self.invoke("new_abc_rg", "--dry-run")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing explicit route", result.stderr)

    @unittest.skipUnless(source_path("run_all_training_traci.sh", ACTUAL).exists(), "Actual shared driver unavailable")
    def test_unchanged_real_driver_100_fake_jobs_dryrun_only(self):
        self.driver.write_bytes(source_path("run_all_training_traci.sh", ACTUAL).read_bytes())
        result = self.invoke("new_abc_rg", "--jobs", "8", "--dry-run")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("[DRY-RUN "), 100)
        self.assertIn("dry_run=100 skipped=0", result.stdout)
        self.assertIn("Parallel jobs: 8", result.stdout)
        self.assertFalse((self.dataset / "traci_runs").exists())

    @unittest.skipUnless((ACTUAL / "train_data/variants/new4x_sf/commands/traci_jobs.tsv").exists(), "Actual dataset unavailable")
    def test_actual_100_manifest_with_real_driver_readonly_dryrun(self):
        self.driver.write_bytes(source_path("run_all_training_traci.sh", ACTUAL).read_bytes())
        dataset = ACTUAL / "train_data/variants/new4x_sf"
        manifest = dataset / "commands/traci_jobs.tsv"
        before = hashlib.sha256(manifest.read_bytes()).hexdigest()
        tag = "abc_rg_wrapper_preflight_" + uuid.uuid4().hex
        result = self.invoke(tag, "--jobs", "8", "--dry-run", env={"DATASET_DIR": dataset.as_posix()})
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.count("[DRY-RUN "), 100)
        self.assertEqual(sum("[DRY-RUN " in line and "split=train" in line for line in result.stdout.splitlines()), 80)
        self.assertEqual(sum("[DRY-RUN " in line and "split=validation" in line for line in result.stdout.splitlines()), 20)
        self.assertFalse((dataset / "traci_runs" / tag).exists())
        self.assertEqual(hashlib.sha256(manifest.read_bytes()).hexdigest(), before)


if __name__ == "__main__":
    unittest.main()
