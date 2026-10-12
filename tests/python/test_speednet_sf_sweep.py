"""Small deterministic runner tests; no SUMO or simulation processes needed."""

import json
from pathlib import Path
import shutil
import subprocess
import unittest
from unittest import mock
import uuid

import evaluation.speednet_sf_sweep as sweep


class SweepTests(unittest.TestCase):
    def setUp(self):
        test_workspace = Path(__file__).resolve().parent
        # Python 3.12 TemporaryDirectory(mode=0700) conflicts with the Windows
        # sandbox's ACLs; inherit workspace permissions for disposable fixtures.
        self.root = test_workspace / ("sf_sweep_test_" + uuid.uuid4().hex)
        self.root.mkdir()
        def cleanup():
            if self.root.resolve().parent != test_workspace:
                raise ValueError("Test cleanup path escaped its workspace")
            shutil.rmtree(self.root)
        self.addCleanup(cleanup)

    def write(self, name, contents):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents, encoding="utf-8")
        return path

    def pair(self, headway="0.2", delay="3"):
        return {"id": sweep.parameter_id(headway, delay), "headway": headway,
                "delay_max": delay, "labels": []}

    def input_data(self):
        return {"net": "test.net.xml", "route": "test.rou.xml", "truth": "tripinfo.xml",
                "expected_vehicles": 2, "truth_durations": {"v0": 10.0, "v1": 20.0}}

    def fake_process(self, command, **kwargs):
        run_dir = Path(kwargs["cwd"])
        (run_dir / "prediction_tripinfo.xml").write_text(
            '<tripinfos><tripinfo id="v0" duration="8"/><tripinfo id="v1" duration="25"/></tripinfos>',
            encoding="utf-8")
        (run_dir / "sumo_eval.csv").write_text("retained,never parsed\n", encoding="utf-8")
        (run_dir / "automatic_extra.xml").write_text("<extra/>", encoding="utf-8")
        kwargs["stdout"].write(b"Finished vehicles: 2 / 2\nInvalid vehicles: 0\n")
        return subprocess.CompletedProcess(command, 0)

    def test_parameter_id_normalizes_decimals(self):
        self.assertEqual(sweep.parameter_id("0.2000", "3e0"), "h0p2_d3")
        self.assertEqual(sweep.parameter_id("-0", "0.00"), "h0_d0")
        self.assertEqual(sweep.parameter_id(0.55, 3.125), "h0p55_d3p125")

    def test_parameter_id_rejects_invalid_values(self):
        for value in (-1, "nan", "inf", True, "abc", 1000001):
            with self.subTest(value=value), self.assertRaises(ValueError):
                sweep.parameter_decimal(value)

    def test_grid_deduplicates_numeric_identity_only(self):
        path = self.write("grid.json", json.dumps([
            {"headway": 0.2, "delay_max": 3, "name": "first"},
            {"headway": "0.200", "delay_max": 3, "case_name": "same"},
            {"headway": 0, "delay_max": 3}, {"headway": 0, "delay_max": 4},
        ]))
        pairs = sweep.load_grid(path)
        self.assertEqual(len(pairs), 3)
        self.assertEqual(pairs[0]["labels"], ["first", "same"])

    def test_independent_metrics(self):
        values = sweep.metrics([-2, 5])
        self.assertEqual(values["mae_s"], 3.5)
        self.assertEqual(values["bias_s"], 1.5)
        self.assertAlmostEqual(values["rmse_s"], (29 / 2) ** 0.5)
        self.assertAlmostEqual(values["p95_abs_error_s"], 4.85)

    def test_metrics_reject_empty_or_nonfinite(self):
        for values in ([], [1, float("nan")], [float("inf")]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                sweep.metrics(values)

    def test_records_reject_duplicate_vehicle_ids(self):
        path = self.write("duplicate.xml", '<tripinfos><tripinfo id="v0"/><tripinfo id="v0"/></tripinfos>')
        with self.assertRaises(ValueError):
            sweep.read_records(path, "tripinfo")

    def test_log_requires_exact_completion_and_no_invalid(self):
        valid = self.write("valid.log", "Finished vehicles: 2 / 2\nInvalid vehicles: 0\n")
        sweep.validate_log(valid, 2)
        for contents in ("", "Finished vehicles: 1 / 2\nInvalid vehicles: 0",
                         "Finished vehicles: 2 / 2\nInvalid vehicles: 1",
                         "Finished vehicles: 2 / 2\n"):
            invalid = self.write("invalid.log", contents)
            with self.subTest(contents=contents), self.assertRaises(ValueError):
                sweep.validate_log(invalid, 2)

    def test_load_manifest_relative_paths_and_explicit_sf(self):
        self.write("fixture/net.xml", "<net/>")
        self.write("fixture/route.xml", '<routes><vehicle id="v0" speedFactor="0.9"/><vehicle id="v1" speedFactor="1.1"/></routes>')
        self.write("fixture/truth.xml", '<tripinfos><tripinfo id="v0" duration="10"/><tripinfo id="v1" duration="20"/></tripinfos>')
        self.write("fixture/case.sumocfg", '<configuration><input><net-file value="net.xml"/><route-files value="route.xml"/></input><output><tripinfo-output value="truth.xml"/></output></configuration>')
        manifest = self.write("manifest.json", json.dumps({"cases": [{"name": "independent", "config": "fixture/case.sumocfg", "expected_vehicles": 2}]}))
        inputs, hashes = sweep.load_inputs(self.root, manifest)
        self.assertEqual(inputs["independent"]["truth_durations"], {"v0": 10, "v1": 20})
        self.assertEqual(len(hashes), 5)
        self.write("fixture/route.xml", '<routes><vehicle id="v0"/><vehicle id="v1" speedFactor="1.1"/></routes>')
        with self.assertRaisesRegex(ValueError, "explicit finite positive SF"):
            sweep.load_inputs(self.root, manifest)

    def test_reject_unowned_output_directory(self):
        output = self.root / "unowned"
        output.mkdir()
        for resume in (False, True):
            with self.subTest(resume=resume), self.assertRaises(ValueError):
                sweep.prepare_experiment(output, {"hashes": {}}, [self.pair()], resume)

    def test_resume_appends_parameters_without_losing_previous(self):
        output = self.root / "output"
        fingerprint = {"hashes": {"source": "abc"}}
        sweep.prepare_experiment(output, fingerprint, [self.pair()], False)
        result = sweep.prepare_experiment(output, fingerprint, [self.pair("0.3", "4")], True)
        self.assertEqual(set(result["parameters"]), {"h0p2_d3", "h0p3_d4"})
        with self.assertRaises(ValueError):
            sweep.prepare_experiment(output, {"hashes": {"source": "changed"}}, [self.pair()], True)

    def test_execute_preserves_artifacts_and_reuses_success(self):
        output = self.root / "output"
        fingerprint = {"hashes": {}}
        with mock.patch.object(sweep.subprocess, "run", side_effect=self.fake_process) as process:
            result = sweep.execute_run(output, Path("binary.exe"), self.pair(), "sample", self.input_data(), fingerprint, 30)
            reused = sweep.execute_run(output, Path("binary.exe"), self.pair(), "sample", self.input_data(), fingerprint, 30)
        self.assertEqual(process.call_count, 1)
        self.assertEqual(result, reused)
        self.assertEqual(result["independent_metrics"]["mae_s"], 3.5)
        self.assertIn("automatic_extra.xml", result["output_hashes"])
        command = result["command"]
        self.assertEqual(command[command.index("--speed-net-speed-factor") + 1], "on")
        self.assertEqual(command[command.index("--speed-net-queue-trim") + 1], "off")

    def test_changed_successful_artifact_is_never_reused(self):
        output = self.root / "output"
        with mock.patch.object(sweep.subprocess, "run", side_effect=self.fake_process):
            result = sweep.execute_run(output, Path("binary.exe"), self.pair(), "sample", self.input_data(), {}, 30)
        (Path(result["cwd"]) / "automatic_extra.xml").write_text("<changed/>", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "output missing, added, or changed"):
            sweep.execute_run(output, Path("binary.exe"), self.pair(), "sample", self.input_data(), {}, 30)

    def test_failed_attempt_is_retained_and_retried_separately(self):
        output = self.root / "output"
        with mock.patch.object(sweep.subprocess, "run", side_effect=RuntimeError("intentional failure")):
            with self.assertRaises(RuntimeError):
                sweep.execute_run(output, Path("binary.exe"), self.pair(), "sample", self.input_data(), {}, 30)
        with mock.patch.object(sweep.subprocess, "run", side_effect=self.fake_process):
            result = sweep.execute_run(output, Path("binary.exe"), self.pair(), "sample", self.input_data(), {}, 30)
        self.assertEqual(Path(result["cwd"]).name, "attempt_0002")
        first = Path(result["cwd"]).parent / "attempt_0001/run.json"
        self.assertEqual(sweep.read_json(first)["status"], "failed")

    def test_summary_pooled_is_vehicle_weighted_not_scenario_mean(self):
        output = self.root / "output"
        pair = self.pair()
        experiment = sweep.prepare_experiment(output, {}, [pair], False)
        inputs = {"large": self.input_data(), "small": {**self.input_data(), "expected_vehicles": 1, "truth_durations": {"v0": 10}}}
        def process(command, **kwargs):
            result = self.fake_process(command, **kwargs)
            if "small" in str(kwargs["cwd"]):
                directory = Path(kwargs["cwd"])
                (directory / "prediction_tripinfo.xml").write_text('<tripinfos><tripinfo id="v0" duration="20"/></tripinfos>', encoding="utf-8")
                kwargs["stdout"].seek(0)
                kwargs["stdout"].truncate()
                kwargs["stdout"].write(b"Finished vehicles: 1 / 1\nInvalid vehicles: 0\n")
            return result
        with mock.patch.object(sweep.subprocess, "run", side_effect=process):
            for name, data in inputs.items():
                sweep.execute_run(output, Path("binary.exe"), pair, name, data, {}, 30)
        result = sweep.summarize(output, experiment, inputs)["parameters"][0]
        self.assertAlmostEqual(result["pooled"]["mae_s"], 17 / 3)
        self.assertAlmostEqual(result["pooled"]["bias_s"], 13 / 3)
        self.assertEqual(result["equal_scenario_mae_s"], 6.75)


if __name__ == "__main__":
    unittest.main()
