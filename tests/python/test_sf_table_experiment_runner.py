"""Read-only runner unit checks; no simulator or fixture directories are created."""

import importlib.util
from pathlib import Path
import unittest
from unittest import mock


from roadnet_paths import source_path

SCRIPT = source_path("scripts/run_sf_table_experiment.py")
SPEC = importlib.util.spec_from_file_location("sf_table_runner", SCRIPT)
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class RunnerChecks(unittest.TestCase):
    BASELINE_LOG = "Finished vehicles: 10 / 10\nInvalid vehicles: 0\n[TravelTime] mode: speed-net\n"
    TABLE_LOG = (
        "Finished vehicles: 10 / 10\nInvalid vehicles: 0\n[TravelTime] mode: table\n"
        "[TravelTime] table format: CAMS_SF_TABLE_V1\n"
        "[TravelTime] table hits: 40\n[TravelTime] table misses: 0\n"
        "[TravelTime] fallback: disabled (strict SF table)\n"
        "[SFTable] queries=40 road_flow_outside_training=4 lane_flow_outside_training=2 road_flow_max_seen=52 lane_flow_max_seen=41\n"
    )

    def test_baseline_without_sf_table_audit(self):
        value = RUNNER.parse_run_log(self.BASELINE_LOG, RUNNER.BASELINE, 10)
        self.assertEqual(value["finished_vehicles"], 10)
        self.assertNotIn("table_hits", value)

    def test_table_audit_and_extrapolation(self):
        value = RUNNER.parse_run_log(self.TABLE_LOG, "gt_0p1", 10)
        self.assertEqual(value["table_misses"], 0)
        self.assertEqual(value["sf_queries"], 40)
        self.assertEqual(value["road_flow_outside_training_fraction"], 0.1)
        self.assertEqual(value["lane_flow_outside_training_fraction"], 0.05)

    def test_windows_log_line_endings(self):
        value = RUNNER.parse_run_log(self.TABLE_LOG.replace("\n", "\r\n"), "gt_0p1", 10)
        self.assertEqual(value["table_hits"], 40)

    def test_missing_table_magic_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG.replace("[TravelTime] table format: CAMS_SF_TABLE_V1\n", ""), "gt_1", 10)

    def test_nonzero_miss_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG.replace("table misses: 0", "table misses: 1"), "gt_1", 10)

    def test_no_hits_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG.replace("table hits: 40", "table hits: 0"), "gt_1", 10)

    def test_enabled_fallback_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG.replace("disabled (strict SF table)", "speed-net"), "gt_1", 10)

    def test_mismatched_query_count_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG.replace("queries=40", "queries=41"), "gt_1", 10)

    def test_outside_count_cannot_exceed_queries(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG.replace("road_flow_outside_training=4", "road_flow_outside_training=41"), "gt_1", 10)

    def test_incomplete_and_invalid_vehicles_rejected(self):
        for log in (self.TABLE_LOG.replace("10 / 10", "9 / 10"), self.TABLE_LOG.replace("Invalid vehicles: 0", "Invalid vehicles: 1")):
            with self.subTest(log=log), self.assertRaises(ValueError):
                RUNNER.parse_run_log(log, "gt_1", 10)

    def test_duplicate_log_audit_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.TABLE_LOG + "[TravelTime] table hits: 40\n", "gt_1", 10)

    def test_wrong_mode_rejected(self):
        with self.assertRaises(ValueError):
            RUNNER.parse_run_log(self.BASELINE_LOG, "gt_1", 10)

    def test_existing_output_refused_before_loading_inputs(self):
        with mock.patch.object(RUNNER, "import_helper") as helper:
            with self.assertRaises(FileExistsError):
                RUNNER.main(["--repo", "missing", "--binary", "missing", "--baseline-binary", "missing",
                             "--table-root", "missing", "--output", str(SCRIPT.parent)])
            helper.assert_not_called()


if __name__ == "__main__":
    unittest.main()
