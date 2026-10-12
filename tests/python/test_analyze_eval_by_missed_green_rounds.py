import importlib.util
import math
import unittest
from pathlib import Path


from roadnet_paths import ROOT, source_path

SCRIPT_PATH = source_path("scripts/analyze_eval_by_missed_green_rounds.py")
SPEC = importlib.util.spec_from_file_location("missed_green_analysis", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MissedGreenAnalysisTest(unittest.TestCase):
    def test_groups_mae_and_truth_duration_after_vehicle_join(self):
        fixture_root = ROOT / "testdata" / "missed_green_analysis"
        summaries, joined, coverage = MODULE.analyze(
            str(fixture_root / "eval.csv"), str(fixture_root / "traci.csv")
        )
        by_group = {row["groupName"]: row for row in summaries}

        self.assertEqual([row["groupName"] for row in summaries], ["ALL", "0", "2"])
        self.assertEqual(len(joined), 3)
        self.assertEqual(coverage["matchedVehicles"], 3)
        self.assertEqual(coverage["evalVehiclesMissingTraCI"], 1)
        self.assertEqual(coverage["traciVehiclesMissingEval"], 1)
        self.assertEqual(coverage["invalidEvalVehiclesSkipped"], 1)
        self.assertEqual(by_group["0"]["n"], 2)
        self.assertAlmostEqual(by_group["0"]["mae"], 2.0)
        self.assertAlmostEqual(by_group["0"]["meanTruthDuration"], 9.0)
        self.assertAlmostEqual(by_group["2"]["mae"], 10.0)
        self.assertEqual(by_group["2"]["teleportedVehicles"], 1)
        self.assertAlmostEqual(by_group["ALL"]["mae"], 14.0 / 3.0)
        self.assertAlmostEqual(by_group["ALL"]["meanTruthDuration"], 28.0 / 3.0)

    def test_round_cap_combines_tail_groups(self):
        rows = [
            {
                "durationError": 1.0,
                "truthDuration": 10.0,
                "predDuration": 11.0,
                "truthWaitingTime": None,
                "successfulGreenRounds": None,
                "totalGreenRounds": None,
                "vehicleTeleported": False,
            }
        ]
        self.assertEqual(MODULE.group_label(4, 5), "4")
        self.assertEqual(MODULE.group_label(5, 5), "5+")
        summary = MODULE.summarize("5+", rows, 1)
        self.assertTrue(math.isnan(summary["meanTruthWaitingTime"]))


if __name__ == "__main__":
    unittest.main()
