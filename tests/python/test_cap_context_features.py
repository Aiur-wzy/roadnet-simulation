import math
import json
import os
from pathlib import Path
import subprocess
import unittest

from features.cap_context_features import (
    CONTEXT_CONTRACT, FEATURE_COLS, FEATURE_SCHEMA, ContextConfig, ShadowLedger,
    ShadowRecord, classify_context, free_speed_cap, make_context_contract,
    travel_distance, validate_context_contract,
)


class CapContextFeaturesTest(unittest.TestCase):
    def record(self, vid="ego", index=0, road="a", lane=0, enter=10.0, distance=100.0, cap=10.0):
        return ShadowRecord(vid, index, road, lane, enter, distance, cap)

    def test_schema(self):
        self.assertEqual(FEATURE_SCHEMA, "road-time-cap-context-v1")
        self.assertEqual(FEATURE_COLS, ["road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context"])
        self.assertNotIn("speed_factor", FEATURE_COLS)
        self.assertNotIn("speed_limit", FEATURE_COLS)
        self.assertNotIn("P", FEATURE_COLS)
        self.assertEqual(make_context_contract(), CONTEXT_CONTRACT)

    def test_contract_validates_exact_keys_and_supported_version(self):
        for replacement in ({"other": 1}, {**CONTEXT_CONTRACT, "version": "bad"}, {**CONTEXT_CONTRACT, "p_threshold": 0},
                            {**CONTEXT_CONTRACT, "e_threshold": .9}, {**CONTEXT_CONTRACT, "epsilon_seconds": -1},
                            {**CONTEXT_CONTRACT, "crossing_time_seconds": math.inf},
                            {**CONTEXT_CONTRACT, "crossing_time_seconds": "3"},
                            {**CONTEXT_CONTRACT, "p_threshold": .1},
                            {**CONTEXT_CONTRACT, "e_threshold": 3},
                            {**CONTEXT_CONTRACT, "epsilon_seconds": 1e-8}):
            with self.subTest(replacement=replacement), self.assertRaises(ValueError):
                validate_context_contract(replacement)
        self.assertEqual(make_context_contract(crossing_time_seconds=0)["crossing_time_seconds"], 0)

    def test_cap_all_three_limits(self):
        self.assertAlmostEqual(free_speed_cap(.8, 13.89, 2777, 13.89), 11.112)
        self.assertEqual(free_speed_cap(1.2, 13.89, 2777, 13.89), 13.89)
        self.assertEqual(free_speed_cap(2, 30, 5, 20), 10)
        self.assertEqual(free_speed_cap(2, 30, 20, 5), 10)
        self.assertEqual(free_speed_cap(2, 5, 20, 20), 5)

    def test_invalid_cap_sources_fail(self):
        for column in range(4):
            for invalid in (None, math.nan, math.inf, 0, -1, True):
                values = [1.0, 10.0, 20.0, 10.0]
                values[column] = invalid
                with self.subTest(column=column, invalid=invalid), self.assertRaises(ValueError):
                    free_speed_cap(*values)

    def test_first_distance_and_later_visit(self):
        self.assertAlmostEqual(travel_distance(545.6, route_index=0, vehicle_length=5), 540.5)
        self.assertAlmostEqual(travel_distance(100, route_index=0, vehicle_length=8), 91.9)
        self.assertEqual(travel_distance(100, route_index=2, vehicle_length=5), 100)
        for args in ({"route_index": 0, "vehicle_length": 100},
                     {"route_index": 0, "vehicle_length": 5, "depart_pos": "random"},
                     {"route_index": -1, "vehicle_length": 5}):
            with self.subTest(args=args), self.assertRaises(ValueError):
                travel_distance(100, **args)

    def test_classifier_half_open_edges_and_noninteger_equivalent_count(self):
        cases = [(0, 0, 0), (1e-9, 1e-9, 0), (.49, .98, 1), (.5, .5, 2), (.5, .999, 2), (.5, 1.0, 3)]
        for maximum, total, expected in cases:
            with self.subTest(maximum=maximum, total=total):
                self.assertEqual(classify_context(maximum, total, 10)["platoon_context"], expected)
        self.assertEqual(classify_context(6, 12, 10)["E"], 2)
        self.assertEqual(classify_context(20, 20, 10)["P"], 2)  # not clipped at one

    def test_empty_and_self_exclusion(self):
        ledger, ego = ShadowLedger(), self.record()
        self.assertEqual(ledger.query(ego, 10)["platoon_context"], 0)
        ledger.reserve(ego)
        result = ledger.query(ego, 10)
        self.assertEqual(result["candidate_count"], 0)
        self.assertEqual(result["platoon_context"], 0)

    def test_near_slow_far_slow_and_faster(self):
        ego = self.record()
        for enter, speed, expected in ((9, 5, 2), (-15, 5, 0), (9, 11, 0), (9, 10, 0)):
            ledger = ShadowLedger()
            ledger.reserve(self.record("leader", enter=enter, cap=speed))
            with self.subTest(enter=enter, speed=speed):
                self.assertEqual(ledger.query(ego, 10)["platoon_context"], expected)

    def test_slow_group_class_three(self):
        ledger = ShadowLedger()
        for vid in ("one", "two"):
            ledger.reserve(self.record(vid, enter=9, cap=5))
        result = ledger.query(self.record(), 10)
        self.assertEqual(result["platoon_context"], 3)
        self.assertEqual(result["E"], 2)
        self.assertLessEqual(result["E"], result["candidate_count"])

    def test_pending_boundary_is_strict_not_epsilon_future(self):
        ego = self.record(enter=13)
        ledger = ShadowLedger()
        ledger.reserve(self.record("pending", enter=10 + 5e-10, cap=1))
        before = ledger.query(ego, 10)
        self.assertEqual(before["candidate_count"], 0)
        self.assertEqual(before["pending_count"], 1)
        later = ledger.query(ego, 10 + 5e-10)
        self.assertEqual(later["candidate_count"], 1)
        # Querying a later time did not mutate an activation clock.
        self.assertEqual(ledger.query(ego, 10)["candidate_count"], 0)

    def test_different_lane_and_road_are_not_scanned(self):
        ledger = ShadowLedger()
        ledger.reserve(self.record("other_lane", lane=1, cap=1))
        ledger.reserve(self.record("other_road", road="b", cap=1))
        self.assertEqual(ledger.query(self.record(), 10)["scanned_count"], 0)

    def test_release_and_route_occurrence(self):
        ledger = ShadowLedger()
        ledger.reserve(self.record("leader", index=0, cap=1))
        with self.assertRaises(ValueError):
            ledger.reserve(self.record("leader", index=2))
        with self.assertRaises(ValueError):
            ledger.release("leader", 2)
        self.assertTrue(ledger.release("leader", 0))
        ledger.reserve(self.record("leader", index=2, cap=1))
        self.assertEqual(len(ledger), 1)
        self.assertTrue(ledger.release("leader", 2))
        self.assertFalse(ledger.release("leader"))
        self.assertEqual(ledger.query(self.record(), 10)["platoon_context"], 0)

    def test_no_quantization_before_context(self):
        ledger = ShadowLedger()
        ledger.reserve(self.record("leader", enter=10, cap=9.51))
        result = ledger.query(self.record(), 10)
        self.assertAlmostEqual(result["P"], (100 / 9.51 - 10) / 10)
        self.assertEqual(result["platoon_context"], 2)

    def test_shared_cpp_event_fixture(self):
        from roadnet_paths import ROOT
        fixture_path = ROOT / "tests/fixtures/cap_context_event_fixture.json"
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        self.assertEqual(fixture["schema"], "cap-context-shared-events-v1")
        ledger = ShadowLedger()
        query_count = 0
        python_rows = []
        for event in fixture["events"]:
            if event["op"] == "release":
                self.assertTrue(ledger.release(str(event["vehicle_id"]), event["route_index"]))
                continue
            record = ShadowRecord(str(event["vehicle_id"]), event["route_index"], "0", 0,
                                  event["planned_enter_time"], event["distance"], event["free_speed_cap"])
            if event["op"] == "reserve":
                ledger.reserve(record)
            else:
                self.assertEqual(event["op"], "query")
                result = ledger.query(record, event["decision_time"])
                query_count += 1
                python_rows.append({"name": event["name"], "context": result["platoon_context"],
                                    "P": result["P"], "E": result["E"], "eligible_count": result["candidate_count"]})
                with self.subTest(name=event["name"]):
                    self.assertEqual(result["platoon_context"], event["expected_context"])
                    self.assertAlmostEqual(result["P"], event["expected_P"], places=14)
                    self.assertAlmostEqual(result["E"], event["expected_E"], places=14)
                    self.assertEqual(result["candidate_count"], event["expected_eligible"])
        self.assertEqual(query_count, 7)
        build = Path(os.environ.get("ROADNET_TEST_BUILD_DIR", ROOT / "build"))
        binary = build / ("CapContextTest.exe" if os.name == "nt" else "CapContextTest")
        if not binary.is_file():
            binary = ROOT / "test_cap_context_f13.exe"
        if binary.is_file():
            output = subprocess.check_output([str(binary), "--event-fixture", str(fixture_path)], text=True)
            cpp_rows = [json.loads(line) for line in output.splitlines() if line.strip()]
            # Exact Python float equality after round-trip JSON serialization,
            # not a loose tolerance hiding a context-boundary discrepancy.
            self.assertEqual(cpp_rows, python_rows)


if __name__ == "__main__":
    unittest.main()
