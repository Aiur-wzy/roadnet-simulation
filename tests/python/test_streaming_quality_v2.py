"""Deterministic native-quality unit checks; no SUMO or external processes."""
import io
import json
import unittest
import xml.etree.ElementTree as ET

from collection.native_boundary_v2.streaming_quality_v2 import StreamingQualityV2
from collection.native_boundary_v2.sumo_native_path_v2 import NoInternalGeometry
from collection.native_boundary_v2.sumo_native_boundary_v2 import LABELS


def graph(length=10.0):
    root = ET.Element("net")
    for road in "abc":
        edge = ET.SubElement(root, "edge", id=road)
        ET.SubElement(edge, "lane", id=road + "_0", index="0", length=str(length), speed="10")
    for left, right in (("a", "b"), ("b", "c")):
        ET.SubElement(root, "connection", **{"from": left, "to": right, "fromLane": "0", "toLane": "0"})
    return NoInternalGeometry(root)


def observation(time, index=0, position=0.0, speed=10.0, distance=0.0, vehicle="v"):
    return dict(id=vehicle, edge="abc"[index], lane="abc"[index] + "_0", route_index=index,
                position=float(position), speed=float(speed), distance=float(distance), time=float(time))


def feature(index=0, anchor=0.0, vehicle="v", **updates):
    value = dict(vehicle_id=vehicle, route_index=index, road_id="abc"[index], feature_valid=1,
                 observed_enter_time=anchor, road_length=10.0, road_flow=1, lane_flow=1,
                 free_speed_cap=10.0, platoon_context=0, entry_signal_type="G",
                 entry_signal_remaining_seconds=30.0, next_signal_type="R", next_signal_duration_seconds=60.0,
                 entry_speed=10.0, entry_position=0.0, label_valid=0, trainable=0)
    value.update(updates)
    return value


def trips(**updates):
    value = dict(id="v", depart="0", arrival="1", duration="1", arrivalSpeed="10",
                 arrivalLane="a_0", arrivalPos="10", routeLength="10", vaporized="")
    value.update({key: str(item) for key, item in updates.items()})
    root = ET.Element("tripinfos")
    ET.SubElement(root, "tripinfo", value)
    return io.BytesIO(ET.tostring(root))


class StreamingQualityTests(unittest.TestCase):
    def make(self, route=("a",), length=10.0):
        self.emitted = []
        engine = StreamingQualityV2(graph(length), 1.0, ":memory:", self.emitted.append)
        self.addCleanup(engine.close)
        engine.depart("v", route, dict(actual_depart=0.0, planned_depart=0.0, speed_factor=1.0))
        return engine

    def test_single_snapshot_subsecond_label_is_trainable(self):
        engine = self.make(length=1.0)
        engine.step(0.0, [observation(0)])
        engine.step(1.0, [], arrived=("v",), feature_rows=[feature(road_length=1.0)])
        self.assertFalse(engine.active)
        self.assertEqual(self.emitted, [])
        self.assertEqual(engine.db.execute("SELECT COUNT(*) FROM vehicles WHERE status='pending'").fetchone()[0], 1)
        engine.finalize_tripinfo(trips(arrivalPos=1, routeLength=1))
        row = self.emitted[0]
        self.assertEqual(row["trainable"], 1)
        self.assertAlmostEqual(row[LABELS[0]], 0.1)
        self.assertEqual(row["feature_snapshot_lag_seconds"], 0.0)
        self.assertAlmostEqual(engine.finish()["seconds"]["terminal_rounding_seconds"], 0.9)

    def test_valid_all_zero_label_is_preserved(self):
        engine = self.make()
        engine.step(0.0, [observation(0, position=10, speed=0)])
        engine.step(1.0, [observation(1, position=10, speed=0)])
        engine.step(2.0, [], arrived=("v",), feature_rows=[feature()])
        engine.finalize_tripinfo(trips(arrival=2, duration=2, routeLength=0))
        self.assertEqual(self.emitted[0]["trainable"], 1)
        self.assertTrue(all(self.emitted[0][name] == 0 for name in LABELS))
        self.assertEqual(engine.finish()["counts"]["trainable"], 1)

    def test_no_feature_does_not_become_zero_filled_feature(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        engine.step(1, [], arrived=("v",))
        engine.finalize_tripinfo(trips())
        row = self.emitted[0]
        self.assertEqual(row["label_valid"], 1)
        self.assertEqual(row["trainable"], 0)
        self.assertNotIn("road_flow", row)
        self.assertIn("missing_or_unknown_features", row["quality_reasons"])
        self.assertEqual(engine.finish()["counts"]["excluded"], 1)

    def test_unknown_crossing_poisons_both_ends_not_clean_later_visit(self):
        engine = self.make(("a", "b", "c"))
        engine.step(0, [observation(0, position=8)])
        engine.step(1, [observation(1, 1, position=1, speed=4, distance=3)], feature_rows=[feature()])
        self.assertEqual(self.emitted[0]["label_valid"], 0)
        self.assertEqual(self.emitted[0][LABELS[0]], "")
        engine.step(2, [observation(2, 2, position=1, speed=10, distance=13)], feature_rows=[feature(1, 1)])
        self.assertEqual(self.emitted[1]["label_valid"], 0)
        engine.step(3, [], arrived=("v",), feature_rows=[feature(2, 2)])
        engine.finalize_tripinfo(trips(arrival=3, duration=3, arrivalLane="c_0", routeLength=22))
        self.assertEqual([row["trainable"] for row in self.emitted], [0, 0, 1])
        self.assertAlmostEqual(self.emitted[2][LABELS[0]], 1.0)
        self.assertAlmostEqual(engine.finish()["seconds"]["unresolved_seconds"], 1.0)

    def test_skipped_middle_road_has_label_but_no_invented_feature(self):
        engine = self.make(("a", "b", "c"), length=1)
        engine.step(0, [observation(0)])
        engine.step(1, [observation(1, 2, position=0.5, speed=2.5, distance=2.5)],
                    feature_rows=[feature(), feature(1, feature_valid=0)])
        self.assertEqual(len(self.emitted), 2)
        self.assertAlmostEqual(self.emitted[1][LABELS[0]], 0.4)
        self.assertEqual(self.emitted[1]["trainable"], 0)
        self.assertEqual(set(engine.active["v"].rows), {2})
        engine.step(2, [], arrived=("v",), feature_rows=[feature(2, 1)])
        engine.finalize_tripinfo(trips(arrival=2, duration=2, arrivalSpeed=2.5, arrivalLane="c_0", arrivalPos=1, routeLength=3))
        self.assertEqual(engine.finish()["counts"]["road_occurrences"], 3)

    def test_missing_subscription_invalidates_run_before_feature_acceptance(self):
        engine = self.make()
        with self.assertRaisesRegex(ValueError, "incomplete native frame"):
            engine.step(0, [])
        with self.assertRaisesRegex(ValueError, "closed or failed"):
            engine.finish()
        self.assertEqual(self.emitted, [])

    def test_physical_id_mismatch_is_fatal(self):
        engine = self.make()
        with self.assertRaisesRegex(ValueError, "incomplete native frame"):
            engine.step(0, [observation(0)], expected_live_ids={"v", "missing"})

    def test_missing_whole_frame_clock_is_fatal(self):
        engine = self.make()
        engine.step(0, [observation(0, speed=0)])
        with self.assertRaisesRegex(ValueError, "nonconsecutive global snapshot"):
            engine.step(2, [observation(2, speed=0)])

    def test_duplicate_departure_identity_is_fatal(self):
        engine = self.make()
        with self.assertRaisesRegex(ValueError, "duplicate departure"):
            engine.depart("v", ("a",), dict(actual_depart=0))

    def test_feature_lag_outside_step_excludes_without_rewriting_label(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        engine.step(1, [], arrived=("v",), feature_rows=[feature(anchor=2)])
        engine.finalize_tripinfo(trips())
        self.assertEqual(self.emitted[0][LABELS[0]], 1.0)
        self.assertEqual(self.emitted[0]["trainable"], 0)
        self.assertIn("feature_snapshot_lag_outside_step", self.emitted[0]["quality_reasons"])
        engine.finish()

    def test_missing_formal_feature_excludes(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        value = feature()
        del value["free_speed_cap"]
        engine.step(1, [], arrived=("v",), feature_rows=[value])
        engine.finalize_tripinfo(trips())
        self.assertEqual(self.emitted[0]["trainable"], 0)
        self.assertIn("incomplete_required_features", self.emitted[0]["quality_reasons"])

    def test_duplicate_feature_occurrence_is_fatal(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        with self.assertRaisesRegex(ValueError, "feature occurrence"):
            engine.step(1, [], arrived=("v",), feature_rows=[feature(), feature()])

    def test_invalid_numeric_feature_domains_are_excluded(self):
        for update in ({"road_flow": -1}, {"lane_flow": 1.5}, {"lane_flow": 2},
                       {"free_speed_cap": -2}, {"road_length": 0}, {"platoon_context": 99},
                       {"entry_signal_remaining_seconds": -1}):
            with self.subTest(update=update):
                engine = self.make()
                engine.step(0, [observation(0)])
                engine.step(1, [], arrived=("v",), feature_rows=[feature(**update)])
                engine.finalize_tripinfo(trips())
                self.assertEqual(self.emitted[0]["trainable"], 0)
                self.assertEqual(self.emitted[0]["label_valid"], 1)
                engine.finish()

    def test_zero_signal_placeholder_is_final_only(self):
        engine = self.make(("a", "b"))
        engine.step(0, [observation(0)])
        engine.step(1, [observation(1, 1, distance=10)], feature_rows=[
            feature(entry_signal_remaining_seconds=0, next_signal_type="G", next_signal_duration_seconds=0)])
        self.assertEqual(self.emitted[0]["trainable"], 0)
        self.assertIn("invalid_signal_feature_duration", self.emitted[0]["quality_reasons"])
        engine.step(2, [], arrived=("v",), feature_rows=[
            feature(1, 1, entry_signal_remaining_seconds=0, next_signal_type="G", next_signal_duration_seconds=0)])
        engine.finalize_tripinfo(trips(arrival=2, duration=2, arrivalLane="b_0", routeLength=20))
        self.assertEqual(self.emitted[1]["trainable"], 1)
        engine.finish()

    def test_compensated_totals_do_not_reject_36000_closed_budgets(self):
        engine = self.make()
        for _ in range(36000):
            engine._add_seconds("allocated_seconds", 5999.123456789)
            engine._add_seconds("terminal_rounding_seconds", 0.876543211)
            engine._add_seconds("tripinfo_total_duration", 6000.0)
        residual = (engine.seconds["tripinfo_total_duration"] - engine.seconds["allocated_seconds"]
                    - engine.seconds["terminal_rounding_seconds"])
        self.assertLess(abs(residual), 1e-6)

    def test_abnormal_tripinfo_cannot_publish(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        engine.step(1, [], arrived=("v",), feature_rows=[feature()])
        with self.assertRaisesRegex(ValueError, "abnormal"):
            engine.finalize_tripinfo(trips(vaporized="teleport"))
        with self.assertRaises(ValueError):
            engine.finish()

    def test_empty_or_incomplete_terminal_ledger_fails(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        engine.step(1, [], arrived=("v",), feature_rows=[feature()])
        engine.finalize_tripinfo(io.BytesIO(b"<tripinfos/>"))
        with self.assertRaisesRegex(ValueError, "incomplete native terminal ledger"):
            engine.finish()

    def test_terminal_native_acceptance_band_is_preserved(self):
        engine = self.make(length=1)
        engine.step(0, [observation(0, position=0.85, speed=0.1)])
        engine.step(1, [], arrived=("v",), feature_rows=[feature(road_length=1)])
        engine.finalize_tripinfo(trips(arrivalSpeed=0.1, arrivalPos=1, routeLength=0.15))
        row = self.emitted[0]
        self.assertEqual(row["label_valid"], 1)
        self.assertAlmostEqual(row["native_arrival_untravelled_target_m"], 0.05)
        self.assertEqual(row[LABELS[0]], 0.0)
        self.assertEqual(engine.finish()["counts"]["native_arrival_acceptance_band"], 1)

    def test_terminal_outside_acceptance_band_is_unknown_not_compensated(self):
        engine = self.make(length=1)
        engine.step(0, [observation(0, position=0.79, speed=0.1)])
        engine.step(1, [], arrived=("v",), feature_rows=[feature(road_length=1)])
        engine.finalize_tripinfo(trips(arrivalSpeed=0.1, arrivalPos=1, routeLength=0.21))
        row = self.emitted[0]
        self.assertEqual(row["label_valid"], 0)
        self.assertEqual(row[LABELS[0]], "")
        self.assertIn("terminal_distance_speed_not_closed", row["quality_reasons"])
        engine.finish()

    def test_finished_report_is_json_serializable_and_has_bounded_bins(self):
        engine = self.make()
        engine.step(0, [observation(0)])
        engine.step(1, [], arrived=("v",), feature_rows=[feature()])
        engine.finalize_tripinfo(trips())
        report = engine.finish()
        json.dumps(report, allow_nan=False)
        self.assertEqual(report["counts"]["road_occurrences"], 1)
        self.assertEqual(report["counts"]["trainable"], 1)
        self.assertEqual(report["counts"]["excluded"], 0)
        self.assertEqual(report["coverage_by_length"]["10_to_25"]["total"], 1)


if __name__ == "__main__":
    unittest.main()
