"""Regression and adversarial tests for local E1 quarantine, no SUMO required."""
import copy
from pathlib import Path
import shutil
import unittest
import uuid

from features import discharge_sequences as legacy
from features.discharge_e1_quality import (
    E1VisitTracker, MemoryStore, SQLiteStore, SequenceExtractor, crossing_quality,
)
from training.train_discharge_sequences import validate_sequence


def event(kind, time, lane="L", **kw):
    return dict(schema=legacy.EVENT_SCHEMA, run_id="r", event=kind,
                lane_id=lane, section_id="end", time=time, **kw)


def start(time=0, keys=("a", "b", "c"), lane="L"):
    return event("start", time, lane, valid_start=True, start_kind="stopped_green_start",
                 cohort=[dict(vehicle_key=k, vehicle_id=k) for k in keys])


def passage(key, time, lane="L", **kw):
    return event("passage", time, lane, vehicle_key=key, vehicle_id=kw.pop("vehicle_id", key),
                 type_id="car", turn="s", **kw)


class QuarantineTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / "_quality_test_artifacts" / uuid.uuid4().hex
        self.root.mkdir(parents=True)
        self.stores = []

    def tearDown(self):
        for store in self.stores:
            store.close()
        if self.root.parent.name != "_quality_test_artifacts":
            raise RuntimeError("unsafe cleanup")
        shutil.rmtree(self.root)

    def extractors(self):
        for kind in ("memory", "sqlite"):
            store = MemoryStore() if kind == "memory" else SQLiteStore(self.root / (uuid.uuid4().hex + ".sqlite"))
            self.stores.append(store)
            yield SequenceExtractor("r", store=store)

    def feed(self, extractor, *events):
        for row in events:
            extractor.feed(copy.deepcopy(row))

    def rows(self, extractor):
        extractor.finish()
        rows = list(extractor.store.iter_sequences())
        for row in rows:
            validate_sequence(row, "r")
        return rows

    def test_normal_times_and_labels_are_unchanged(self):
        inputs = [start(), passage("a", 1), passage("b", 3), passage("c", 5)]
        old = legacy.SequenceExtractor("r")
        self.feed(old, *inputs)
        expected = self.rows(old)
        for new in self.extractors():
            self.feed(new, *inputs)
            self.assertEqual(self.rows(new), expected)

    def test_conflict_middle_retracts_suffix_not_join_across_bad_car(self):
        for new in self.extractors():
            self.feed(new, start(), passage("a", 1), passage("b", 3),
                      passage("b", 4), passage("c", 5))
            rows = self.rows(new)
            self.assertEqual([p["vehicle_key"] for p in rows[0]["passages"]], ["a"])
            self.assertEqual(rows[0]["startup_delay"], 1)
            self.assertEqual(rows[0]["headways"], [])
            self.assertEqual(new.counts["quarantined_passage_time_conflicts"], 1)
            bad = next(p for p in new.store.iter_passages() if p["vehicle_key"] == "b")
            self.assertTrue(bad["passage_quarantined"])
            self.assertEqual(bad["conflicting_entry_times"], [3, 4])

    def test_late_conflict_retracts_already_closed_sequence(self):
        for new in self.extractors():
            self.feed(new, start(), passage("a", 1), passage("b", 3), passage("c", 5),
                      passage("a", 6))
            row = self.rows(new)[0]
            self.assertEqual(row["passages"], [])
            self.assertIsNone(row["startup_delay"])
            self.assertEqual(new.counts["revoked_passage_labels"], 3)

    def test_stable_raw_clock_with_later_read_time_or_tail_exit_is_one_event(self):
        for new in self.extractors():
            self.feed(new, start(), passage("a", 1, detector_entry_time=1, detector_exit_time=-1),
                      passage("a", 2, detector_entry_time=1, detector_exit_time=2),
                      passage("b", 3), passage("c", 5))
            row = self.rows(new)[0]
            self.assertEqual([p["time"] for p in row["passages"]], [1, 3, 5])
            self.assertEqual(new.counts["duplicate_passage_observations"], 1)

    def test_conflicting_raw_clocks_even_when_event_time_clamped_equal(self):
        for new in self.extractors():
            self.feed(new, start(), passage("a", 2, detector_entry_time=1),
                      passage("a", 2, detector_entry_time=2))
            self.assertEqual(self.rows(new)[0]["passages"], [])
            self.assertEqual(new.counts["quarantined_passage_time_conflicts"], 1)

    def test_no_active_training_sequence_still_audits_conflict_without_run_failure(self):
        for new in self.extractors():
            self.feed(new, passage("a", 1, quality="unknown"), passage("a", 2))
            self.assertEqual(self.rows(new), [])
            self.assertEqual(len(list(new.store.iter_passages())), 1)
            self.assertEqual(new.counts["raw_passages"], 1)

    def test_repeated_conflict_does_not_repeatedly_revoke_new_independent_segment(self):
        for new in self.extractors():
            self.feed(new, passage("a", 1), passage("a", 2),
                      start(3, ("d", "e")), passage("d", 4), passage("a", 5), passage("e", 6))
            row = self.rows(new)[0]
            self.assertEqual([p["vehicle_key"] for p in row["passages"]], ["d", "e"])
            self.assertEqual(row["headways"][0]["headway"], 2)
            self.assertEqual(new.counts["quarantined_passage_time_conflicts"], 1)

    def test_true_loop_uses_distinct_visit_keys(self):
        for new in self.extractors():
            self.feed(new, start(0, ("v|visit=1|route=0",)),
                      passage("v|visit=1|route=0", 1, vehicle_id="v"),
                      start(10, ("v|visit=4|route=3",)),
                      passage("v|visit=4|route=3", 11, vehicle_id="v"))
            self.assertEqual(len(self.rows(new)), 2)
            self.assertEqual(len(list(new.store.iter_passages())), 2)
            self.assertEqual(new.counts["quarantined_passage_time_conflicts"], 0)

    def test_unknown_record_breaks_continuity_but_other_lane_unaffected(self):
        for new in self.extractors():
            self.feed(new, start(), start(lane="X"), passage("a", 1), passage("a", 1, lane="X"),
                      passage("b", 3, quality="unknown"), passage("b", 3, lane="X"),
                      passage("c", 5), passage("c", 5, lane="X"))
            rows = {row["lane_id"]: row for row in self.rows(new)}
            self.assertEqual(len(rows["L"]["passages"]), 1)
            self.assertEqual(len(rows["X"]["headways"]), 2)

    def test_malformed_identity_and_nonfinite_clock_are_local_rejections(self):
        for new in self.extractors():
            self.feed(new, passage("a", 1, vehicle_id="a"))
            new.feed(passage("a", 2, vehicle_id="b"))
            self.assertEqual(new.counts["quarantined_passage_identity_conflicts"], 1)
            self.feed(new, start(3, ("d", "e")), passage("d", 4), passage("e", 6))
            self.assertEqual(self.rows(new)[0]["headways"][0]["headway"], 2)
        for new in self.extractors():
            new.feed(passage("a", 1, detector_entry_time=float("nan")))
            self.assertTrue(next(new.store.iter_passages())["passage_quarantined"])

    def test_legacy_store_and_extractor_remain_strict(self):
        old = legacy.SequenceExtractor("r")
        old.feed(passage("a", 1))
        with self.assertRaisesRegex(ValueError, "conflicting passage"):
            old.feed(passage("a", 2))

    def test_raw_sink_keeps_both_reports_for_evidence(self):
        raw = []
        new = SequenceExtractor("r", raw_sink=raw.append)
        self.feed(new, passage("a", 1), passage("a", 2))
        self.assertEqual([r["time"] for r in raw], [1, 2])


class VisitTests(unittest.TestCase):
    def test_changed_entry_is_not_new_visit_after_source_snapshot_disappears(self):
        tracker = E1VisitTracker()
        self.assertEqual(tracker.observe("d", "a", 1, "a:1", None, "initial"), ("a:1", "initial"))
        tracker.end_step()
        self.assertEqual(tracker.observe("d", "a", 2, "unobserved", None, "unknown"),
                         ("a:1", "e1_entry_time_conflict"))

    def test_new_upstream_visit_can_cross_same_detector(self):
        tracker = E1VisitTracker()
        tracker.observe("d", "a", 1, "a:1", "a:1", "e1_front_entry")
        tracker.end_step()
        self.assertEqual(tracker.observe("d", "a", 10, "a:3", "a:3", "e1_front_entry"),
                         ("a:3", "e1_front_entry"))

    def test_stable_old_occupancy_not_renamed_for_future_visit(self):
        tracker = E1VisitTracker()
        tracker.observe("d", "a", 1, "a:1", "a:1", "e1_front_entry")
        tracker.end_step()
        self.assertEqual(tracker.observe("d", "a", 1, "a:3", "a:3", "e1_front_entry")[0], "a:1")

    def test_tracker_prunes_absent_occupancies(self):
        tracker = E1VisitTracker()
        tracker.observe("d", "a", 1, "a:1", None, "unknown")
        tracker.end_step()
        tracker.end_step()
        self.assertEqual(tracker.active, {})

    def test_upstream_sample_required_not_merely_current_position(self):
        observed = dict(lane_id="L", vehicle_key="a", route_index=0, position=4.05)
        self.assertEqual(crossing_quality(None, observed, "L", 4.05, "a")[0],
                         "e1_initial_entry_without_upstream_sample")
        self.assertEqual(crossing_quality(observed, observed, "L", 4.05, "a")[0],
                         "e1_boundary_without_upstream_sample")
        old = dict(observed, position=1.0)
        self.assertEqual(crossing_quality(old, observed, "L", 4.05, "a"), ("e1_front_entry", "a"))
        old["lane_id"] = "L1"
        self.assertEqual(crossing_quality(old, observed, "L", 4.05, "a")[0],
                         "e1_unobserved_longitudinal_entry")


if __name__ == "__main__":
    unittest.main()
