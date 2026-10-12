"""Single-vehicle data errors never stop a healthy collector; I/O faults do."""
import copy
import json
import sqlite3
import unittest
from unittest.mock import patch

import test_discharge_e1_quality as q
import test_collect_discharge_batch as b
import test_discharge_e1_adapter as a
from features.discharge_e1_quality import SequenceExtractor, usable_passage
from training import train_discharge_sequences as training


class RecordTests(unittest.TestCase):
    setUp = q.QuarantineTests.setUp
    tearDown = q.QuarantineTests.tearDown
    extractors = q.QuarantineTests.extractors
    feed = q.QuarantineTests.feed
    rows = q.QuarantineTests.rows

    def test_bad_vehicle_fields_rejected_and_later_segment_kept(self):
        for update in ({"detector_entry_time": float("nan")}, {"vehicle_id": ""},
                       {"vehicle_key": ""}, {"vehicle_key": []}, {"type_id": None}, {"turn": []},
                       {"quality": []}, {"permission_class": None},
                       {"speed_factor": float("inf")}, {"time": float("nan")}):
            with self.subTest(update=update):
                for engine in self.extractors():
                    raw, rejected = [], []
                    engine.raw_sink, engine.rejection_sink = raw.append, rejected.append
                    bad = q.passage("b", 3)
                    bad.update(update)
                    self.feed(engine, q.start(), q.passage("a", 1), bad, q.passage("c", 5),
                              q.start(10, ("d", "e")), q.passage("d", 11), q.passage("e", 13))
                    rows = self.rows(engine)
                    self.assertEqual(rows[-1]["headways"][0]["headway"], 2)
                    if update.get("time") is None:
                        self.assertEqual([p["vehicle_key"] for p in rows[0]["passages"]], ["a"])
                    self.assertFalse(any(h["prev_vehicle_key"] == "a" and h["vehicle_key"] == "c"
                                         for row in rows for h in row["headways"]))
                    self.assertEqual(len(rejected), 1)
                    json.dumps(raw, allow_nan=False)
                    json.dumps(rejected, allow_nan=False)
                    self.assertFalse(any(p["vehicle_key"] == "b" and usable_passage(p)
                                         for p in engine.store.iter_passages()))

    def test_bad_motion_does_not_stop_later_collection(self):
        for update in ({"position": float("nan")}, {"speed": float("nan")},
                       {"cut_time": 99}, {"speed": -1}, {"vehicle_key": []},
                       {"present": False, "speed": float("inf")}):
            for engine in self.extractors():
                raw = []
                engine.raw_sink = raw.append
                motion = q.event("motion", 2, vehicle_key="b", position=4, speed=1)
                motion.update(update)
                self.feed(engine, q.start(), q.passage("a", 1), motion,
                          q.start(10, ("d", "e")), q.passage("d", 11), q.passage("e", 13))
                self.assertEqual(self.rows(engine)[-1]["headways"][0]["headway"], 2)
                json.dumps(raw, allow_nan=False)

    def test_bad_unrelated_motion_does_not_cut_valid_cohort(self):
        for engine in self.extractors():
            self.feed(engine, q.start(), q.passage("a", 1),
                      q.event("motion", 2, vehicle_key="outsider", speed=1, position=float("nan")),
                      q.passage("b", 3), q.passage("c", 5))
            self.assertEqual(len(self.rows(engine)[0]["headways"]), 2)

    def test_quarantined_late_repetition_does_not_cut_new_segment(self):
        for engine in self.extractors():
            self.feed(engine, q.passage("a", 1), q.passage("a", 2),
                      q.start(3, ("d", "e")), q.passage("d", 4), q.passage("a", 1), q.passage("e", 6))
            self.assertEqual(self.rows(engine)[0]["headways"][0]["headway"], 2)

    def test_late_malformed_repetition_revokes_earlier_labels(self):
        for engine in self.extractors():
            self.feed(engine, q.start(), q.passage("a", 1), q.passage("b", 3), q.passage("c", 5),
                      q.passage("b", 6, detector_entry_time=float("nan")))
            row = self.rows(engine)[0]
            self.assertEqual([p["vehicle_key"] for p in row["passages"]], ["a"])
            self.assertEqual(row["headways"], [])

    def test_future_bad_raw_clock_does_not_advance_lane_clock(self):
        for engine in self.extractors():
            self.feed(engine, q.passage("a", 1, detector_entry_time=999),
                      q.start(3, ("d", "e")), q.passage("d", 4), q.passage("e", 6))
            self.assertEqual(self.rows(engine)[-1]["headways"][0]["headway"], 2)

    def test_other_lane_unaffected_by_identity_conflict(self):
        for engine in self.extractors():
            self.feed(engine, q.start(), q.start(lane="X"), q.passage("a", 1),
                      q.passage("a", 1, lane="X"), q.passage("a", 2, vehicle_id="wrong"),
                      q.passage("b", 3, lane="X"), q.passage("c", 5, lane="X"))
            rows = {row["lane_id"]: row for row in self.rows(engine)}
            self.assertEqual(len(rows["X"]["headways"]), 2)

    def test_contract_and_storage_failures_are_not_swallowed(self):
        engine = SequenceExtractor("r")
        for update in ({"run_id": "other"}, {"schema": "wrong"}, {"lane_id": None}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                engine.feed(dict(q.passage("a", 1), **update))
        with patch.object(engine.store, "lookup_passage", side_effect=sqlite3.OperationalError("disk")):
            with self.assertRaises(sqlite3.OperationalError):
                engine.feed(q.passage("a", 1))

    def test_rejection_log_write_failure_still_stops(self):
        def failed_sink(_):
            raise OSError("disk full")
        engine = SequenceExtractor("r", rejection_sink=failed_sink)
        with self.assertRaises(OSError):
            engine.feed(q.passage("a", 1, detector_entry_time=float("nan")))


class ExportTests(b.BatchFixture):
    def assert_training_ingest(self, expected_headways):
        with sqlite3.connect(":memory:") as database:
            training._create_database(database)
            training.ingest_datasets(database, [self.output], prefix_length=8)
            self.assertEqual(database.execute("SELECT value FROM samples WHERE target='headway'").fetchall(),
                             [(value,) for value in expected_headways])
            self.assertNotIn(("b",), database.execute("SELECT vehicle_key FROM passages").fetchall())

    def test_record_error_does_not_prevent_natural_completion_or_export(self):
        events = [q.start(), q.passage("a", 1), q.passage("b", 3),
                  q.passage("b", 4, detector_entry_time=float("nan")),
                  q.start(10, ("a|second", "b|second")),
                  q.passage("a|second", 11, vehicle_id="a"),
                  q.passage("b|second", 13, vehicle_id="b")]
        events = [dict(row, run_id=self.job["run_id"], permission_class="protected") for row in events]
        result = b.batch.collect_job(self.job, self.output, self.limits, b.batch.DEFAULT_OBSERVATION,
                                     adapter=self.adapter(events=events))
        self.assertTrue(result["complete"])
        self.assertTrue(b.batch.verify_finalization(self.output)["training_eligible"])
        passages = [json.loads(line) for line in (self.output / "valid_passages.jsonl").read_text().splitlines()]
        self.assertTrue(all(usable_passage(row) for row in passages))
        self.assertNotIn("b", [row["vehicle_key"] for row in passages])
        rejected = [json.loads(line) for line in (self.output / "rejected_records.jsonl").read_text().splitlines()]
        self.assertEqual(len(rejected), 1)
        self.assertEqual(rejected[0]["observation"]["vehicle_key"], "b")
        self.assertEqual(result["rejected_observation_count"], 1)
        self.assert_training_ingest([2])
        ledger = [json.loads(line) for line in (self.output / "passages.jsonl").read_text().splitlines()]
        self.assertEqual([p["lane_passage_ordinal"] for p in ledger], [1, 2, 3, 4])
        self.assertTrue(ledger[1]["passage_quarantined"])
        # The audit stream is hash-bound, not an untracked discard bin.
        (self.output / "rejected_records.jsonl").write_text("changed")
        with self.assertRaises(b.batch.QualityError):
            b.batch.verify_finalization(self.output)

    def test_new_bad_record_tombstones_do_not_break_training_ingest(self):
        for update in ({"vehicle_id": ""}, {"type_id": None}, {"turn": []},
                       {"permission_class": None}, {"vehicle_key": []},
                       {"time": float("nan")}, {"detector_entry_time": float("inf")}):
            with self.subTest(update=update):
                # Each subcase has an isolated complete dataset.
                self.output = self.root / ("case_" + str(len(list(self.root.glob("case_*")))))
                bad = dict(q.passage("b", 3), permission_class="protected")
                bad.update(update)
                events = [dict(q.start(), permission_class="protected"),
                          dict(q.passage("a", 1), permission_class="protected"), bad,
                          dict(q.start(10, ("a|second", "b|second")), permission_class="protected"),
                          dict(q.passage("a|second", 11, vehicle_id="a"), permission_class="protected"),
                          dict(q.passage("b|second", 13, vehicle_id="b"), permission_class="protected")]
                events = [dict(row, run_id=self.job["run_id"]) for row in events]
                result = b.batch.collect_job(self.job, self.output, self.limits, b.batch.DEFAULT_OBSERVATION,
                                            adapter=self.adapter(events=events))
                self.assertTrue(result["complete"])
                b.batch.verify_finalization(self.output)
                self.assert_training_ingest([2])
                valid = [json.loads(line) for line in (self.output / "valid_passages.jsonl").read_text().splitlines()]
                self.assertTrue(all(usable_passage(p) for p in valid))
                self.assertNotIn("b", [p["vehicle_key"] for p in valid])


class DetectorTests(a.mock.AdapterTests):
    def test_malformed_single_detector_reply_does_not_kill_adapter(self):
        samples = [("a", 5, float("nan"), -1, "car"), ("a", 5),
                   ("a", 5, 100, -1, "car"), ("a", 5, 3.5, 3, "car"),
                   (None, 5, 3.5, -1, "car")]
        for datum in samples:
            connection = a.mock.FakeConnection()
            old = connection.inductionloop.value
            connection.inductionloop.value = lambda d: (
                (datum,) if d == "detector_in_0" and connection.step == 4 else old(d))
            events, meta, _ = self.collect(a.repaired, connection)
            rejected = []
            engine = SequenceExtractor("same_run", rejection_sink=rejected.append)
            for row in events:
                engine.feed(copy.deepcopy(row))
            engine.finish()
            self.assertTrue(meta["completed"])
            self.assertTrue(rejected)
            self.assertTrue(any(row.get("vehicle_id") == "b" for row in engine.store.iter_passages()))


if __name__ == "__main__":
    unittest.main()
