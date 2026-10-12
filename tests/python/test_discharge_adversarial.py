"""Independent adversarial contracts for lane-exit discharge extraction.

These cases exercise observed event semantics, not a second implementation of
the segmenter. SUMO integration and model/data validation cases are added here
as the corresponding public interfaces become available.
"""

from __future__ import annotations

import copy
import shutil
import unittest
import uuid
from contextlib import contextmanager
from pathlib import Path

from features.discharge_sequences import EVENT_SCHEMA, SQLiteStore, SequenceExtractor


@contextmanager
def fixture_directory():
    # Python 3.12's mode-0700 tempfile directories can deny Windows sandbox ACLs.
    from roadnet_paths import ROOT
    root = ROOT
    directory = root / (".test_discharge_" + uuid.uuid4().hex)
    directory.mkdir(mode=0o777)
    try:
        yield directory
    finally:
        resolved = directory.resolve()
        if resolved.parent != root or not resolved.name.startswith(".test_discharge_"):
            raise ValueError("unexpected fixture cleanup path")
        shutil.rmtree(resolved)


def event(kind, time, lane="road_0", section="exit", **extra):
    row = {"schema": EVENT_SCHEMA, "run_id": "adversarial-run", "event": kind,
           "time": time, "lane_id": lane, "section_id": section}
    row.update(extra)
    return row


def start(time=0.0, vehicles=("a", "b", "c"), **extra):
    return event("start", time, valid_start=True, start_kind="stopped_green_start",
                 cohort=[{"vehicle_key": vehicle, "vehicle_id": vehicle,
                          "position": 90.0 - index * 7.5}
                         for index, vehicle in enumerate(vehicles)], **extra)


def passage(vehicle, time, **extra):
    extra.setdefault("turn", "s")
    return event("passage", time, vehicle_key=vehicle, vehicle_id=vehicle,
                 type_id="car", **extra)


def motion(vehicle, time, speed, **extra):
    return event("motion", time, vehicle_key=vehicle, speed=speed, **extra)


def complete(extractor):
    extractor.finish()
    return list(extractor.store.iter_sequences())


class LaneExitAdversarialTests(unittest.TestCase):
    def setUp(self):
        self.extractor = SequenceExtractor("adversarial-run")

    def feed(self, *events):
        for row in events:
            self.extractor.feed(row)

    def test_mixed_movements_keep_physical_adjacency(self):
        self.feed(start(), passage("a", 1, movement_id="straight", turn="s"),
                  passage("b", 3, movement_id="left", turn="l"),
                  passage("c", 4.5, movement_id="straight", turn="s"))
        row, = complete(self.extractor)
        self.assertEqual([2, 1.5], [pair["headway"] for pair in row["headways"]])
        self.assertEqual([("s", "l"), ("l", "s")],
                         [(pair["prev_turn"], pair["turn"]) for pair in row["headways"]])

    def test_one_lane_restop_does_not_cut_neighbor_lane(self):
        self.feed(start(lane="road_0"), start(lane="road_1"),
                  motion("b", 0.5, 2, lane="road_0"),
                  passage("a", 1, lane="road_0"), passage("a", 1, lane="road_1"),
                  motion("b", 1.5, 0, lane="road_0"),
                  passage("b", 3, lane="road_1"), passage("c", 4, lane="road_1"))
        rows = {row["lane_id"]: row for row in complete(self.extractor)}
        self.assertEqual("re_stopped_after_start", rows["road_0"]["cut_reason"])
        self.assertEqual(2, len(rows["road_1"]["headways"]))

    def test_initial_unstarted_rear_car_zero_is_not_restop(self):
        self.feed(start(), motion("a", 0.2, 2), motion("b", 0.3, 0),
                  motion("c", 0.4, 0), passage("a", 1), passage("b", 3), passage("c", 5))
        row, = complete(self.extractor)
        self.assertEqual("cohort_end", row["cut_reason"])
        self.assertEqual(3, len(row["passages"]))

    def test_first_car_restop_before_crossing_records_failed_start(self):
        self.feed(start(), motion("a", 0.2, 2), motion("a", 0.3, 0),
                  passage("a", 1))
        row, = complete(self.extractor)
        self.assertEqual([], row["passages"])
        self.assertIsNone(row["startup_delay"])
        self.assertEqual(1, self.extractor.summary()["totals"]["zero_passage_sequences"])
        self.assertEqual(1, len(list(self.extractor.store.iter_passages())))

    def test_unrelated_vehicle_stop_does_not_cut(self):
        self.feed(start(), motion("outside", 0.1, 2), motion("outside", 0.2, 0),
                  passage("a", 1), passage("b", 3), passage("c", 5))
        row, = complete(self.extractor)
        self.assertEqual("cohort_end", row["cut_reason"])

    def test_closed_sequence_cannot_resume_but_new_start_can(self):
        self.feed(start(), passage("a", 1), event("cut", 2, reason="downstream_blocked"),
                  passage("b", 3), start(5, ("d", "e")), passage("d", 6), passage("e", 8))
        first, second = complete(self.extractor)
        self.assertEqual(["a"], [row["vehicle_key"] for row in first["passages"]])
        self.assertEqual(["d", "e"], [row["vehicle_key"] for row in second["passages"]])
        self.assertNotEqual(first["sequence_id"], second["sequence_id"])
        self.assertEqual([2], [pair["headway"] for pair in second["headways"]])
        self.assertEqual([3, 4], [row["lane_passage_ordinal"] for row in second["passages"]])

    def test_late_cut_revokes_saved_tail_in_memory_and_sqlite(self):
        for persistent in (False, True):
            with self.subTest(persistent=persistent), fixture_directory() as directory:
                store = SQLiteStore(Path(directory) / "spool.sqlite") if persistent else None
                extractor = SequenceExtractor("adversarial-run", store=store)
                for row in [start(), passage("a", 2), passage("b", 4.5), passage("c", 6.7),
                            event("cut", 8, cut_time=6.5, reason="downstream_blocked")]:
                    extractor.feed(row)
                segment, = complete(extractor)
                self.assertEqual([2, 4.5], [row["time"] for row in segment["passages"]])
                self.assertEqual([2.5], [row["headway"] for row in segment["headways"]])
                self.assertEqual(3, len(list(extractor.store.iter_passages())))
                extractor.store.close()

    def test_cut_on_passage_boundary_is_order_independent(self):
        outputs = []
        boundary_events = [passage("b", 3), event("cut", 3, reason="ambiguous_substep")]
        for ordered in (boundary_events, list(reversed(boundary_events))):
            extractor = SequenceExtractor("adversarial-run")
            for row in [start(), passage("a", 1)] + ordered:
                extractor.feed(copy.deepcopy(row))
            segment, = complete(extractor)
            outputs.append([row["vehicle_key"] for row in segment["passages"]])
        self.assertEqual([["a"], ["a"]], outputs)

    def test_negative_speed_unknown_is_not_zero(self):
        self.feed(start(), motion("a", 0.2, 2), motion("a", 0.3, -1))
        row, = complete(self.extractor)
        self.assertEqual("unknown_motion_or_disappearance", row["cut_reason"])

    def test_missing_motion_after_time_zero_revokes_unobserved_interval(self):
        self.feed(start(), passage("a", 0.5), motion("b", 1, None, present=False))
        row, = complete(self.extractor)
        self.assertEqual(0, row["cut_time"])
        self.assertEqual([], row["passages"])
        self.assertEqual(1, len(list(self.extractor.store.iter_passages())))

    def test_lane_insertion_does_not_bridge_around_unknown_car(self):
        self.feed(start(), passage("a", 1), passage("inserted", 2),
                  passage("b", 3), passage("c", 5))
        row, = complete(self.extractor)
        self.assertEqual(["a"], [value["vehicle_key"] for value in row["passages"]])
        self.assertEqual([], row["headways"])
        self.assertEqual(4, len(list(self.extractor.store.iter_passages())))

    def test_raw_duplicate_does_not_consume_ordinal(self):
        self.feed(start(), passage("a", 1), passage("a", 1), passage("b", 3), passage("c", 5))
        row, = complete(self.extractor)
        self.assertEqual([1, 2, 3], [value["lane_passage_ordinal"] for value in row["passages"]])
        self.assertEqual(1, self.extractor.summary()["events"]["duplicate_passage_observations"])

    def test_equal_passage_times_are_not_a_zero_headway_training_label(self):
        self.feed(start(), passage("a", 1), passage("b", 1), passage("c", 2))
        row, = complete(self.extractor)
        self.assertEqual([], row["headways"])
        self.assertEqual(3, len(list(self.extractor.store.iter_passages())))

    def test_future_cut_and_schema_mismatch_rejected(self):
        for row in (event("cut", 1, cut_time=2, reason="future"),
                    dict(start(), schema="lane-discharge-event-v999")):
            with self.subTest(row=row), self.assertRaises(ValueError):
                SequenceExtractor("adversarial-run").feed(row)

    def test_numeric_error_speeds_never_become_clean_stop_labels(self):
        for speed in (float("nan"), float("inf"), True, "0"):
            with self.subTest(speed=speed):
                extractor = SequenceExtractor("adversarial-run")
                for row in (start(), motion("a", 0.2, 2), motion("a", 0.3, speed)):
                    extractor.feed(row)
                segment, = complete(extractor)
                self.assertEqual("unknown_motion_or_disappearance", segment["cut_reason"])

    def test_permission_change_cannot_bridge_gross_headways(self):
        self.feed(start(permission_class="protected"),
                  passage("a", 1, permission_class="protected"),
                  passage("b", 3, permission_class="yield"),
                  passage("c", 5, permission_class="protected"))
        row, = complete(self.extractor)
        self.assertEqual("permission_class_changed", row["cut_reason"])
        self.assertEqual(["a"], [value["vehicle_key"] for value in row["passages"]])
        self.assertEqual([], row["headways"])
        self.assertEqual(3, len(list(self.extractor.store.iter_passages())))

    def test_missing_passage_permission_is_not_assumed_protected(self):
        self.feed(start(permission_class="protected"), passage("a", 1))
        row, = complete(self.extractor)
        self.assertEqual([], row["passages"])
        self.assertEqual("permission_class_changed", row["cut_reason"])

    def test_unknown_start_permission_rejected(self):
        with self.assertRaises(ValueError):
            self.extractor.feed(start(permission_class="invented_unknown"))

    def test_same_permission_different_movement_continues(self):
        self.feed(start(permission_class="protected"),
                  passage("a", 1, permission_class="protected", movement_id="straight"),
                  passage("b", 3, permission_class="protected", movement_id="left"),
                  passage("c", 4, permission_class="protected", movement_id="straight"))
        row, = complete(self.extractor)
        self.assertEqual([2, 1], [pair["headway"] for pair in row["headways"]])


class TrainingContractAdversarialTests(unittest.TestCase):
    @staticmethod
    def sample():
        extractor = SequenceExtractor("adversarial-run")
        for row in (start(), passage("a", 1), passage("b", 3), passage("c", 5)):
            extractor.feed(row)
        return complete(extractor)[0]

    @staticmethod
    def validator():
        from training.train_discharge_sequences import DatasetError, validate_sequence
        return DatasetError, validate_sequence

    def test_collector_sequence_satisfies_training_contract(self):
        _, validate = self.validator()
        row = self.sample()
        self.assertIs(row, validate(row, "adversarial-run"))

    def test_malformed_prefix_is_rejected_without_repair(self):
        error, validate = self.validator()
        mutations = {
            "jump_local_rank": lambda s: s["passages"][1].update(local_j=3),
            "jump_raw_ordinal": lambda s: s["passages"][1].update(lane_passage_ordinal=7),
            "cross_lane": lambda s: s["passages"][1].update(lane_id="other_lane"),
            "headway_nan": lambda s: s["headways"][0].update(headway=float("nan")),
            "cohort_insertion": lambda s: s["cohort_vehicle_keys"].__setitem__(1, "inserted"),
            "wrong_pair_front": lambda s: s["headways"][0].update(prev_vehicle_key="c"),
            "still_open": lambda s: s.update(closed=False),
            "unknown_schema": lambda s: s.update(schema="lane-discharge-sequence-v999"),
            "changed_rule": lambda s: s.update(rule_version="arbitrary-incompatible-segmentation"),
            "rolling_start": lambda s: s.update(start_kind="rolling_no_stopped_evidence"),
        }
        for label, mutate in mutations.items():
            row = self.sample()
            mutate(row)
            with self.subTest(label=label), self.assertRaises(error):
                validate(row, "adversarial-run")

    def test_run_split_never_divides_one_run(self):
        from training.train_discharge_sequences import DatasetError, choose_split
        with self.assertRaises(DatasetError):
            choose_split(["same", "same"])
        assignments = choose_split(["a", "b", "c"], holdout_runs=["b"])
        self.assertEqual({"a": "train", "b": "validation", "c": "train"}, assignments)
        self.assertEqual({"only": "train"}, choose_split(["only"]))


class ReceiverInterferenceAdversarialTests(unittest.TestCase):
    @staticmethod
    def gate(foreign_position=None):
        from collection.discharge_sumo import Connection, classify_gate
        connection = Connection("in", "in_0", "", "out", "out_0", "j", 0, "s")
        registry = {"lanes": {"out_0": {"length": 100, "internal": False}},
                    "internal_successors": {}}
        rows = [{"vehicle_key": "same_cohort", "vehicle_id": "a", "position": 6,
                 "length": 5, "speed": 0.2, "min_gap": 2.5}]
        if foreign_position is not None:
            rows.append({"vehicle_key": "foreign", "vehicle_id": "b",
                         "position": foreign_position, "length": 5,
                         "speed": 0, "min_gap": 2.5})
        candidate = {"vehicle_key": "next", "length": 5, "min_gap": 2.5}
        links = [("out_0", True, True, False, "", "G", "s", 10)]
        return classify_gate(connection, links, {"out_0": rows}, candidate,
                             registry, own_keys={"same_cohort"})

    def test_own_moving_predecessor_without_external_queue_can_be_clear(self):
        self.assertEqual("clear", self.gate()["status"])

    def test_own_creeping_predecessor_against_stopped_queue_is_not_clear(self):
        self.assertIn(self.gate(13.5)["status"], {"blocked", "unknown"})

    def test_far_stopped_receiver_vehicle_does_not_prove_entry_blockage(self):
        self.assertEqual("clear", self.gate(90)["status"])


if __name__ == "__main__":
    unittest.main()
