"""Unit tests for movement-specific missed-green counting without SUMO."""

import os
import sys
import types
import unittest


os.environ.setdefault("SUMO_HOME", "/tmp")
sys.modules.setdefault("traci", types.SimpleNamespace())
import TraCI_Python_Adjusted as cams


def config():
    return cams.CamsConfig(
        "", "", "", "", "", "", 80.0, 0.1, 2.0, 2.0, 85.0, "", 2.0, set()
    )


def snapshot(vehicle_id="v", key=("tls", 1), distance=10.0, speed=0.0):
    return cams.VehicleSnapshot(
        vehicle_id, "r", ("a", "b"), 0, "a", "b", "a_0", key[0], key[1], "G",
        distance, speed, 0.0, 100.0, "s", 2, 5.0, 2.5,
    )


def event(vehicle_id="v", entered=1.0, passed=None):
    return cams.VehicleMovementEvent(
        vehicle_id, "r", ("a", "b"), 0, "a", "b", "a_0", "tls", 1,
        0.0, "s", 5.0, 2.5, waiting_region_enter_time=entered,
        pass_stopline_time=passed,
    )


def cycle(cycle_id=1, start=0.0, blocked=False):
    result = cams.MovementCycleSummary("tls", 1, cycle_id, start, 1)
    if blocked:
        result.downstream_blocked_vehicle_ids.add("v")
    return result


class MissedGreenTests(unittest.TestCase):
    def count(self, e, s, c, now=5.0):
        cams.count_missed_green_at_cycle_end(("tls", 1), c, {"v": s}, {"v": e}, config(), now)

    def test_direct_and_first_green_passes_are_zero(self):
        for entered in (1.0, -2.0):
            e, c = event(entered=entered, passed=5.0), cycle()
            self.count(e, snapshot(), c)
            self.assertEqual(e.missed_green_rounds, 0)

    def test_misses_one_then_two_distinct_windows(self):
        e = event()
        self.count(e, snapshot(), cycle(1), 5.0)
        self.count(e, snapshot(), cycle(2, 10.0), 15.0)
        self.assertEqual(e.missed_green_rounds, 2)
        self.assertEqual(len(set(e.missed_green_cycle_ids)), 2)

    def test_late_arrival_has_no_opportunity(self):
        e = event(entered=4.0)
        self.count(e, snapshot(), cycle(), 5.0)
        self.assertEqual(e.missed_green_rounds, 0)

    def test_queued_at_start_overrides_short_green(self):
        e, c = event(entered=4.9), cycle()
        e.queued_at_green_start_cycle = cams.cycle_identity(c)
        self.count(e, snapshot(), c, 5.0)
        self.assertEqual(e.missed_green_rounds, 1)

    def test_repeat_movement_pass_and_speed_guards(self):
        e, c = event(), cycle()
        self.count(e, snapshot(), c)
        self.count(e, snapshot(), c)  # duplicate cycle is idempotent
        self.assertEqual(e.missed_green_rounds, 1)
        for guarded_event, guarded_snapshot in (
            (event(), snapshot(key=("tls", 2))),
            (event(passed=5.0), snapshot()),
            (event(), snapshot(speed=2.1)),
        ):
            self.count(guarded_event, guarded_snapshot, cycle())
            self.assertEqual(guarded_event.missed_green_rounds, 0)

    def test_reason_accounting_and_same_step_pass(self):
        blocked_event, blocked_cycle = event(), cycle(blocked=True)
        self.count(blocked_event, snapshot(), blocked_cycle)
        self.assertEqual(blocked_event.missed_green_due_to_downstream_block, 1)
        queue_event, queue_cycle = event(), cycle()
        self.count(queue_event, snapshot(), queue_cycle)
        self.assertEqual(queue_event.missed_green_due_to_queue, 1)
        self.assertEqual(queue_event.missed_green_rounds,
                         queue_event.missed_green_due_to_queue + queue_event.missed_green_due_to_downstream_block)
        same_step_event = event(passed=5.0)
        self.count(same_step_event, snapshot(), cycle())
        self.assertEqual(same_step_event.missed_green_rounds, 0)

    def test_no_end_transition_means_no_count(self):
        # The caller only invokes count_missed_green_at_cycle_end on G/g -> non-green.
        e = event()
        active = cycle()
        self.assertEqual(e.missed_green_rounds, 0)
        self.assertIsNone(active.green_end_time)


if __name__ == "__main__":
    unittest.main()
