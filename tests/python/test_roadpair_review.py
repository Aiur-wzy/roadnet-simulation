"""Independent pure-Python contract review; no SUMO process or file fixtures."""
from collections import defaultdict, deque
from types import SimpleNamespace as NS
import unittest

from collection.roadpair_context import RoadPairCapContextObserver, ReservationLedger
from features.cap_context_features import ContextConfig, ShadowRecord


def skeletal_observer():
    observer = RoadPairCapContextObserver.__new__(RoadPairCapContextObserver)
    observer.context_config = ContextConfig()
    observer.ledger = ReservationLedger(observer.context_config)
    observer.previous = {}
    observer.routes = {}
    observer.frozen = {}
    observer.reservations = 0
    observer.entry_queues = defaultdict(deque)
    observer.last_time = 0.0
    observer.assign_due_departures = lambda time: None
    return observer


class IndependentRoadPairReview(unittest.TestCase):
    def test_matching_completed_and_valid_occurrences_pass(self):
        observer = skeletal_observer()
        observer.frozen = {("v", 0): {}, ("v", 1): {}}
        events = [NS(vehicle_id="v", route_index=i) for i in (0, 1)]
        observer.validate_training_events(events, NS(iter_valid_training_events=iter))

    def test_missing_completed_occurrence_is_not_certified(self):
        observer = skeletal_observer()
        observer.frozen = {("v", 0): {}, ("v", 1): {}}
        events = [NS(vehicle_id="v", route_index=1)]
        with self.assertRaisesRegex(ValueError, "incomplete/duplicate label"):
            observer.validate_training_events(events, NS(iter_valid_training_events=iter))

    def test_duplicate_occurrence_is_not_certified(self):
        observer = skeletal_observer()
        observer.frozen = {("v", 0): {}}
        events = [NS(vehicle_id="v", route_index=0)] * 2
        with self.assertRaisesRegex(ValueError, "incomplete/duplicate label"):
            observer.validate_training_events(events, NS(iter_valid_training_events=iter))

    def test_legacy_filter_cannot_silently_remove_zero_label(self):
        observer = skeletal_observer()
        observer.frozen = {("v", 0): {}, ("v", 1): {}}
        events = [NS(vehicle_id="v", route_index=i) for i in (0, 1)]
        base = NS(iter_valid_training_events=lambda values: iter(values[:1]))
        with self.assertRaisesRegex(ValueError, "label validity would drop"):
            observer.validate_training_events(events, base)

    def test_wrong_occurrence_release_leaves_incremental_totals_intact(self):
        ledger = ReservationLedger()
        ledger.reserve(ShadowRecord("v", 3, "A", 0, 1, 100, 10, 7.5))
        with self.assertRaisesRegex(ValueError, "wrong occurrence release"):
            ledger.release("v", 2)
        self.assertEqual(ledger.lane_load("A", 0), (1, 7.5))
        self.assertEqual(ledger.road_counts["A"], 1)
        self.assertTrue(ledger.release("v", 3))
        self.assertEqual(ledger.lane_load("A", 0), (0, 0.0))
        self.assertEqual(ledger.road_counts["A"], 0)

    def test_same_road_revisit_does_not_count_previous_ego(self):
        observer = skeletal_observer()
        observer.routes = {"v": ("A", "A")}
        observer.metadata = NS(vehicle_options={"v": {"depart": "0"}}, type_options={})
        observer.network = NS(lengths={"A": 100}, speed_limits={"A": 10},
                              capacities={"A": 14}, select=lambda *args: (0, 0, 0))
        observer.signals = NS(features=lambda *args: {}, incoming_junction_status=lambda *args: "unsignalized_node")
        observer.entry_crossing_seconds = lambda *args: 0.0
        observer._vehicle_data = lambda vid: dict(vehicle_type="DEFAULT_VEHTYPE", speed_factor=1,
            vehicle_max_speed=10, desired_max_speed=10, vehicle_length=5, vehicle_min_gap=2.5, depart_pos="base")
        observer.ledger.reserve(ShadowRecord("v", 0, "A", 0, 0, 100, 10, 7.5))
        observer.frozen["v", 0] = {}
        observer.reserve("v", 1, 11)
        self.assertEqual(observer.frozen["v", 1]["candidate_count"], 0)
        self.assertEqual(observer.frozen["v", 1]["shadow_lane_flow"], 1)
        self.assertEqual(observer.ledger.record_for("v").route_index, 1)

    def test_insertions_follow_schedule_order_not_lexical_order(self):
        observer = skeletal_observer()
        observer.departure_rank = {"z": (0, 0), "a": (0, 1)}
        observer.entry_queues["A", 0].extend(["z", "a"])
        observer.api = NS(simulation=NS(getTime=lambda: 1, getStartingTeleportIDList=lambda: [],
            getArrivedIDList=lambda: []), vehicle=NS(getIDList=lambda: ["a", "z"],
            getRoadID=lambda vid: "A", getRouteIndex=lambda vid: 0))
        order = []
        def reserve(vid, index, time):
            self.assertEqual(observer.entry_queues["A", 0].popleft(), vid)
            order.append(vid)
        observer.reserve = reserve
        observer.on_step()
        self.assertEqual(order, ["z", "a"])

    def test_unobserved_terminal_arrival_fails(self):
        observer = skeletal_observer()
        observer.previous = {"v": ("A", 0)}
        observer.routes = {"v": ("A", "B")}
        observer.api = NS(simulation=NS(getTime=lambda: 1, getStartingTeleportIDList=lambda: [],
            getArrivedIDList=lambda: ["v"]), vehicle=NS(getIDList=lambda: []))
        with self.assertRaisesRegex(ValueError, "unobserved terminal road"):
            observer.on_step()

    def test_same_step_capacity_disagreement_remains_explicit(self):
        # This documents a limitation, not an impossible SUMO transition:
        # B's outgoing vehicle sorts after its incoming vehicle in the batch.
        observer = skeletal_observer()
        observer.previous = {"a_in": ("A", 0), "z_out": ("B", 0)}
        observer.departure_rank = {}
        observer.ledger.reserve(ShadowRecord("a_in", 0, "A", 0, 0, 100, 10))
        observer.ledger.reserve(ShadowRecord("z_out", 0, "B", 0, 0, 100, 10))
        observer.api = NS(simulation=NS(getTime=lambda: 1, getStartingTeleportIDList=lambda: [],
            getArrivedIDList=lambda: []), vehicle=NS(getIDList=lambda: ["a_in", "z_out"],
            getRoadID=lambda vid: "B" if vid == "a_in" else "C", getRouteIndex=lambda vid: 1))
        def reserve(vid, index, time):
            if vid == "a_in" and observer.ledger.lane_load("B", 0)[0] >= 1:
                raise ValueError("shadow admission disagreement")
            observer.ledger.release(vid)
        observer.reserve = reserve
        with self.assertRaisesRegex(ValueError, "shadow admission disagreement"):
            observer.on_step()
        self.assertEqual(observer.ledger.record_for("a_in").road_id, "A")
        self.assertEqual(observer.ledger.record_for("z_out").road_id, "B")


if __name__ == "__main__":
    unittest.main()
