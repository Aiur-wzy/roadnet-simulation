"""Transport/natural-drain checks; no real SUMO process is launched."""

import copy
from contextlib import ExitStack
import importlib.util
from pathlib import Path
import shutil
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import uuid

import collection.discharge_sumo as legacy


ADAPTER_PATH = Path(__file__).resolve().parents[2] / "collection" / "discharge_batch_sumo.py"
SPEC = importlib.util.spec_from_file_location("discharge_batch_sumo_under_test", ADAPTER_PATH)
adapter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(adapter)

E1_VAR = 0x17


class FakeTraCIException(Exception):
    pass


def registry():
    connection = legacy.Connection("in", "in_0", "", "out", "out_0", "tls", 0, "s")
    unused = legacy.Connection("empty", "empty_0", "", "out", "out_0", "tls", 1, "s")
    lanes = {name: {"lane_id": name, "edge_id": edge, "length": 100.0,
                    "speed_limit": 13.89, "internal": False}
             for name, edge in (("in_0", "in"), ("empty_0", "empty"), ("out_0", "out"))}
    sections = {name: {"section_id": "lane_outlet:" + name,
                       "detector_id": "detector_" + name, "position": 100.0,
                       "offset_from_lane_end": 0.0, "road_lane_count": 1}
                for name in ("in_0", "empty_0")}
    return {"lanes": lanes, "connections": {"in_0": [connection], "empty_0": [unused]},
            "internal_successors": {}, "sections": sections}


class FakeSimulation:
    def __init__(self, owner):
        self.owner = owner

    def getTime(self):
        return float(self.owner.step)

    def getDeltaT(self):
        return 1.0

    def getMinExpectedNumber(self):
        return 0 if self.owner.step >= 6 and not self.owner.never_drain else 2

    def getDepartedNumber(self):
        return 2 if self.owner.step == 1 else 0

    def getArrivedNumber(self):
        return 2 if self.owner.step == 6 and not self.owner.never_drain else 0

    def getStartingTeleportIDList(self):
        return ("a",) if self.owner.teleport and self.owner.step == 4 else ()

    def getStartingTeleportNumber(self):
        return len(self.getStartingTeleportIDList())

    def getEndingTeleportNumber(self):
        return int(self.owner.teleport and self.owner.step == 5)

    def getCollisions(self):
        return (object(),) if self.owner.collision and self.owner.step == 4 else ()

    def getCollidingVehiclesNumber(self):
        return 2 if self.owner.collision and self.owner.step == 4 else 0


class FakeVehicles:
    def __init__(self, owner):
        self.owner = owner

    def state(self, vehicle):
        step = self.owner.step
        if step <= 2:
            return "in_0", (97.0 if vehicle == "a" else 89.0), 0.0
        if step == 3:
            return "in_0", (98.0 if vehicle == "a" else 90.0), 1.0
        if step == 4:
            return ("out_0", 3.0, 5.0) if vehicle == "a" else ("in_0", 95.0, 3.0)
        return "out_0", (18.0 if vehicle == "a" else 4.0), 4.0

    def getIDList(self):
        return ("a", "b") if 1 <= self.owner.step <= 5 else ()

    def getLaneID(self, vehicle):
        return self.state(vehicle)[0]

    def getRouteIndex(self, vehicle):
        return int(self.state(vehicle)[0] == "out_0")

    def getLanePosition(self, vehicle):
        return self.state(vehicle)[1]

    def getSpeed(self, vehicle):
        return self.state(vehicle)[2]

    def getTypeID(self, vehicle):
        return "car"

    def getSpeedFactor(self, vehicle):
        return 1.0

    def getStopState(self, vehicle):
        return 0

    def getLength(self, vehicle):
        return 5.0

    def getMinGap(self, vehicle):
        return 2.5

    def getRoute(self, vehicle):
        return ("in", "out")


class FakeDetectors:
    def __init__(self, owner):
        self.owner = owner
        self.subscribed = []
        self.getter_calls = 0
        self.cache_calls = 0

    def subscribe(self, detector, variables):
        self.subscribed.append((detector, variables))

    def value(self, detector):
        if detector != "detector_in_0":
            return ()
        if self.owner.step == 4:
            return (("a", 5.0, 3.5, 3.9, "car"),)
        if self.owner.step == 5:
            return (("b", 5.0, 4.6, 4.9, "car"),)
        return ()

    def getVehicleData(self, detector):
        self.getter_calls += 1
        return self.value(detector)

    def getAllSubscriptionResults(self):
        self.cache_calls += 1
        values = {detector: {E1_VAR: self.value(detector)}
                  for detector, _ in self.subscribed}
        if self.owner.missing_detector and self.owner.step == 2:
            values.pop("detector_empty_0")
        return values


class FakeConnection:
    def __init__(self, never_drain=False, missing_detector=False, teleport=False, collision=False):
        self.step = 0
        self.closed = False
        self.never_drain = never_drain
        self.missing_detector = missing_detector
        self.teleport = teleport
        self.collision = collision
        self.simulation = FakeSimulation(self)
        self.vehicle = FakeVehicles(self)
        self.inductionloop = FakeDetectors(self)
        self.vehicletype = SimpleNamespace(
            getLength=lambda _: 5.0, getMinGap=lambda _: 2.5,
            getTau=lambda _: 1.0, getAccel=lambda _: 2.6,
            getDecel=lambda _: 4.5, getActionStepLength=lambda _: 1.0)
        self.lane = SimpleNamespace(getLinks=self.links)

    def links(self, lane, extended=True):
        return (("out_0", True, True, False, "", "r" if self.step <= 2 else "G", "s", 0.0),)

    def getVersion(self):
        return (22, "fake-SUMO")

    def simulationStep(self):
        self.step += 1

    def close(self):
        self.closed = True


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / "_adapter_test_artifacts" / uuid.uuid4().hex
        self.root.mkdir(parents=True)
        self.started = []

    def tearDown(self):
        if self.root.parent.name != "_adapter_test_artifacts":
            raise RuntimeError("unexpected test cleanup path")
        shutil.rmtree(self.root)

    def collect(self, module, connection=None, **overrides):
        connection = connection or FakeConnection()
        fake_traci = SimpleNamespace(
            constants=SimpleNamespace(LAST_STEP_VEHICLE_DATA=E1_VAR),
            TraCIException=FakeTraCIException,
            start=lambda command, **kwargs: self.started.append(command),
            getConnection=lambda _: connection)
        metadata = overrides.pop("metadata", {})
        config = {"output_dir": self.root / uuid.uuid4().hex, "end": 10.0,
                  "run_id": "same_run", "metadata": metadata, **overrides}
        inputs = {"net": "fake.net.xml", "routes": ["fake.rou.xml"],
                  "additional_files": [], "sumo_config": None,
                  "config_values": {"seed": "42", "time-to-teleport": "-1"}}
        isolation_mock = (("_prepare_isolated_inputs", (None, [])) if module is legacy
                          else ("_prepare_batch_isolated_inputs", (None, [], [])))
        with ExitStack() as stack:
            for name, value in (("_import_traci", fake_traci), ("resolve_inputs", inputs),
                                ("build_network_registry", registry()),
                                ("_write_detectors", "observer.add.xml"),
                                isolation_mock, ("_hash", "f" * 64)):
                stack.enter_context(patch.object(module, name, return_value=copy.deepcopy(value)
                                                if name == "build_network_registry" else value))
            events = list(module.iter_sumo_events(config))
        return events, metadata, connection

    def test_events_identical_to_original_for_complete_run(self):
        old, _, old_connection = self.collect(legacy)
        new, metadata, new_connection = self.collect(adapter)
        self.assertEqual(new, old)
        self.assertTrue(any(item["kind"] == "start" for item in new))
        self.assertEqual(sum(item["kind"] == "passage" for item in new), 2)
        self.assertEqual(metadata["termination"], "natural_drain")
        self.assertTrue(metadata["completed"])
        self.assertEqual(metadata["remaining"], 0)
        self.assertEqual(metadata["cumulative_counts"]["departed"], 2)
        self.assertEqual(metadata["cumulative_counts"]["arrived"], 2)
        self.assertEqual(old_connection.inductionloop.getter_calls, 12)
        self.assertEqual(new_connection.inductionloop.getter_calls, 0)
        self.assertEqual(new_connection.inductionloop.cache_calls, 6)
        self.assertEqual(len(new_connection.inductionloop.subscribed), 2)

    def test_diagnostic_horizon_identical_but_not_complete(self):
        old, _, _ = self.collect(legacy, end=4)
        new, metadata, connection = self.collect(adapter, end=4, diagnostic_horizon=True)
        self.assertEqual(new, old)
        self.assertFalse(metadata["completed"])
        self.assertEqual(metadata["termination"], "diagnostic_horizon")
        self.assertEqual(metadata["remaining"], 2)
        self.assertTrue(connection.closed)

    def test_safety_cap_is_failure_not_complete(self):
        metadata = {}
        connection = FakeConnection(never_drain=True)
        with self.assertRaises(adapter.ObservationHorizonExceeded) as caught:
            self.collect(adapter, connection, metadata=metadata, end=4)
        self.assertEqual(caught.exception.remaining, 2)
        self.assertFalse(metadata["completed"])
        self.assertEqual(metadata["termination"], "safety_cap_with_remaining")
        self.assertTrue(connection.closed)

    def test_missing_even_empty_lane_subscription_fails_closed(self):
        metadata = {}
        connection = FakeConnection(missing_detector=True)
        with self.assertRaisesRegex(RuntimeError, "missing E1 subscription"):
            self.collect(adapter, connection, metadata=metadata)
        self.assertFalse(metadata["completed"])
        self.assertEqual(metadata["termination"], "adapter_failure")
        self.assertTrue(connection.closed)

    def test_callback_failure_closes_private_connection(self):
        metadata = {}
        connection = FakeConnection()
        def protect(_):
            raise RuntimeError("disk guard")
        with self.assertRaisesRegex(RuntimeError, "disk guard"):
            self.collect(adapter, connection, metadata=metadata, progress_callback=protect)
        self.assertFalse(metadata["completed"])
        self.assertTrue(connection.closed)

    def test_progress_and_native_outputs_are_explicit(self):
        progress = []
        _, metadata, _ = self.collect(adapter, progress_callback=progress.append, progress_interval=2)
        self.assertEqual([item["simulation_time"] for item in progress], [1, 3, 5])
        self.assertEqual(progress[0]["running"], 2)
        command = self.started[-1]
        self.assertEqual(command[command.index("--threads") + 1], "1")
        self.assertEqual(command[command.index("--tripinfo-output") + 1], metadata["tripinfo_path"])
        self.assertEqual(command[command.index("--tripinfo-output.write-unfinished") + 1], "true")

    def test_teleport_and_collision_counts_are_not_hidden(self):
        _, metadata, _ = self.collect(adapter, FakeConnection(teleport=True, collision=True))
        counts = metadata["cumulative_counts"]
        self.assertEqual(counts["teleport_start"], 1)
        self.assertEqual(counts["teleport_end"], 1)
        self.assertEqual(counts["collision_events"], 1)
        self.assertEqual(counts["colliding_vehicle_reports"], 2)

    def test_invalid_progress_interval_rejected_before_launch(self):
        with self.assertRaises(ValueError):
            self.collect(adapter, progress_interval=0)
        self.assertFalse(self.started)

    def test_diagnostic_switch_requires_boolean_before_launch(self):
        with self.assertRaises(ValueError):
            self.collect(adapter, diagnostic_horizon="false")
        self.assertFalse(self.started)

    def test_boolean_log_compatibility_is_private_and_preserves_input(self):
        cfg = self.root / "source.sumocfg"
        original = '<configuration><report><no-step-log value="true"/></report></configuration>'
        cfg.write_text(original, encoding="utf-8")
        inputs = {"sumo_config": str(cfg), "additional_files": []}
        isolated, additions, changes = adapter._prepare_batch_isolated_inputs(inputs, self.root)
        self.assertEqual(cfg.read_text(encoding="utf-8"), original)
        self.assertEqual(inputs["sumo_config"], str(cfg))
        self.assertEqual(changes[0]["tag"], "no-step-log")
        self.assertEqual(changes[0]["effective_value"], "true")
        self.assertNotIn("no-step-log", Path(isolated).read_text(encoding="utf-8"))
        self.assertEqual(additions, [])


class ReaderTests(unittest.TestCase):
    def test_duplicate_detector_ids_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            adapter.E1SubscriptionReader(SimpleNamespace(), {"a": {"detector_id": "x"},
                                                            "b": {"detector_id": "x"}}, E1_VAR)

    def test_cache_cannot_be_read_before_refresh(self):
        reader = adapter.E1SubscriptionReader(SimpleNamespace(subscribe=lambda *args: None),
                                             {"a": {"detector_id": "x"}}, E1_VAR)
        with self.assertRaisesRegex(RuntimeError, "not been refreshed"):
            reader.vehicle_data("x")

    def test_unknown_cache_is_not_empty_traffic(self):
        reader = adapter.E1SubscriptionReader(SimpleNamespace(subscribe=lambda *args: None,
                                             getAllSubscriptionResults=lambda: None), {}, E1_VAR)
        with self.assertRaises(RuntimeError):
            reader.refresh()


if __name__ == "__main__":
    unittest.main()
