"""Collector guards and publication failure injection; no SUMO or network."""
from contextlib import ExitStack, redirect_stdout, redirect_stderr
from copy import deepcopy
import io
import json
from pathlib import Path
import shutil
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import uuid
import xml.etree.ElementTree as ET

from collection.native_boundary_v2 import collect_native_v2 as collector
from collection.sumo_native_state import LABELS


def eligible_row():
    return dict(schema=collector.SCHEMA, vehicle_id="v", route_index=0, road_id="a",
                trainable=1, feature_valid=1, label_valid=1,
                road_length=10., road_flow=1, lane_flow=1, free_speed_cap=10.,
                platoon_context=0, entry_signal_type="G", entry_signal_remaining_seconds=0.,
                next_signal_type="G", next_signal_duration_seconds=0.,
                **{name: .5 for name in LABELS})


class FakeAPI:
    def __init__(self, constants, failure=None):
        self.tc, self.failure, self.frame = constants, failure, -1
        self.closed, self.saved, self.subscribed = False, [], []
        self.simulation = NS(
            getTime=lambda: self.frame + 1. + int(self.failure == "skip_step" and self.frame == 1),
            getDeltaT=lambda: 1., getMinExpectedNumber=lambda: int(self.frame < 1),
            getDepartedIDList=lambda: ["v"] if self.frame == 0 else [],
            getArrivedIDList=lambda: ["v"] if self.frame == 1 else [],
            getStartingTeleportIDList=lambda: [],
            getCollidingVehiclesNumber=lambda: int(self.failure == "collision"),
            saveState=lambda path: self.saved.append(path))
        self.vehicle = NS(getRoute=lambda _: ("a",), getSpeedFactor=lambda _: 1.,
            getDeparture=lambda _: 0., getTypeID=lambda _: "DEFAULT_VEHTYPE",
            getMaxSpeed=lambda _: 10., getLaneChangeMode=lambda _: 1621,
            subscribe=lambda v, keys: self.subscribed.append((v, keys)),
            getAllSubscriptionResults=self.snapshots,
            getIDList=lambda: ["v"] if self.frame == 0 else [])

    def simulationStep(self):
        self.frame += 1

    def getVersion(self):
        return (22, "SUMO 1.27.0")

    def snapshots(self):
        if self.frame == 1 or self.failure == "missing_subscription":
            return {}
        values = ("a", "a_0", 0, 0., 5., 10., 0.)
        return {"v": dict(zip(range(7), values))}

    def close(self):
        self.closed = True
        if self.failure == "native_close":
            raise OSError("injected native close failure")


class FakeSignals:
    def __init__(self, api, network):
        self.api = api

    def features(self, *args):
        return {k: v for k, v in eligible_row().items() if k.startswith("entry_signal") or k.startswith("next_signal")}


class FakeEngine:
    instances = []

    def __init__(self, graph, dt, path, emit):
        self.emit, self.frames, self.closed, self.finalized = emit, [], False, False
        self.departed = []
        self.instances.append(self)

    def depart(self, vid, route, metadata):
        self.departed.append((vid, route, deepcopy(metadata)))

    def step(self, now, observations, **kwargs):
        self.frames.append((now, deepcopy(observations), deepcopy(kwargs)))

    def finalize_tripinfo(self, path):
        self.finalized = True
        self.emit(eligible_row())

    def finish(self):
        return {"counts": {"road_occurrences": 1, "trainable": 1, "excluded": 0}}

    def close(self):
        self.closed = True


class CollectorPureGuardsTests(unittest.TestCase):
    def test_complete_frame_has_no_missing_or_extra_vehicle(self):
        collector.validate_snapshot_frame({"v"}, ["v"], {"v": {1: 3}}, (1,))
        for live, snapshots in ((["v"], {}), ([], {"v": {1: 3}}),
                                (["v", "v"], {"v": {1: 3}}), (["v"], {"v": {}})):
            with self.subTest(live=live, snapshots=snapshots), self.assertRaises(ValueError):
                collector.validate_snapshot_frame({"v"}, live, snapshots, (1,))

    def test_time_guard_rejects_skipped_duplicated_and_reversed_steps(self):
        collector.validate_snapshot_clock({"step_seconds": 1., "snapshot_time": 1.}, 1., 0.)
        for value in (0., -.1, 2., float("nan")):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "snapshot step"):
                collector.validate_snapshot_clock({"step_seconds": 1., "snapshot_time": value}, 1., 0.)
        with self.assertRaisesRegex(ValueError, "step length"):
            collector.validate_snapshot_clock({"step_seconds": .5, "snapshot_time": 1.}, 1., 0.)

    def test_valid_zero_and_subsecond_labels_are_not_filtered(self):
        for label in (0., .05, .5):
            row = eligible_row()
            row.update({name: label for name in LABELS})
            collector.validate_row(row)

    def test_bad_eligible_row_or_threshold_order_is_rejected(self):
        for change in ({"feature_valid": 0}, {"road_flow": float("nan")},
                       {"entry_signal_type": "Y"}, {LABELS[0]: -1.},
                       {LABELS[-1]: 2.}, {"schema": "legacy"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                collector.validate_row(dict(eligible_row(), **change))


class CollectorRunFailureTests(unittest.TestCase):
    def run_mock(self, failure=None):
        parent = Path(__file__).resolve().parent
        root = parent / (".native_streaming_guard_" + uuid.uuid4().hex)
        root.mkdir()
        def cleanup():
            target = root.resolve()
            if target.parent != parent or not target.name.startswith(".native_streaming_guard_"):
                raise ValueError("unsafe collector-test cleanup")
            shutil.rmtree(target)
        self.addCleanup(cleanup)
        args = NS(sumo_config=str(root / "run.sumocfg"), net_file=str(root / "network.net.xml"),
                  output_dir=str(root / "result"), sumo_binary="mock_sumo", stall_seconds=3600., status_period=60)
        names = ("VAR_ROAD_ID", "VAR_LANE_ID", "VAR_ROUTE_INDEX", "VAR_LANEPOSITION",
                 "VAR_SPEED", "VAR_ALLOWED_SPEED", "VAR_DISTANCE")
        constants = NS(**dict(zip(names, range(7))))
        api = FakeAPI(constants, failure)
        traci = NS(constants=constants, start=lambda *a, **kw: None, getConnection=lambda _: api)
        expected = {"v": dict(sf=1., depart=0., route_sha256=collector.route_digest(("a",)), roads=1)}
        edge = ET.fromstring('<edge id="a"><lane id="a_0" length="10" speed="10"/></edge>')
        FakeEngine.instances.clear()
        with ExitStack() as stack:
            stack.enter_context(patch.dict("sys.modules", {"traci": traci}))
            for name, value in (("safe_inputs", lambda *a: (expected, [])),
                                ("NoInternalGeometry", lambda _: NS(lane_roads={"a_0": "a"})),
                                ("load_traci", lambda: traci), ("NativeSignalReaderV2", FakeSignals),
                                ("StreamingQualityV2", FakeEngine), ("sha256", lambda _: "fixed_sha"),
                                ("tripinfo_audit", lambda *a: dict(normal_arrived=1, missing_ids=0, missing_runtime_sf=0)),
                                ("audit_native_xml", lambda *a: 1)):
                stack.enter_context(patch.object(collector, name, value))
            stack.enter_context(patch.object(collector.ET, "iterparse", return_value=[("end", edge)]))
            stack.enter_context(patch("collection.run_manhattan_training_batch_v2.collector_sources", return_value={"source.py": "sha"}))
            if failure == "csv_audit":
                stack.enter_context(patch.object(collector, "audit_closed_csv", side_effect=ValueError("injected closed CSV failure")))
            if failure == "engine_close":
                stack.enter_context(patch.object(FakeEngine, "close", side_effect=OSError("injected output close failure")))
            if failure == "hash_output":
                def guarded_hash(path):
                    if Path(path).name == "tripinfo.xml":
                        raise OSError("injected closed artifact hash failure")
                    return "fixed_sha"
                stack.enter_context(patch.object(collector, "sha256", guarded_hash))
            if failure == "stdout_close":
                original_open = Path.open
                class CloseFailure(io.StringIO):
                    def close(self):
                        if not self.closed:
                            super().close()
                            raise OSError("injected stdout close failure")
                def guarded_open(path, *args, **kwargs):
                    if path.name == "sumo.stdout.log":
                        return CloseFailure()
                    return original_open(path, *args, **kwargs)
                stack.enter_context(patch.object(Path, "open", guarded_open))
            with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                result = collector.run(args)
        out = Path(args.output_dir)
        audit = json.loads((out / "audit.json").read_text(encoding="utf-8"))
        return result, audit, out, api, FakeEngine.instances[0]

    def test_success_calls_native_feature_source_before_quality_join_and_releases(self):
        result, audit, out, api, engine = self.run_mock()
        self.assertEqual(result, 0)
        self.assertTrue(audit["training_released"])
        self.assertTrue((out / "training.csv.gz").is_file())
        self.assertTrue(api.closed and engine.closed and engine.finalized)
        self.assertEqual([f[0] for f in engine.frames], [0., 1.])
        closed_features = engine.frames[-1][2]["feature_rows"]
        self.assertEqual(len(closed_features), 1)
        self.assertEqual(closed_features[0]["feature_valid"], 1)
        self.assertEqual(closed_features[0]["label_valid"], 0)
        self.assertEqual(closed_features[0]["observed_enter_time"], 0.)

    def test_missing_subscription_prevents_all_feature_and_training_publication(self):
        result, audit, out, api, engine = self.run_mock("missing_subscription")
        self.assertEqual(result, 2)
        self.assertFalse(audit["training_released"])
        self.assertIn("incomplete physical snapshot", audit["error"])
        self.assertEqual(engine.frames, [])
        self.assertFalse(engine.finalized)
        self.assertFalse((out / "training.csv.gz").exists())
        self.assertTrue(api.closed and engine.closed)

    def test_skipped_native_step_fails_run_before_quality_join(self):
        result, audit, out, _, engine = self.run_mock("skip_step")
        self.assertEqual(result, 2)
        self.assertIn("nonconsecutive", audit["error"])
        self.assertEqual(len(engine.frames), 1)
        self.assertFalse(engine.finalized)
        self.assertFalse((out / "training.csv.gz").exists())

    def test_collision_or_close_or_closed_file_failure_cannot_release_training(self):
        for failure in ("collision", "native_close", "stdout_close", "engine_close", "csv_audit", "hash_output"):
            with self.subTest(failure=failure):
                result, audit, out, _, _ = self.run_mock(failure)
                self.assertEqual(result, 2)
                self.assertFalse(audit["training_released"])
                self.assertFalse((out / "training.csv.gz").exists())
                self.assertTrue((out / "training.partial.csv.gz").exists())


if __name__ == "__main__":
    unittest.main()
