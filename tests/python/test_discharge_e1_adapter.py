"""Actual v2 adapter wiring with deterministic transport and real tiny SUMO."""
import importlib.util
from pathlib import Path
import shutil
import unittest

from collection import discharge_batch_sumo_v2 as repaired
from collection import discharge_batch_sumo as original
from features.discharge_e1_quality import SequenceExtractor
from features.discharge_sequences import SequenceExtractor as OldExtractor
from training.train_discharge_sequences import validate_sequence


def load_test(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(name + ".py"))
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


mock = load_test("test_discharge_batch_sumo")
fixture = load_test("test_discharge_sumo")


class V2TransportTests(mock.AdapterTests):
    def collect(self, module, *args, **kw):
        return super().collect(repaired if module is mock.adapter else module, *args, **kw)

    def test_safety_cap_is_failure_not_complete(self):
        metadata = {}
        connection = mock.FakeConnection(never_drain=True)
        with self.assertRaises(repaired.ObservationHorizonExceeded):
            self.collect(repaired, connection, metadata=metadata, end=4)
        self.assertFalse(metadata["completed"])
        self.assertTrue(connection.closed)

    def test_insertion_conflict_is_local_not_run_failure(self):
        connection = mock.FakeConnection()
        connection.vehicle.getIDList = lambda: ("a",) if 1 <= connection.step <= 5 else ()
        connection.vehicle.state = lambda _: ("in_0", 100.0, 0.0) if connection.step == 1 else ("out_0", 6.0, 6.0)
        connection.inductionloop.value = lambda d: (
            (("a", 5.0, float(connection.step - 1), -1.0, "car"),)
            if d == "detector_in_0" and connection.step <= 2 else ())
        events, meta, _ = self.collect(repaired, connection)
        engine = SequenceExtractor("same_run")
        for row in events:
            engine.feed(row)
        engine.finish()
        rows = [r for r in events if r["event"] == "passage"]
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[0]["vehicle_key"], rows[1]["vehicle_key"])
        self.assertTrue(all(r["quality"] == "unknown" for r in rows))
        self.assertEqual(engine.counts["quarantined_passage_time_conflicts"], 1)
        self.assertEqual(list(engine.store.iter_sequences()), [])
        self.assertTrue(meta["completed"])

    def test_real_loop_identity_remains_distinct_in_adapter(self):
        connection = mock.FakeConnection()
        connection.vehicle.getIDList = lambda: ("a",) if 1 <= connection.step <= 5 else ()
        frames = {1: ("in_0", 80.0, 10.0), 2: ("out_0", 10.0, 10.0),
                  3: ("in_0", 80.0, 10.0), 4: ("out_0", 10.0, 10.0), 5: ("out_0", 20.0, 10.0)}
        connection.vehicle.state = lambda _: frames[connection.step]
        connection.vehicle.getRoute = lambda _: ("in", "out", "in", "out")
        connection.vehicle.getRouteIndex = lambda _: min(connection.step - 1, 3)
        connection.inductionloop.value = lambda d: (
            (("a", 5.0, connection.step - .5, connection.step - .1, "car"),)
            if d == "detector_in_0" and connection.step in (2, 4) else ())
        events, _, _ = self.collect(repaired, connection)
        engine = SequenceExtractor("same_run")
        for row in events:
            engine.feed(row)
        rows = list(engine.store.iter_passages())
        self.assertEqual(len(rows), 2)
        self.assertNotEqual(rows[0]["vehicle_key"], rows[1]["vehicle_key"])
        self.assertTrue(all(r["quality"] == "observed" for r in rows))

    def test_same_lane_route_index_advance_uses_pre_crossing_visit(self):
        connection = mock.FakeConnection()
        connection.vehicle.getIDList = lambda: ("a",) if 1 <= connection.step <= 5 else ()
        frames = {1: ("in_0", 80.0, 10.0), 2: ("in_0", 10.0, 10.0),
                  3: ("in_0", 80.0, 10.0), 4: ("out_0", 10.0, 10.0), 5: ("out_0", 20.0, 10.0)}
        connection.vehicle.state = lambda _: frames[connection.step]
        connection.vehicle.getRoute = lambda _: ("in", "in", "out")
        connection.vehicle.getRouteIndex = lambda _: {1: 0, 2: 1, 3: 1, 4: 2, 5: 2}[connection.step]
        connection.inductionloop.value = lambda d: (
            (("a", 5.0, connection.step - .5, connection.step - .1, "car"),)
            if d == "detector_in_0" and connection.step in (2, 4) else ())
        events, _, _ = self.collect(repaired, connection)
        rows = [r for r in events if r["event"] == "passage"]
        self.assertEqual([r["vehicle_key"] for r in rows], ["a|visit=1|route=0", "a|visit=2|route=1"])


@unittest.skipUnless(shutil.which("sumo") and shutil.which("netconvert"), "SUMO unavailable")
class LiveParityTests(unittest.TestCase):
    def test_actual_sumo_vehicle_can_repeat_same_road_without_false_quarantine(self):
        import subprocess
        with fixture.fixture_directory() as temporary:
            root = Path(temporary)
            (root / "nodes.xml").write_text(
                '<nodes><node id="a" x="0" y="0"/><node id="b" x="100" y="0"/>'
                '<node id="c" x="100" y="100"/><node id="d" x="0" y="100"/></nodes>')
            (root / "edges.xml").write_text(
                '<edges><edge id="ab" from="a" to="b" speed="10"/><edge id="bc" from="b" to="c" speed="10"/>'
                '<edge id="cd" from="c" to="d" speed="10"/><edge id="da" from="d" to="a" speed="10"/></edges>')
            subprocess.run([shutil.which("netconvert"), "-n", str(root / "nodes.xml"), "-e", str(root / "edges.xml"),
                            "--no-internal-links", "true", "-o", str(root / "net.xml")],
                           capture_output=True, text=True, check=True, timeout=30)
            (root / "routes.xml").write_text(
                '<routes><vType id="car" sigma="0" speedFactor="1" speedDev="0"/>'
                '<vehicle id="v" type="car" depart="0"><route edges="ab bc cd da ab bc"/></vehicle></routes>')
            metadata = {}
            engine = SequenceExtractor("loop")
            for row in repaired.iter_sumo_events(dict(net=str(root / "net.xml"), routes=[str(root / "routes.xml")],
                output_dir=root / "observations", end=120, seed=17, step_length=1, run_id="loop", metadata=metadata)):
                engine.feed(row)
            engine.finish()
            ab = [r for r in engine.store.iter_passages() if r["lane_id"] == "ab_0"]
            self.assertTrue(metadata["completed"])
            self.assertEqual(len(ab), 2)
            self.assertNotEqual(ab[0]["vehicle_key"], ab[1]["vehicle_key"])
            self.assertTrue(all(r["quality"] == "observed" for r in ab))
            self.assertEqual(engine.counts["quarantined_passage_time_conflicts"], 0)

    def test_real_queued_start_labels_and_tripinfo_unchanged(self):
        import xml.etree.ElementTree as ET
        for blocked in (False, True):
            with self.subTest(blocked=blocked), fixture.fixture_directory() as temporary:
                root = Path(temporary)
                net, routes = fixture.make_fixture(root, blocked_lane=blocked)
                results, trips = [], []
                for name, module, engine_type in (("old", original, OldExtractor),
                                                   ("new", repaired, SequenceExtractor)):
                    metadata = {}
                    engine = engine_type("parity")
                    config = dict(net=str(net), routes=[str(routes)], output_dir=root / name,
                                  end=130, seed=17, step_length=.1, run_id="parity", metadata=metadata)
                    for row in module.iter_sumo_events(config):
                        engine.feed(row)
                    engine.finish()
                    rows = list(engine.store.iter_sequences())
                    for row in rows:
                        validate_sequence(row, "parity")
                    results.append(rows)
                    trips.append({t.attrib["id"]: t.attrib for t in ET.parse(root / name / "tripinfo.xml").getroot().findall("tripinfo")})
                    self.assertTrue(metadata["completed"])
                self.assertGreaterEqual(sum(len(r["headways"]) for r in results[0]), 3)
                self.assertEqual(results[0], results[1])
                self.assertEqual(trips[0], trips[1])


if __name__ == "__main__":
    unittest.main()
