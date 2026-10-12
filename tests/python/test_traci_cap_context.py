import csv
import json
from pathlib import Path
from types import SimpleNamespace
import shutil
import unittest
import uuid

from features.cap_context_features import FEATURE_COLS, ContextConfig
from features.turn_context import TURN_CONTEXT_COLUMNS, TURN_CONTEXT_CONTRACT, TURN_CONTEXT_SCHEMA
from collection._shared.TraCI_Python_Cap_Context import CapContextObserver, RouteMetadata, file_sha256, write_feature_rows


class FakeTraCI:
    def __init__(self):
        self.rows, self.time, self.teleports = {}, 0.0, []
        self.vehicle = SimpleNamespace(
            getIDList=lambda: list(reversed(self.rows)), getRoadID=lambda vid: self.rows[vid][0],
            getRouteIndex=lambda vid: self.rows[vid][1], getRoute=lambda vid: ("a", "b", "c"),
            getTypeID=lambda vid: "car", getSpeedFactor=lambda vid: .5 if vid in ("leader", "alpha") else 1.,
            getMaxSpeed=lambda vid: 10., getLength=lambda vid: 5., getMinGap=lambda vid: 2.5)
        self.simulation = SimpleNamespace(getTime=lambda: self.time, getStartingTeleportIDList=lambda: self.teleports)

    def step(self, time, rows, teleports=()):
        self.time, self.rows, self.teleports = time, dict(rows), list(teleports)


class TraCICapContextTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parent / (".test_cap_context_" + uuid.uuid4().hex)
        self.root.mkdir()
        (self.root / "route.xml").write_text('<routes><vType id="car" length="5" maxSpeed="10"/>' +
            ''.join(f'<vehicle id="{vid}" type="car" departPos="base" speedFactor="{.5 if vid in ("leader", "alpha") else 1.}" />' for vid in ("leader", "ego", "alpha", "zulu")) + '</routes>', encoding="utf-8")
        (self.root / "config.xml").write_text('<configuration><input><route-files value="route.xml"/></input></configuration>', encoding="utf-8")
        (self.root / "net.xml").write_text('<net>' +
            ''.join(f'<edge id="{road}"><lane id="{road}_0" index="0" length="100" speed="10"/></edge>' for road in "abc") +
            '<connection from="a" to="b" fromLane="0" toLane="0"/><connection from="b" to="c" fromLane="0" toLane="0"/></net>', encoding="utf-8")
        self.api = FakeTraCI()
        self.observer = CapContextObserver(self.api, self.root / "config.xml", self.root / "net.xml")

    def tearDown(self):
        resolved = self.root.resolve()
        if resolved.parent != Path(__file__).resolve().parent or not resolved.name.startswith(".test_cap_context_"):
            raise RuntimeError("unsafe test cleanup target")
        shutil.rmtree(resolved)

    def test_freeze_at_stopline_not_future_true_entry(self):
        for time, rows in ((0, {"leader": ("a", 0)}),
                           (1, {"leader": ("a", 0), "ego": ("a", 0)}),
                           (2, {"leader": (":ab", 0), "ego": ("a", 0)}),
                           (3, {"leader": (":ab", 0), "ego": (":ab", 0)})):
            self.api.step(time, rows)
            self.observer.on_step()
        frozen = self.observer.frozen[("ego", 1)].copy()
        self.assertEqual(frozen["decision_time"], 3)
        self.assertEqual(frozen["planned_enter_time"], 6)
        self.assertEqual(frozen["pending_count"], 1)
        self.assertEqual(frozen["platoon_context"], 0)
        self.api.step(12, {"leader": ("b", 1), "ego": ("b", 1)})
        self.observer.on_step()
        event = SimpleNamespace(vehicle_id="ego", route_index=1, from_edge="b", speed_factor=1.)
        self.observer.attach(event)
        self.assertEqual(event.cap_context_snapshot, frozen)
        self.assertNotEqual(event.cap_context_snapshot["planned_enter_time"], 12)

    def test_same_step_order_is_stable_not_set_iteration(self):
        self.api.step(0, {"zulu": ("a", 0), "alpha": ("a", 0)})
        self.observer.on_step()
        self.assertEqual(self.observer.frozen[("alpha", 0)]["platoon_context"], 0)
        self.assertEqual(self.observer.frozen[("zulu", 0)]["platoon_context"], 2)

    def test_label_invalidity_does_not_remove_neighbor(self):
        # No training event/label is created for either vehicle in this test.
        self.api.step(0, {"leader": ("a", 0)})
        self.observer.on_step()
        self.api.step(1, {"leader": ("a", 0), "ego": ("a", 0)})
        self.observer.on_step()
        self.assertEqual(self.observer.frozen[("ego", 0)]["candidate_count"], 1)
        self.assertEqual(len(self.observer.ledger), 2)
        self.api.step(2, {"ego": ("a", 0)})
        self.observer.on_step()
        self.assertEqual(len(self.observer.ledger), 1)

    def test_teleport_removes_and_does_not_reinsert(self):
        self.api.step(0, {"leader": ("a", 0)})
        self.observer.on_step()
        self.api.step(1, {"leader": ("a", 0)}, teleports=("leader",))
        self.observer.on_step()
        self.assertEqual(len(self.observer.ledger), 0)
        self.api.step(2, {"leader": ("b", 1)})
        self.observer.on_step()
        self.assertEqual(len(self.observer.ledger), 0)

    def test_bad_departure_policy_rejected(self):
        text = (self.root / "route.xml").read_text(encoding="utf-8").replace('departPos="base"', 'departPos="random"')
        (self.root / "route.xml").write_text(text, encoding="utf-8")
        metadata = RouteMetadata(self.root / "config.xml")
        with self.assertRaises(ValueError):
            metadata.for_vehicle("ego", "car")

    def test_missing_or_mismatched_explicit_speed_factor_rejected(self):
        route_path = self.root / "route.xml"
        original = route_path.read_text(encoding="utf-8")
        route_path.write_text(original.replace(' speedFactor="1.0"', ''), encoding="utf-8")
        with self.assertRaises(ValueError):
            RouteMetadata(self.root / "config.xml").for_vehicle("ego", "car")
        route_path.write_text(original.replace('speedFactor="1.0"', 'speedFactor="0.9"'), encoding="utf-8")
        observer = CapContextObserver(self.api, self.root / "config.xml", self.root / "net.xml")
        self.api.step(0, {"ego": ("a", 0)})
        with self.assertRaises(ValueError):
            observer.on_step()

    def test_nonuniform_lane_geometry_rejected(self):
        net_path = self.root / "net.xml"
        text = net_path.read_text(encoding="utf-8").replace(
            '<lane id="a_0" index="0" length="100" speed="10"/>',
            '<lane id="a_0" index="0" length="100" speed="10"/><lane id="a_1" index="1" length="99" speed="10"/>')
        net_path.write_text(text, encoding="utf-8")
        with self.assertRaises(ValueError):
            CapContextObserver(self.api, self.root / "config.xml", net_path)

    def test_runtime_type_defaults_cannot_silently_diverge_from_cpp(self):
        self.api.vehicle.getMaxSpeed = lambda vid: 55.55
        self.api.step(0, {"ego": ("a", 0)})
        with self.assertRaisesRegex(ValueError, "SUMO/C\\+\\+ vType maxSpeed mismatch"):
            self.observer.on_step()

    def test_missing_initial_event_rejected(self):
        self.api.step(0, {"ego": ("b", 1)})
        with self.assertRaises(ValueError):
            self.observer.on_step()

    def test_outputs_exact_feature_schema_and_separate_audit(self):
        self.api.step(0, {"ego": ("a", 0)})
        self.observer.on_step()
        event = SimpleNamespace(vehicle_id="ego", route_index=0, from_edge="a", speed_factor=1.,
                                sampled_time=0., edge_enter_time=0., lane_index=0, is_final_road=False,
                                road_length=100., road_flow=1, lane_flow=1, driving_time=8., low_speed_time=1.,
                                route_edges=("a", "b", "c"), turn_type=3, turn_direction="r",
                                incoming_turn_type=0, incoming_turn_direction="unknown")
        self.observer.attach(event)
        label_cols = ["travel_time_label_gt_" + token for token in ("0p1", "0p5", "1", "1p5", "2", "3")]
        extras = {name: float(10 - index) for index, name in enumerate(label_cols)}
        extras.update(speed_time_le_0p1=1., speed_bin_time_total=10., speed_bin_conservation_residual=0.)
        sweep = SimpleNamespace(DEFAULT_LABEL_SPEED_THRESHOLDS=(.1, .5, 1, 1.5, 2, 3),
                                candidate_label_columns=lambda thresholds: label_cols,
                                speed_bin_columns=lambda thresholds: ["speed_time_le_0p1"],
                                _event_extra_values=lambda event: extras)
        base = SimpleNamespace(iter_valid_training_events=lambda events: iter(events), training_event_end_time=lambda event: 10.)
        path = self.root / "new.csv"
        self.assertEqual(write_feature_rows(path, [event], base=base, sweep=sweep, observer=self.observer, source={}), 1)
        with path.open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
            for forbidden in ("speed_factor", "speed_limit", "P", "E", "decision_time", *TURN_CONTEXT_COLUMNS):
                self.assertNotIn(forbidden, reader.fieldnames)
            self.assertEqual(reader.fieldnames,
                             ["time", "vehicleID", "roadID", "laneIndex", "route_index", "is_final_road"] +
                             FEATURE_COLS + ["travel_time_label"] + label_cols)
        self.assertEqual(float(rows[0][label_cols[0]]), 10)
        self.assertEqual(float(rows[0][label_cols[-1]]), 5)
        contract = json.loads(Path(str(path) + ".contract.json").read_text(encoding="utf-8"))
        self.assertEqual(contract["raw_features"], FEATURE_COLS)
        self.assertEqual(contract["data_sha256"], file_sha256(path))
        self.assertEqual(contract["turn_context_contract"], TURN_CONTEXT_CONTRACT)
        with Path(str(path) + ".audit.csv").open(encoding="utf-8", newline="") as stream:
            row = next(csv.DictReader(stream))
            self.assertEqual(float(row["speed_factor"]), 1.)
            self.assertEqual(float(row["speed_limit"]), 10.)
            self.assertEqual(row["turn_context_schema"], TURN_CONTEXT_SCHEMA)
            self.assertEqual((row["incoming_turn_type"], row["outgoing_turn_type"]), ("0", "3"))
            self.assertEqual((row["incoming_turn_present"], row["outgoing_turn_present"]), ("0", "1"))
            self.assertEqual(row["outgoing_turn_direction"], "r")


if __name__ == "__main__":
    unittest.main()
