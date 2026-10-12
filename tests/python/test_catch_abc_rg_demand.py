"""Tiny mocked closed-loop driver tests; no real model is trained or loaded."""
from contextlib import redirect_stdout
import csv
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import unittest
import uuid

from roadnet_paths import ROOT, source_path
sys.path.insert(0, str(ROOT))
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, mock_predictor
import catching.model_catching_sumo_cap_context_abc_rg as core

spec = importlib.util.spec_from_file_location("demand_driver", source_path("scripts/catch_abc_rg_demand.py"))
demand = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demand)


class DemandDriverTests(unittest.TestCase):
    def setUp(self):
        self.root = ROOT / (".test_demand_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup)
        self.model, self.net, _ = prepare_fixture(self.root / "fixture")
        self.binary = self.root / "fake_simulator.exe"
        self.binary.write_bytes(b"mock executable")
        self.route = self.root / "route.xml"
        self.route.write_text('<routes><route id="r" edges="a b"/><vehicle id="v" route="r"/></routes>')
        self.truth = self.root / "truth.xml"
        self.truth.write_text('<tripinfos><tripinfo id="v" duration="30"/></tripinfos>')
        self.output = self.root / "output"
        self.loads, self.commands = [], []

    def cleanup(self):
        target = self.root.resolve()
        if target.parent != ROOT.resolve() or not target.name.startswith(".test_demand_"):
            raise RuntimeError("Unsafe test cleanup")
        shutil.rmtree(target)

    def args(self, *extra):
        return demand.build_parser().parse_args([
            "--model-dir", str(self.model), "--expected-threshold", "1", "--net-file", str(self.net),
            "--simulator", str(self.binary), "--output-dir", str(self.output),
            "--scenario", "tiny", str(self.route), str(self.truth),
            "--signal-remaining-step", "5", "--road-flow-step", "5", "--cap-step", "1",
            "--max-road-flow", "16", "--max-lane-flow", "2", *extra])

    def load(self, path):
        self.loads.append(path)
        return mock_predictor()

    def fake_run(self, command, log):
        self.commands.append(command)
        option = lambda name: Path(command[command.index(name) + 1])
        cache = option("--travel-time-table")
        with cache.open("rb") as stream:
            self.assertEqual(stream.readline().decode().strip(), demand.MAGIC)
            header = json.loads(stream.readline())
            payload = stream.read()
        keys = {(group, road) for group, road, _ in demand.RECORD.iter_unpack(payload)}
        # The second query is not visited until the first miss has been fitted.
        missing = {(0, 0)} if not keys else ({(1, 1)} if (1, 1) not in keys else set())
        discovery = "--demand-requests-output" in command
        if missing and not discovery:
            return 3
        if discovery:
            with option("--demand-requests-output").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["group_index", "road_flow"])
                writer.writerows(sorted(missing))
        status = {"schema": "abc-rg-demand-status-v1", "scope_sha256": header["scope_sha256"],
                  "complete": True, "exact": not missing, "discovery_enabled": discovery,
                  "invalid_count": 0, "expected_queries": 2, "vehicle_count": 1,
                  "finished_count": 1, "query_count": 2, "missing_queries": len(missing),
                  "missing_unique": len(missing)}
        option("--demand-status-output").write_text(json.dumps(status))
        option("--cap-context-trace").write_text('sumoVehicleID,route_index,sumoRoadID\nv,0,a\nv,1,b\n')
        if not missing:
            option("--prediction-tripinfo-output").write_text('<tripinfos><tripinfo id="v" duration="31"/></tripinfos>')
            option("--eval-output").write_text("mock,not_used_for_metrics\n")
        Path(log).write_text("MOCK SIMULATOR\n")
        return 0

    def run_driver(self, *extra, runner=None, loader=None):
        with redirect_stdout(io.StringIO()):
            return demand.run(self.args(*extra), predictor_loader=loader or self.load,
                              runner=runner or self.fake_run)

    def test_zero_miss_then_strict_replay_and_independent_metrics(self):
        result = self.run_driver()
        self.assertEqual(len(self.loads), 1)
        self.assertEqual(result["discovery_rounds"], 3)
        self.assertEqual(result["entry_count"], 2)
        self.assertEqual(result["combined"], {"vehicle_count": 1, "mae_seconds": 1., "bias_seconds": 1.})
        self.assertNotIn("--demand-requests-output", self.commands[-1])
        self.assertEqual(self.commands[-1][self.commands[-1].index("--read-num") + 1], "1")
        self.assertFalse((self.output / "round_001" / "tiny" / "predictions.xml").exists())
        self.assertTrue((self.output / "summary.json").is_file())
        with (self.output / "round_001" / "predicted_nodes.csv").open(newline="") as stream:
            reader = csv.DictReader(stream)
            self.assertEqual(reader.fieldnames, ["group_index", *core.FEATURE_COLS, core.LABEL_COL])
            self.assertEqual(reader.fieldnames.count("road_flow"), 1)
            self.assertEqual(len(list(reader)), 1)

    def test_dry_run_no_model_or_output(self):
        result = self.run_driver("--dry-run")
        self.assertEqual(result["status"], "dry_run")
        self.assertFalse(self.output.exists())
        self.assertEqual(self.loads, [])
        self.assertEqual(self.commands, [])

    def test_round_limit_never_claims_metrics(self):
        with self.assertRaisesRegex(RuntimeError, "No zero-miss fixed point"):
            self.run_driver("--max-rounds", "1")
        self.assertFalse((self.output / "summary.json").exists())
        self.assertFalse(json.loads((self.output / "failure.json").read_text())["formal_results_accepted"])
        self.assertEqual(self.loads, [])

    def test_cache_guard_stops_before_exceeding_limit(self):
        with self.assertRaisesRegex(RuntimeError, "max-cache-entries"):
            self.run_driver("--max-cache-entries", "1")
        self.assertFalse((self.output / "summary.json").exists())

    def test_existing_output_is_never_overwritten(self):
        self.output.mkdir()
        marker = self.output / "user.txt"
        marker.write_text("keep")
        with self.assertRaisesRegex(ValueError, "Output exists"):
            self.run_driver()
        self.assertEqual(marker.read_text(), "keep")

    def test_output_cannot_be_inside_model_tree(self):
        with self.assertRaisesRegex(ValueError, "inside the model"):
            self.run_driver("--output-dir", str(self.model / "forbidden"))

    def test_nonfinite_prediction_refused(self):
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.run_driver(loader=lambda _path: mock_predictor(lambda frame: [float("nan")] * len(frame)))
        self.assertFalse((self.output / "summary.json").exists())

    def test_wrong_status_scope_refused(self):
        def runner(command, log):
            self.fake_run(command, log)
            path = Path(command[command.index("--demand-status-output") + 1])
            data = json.loads(path.read_text())
            data["scope_sha256"] = "0" * 64
            path.write_text(json.dumps(data))
            return 0
        with self.assertRaisesRegex(ValueError, "wrong-scope"):
            self.run_driver(runner=runner)

    def test_nonrepresentative_key_refused(self):
        with redirect_stdout(io.StringIO()):
            plan, _, _ = demand.prepare(self.args())
        with self.assertRaisesRegex(ValueError, "not a legal quantized"):
            demand.decode_key(plan, (0, 12))
        with self.assertRaises(ValueError):
            demand.decode_key(plan, (plan["group_count"], 0))

    def test_key_decoder_matches_full_node_order(self):
        with redirect_stdout(io.StringIO()):
            plan, _, _ = demand.prepare(self.args())
        decoder = demand.KeyDecoder(plan)
        expected = list(core.iter_prediction_cases(plan))
        actual = []
        for group in range(plan["group_count"]):
            lane = group % (plan["lane_flow_max"] + 1)
            for road in core.road_flow_nodes(lane, plan["road_flow_max"], 5, 10):
                actual.append(decoder((group, road)))
        self.assertEqual(actual, [item[0] for item in expected])

    def test_piecewise_dry_run_forwards_policy_and_logs_nodes(self):
        stream = io.StringIO()
        with redirect_stdout(stream):
            result = demand.run(self.args("--dry-run", "--road-flow-step", "1",
                                          "--road-flow-segments", "2:1", "10:5", "16:8"),
                                predictor_loader=self.load, runner=self.fake_run)
        policy = result["plan"]["road_flow_quantization"]
        self.assertEqual(policy["nodes"], [0, 1, 2, 5, 10, 16])
        self.assertIn("road_flow_grid=explicit road_flow_nodes=6", stream.getvalue())
        self.assertFalse(self.output.exists())
        self.assertEqual(self.loads, [])
        self.assertEqual(self.commands, [])

    def test_piecewise_decoder_matches_full_nodes_and_lane_lower_bound(self):
        with redirect_stdout(io.StringIO()):
            plan, _, _ = demand.prepare(self.args(
                "--road-flow-step", "1", "--max-lane-flow", "4",
                "--road-flow-segments", "2:1", "10:5", "16:8"))
        decoder = demand.KeyDecoder(plan)
        self.assertEqual(decoder((3, 3))[1:3], (3, 3))
        for key in ((0, 3), (3, 2), (3, 4)):
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "not a legal quantized"):
                decoder(key)
        actual = []
        for group in range(plan["group_count"]):
            lane = group % (plan["lane_flow_max"] + 1)
            for road in core.road_flow_nodes(lane, plan["road_flow_max"],
                                             nodes=plan["road_flow_quantization"]["nodes"]):
                actual.append(decoder((group, road)))
        self.assertEqual(actual, [item[0] for item in core.iter_prediction_cases(plan)])

    def test_piecewise_zero_miss_and_strict_replay(self):
        result = self.run_driver("--road-flow-step", "1", "--road-flow-segments", "2:1", "10:5", "16:8")
        self.assertEqual(result["entry_count"], 2)
        self.assertEqual(result["final_strict_missing_queries"], 0)
        self.assertEqual(result["combined"]["mae_seconds"], 1.)

    def test_piecewise_conflict_rejected_before_model_load_or_simulation(self):
        with self.assertRaises(ValueError):
            self.run_driver("--road-flow-segments", "16:2")
        self.assertEqual(self.loads, [])
        self.assertEqual(self.commands, [])
        self.assertFalse(self.output.exists())

    def test_scope_changes_with_model_bytes(self):
        with redirect_stdout(io.StringIO()):
            _, first, _ = demand.prepare(self.args())
            (self.model / "predictor" / "child.bin").write_bytes(b"model child")
            _, second, _ = demand.prepare(self.args())
        self.assertNotEqual(first["scope_sha256"], second["scope_sha256"])

    def test_baseline_configs_resolve_three_scenarios(self):
        directory = self.root / "configs"
        directory.mkdir()
        for name in ("free", "tight", "congested"):
            (directory / (name + ".sumocfg")).write_text(
                '<configuration><input><net-file value="../fixture/net.xml"/>'
                '<route-files value="../route.xml"/></input><output>'
                '<tripinfo-output value="../truth.xml"/></output></configuration>')
        cases, sources = demand.configured_scenarios(directory, core.abc.sha256_file(self.net))
        self.assertEqual([item[0] for item in cases], ["free", "tight", "congested"])
        self.assertEqual(len(sources), 3)
        self.assertEqual(cases[0][1:], (str(self.route.resolve()), str(self.truth.resolve())))
        with self.assertRaisesRegex(ValueError, "network differs"):
            demand.configured_scenarios(directory, "0" * 64)

    def test_status_flags_are_strict(self):
        mutations = [{"schema": "wrong"}, {"invalid_count": 1}, {"exact": True},
                     {"discovery_enabled": False}, {"query_count": 1}, {"complete": 1}]
        for index, mutation in enumerate(mutations):
            def runner(command, log):
                self.fake_run(command, log)
                path = Path(command[command.index("--demand-status-output") + 1])
                data = json.loads(path.read_text())
                data.update(mutation)
                path.write_text(json.dumps(data))
                return 0
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                self.run_driver("--output-dir", str(self.root / f"bad_status_{index}"), runner=runner)

    def test_truth_id_mismatch_fails_preflight(self):
        self.truth.write_text('<tripinfos><tripinfo id="wrong" duration="30"/></tripinfos>')
        with self.assertRaisesRegex(ValueError, "vehicle IDs differ"):
            self.run_driver()
        self.assertFalse(self.output.exists())

    def test_invalid_chunk_and_round_count(self):
        for name in ("--chunk-size", "--max-rounds", "--max-cache-entries", "--simulation-timeout"):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.run_driver(name, "0")

    def test_simulator_timeout_terminates_child(self):
        with self.assertRaisesRegex(RuntimeError, "simulation-timeout"):
            demand.run_process([sys.executable, "-c", "import time; time.sleep(10)"],
                               self.root / "timeout.log", timeout=.05)

    def test_final_identity_failure_refuses_summary(self):
        def runner(command, log):
            code = self.fake_run(command, log)
            if "--demand-requests-output" not in command:
                Path(command[command.index("--prediction-tripinfo-output") + 1]).write_text(
                    '<tripinfos><tripinfo id="wrong" duration="31"/></tripinfos>')
            return code
        with self.assertRaisesRegex(ValueError, "IDs do not match"):
            self.run_driver(runner=runner)
        self.assertFalse((self.output / "summary.json").exists())


if __name__ == "__main__":
    unittest.main()
