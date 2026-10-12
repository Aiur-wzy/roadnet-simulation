"""Pure native catcher tests: no AutoGluon load, SUMO, server, or real replay."""
from __future__ import annotations

import argparse
import copy
import csv
import json
from pathlib import Path
import struct
import tempfile
import unittest

from catching import catch_road_time as catch
from training import native_contract as contract


NETWORK = '''<net>
<edge id="a"><lane id="a_0" index="0" length="7.4" speed="13.89"/>
<lane id="a_1" index="1" length="7.4" speed="13.89"/>
<lane id="a_2" index="2" length="7.4" speed="13.89"/></edge>
<edge id="b"><lane id="b_0" index="0" length="20" speed="8.94"/></edge>
<tlLogic id="t" type="static" programID="0" offset="0">
<phase duration="5" state="G"/><phase duration="2" state="y"/>
<phase duration="5" state="r"/><phase duration="2" state="y"/></tlLogic>
<connection from="a" to="b" fromLane="0" toLane="0" tl="t" linkIndex="0"/>
<connection from="b" to="a" fromLane="0" toLane="0"/>
</net>'''


def fake_model(root, threshold="1"):
    directory = Path(root) / ("model_" + threshold)
    predictor = directory / "predictor"
    predictor.mkdir(parents=True)
    (predictor / "predictor.pkl").write_bytes(b"FAKE-NOT-A-PICKLE")
    ranges = {}
    for name in contract.FEATURE_COLUMNS:
        if name in contract.CATEGORICAL_FEATURES:
            categories = contract.CATEGORICAL_FEATURES[name]
            ranges[name] = {"categories": categories, "observed": categories}
        else:
            ranges[name] = {"min": .01 if name in ("road_length", "free_speed_cap") else 0., "max": 2e9}
    metrics_path = directory / "validation_metrics.json"
    metrics_path.write_text("{}", encoding="utf-8")
    value = {
        "schema": contract.MODEL_SCHEMA, "status": "COMPLETE", "source_schema": contract.SOURCE_SCHEMA,
        "source_label_column": "travel_time_label_gt_" + threshold, "feature_columns": contract.FEATURE_COLUMNS,
        "categorical": contract.CATEGORICAL_FEATURES, "model_label": contract.MODEL_LABEL,
        "predictor_directory": "predictor", "collection_contract": copy.deepcopy(contract.COLLECTION_SEMANTICS),
        "deployment_contract": contract.deployment_contract(), "feature_ranges": ranges,
        "predictor_files_sha256": {"predictor.pkl": contract.sha256_file(predictor / "predictor.pkl")},
        "validation_metrics_sha256": contract.sha256_file(metrics_path),
        "framework_raw_features": list(contract.FEATURE_COLUMNS), "framework_removed_raw_features": [],
    }
    (directory / "model_contract.json").write_text(json.dumps(value), encoding="utf-8")
    return directory


def fixture_args(root):
    root = Path(root)
    network = root / "tiny.net.xml"
    network.write_text(NETWORK, encoding="utf-8")
    model = fake_model(root)
    return catch.build_parser().parse_args([
        "--model-dir", str(model), "--net-file", str(network), "--output-dir", str(root / "result"),
        "--signal-remaining-step", "2.5", "--cap-step", "1", "--cap-min", "1", "--cap-max", "3",
        "--max-road-flow", "20", "--max-lane-flow", "3", "--road-flow-step", "5", "--text-format", "both",
    ])


class FakePredictor:
    label = contract.MODEL_LABEL

    class Metadata:
        def get_features(self):
            return list(contract.FEATURE_COLUMNS)

    feature_metadata_in = Metadata()
    feature_metadata = Metadata()

    def predict(self, frame):
        assert list(frame.columns) == contract.FEATURE_COLUMNS
        assert str(frame.platoon_context.dtype) == "category"
        return [float(length) / float(cap) for length, cap in zip(frame.road_length, frame.free_speed_cap)]


class NativeCatchingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.args = fixture_args(self.root)

    def plan(self):
        return catch.build_plan(self.args)

    def test_no_historical_imports(self):
        import ast
        tree = ast.parse(Path(catch.__file__).read_text(encoding="utf-8"))
        imported = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
        imported += [alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names]
        self.assertFalse(any(name and ("_shared" in name or "legacy" in name or name.startswith("features.")
                                     or name.startswith("collection.") or "model_catching" in name) for name in imported))

    def test_dry_run_never_loads_or_writes(self):
        self.args.dry_run = True
        result = catch.run(self.args, predictor_loader=lambda _: self.fail("loaded predictor"),
                           runner=lambda *_: self.fail("ran simulator"))
        self.assertEqual(result["status"], "dry_run")
        self.assertFalse(Path(self.args.output_dir).exists())
        self.assertEqual(len(result["plan_sha256"]), 64)
        self.assertFalse(result["plan"]["deployment_contract"]["equivalent"])

    def test_native_model_required(self):
        path = Path(self.args.model_dir) / "model_contract.json"
        value = catch.read_json(path)
        value["schema"] = "road-time-cap-context-abc-rg-v1"
        path.write_text(json.dumps(value), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "schema"):
            self.plan()

    def test_actual_lane_length_not_float_mean(self):
        lengths, _, network = catch.network_catalog(self.args.net_file, 2.5)
        self.assertEqual(lengths, [7.4, 20.])
        self.assertEqual(network["external_edges"], 2)
        self.assertTrue(network["without_internal_links"])

    def test_physical_lane_mismatch_rejected(self):
        Path(self.args.net_file).write_text(NETWORK.replace('id="a_2" index="2" length="7.4"',
                                                           'id="a_2" index="2" length="7.5"'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "equal physical"):
            self.plan()

    def test_physical_limit_mismatch_rejected(self):
        Path(self.args.net_file).write_text(NETWORK.replace('id="a_2" index="2" length="7.4" speed="13.89"',
                                                           'id="a_2" index="2" length="7.4" speed="13.90"'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "equal physical"):
            self.plan()

    def test_nonrepresentable_physical_length_is_not_silently_rounded(self):
        Path(self.args.net_file).write_text(NETWORK.replace('length="7.4"', 'length="7.400123"'), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "four-decimal"):
            self.plan()

    def test_cycle_boundary_merges_and_yellow_separates(self):
        self.assertEqual(catch.cyclic_segments([2, 5, 3], ["G", "R", "g"]), [("G", 5.), ("R", 5.)])
        self.assertEqual(catch.cyclic_segments([2, 1, 3, 4], ["G", "y", "g", "R"]),
                         [("G", 2.), ("Y", 1.), ("G", 3.), ("R", 4.)])
        with self.assertRaises(ValueError):
            catch.cyclic_segments([2, 3], ["G", "G"])

    def test_static_nonsequential_and_unknown_links_rejected(self):
        for bad in (NETWORK.replace('type="static"', 'type="actuated"'),
                    NETWORK.replace('<phase duration="5" state="G"/>', '<phase duration="5" state="G" next="2"/>'),
                    NETWORK.replace('linkIndex="0"', 'linkIndex="1"')):
            Path(self.args.net_file).write_text(bad, encoding="utf-8")
            with self.assertRaises(ValueError):
                self.plan()

    def test_signal_and_flow_and_cap_ties(self):
        codec = catch.KeyCodec(self.plan())
        key = codec.encode((7.4, 12, 1, 1.5, 4, "G", 3.75, "R", 5.))
        self.assertEqual(codec.decode(key), (7.4, 10, 1, 2., 4, "G", 5., "R", 5.))
        self.assertEqual(codec.decode(codec.encode((7.4, 13, 1, 1., 0, "G", 0., "G", 0.)))[1], 15)
        self.assertEqual(catch.nearest_index([10, 14], 12), 1)

    def test_preserves_lane_and_road_flow_endpoints(self):
        policy = catch.flow_policy(5, 10)
        self.assertEqual(catch.flow_nodes(13, 22, policy), [13, 15, 20, 22])
        policy = catch.flow_policy(segments=["10:1", "20:5"], maximum=20)
        self.assertEqual(catch.flow_nodes(13, 20, policy), [13, 15, 20])

    def test_o1_flow_mapping_matches_enumerated_small_grids(self):
        for maximum in (0, 1, 20, 22):
            for step in (1, 2, 5, 99):
                for dense in (0, 10, 30):
                    policy = catch.flow_policy(step, dense)
                    for lane in range(maximum + 1):
                        nodes = catch.flow_nodes(lane, maximum, policy)
                        for road in range(lane, maximum + 1):
                            expected = nodes[catch.nearest_index(nodes, road)]
                            self.assertEqual(catch.representative_road_flow(road, lane, maximum, policy), expected)

    def test_signed_int_flow_domain_has_no_large_per_lane_allocations(self):
        self.args.max_road_flow = 2**31 - 1
        self.args.max_lane_flow = 3
        codec = catch.KeyCodec(self.plan())
        key = codec.encode((7.4, 2**31 - 1, 3, 1., 0, "G", 0., "G", 0.))
        self.assertEqual(codec.decode(key)[1], 2**31 - 1)
        with self.assertRaisesRegex(ValueError, "budget"):
            catch.flow_policy(segments=["2147483647:1"], maximum=2**31 - 1)

    def test_out_of_domain_no_clamp(self):
        self.args.cap_out_of_range = "error"
        codec = catch.KeyCodec(self.plan())
        rows = [(7.4, 21, 1, 2., 0, "G", 0., "G", 0.),
                (7.4, 2, 3, 2., 0, "G", 0., "G", 0.),
                (7.4, 2, 1, 4., 0, "G", 0., "G", 0.),
                (7.4, 2, 1, 2., 0, "G", 5.1, "R", 5.),
                (7.4, 2, 1, 2., 0, "G", 0., "R", 5.),
                (7.4, 2, 1, 2., 0, "G", 2., "R", 6.)]
        for row in rows:
            with self.assertRaises(ValueError):
                codec.encode(row)

    def test_default_clamps_only_positive_finite_cap(self):
        codec = catch.KeyCodec(self.plan())
        for raw, expected in ((.01, 1.), (1., 1.), (3., 3.), (900., 3.)):
            row = (7.4, 2, 1, raw, 0, "G", 0., "G", 0.)
            self.assertEqual(codec.decode(codec.encode(row))[3], expected)
            self.assertEqual(row[3], raw)
        for invalid in (0., -.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                codec.encode((7.4, 2, 1, invalid, 0, "G", 0., "G", 0.))
        for row in ((7.4, 21, 1, 2., 0, "G", 0., "G", 0.),
                    (7.4, 4, 4, 2., 0, "G", 0., "G", 0.),
                    (7.4, 1, 2, 2., 0, "G", 0., "G", 0.),
                    (7.4, 2, 1, 2., 0, "G", 5.1, "R", 5.),
                    (7.4, 2, 1, 2., 6, "G", 0., "G", 0.),
                    (7.4, 2, 1, 2., 0, "Y", 1., "R", 5.),
                    (7.4, 2, 1, 2., 0, "G", 1., "R", 6.),
                    (8., 2, 1, 2., 0, "G", 0., "G", 0.)):
            with self.assertRaises(ValueError):
                codec.encode(row)

    def test_manhattan_observed_cap_clamps_to_grid_not_training_minimum(self):
        self.args.cap_step = .1389
        self.args.cap_min = 1.19681416237563
        self.args.cap_max = 55.55555555555556
        codec = catch.KeyCodec(self.plan())
        raw = 1.0874801041404398
        decoded = codec.decode(codec.encode((7.4, 2, 1, raw, 0, "G", 0., "G", 0.)))
        self.assertAlmostEqual(decoded[3], 1.1112)

    def test_query_policies_are_scope_bound_and_alias_supported(self):
        original = self.plan()
        self.assertEqual(original["deployment_contract"]["first_road_position_correction"], "on")
        self.assertEqual(original["deployment_contract"]["cap_out_of_range"], "clamp")
        self.args.first_road_position_correction = "off"
        off = self.plan()
        self.assertNotEqual(catch.canonical_sha(original), catch.canonical_sha(off))
        self.args.cap_out_of_range = "error"
        self.assertNotEqual(catch.canonical_sha(off), catch.canonical_sha(self.plan()))
        parsed = catch.build_parser().parse_args([
            "--model-dir", "x", "--net-file", "x", "--output-dir", "x",
            "--signal-remaining-step", "5", "--speed-net-depart-position", "off"])
        self.assertEqual(parsed.first_road_position_correction, "off")

    def test_correction_cannot_be_overridden_via_extra_arguments(self):
        for option in ("--first-road-position-correction", "--speed-net-depart-position"):
            self.args.simulator_arg = [option, "off"]
            with self.assertRaisesRegex(ValueError, "override frozen"):
                catch.run(self.args, predictor_loader=lambda _: self.fail("loaded predictor"))

    def test_greater_than_u32_group_is_sparse(self):
        self.args.cap_max = 1000000001.
        plan = self.plan()
        self.assertGreater(plan["expanded_rows"], 4_000_000_000)
        codec = catch.KeyCodec(plan)
        key = codec.encode((20., 15, 3, 1000000001., 5, "R", 5., "G", 5.))
        self.assertGreater(key[0], 2**32)
        path = self.root / "sparse.demand"
        header = catch.write_cache(path, plan, "0" * 64, {key: 9.})
        self.assertEqual(header["payload_bytes"], 20)
        self.assertEqual(catch.read_cache(path, plan, "0" * 64)[1], {key: 9.})

    def test_payload_roundtrip_floor_and_corruption(self):
        plan = self.plan()
        codec = catch.KeyCodec(plan)
        key = codec.encode((7.4, 0, 0, 1., 0, "G", 0., "G", 0.))
        with self.assertRaisesRegex(ValueError, "floor"):
            catch.write_cache(self.root / "invalid", plan, "0" * 64, {key: .5})
        path = self.root / "cache"
        catch.write_cache(path, plan, "0" * 64, {key: 3.})
        self.assertEqual(catch.read_cache(path, plan, "0" * 64)[1], {key: 3.})
        with path.open("ab") as stream:
            stream.write(b"trailing")
        with self.assertRaisesRegex(ValueError, "Trailing"):
            catch.read_cache(path, plan)

    def test_request_scope_binding_and_predictor_only_on_demand(self):
        plan = self.plan()
        path = self.root / "requests.csv"
        path.write_text("group_index,road_flow\n0,0\n", encoding="utf-8")
        self.args.requests = str(path)
        with self.assertRaisesRegex(ValueError, "requests-plan-sha256"):
            catch.run(self.args, predictor_loader=lambda _: self.fail("loaded predictor"))
        self.args.requests_plan_sha256 = catch.canonical_sha(plan)
        result = catch.run(self.args, predictor_loader=lambda _: FakePredictor())
        self.assertEqual(result["entry_count"], 1)
        self.assertEqual(result["result_role"], "request_scoped_table_not_closed_loop_scores")
        self.assertTrue((Path(self.args.output_dir) / "predicted_nodes.txt").is_file())
        self.assertNotIn("combined", result)

    def test_request_memory_budget_and_illegal_representative(self):
        path = self.root / "requests.csv"
        path.write_text("group_index,road_flow\n0,11\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "representative"):
            catch.read_requests(path, self.plan())
        path.write_text("group_index,road_flow\n0,0\n1,1\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "memory budget"):
            catch.read_requests(path, self.plan(), 1)

    def test_predicted_nonfinite_and_negative_policy(self):
        class Negative(FakePredictor):
            def predict(self, frame):
                return [-.2] * len(frame)
        self.assertEqual(catch.predict_keys(Negative(), self.plan(), [(0, 0)], 10), {(0, 0): 1.})
        class Nonfinite(FakePredictor):
            def predict(self, frame):
                return [float("nan")] * len(frame)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            catch.predict_keys(Nonfinite(), self.plan(), [(0, 0)], 10)
        class WrongShape(FakePredictor):
            def predict(self, frame):
                return [[1.]] * len(frame)
        with self.assertRaisesRegex(ValueError, "shape"):
            catch.predict_keys(WrongShape(), self.plan(), [(0, 0)], 10)

    def test_pruned_framework_features_and_injected_features(self):
        plan = self.plan()
        plan["model_contract"]["framework_raw_features"] = ["road_length"]
        plan["model_contract"]["framework_removed_raw_features"] = [name for name in contract.FEATURE_COLUMNS if name != "road_length"]
        class Pruned(FakePredictor):
            class Metadata:
                def get_features(self):
                    return ["road_length"]
            feature_metadata = Metadata()
        self.assertEqual(len(catch.predict_keys(Pruned(), plan, [(0, 0)], 10)), 1)
        with self.assertRaisesRegex(ValueError, "raw features"):
            catch.predict_keys(FakePredictor(), plan, [(0, 0)], 10)

    def test_predictor_artifact_tampering_rejected_before_load(self):
        path = Path(self.args.model_dir) / "predictor" / "predictor.pkl"
        path.write_bytes(b"TAMPERED")
        with self.assertRaisesRegex(ValueError, "hashes"):
            self.plan()

    def test_model_mutation_during_load_never_publishes(self):
        plan = self.plan()
        request = self.root / "requests.csv"
        request.write_text("group_index,road_flow\n0,0\n", encoding="utf-8")
        self.args.requests, self.args.requests_plan_sha256 = str(request), catch.canonical_sha(plan)
        def loader(_):
            (Path(self.args.model_dir) / "unexpected_file").write_text("injected", encoding="utf-8")
            return FakePredictor()
        with self.assertRaisesRegex(ValueError, "file set"):
            catch.run(self.args, predictor_loader=loader)
        self.assertFalse((Path(self.args.output_dir) / "summary.json").exists())

    def _closed_loop(self, strict_missing=False):
        route = self.root / "route.xml"
        truth = self.root / "truth.xml"
        binary = self.root / "fake_simulator"
        route.write_text('<routes><vehicle id="v"><route edges="a b"/></vehicle></routes>', encoding="utf-8")
        truth.write_text('<tripinfos><tripinfo id="v" duration="10"/></tripinfos>', encoding="utf-8")
        binary.write_bytes(b"FAKE-NOT-EXECUTABLE")
        self.args.scenario = [["toy", str(route), str(truth)]]
        self.args.simulator = str(binary)
        calls = []
        def runner(command, log, timeout):
            directory = Path(log).parent
            header, values = catch.read_cache(command[command.index("--travel-time-table") + 1])
            discovery = "--demand-requests-output" in command
            missing = 0 if values else 1
            if not discovery and strict_missing:
                missing = 1
            calls.append(discovery)
            catch.write_json(directory / "status.json", {
                "schema": "native-road-time-demand-status-v2", "scope_sha256": header["scope_sha256"],
                "complete": True, "discovery_enabled": discovery, "vehicle_count": 1, "finished_count": 1,
                "query_count": 2, "expected_queries": 2, "invalid_count": 0,
                "missing_queries": missing, "missing_unique": missing, "exact": missing == 0,
            })
            if discovery:
                (directory / "missing.csv").write_text("group_index,road_flow\n" + ("0,0\n" if missing else ""), encoding="utf-8")
            else:
                (directory / "predictions.xml").write_text('<tripinfos><tripinfo id="v" duration="12"/></tripinfos>', encoding="utf-8")
                (directory / "trace.csv").write_text("sumoVehicleID,route_index,sumoRoadID\nv,0,a\nv,1,b\n", encoding="utf-8")
            return 0
        return calls, runner

    def test_closed_loop_requires_separate_strict_replay(self):
        calls, runner = self._closed_loop()
        result = catch.run(self.args, predictor_loader=lambda _: FakePredictor(), runner=runner)
        self.assertEqual(calls, [True, True, False])
        self.assertEqual(result["combined"]["mae_seconds"], 2.)
        self.assertEqual(result["result_role"], "strict_scenario_replay")
        self.assertEqual(result["final_strict_missing_queries"], 0)

    def test_strict_missing_never_publishes_scores(self):
        _, runner = self._closed_loop(strict_missing=True)
        with self.assertRaisesRegex(ValueError, "Strict native replay"):
            catch.run(self.args, predictor_loader=lambda _: FakePredictor(), runner=runner)
        directory = Path(self.args.output_dir)
        self.assertFalse((directory / "summary.json").exists())
        self.assertFalse(catch.read_json(directory / "failure.json")["formal_results_accepted"])

    def test_cache_mutation_during_replay_never_publishes(self):
        _, delegate = self._closed_loop()
        def runner(command, log, timeout):
            result = delegate(command, log, timeout)
            cache = Path(command[command.index("--travel-time-table") + 1])
            with cache.open("ab") as stream:
                stream.write(b"corruption")
            return result
        with self.assertRaisesRegex(ValueError, "Source changed"):
            catch.run(self.args, predictor_loader=lambda _: FakePredictor(), runner=runner)
        self.assertFalse((Path(self.args.output_dir) / "summary.json").exists())

    def test_no_fixed_point_never_publishes_scores(self):
        _, runner = self._closed_loop()
        self.args.max_rounds = 1
        with self.assertRaisesRegex(RuntimeError, "fixed point"):
            catch.run(self.args, predictor_loader=lambda _: self.fail("loaded predictor"), runner=runner)
        self.assertFalse((Path(self.args.output_dir) / "summary.json").exists())

    def test_current_runtime_modes_are_explicit(self):
        _, runner = self._closed_loop()
        self.args.first_road_position_correction = "off"
        scenario = catch.prepare_scenarios(self.args)[0]
        command = catch.command_for(self.args, scenario, Path("cache"), Path("out"), False)
        for name, wanted in (("--movement-grouping", "road-pair"), ("--lane-entry-reselection", "on"),
                             ("--lane-borrowing", "off"), ("--lane-discharge-interval", "2.0"),
                             ("--first-road-position-correction", "off")):
            self.assertEqual(command[command.index(name) + 1], wanted)

    def test_interleaved_trace_checks_per_vehicle_order_and_coverage(self):
        directory = self.root / "trace_test"
        directory.mkdir()
        (directory / "predictions.xml").write_text('<tripinfos><tripinfo id="v" duration="10"/><tripinfo id="w" duration="11"/></tripinfos>', encoding="utf-8")
        scenario = {"name": "interleaved", "vehicles": {"v": ("a", "b"), "w": ("b", "a")},
                    "truth_durations": {"v": 10., "w": 11.}, "road_count": 4}
        trace = directory / "trace.csv"
        header = "sumoVehicleID,route_index,sumoRoadID\n"
        trace.write_text(header + "v,0,a\nw,0,b\nv,1,b\nw,1,a\n", encoding="utf-8")
        self.assertEqual(catch.check_final(directory, scenario)["road_count"], 4)
        for rows in ("v,0,a\nv,0,a\n", "v,1,b\nv,0,a\n", "v,0,a\nw,0,b\nv,1,b\n"):
            trace.write_text(header + rows, encoding="utf-8")
            with self.assertRaises(ValueError):
                catch.check_final(directory, scenario)

    def test_route_iterparse_resolves_shared_definitions_and_rejects_partial(self):
        path = self.root / "route.xml"
        path.write_text('<routes><route id="r" edges="a b a"/><vehicle id="v" route="r"/><vehicle id="w" route="r"/></routes>', encoding="utf-8")
        routes = catch.route_vehicles(path)
        self.assertIs(routes["v"], routes["w"])
        self.assertEqual(routes["v"], ("a", "b", "a"))
        path.write_text('<routes><vehicle id="v" arrivalEdge="0"><route edges="a b"/></vehicle></routes>', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Partial"):
            catch.route_vehicles(path)

    def test_manhattan_bounded_static_plan(self):
        from roadnet_paths import ROOT
        network = Path(ROOT) / "baselines" / "ICDE2025" / "manhattan" / "network.net.xml"
        if not network.exists():
            self.skipTest("Manhattan static network absent")
        self.args.net_file = str(network)
        self.args.signal_remaining_step = 1.
        self.args.cap_min = self.args.cap_max = 1.
        self.args.max_road_flow, self.args.max_lane_flow = 222, 80
        plan = self.plan()
        self.assertEqual(len(plan["road_lengths"]), 8313)
        self.assertEqual(plan["signal_point_count"], 2206)
        self.assertEqual(plan["group_count"], 8912500308)
        self.assertEqual(plan["expanded_rows"], 1630987556364)
        self.assertLess(len(catch.encoded_json(plan)), catch.MAX_HEADER)


if __name__ == "__main__":
    unittest.main()
