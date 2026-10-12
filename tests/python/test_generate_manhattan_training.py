"""Small deterministic generator tests; no SUMO or full dataset generation."""
from collections import Counter
import importlib.util
import json
from pathlib import Path
import shutil
import types
import unittest
from unittest import mock
import uuid
import xml.etree.ElementTree as ET


STAGE = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("manhattan_generator", STAGE / "data_generation/generate_manhattan_training.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def pool_fixture():
    groups = {"south", "central", "north", "single_lane", "short_edges"}
    groups.update(f"merge_{i}_{side}" for i in range(3) for side in ("a", "b"))
    return [{"route_id": f"r_{i:04d}", "edges": (f"a{i}", f"b{i}"),
             "length_m": (1000, 3000, 7000, 12000, 17000)[i % 5], "bin": i % 5,
             "frequency": 1, "groups": groups, "sources": ["fixture"], "quality_flags": {}}
            for i in range(250)]


def record(structure="B", profile="steady", family="family1", seed=123):
    factors = [1] * 6 if profile == "steady" else gen.WAVE
    return {"family_id": family, "structure": structure, "profile": profile,
            "planned_average_vehicles_per_hour": 60, "route_seed_proposal": seed,
            "time_seed_proposal": seed + 10000, "sumo_seed_proposal": seed + 20000,
            "scenario_id": f"fixture_{structure}_{profile}",
            "all_hourly": [36 + int(24 * factor) for factor in factors]}


class GeneratorTests(unittest.TestCase):
    def setUp(self):
        self.pool = pool_fixture()

    def test_transport_distributes_missing_local_bins_globally(self):
        result = gen.transport([2, 2], [1, 3], [[0, 2], [2, 2]])
        self.assertEqual([sum(row) for row in result], [2, 2])
        self.assertEqual([sum(row[b] for row in result) for b in range(2)], [1, 3])

    def test_transport_insufficient_support_fails(self):
        with self.assertRaisesRegex(ValueError, "infeasible"):
            gen.transport([2, 2], [2, 2], [[0, 1], [2, 2]])

    def test_all_structures_count_quotas_and_paired_no_replacement(self):
        for structure in "BSCNMG":
            with self.subTest(structure=structure):
                assignments, report = gen.build_family(self.pool, record(structure))
                self.assertEqual(len(assignments), 360)
                self.assertEqual(report["background_count"], 216)
                self.assertEqual(sum(report["target_groups"].values()), 144)
                self.assertLessEqual(max(report["target_groups"].values()) - min(report["target_groups"].values()), 1)
                for profile in ("steady", "wave"):
                    observed = {}
                    for a in assignments:
                        key = a["role"], a[profile + "_block"]
                        observed.setdefault(key, set())
                        self.assertNotIn(a["path_index"], observed[key])
                        observed[key].add(a["path_index"])

    def test_pair_keeps_identity_route_sf_and_background_time(self):
        assignments, _ = gen.build_family(self.pool, record())
        steady = {r["vehicle_id"]: r for r in gen.materialize(assignments, record())}
        wave = {r["vehicle_id"]: r for r in gen.materialize(assignments, record(profile="wave"))}
        self.assertEqual(set(steady), set(wave))
        for vid, a in steady.items():
            b = wave[vid]
            self.assertEqual((a["path_index"], a["speed_factor"]), (b["path_index"], b["speed_factor"]))
            if a["role"] == "background":
                self.assertEqual(a["depart_tick"], b["depart_tick"])
        self.assertTrue(any(steady[v]["depart_tick"] != wave[v]["depart_tick"] for v in steady))

    def test_exact_hour_counts_and_boundaries(self):
        assignments, _ = gen.build_family(self.pool, record())
        for profile in ("steady", "wave"):
            r = record(profile=profile)
            rows = gen.materialize(assignments, r)
            counts = Counter(a["depart_tick"] // 3600000000 for a in rows)
            self.assertEqual([counts[h] for h in range(6)], r["all_hourly"])
            self.assertTrue(all(0 <= a["depart_tick"] < 21600000000 for a in rows))
            self.assertEqual(rows, sorted(rows, key=lambda a: (a["depart_tick"], a["vehicle_id"])))

    def test_deterministic_and_independent_families(self):
        a, _ = gen.build_family(self.pool, record())
        b, _ = gen.build_family(self.pool, record())
        c, _ = gen.build_family(self.pool, record(family="validation_family"))
        self.assertEqual(a, b)
        self.assertFalse({x["vehicle_id"] for x in a} & {x["vehicle_id"] for x in c})
        self.assertNotEqual([x["path_index"] for x in a], [x["path_index"] for x in c])

    def test_zero_frequency_path_ineligible_for_neutral_not_deleted(self):
        self.pool[0]["frequency"] = 0
        assignments, _ = gen.build_family(self.pool, record())
        self.assertNotIn(0, [a["path_index"] for a in assignments])
        self.assertEqual(len(self.pool), 250)

    def test_sampler_no_silent_replacement(self):
        with self.assertRaisesRegex(ValueError, "without-replacement"):
            gen.weighted_sample([0], 2, gen.random.Random(1), self.pool, True)

    def test_missing_selector_support_raises(self):
        for p in self.pool:
            p["groups"] = set()
        with self.assertRaisesRegex(ValueError, "infeasible"):
            gen.build_family(self.pool, record("S"))

    def test_sf_reproducible_bounded_and_frozen_distribution(self):
        values = [gen.synthetic_sf("f", f"v{i}", 1) for i in range(1000)]
        self.assertEqual(values, [gen.synthetic_sf("f", f"v{i}", 1) for i in range(1000)])
        self.assertTrue(all(.2 <= float(x) <= 2 for x in values))
        self.assertTrue(all(len(x.split(".")[1]) == 12 for x in values))
        self.assertAlmostEqual(sum(map(float, values)) / len(values), 1., delta=.02)

    def test_xml_cfg_and_manifest_without_smallnet_or_lc_overrides(self):
        directory = STAGE / "tests" / ("fixture_generator_" + uuid.uuid4().hex)
        directory.mkdir()
        try:
            net = directory / "network.net.xml"
            net.write_text("<net/>", encoding="utf8")
            assignments, report = gen.build_family(self.pool, record())
            out = directory / "generated"
            hashes = {"network": {"path": str(net), "sha256": gen.sha(net)}}
            gen.write_scenario(out, self.pool, assignments, record(), hashes, report)
            xml = ET.parse(out / "routes.rou.xml").getroot()
            self.assertEqual(len(xml.findall("vehicle")), 360)
            self.assertEqual(xml.findall("vType"), [])
            for vehicle in xml.findall("vehicle"):
                self.assertEqual(set(vehicle.attrib), {"id", "depart", "speedFactor"})
                self.assertEqual(len(vehicle), 1)
                self.assertEqual(vehicle[0].tag, "route")
            cfg = ET.parse(out / "run.sumocfg").getroot()
            self.assertIsNone(cfg.find("time/end"))
            self.assertEqual(cfg.find("processing/time-to-teleport").get("value"), "-1")
            self.assertEqual(cfg.find("processing/collision.action").get("value"), "warn")
            self.assertEqual((out / cfg.find("input/net-file").get("value")).resolve(), net.resolve())
            self.assertFalse((out / "tripinfo.xml").exists())
            manifest = json.loads((out / "manifest.json").read_text(encoding="utf8"))
            self.assertEqual(manifest["status"], "DEMAND_GENERATED_NOT_SIMULATED")
            for name, item in manifest["outputs"].items():
                self.assertEqual(gen.sha(out / name), item["sha256"])
            with self.assertRaises(FileExistsError):
                gen.write_scenario(out, self.pool, assignments, record(), hashes, report)
        finally:
            # Explicitly confined synthetic fixture created by this test.
            if directory.parent == STAGE / "tests" and directory.name.startswith("fixture_generator_"):
                shutil.rmtree(directory)

    def test_final_assignment_audit_detects_mutation(self):
        assignments, _ = gen.build_family(self.pool, record())
        gen.audit_assignments(assignments, self.pool, record())
        assignments[1]["path_index"] = assignments[0]["path_index"]
        with self.assertRaisesRegex(ValueError, "reused"):
            gen.audit_assignments(assignments, self.pool, record())

    def test_dataset_manifest_batch_contract_and_preflight_no_xml(self):
        directory = STAGE / "tests" / ("fixture_generator_" + uuid.uuid4().hex)
        directory.mkdir()
        try:
            network = directory / "network.net.xml"
            network.write_text("<net/>", encoding="utf8")
            hashes = {"network": {"path": str(network), "sha256": gen.sha(network)}}
            records = [dict(record(profile=p), split="train", vehicle_count=360) for p in ("steady", "wave")]
            plan = {"training_files": records, "validation_files": []}
            for preflight in (True, False):
                out = directory / str(preflight)
                args = types.SimpleNamespace(output_dir=str(out), plan="ignored", pool_dir="ignored", descriptors="ignored",
                    structure_summary="ignored", network="ignored", scenario=None, preflight_only=preflight)
                with mock.patch.object(gen, "load_inputs", return_value=(plan, self.pool, hashes)):
                    self.assertEqual(gen.run(args), 0)
                if preflight:
                    self.assertEqual([p.name for p in out.iterdir()], ["preflight.json"])
                    continue
                manifest = json.loads((out / "manifest.json").read_text(encoding="utf8"))
                self.assertTrue(manifest["ready"])
                self.assertEqual(len(manifest["scenarios"]), 2)
                for scene in manifest["scenarios"]:
                    self.assertEqual(scene["split"], "train")
                    for path_key, hash_key in (("route_file", "route_sha256"), ("sumo_config", "config_sha256"),
                                               ("scenario_manifest", "scenario_manifest_sha256")):
                        self.assertEqual(gen.sha(out / scene[path_key]), scene[hash_key])
                    self.assertEqual(scene["network_sha256"], gen.sha(network))
        finally:
            if directory.parent == STAGE / "tests" and directory.name.startswith("fixture_generator_"):
                shutil.rmtree(directory)


if __name__ == "__main__":
    unittest.main()
