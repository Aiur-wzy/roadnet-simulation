"""Pure fixture/fake-TraCI tests for explicit speed-factor materialization."""
from __future__ import annotations

import contextlib
import copy
import csv
import importlib.util
import io
import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import patch
import uuid
import xml.etree.ElementTree as ET


from roadnet_paths import source_path

MODULE_PATH = source_path("scripts/materialize_speed_factors.py")
SPEC = importlib.util.spec_from_file_location("materialize_speed_factors_under_test", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class MaterializeFixture(unittest.TestCase):
    def setUp(self):
        fixture_parent = Path(__file__).resolve().parent
        # Python 3.12's Windows mkdtemp mode=0700 can exclude the sandbox token.
        self.root = fixture_parent / ("sf_materialize_fixture_" + uuid.uuid4().hex)
        self.root.mkdir()
        def cleanup_fixture():
            resolved = self.root.resolve()
            if resolved.parent != fixture_parent or not resolved.name.startswith("sf_materialize_fixture_"):
                raise RuntimeError("Refusing cleanup outside the exact test fixture directory")
            shutil.rmtree(resolved)
        self.addCleanup(cleanup_fixture)
        self.source = self.root / "source"
        self.source.mkdir()
        self.net = self.source / "net.xml"
        self.net.write_text("<net/>\n", encoding="utf-8")
        self.route = self.source / "route.xml"
        self.route.write_text(
            '<routes><!-- preserved --><vType id="car" speedFactor="normc(1,0.1,0.2,2)"/>'
            '<route id="r" edges="a b"/><vehicle id="v0" type="car" route="r" depart="0"/>'
            '<vehicle id="v1" type="car" depart="1"><route edges="b c"/>'
            '<param key="custom" value="keep"/></vehicle></routes>', encoding="utf-8")
        self.config = self.make_config("sample")

    def make_config(self, name, additions=None):
        root = ET.Element("configuration")
        group = ET.SubElement(root, "input")
        ET.SubElement(group, "net-file", value="net.xml")
        ET.SubElement(group, "route-files", value="route.xml")
        group = ET.SubElement(root, "output")
        ET.SubElement(group, "tripinfo-output", value="truth.xml")
        group = ET.SubElement(root, "time")
        ET.SubElement(group, "begin", value="0")
        ET.SubElement(group, "step-length", value="0.5")
        group = ET.SubElement(root, "processing")
        ET.SubElement(group, "route-steps", value="200")
        group = ET.SubElement(root, "random_number")
        ET.SubElement(group, "seed", value="42")
        tree = ET.ElementTree(root)
        for group, key, value in additions or []:
            MODULE.set_option(tree, group, key, value)
        path = self.source / f"{name}.sumocfg"
        tree.write(path, encoding="utf-8", xml_declaration=True)
        return path


class FactorAndRouteTests(MaterializeFixture):
    def test_reject_nonpositive_nonfinite_and_nonnumeric(self):
        for value in (0, -1, float("inf"), float("-inf"), float("nan"), "nan", None, "bad"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MODULE.positive_factor(value, "v0")

    def test_materialize_roundtrips_full_float_precision(self):
        factors = {"v0": 0.8123456789012345, "v1": 1.1234567890123457}
        updated = MODULE.materialized_tree(MODULE.read_xml(self.route), factors)
        self.assertEqual({v.get("id"): float(v.get("speedFactor")) for v in updated.getroot().findall("vehicle")}, factors)

    def test_only_vehicle_speedfactor_changes(self):
        original = MODULE.read_xml(self.route)
        before = ET.tostring(original.getroot())
        updated = MODULE.materialized_tree(original, {"v0": 0.81, "v1": 1.19})
        self.assertEqual(ET.tostring(original.getroot()), before)
        self.assertTrue(MODULE.route_equivalent_except_sf(original, updated))
        self.assertEqual(updated.getroot().find("vType").get("speedFactor"), "normc(1,0.1,0.2,2)")
        self.assertEqual(updated.getroot().find("vehicle[@id='v1']/param").get("value"), "keep")
        self.assertIn(b"preserved", ET.tostring(updated.getroot()))

    def test_route_comparison_detects_other_changes(self):
        original = MODULE.read_xml(self.route)
        updated = MODULE.materialized_tree(original, {"v0": 1.0, "v1": 1.0})
        updated.getroot().find("vehicle").set("depart", "8")
        self.assertFalse(MODULE.route_equivalent_except_sf(original, updated))

    def test_existing_explicit_factor_cannot_be_changed(self):
        original = MODULE.read_xml(self.route)
        original.getroot().find("vehicle").set("speedFactor", "0.8")
        with self.assertRaisesRegex(ValueError, "already explicit"):
            MODULE.materialized_tree(original, {"v0": 0.81, "v1": 1.0})
        updated = MODULE.materialized_tree(original, {"v0": 0.8, "v1": 1.0})
        self.assertEqual(updated.getroot().find("vehicle").get("speedFactor"), "0.8")

    def test_missing_extra_and_duplicate_vehicle_coverage_fail(self):
        original = MODULE.read_xml(self.route)
        for factors in ({"v0": 1.0}, {"v0": 1.0, "v1": 1.0, "v2": 1.0}):
            with self.subTest(factors=factors), self.assertRaises(ValueError):
                MODULE.materialized_tree(original, factors)
        original.getroot().find("vehicle[@id='v1']").set("id", "v0")
        with self.assertRaises(ValueError):
            MODULE.materialized_tree(original, {"v0": 1.0})


class ConfigurationAndDiscoveryTests(MaterializeFixture):
    def test_inspect_preserves_seed_and_resolves_inputs(self):
        job = MODULE.inspect_config(self.config)
        self.assertEqual(job["seed"], 42)
        self.assertEqual(job["vehicle_count"], 2)
        self.assertEqual(Path(job["route"]), self.route.resolve())
        self.assertEqual(Path(job["net"]), self.net.resolve())

    def test_omitted_seed_is_explicit_default_23423(self):
        tree = MODULE.read_xml(self.config)
        tree.getroot().remove(tree.getroot().find("random_number"))
        MODULE.write_xml(tree, self.config)
        self.assertEqual(MODULE.inspect_config(self.config)["seed"], 23423)

    def test_random_true_rejected(self):
        for value in ("true", "1"):
            config = self.make_config("random", [("random_number", "random", value)])
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "random"):
                MODULE.inspect_config(config)

    def test_unknown_output_and_input_rejected(self):
        for group, name in (("output", "fcd-output"), ("input", "additional-files"),
                            ("report", "log"), ("processing", "save-state.times")):
            config = self.make_config("unsafe", [(group, name, "unsafe.xml")])
            with self.subTest(name=name), self.assertRaises(ValueError):
                MODULE.inspect_config(config)

    def test_configuration_writing_options_rejected(self):
        for name in ("save-configuration", "save-template", "save-schema"):
            config = self.make_config("save_options", [("configuration", name, "unsafe.xml")])
            with self.subTest(name=name), self.assertRaises(ValueError):
                MODULE.inspect_config(config)

    def test_duplicate_sumocfg_option_rejected(self):
        tree = MODULE.read_xml(self.config)
        ET.SubElement(tree.getroot().find("random_number"), "seed", value="10")
        MODULE.write_xml(tree, self.config)
        with self.assertRaisesRegex(ValueError, "Repeated SUMO option"):
            MODULE.inspect_config(self.config)

    def test_multiple_route_files_rejected(self):
        config = self.make_config("multiple", [("input", "route-files", "route.xml,other.xml")])
        with self.assertRaisesRegex(ValueError, "one input file"):
            MODULE.inspect_config(config)

    def test_flows_and_duplicate_vehicle_ids_rejected(self):
        tree = MODULE.read_xml(self.route)
        original = copy.deepcopy(tree)
        ET.SubElement(tree.getroot(), "flow", id="flow0")
        MODULE.write_xml(tree, self.route)
        with self.assertRaisesRegex(ValueError, "explicit vehicle"):
            MODULE.inspect_config(self.config)
        original.getroot().find("vehicle[@id='v1']").set("id", "v0")
        MODULE.write_xml(original, self.route)
        with self.assertRaisesRegex(ValueError, "duplicate vehicle"):
            MODULE.inspect_config(self.config)

    def test_invalid_existing_speedfactor_rejected(self):
        tree = MODULE.read_xml(self.route)
        tree.getroot().find("vehicle").set("speedFactor", "nan")
        MODULE.write_xml(tree, self.route)
        with self.assertRaisesRegex(ValueError, "finite and positive"):
            MODULE.inspect_config(self.config)

    def test_manifest_100_active_14_dead_configs_not_globbed(self):
        commands = self.source / "commands"
        commands.mkdir()
        manifest = commands / "traci_jobs.tsv"
        with manifest.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=MODULE.JOB_COLUMNS, delimiter="\t")
            writer.writeheader()
            for index in range(100):
                name = f"active_{index:03}"
                path = self.make_config(name)
                writer.writerow({"episode_id": name, "split": "train", "sumocfg_file": path.name})
        for index in range(14):
            (self.source / f"dead_{index:03}.sumocfg").write_text("not valid XML", encoding="utf-8")
        jobs = MODULE.discover_jobs(self.source, [])
        self.assertEqual(len(jobs), 100)
        self.assertTrue(all(job["episode_id"].startswith("active_") for job in jobs))
        self.assertTrue(all(str(manifest.resolve()) in job["source_hashes"] for job in jobs))

    def test_repeated_episode_config_rejected(self):
        with self.assertRaisesRegex(ValueError, "repeated episode"):
            MODULE.discover_jobs(None, [self.config, self.config])

    def test_prepared_configs_are_portable_and_preserve_motion_options(self):
        job = MODULE.inspect_config(self.config)
        hashes_before = {path: MODULE.sha256(path) for path in (self.config, self.route, self.net)}
        output = self.root / "portable"
        paths = MODULE.prepare_files(job, output)
        for role in ("reference", "explicit"):
            config = Path(paths[role + "_config"])
            tree = MODULE.read_xml(config)
            for name in ("net-file", "route-files", "tripinfo-output", "statistic-output"):
                value = MODULE.option(tree, name)
                self.assertFalse(Path(value).is_absolute())
                resolved = (config.parent / value).resolve()
                self.assertIn(output.resolve(), resolved.parents)
            for name, value in (("seed", "42"), ("begin", "0"), ("step-length", "0.5"), ("route-steps", "200")):
                self.assertEqual(MODULE.option(tree, name), value)
        self.assertEqual(hashes_before, {path: MODULE.sha256(path) for path in hashes_before})
        self.assertEqual(Path(paths["source_route"]).read_bytes(), self.route.read_bytes())
        received = output / "original" / "config_as_received" / "sample.sumocfg"
        self.assertEqual(received.read_bytes(), self.config.read_bytes())

    def test_copy_checked_does_not_overwrite_changed_file(self):
        destination = self.root / "copy.xml"
        MODULE.copy_checked(self.route, destination)
        destination.write_text("user changes", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Refusing to overwrite"):
            MODULE.copy_checked(self.route, destination)
        self.assertEqual(destination.read_text(encoding="utf-8"), "user changes")


class FakeSimulation:
    def __init__(self, factor_change=False, unexpected=False, teleports=0):
        self.time = 0
        self.factor_change = factor_change
        self.unexpected = unexpected
        self.teleports = teleports

    def getLoadedIDList(self):
        return {0: ("v0",), 1: (("unknown",) if self.unexpected else ("v1",))}.get(self.time, ())

    def getDepartedIDList(self):
        return {1: ("v0",), 2: ("v1",)}.get(self.time, ())

    def getArrivedIDList(self):
        return {2: ("v0",), 3: ("v1",)}.get(self.time, ())

    def getMinExpectedNumber(self):
        return 2 if self.time < 3 else 0

    def getTime(self):
        return self.time

    def getStartingTeleportNumber(self):
        return self.teleports if self.time == 1 else 0


class FakeVehicle:
    def __init__(self, simulation):
        self.simulation = simulation
        self.calls = []

    def getSpeedFactor(self, identifier):
        self.calls.append((self.simulation.time, identifier))
        if identifier == "v0":
            return 0.9 if self.simulation.factor_change and self.simulation.time > 0 else 0.8123456789012345
        return 1.1234567890123457


class FakeTraci:
    def __init__(self, **kwargs):
        self.simulation = FakeSimulation(**kwargs)
        self.vehicle = FakeVehicle(self.simulation)
        self.closed = False
        self.command = None

    def start(self, command, **kwargs):
        self.command = command

    def getVersion(self):
        return 22, "SUMO 1.27.0"

    def simulationStep(self):
        self.simulation.time += 1

    def close(self):
        self.closed = True


class ReplayTests(MaterializeFixture):
    def replay_fake(self, fake, max_time=100):
        with patch.object(MODULE, "load_traci", return_value=fake):
            return MODULE.replay(self.config, self.route, self.root / "output", "reference", "sumo", max_time)

    def test_initial_loaded_and_departure_factors_both_sampled(self):
        fake = FakeTraci()
        result = self.replay_fake(fake)
        self.assertEqual(fake.vehicle.calls, [(0, "v0"), (1, "v1"), (1, "v0"), (2, "v1")])
        self.assertEqual(result["vehicles"], 2)
        self.assertEqual(result["simulation_end"], 3)
        self.assertEqual(result["loaded_departure_changes"], 0)
        self.assertTrue(fake.closed)
        self.assertNotIn("--route-steps", fake.command)
        self.assertNotIn("--seed", fake.command)

    def test_departure_factor_change_is_detected_and_actual_value_used(self):
        result = self.replay_fake(FakeTraci(factor_change=True))
        self.assertEqual(result["loaded_departure_changes"], 1)
        self.assertEqual(result["factors"]["v0"], 0.9)

    def test_unexpected_loaded_id_fails_and_connection_closes(self):
        fake = FakeTraci(unexpected=True)
        with self.assertRaisesRegex(ValueError, "Unexpected loaded vehicle"):
            self.replay_fake(fake)
        self.assertTrue(fake.closed)

    def test_max_time_requires_completion(self):
        fake = FakeTraci()
        with self.assertRaisesRegex(RuntimeError, "before all vehicles completed"):
            self.replay_fake(fake, max_time=1)
        self.assertTrue(fake.closed)

    def test_config_end_requires_completion(self):
        self.config = self.make_config("ending", [("time", "end", "1")])
        with self.assertRaisesRegex(RuntimeError, "before all vehicles completed"):
            self.replay_fake(FakeTraci())

    def test_teleports_fail_validation(self):
        with self.assertRaisesRegex(RuntimeError, "teleports"):
            self.replay_fake(FakeTraci(teleports=1))

    def test_missing_arrival_fails_coverage(self):
        fake = FakeTraci()
        fake.simulation.getArrivedIDList = lambda: ()
        with self.assertRaisesRegex(RuntimeError, "Incomplete vehicle coverage"):
            self.replay_fake(fake)


class ResumeAndComparisonTests(MaterializeFixture):
    def make_complete_manifest(self):
        output = self.root / "output"
        output.mkdir()
        artifact = output / "artifact.xml"
        artifact.write_text("complete", encoding="utf-8")
        job = MODULE.inspect_config(self.config)
        old = {"status": "success", "source_hashes": job["source_hashes"],
               "artifact_hashes": {"artifact.xml": MODULE.sha256(artifact)}}
        MODULE.write_json(output / "manifests" / "sample.json", old)
        return output, artifact, job, old

    def test_successful_resume_returns_verified_existing_result(self):
        output, _, job, old = self.make_complete_manifest()
        with patch.object(MODULE, "replay", side_effect=AssertionError("must not rerun")):
            self.assertEqual(MODULE.run_job(job, str(output), "sumo", 100, True), old)

    def test_resume_rejects_changed_source_hash(self):
        output, _, job, _ = self.make_complete_manifest()
        changed = copy.deepcopy(job)
        changed["source_hashes"][str(self.config)] = "changed"
        with self.assertRaisesRegex(ValueError, "Source inputs changed"):
            MODULE.run_job(changed, str(output), "sumo", 100, True)

    def test_resume_rejects_changed_completed_artifact(self):
        output, artifact, job, _ = self.make_complete_manifest()
        artifact.write_text("user modification", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Completed artifacts changed"):
            MODULE.run_job(job, str(output), "sumo", 100, True)
        self.assertEqual(artifact.read_text(encoding="utf-8"), "user modification")

    def test_cli_existing_unowned_output_rejected_even_resume(self):
        output = self.root / "unowned"
        output.mkdir()
        keep = output / "keep.txt"
        keep.write_text("preserve", encoding="utf-8")
        fake_binary = self.root / "sumo.exe"
        fake_binary.write_bytes(b"not executed; hash-only fixture")
        for extra in ([], ["--resume"]):
            with self.subTest(extra=extra), patch.object(MODULE.shutil, "which", return_value=str(fake_binary)), \
                    contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                MODULE.main(["--config", str(self.config), "--output-dir", str(output), *extra])
        self.assertEqual(keep.read_text(encoding="utf-8"), "preserve")

    def test_cli_output_cannot_contain_source(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            MODULE.main(["--config", str(self.config), "--output-dir", str(self.source)])

    def test_cli_resume_rejects_changed_sumo_binary_signature(self):
        output = self.root / "owned"
        output.mkdir()
        fake_binary = self.root / "sumo.exe"
        fake_binary.write_bytes(b"version A")
        jobs = MODULE.discover_jobs(None, [self.config])
        signature = {"schema": MODULE.SCHEMA, "sumo_binary_sha256": MODULE.sha256(fake_binary),
                     "jobs": [{key: job[key] for key in ("episode_id", "split", "source_hashes")} for job in jobs]}
        marker = output / "manifests" / "materialization_run.json"
        MODULE.write_json(marker, signature)
        before = marker.read_bytes()
        fake_binary.write_bytes(b"version B")
        with patch.object(MODULE.shutil, "which", return_value=str(fake_binary)), \
                patch.object(MODULE, "ProcessPoolExecutor", side_effect=AssertionError("must reject before dispatch")), \
                contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            MODULE.main(["--config", str(self.config), "--output-dir", str(output), "--resume"])
        self.assertEqual(marker.read_bytes(), before)

    def test_cli_resume_rejects_changed_source_signature(self):
        output = self.root / "owned"
        output.mkdir()
        fake_binary = self.root / "sumo.exe"
        fake_binary.write_bytes(b"same version")
        jobs = MODULE.discover_jobs(None, [self.config])
        signature = {"schema": MODULE.SCHEMA, "sumo_binary_sha256": MODULE.sha256(fake_binary),
                     "jobs": [{key: job[key] for key in ("episode_id", "split", "source_hashes")} for job in jobs]}
        MODULE.write_json(output / "manifests" / "materialization_run.json", signature)
        tree = MODULE.read_xml(self.route)
        tree.getroot().find("vehicle").set("depart", "0.5")
        MODULE.write_xml(tree, self.route)
        with patch.object(MODULE.shutil, "which", return_value=str(fake_binary)), \
                patch.object(MODULE, "ProcessPoolExecutor", side_effect=AssertionError("must reject before dispatch")), \
                contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            MODULE.main(["--config", str(self.config), "--output-dir", str(output), "--resume"])

    def test_tripinfo_compares_fields_beyond_duration(self):
        left, right = self.root / "a.xml", self.root / "b.xml"
        left.write_text('<tripinfos><tripinfo id="v0" duration="10" speedFactor="0.8"/></tripinfos>', encoding="utf-8")
        right.write_text('<tripinfos><tripinfo id="v0" duration="10" speedFactor="0.9"/></tripinfos>', encoding="utf-8")
        result = MODULE.compare_tripinfo(left, right)
        self.assertFalse(result["all_attributes_equal"])
        self.assertEqual(result["differing_vehicles"], 1)
        self.assertEqual(result["max_duration_delta"], 0)
        self.assertIn("speedFactor", result["examples"][0]["changed_fields"])

    def test_duplicate_tripinfo_ids_rejected(self):
        trip = self.root / "duplicate.xml"
        trip.write_text('<tripinfos><tripinfo id="v0" duration="10"/><tripinfo id="v0" duration="11"/></tripinfos>', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "Duplicate tripinfo"):
            MODULE.tripinfo_rows(trip)


if __name__ == "__main__":
    unittest.main()
