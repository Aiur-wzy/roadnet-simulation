"""Manifest, pilot and lifecycle tests with a fake collector, never SUMO."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock
import uuid
import xml.etree.ElementTree as ET


RUNNER = Path(__file__).resolve().parents[2] / "collection/run_manhattan_training_batch_v2.py"
SPEC = importlib.util.spec_from_file_location("batch_v2_under_test", RUNNER)
BATCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BATCH)
FAKE_COLLECTOR = r'''
import argparse, gzip, json, os, time
from pathlib import Path
import xml.etree.ElementTree as ET
from collection.run_manhattan_training_batch_v2 import collector_sources, sha
p=argparse.ArgumentParser()
for key in ("sumo-config", "net-file", "output-dir", "sumo-binary", "stall-seconds"):
    p.add_argument("--"+key, required=True)
a=p.parse_args()
output=Path(a.output_dir)
output.mkdir(exist_ok=False)
root=ET.parse(a.sumo_config).getroot()
options={n.tag for n in root.iter() if "value" in n.attrib}
if root.find("output") is not None or any("output" in key or key in {"log","error-log"} for key in options):
    raise SystemExit("fake collector refuses old output destinations")
seed=int(root.find("random_number/seed").get("value"))
route=Path(root.find("input/route-files").get("value"))
vehicles=ET.parse(route).getroot().findall("vehicle")
count=len(vehicles)
(output/"started.json").write_text(json.dumps({"time":time.time(), "pid":os.getpid()}))
if seed == 83:
    (output.parents[2]/"STOP").write_text("test stop requested")
time.sleep(1.2 if seed == 84 else .35 if seed == 82 else .03)
excluded=1 if seed==79 else 0
counts={"expected_vehicles":count,"arrived":count,"tripinfo_count":count,"normal_arrived":count,
        "teleports":0,"collisions":1 if seed==81 else 0,"valid_feature_rows":count,
        "feature_errors":excluded,"road_occurrences":count*3,"trainable":count*3-excluded,
        "excluded":excluded}
for name in ("training.csv.gz","road_visits.csv.gz","vehroute.xml.gz"):
    with gzip.open(output/name,"wt") as stream:
        stream.write("<routes/>\n" if name.startswith("vehroute") else "vehicle_id,route_index\n")
(output/"tripinfo.xml").write_text("<tripinfos/>\n")
manifest={"inputs":{str(Path(x).resolve()):sha(x) for x in (a.sumo_config,a.net_file,str(route))},
          "sources":collector_sources(Path(__file__).resolve().parents[2])}
(output/"manifest.json").write_text(json.dumps(manifest))
audit={"status":"CENSORED" if seed==78 else "FAILED" if seed==77 else "COMPLETE",
       "error":None,"training_released":seed not in (77,78),
       "counts":counts,"explicit_speed_factor_audited":seed!=80,"native_outputs_closed":True,
       "manifest_sha256":sha(output/"manifest.json"),
       "closed_outputs_sha256":{name:sha(output/name) for name in (
           "training.csv.gz","road_visits.csv.gz","tripinfo.xml","vehroute.xml.gz")}}
(output/"audit.json").write_text(json.dumps(audit))
(output/"finished.json").write_text(json.dumps({"time":time.time(), "pid":os.getpid()}))
print("fake collector complete", flush=True)
raise SystemExit(2 if seed in (77,78) else 0)
'''


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.base = RUNNER.parents[2] / "test_runs" / ("batch_" + uuid.uuid4().hex)
        self.base.mkdir(parents=True)
        self.package = self.base / "package"
        (self.package / "collection").mkdir(parents=True)
        (self.package / "collection/__init__.py").write_text("")
        (self.package / "collection/TraCI_Python_SUMO_Native.py").write_text("# Frozen v1 dependency stub.\n")
        (self.package / "collection/native_boundary_v2").mkdir()
        (self.package / "collection/native_boundary_v2/__init__.py").write_text("")
        (self.package / "collection/native_boundary_v2/collect_native_v2.py").write_text(FAKE_COLLECTOR)
        (self.package / "collection/run_manhattan_training_batch_v2.py").write_bytes(RUNNER.read_bytes())
        for relative in ("collection/sumo_native_state.py", "collection/sumo_native_signals.py",
                         "collection/traci_signal_rg.py", "features/__init__.py",
                         "features/cap_context_features.py", "features/signal_rg_features.py"):
            path = self.package / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("# Dummy dependency for lifecycle tests only.\n")
        self.dataset = self.base / "dataset"
        self.dataset.mkdir()
        self.network = self.dataset / "network.net.xml"
        self.network.write_text("<net/>\n")
        self.manifest = self.dataset / "manifest.json"
        self.output = self.base / "collection_out"

    def make_dataset(self, seeds=(1,), vehicles=40):
        rows = []
        for index, seed in enumerate(seeds):
            sid = "scenario_" + str(index)
            directory = self.dataset / sid
            directory.mkdir()
            routes = ET.Element("routes")
            ET.SubElement(routes, "vType", id="original", length="5")
            for number in range(vehicles):
                vehicle = ET.SubElement(routes, "vehicle", id=f"v{number}", depart=f"{number}.25",
                                        speedFactor=f"{.85+number*.002:.6f}", type="original")
                ET.SubElement(vehicle, "route", edges="a b c")
            route = directory / "routes.rou.xml"
            ET.ElementTree(routes).write(route, encoding="utf-8", xml_declaration=True)
            config = ET.Element("configuration")
            inp = ET.SubElement(config, "input")
            ET.SubElement(inp, "net-file", value="../network.net.xml")
            ET.SubElement(inp, "route-files", value="routes.rou.xml")
            time_cfg = ET.SubElement(config, "time")
            ET.SubElement(time_cfg, "begin", value="0")
            ET.SubElement(time_cfg, "step-length", value="1")
            settings = ET.SubElement(config, "processing")
            for key in ("time-to-teleport", "time-to-teleport.highways", "time-to-teleport.disconnected",
                        "time-to-teleport.ride", "time-to-teleport.bidi", "time-to-teleport.railsignal-deadlock"):
                ET.SubElement(settings, key, value="-1")
            for key in ("collision.action", "intermodal-collision.action"):
                ET.SubElement(settings, key, value="warn")
            random = ET.SubElement(config, "random_number")
            ET.SubElement(random, "seed", value=str(seed))
            truth = ET.SubElement(config, "output")
            ET.SubElement(truth, "tripinfo-output", value="tripinfo.xml")
            ET.SubElement(truth, "tripinfo-output.write-unfinished", value="true")
            ET.SubElement(truth, "statistic-output", value="statistics.xml")
            ET.SubElement(truth, "summary-output", value="summary.xml")
            cfg = directory / "run.sumocfg"
            ET.ElementTree(config).write(cfg, encoding="utf-8", xml_declaration=True)
            record = {"scenario_id": sid, "family_id": "family_" + str(index), "split": "train", "vehicle_count": vehicles}
            scene_manifest = directory / "manifest.json"
            BATCH.fresh_json(scene_manifest, {"schema": BATCH.DATASET_SCHEMA,
                "status": "DEMAND_GENERATED_NOT_SIMULATED", "scenario": record,
                "inputs": {"network": {"sha256": BATCH.sha(self.network)}},
                "outputs": {"routes.rou.xml": {"sha256": BATCH.sha(route)}, "run.sumocfg": {"sha256": BATCH.sha(cfg)}}})
            rows.append({**record, "route_file": sid + "/routes.rou.xml", "sumo_config": sid + "/run.sumocfg",
                         "scenario_manifest": sid + "/manifest.json", "route_sha256": BATCH.sha(route),
                         "config_sha256": BATCH.sha(cfg), "scenario_manifest_sha256": BATCH.sha(scene_manifest),
                         "network_file": str(self.network), "network_sha256": BATCH.sha(self.network)})
        BATCH.fresh_json(self.manifest, {"schema": BATCH.DATASET_SCHEMA, "status": "DEMAND_GENERATED_NOT_SIMULATED",
                         "ready": True, "inputs": {"network": {"path": str(self.network), "sha256": BATCH.sha(self.network)}}, "scenarios": rows})

    def args(self, *extra):
        return BATCH.parser().parse_args(["--dataset-manifest", str(self.manifest), "--output-root", str(self.output),
                                         "--poll-seconds", ".05", *extra])

    def run_batch(self, *extra):
        with contextlib.redirect_stdout(io.StringIO()):
            return BATCH.run_batch(self.args(*extra), package_root=self.package)

    def status(self, index=0):
        return BATCH.read_json(self.output / "jobs" / ("scenario_" + str(index)) / "status.json")

    def test_dry_run_creates_no_outputs_or_workers(self):
        self.make_dataset()
        self.assertEqual(self.run_batch("--dry-run", "--pilot"), 0)
        self.assertFalse(self.output.exists())

    def test_pilot_preserves_first_32_vehicles_and_all_dynamics(self):
        self.make_dataset()
        source_route = self.dataset / "scenario_0/routes.rou.xml"
        source_hash = BATCH.sha(source_route)
        source_config = self.dataset / "scenario_0/run.sumocfg"
        config_hash = BATCH.sha(source_config)
        self.assertEqual(self.run_batch("--pilot"), 0)
        job = self.output / "jobs/scenario_0"
        originals = ET.parse(source_route).getroot().findall("vehicle")[:32]
        copied = ET.parse(job / "inputs/routes.rou.xml").getroot().findall("vehicle")
        self.assertEqual([ET.tostring(v) for v in originals], [ET.tostring(v) for v in copied])
        self.assertEqual(BATCH.sha(source_route), source_hash)
        self.assertEqual(BATCH.sha(source_config), config_hash)
        original_cfg = ET.parse(self.dataset / "scenario_0/run.sumocfg").getroot()
        copied_cfg = ET.parse(job / "inputs/run.sumocfg").getroot()
        self.assertIsNone(copied_cfg.find("output"))
        original_cfg.remove(original_cfg.find("output"))
        for root in (original_cfg, copied_cfg):
            root.remove(root.find("input"))
        self.assertEqual(ET.tostring(original_cfg), ET.tostring(copied_cfg))
        self.assertEqual(self.status()["counts"]["expected_vehicles"], 32)
        self.assertEqual(self.status()["state"], "COMPLETE")
        for directory in (source_config.parent, job / "inputs"):
            for name in ("tripinfo.xml", "statistics.xml", "summary.xml"):
                self.assertFalse((directory / name).exists())

    def test_small_pilot_retains_all_existing_vehicles(self):
        self.make_dataset(vehicles=7)
        self.assertEqual(self.run_batch("--pilot"), 0)
        self.assertEqual(self.status()["counts"]["arrived"], 7)

    def refreeze_test_config(self, root):
        """Recreate the test-only generator hashes after a deliberate variant."""
        cfg = self.dataset / "scenario_0/run.sumocfg"
        ET.ElementTree(root).write(cfg, encoding="utf-8", xml_declaration=True)
        scene_path = self.dataset / "scenario_0/manifest.json"
        scene = BATCH.read_json(scene_path)
        scene["outputs"]["run.sumocfg"]["sha256"] = BATCH.sha(cfg)
        BATCH.atomic_json(scene_path, scene)
        dataset = BATCH.read_json(self.manifest)
        dataset["scenarios"][0]["config_sha256"] = BATCH.sha(cfg)
        dataset["scenarios"][0]["scenario_manifest_sha256"] = BATCH.sha(scene_path)
        BATCH.atomic_json(self.manifest, dataset)
        return cfg

    def test_unknown_output_and_log_settings_fail_before_any_worker(self):
        self.make_dataset()
        original = ET.tostring(ET.parse(self.dataset / "scenario_0/run.sumocfg").getroot())
        for section, tag, value in (("output", "fcd-output", "old_fcd.xml"),
                                    ("report", "log", "old.log"),
                                    ("report", "error-log", "old_errors.log"),
                                    ("processing", "save-state.times", "100"),
                                    ("processing", "save-state.files", "old_state.xml"),
                                    ("report", "summary-output", "summary.xml")):
            root = ET.fromstring(original)
            parent = root.find(section)
            if parent is None:
                parent = ET.SubElement(root, section)
            ET.SubElement(parent, tag, value=value)
            cfg = self.refreeze_test_config(root)
            before = BATCH.sha(cfg)
            with self.subTest(tag=tag), self.assertRaisesRegex(ValueError, "output|log"):
                self.run_batch("--dry-run", "--pilot")
            self.assertEqual(BATCH.sha(cfg), before)
            self.assertFalse(self.output.exists())

    def test_changed_known_output_and_duplicate_sections_are_not_silently_removed(self):
        self.make_dataset()
        original = ET.tostring(ET.parse(self.dataset / "scenario_0/run.sumocfg").getroot())
        for kind in ("different_path", "different_unfinished_flag", "duplicate_section"):
            root = ET.fromstring(original)
            if kind == "different_path":
                root.find("output/summary-output").set("value", "other.xml")
            elif kind == "different_unfinished_flag":
                root.find("output/tripinfo-output.write-unfinished").set("value", "false")
            else:
                ET.SubElement(root, "output")
            self.refreeze_test_config(root)
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, "output"):
                self.run_batch("--dry-run", "--pilot")
            self.assertFalse(self.output.exists())

    def test_already_output_free_config_is_not_given_new_output_options(self):
        self.make_dataset(vehicles=7)
        root = ET.parse(self.dataset / "scenario_0/run.sumocfg").getroot()
        root.remove(root.find("output"))
        cfg = self.refreeze_test_config(root)
        before = BATCH.sha(cfg)
        self.assertEqual(self.run_batch("--pilot"), 0)
        copied = ET.parse(self.output / "jobs/scenario_0/inputs/run.sumocfg").getroot()
        self.assertIsNone(copied.find("output"))
        self.assertEqual(BATCH.sha(cfg), before)

    def test_generated_output_config_becomes_acceptable_to_real_collector_input_gate(self):
        with mock.patch.object(sys, "path", [str(RUNNER.parents[1]), *sys.path]):
            from collection.native_boundary_v2.collect_native_v2 import safe_inputs
        self.make_dataset(vehicles=7)
        self.assertEqual(self.run_batch("--pilot"), 0)
        spec = BATCH.read_json(self.output / "jobs/scenario_0/job.json")
        expected, routes = safe_inputs(Path(spec["config"]), Path(spec["network"]))
        self.assertEqual(len(expected), 7)
        self.assertEqual(routes, [Path(spec["route"])])

    def test_full_collection_has_separate_contract_and_no_demand_copy(self):
        self.make_dataset(vehicles=40)
        self.assertEqual(self.run_batch(), 0)
        self.assertEqual(self.status()["counts"]["arrived"], 40)
        self.assertFalse((self.output / "jobs/scenario_0/inputs/routes.rou.xml").exists())
        with self.assertRaisesRegex(ValueError, "Resume contract changed"):
            self.run_batch("--resume", "--pilot")

    def test_resume_completed_batch_does_not_launch_again(self):
        self.make_dataset()
        self.assertEqual(self.run_batch("--pilot"), 0)
        before = BATCH.sha(self.output / "jobs/scenario_0/result/started.json")
        self.assertEqual(self.run_batch("--resume"), 0)
        self.assertEqual(BATCH.sha(self.output / "jobs/scenario_0/result/started.json"), before)

    def test_existing_root_without_resume_is_refused(self):
        self.make_dataset()
        self.output.mkdir()
        with self.assertRaisesRegex(ValueError, "Output root exists"):
            self.run_batch()

    def test_changed_manifest_on_resume_is_refused(self):
        self.make_dataset()
        self.assertEqual(self.run_batch("--pilot"), 0)
        self.manifest.write_text(self.manifest.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "Resume contract changed"):
            self.run_batch("--resume")

    def test_changed_helper_module_on_resume_is_refused(self):
        self.make_dataset()
        self.assertEqual(self.run_batch("--pilot"), 0)
        (self.package / "features/signal_rg_features.py").write_text("# Changed after the frozen batch.\n")
        with self.assertRaisesRegex(ValueError, "Resume contract changed"):
            self.run_batch("--resume")

    def test_tampered_route_is_rejected_before_output_creation(self):
        self.make_dataset()
        route = self.dataset / "scenario_0/routes.rou.xml"
        route.write_text(route.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.run_batch("--dry-run")
        self.assertFalse(self.output.exists())

    def test_completed_audit_drift_is_rejected_on_resume(self):
        self.make_dataset()
        self.assertEqual(self.run_batch("--pilot"), 0)
        audit = self.output / "jobs/scenario_0/result/audit.json"
        audit.write_text(audit.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.run_batch("--resume")

    def test_unknown_launch_is_never_automatically_repeated(self):
        self.make_dataset()
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            BATCH.run_batch(self.args("--dry-run", "--pilot"), package_root=self.package)
        contract = json.loads(captured.getvalue())["contract"]
        self.output.mkdir()
        (self.output / "jobs").mkdir()
        BATCH.fresh_json(self.output / "batch_manifest.json", {"schema": BATCH.BATCH_SCHEMA, "contract": contract})
        job = BATCH.prepare_job(self.output, contract["scenarios"][0], contract, self.package)
        state = BATCH.read_json(job / "status.json")
        state["state"] = "LAUNCHING"
        BATCH.atomic_json(job / "status.json", state)
        self.assertEqual(self.run_batch("--resume"), 2)
        self.assertEqual(self.status()["state"], "INTERRUPTED")
        self.assertFalse((job / "result").exists())
        self.assertFalse((job / "worker.stdout.log").exists())

    def test_preflight_only_manifest_is_not_runnable(self):
        self.make_dataset()
        data = BATCH.read_json(self.manifest)
        data["ready"] = False
        BATCH.atomic_json(self.manifest, data)
        with self.assertRaisesRegex(ValueError, "not a ready"):
            self.run_batch("--dry-run")

    def test_traversal_and_unknown_scenario_are_rejected(self):
        self.make_dataset()
        with self.assertRaisesRegex(ValueError, "Unknown"):
            BATCH.load_dataset(self.manifest, ["unknown"])
        manifest = BATCH.read_json(self.manifest)
        manifest["scenarios"][0]["route_file"] = "../outside.xml"
        BATCH.atomic_json(self.manifest, manifest)
        with self.assertRaisesRegex(ValueError, "escapes"):
            BATCH.load_dataset(self.manifest)

    def test_network_relocation_requires_same_sha(self):
        self.make_dataset()
        moved = self.base / "same_network.xml"
        moved.write_bytes(self.network.read_bytes())
        self.network.unlink()
        self.assertEqual(self.run_batch("--pilot", "--net-file", str(moved)), 0)
        self.assertEqual(BATCH.read_json(self.output / "jobs/scenario_0/job.json")["network"], str(moved))

    def test_censored_worker_stops_new_launches_and_is_never_retried(self):
        self.make_dataset(seeds=(78, 1))
        self.assertEqual(self.run_batch("--max-parallel", "1", "--pilot"), 2)
        self.assertEqual(self.status()["state"], "CENSORED")
        self.assertFalse((self.output / "jobs/scenario_1").exists())
        started = BATCH.sha(self.output / "jobs/scenario_0/result/started.json")
        self.assertEqual(self.run_batch("--resume"), 2)
        self.assertEqual(BATCH.sha(self.output / "jobs/scenario_0/result/started.json"), started)

    def test_isolated_feature_errors_do_not_fail_the_run_or_next_dispatch(self):
        self.make_dataset(seeds=(79, 1))
        self.assertEqual(self.run_batch("--pilot", "--max-parallel", "1"), 0)
        self.assertEqual(self.status()["state"], "COMPLETE")
        self.assertEqual(self.status()["counts"]["excluded"], 1)
        self.assertEqual(self.status(1)["state"], "COMPLETE")

    def test_collision_and_missing_sf_are_not_complete(self):
        base = {"status": "COMPLETE", "error": None, "training_released": True,
                "explicit_speed_factor_audited": True, "native_outputs_closed": True,
                "counts": {"expected_vehicles": 32, "arrived": 32, "tripinfo_count": 32,
                           "normal_arrived": 32, "teleports": 0, "collisions": 0,
                           "valid_feature_rows": 20, "feature_errors": 0,
                           "road_occurrences": 32, "trainable": 20, "excluded": 12}}
        for field in ("collisions", "teleports"):
            base["counts"][field] = 1
            self.assertNotEqual(BATCH.assess_audit(base, 0, 32, True)[0], "COMPLETE")
            base["counts"][field] = 0
        base["explicit_speed_factor_audited"] = False
        self.assertNotEqual(BATCH.assess_audit(base, 0, 32, True)[0], "COMPLETE")
        base["explicit_speed_factor_audited"] = True
        base["counts"]["trainable"] = 0
        base["counts"]["excluded"] = 32
        self.assertNotEqual(BATCH.assess_audit(base, 0, 32, True)[0], "COMPLETE")
        base["counts"]["trainable"] = 20
        base["counts"]["excluded"] = 12
        base["native_outputs_closed"] = False
        self.assertNotEqual(BATCH.assess_audit(base, 0, 32, True)[0], "COMPLETE")
        base.pop("native_outputs_closed")
        self.assertNotEqual(BATCH.assess_audit(base, 0, 32, True)[0], "COMPLETE")

    def complete_evidence(self):
        self.make_dataset(vehicles=7)
        self.assertEqual(self.run_batch("--pilot"), 0)
        job = self.output / "jobs/scenario_0"
        return BATCH.read_json(job / "job.json"), BATCH.read_json(job / "result/audit.json")

    def test_complete_error_and_release_flags_are_independent_gates(self):
        _, audit = self.complete_evidence()
        for key, bad in (("error", "close failed"), ("error", ""),
                         ("training_released", False), ("native_outputs_closed", False)):
            with self.subTest(key=key, bad=bad):
                changed = dict(audit, **{key: bad})
                self.assertNotEqual(BATCH.assess_audit(changed, 0, 7, True)[0], "COMPLETE")
        for key in ("error", "training_released", "native_outputs_closed"):
            changed = dict(audit)
            changed.pop(key)
            self.assertNotEqual(BATCH.assess_audit(changed, 0, 7, True)[0], "COMPLETE")

    def test_quality_counts_require_integers_and_exact_partition(self):
        _, audit = self.complete_evidence()
        for key, bad in (("trainable", True), ("excluded", -1), ("road_occurrences", 22),
                         ("collisions", False), ("normal_arrived", 7.0)):
            changed = dict(audit, counts=dict(audit["counts"], **{key: bad}))
            with self.subTest(key=key, bad=bad):
                self.assertNotEqual(BATCH.assess_audit(changed, 0, 7, True)[0], "COMPLETE")

    def test_closed_training_output_hash_is_rechecked(self):
        spec, audit = self.complete_evidence()
        training = Path(spec["result"]) / "training.csv.gz"
        training.write_bytes(training.read_bytes() + b"changed")
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            BATCH.verify_result_bindings(spec, audit)
        with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
            self.run_batch("--resume")

    def test_manifest_hash_cannot_be_omitted_or_changed(self):
        spec, audit = self.complete_evidence()
        audit.pop("manifest_sha256")
        with self.assertRaisesRegex(ValueError, "SHA256"):
            BATCH.verify_result_bindings(spec, audit)

    def test_manifest_input_binding_must_match_this_job(self):
        spec, audit = self.complete_evidence()
        path = Path(spec["result"]) / "manifest.json"
        manifest = BATCH.read_json(path)
        manifest["inputs"].pop(spec["route"])
        BATCH.atomic_json(path, manifest)
        audit["manifest_sha256"] = BATCH.sha(path)
        with self.assertRaisesRegex(ValueError, "inputs differ"):
            BATCH.verify_result_bindings(spec, audit)

    def test_manifest_source_binding_must_match_frozen_bundle(self):
        spec, audit = self.complete_evidence()
        path = Path(spec["result"]) / "manifest.json"
        manifest = BATCH.read_json(path)
        manifest["sources"].pop("collection/run_manhattan_training_batch_v2.py")
        BATCH.atomic_json(path, manifest)
        audit["manifest_sha256"] = BATCH.sha(path)
        with self.assertRaisesRegex(ValueError, "sources differ"):
            BATCH.verify_result_bindings(spec, audit)

    def test_missing_native_output_binding_is_refused(self):
        spec, audit = self.complete_evidence()
        audit["closed_outputs_sha256"].pop("vehroute.xml.gz")
        with self.assertRaisesRegex(ValueError, "lacks closed"):
            BATCH.verify_result_bindings(spec, audit)

    def test_closed_output_path_traversal_and_partial_are_refused(self):
        spec, audit = self.complete_evidence()
        for bad in ("../outside.xml", "training.csv.gz.partial", "audit.json"):
            changed = dict(audit, closed_outputs_sha256=dict(audit["closed_outputs_sha256"],
                                                           **{bad: "0" * 64}))
            with self.subTest(path=bad), self.assertRaises(ValueError):
                BATCH.verify_result_bindings(spec, changed)

    def test_v2_source_bundle_includes_runner_and_nested_runtime(self):
        runtime = self.package / "collection/native_boundary_v2/streaming_state.py"
        runtime.write_text("# immutable runtime dependency\n")
        sources = BATCH.collector_sources(self.package)
        self.assertEqual(sources["collection/run_manhattan_training_batch_v2.py"], BATCH.sha(RUNNER))
        self.assertIn("collection/native_boundary_v2/collect_native_v2.py", sources)
        self.assertEqual(sources["collection/native_boundary_v2/streaming_state.py"], BATCH.sha(runtime))

    def test_shared_reader_released_within_retry_allows_atomic_replace(self):
        path = self.base / "status.json"
        BATCH.atomic_json(path, {"version": 1})
        with BATCH.open_shared_read(path) as old_handle:
            self.assertEqual(json.loads(old_handle.read()), {"version": 1})
            # Some Windows volumes/sandbox filters still deny MoveFileEx while
            # an old handle exists despite SHARE_DELETE. The bounded fallback
            # must succeed after that exact reader closes, without truncation.
            release = threading.Timer(.1, old_handle.close)
            release.start()
            try:
                BATCH.atomic_json(path, {"version": 2})
            finally:
                release.join()
        self.assertEqual(BATCH.read_json(path), {"version": 2})

    @unittest.skipUnless(os.name == "nt", "Win32 share flags apply only on Windows")
    def test_windows_shared_reader_requests_read_write_delete_sharing(self):
        import ctypes
        from ctypes import wintypes
        path = self.base / "status.json"
        BATCH.atomic_json(path, {"version": 1})
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.CreateFileW.argtypes = (wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD,
                                      wintypes.LPVOID, wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE)
        kernel.CreateFileW.restype = wintypes.HANDLE
        create = mock.Mock(wraps=kernel.CreateFileW)
        recording = SimpleNamespace(CreateFileW=create, CloseHandle=kernel.CloseHandle)
        with mock.patch.object(ctypes, "WinDLL", return_value=recording):
            self.assertEqual(BATCH.read_json(path), {"version": 1})
        self.assertEqual(create.call_args.args[1:3], (0x80000000, 0x1 | 0x2 | 0x4))

    def test_transient_read_permission_error_is_retried(self):
        error = PermissionError("Windows replacement race")
        error.winerror = 5
        with mock.patch.object(BATCH, "read_json_once", side_effect=[error, error, {"version": 3}]) as reader:
            with mock.patch.object(BATCH.time, "sleep"):
                self.assertEqual(BATCH.read_json(self.base / "status.json"), {"version": 3})
        self.assertEqual(reader.call_count, 3)

    def test_read_retry_is_bounded_and_does_not_mask_invalid_json(self):
        error = PermissionError("persistent sharing violation")
        error.winerror = 32
        operation = mock.Mock(side_effect=error)
        with self.assertRaises(PermissionError) as caught:
            BATCH.retry_windows_io(operation, timeout=0)
        self.assertIs(caught.exception, error)
        self.assertEqual(operation.call_count, 1)
        broken = self.base / "broken.json"
        broken.write_text("{bad json}")
        with mock.patch.object(BATCH.time, "sleep") as sleeper:
            with self.assertRaises(json.JSONDecodeError):
                BATCH.read_json(broken)
            sleeper.assert_not_called()

    def test_permanent_atomic_replace_error_retains_old_and_new_files(self):
        path = self.base / "status.json"
        BATCH.atomic_json(path, {"version": 1})
        error = PermissionError("persistent Windows share lock")
        error.winerror = 32
        with mock.patch.object(BATCH.os, "replace", side_effect=error):
            with mock.patch.object(BATCH, "retry_windows_io", side_effect=lambda operation: operation()):
                with self.assertRaises(PermissionError):
                    BATCH.atomic_json(path, {"version": 2})
        self.assertEqual(BATCH.read_json(path), {"version": 1})
        retained = list(self.base.glob("status.json.*.tmp"))
        self.assertEqual(len(retained), 1)
        self.assertEqual(BATCH.read_json(retained[0]), {"version": 2})

    def test_stop_finishes_current_worker_and_resume_only_starts_pending(self):
        self.make_dataset(seeds=(83, 1))
        self.assertEqual(self.run_batch("--pilot", "--max-parallel", "1"), 2)
        self.assertEqual(self.status()["state"], "COMPLETE")
        self.assertFalse((self.output / "jobs/scenario_1").exists())
        self.assertEqual(BATCH.read_json(self.output / "status.json")["status"], "STOPPED")
        started = BATCH.sha(self.output / "jobs/scenario_0/result/started.json")
        (self.output / "STOP").unlink()
        self.assertEqual(self.run_batch("--resume"), 0)
        self.assertEqual(BATCH.sha(self.output / "jobs/scenario_0/result/started.json"), started)
        self.assertEqual(self.status(1)["state"], "COMPLETE")

    def test_max_parallel_two_limits_real_fake_worker_overlap(self):
        self.make_dataset(seeds=(82, 82, 82))
        self.assertEqual(self.run_batch("--pilot", "--max-parallel", "2"), 0)
        events = []
        for index in range(3):
            directory = self.output / f"jobs/scenario_{index}/result"
            events += [(BATCH.read_json(directory / "started.json")["time"], 1),
                       (BATCH.read_json(directory / "finished.json")["time"], -1)]
        concurrent = maximum = 0
        for _, change in sorted(events):
            concurrent += change
            maximum = max(maximum, concurrent)
        self.assertEqual(maximum, 2)

    def test_kernel_lock_prevents_second_supervisor(self):
        self.make_dataset()
        self.output.mkdir()
        with BATCH.FileLock(self.output / "test.lock"):
            with self.assertRaisesRegex(RuntimeError, "Another process"):
                with BATCH.FileLock(self.output / "test.lock"):
                    pass

    def test_resume_follows_worker_after_supervisor_is_terminated(self):
        self.make_dataset(seeds=(84,))
        driver = self.base / "driver.py"
        driver.write_text("import importlib.util,sys\n"
            "spec=importlib.util.spec_from_file_location('runner',sys.argv[1])\n"
            "m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)\n"
            "args=m.parser().parse_args(sys.argv[3:])\n"
            "raise SystemExit(m.run_batch(args, package_root=sys.argv[2]))\n")
        with (self.base / "supervisor.stdout").open("wb") as stdout, (self.base / "supervisor.stderr").open("wb") as stderr:
            process = subprocess.Popen([sys.executable, "-B", str(driver), str(RUNNER), str(self.package),
                        "--dataset-manifest", str(self.manifest), "--output-root", str(self.output),
                        "--pilot", "--poll-seconds", ".05"], stdout=stdout, stderr=stderr, **BATCH.popen_options())
            try:
                deadline = time.monotonic() + 10
                marker = self.output / "jobs/scenario_0/result/started.json"
                while not marker.exists() and process.poll() is None and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(marker.exists(), (self.base / "supervisor.stderr").read_text())
                started = BATCH.sha(marker)
                process.terminate()  # Only this test's isolated fake supervisor.
                process.wait(timeout=10)
                self.assertEqual(self.run_batch("--resume"), 0)
                self.assertEqual(BATCH.sha(marker), started)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=10)


if __name__ == "__main__":
    unittest.main(verbosity=2)
