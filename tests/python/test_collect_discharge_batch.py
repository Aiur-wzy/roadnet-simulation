"""Small deterministic tests: no SUMO, SSH, training, or external processes."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import unittest
from unittest.mock import Mock, patch
import uuid

from collection import collect_discharge_batch as batch


class BatchFixture(unittest.TestCase):
    def setUp(self):
        # Ordinary mkdir inherits this workspace's ACL on Windows; Python's
        # mode-0700 TemporaryDirectory can be inaccessible in the sandbox.
        self.artifacts = Path(__file__).resolve().parent / "_batch_test_artifacts"
        self.artifacts.mkdir(exist_ok=True)
        self.root = self.artifacts / ("fixture_" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup_fixture)
        self.net = self.root / "network.net.xml"
        self.net.write_text("<net/>", encoding="utf-8")
        self.route = self.root / "route.rou.xml"
        self.route.write_text('<routes><vehicle id="a" depart="0"><route edges="x"/></vehicle>'
                              '<vehicle id="b" depart="2"><route edges="x"/></vehicle></routes>', encoding="utf-8")
        self.config = self.root / "input.sumocfg"
        self.config.write_text('<configuration><input><net-file value="network.net.xml"/>'
                               '<route-files value="route.rou.xml"/></input></configuration>', encoding="utf-8")
        self.job = {"run_id": "mock_01", "domain": "small", "network": str(self.net),
            "route": str(self.route), "sumo_config": str(self.config), "sumo_seed": 42,
            "source_hashes": {str(p): batch.sha256_file(p) for p in (self.net, self.route, self.config)},
            "expected_vehicles": 2, "planned_seconds": 10, "safety_end_seconds": 100}
        self.plan = {"schema": batch.PLAN_SCHEMA, "jobs": [self.job]}
        self.limits = {**batch.DEFAULT_LIMITS, "min_free_gib": 0.00001}
        self.output = self.root / "output"

    def cleanup_fixture(self):
        target = self.root.resolve()
        if target.parent != self.artifacts.resolve() or not target.name.startswith("fixture_"):
            raise RuntimeError("refusing cleanup outside this test's isolated fixture")
        if target.exists():
            shutil.rmtree(target)

    def metadata(self):
        return {"termination": "natural_drain", "completed": True, "remaining": 0,
                "cumulative_counts": {"departed": 2, "arrived": 2, "teleport_start": 0,
                    "teleport_end": 0, "collision_events": 0, "colliding_vehicle_reports": 0}}

    def tripinfo(self, path=None, changes=None):
        target = path or (self.root / "tripinfo.xml")
        attrs = [dict(id="a", depart="0", arrival="8", duration="8", vaporized=""),
                 dict(id="b", depart="2", arrival="12", duration="10", vaporized="")]
        for i, update in (changes or {}).items():
            attrs[i].update(update)
        from xml.etree import ElementTree as ET
        root = ET.Element("tripinfos")
        for row in attrs:
            ET.SubElement(root, "tripinfo", row)
        ET.ElementTree(root).write(target, encoding="utf-8")
        return target

    def adapter(self, updates=None, mutate=None, events=()):
        def run(config):
            self.tripinfo(Path(config["output_dir"]) / "tripinfo.xml")
            config["metadata"].update(self.metadata())
            config["metadata"].update(updates or {})
            if mutate:
                mutate()
            yield from events
        return run

    def collect(self, **kwargs):
        return batch.collect_job(self.job, self.output, self.limits,
                                 batch.DEFAULT_OBSERVATION, adapter=self.adapter(), **kwargs)


class PlanTests(BatchFixture):
    def test_valid_explicit_plan(self):
        result = batch.validate_plan(self.plan)
        self.assertEqual(result["jobs"][0]["vehicles"], 2)
        self.assertEqual(result["resource_limits"]["max_workers"], 1)

    def test_duplicate_run_id_rejected(self):
        self.plan["jobs"].append(deepcopy(self.job))
        with self.assertRaisesRegex(ValueError, "duplicate run_id"):
            batch.validate_plan(self.plan)

    def test_unsafe_name_rejected(self):
        self.job["run_id"] = "../wrong"
        with self.assertRaises(ValueError):
            batch.validate_plan(self.plan)

    def test_modified_input_rejected(self):
        self.net.write_text("<changed/>")
        with self.assertRaises(batch.QualityError):
            batch.validate_plan(self.plan)

    def test_unbound_additional_rejected(self):
        add = self.root / "signal.add.xml"
        add.write_text("<additional/>")
        self.config.write_text(self.config.read_text().replace('</input>', '<additional-files value="signal.add.xml"/></input>'))
        self.job["source_hashes"][str(self.config)] = batch.sha256_file(self.config)
        with self.assertRaisesRegex(ValueError, "unbound SUMO input"):
            batch.validate_plan(self.plan)

    def test_config_route_mismatch_rejected(self):
        other = self.root / "other.xml"
        other.write_text(self.route.read_text())
        self.job["route"] = str(other)
        self.job["source_hashes"][str(other)] = batch.sha256_file(other)
        with self.assertRaisesRegex(ValueError, "selected route"):
            batch.validate_plan(self.plan)

    def test_wrong_count_rejected(self):
        self.job["expected_vehicles"] = 3
        with self.assertRaisesRegex(ValueError, "expected_vehicles"):
            batch.validate_plan(self.plan)

    def test_flow_expansion_not_guessed(self):
        self.route.write_text('<routes><flow id="f" begin="0" end="10" number="2"/></routes>')
        with self.assertRaisesRegex(ValueError, "explicit vehicle"):
            batch.route_vehicles(self.route)

    def test_route_duplicate_rejected(self):
        self.route.write_text(self.route.read_text().replace('id="b"', 'id="a"'))
        with self.assertRaisesRegex(ValueError, "duplicate route"):
            batch.route_vehicles(self.route)

    def test_route_nonfinite_rejected(self):
        self.route.write_text(self.route.read_text().replace('depart="2"', 'depart="nan"'))
        with self.assertRaises(ValueError):
            batch.route_vehicles(self.route)

    def test_threads_and_parallel_immutable(self):
        for key in ("threads", "max_workers"):
            with self.subTest(key=key):
                self.plan["resource_limits"] = {key: 2}
                with self.assertRaises(ValueError):
                    batch.validate_plan(self.plan)

    def test_horizon_after_departures(self):
        self.job["safety_end_seconds"] = 1
        with self.assertRaises(ValueError):
            batch.validate_plan(self.plan)


class TerminalAuditTests(BatchFixture):
    def audit(self, changes=None, metadata=None):
        return batch.audit_tripinfo(self.tripinfo(changes=changes),
                                   batch.route_vehicles(self.route), metadata or self.metadata())

    def test_exact_normal_completion(self):
        result = self.audit()
        self.assertTrue(result["passed"])
        self.assertEqual(result["duration_mean_seconds"], 9)

    def test_collision_explicit_not_teleport(self):
        metadata = self.metadata()
        metadata["cumulative_counts"]["collision_events"] = 2
        result = self.audit(metadata=metadata)
        self.assertFalse(result["passed"])
        self.assertEqual(result["collision_events"], 2)
        self.assertNotIn("teleport_start", result["errors"])

    def test_teleport_rejects(self):
        metadata = self.metadata()
        metadata["cumulative_counts"]["teleport_start"] = 1
        self.assertFalse(self.audit(metadata=metadata)["passed"])

    def test_unknown_counter_never_zero(self):
        metadata = self.metadata()
        del metadata["cumulative_counts"]["collision_events"]
        self.assertFalse(self.audit(metadata=metadata)["passed"])

    def test_unfinished_rejects(self):
        self.assertFalse(self.audit({1: {"arrival": "-1", "duration": "-1"}})["passed"])

    def test_not_departed_rejects(self):
        self.assertFalse(self.audit({1: {"depart": "-1"}})["passed"])

    def test_vaporized_rejects(self):
        self.assertFalse(self.audit({1: {"vaporized": "true"}})["passed"])

    def test_missing_and_unexpected_ids(self):
        result = self.audit({1: {"id": "wrong"}})
        self.assertEqual(result["errors"]["missing_ids"], 1)
        self.assertEqual(result["errors"]["unexpected_ids"], 1)

    def test_duplicate_terminal_id_rejects(self):
        self.assertFalse(self.audit({1: {"id": "a"}})["passed"])

    def test_nonfinite_rejects(self):
        self.assertFalse(self.audit({1: {"arrival": "nan"}})["passed"])

    def test_duration_mismatch_rejects(self):
        self.assertFalse(self.audit({1: {"duration": "15"}})["passed"])

    def test_horizon_not_completion(self):
        metadata = self.metadata()
        metadata.update(termination="diagnostic_horizon", completed=False, remaining=1)
        self.assertFalse(self.audit(metadata=metadata)["passed"])


class CollectorTests(BatchFixture):
    def test_publish_only_after_audit(self):
        observed = []
        real = batch.audit_tripinfo
        def inspect(*args):
            observed.append(batch.load_json(self.output / "manifest.json")["complete"])
            return real(*args)
        with patch.object(batch, "audit_tripinfo", side_effect=inspect):
            result = self.collect()
        self.assertEqual(observed, [False])
        self.assertTrue(result["complete"])
        self.assertEqual(result["status"], "complete_no_valid_labels")
        self.assertFalse(batch.verify_finalization(self.output, require_trainable=False)["training_eligible"])
        with self.assertRaisesRegex(batch.QualityError, "no valid training labels"):
            batch.verify_finalization(self.output)

    def test_output_never_overwritten(self):
        self.output.mkdir()
        marker = self.output / "KEEP"
        marker.write_text("original")
        with self.assertRaises(FileExistsError):
            self.collect()
        self.assertEqual(marker.read_text(), "original")

    def test_pilot_never_trainable_even_if_network_drained(self):
        result = self.collect(horizon=20)
        self.assertFalse(result["complete"])
        self.assertEqual(result["status"], "diagnostic_complete")
        with self.assertRaises(batch.QualityError):
            batch.verify_finalization(self.output)

    def test_changed_source_quarantined(self):
        with self.assertRaises(batch.QualityError):
            batch.collect_job(self.job, self.output, self.limits, batch.DEFAULT_OBSERVATION,
                adapter=self.adapter(mutate=lambda: self.route.write_text("<routes/>")))
        manifest = batch.load_json(self.output / "manifest.json")
        self.assertFalse(manifest["complete"])
        self.assertFalse(batch.load_json(self.output / "finalization.json")["training_eligible"])

    def test_failed_audit_not_published(self):
        with self.assertRaises(batch.QualityError):
            batch.collect_job(self.job, self.output, self.limits, batch.DEFAULT_OBSERVATION,
                               adapter=self.adapter({"remaining": 2}))
        self.assertFalse(batch.load_json(self.output / "manifest.json")["complete"])

    def test_resource_failure_marked_stop_batch(self):
        def fail(config):
            raise batch.ResourceError("disk")
            yield
        with self.assertRaises(batch.ResourceError):
            batch.collect_job(self.job, self.output, self.limits, batch.DEFAULT_OBSERVATION, adapter=fail)
        final = batch.load_json(self.output / "finalization.json")
        self.assertTrue(final["resource_failure"])
        self.assertFalse(final["training_eligible"])

    def test_output_hash_tamper_detected(self):
        self.collect()
        (self.output / "sequences.jsonl").write_text("tamper")
        with self.assertRaises(batch.QualityError):
            batch.verify_finalization(self.output)

    def test_supplemental_hash_tamper_detected(self):
        result = self.collect()
        self.assertEqual(set(result["file_sha256"]),
            {"raw_events.jsonl", "sequences.jsonl", "passages.jsonl", "coverage.json"})
        (self.output / "tripinfo.xml").write_text("<tripinfos/>")
        with self.assertRaises(batch.QualityError):
            batch.verify_finalization(self.output)

    def test_parent_quarantine_downgrades_published_manifest(self):
        self.collect()
        batch.quarantine_output(self.output, "own orphan group")
        self.assertFalse(batch.load_json(self.output / "manifest.json")["complete"])
        self.assertFalse(batch.load_json(self.output / "finalization.json")["training_eligible"])


class LabelSupportTests(BatchFixture):
    def collect_prefix(self, passage_times, **kwargs):
        from features.discharge_sequences import EVENT_SCHEMA
        def event(kind, time, **extra):
            return {"schema": EVENT_SCHEMA, "run_id": self.job["run_id"],
                    "event": kind, "time": time, "lane_id": "x_0", "section_id": "lane-end", **extra}
        events = [event("start", 0, anchor_time=0, valid_start=True,
                        start_kind="stopped_green_start", permission_class="protected",
                        cohort=[{"vehicle_key": key, "vehicle_id": key} for key in ("a", "b")])]
        events += [event("passage", time, vehicle_key=key, vehicle_id=key,
                         permission_class="protected", turn="s", type_id="car")
                   for key, time in zip(("a", "b"), passage_times)]
        return batch.collect_job(self.job, self.output, self.limits, batch.DEFAULT_OBSERVATION,
                                 adapter=self.adapter(events=events), **kwargs)

    def rebind(self, manifest):
        for name in manifest["file_sha256"]:
            manifest["file_sha256"][name] = batch.sha256_file(self.output / name)
        batch.write_json(self.output / "manifest.json", manifest)
        final = batch.load_json(self.output / "finalization.json")
        final["manifest_sha256"] = batch.sha256_file(self.output / "manifest.json")
        batch.write_json(self.output / "finalization.json", final)

    def test_zero_passage_start_auditable_but_not_trainable(self):
        result = self.collect_prefix([])
        self.assertEqual(result["label_support"]["sequences"], 1)
        self.assertEqual(result["label_support"]["startup_samples"], 0)
        self.assertEqual(result["status"], "complete_no_valid_labels")
        self.assertTrue(batch.verify_finalization(self.output, require_trainable=False)["complete"])

    def test_valid_zero_startup_delay_is_startup_only(self):
        result = self.collect_prefix([0])
        self.assertEqual(result["label_support"]["startup_samples"], 1)
        self.assertTrue(result["training_eligible"])
        self.assertFalse(result["headway_training_eligible"])
        self.assertTrue(batch.verify_finalization(self.output)["training_eligible"])

    def test_valid_safe_prefix_keeps_headway(self):
        result = self.collect_prefix([1, 3])
        self.assertEqual(result["label_support"]["headway_samples"], 1)
        self.assertTrue(batch.verify_finalization(self.output)["headway_training_eligible"])

    def test_diagnostic_labels_never_make_run_trainable(self):
        result = self.collect_prefix([1, 3], horizon=20)
        self.assertTrue(result["label_support"]["has_headway_labels"])
        self.assertFalse(result["complete"])
        self.assertFalse(result["training_eligible"])
        self.assertFalse(result["headway_training_eligible"])
        with self.assertRaises(batch.QualityError):
            batch.verify_finalization(self.output, require_trainable=False)

    def test_bad_label_not_accepted_even_after_hash_rebinding(self):
        manifest = self.collect_prefix([1, 3])
        path = self.output / "sequences.jsonl"
        row = json.loads(path.read_text())
        row["headways"][0]["headway"] = -2
        path.write_text(json.dumps(row) + "\n", encoding="utf-8")
        self.rebind(manifest)
        with self.assertRaises(ValueError):
            batch.verify_finalization(self.output)

    def test_strict_nonnegative_count_types(self):
        self.collect_prefix([1, 3])
        coverage = batch.load_json(self.output / "coverage.json")
        for bad in (True, False, -1, 1.0, "1", None):
            with self.subTest(bad=bad):
                changed = deepcopy(coverage)
                changed["totals"]["startup_samples"] = bad
                with self.assertRaisesRegex(batch.QualityError, "nonnegative integers"):
                    batch.label_support(self.output, self.job["run_id"], changed)

    def test_missing_count_in_nonempty_totals_rejected(self):
        self.collect_prefix([1])
        coverage = batch.load_json(self.output / "coverage.json")
        del coverage["totals"]["headway_samples"]
        with self.assertRaises(batch.QualityError):
            batch.label_support(self.output, self.job["run_id"], coverage)

    def test_coverage_rehash_cannot_fabricate_labels(self):
        manifest = self.collect()
        coverage = batch.load_json(self.output / "coverage.json")
        coverage["totals"] = dict(sequences=1, zero_passage_sequences=0,
                                  startup_samples=1, headway_samples=1)
        batch.write_json(self.output / "coverage.json", coverage)
        manifest["totals"] = coverage["totals"]
        self.rebind(manifest)
        with self.assertRaisesRegex(batch.QualityError, "final safe sequences"):
            batch.verify_finalization(self.output, require_trainable=False)

    def test_manifest_count_rehash_does_not_override_coverage(self):
        manifest = self.collect_prefix([1])
        manifest["totals"]["headway_samples"] = 1
        self.rebind(manifest)
        with self.assertRaisesRegex(batch.QualityError, "totals differ"):
            batch.verify_finalization(self.output)

    def test_support_or_eligibility_rehash_cannot_override_records(self):
        manifest = self.collect()
        manifest["training_eligible"] = True
        manifest["status"] = "complete"
        self.rebind(manifest)
        with self.assertRaisesRegex(batch.QualityError, "eligibility/status"):
            batch.verify_finalization(self.output, require_trainable=False)

    def test_removed_hash_rejected(self):
        manifest = self.collect_prefix([1])
        del manifest["file_sha256"]["sequences.jsonl"]
        self.rebind(manifest)
        with self.assertRaisesRegex(batch.QualityError, "required output hashes"):
            batch.verify_finalization(self.output)

    def test_parent_excludes_empty_then_accepts_next_legal_run(self):
        self.collect()
        state = {"training_eligible": [], "headway_training_eligible": [], "complete_no_valid_labels": []}
        empty_entry = {}
        batch.record_completed_run(state, empty_entry, self.output)
        self.assertEqual(empty_entry["status"], "complete_no_valid_labels")
        self.assertEqual(state["training_eligible"], [])
        self.assertEqual(state["headway_training_eligible"], [])
        self.assertEqual(state["complete_no_valid_labels"], [str(self.output)])
        self.output = self.root / "next_output"
        self.collect_prefix([1, 3])
        next_entry = {}
        batch.record_completed_run(state, next_entry, self.output)
        self.assertEqual(next_entry["status"], "complete")
        self.assertEqual(state["training_eligible"], [str(self.output)])
        self.assertEqual(state["headway_training_eligible"], [str(self.output)])

    def test_empty_worker_exits_successfully_not_as_failed_run(self):
        args = ["_worker", "--plan", str(self.root / "plan.json"), "--run-id", self.job["run_id"],
                "--output-root", str(self.output)]
        batch.write_json(self.root / "plan.json", self.plan)
        with patch.object(batch, "verify_admission"), \
             patch.object(batch, "collect_job", return_value={"status": "complete_no_valid_labels", "complete": True}):
            self.assertEqual(batch.main(args), 0)


class OwnedGroupTests(unittest.TestCase):
    def setUp(self):
        # Make Linux-only group operations mockable on Windows, never real.
        for target, name, value in ((batch.os, "getpgrp", lambda: 99),
                                    (batch.os, "killpg", lambda *args: None),
                                    (batch.signal, "SIGKILL", 9)):
            patcher = patch.object(target, name, value, create=True)
            patcher.start()
            self.addCleanup(patcher.stop)

    def worker(self):
        process = Mock(pid=456789)
        process.poll.return_value = 0
        process._discharge_owned_session = True
        process._discharge_birth_ticks = 12345
        return process

    def test_exited_leader_does_not_skip_living_children(self):
        process = self.worker()
        with patch.object(batch, "group_alive", side_effect=[True, False]), \
             patch.object(batch, "process_identity", return_value=None), \
             patch.object(batch.os, "getpgrp", return_value=99), \
             patch.object(batch.os, "killpg") as kill:
            self.assertTrue(batch.stop_owned_group(process))
            kill.assert_called_once_with(456789, batch.signal.SIGTERM)

    def test_unowned_group_never_signaled(self):
        process = self.worker()
        process._discharge_owned_session = False
        with patch.object(batch.os, "getpgrp", return_value=99), patch.object(batch.os, "killpg") as kill:
            with self.assertRaises(RuntimeError):
                batch.stop_owned_group(process)
            kill.assert_not_called()

    def test_reused_pid_never_signaled(self):
        process = self.worker()
        with patch.object(batch, "group_alive", return_value=True), \
             patch.object(batch, "process_identity", return_value=(456789, 456789, 99999)), \
             patch.object(batch.os, "getpgrp", return_value=99), patch.object(batch.os, "killpg") as kill:
            with self.assertRaises(RuntimeError):
                batch.stop_owned_group(process)
            kill.assert_not_called()

    def test_normal_empty_group_requires_no_signal(self):
        with patch.object(batch, "group_alive", return_value=False), \
             patch.object(batch.os, "getpgrp", return_value=99), patch.object(batch.os, "killpg") as kill:
            self.assertFalse(batch.stop_owned_group(self.worker()))
            kill.assert_not_called()


class GuardTests(BatchFixture):
    def test_low_disk_guard(self):
        with patch.object(batch.shutil, "disk_usage", return_value=Mock(free=0)):
            with self.assertRaises(batch.ResourceError):
                batch.ensure_disk(self.root, self.limits)

    def test_progress_stagnation_censored_not_deadlock(self):
        limits = {**self.limits, "stall_sim_seconds": 100}
        guard = batch.ProgressGuard(self.root, limits)
        snapshot = {"simulation_time": 1, "running": 1, "remaining": 1,
                    "cumulative_counts": {"departed": 2, "arrived": 1}}
        guard(snapshot)
        with self.assertRaisesRegex(batch.StagnationError, "not proof of deadlock"):
            guard({**snapshot, "simulation_time": 101})

    def test_arrival_resets_stagnation(self):
        limits = {**self.limits, "stall_sim_seconds": 100}
        guard = batch.ProgressGuard(self.root, limits)
        guard({"simulation_time": 1, "running": 1, "cumulative_counts": {"departed": 2, "arrived": 0}})
        guard({"simulation_time": 200, "running": 1, "cumulative_counts": {"departed": 2, "arrived": 1}})

    def test_road_passage_resets_stagnation(self):
        limits = {**self.limits, "stall_sim_seconds": 100}
        guard = batch.ProgressGuard(self.root, limits)
        guard({"simulation_time": 1, "running": 1, "observed_passages": 1, "cumulative_counts": {"departed": 2, "arrived": 0}})
        guard({"simulation_time": 200, "running": 1, "observed_passages": 2, "cumulative_counts": {"departed": 2, "arrived": 0}})

    def test_no_running_vehicles_not_stagnation(self):
        limits = {**self.limits, "stall_sim_seconds": 100}
        guard = batch.ProgressGuard(self.root, limits)
        guard({"simulation_time": 1, "running": 0, "cumulative_counts": {"departed": 0, "arrived": 0}})
        guard({"simulation_time": 200, "running": 0, "cumulative_counts": {"departed": 0, "arrived": 0}})

    def test_walltime_resource_guard(self):
        ticks = iter((0.0, 4000.0))
        limits = {**self.limits, "max_wall_hours": 1}
        guard = batch.ProgressGuard(self.root, limits, clock=lambda: next(ticks))
        with self.assertRaises(batch.ResourceError):
            guard({"simulation_time": 1, "running": 1, "cumulative_counts": {}})

    def test_formal_cli_requires_receipt(self):
        with self.assertRaises(SystemExit):
            batch.parser().parse_args(["run", "--plan", "a", "--output-root", "b"])

    def test_admission_binds_plan_code_and_evidence(self):
        evidence = self.root / "evidence.json"
        evidence.write_text('{"passed":true}')
        receipt = {"schema": "discharge-batch-validation-v1", "passed": True,
                   "plan_sha256": batch.canonical_hash(self.plan), "code_sha256": batch.runtime_code_hashes(),
                   "checks": {name: {"passed": True, "path": str(evidence), "sha256": batch.sha256_file(evidence)}
                              for name in ("unit_tests", "small_network_pilot", "manhattan_network_pilot")}}
        target = self.root / "receipt.json"
        batch.write_json(target, receipt)
        self.assertTrue(batch.verify_admission(target, self.plan)["passed"])
        self.plan["jobs"][0]["planned_seconds"] = 11
        with self.assertRaises(batch.QualityError):
            batch.verify_admission(target, self.plan)

    def test_code_change_invalidates_admission(self):
        receipt = {"schema": "discharge-batch-validation-v1", "passed": True,
                   "plan_sha256": batch.canonical_hash(self.plan), "code_sha256": {"wrong": "0"}}
        target = self.root / "receipt.json"
        batch.write_json(target, receipt)
        with self.assertRaisesRegex(batch.QualityError, "current code"):
            batch.verify_admission(target, self.plan)


if __name__ == "__main__":
    unittest.main()
