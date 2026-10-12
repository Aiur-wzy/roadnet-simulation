"""Success-only release safety tests; synthetic bytes, no SUMO or model fitting."""
from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from training import native_contract as contract
from training import native_data as data
from training import native_release as release
from test_native_training import ReleaseFixture, write_json


def refresh_contract(fixture):
    """Re-sign synthetic fixture evidence after a deliberate fixture change."""
    batch = data.read_json(fixture.root / "batch_manifest.json")
    digest = contract.canonical_digest(batch["contract"])
    for row in batch["contract"]["scenarios"]:
        sid = row["scenario_id"]
        spec_path = fixture.root / "jobs" / sid / "job.json"
        spec = data.read_json(spec_path)
        spec.update(contract_sha256=digest, source_scenario=row,
                    collector_sources_sha256=batch["contract"]["collector_sources_sha256"],
                    collector_sha256=batch["contract"]["collector_sha256"],
                    runner_sha256=batch["contract"]["runner_sha256"])
        write_json(spec_path, spec)
        fixture.edit("jobs/" + sid + "/status.json",
                     lambda value: value.update(contract_sha256=digest))
        fixture.rebind(sid)


def keep_scenarios(fixture, identifiers):
    wanted = set(identifiers)
    batch = data.read_json(fixture.root / "batch_manifest.json")
    batch["contract"]["scenarios"] = [row for row in batch["contract"]["scenarios"]
                                         if row["scenario_id"] in wanted]
    batch["contract"]["scenario_ids"] = [row["scenario_id"] for row in batch["contract"]["scenarios"]]
    write_json(fixture.root / "batch_manifest.json", batch)
    fixture.edit("status.json", lambda value: value.update(scenarios={sid: "COMPLETE" for sid in identifiers}))
    refresh_contract(fixture)


class SuccessReleaseFixture:
    def __init__(self, root, large=False):
        self.root = Path(root)
        self.root.mkdir()
        counts = {"train_count": 36, "validation_count": 6} if large else {"train_count": 2, "validation_count": 1}
        self.first = ReleaseFixture(self.root / "first", **counts)
        self.replay = ReleaseFixture(self.root / "replay", **counts)
        rows = data.read_json(self.first.root / "batch_manifest.json")["contract"]["scenarios"]
        self.all_rows = copy.deepcopy(rows)
        self.replay_sid = "validation_5" if large else "validation_0"
        self.censored = {"train_" + str(i) for i in (range(31, 36) if large else [1])}
        keep_scenarios(self.replay, [self.replay_sid])
        state = data.read_json(self.first.root / "status.json")
        state.update(status="FAILED", active=[self.replay_sid], failures=sorted(self.censored))
        for sid in self.censored:
            state["scenarios"][sid] = "CENSORED"
            self.first.edit("jobs/" + sid + "/status.json",
                            lambda value: value.update(state="CENSORED", returncode=2, audit_status="CENSORED"))
        state["scenarios"][self.replay_sid] = "RUNNING"
        self.first.edit("jobs/" + self.replay_sid + "/status.json",
                        lambda value: value.update(state="RUNNING", returncode=None))
        write_json(self.first.root / "status.json", state)
        self.destination = self.root / "unified"
        self.destination.mkdir()
        dataset_rows = []
        for row in rows:
            dataset_rows.append({key: row[key] for key in ("scenario_id", "family_id", "split", "vehicle_count", "network_sha256")}
                                | {"route_sha256": row["hashes"]["route"], "config_sha256": row["hashes"]["config"],
                                   "scenario_manifest_sha256": row["hashes"]["manifest"]})
        self.dataset = {"schema": "manhattan-structured-demand-v1", "ready": True,
                        "status": "DEMAND_GENERATED_NOT_SIMULATED", "scenarios": dataset_rows,
                        "inputs": {"network": {"sha256": "c" * 64}}}
        write_json(self.destination / "dataset_manifest.json", self.dataset)
        self.sources = {"first": self.first, "replay": self.replay}
        self.bind_dataset()
        self.build_release()

    def bind_dataset(self):
        dataset_sha = contract.sha256_file(self.destination / "dataset_manifest.json")
        for fixture in self.sources.values():
            fixture.edit("batch_manifest.json", lambda value: value["contract"].update(dataset_sha256=dataset_sha))
            refresh_contract(fixture)

    def build_release(self):
        sources, selected, excluded = [], [], []
        for source_id, fixture in self.sources.items():
            sources.append({"source_id": source_id, "root": str(fixture.root.resolve()),
                            "batch_manifest_sha256": contract.sha256_file(fixture.root / "batch_manifest.json"),
                            "batch_status_sha256": contract.sha256_file(fixture.root / "status.json")})
            rows = data.read_json(fixture.root / "batch_manifest.json")["contract"]["scenarios"]
            for row in rows:
                sid = row["scenario_id"]
                member = {"scenario_id": sid, "source_id": source_id}
                if source_id == "first" and (sid in self.censored or sid == self.replay_sid):
                    excluded.append(member | {"reason": "censored" if sid in self.censored else "historical_interrupted_replaced"})
                else:
                    selected.append(member)
        selected.sort(key=lambda row: row["scenario_id"])
        counts = {"train": sum(row["scenario_id"].startswith("train_") for row in selected),
                  "validation": sum(row["scenario_id"].startswith("validation_") for row in selected),
                  "selected": len(selected), "excluded_scenarios": len(self.censored)}
        manifest = {"schema": "native-success-subset-v1", "sources": sources, "selected": selected,
                    "excluded": excluded, "dataset_sha256": contract.sha256_file(self.destination / "dataset_manifest.json"),
                    "expected_counts": counts, "original_batch_complete": False, "derived_release_complete": True}
        write_json(self.destination / "release_manifest.json", manifest)
        self.rebind_release_status()

    def rebind_release_status(self, status="COMPLETE"):
        write_json(self.destination / "status.json", {"schema": "native-success-subset-status-v1", "status": status,
                   "release_manifest_sha256": contract.sha256_file(self.destination / "release_manifest.json")})

    def change_manifest(self, edit):
        path = self.destination / "release_manifest.json"
        value = data.read_json(path)
        edit(value)
        write_json(path, value)
        self.rebind_release_status()


class NativeSuccessReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="native_success_release_")
        self.fixture = SuccessReleaseFixture(Path(self.temp.name) / "fixture")

    def tearDown(self):
        self.temp.cleanup()

    def load(self, **kwargs):
        return data.load_batch(self.fixture.destination, **kwargs)

    def test_failed_source_successes_and_only_validation_replay_combine(self):
        batch = self.load(expected_train=1, expected_validation=1)
        self.assertEqual([row.scenario_id for row in batch.scenarios], ["train_0", "validation_0"])
        self.assertEqual(batch.provenance["split_counts"], {"train": 1, "validation": 1})
        self.assertEqual(data.read_json(self.fixture.first.root / "status.json")["status"], "FAILED")
        self.assertEqual(data.read_json(self.fixture.first.root / "jobs/validation_0/status.json")["state"], "RUNNING")

    def test_large_31_train_6_validation_5_excluded(self):
        other = SuccessReleaseFixture(Path(self.temp.name) / "large", large=True)
        batch = data.load_batch(other.destination, expected_train=31, expected_validation=6)
        self.assertEqual(len(batch.scenarios), 37)
        self.assertTrue(other.censored.isdisjoint(row.scenario_id for row in batch.scenarios))

    def test_load_and_full_scan_do_not_copy_or_modify_source_bytes(self):
        before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                  for fixture in self.fixture.sources.values() for path in fixture.root.rglob("*") if path.is_file()}
        batch = self.load()
        result = data.preflight(batch, chunk_size=1)
        self.assertEqual(result["status"], "PREFLIGHT_COMPLETE")
        self.assertEqual(len(result["scenarios"]), 2)
        after = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                 for fixture in self.fixture.sources.values() for path in fixture.root.rglob("*") if path.is_file()}
        self.assertEqual(before, after)
        self.assertEqual(sorted(path.name for path in self.fixture.destination.iterdir()),
                         ["dataset_manifest.json", "release_manifest.json", "status.json"])
        self.assertTrue(all(self.fixture.destination not in row.training_path.parents for row in batch.scenarios))

    def test_repeated_load_has_stable_scenario_order(self):
        one, two = self.load(), self.load()
        self.assertEqual([row.scenario_id for row in one.scenarios], [row.scenario_id for row in two.scenarios])
        self.assertEqual(one.provenance, two.provenance)

    def test_three_sources_with_empty_success_source_are_supported(self):
        continuation = ReleaseFixture(self.fixture.root / "continuation", train_count=2, validation_count=1)
        keep_scenarios(continuation, ["train_0"])
        self.fixture.sources["continuation"] = continuation
        self.fixture.first.edit("jobs/train_0/status.json", lambda value: value.update(state="PENDING", returncode=None))
        self.fixture.first.edit("status.json", lambda value: value["scenarios"].update(train_0="PENDING"))
        self.fixture.bind_dataset()
        self.fixture.build_release()
        self.fixture.change_manifest(lambda value: value["selected"].__setitem__(slice(None),
            [row for row in value["selected"] if row["scenario_id"] != "train_0" or row["source_id"] == "continuation"]))
        def exclude_old(value):
            value["excluded"].append({"scenario_id": "train_0", "source_id": "first", "reason": "historical_pending"})
            value["expected_counts"].update(train=1, validation=1, selected=2, excluded_scenarios=1)
        self.fixture.change_manifest(exclude_old)
        batch = self.load()
        self.assertEqual([row.scenario_id for row in batch.scenarios], ["train_0", "validation_0"])
        self.assertEqual(batch.scenarios[0].evidence["source_id"], "continuation")
        self.assertEqual(len(batch.provenance["sources"]), 3)

    def test_source_metadata_mutation_mid_load_is_rejected(self):
        original = release._load_batch
        def mutate(*args, **kwargs):
            result = original(*args, **kwargs)
            path = self.fixture.first.root / "status.json"
            path.write_bytes(path.read_bytes() + b"\n")
            return result
        with patch("training.native_release._load_batch", side_effect=mutate):
            with self.assertRaises(ValueError):
                self.load()

    def test_every_selected_closed_output_is_hashed(self):
        with patch("training.native_data.verify_file", wraps=data.verify_file) as verify:
            self.load()
        verified = {Path(call.args[0]) for call in verify.call_args_list}
        for fixture, sid in ((self.fixture.first, "train_0"), (self.fixture.replay, "validation_0")):
            result = fixture.root / "jobs" / sid / "result"
            for name in data.read_json(result / "audit.json")["closed_outputs_sha256"]:
                self.assertIn(result / name, verified)

    def test_duplicate_selected_scenario_rejected(self):
        self.fixture.change_manifest(lambda value: value["selected"].append(copy.deepcopy(value["selected"][0])))
        with self.assertRaises(ValueError):
            self.load()

    def test_same_scenario_selected_from_two_attempts_rejected(self):
        def change(value):
            value["excluded"] = [row for row in value["excluded"] if row["scenario_id"] != "validation_0"]
            value["selected"].append({"scenario_id": "validation_0", "source_id": "first"})
        self.fixture.change_manifest(change)
        with self.assertRaises(ValueError):
            self.load()

    def test_censored_selection_rejected(self):
        def change(value):
            value["excluded"] = [row for row in value["excluded"] if row["scenario_id"] != "train_1"]
            value["selected"].append({"scenario_id": "train_1", "source_id": "first"})
            value["expected_counts"].update(train=2, selected=3, excluded_scenarios=0)
        self.fixture.change_manifest(change)
        with self.assertRaises(ValueError):
            self.load()

    def test_omitted_attempt_cannot_silently_disappear(self):
        self.fixture.change_manifest(lambda value: value["excluded"].pop())
        with self.assertRaises(ValueError):
            self.load()

    def test_unknown_source_reference_rejected(self):
        self.fixture.change_manifest(lambda value: value["selected"][0].update(source_id="missing"))
        with self.assertRaises(ValueError):
            self.load()

    def test_wrong_expected_counts_rejected(self):
        self.fixture.change_manifest(lambda value: value["expected_counts"].update(train=31))
        with self.assertRaises(ValueError):
            self.load()

    def test_changed_source_status_hash_rejected(self):
        path = self.fixture.first.root / "status.json"
        path.write_bytes(path.read_bytes() + b"\n")
        with self.assertRaises(ValueError):
            self.load()

    def test_changed_selected_output_hash_rejected(self):
        path = self.fixture.first.root / "jobs/train_0/result/tripinfo.xml"
        path.write_bytes(b"tampered")
        with self.assertRaises(ValueError):
            self.load()

    def test_partial_training_output_not_accepted(self):
        path = self.fixture.first.root / "jobs/train_0/result/training.csv.gz"
        path.rename(path.with_name("training.csv.gz.partial"))
        with self.assertRaises((ValueError, FileNotFoundError)):
            self.load()

    def test_unclosed_outputs_not_accepted_even_with_rebound_hash(self):
        self.fixture.first.edit("jobs/train_0/result/audit.json", lambda value: value.update(native_outputs_closed=False))
        self.fixture.first.rebind("train_0")
        with self.assertRaises(ValueError):
            self.load()

    def test_dataset_change_rejected(self):
        path = self.fixture.destination / "dataset_manifest.json"
        path.write_bytes(path.read_bytes() + b"\n")
        with self.assertRaises(ValueError):
            self.load()

    def test_source_dataset_identity_disagreement_rejected(self):
        self.fixture.replay.edit("batch_manifest.json", lambda value: value["contract"]["scenarios"][0].update(family_id="unexpected"))
        refresh_contract(self.fixture.replay)
        self.fixture.build_release()
        with self.assertRaises(ValueError):
            self.load()

    def test_cross_source_family_leakage_rejected(self):
        self.fixture.dataset["scenarios"][-1]["family_id"] = "train_family"
        write_json(self.fixture.destination / "dataset_manifest.json", self.fixture.dataset)
        for source in self.fixture.sources.values():
            def change(value):
                for row in value["contract"]["scenarios"]:
                    if row["split"] == "validation":
                        row["family_id"] = "train_family"
            source.edit("batch_manifest.json", change)
        self.fixture.bind_dataset()
        self.fixture.build_release()
        with self.assertRaises(ValueError):
            self.load()

    def test_mixed_collector_code_rejected(self):
        source = self.fixture.replay
        def change(value):
            value["contract"]["collector_sha256"] = "9" * 64
            value["contract"]["collector_sources_sha256"]["collection/native_boundary_v2/collect_native_v2.py"] = "9" * 64
        source.edit("batch_manifest.json", change)
        source.edit("jobs/validation_0/result/manifest.json", lambda value: value["sources"].update(
            {"collection/native_boundary_v2/collect_native_v2.py": "9" * 64}))
        refresh_contract(source)
        self.fixture.build_release()
        with self.assertRaises(ValueError):
            self.load()

    def test_mixed_runtime_rejected(self):
        self.fixture.replay.edit("jobs/validation_0/result/manifest.json", lambda value: value.update(sumo_version=[22, "SUMO other"]))
        self.fixture.replay.rebind("validation_0")
        with self.assertRaises(ValueError):
            self.load()

    def test_mixed_network_rejected(self):
        source = self.fixture.replay
        source.edit("batch_manifest.json", lambda value: value["contract"]["scenarios"][0].update(network_sha256="8" * 64))
        refresh_contract(source)
        self.fixture.build_release()
        with self.assertRaises(ValueError):
            self.load()

    def test_release_must_be_complete_for_training(self):
        self.fixture.rebind_release_status("PUBLISHING")
        with self.assertRaises(ValueError):
            self.load()

    def test_release_manifest_tampering_rejected(self):
        path = self.fixture.destination / "release_manifest.json"
        path.write_bytes(path.read_bytes() + b"\n")
        with self.assertRaises(ValueError):
            self.load()

    def test_output_symlink_rejected(self):
        path = self.fixture.first.root / "jobs/train_0/result/tripinfo.xml"
        target = path.with_name("physical-tripinfo.xml")
        path.rename(target)
        try:
            path.symlink_to(target)
        except (OSError, NotImplementedError) as exc:
            self.skipTest("File symlink unavailable: " + str(exc))
        with self.assertRaises(ValueError):
            self.load()

    def test_source_root_symlink_rejected(self):
        link = self.fixture.root / "source_link"
        try:
            link.symlink_to(self.fixture.first.root, target_is_directory=True)
        except (OSError, NotImplementedError) as exc:
            self.skipTest("Directory symlink unavailable: " + str(exc))
        self.fixture.change_manifest(lambda value: value["sources"][0].update(root=str(link)))
        with self.assertRaises(ValueError):
            self.load()


class NativeReleasePublisherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        staging = Path(__file__).resolve().parents[2]
        publisher_path = staging / "tools/maintenance/publish_native_release.py"
        spec = importlib.util.spec_from_file_location("native_release_publisher_under_test", publisher_path)
        cls.publisher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.publisher)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="native_release_publish_")
        self.fixture = SuccessReleaseFixture(Path(self.temp.name) / "fixture")
        self.output = Path(self.temp.name) / "published"

    def tearDown(self):
        self.temp.cleanup()

    def args(self, publish=False):
        args = []
        for name, fixture in self.fixture.sources.items():
            args.extend(["--source", name, str(fixture.root)])
        args.extend(["--dataset-manifest", str(self.fixture.destination / "dataset_manifest.json"),
                     "--output-root", str(self.output), "--expected-train", "1",
                     "--expected-validation", "1", "--expected-excluded", "1"])
        if publish:
            args.append("--publish")
        return self.publisher.build_parser().parse_args(args)

    def test_default_dryrun_does_not_create_output_or_claim_full_validation(self):
        result = self.publisher.run(self.args())
        self.assertFalse(self.output.exists())
        self.assertEqual(result["status"], "DRY_RUN_PLAN")
        self.assertIs(result["full_output_hashes_verified"], False)
        self.assertIs(result["training_released"], False)
        self.assertEqual(len(result["manifest"]["selected"]), 2)
        self.assertEqual(len(result["manifest"]["excluded"]), 2)

    def test_publication_runs_full_hashes_and_creates_only_metadata(self):
        originals = {str(path): contract.sha256_file(path) for fixture in self.fixture.sources.values()
                     for path in fixture.root.rglob("*") if path.is_file()}
        result = self.publisher.run(self.args(publish=True))
        self.assertEqual(result["status"], "COMPLETE")
        self.assertIs(result["full_output_hashes_verified"], True)
        self.assertIs(result["training_started"], False)
        self.assertEqual(result["summary"]["by_split"]["train"]["scenarios"], 1)
        self.assertEqual(result["summary"]["by_split"]["train"]["vehicles"], 1)
        self.assertEqual(result["summary"]["by_split"]["train"]["training_csv_rows_before_positive_label_policy"], 3)
        self.assertEqual(data.read_json(self.output / "release_manifest.json")["expected_counts"],
                         {"train": 1, "validation": 1, "selected": 2, "excluded_scenarios": 1})
        self.assertEqual((self.output / "dataset_manifest.json").read_bytes(),
                         (self.fixture.destination / "dataset_manifest.json").read_bytes())
        self.assertFalse(list(self.output.rglob("*.gz")))
        for path, digest in originals.items():
            self.assertEqual(contract.sha256_file(path), digest)
        self.assertEqual(len(data.load_batch(self.output).scenarios), 2)

    def test_full_hash_failure_never_publishes_complete_or_modifies_sources(self):
        path = self.fixture.first.root / "jobs/train_0/result/tripinfo.xml"
        path.write_bytes(b"changed after source audit")
        with self.assertRaises(ValueError):
            self.publisher.run(self.args(publish=True))
        self.assertEqual(data.read_json(self.output / "status.json")["status"], "FAILED")
        self.assertEqual(path.read_bytes(), b"changed after source audit")
        with self.assertRaises(ValueError):
            data.load_batch(self.output)

    def test_existing_output_is_never_overwritten(self):
        self.output.mkdir()
        sentinel = self.output / "sentinel"
        sentinel.write_bytes(b"keep")
        with self.assertRaises(ValueError):
            self.publisher.run(self.args(publish=True))
        self.assertEqual(sentinel.read_bytes(), b"keep")
        self.assertEqual(list(self.output.iterdir()), [sentinel])

    def test_multiple_complete_attempts_require_adjudication(self):
        self.fixture.first.edit("jobs/validation_0/status.json", lambda value: value.update(state="COMPLETE", returncode=0))
        with self.assertRaisesRegex(ValueError, "Multiple COMPLETE"):
            self.publisher.run(self.args())
        self.assertFalse(self.output.exists())

    def test_source_metadata_change_during_full_verification_rejected(self):
        original = release.load_success_release
        def change_then_return(*args, **kwargs):
            result = original(*args, **kwargs)
            path = self.fixture.first.root / "status.json"
            path.write_bytes(path.read_bytes() + b"\n")
            return result
        with patch("training.native_release.load_success_release", side_effect=change_then_return):
            with self.assertRaises(ValueError):
                self.publisher.run(self.args(publish=True))
        self.assertEqual(data.read_json(self.output / "status.json")["status"], "FAILED")

    def test_publisher_requires_whole_dataset_attempt_accounting(self):
        args = self.args()
        args.source = [["replay", str(self.fixture.replay.root)]]
        with self.assertRaises(ValueError):
            self.publisher.run(args)
        self.assertFalse(self.output.exists())

    def test_previously_absent_worker_status_appearing_is_rejected(self):
        path = self.fixture.first.root / "jobs/train_1/status.json"
        payload = path.read_bytes()
        path.rename(path.with_name("historical_status.json"))
        original = release.load_success_release
        def create_then_return(*args, **kwargs):
            result = original(*args, **kwargs)
            path.write_bytes(payload)
            return result
        with patch("training.native_release.load_success_release", side_effect=create_then_return):
            with self.assertRaises(ValueError):
                self.publisher.run(self.args(publish=True))
        self.assertEqual(data.read_json(self.output / "status.json")["status"], "FAILED")


if __name__ == "__main__":
    unittest.main()
