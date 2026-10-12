"""Explicit success-only composition of immutable native collection attempts.

This is a logical release: CSV/native bytes remain in physical archived batch
directories. Incomplete source batches are never relabelled as complete.
"""
from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from training.native_contract import sha256_file
from training.native_data import (Batch, BATCH_SCHEMAS, _identifier, _load_batch,
                                  checked_digest, read_json, safe_file, verify_file)

RELEASE_SCHEMA = "native-success-subset-v1"
STATUS_SCHEMA = "native-success-subset-status-v1"


def physical_root(value):
    """Reject aliases before resolve(), including symlinked ancestor directories."""
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError("Release/source root must be an absolute physical path")
    for ancestor in (path, *path.parents):
        if ancestor.is_symlink() or getattr(ancestor, "is_junction", lambda: False)():
            raise ValueError("Symlink in release/source root")
    path = path.resolve(strict=True)
    if not path.is_dir():
        raise ValueError("Release/source root must be a directory")
    return path


def _entries(value, name):
    if not isinstance(value, list) or any(not isinstance(row, dict) for row in value):
        raise ValueError("Invalid release index: " + name)
    return value


def _dataset_index(dataset):
    if (dataset.get("schema") != "manhattan-structured-demand-v1"
            or dataset.get("ready") is not True
            or dataset.get("status") != "DEMAND_GENERATED_NOT_SIMULATED"):
        raise ValueError("Dataset is not a ready structured-demand dataset")
    rows = _entries(dataset.get("scenarios"), "dataset scenarios")
    if not rows:
        raise ValueError("Empty dataset scenario index")
    index, families = {}, {"train": set(), "validation": set()}
    for row in rows:
        sid = _identifier(row.get("scenario_id"))
        if sid in index:
            raise ValueError("Duplicate dataset scenario")
        split = row.get("split")
        if split not in families:
            raise ValueError("Dataset split must be train or validation")
        families[split].add(_identifier(row.get("family_id")))
        if type(row.get("vehicle_count")) is not int or row["vehicle_count"] <= 0:
            raise ValueError("Invalid dataset vehicle count")
        for key in ("network_sha256", "route_sha256", "config_sha256", "scenario_manifest_sha256"):
            checked_digest(row.get(key))
        index[sid] = row
    if not all(families.values()) or families["train"] & families["validation"]:
        raise ValueError("Dataset train/validation families must be nonempty and disjoint")
    if len({row["network_sha256"] for row in rows}) != 1:
        raise ValueError("Mixed networks in dataset manifest")
    return index


def _match_dataset(row, expected):
    for key in ("scenario_id", "split", "family_id", "vehicle_count", "network_sha256"):
        if row.get(key) != expected.get(key):
            raise ValueError("Source scenario differs from dataset: " + key)
    hashes = {"route": expected["route_sha256"], "config": expected["config_sha256"],
              "manifest": expected["scenario_manifest_sha256"]}
    if row.get("hashes") != hashes:
        raise ValueError("Source scenario input digests differ from dataset")


def load_success_release(root, expected_train=None, expected_validation=None,
                         dataset_manifest=None, progress=None, *, require_complete=True):
    """Fully revalidate a published release, or validate a publisher's candidate.

    ``require_complete=False`` omits only the new release's final status marker.
    It is for the publisher before that marker exists, never for model training.
    All selected original worker/audit/native output checks still execute.
    """
    root = physical_root(Path(root).absolute())
    manifest_path = safe_file(root, "release_manifest.json")
    manifest_digest = sha256_file(manifest_path)
    release = read_json(manifest_path)
    if (release.get("schema") != RELEASE_SCHEMA
            or release.get("original_batch_complete") is not False
            or release.get("derived_release_complete") is not True):
        raise ValueError("Not an explicit native success-subset release")
    metadata = {manifest_path: manifest_digest}
    if require_complete:
        status_path = safe_file(root, "status.json")
        metadata[status_path] = sha256_file(status_path)
        status = read_json(status_path)
        if (status.get("schema") != STATUS_SCHEMA or status.get("status") != "COMPLETE"
                or status.get("release_manifest_sha256") != manifest_digest):
            raise ValueError("Success-subset release is not COMPLETE/bound")
    dataset_sha = checked_digest(release.get("dataset_sha256"))
    dataset_path = safe_file(root, "dataset_manifest.json")
    verify_file(dataset_path, dataset_sha)
    metadata[dataset_path] = dataset_sha
    if dataset_manifest is not None:
        verify_file(dataset_manifest, dataset_sha)
    dataset = _dataset_index(read_json(dataset_path))
    sources, roots, attempts, worker_states = {}, set(), {}, {}
    absent_worker_states = []
    shared_identity = None
    for source in _entries(release.get("sources"), "sources"):
        source_id = _identifier(source.get("source_id"))
        source_root = physical_root(source.get("root", ""))
        if source_id in sources or source_root in roots or source_root == root:
            raise ValueError("Duplicate or recursive source batch")
        roots.add(source_root)
        source_manifest = safe_file(source_root, "batch_manifest.json")
        source_status = safe_file(source_root, "status.json")
        for path, field in ((source_manifest, "batch_manifest_sha256"),
                            (source_status, "batch_status_sha256")):
            verify_file(path, source.get(field))
            metadata[path] = source[field]
        original = read_json(source_manifest)
        contract = original.get("contract", {})
        if (original.get("schema") not in BATCH_SCHEMAS or contract.get("pilot") is not False
                or contract.get("dataset_sha256") != dataset_sha):
            raise ValueError("Source batch is not the same non-pilot dataset")
        identity = {key: contract.get(key) for key in
                    ("collector_sources_sha256", "collector_sha256", "runner_sha256")}
        if shared_identity is not None and identity != shared_identity:
            raise ValueError("Mixed collection code identities across source batches")
        shared_identity = identity
        rows = _entries(contract.get("scenarios"), "source scenarios")
        ids = [_identifier(row.get("scenario_id")) for row in rows]
        if not rows or len(set(ids)) != len(ids) or contract.get("scenario_ids") != ids:
            raise ValueError("Invalid source scenario index")
        for row in rows:
            sid = row["scenario_id"]
            if sid not in dataset:
                raise ValueError("Source scenario absent from original dataset")
            _match_dataset(row, dataset[sid])
            key = (source_id, sid)
            attempts[key] = row
            relative = "jobs/" + sid + "/status.json"
            if (source_root / relative).exists():
                worker_path = safe_file(source_root, relative)
                metadata[worker_path] = sha256_file(worker_path)
                worker_states[key] = read_json(worker_path).get("state")
            else:
                worker_states[key] = None
                absent_worker_states.append(source_root / relative)
        sources[source_id] = source_root
    if not sources or {sid for _, sid in attempts} != set(dataset):
        raise ValueError("Declared source batches do not cover the original dataset")
    selected, selected_ids, excluded = set(), set(), set()
    for entry in _entries(release.get("selected"), "selected"):
        key = (_identifier(entry.get("source_id")), _identifier(entry.get("scenario_id")))
        if key not in attempts or key in selected or key[1] in selected_ids:
            raise ValueError("Unknown or duplicate selected scenario")
        if worker_states[key] != "COMPLETE":
            raise ValueError("Selected worker is not COMPLETE")
        selected.add(key)
        selected_ids.add(key[1])
    for entry in _entries(release.get("excluded"), "excluded"):
        key = (_identifier(entry.get("source_id")), _identifier(entry.get("scenario_id")))
        if key not in attempts or key in selected or key in excluded:
            raise ValueError("Unknown, duplicate or overlapping excluded attempt")
        if not isinstance(entry.get("reason"), str) or not entry["reason"].strip():
            raise ValueError("Excluded attempt needs an explicit reason")
        excluded.add(key)
    if selected | excluded != set(attempts):
        raise ValueError("Every declared source attempt must be selected or excluded")
    completed = [key for key, state in worker_states.items() if state == "COMPLETE"]
    if len({sid for _, sid in completed}) != len(completed):
        raise ValueError("Multiple COMPLETE attempts for one scenario; explicit adjudication required")
    if set(completed) != selected:
        raise ValueError("Release must contain every unique COMPLETE source attempt")
    for source_id, sid in selected:
        for relative in ("job.json", "result/audit.json", "result/manifest.json"):
            path = safe_file(sources[source_id], "jobs/" + sid + "/" + relative)
            metadata[path] = sha256_file(path)
    records, runtime, csv_hashes, source_provenance = [], None, set(), {}
    for source_id, source_root in sources.items():
        subset = {sid for candidate, sid in selected if candidate == source_id}
        batch = _load_batch(source_root, progress=progress, selected_ids=subset)
        source_provenance[source_id] = batch.provenance
        for scenario in batch.scenarios:
            identity = (batch.provenance["network_sha256"], batch.provenance["sumo_version"],
                        batch.provenance["step_seconds"], batch.collection_contract)
            if runtime is not None and runtime != identity:
                raise ValueError("Mixed network/runtime/collection semantics in release")
            runtime = identity
            if scenario.training_sha256 in csv_hashes:
                raise ValueError("Duplicate training file bytes across release scenarios")
            csv_hashes.add(scenario.training_sha256)
            records.append(replace(scenario, evidence={**scenario.evidence,
                "source_id": source_id, "source_batch_root": str(source_root),
                "source_batch_contract_sha256": batch.provenance["batch_contract_sha256"]}))
    families = {split: {row.family_id for row in records if row.split == split}
                for split in ("train", "validation")}
    if not all(families.values()) or families["train"] & families["validation"]:
        raise ValueError("Release train/validation families must be nonempty and disjoint")
    counts = {split: sum(row.split == split for row in records) for split in families}
    expected_counts = {**counts, "selected": len(records),
                       "excluded_scenarios": len(set(dataset) - selected_ids)}
    if (release.get("expected_counts") != expected_counts
            or any(type(value) is not int for value in release["expected_counts"].values())):
        raise ValueError("Success release count contract does not match its records")
    for split, expected in (("train", expected_train), ("validation", expected_validation)):
        if expected is not None and counts[split] != expected:
            raise ValueError("Unexpected " + split + " scenario count")
    for path, digest in metadata.items():
        verify_file(path, digest)
    for path in absent_worker_states:
        if path.exists() or path.is_symlink():
            raise ValueError("Previously absent worker status appeared during verification")
    # Preserve deterministic manifest selection order, not directory or set order.
    order = {row["scenario_id"]: index for index, row in enumerate(release["selected"])}
    records.sort(key=lambda row: order[row.scenario_id])
    return Batch(root, tuple(records), runtime[3], {
        "release_schema": RELEASE_SCHEMA, "release_manifest_sha256": manifest_digest,
        "original_batch_complete": False, "derived_release_complete": True,
        "dataset_sha256": dataset_sha, "collector_sources_sha256": shared_identity["collector_sources_sha256"],
        "network_sha256": runtime[0], "sumo_version": runtime[1], "step_seconds": runtime[2],
        "split_counts": counts, "families": {key: sorted(value) for key, value in families.items()},
        "sources": source_provenance, "selected": release["selected"], "excluded": release["excluded"],
        "expected_counts": expected_counts})
