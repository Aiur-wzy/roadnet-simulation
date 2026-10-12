"""Bounded native-v2 release verification and six-target data preflight."""
from __future__ import annotations

import csv
from dataclasses import dataclass
import gzip
import json
import math
from pathlib import Path, PurePosixPath
import re
import time

import numpy as np
import pandas as pd

from training.native_contract import (CLOCK_SCHEMA, FEATURE_COLUMNS, LABEL_COLUMNS,
    SOURCE_SCHEMA, canonical_digest, encode_features, sha256_file, validate_collection_contract)

# Both describe the same current native release structure. Network name/count is
# not part of the data semantics: a small-network batch uses the generic tag.
BATCH_SCHEMAS = {"native-collection-batch-v2", "manhattan-native-collection-batch-v2"}
FLAGS = ["trainable", "label_valid", "feature_valid", "v2_label_valid", "v2_trainable_observed_snapshot"]
META_COLUMNS = ["schema", "vehicle_id", "route_index", "is_final_road", *FLAGS,
    "training_release_scope", "snapshot_clock_schema", "signal_clock_schema",
    "snapshot_clock_step_seconds", "signal_step_seconds", "signal_api_return_time",
    "signal_snapshot_time", "observed_enter_time", "signal_reference_time",
    "signal_projection_seconds", "v2_enter", "v2_exit", "feature_snapshot_lag_seconds"]
READ_COLUMNS = FEATURE_COLUMNS + LABEL_COLUMNS + META_COLUMNS


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def read_json(path):
    with Path(path).open(encoding="utf-8-sig") as stream:
        return json.load(stream, object_pairs_hook=_unique_object,
                         parse_constant=lambda value: (_ for _ in ()).throw(ValueError("Invalid JSON " + value)))


def checked_digest(value):
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise ValueError("Missing or invalid SHA256")
    return value


def verify_file(path, expected):
    if sha256_file(path) != checked_digest(expected):
        raise ValueError("SHA256 mismatch: " + str(path))


def safe_file(root, relative):
    if not isinstance(relative, str) or "\\" in relative or ":" in relative:
        raise ValueError("Unsafe release path")
    parts = PurePosixPath(relative).parts
    if (not parts or relative != "/".join(parts) or parts[0] == "/"
            or any(part in {".", "..", ""} for part in parts)):
        raise ValueError("Release path escapes root")
    path = Path(root)
    for part in parts:
        path /= part
        if path.is_symlink():
            raise ValueError("Symlink in release path")
    path = path.resolve(strict=True)
    path.relative_to(Path(root).resolve(strict=True))
    if not path.is_file():
        raise ValueError("Release member is not a file")
    return path


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", value):
        raise ValueError("Invalid scenario/family identifier")
    return value


def _hash_map(value, name):
    if not isinstance(value, dict) or not value:
        raise ValueError("Missing digest map: " + name)
    for key, digest in value.items():
        if not isinstance(key, str) or not key:
            raise ValueError("Invalid digest-map key")
        checked_digest(digest)
    return value


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    family_id: str
    split: str
    training_path: Path
    training_sha256: str
    expected_rows: int
    expected_vehicles: int
    step_seconds: float
    evidence: dict


@dataclass(frozen=True)
class Batch:
    root: Path
    scenarios: tuple
    collection_contract: dict
    provenance: dict


def load_batch(batch_root, expected_train=None, expected_validation=None, dataset_manifest=None, progress=None):
    """Load a native batch or a separately audited, cross-batch success release."""
    if (Path(batch_root) / "release_manifest.json").exists():
        from training.native_release import load_success_release
        return load_success_release(batch_root, expected_train, expected_validation,
                                    dataset_manifest, progress)
    return _load_batch(batch_root, expected_train, expected_validation, dataset_manifest, progress)


def _load_batch(batch_root, expected_train=None, expected_validation=None, dataset_manifest=None,
                progress=None, *, selected_ids=None):
    """Validate immutable release evidence; never resolve recorded remote paths.

    All closed output bytes are rehashed. Input/source hashes are compared to the
    frozen worker bindings rather than to a potentially newer local checkout.
    An optional relocated dataset_manifest additionally verifies its file bytes.
    """
    root = Path(batch_root).resolve(strict=True)
    manifest_path = safe_file(root, "batch_manifest.json")
    batch = read_json(manifest_path)
    schema = batch.get("schema")
    if schema not in BATCH_SCHEMAS:
        raise ValueError("Not a current native collection batch")
    contract = batch.get("contract", {})
    if contract.get("pilot") is not False:
        raise ValueError("Pilot or unspecified-purpose batch cannot train a model")
    digest = canonical_digest(contract)
    checked_digest(contract.get("dataset_sha256"))
    if dataset_manifest is not None:
        verify_file(dataset_manifest, contract["dataset_sha256"])
    sources = _hash_map(contract.get("collector_sources_sha256"), "collector sources")
    collector_key = "collection/native_boundary_v2/collect_native_v2.py"
    if sources.get(collector_key) != checked_digest(contract.get("collector_sha256")):
        raise ValueError("Collector identity differs from frozen source bundle")
    checked_digest(contract.get("runner_sha256"))
    rows = contract.get("scenarios")
    if not isinstance(rows, list) or not rows:
        raise ValueError("Empty batch scenario index")
    ids = [_identifier(row.get("scenario_id")) for row in rows]
    if len(set(ids)) != len(ids) or contract.get("scenario_ids") != ids:
        raise ValueError("Duplicate or inconsistent batch scenario index")
    declared_families = {"train": set(), "validation": set()}
    for row in rows:
        split = row.get("split")
        if split not in declared_families:
            raise ValueError("Scenario split must explicitly be train or validation")
        declared_families[split].add(_identifier(row.get("family_id")))
    if ((selected_ids is None and not all(declared_families.values()))
            or declared_families["train"] & declared_families["validation"]):
        raise ValueError("Train/validation families must be nonempty and disjoint")
    for split, expected in (("train", expected_train), ("validation", expected_validation)):
        if expected is not None and sum(row["split"] == split for row in rows) != expected:
            raise ValueError("Unexpected " + split + " scenario count")
    state_path = safe_file(root, "status.json")
    state = read_json(state_path)
    if selected_ids is None and (state.get("schema") != schema or state.get("status") != "COMPLETE"
            or state.get("pilot") is not False or state.get("active") != []
            or state.get("pending") != [] or state.get("failures") != []
            or state.get("scenarios") != {sid: "COMPLETE" for sid in ids}):
        raise ValueError("Entire batch must be COMPLETE with no running/pending/failed jobs")
    if selected_ids is not None:
        if not isinstance(selected_ids, set) or selected_ids - set(ids):
            raise ValueError("Unknown or invalid success-subset selection")
        if state.get("schema") != schema or state.get("pilot") is not False:
            raise ValueError("Source state does not bind this native non-pilot batch")
        rows = [row for row in rows if row["scenario_id"] in selected_ids]
    families = {"train": set(), "validation": set()}
    split_counts = {"train": 0, "validation": 0}
    contracts, run_identity, records, csv_hashes = [], None, [], set()
    for row in rows:
        sid, family, split = row["scenario_id"], _identifier(row.get("family_id")), row.get("split")
        started = time.monotonic()
        if progress:
            progress("VERIFY scenario=" + sid + " start")
        if split not in families:
            raise ValueError("Scenario split must explicitly be train or validation")
        families[split].add(family)
        split_counts[split] += 1
        expected = row.get("vehicle_count")
        if type(expected) is not int or expected <= 0:
            raise ValueError("Invalid scenario vehicle count")
        hashes = _hash_map(row.get("hashes"), "scenario inputs")
        if set(hashes) != {"config", "route", "manifest"}:
            raise ValueError("Incomplete scenario input digests")
        network_sha = checked_digest(row.get("network_sha256"))
        job_root = root / "jobs" / sid
        spec_path = safe_file(root, "jobs/" + sid + "/job.json")
        status_path = safe_file(root, "jobs/" + sid + "/status.json")
        metadata_before = {path: sha256_file(path) for path in (spec_path, status_path)}
        spec, status = read_json(spec_path), read_json(status_path)
        verify_file(spec_path, status.get("job_spec_sha256"))
        if (status.get("state") != "COMPLETE" or type(status.get("returncode")) is not int or status["returncode"] != 0
                or status.get("audit_status") != "COMPLETE" or status.get("gate_problems") != []
                or status.get("scenario_id") != sid or status.get("contract_sha256") != digest):
            raise ValueError("Worker did not complete its frozen batch: " + sid)
        if (spec.get("schema") != schema or spec.get("contract_sha256") != digest
                or spec.get("scenario_id") != sid or spec.get("source_scenario") != row
                or spec.get("expected_vehicles") != expected or spec.get("pilot") is not False
                or spec.get("pilot_vehicle_ids") is not None
                or spec.get("collector_sources_sha256") != sources
                or spec.get("collector_sha256") != contract["collector_sha256"]
                or spec.get("runner_sha256") != contract["runner_sha256"]):
            raise ValueError("Worker specification differs from batch: " + sid)
        prepared = _hash_map(spec.get("prepared_sha256"), "prepared inputs")
        if (set(prepared) != {spec.get("network"), spec.get("route"), spec.get("config")}
                or prepared.get(spec.get("network")) != network_sha
                or prepared.get(spec.get("route")) != hashes["route"]):
            raise ValueError("Prepared full-run inputs do not match scenario")
        result = job_root / "result"
        audit_path = safe_file(root, "jobs/" + sid + "/result/audit.json")
        run_path = safe_file(root, "jobs/" + sid + "/result/manifest.json")
        metadata_before.update({path: sha256_file(path) for path in (audit_path, run_path)})
        audit, run = read_json(audit_path), read_json(run_path)
        verify_file(audit_path, status.get("audit_sha256"))
        verify_file(run_path, audit.get("manifest_sha256"))
        if (audit.get("status") != "COMPLETE" or "error" not in audit or audit["error"] is not None
                or any(audit.get(key) is not True for key in
                       ("native_outputs_closed", "training_released", "explicit_speed_factor_audited"))):
            raise ValueError("Native outputs were not quality-released: " + sid)
        counts = audit.get("counts", {})
        for key in ("expected_vehicles", "arrived", "tripinfo_count", "normal_arrived"):
            if type(counts.get(key)) is not int or counts[key] != expected:
                raise ValueError("Incomplete vehicle audit: " + key)
        for key in ("collisions", "teleports", "missing_ids", "missing_runtime_sf"):
            if type(counts.get(key)) is not int or counts[key] != 0:
                raise ValueError("Untrusted native audit counter: " + key)
        for key in ("trainable", "excluded", "road_occurrences"):
            if type(counts.get(key)) is not int or counts[key] < 0:
                raise ValueError("Invalid quality counter: " + key)
        if (counts["trainable"] + counts["excluded"] != counts["road_occurrences"]
                or counts["road_occurrences"] < expected or status.get("counts") != counts):
            raise ValueError("Road-quality counts do not close")
        if (run.get("schema") != "sumo-native-collection-run-v2" or run.get("inputs") != prepared
                or run.get("sources") != sources or run.get("expected_vehicles") != expected
                or run.get("no_hard_end") is not True or run.get("behavior_setters_used") is not False
                or run.get("expected_road_occurrences") != counts["road_occurrences"]):
            raise ValueError("Native run manifest does not bind this job")
        collection = validate_collection_contract(run.get("contract"))
        if audit.get("contract") != collection:
            raise ValueError("Audit/manifest collection contract mismatch")
        step = run.get("step_seconds")
        if type(step) not in (int, float) or not math.isfinite(step) or step <= 0:
            raise ValueError("Invalid simulation step")
        if not isinstance(run.get("sumo_version"), list) or len(run["sumo_version"]) != 2:
            raise ValueError("Missing SUMO version identity")
        identity = (network_sha, run["sumo_version"], step, collection)
        if run_identity is not None and identity != run_identity:
            raise ValueError("Mixed network/runtime/collection semantics in batch")
        run_identity = identity
        contracts.append(collection)
        outputs = _hash_map(audit.get("closed_outputs_sha256"), "closed outputs")
        if not {"training.csv.gz", "road_visits.csv.gz", "tripinfo.xml", "vehroute.xml.gz"}.issubset(outputs):
            raise ValueError("Missing closed native/training output bindings")
        if status.get("verified_bindings") != {"manifest_sha256": audit["manifest_sha256"],
                                               "closed_outputs_sha256": outputs}:
            raise ValueError("Worker verified bindings differ from audit")
        for relative, output_digest in outputs.items():
            if relative in {"manifest.json", "audit.json"} or ".partial" in relative:
                raise ValueError("Invalid closed output name")
            verify_file(safe_file(result, relative), output_digest)
        for metadata_path, metadata_digest in metadata_before.items():
            verify_file(metadata_path, metadata_digest)
        training_sha = outputs["training.csv.gz"]
        if training_sha in csv_hashes:
            raise ValueError("Duplicate training file bytes across scenarios")
        csv_hashes.add(training_sha)
        records.append(Scenario(sid, family, split, result / "training.csv.gz", training_sha,
            counts["trainable"], expected, float(step), {
                "job_sha256": sha256_file(spec_path), "status_sha256": sha256_file(status_path),
                "audit_sha256": sha256_file(audit_path),
                "manifest_sha256": audit["manifest_sha256"], "closed_outputs_sha256": outputs,
                "counts": counts, "quality": audit.get("quality", {})}))
        if progress:
            progress(f"VERIFY scenario={sid} complete rows={counts['trainable']} seconds={time.monotonic() - started:.1f}")
    if ((selected_ids is None and not all(families.values()))
            or families["train"] & families["validation"]):
        raise ValueError("Train/validation families must be nonempty and disjoint")
    for split, expected in (("train", expected_train), ("validation", expected_validation)):
        if expected is not None and split_counts[split] != expected:
            raise ValueError("Unexpected " + split + " scenario count")
    return Batch(root, tuple(records), contracts[0] if contracts else None, {
        "batch_manifest_sha256": sha256_file(manifest_path), "batch_status_sha256": sha256_file(state_path),
        "batch_contract_sha256": digest, "dataset_sha256": contract["dataset_sha256"],
        "collector_sources_sha256": sources, "collector_sha256": contract["collector_sha256"],
        "runner_sha256": contract["runner_sha256"],
        "network_sha256": run_identity[0] if run_identity else None,
        "sumo_version": run_identity[1] if run_identity else None,
        "step_seconds": run_identity[2] if run_identity else None,
        "split_counts": split_counts, "families": {key: sorted(value) for key, value in families.items()}})


def iter_chunks(scenario, chunk_size):
    if type(chunk_size) is not int or chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    with gzip.open(scenario.training_path, "rt", encoding="utf-8", newline="") as stream:
        header = next(csv.reader(stream), [])
    if len(set(header)) != len(header) or set(READ_COLUMNS) - set(header):
        raise ValueError("Missing/duplicate native training columns: " + scenario.scenario_id)
    yield from pd.read_csv(scenario.training_path, usecols=READ_COLUMNS, dtype=str,
                           keep_default_na=False, chunksize=chunk_size, float_precision="round_trip")


def _numeric(frame, name):
    values = pd.to_numeric(frame[name], errors="raise").astype("float64")
    if not np.isfinite(values.to_numpy()).all():
        raise ValueError("Non-finite native column: " + name)
    return values


def validate_chunk(raw, scenario, seen):
    """Per-vehicle state is bounded by scenario vehicles, not by road visits."""
    if not raw["schema"].eq(SOURCE_SCHEMA).all() or any(not raw[key].eq("1").all() for key in FLAGS):
        raise ValueError("Unreleased, mixed-schema, or low-quality row in training output")
    if not raw["training_release_scope"].eq("provisional_until_run_audit_passes").all():
        raise ValueError("Unknown row release scope")
    for key in ("snapshot_clock_schema", "signal_clock_schema"):
        if not raw[key].eq(CLOCK_SCHEMA).all():
            raise ValueError("Unknown native snapshot clock")
    if raw["vehicle_id"].str.strip().eq("").any():
        raise ValueError("Empty native vehicle identity")
    indices = _numeric(raw, "route_index")
    finals = _numeric(raw, "is_final_road")
    if ((indices < 0) | (indices != np.floor(indices)) | (indices > 2**53)).any() or not finals.isin([0, 1]).all():
        raise ValueError("Invalid road occurrence metadata")
    identity = pd.DataFrame({"vehicle": raw["vehicle_id"], "index": indices, "final": finals})
    grouped = identity.groupby("vehicle", sort=False)
    if (grouped["index"].diff() <= 0).any():
        raise ValueError("Duplicate or regressing road occurrence")
    # Once a final road is observed no subsequent occurrence may follow, even in
    # another chunk; legitimate excluded roads can leave gaps in route indices.
    if ((grouped["final"].cumsum() - identity["final"]) > 0).any():
        raise ValueError("Road occurrence after final road")
    summary = grouped.agg(first=("index", "first"), last=("index", "last"), final=("final", "max"))
    for vehicle, first, last, final in summary.itertuples():
        prior = seen.get(vehicle)
        if prior is not None and (first <= prior[0] or prior[1]):
            raise ValueError("Duplicate/regressing/post-final occurrence across chunks")
        seen[vehicle] = (last, bool(final))
    if len(seen) > scenario.expected_vehicles:
        raise ValueError("More training vehicles than released scenario")
    features = encode_features(raw)
    if (features["road_flow"] < 1).any() or (features["lane_flow"] < 1).any():
        raise ValueError("Native flow must include the observed ego")
    sentinel = features["entry_signal_remaining_seconds"].eq(0)
    if (sentinel != finals.eq(1)).any():
        raise ValueError("G0G0 is required exactly on final roads")
    step = scenario.step_seconds
    for name in ("snapshot_clock_step_seconds", "signal_step_seconds"):
        if not np.isclose(_numeric(raw, name), step, rtol=0, atol=1e-9).all():
            raise ValueError("Row step differs from run step")
    observed = _numeric(raw, "observed_enter_time")
    for name in ("signal_snapshot_time", "signal_reference_time"):
        if not np.isclose(_numeric(raw, name), observed, rtol=0, atol=1e-7).all():
            raise ValueError("Signal and feature snapshots disagree")
    if (not np.isclose(_numeric(raw, "signal_api_return_time") - observed, step, rtol=0, atol=1e-7).all()
            or not _numeric(raw, "signal_projection_seconds").eq(0).all()):
        raise ValueError("Invalid native signal clock/projection")
    enter, leave = _numeric(raw, "v2_enter"), _numeric(raw, "v2_exit")
    lag = _numeric(raw, "feature_snapshot_lag_seconds")
    if ((leave < enter) | (lag < -1e-7) | (lag > step + 1e-7)).any():
        raise ValueError("Invalid native occurrence/feature lag")
    if not np.isclose(observed - enter, lag, rtol=0, atol=1e-6).all():
        raise ValueError("Feature snapshot lag disagrees with boundary")
    labels = pd.DataFrame({name: _numeric(raw, name) for name in LABEL_COLUMNS})
    ordered = ["travel_time_label_gt_" + item for item in ("0p1", "0p5", "1", "1p5", "2", "3")]
    if ((labels < 0).any().any() or (labels.sub(leave - enter, axis=0) > 1e-6).any().any()
            or (labels[ordered].diff(axis=1).iloc[:, 1:] > 1e-8).any().any()):
        raise ValueError("Invalid native target magnitude/order/budget")
    return features, labels


def preflight(batch, chunk_size=100000, progress=None):
    totals = {split: {label: {"positive": 0, "zero_policy_excluded": 0, "positive_subsecond": 0}
                      for label in LABEL_COLUMNS} for split in ("train", "validation")}
    reports = []
    for scenario in batch.scenarios:
        started = time.monotonic()
        if progress:
            progress("PREFLIGHT scenario=" + scenario.scenario_id + " start")
        seen, count = {}, 0
        counts = {label: {"positive": 0, "zero_policy_excluded": 0, "positive_subsecond": 0}
                  for label in LABEL_COLUMNS}
        for raw in iter_chunks(scenario, chunk_size):
            _, labels = validate_chunk(raw, scenario, seen)
            count += len(raw)
            if progress and count // 1000000 > (count - len(raw)) // 1000000:
                progress(f"PREFLIGHT scenario={scenario.scenario_id} rows={count} seconds={time.monotonic() - started:.1f}")
            for label in LABEL_COLUMNS:
                values = labels[label]
                counts[label]["positive"] += int(values.gt(0).sum())
                counts[label]["zero_policy_excluded"] += int(values.eq(0).sum())
                counts[label]["positive_subsecond"] += int(((values > 0) & (values < 1)).sum())
        verify_file(scenario.training_path, scenario.training_sha256)
        if count != scenario.expected_rows:
            raise ValueError("Training row count differs from audit: " + scenario.scenario_id)
        for label in LABEL_COLUMNS:
            for key, value in counts[label].items():
                totals[scenario.split][label][key] += value
        reports.append({"scenario_id": scenario.scenario_id, "family_id": scenario.family_id,
                        "split": scenario.split, "rows": count, "observed_vehicles": len(seen),
                        "training_sha256": scenario.training_sha256, "labels": counts,
                        "evidence": scenario.evidence})
        if progress:
            progress(f"PREFLIGHT scenario={scenario.scenario_id} complete rows={count} seconds={time.monotonic() - started:.1f}")
    return {"status": "PREFLIGHT_COMPLETE", "source_schema": SOURCE_SCHEMA,
            "rows_by_split_and_label": totals, "scenarios": reports, "provenance": batch.provenance,
            "zero_policy": "valid source zero; excluded by unchanged positive-label training policy",
            "positive_subsecond_policy": "retained; export floor remains one second"}
