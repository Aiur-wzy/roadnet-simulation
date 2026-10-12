"""Publish an immutable index of successful native-v2 collection attempts.

The default is a metadata-only dry run: no output directory or model is made.
--publish creates a fresh index directory, verifies every selected closed output
through the training loader, and only then writes a COMPLETE publication marker.
Source batches, including failed or stale supervisor states, are never changed.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import sys

from training.native_contract import canonical_digest, sha256_file
from training.native_data import BATCH_SCHEMAS, read_json, safe_file, verify_file


SCHEMA = "native-success-subset-v1"
STATUS_SCHEMA = "native-success-subset-status-v1"
DATASET_SCHEMA = "manhattan-structured-demand-v1"


def _now():
    return datetime.now(timezone.utc).isoformat()


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", value):
        raise ValueError("Invalid scenario/source identifier")
    return value


def _physical_path(value, directory=False):
    path = Path(value).absolute()
    for member in (path, *path.parents):
        if member.is_symlink() or getattr(member, "is_junction", lambda: False)():
            raise ValueError("Use a physical source, not a symlink/junction: " + str(member))
    path = path.resolve(strict=True)
    if directory and not path.is_dir() or not directory and not path.is_file():
        raise ValueError("Wrong source path type: " + str(path))
    return path


def _read_bound(path, bindings):
    before = sha256_file(path)
    value = read_json(path)
    verify_file(path, before)
    bindings[str(path)] = before
    return value


def _check_bindings(bindings):
    for name, digest in bindings.items():
        if digest is None:
            if Path(name).exists() or Path(name).is_symlink():
                raise ValueError("Previously absent worker status appeared during publication: " + name)
        else:
            verify_file(name, digest)


def _source_row_matches_dataset(row, original):
    for key in ("scenario_id", "family_id", "split", "vehicle_count", "network_sha256"):
        if row.get(key) != original.get(key):
            raise ValueError("Source scenario differs from frozen dataset: " + row.get("scenario_id", "?") + "/" + key)
    expected = {"route": original.get("route_sha256"), "config": original.get("config_sha256"),
                "manifest": original.get("scenario_manifest_sha256")}
    if row.get("hashes") != expected:
        raise ValueError("Source input hashes differ from frozen dataset: " + row["scenario_id"])


def plan_release(sources, dataset_manifest, expected_train, expected_validation, expected_excluded):
    """Read only small immutable metadata; COMPLETE rows are candidates until full verification."""
    for value in (expected_train, expected_validation):
        if type(value) is not int or value < 1:
            raise ValueError("Expected train/validation counts must be positive")
    if type(expected_excluded) is not int or expected_excluded < 0:
        raise ValueError("Expected excluded count must be nonnegative")
    if not sources:
        raise ValueError("At least one source batch is required")
    bindings = {}
    dataset_path = _physical_path(dataset_manifest)
    dataset = _read_bound(dataset_path, bindings)
    dataset_sha = bindings[str(dataset_path)]
    if (dataset.get("schema") != DATASET_SCHEMA or dataset.get("ready") is not True
            or dataset.get("status") != "DEMAND_GENERATED_NOT_SIMULATED"):
        raise ValueError("Expected a frozen generated structured-demand dataset manifest")
    dataset_rows = dataset.get("scenarios")
    if not isinstance(dataset_rows, list) or not dataset_rows:
        raise ValueError("Dataset scenario index is empty")
    originals = {}
    families = {"train": set(), "validation": set()}
    for row in dataset_rows:
        if not isinstance(row, dict):
            raise ValueError("Invalid dataset scenario row")
        sid = _identifier(row.get("scenario_id"))
        if sid in originals:
            raise ValueError("Duplicate dataset scenario: " + sid)
        split = row.get("split")
        if split not in families:
            raise ValueError("Invalid frozen dataset split")
        families[split].add(_identifier(row.get("family_id")))
        originals[sid] = row
    if not all(families.values()) or families["train"] & families["validation"]:
        raise ValueError("Frozen dataset train/validation families must be disjoint and nonempty")
    source_records, attempts, source_ids, physical_roots = [], [], set(), set()
    for source_id, supplied_root in sources:
        source_id = _identifier(source_id)
        root = _physical_path(supplied_root, directory=True)
        if source_id in source_ids or str(root) in physical_roots:
            raise ValueError("Duplicate source identifier or physical batch root")
        source_ids.add(source_id)
        physical_roots.add(str(root))
        manifest_path = safe_file(root, "batch_manifest.json")
        status_path = safe_file(root, "status.json")
        manifest, state = _read_bound(manifest_path, bindings), _read_bound(status_path, bindings)
        schema = manifest.get("schema")
        contract = manifest.get("contract", {})
        if (schema not in BATCH_SCHEMAS or state.get("schema") != schema
                or contract.get("pilot") is not False or state.get("pilot") is not False
                or contract.get("dataset_sha256") != dataset_sha):
            raise ValueError("Source is not a full-run batch bound to this dataset: " + source_id)
        rows = contract.get("scenarios")
        if not isinstance(rows, list) or not rows:
            raise ValueError("Empty source scenario index")
        ids = [_identifier(row.get("scenario_id")) for row in rows]
        if len(set(ids)) != len(ids) or contract.get("scenario_ids") != ids:
            raise ValueError("Duplicate or inconsistent source scenario index")
        contract_sha = canonical_digest(contract)
        source_records.append({"source_id": source_id, "root": str(root),
            "batch_manifest_sha256": bindings[str(manifest_path)],
            "batch_status_sha256": bindings[str(status_path)]})
        for row in rows:
            sid = row["scenario_id"]
            if sid not in originals:
                raise ValueError("Source scenario not present in frozen dataset: " + sid)
            _source_row_matches_dataset(row, originals[sid])
            relative_status = "jobs/" + sid + "/status.json"
            job_status_path = root / relative_status
            worker = None
            if job_status_path.exists() or job_status_path.is_symlink():
                job_status_path = safe_file(root, relative_status)
                worker = _read_bound(job_status_path, bindings)
                if worker.get("scenario_id") != sid or worker.get("contract_sha256") != contract_sha:
                    raise ValueError("Worker identity/contract differs from source: " + source_id + "/" + sid)
            else:
                bindings[str(job_status_path)] = None
            worker_state = worker.get("state") if worker is not None else None
            if worker_state == "COMPLETE":
                # This cheap gate avoids planning a known invalid success. The
                # publication loader independently verifies all full evidence.
                if (worker.get("returncode") != 0 or isinstance(worker.get("returncode"), bool)
                        or worker.get("audit_status") != "COMPLETE" or worker.get("gate_problems") != []):
                    raise ValueError("Invalid COMPLETE worker metadata: " + source_id + "/" + sid)
                audit_path = safe_file(root, "jobs/" + sid + "/result/audit.json")
                audit = _read_bound(audit_path, bindings)
                verify_file(audit_path, worker.get("audit_sha256"))
                if (audit.get("status") != "COMPLETE" or "error" not in audit or audit["error"] is not None
                        or any(audit.get(key) is not True for key in
                               ("native_outputs_closed", "training_released", "explicit_speed_factor_audited"))):
                    raise ValueError("COMPLETE worker was not quality released: " + source_id + "/" + sid)
            declared = state.get("scenarios", {}).get(sid, "UNDECLARED")
            reason = ("worker_state=" + str(worker_state)) if worker is not None else "no_worker_status;batch_state=" + str(declared)
            attempts.append({"scenario_id": sid, "source_id": source_id,
                             "complete": worker_state == "COMPLETE", "reason": reason})
    covered = {item["scenario_id"] for item in attempts}
    if covered != set(originals):
        raise ValueError("Source batches do not cover the entire frozen dataset: missing="
                         + ",".join(sorted(set(originals) - covered)))
    successful = Counter(item["scenario_id"] for item in attempts if item["complete"])
    duplicate = sorted(sid for sid, count in successful.items() if count > 1)
    if duplicate:
        raise ValueError("Multiple COMPLETE attempts require an explicit new adjudication; never select latest: " + ",".join(duplicate))
    selected = [{"scenario_id": item["scenario_id"], "source_id": item["source_id"]}
                for item in attempts if item["complete"]]
    excluded = [{"scenario_id": item["scenario_id"], "source_id": item["source_id"],
                 "reason": item["reason"] + (";successful_attempt_selected_elsewhere" if item["scenario_id"] in successful else ";no_successful_attempt")}
                for item in attempts if not item["complete"]]
    split_counts = Counter(originals[item["scenario_id"]]["split"] for item in selected)
    counts = {"train": split_counts["train"], "validation": split_counts["validation"],
              "selected": len(selected), "excluded_scenarios": len(set(originals) - set(successful))}
    if counts != {"train": expected_train, "validation": expected_validation,
                  "selected": expected_train + expected_validation, "excluded_scenarios": expected_excluded}:
        raise ValueError("Actual successful subset counts differ from requested counts: " + json.dumps(counts, sort_keys=True))
    _check_bindings(bindings)
    manifest = {"schema": SCHEMA, "dataset_sha256": dataset_sha, "sources": source_records,
                "selected": selected, "excluded": excluded, "expected_counts": counts,
                "original_batch_complete": False, "derived_release_complete": True}
    return manifest, dataset_path, bindings


def _write_json(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, sort_keys=True, ensure_ascii=True, allow_nan=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _sync_directory(path):
    if os.name == "posix":
        descriptor = os.open(str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", nargs=2, action="append", required=True, metavar=("ID", "PATH"))
    parser.add_argument("--dataset-manifest", required=True)
    parser.add_argument("--output-root", required=True, help="New physical index directory; no overwrite or resume")
    parser.add_argument("--expected-train", type=int, default=31)
    parser.add_argument("--expected-validation", type=int, default=6)
    parser.add_argument("--expected-excluded", type=int, default=5, help="Unique dataset scenarios with no successful attempt")
    parser.add_argument("--publish", action="store_true", help="Full-SHA verification and release; default is metadata-only dry run")
    return parser


def run(args):
    output = Path(args.output_root).absolute()
    if output.exists() or output.is_symlink():
        raise ValueError("Output exists; use a fresh release directory")
    parent = _physical_path(output.parent, directory=True)
    output = parent / output.name
    manifest, dataset_path, bindings = plan_release(args.source, args.dataset_manifest,
        args.expected_train, args.expected_validation, args.expected_excluded)
    plan = {"status": "DRY_RUN_PLAN", "full_output_hashes_verified": False,
            "training_released": False, "output_root": str(output), "manifest": manifest,
            "excluded_attempts": len(manifest["excluded"]),
            "note": "Metadata-only candidates; --publish performs all selected output hashes. No training starts."}
    if not args.publish:
        return plan
    # No source mutation, links, big data copies, or retry/resume actions occur.
    output.mkdir(exist_ok=False)
    started = _now()
    try:
        _write_json(output / "publication_started.json", {"schema": STATUS_SCHEMA, "status": "VERIFYING",
            "started_utc": started, "source_metadata_sha256": bindings})
        with dataset_path.open("rb") as source, (output / "dataset_manifest.json").open("xb") as target:
            target.write(source.read())
            target.flush()
            os.fsync(target.fileno())
        verify_file(output / "dataset_manifest.json", manifest["dataset_sha256"])
        _write_json(output / "release_manifest.json", manifest)
        from training.native_release import load_success_release
        progress = lambda message: print(message, file=sys.stderr, flush=True)
        batch = load_success_release(output, expected_train=args.expected_train,
            expected_validation=args.expected_validation, require_complete=False, progress=progress)
        _check_bindings(bindings)
        if len(batch.scenarios) != manifest["expected_counts"]["selected"]:
            raise ValueError("Verified loader count differs from publication plan")
        summary = {"by_split": {}, "by_source": {}}
        for split in ("train", "validation"):
            records = [item for item in batch.scenarios if item.split == split]
            summary["by_split"][split] = {
                "scenarios": len(records), "vehicles": sum(item.expected_vehicles for item in records),
                "training_csv_rows_before_positive_label_policy": sum(item.expected_rows for item in records)}
        selected_sources = {item["scenario_id"]: item["source_id"] for item in manifest["selected"]}
        for source in manifest["sources"]:
            source_id = source["source_id"]
            records = [item for item in batch.scenarios if selected_sources[item.scenario_id] == source_id]
            summary["by_source"][source_id] = {
                "scenarios": len(records), "train": sum(item.split == "train" for item in records),
                "validation": sum(item.split == "validation" for item in records),
                "vehicles": sum(item.expected_vehicles for item in records),
                "training_csv_rows_before_positive_label_policy": sum(item.expected_rows for item in records)}
        state = {"schema": STATUS_SCHEMA, "status": "COMPLETE", "started_utc": started,
                 "completed_utc": _now(), "release_manifest_sha256": sha256_file(output / "release_manifest.json"),
                 "full_output_hashes_verified": True, "source_bytes_modified": False,
                 "expected_counts": manifest["expected_counts"], "summary": summary,
                 "storage_policy": "Metadata-only release; immutable physical source paths must remain available. "
                                   "No CSV or native outputs are copied into this index directory.",
                 "training_started": False}
        _write_json(output / "status.json", state)
        _sync_directory(output)
        return state
    except BaseException as error:
        failure = {"schema": STATUS_SCHEMA, "status": "FAILED", "started_utc": started,
                   "failed_utc": _now(), "error": type(error).__name__ + ": " + str(error),
                   "training_released": False, "source_bytes_modified": False}
        # Never replace even this operation's COMPLETE marker. An exception
        # after completion leaves a separate diagnostic rather than a rewrite.
        failure_path = output / ("status.json" if not (output / "status.json").exists() else "publication_failure.json")
        _write_json(failure_path, failure)
        _sync_directory(output)
        raise


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        result = run(args)
    except (ValueError, OSError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=True, allow_nan=False, sort_keys=True, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
