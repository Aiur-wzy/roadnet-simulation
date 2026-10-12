#!/usr/bin/env python3
"""Isolated, fail-closed discharge collection; never launches model training.

The old collector/segmentation implementation is retained unchanged; this batch
uses the E1 quality adapter and conservative visit-quarantine extractor. Formal
collection completes only on natural network drain AND an exact terminal-ID
audit. A diagnostic horizon is deliberately not a training dataset. Batch
execution is Linux-only, serial, and terminates only its own process group.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
from typing import Callable
import xml.etree.ElementTree as ET

from collection.collect_discharge_sequences import (
    canonical_hash, dataset_domain, export_jsonl, sha256_file,
    simulation_identity, write_json,
)
from collection.discharge_sumo import resolve_inputs
from features.discharge_sequences import (
    DATASET_SCHEMA, RULE_VERSION, SegmentationConfig,
)
from features.discharge_e1_quality import OBSERVATION_POLICY, SequenceExtractor, SQLiteStore, usable_passage

PLAN_SCHEMA = "discharge-batch-plan-v1"
RUN_SCHEMA = "discharge-batch-finalization-v1"
DEFAULT_LIMITS = {
    "min_free_gib": 100.0, "max_rss_gib": 16.0, "max_wall_hours": 24.0,
    "stall_sim_seconds": 3600.0, "heartbeat_wall_seconds": 900.0,
    "check_seconds": 5.0, "nice": 10, "threads": 1, "max_workers": 1,
}
DEFAULT_OBSERVATION = {
    "epsilon_zero": 1e-7, "progress_epsilon": 1e-4,
    "stop_duration": 1.0, "cohort_distance": 100.0, "queue_gap": 12.0,
    "max_start_distance": 15.0, "detector_offset": 0.0,
}


class QualityError(RuntimeError):
    """A complete, trustworthy trainable dataset cannot be certified."""


class ResourceError(RuntimeError):
    """A resource guard requires stopping this batch, not skipping ahead."""


class StagnationError(QualityError):
    """Aggregate progress stopped; this is censoring, not proof of deadlock."""


def utc() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_json(path: Path) -> dict:
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result
    with path.open(encoding="utf-8-sig") as handle:
        return json.load(handle, object_pairs_hook=unique,
                         parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))


def label_support(output: Path, run_id: str, coverage: dict) -> dict:
    """Count only final safe prefixes, using the unchanged trainer validator.

    An empty extractor exports totals={}; that is zero only when the exported
    sequence stream is also empty. Missing counters in nonempty totals are an
    error, never silently interpreted as zero. Memory is bounded by one prefix.
    """
    from training.train_discharge_sequences import json_lines, validate_sequence

    counts = dict(sequences=0, zero_passage_sequences=0,
                  startup_samples=0, headway_samples=0)
    for _, sequence in json_lines(output / "sequences.jsonl"):
        validate_sequence(sequence, run_id)
        counts["sequences"] += 1
        counts["zero_passage_sequences"] += int(not sequence["passages"])
        counts["startup_samples"] += int(sequence["startup_delay"] is not None)
        counts["headway_samples"] += len(sequence["headways"])
    totals = coverage.get("totals")
    if not isinstance(totals, dict):
        raise QualityError("coverage totals must be an object")
    if totals == {} and counts["sequences"] == 0:
        reported = dict.fromkeys(counts, 0)
    else:
        reported = {name: totals.get(name) for name in counts}
        if any(type(value) is not int or value < 0 for value in reported.values()):
            raise QualityError("coverage counts must be explicit nonnegative integers")
    if reported != counts:
        raise QualityError("coverage counts do not match final safe sequences")
    return {"schema": "discharge-label-support-v1", **counts,
            "has_startup_labels": counts["startup_samples"] > 0,
            "has_headway_labels": counts["headway_samples"] > 0,
            "has_any_labels": counts["startup_samples"] + counts["headway_samples"] > 0}


def positive(value, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be positive and finite")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return result


def safe_run_id(value: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,119}", value):
        raise ValueError("run_id must be a safe nonempty basename")
    if value in {".", ".."}:
        raise ValueError("unsafe run_id")
    return value


def xml_stream(path: Path):
    return gzip.open(path, "rb") if str(path).endswith(".gz") else path.open("rb")


def route_vehicles(path: Path) -> dict[str, float]:
    """Only explicit finite-demand vehicle routes can be exactly audited."""
    result = {}
    with xml_stream(path) as handle:
        for _, element in ET.iterparse(handle, events=("end",)):
            if element.tag in {"flow", "person", "personFlow", "container", "trip"}:
                raise ValueError("only explicit vehicle routes are supported; found " + element.tag)
            if element.tag != "vehicle":
                continue
            vehicle_id = element.attrib["id"]
            if vehicle_id in result:
                raise ValueError("duplicate route vehicle ID: " + vehicle_id)
            try:
                departure = float(element.attrib["depart"])
            except (KeyError, ValueError) as error:
                raise ValueError("vehicle depart must be numeric: " + vehicle_id) from error
            if not math.isfinite(departure) or departure < 0:
                raise ValueError("invalid planned departure: " + vehicle_id)
            result[vehicle_id] = departure
            element.clear()
    if not result:
        raise ValueError("empty route")
    return result


def verify_hashes(hashes: dict) -> dict:
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError("source_hashes must contain explicit absolute paths and SHA256")
    checked = {}
    for raw_path, expected in hashes.items():
        path = Path(raw_path)
        if not path.is_absolute() or not isinstance(expected, str) or not re.fullmatch(r"[a-fA-F0-9]{64}", expected):
            raise ValueError("invalid absolute input/hash: " + str(raw_path))
        resolved = path.resolve(strict=True)
        actual = sha256_file(resolved)
        if actual.lower() != expected.lower():
            raise QualityError("input SHA256 mismatch: " + str(resolved))
        checked[str(resolved)] = actual
    return checked


def resource_limits(plan: dict) -> dict:
    unknown = set(plan.get("resource_limits", {})) - set(DEFAULT_LIMITS)
    if unknown:
        raise ValueError("unknown resource limits: " + str(sorted(unknown)))
    values = {**DEFAULT_LIMITS, **plan.get("resource_limits", {})}
    for key in ("min_free_gib", "max_rss_gib", "max_wall_hours", "stall_sim_seconds",
                "heartbeat_wall_seconds", "check_seconds"):
        values[key] = positive(values[key], key)
    for key in ("threads", "max_workers"):
        if values[key] != 1 or isinstance(values[key], bool):
            raise ValueError(key + " must remain 1")
    if type(values["nice"]) is not int or not 1 <= values["nice"] <= 19:
        raise ValueError("nice must be an integer from 1 to 19")
    return values


def validate_job(job: dict) -> dict:
    safe_run_id(job["run_id"])
    if job["domain"] not in {"small", "manhattan"}:
        raise ValueError("domain must be small or manhattan")
    for name in ("network", "route", "sumo_config"):
        if not Path(job[name]).is_absolute():
            raise ValueError(name + " must be absolute")
    hashes = verify_hashes(job["source_hashes"])
    inputs = resolve_inputs({"sumo_config": Path(job["sumo_config"])})
    if Path(inputs["net"]).resolve() != Path(job["network"]).resolve():
        raise ValueError("network does not match sumo_config")
    if [str(Path(x).resolve()) for x in inputs["routes"]] != [str(Path(job["route"]).resolve())]:
        raise ValueError("sumo_config must name exactly the selected route")
    for path in [inputs["net"], *inputs["routes"], *inputs["additional_files"], job["sumo_config"]]:
        if str(Path(path).resolve()) not in hashes:
            raise ValueError("unbound SUMO input: " + str(path))
    history = job.get("history_audit")
    if history:
        history_path = history["path"] if isinstance(history, dict) else history
        expected = history.get("sha256") if isinstance(history, dict) else None
        actual = sha256_file(Path(history_path).resolve(strict=True))
        if expected and actual != expected:
            raise QualityError("history_audit SHA256 mismatch")
        if not expected and str(Path(history_path).resolve()) not in hashes:
            raise ValueError("history_audit must be hash-bound")
    planned = positive(job["planned_seconds"], "planned_seconds")
    end = positive(job["safety_end_seconds"], "safety_end_seconds")
    if end < planned:
        raise ValueError("safety_end_seconds must not precede planned supply duration")
    if job.get("sumo_seed") is not None and (type(job["sumo_seed"]) is not int or job["sumo_seed"] < 0):
        raise ValueError("sumo_seed must be a nonnegative integer")
    vehicles = route_vehicles(Path(job["route"]))
    if type(job["expected_vehicles"]) is not int or len(vehicles) != job["expected_vehicles"]:
        raise ValueError("expected_vehicles does not match route")
    if max(vehicles.values()) >= end:
        raise ValueError("safety horizon must be later than the final planned departure")
    return {"run_id": job["run_id"], "domain": job["domain"], "source_hashes": hashes,
            "vehicles": len(vehicles), "first_planned_depart": min(vehicles.values()),
            "last_planned_depart": max(vehicles.values()), "planned_seconds": planned}


def validate_plan(plan: dict) -> dict:
    if plan.get("schema") != PLAN_SCHEMA:
        raise ValueError("unsupported batch plan schema")
    jobs = plan.get("jobs")
    if not isinstance(jobs, list) or not jobs:
        raise ValueError("jobs must be a nonempty list")
    ids = [safe_run_id(job["run_id"]) for job in jobs]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate run_id")
    observation = {**DEFAULT_OBSERVATION, **plan.get("observation", {})}
    if set(observation) != set(DEFAULT_OBSERVATION):
        raise ValueError("unknown observation option")
    # Reuse the original core's validation; adapter validates its own fields.
    SegmentationConfig(epsilon_zero=observation["epsilon_zero"],
                       progress_epsilon=observation["progress_epsilon"])
    for key in ("stop_duration", "cohort_distance", "queue_gap", "max_start_distance"):
        positive(observation[key], key)
    if not math.isfinite(float(observation["detector_offset"])) or observation["detector_offset"] < 0:
        raise ValueError("invalid detector_offset")
    return {"schema": "discharge-batch-readiness-v1", "plan_sha256": canonical_hash(plan),
            "resource_limits": resource_limits(plan), "observation": observation,
            "jobs": [validate_job(job) for job in jobs], "checked_utc": utc()}


def audit_tripinfo(path: Path, expected: dict[str, float], metadata: dict) -> dict:
    seen, bad, samples = set(), {}, {}
    durations, delays = [], []
    def issue(name, vehicle_id):
        bad[name] = bad.get(name, 0) + 1
        samples.setdefault(name, [])
        if len(samples[name]) < 20:
            samples[name].append(vehicle_id)
    with xml_stream(path) as handle:
        for _, node in ET.iterparse(handle, events=("end",)):
            if node.tag != "tripinfo":
                continue
            row, vehicle_id = node.attrib, node.attrib.get("id", "")
            if vehicle_id in seen:
                issue("duplicate_ids", vehicle_id)
            seen.add(vehicle_id)
            if vehicle_id not in expected:
                issue("unexpected_ids", vehicle_id)
            try:
                depart, arrival = float(row["depart"]), float(row["arrival"])
                duration = float(row["duration"])
                finite = all(math.isfinite(x) for x in (depart, arrival, duration))
            except (ValueError, KeyError):
                finite, depart, arrival, duration = False, -1.0, -1.0, -1.0
            if not finite or depart < 0:
                issue("not_departed_or_invalid_time", vehicle_id)
            if not finite or arrival < depart or arrival < 0 or duration < 0:
                issue("unfinished_or_invalid_time", vehicle_id)
            elif abs(duration - (arrival - depart)) > 0.011:
                issue("duration_mismatch", vehicle_id)
            if row.get("vaporized", "").strip().lower() not in {"", "false", "0"}:
                issue("vaporized", vehicle_id)
            if finite and depart >= 0 and arrival >= depart:
                durations.append(arrival - depart)
                if vehicle_id in expected:
                    delays.append(depart - expected[vehicle_id])
                    if depart + 0.011 < expected[vehicle_id]:
                        issue("departure_before_plan", vehicle_id)
            node.clear()
    missing = set(expected) - seen
    if missing:
        bad["missing_ids"] = len(missing)
        samples["missing_ids"] = sorted(missing)[:20]
    counts = metadata.get("cumulative_counts", {})
    for name in ("departed", "arrived", "teleport_start", "teleport_end", "collision_events", "colliding_vehicle_reports"):
        if type(counts.get(name)) is not int or counts[name] < 0:
            bad["missing_or_invalid_counter_" + name] = 1
    for name in ("departed", "arrived"):
        if counts.get(name) != len(expected):
            bad["counter_mismatch_" + name] = 1
    for name in ("teleport_start", "teleport_end"):
        if counts.get(name, -1) != 0:
            bad[name] = counts.get(name, -1)
    for name in ("collision_events", "colliding_vehicle_reports"):
        if counts.get(name, -1) != 0:
            bad[name] = counts.get(name, -1)
    if metadata.get("termination") != "natural_drain" or metadata.get("remaining") != 0 or metadata.get("completed") is not True:
        bad["not_naturally_drained"] = 1
    return {"schema": "discharge-terminal-audit-v1", "passed": not bad,
            "expected_vehicles": len(expected), "tripinfo_unique_ids": len(seen),
            "errors": bad, "error_id_samples": samples, "cumulative_counts": counts,
            "collision_events": counts.get("collision_events"),
            "colliding_vehicle_reports": counts.get("colliding_vehicle_reports"),
            "collision_policy": "reported separately; any reported collision quarantines formal collection; not proof of geometric overlap",
            "duration_mean_seconds": sum(durations) / len(durations) if durations else None,
            "depart_delay_mean_seconds": sum(delays) / len(delays) if delays else None,
            "tripinfo_sha256": sha256_file(path), "checked_utc": utc()}


def ensure_disk(path: Path, limits: dict) -> float:
    while not path.exists() and path != path.parent:
        path = path.parent
    free = shutil.disk_usage(path).free / (1024 ** 3)
    if free < limits["min_free_gib"]:
        raise ResourceError(f"disk free {free:.3f} GiB below {limits['min_free_gib']:.3f} GiB")
    return free


class ProgressGuard:
    def __init__(self, output: Path, limits: dict, *, clock=time.monotonic):
        self.output, self.limits, self.clock = output, limits, clock
        self.start_wall = clock()
        self.last_counts = None
        self.last_activity_sim = None

    def __call__(self, value: dict):
        now = self.clock()
        snapshot = {**value, "phase": "simulating", "updated_utc": utc(), "wall_seconds": now - self.start_wall}
        write_json(self.output / "status.json", snapshot)
        ensure_disk(self.output, self.limits)
        if now - self.start_wall > self.limits["max_wall_hours"] * 3600:
            raise ResourceError("worker wall-time guard exceeded")
        counts = value.get("cumulative_counts", {})
        identity = (counts.get("departed"), counts.get("arrived"), value.get("observed_passages"))
        simulation_time = float(value["simulation_time"])
        if identity != self.last_counts:
            self.last_counts, self.last_activity_sim = identity, simulation_time
        elif value.get("running", 0) > 0 and self.last_activity_sim is not None and simulation_time - self.last_activity_sim >= self.limits["stall_sim_seconds"]:
            raise StagnationError("no departures, arrivals or outlet passages for configured simulation-time guard; not proof of deadlock")
        if (self.output / "STOP_REQUESTED").exists():
            raise ResourceError("own worker STOP_REQUESTED marker")


def collect_job(job: dict, output: Path, limits: dict, observation: dict,
                *, horizon: float | None = None, adapter: Callable | None = None) -> dict:
    """One independent collection; injection exists for deterministic unit tests."""
    validate_job(job)
    output = output.resolve()
    if output.exists():
        raise FileExistsError("will not reuse output directory: " + str(output))
    ensure_disk(output.parent, limits)
    output.mkdir(parents=True, exist_ok=False)
    config = SegmentationConfig(epsilon_zero=observation["epsilon_zero"],
                                progress_epsilon=observation["progress_epsilon"])
    args = argparse.Namespace(command="collect", sumo_config=Path(job["sumo_config"]),
        net=None, routes=None, additional_files=[], run_id=job["run_id"],
        output_dir=output, end=float(horizon or job["safety_end_seconds"]),
        seed=job.get("sumo_seed"), sumo_binary=job.get("sumo_binary", "sumo"),
        step_length=None, **observation)
    import features.discharge_sequences as core_module
    import collection.discharge_sumo as old_adapter_module
    import collection.collect_discharge_sequences as old_collector_module
    import training.train_discharge_sequences as label_validator_module
    import roadnet_paths as paths_module
    from collection import discharge_batch_sumo_v2 as new_adapter_module
    import features.discharge_e1_quality as quality_module
    source_files = [{"path": path, "sha256": value, "name": Path(path).name}
                    for path, value in job["source_hashes"].items()]
    manifest = {"schema": DATASET_SCHEMA, "complete": False, "status": "running",
        "run_id": job["run_id"], "mode": "collect", "created_utc": utc(),
        "rule_version": RULE_VERSION, "config": asdict(config), "source_files": source_files,
        "batch_job": job, "resource_limits": limits, "diagnostic_only": horizon is not None,
        "observation_contract": "physical lane exit; strict sequence rules; conservative E1 visit quarantine",
        "observation_policy": OBSERVATION_POLICY,
        "code_sha256": {Path(module.__file__).name: sha256_file(Path(module.__file__))
                        for module in (core_module, old_adapter_module, new_adapter_module,
                                       old_collector_module, label_validator_module, paths_module, quality_module)},
        "collector_sha256": sha256_file(Path(__file__))}
    write_json(output / "manifest.json", manifest)
    metadata = {}
    adapter_config = {**vars(args), "metadata": metadata, "diagnostic_horizon": horizon is not None,
                      "progress_callback": ProgressGuard(output, limits)}
    manifest["adapter_metadata"] = metadata
    store, events = SQLiteStore(output / "provisional.sqlite"), None
    try:
        rejected_count = 0
        with (output / "raw_events.jsonl").open("w", encoding="utf-8", newline="\n") as raw, \
                (output / "rejected_records.jsonl").open("w", encoding="utf-8", newline="\n") as rejected:
            def sink(event):
                raw.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
            def reject_sink(event):
                nonlocal rejected_count
                rejected.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
                rejected_count += 1
            extractor = SequenceExtractor(args.run_id, config=config, store=store,
                                          raw_sink=sink, rejection_sink=reject_sink)
            events = (adapter or new_adapter_module.iter_sumo_events)(adapter_config)
            for event in events:
                extractor.feed(event)
            # iter_sumo_events closes native files in its finally before audit.
            extractor.finish()
        write_json(output / "status.json", {"phase": "finalizing", "updated_utc": utc(),
            "note": "native closed; exports, whole-input hashes and exact-ID audit in progress"})
        sequence_count = export_jsonl(output / "sequences.jsonl", store.iter_sequences())
        # The unchanged trainer checks a complete ordinal ledger but learns
        # ONLY from final safe sequences. Removing ledger tombstones would
        # create holes or tempt callers to bridge across a rejected vehicle.
        passage_count = export_jsonl(output / "passages.jsonl", store.iter_passages())
        valid_passage_count = export_jsonl(output / "valid_passages.jsonl",
                                          (row for row in store.iter_passages() if usable_passage(row)))
        coverage = extractor.summary()
        write_json(output / "coverage.json", coverage)
        support = label_support(output, job["run_id"], coverage)
        manifest["domain"] = dataset_domain(args, manifest)
        identity, identity_quality = simulation_identity(args, manifest, output)
        manifest["simulation_identity"] = identity
        manifest["simulation_identity_quality"] = identity_quality
        manifest.update(sequence_count=sequence_count, passage_count=passage_count,
                        rejected_observation_count=rejected_count,
                        valid_passage_count=valid_passage_count,
                        excluded_passage_count=passage_count - valid_passage_count,
                        passage_output_policy={"training_labels": "sequences.jsonl: final safe prefixes only",
                            "valid_records": "valid_passages.jsonl: excludes invalid/unknown/ambiguous records",
                            "ordinal_audit": "passages.jsonl: includes quarantined tombstones; not training targets",
                            "rejection_audit": "rejected_records.jsonl: original rejected values and reasons"},
                        totals=coverage["totals"], label_support=support, finished_utc=utc())
        verify_hashes(job["source_hashes"])
        if horizon is not None:
            audit = {"schema": "discharge-terminal-audit-v1", "passed": False,
                     "reason": "diagnostic_window_not_a_formal_training_dataset",
                     "termination": metadata.get("termination"), "checked_utc": utc()}
            status, complete = "diagnostic_complete", False
        else:
            audit = audit_tripinfo(output / "tripinfo.xml", route_vehicles(Path(job["route"])), metadata)
            write_json(output / "audit.json", audit)
            if not audit["passed"]:
                raise QualityError("terminal audit rejected: " + json.dumps(audit["errors"], sort_keys=True))
            status = "complete" if support["has_any_labels"] else "complete_no_valid_labels"
            complete = True
        write_json(output / "audit.json", audit)
        # Keep the exact four-file whitelist accepted by the unchanged trainer.
        manifest["file_sha256"] = {name: sha256_file(output / name) for name in
            ("raw_events.jsonl", "sequences.jsonl", "passages.jsonl", "coverage.json")}
        manifest["supplemental_file_sha256"] = {name: sha256_file(output / name) for name in
                                               ("audit.json", "tripinfo.xml", "rejected_records.jsonl", "valid_passages.jsonl")}
        training_eligible = complete and support["has_any_labels"]
        headway_training_eligible = complete and support["has_headway_labels"]
        manifest.update(complete=complete, status=status, training_eligible=training_eligible,
                        headway_training_eligible=headway_training_eligible)
        write_json(output / "manifest.json", manifest)
        finalization = {"schema": RUN_SCHEMA, "complete": complete, "status": status,
                        "training_eligible": training_eligible, "label_support": support,
                        "headway_training_eligible": headway_training_eligible, "closed_utc": utc(),
                        "manifest_sha256": sha256_file(output / "manifest.json"),
                        "audit_sha256": sha256_file(output / "audit.json")}
        write_json(output / "finalization.json", finalization)
        return manifest
    except BaseException as error:
        if events is not None and hasattr(events, "close"):
            events.close()
        manifest.update(complete=False, training_eligible=False, headway_training_eligible=False,
                        status="failed", finished_utc=utc(),
                        error=f"{type(error).__name__}: {error}",
                        resource_failure=isinstance(error, ResourceError))
        write_json(output / "manifest.json", manifest)
        write_json(output / "finalization.json", {"schema": RUN_SCHEMA, "complete": False,
            "status": "failed", "training_eligible": False, "headway_training_eligible": False, "closed_utc": utc(),
            "error": manifest["error"], "resource_failure": isinstance(error, ResourceError),
            "manifest_sha256": sha256_file(output / "manifest.json")})
        raise
    finally:
        if events is not None and hasattr(events, "close"):
            events.close()
        store.close()


def owned_rss_bytes(pid: int) -> int:
    """Read only this worker and its descendants, not unrelated processes."""
    pending, visited, rss = [pid], set(), 0
    while pending:
        current = pending.pop()
        if current in visited:
            continue
        visited.add(current)
        base = Path("/proc") / str(current)
        try:
            pages = int((base / "statm").read_text().split()[1])
            rss += pages * os.sysconf("SC_PAGE_SIZE")
            pending.extend(int(x) for x in (base / "task" / str(current) / "children").read_text().split())
        except (OSError, ValueError, IndexError):
            # A just-exited descendant is normal. Parent liveness is separate.
            continue
    return rss


def process_identity(pid: int) -> tuple[int, int, int] | None:
    """Return pgrp/session/birth ticks for one explicitly owned PID."""
    try:
        text = (Path("/proc") / str(pid) / "stat").read_text()
    except FileNotFoundError:
        return None
    fields = text[text.rfind(")") + 2:].split()
    return int(fields[2]), int(fields[3]), int(fields[19])


def group_alive(group: int) -> bool:
    try:
        os.killpg(group, 0)
    except ProcessLookupError:
        return False
    return True


def stop_owned_group(process: subprocess.Popen) -> bool:
    """Reap the *whole* private group, including a child of an exited worker.

    PGIDs still occupied by descendants cannot be allocated as a new process
    group. If a leader PID does exist, its birth ticks must match our launch.
    A lingering/zombie group fails closed rather than launching another job.
    """
    group = process.pid
    if group == os.getpgrp() or not getattr(process, "_discharge_owned_session", False):
        raise RuntimeError("refusing to signal a group not launched by this collector")
    process.poll()  # reap leader when possible, but do not infer child exit
    if not group_alive(group):
        return False
    identity = process_identity(group)
    if identity is not None:
        expected = getattr(process, "_discharge_birth_ticks", None)
        if identity[:2] != (group, group) or expected is None or identity[2] != expected:
            raise RuntimeError("refusing to signal a reused or unverified worker PID")
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            process.poll()
            if not group_alive(group):
                return True
            time.sleep(0.1)
    raise ResourceError("owned process group has not disappeared; no next job will be launched")


def verify_finalization(output: Path, *, require_trainable: bool = True) -> dict:
    final = load_json(output / "finalization.json")
    if final.get("schema") != RUN_SCHEMA or final.get("manifest_sha256") != sha256_file(output / "manifest.json"):
        raise QualityError("missing/mismatched finalization contract")
    manifest = load_json(output / "manifest.json")
    if final.get("complete") is not True or manifest.get("complete") is not True:
        raise QualityError("run is not complete")
    if final.get("audit_sha256") != sha256_file(output / "audit.json") or load_json(output / "audit.json").get("passed") is not True:
        raise QualityError("terminal audit not bound/passed")
    if set(manifest.get("file_sha256", {})) != {"raw_events.jsonl", "sequences.jsonl", "passages.jsonl", "coverage.json"}:
        raise QualityError("missing required output hashes")
    expected_supplemental = {"audit.json", "tripinfo.xml"}
    if manifest.get("observation_policy") == OBSERVATION_POLICY:
        expected_supplemental.update({"rejected_records.jsonl", "valid_passages.jsonl"})
    if set(manifest.get("supplemental_file_sha256", {})) != expected_supplemental:
        raise QualityError("missing required supplemental hashes")
    hashes = {**manifest["file_sha256"], **manifest["supplemental_file_sha256"]}
    for name, expected in hashes.items():
        if Path(name).name != name or sha256_file(output / name) != expected:
            raise QualityError("output hash mismatch: " + name)
    coverage = load_json(output / "coverage.json")
    support = label_support(output, manifest["run_id"], coverage)
    if canonical_hash(manifest.get("totals")) != canonical_hash(coverage["totals"]):
        raise QualityError("manifest totals differ from bound coverage")
    if type(manifest.get("sequence_count")) is not int or manifest["sequence_count"] != support["sequences"]:
        raise QualityError("manifest sequence_count differs from final safe sequences")
    expected_status = "complete" if support["has_any_labels"] else "complete_no_valid_labels"
    for record in (manifest, final):
        if canonical_hash(record.get("label_support")) != canonical_hash(support):
            raise QualityError("label_support differs from final safe sequences")
        if (record.get("training_eligible") is not support["has_any_labels"] or
                record.get("headway_training_eligible") is not support["has_headway_labels"] or
                record.get("status") != expected_status):
            raise QualityError("sample eligibility/status is inconsistent")
    if require_trainable and not support["has_any_labels"]:
        raise QualityError("run is complete but has no valid training labels")
    return final


def record_completed_run(state: dict, entry: dict, output: Path) -> None:
    """A clean empty run is finished, not quarantined and never accepted."""
    final = verify_finalization(output, require_trainable=False)
    entry.update(status=final["status"], label_support=final["label_support"],
                 training_eligible=final["training_eligible"],
                 headway_training_eligible=final["headway_training_eligible"])
    if final["training_eligible"]:
        state["training_eligible"].append(str(output))
    else:
        state["complete_no_valid_labels"].append(str(output))
    if final["headway_training_eligible"]:
        state["headway_training_eligible"].append(str(output))


def quarantine_output(output: Path, error: str) -> None:
    """Parent fail-closed downgrade; preserves every raw/result artifact."""
    path = output / "manifest.json"
    if not path.exists():
        return
    manifest = load_json(path)
    manifest.update(complete=False, training_eligible=False, headway_training_eligible=False,
                    status="quarantined", supervisor_error=error)
    write_json(path, manifest)
    write_json(output / "finalization.json", {"schema": RUN_SCHEMA, "complete": False,
        "training_eligible": False, "headway_training_eligible": False,
        "status": "quarantined", "closed_utc": utc(),
        "error": error, "manifest_sha256": sha256_file(path)})


def runtime_code_hashes() -> dict:
    import features.discharge_sequences as core_module
    import collection.discharge_sumo as old_adapter_module
    import collection.collect_discharge_sequences as old_collector_module
    import collection.discharge_batch_sumo_v2 as new_adapter_module
    import features.discharge_e1_quality as quality_module
    import training.train_discharge_sequences as label_validator_module
    import roadnet_paths as paths_module
    paths = [Path(__file__), *(Path(module.__file__) for module in
             (core_module, old_adapter_module, old_collector_module, new_adapter_module,
              label_validator_module, paths_module, quality_module))]
    return {path.name: sha256_file(path) for path in paths}


def verify_admission(receipt_path: Path, plan: dict) -> dict:
    receipt = load_json(receipt_path)
    if receipt.get("schema") != "discharge-batch-validation-v1" or receipt.get("passed") is not True:
        raise QualityError("formal launch requires a passed validation receipt")
    if receipt.get("plan_sha256") != canonical_hash(plan):
        raise QualityError("validation receipt does not bind this exact plan")
    if receipt.get("code_sha256") != runtime_code_hashes():
        raise QualityError("validation receipt does not bind the current code")
    checks = receipt.get("checks", {})
    for name in ("unit_tests", "small_network_pilot", "manhattan_network_pilot"):
        item = checks.get(name, {})
        if item.get("passed") is not True or not Path(item.get("path", "")).is_absolute():
            raise QualityError("required validation missing: " + name)
        if sha256_file(Path(item["path"])) != item.get("sha256"):
            raise QualityError("validation evidence changed: " + name)
    return receipt


def run_batch(plan_path: Path, root: Path, *, pilot_id: str | None = None,
              horizon: float | None = None, receipt_path: Path | None = None) -> dict:
    if os.name != "posix":
        raise RuntimeError("guarded batch launch is Linux-only; use tests or validate-plan locally")
    plan = load_json(plan_path)
    readiness = validate_plan(plan)
    if horizon is None:
        if receipt_path is None:
            raise QualityError("formal launch requires --validation-receipt after pilot review")
        admission = verify_admission(receipt_path, plan)
    else:
        admission = None
    root = root.resolve()
    # Formal collection must use the user-authorized spacious home volume.
    if not root.is_relative_to(Path("/home")) or root == Path("/home"):
        raise ValueError("batch output must be an explicit new directory below /home")
    if root.exists():
        raise FileExistsError("batch output already exists; no resume/overwrite: " + str(root))
    if pilot_id is not None and pilot_id not in {job["run_id"] for job in plan["jobs"]}:
        raise ValueError("pilot run_id is not in plan")
    limits = readiness["resource_limits"]
    ensure_disk(root.parent, limits)
    root.mkdir(parents=True, exist_ok=False)
    (root / "runs").mkdir()
    write_json(root / "plan.json", plan)
    write_json(root / "readiness.json", readiness)
    if admission is not None:
        write_json(root / "validation_receipt.json", admission)
    state = {"schema": "discharge-batch-state-v1", "status": "running", "created_utc": utc(),
             "plan_sha256": readiness["plan_sha256"], "jobs": [], "training_eligible": [],
             "headway_training_eligible": [], "complete_no_valid_labels": []}
    write_json(root / "status.json", state)
    jobs = [job for job in plan["jobs"] if pilot_id is None or job["run_id"] == pilot_id]
    for job in jobs:
        if (root / "STOP_BEFORE_NEXT_RUN").exists():
            state["status"] = "stopped_before_next_run"
            break
        try:
            ensure_disk(root, limits)
            if canonical_hash(load_json(root / "plan.json")) != readiness["plan_sha256"]:
                raise ResourceError("frozen plan changed; refusing to launch a worker")
            if admission is not None:
                verify_admission(root / "validation_receipt.json", plan)
        except (ResourceError, QualityError, OSError, ValueError) as error:
            state.update(status="resource_blocked" if isinstance(error, ResourceError) else "contract_blocked",
                         error=str(error))
            break
        output = root / "runs" / job["run_id"]
        entry = {"run_id": job["run_id"], "status": "running", "started_utc": utc(), "output": str(output)}
        state["jobs"].append(entry)
        write_json(root / "status.json", state)
        command = [sys.executable, "-u", "-m", "collection.collect_discharge_batch", "_worker",
                   "--plan", str(root / "plan.json"), "--run-id", job["run_id"], "--output-root", str(output)]
        if horizon is not None:
            command += ["--horizon", str(horizon)]
        environment = dict(os.environ)
        environment.update({name: "1" for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                            "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS")})
        process = None
        error = None
        with (root / (job["run_id"] + ".stdout.log")).open("wb") as stdout, (root / (job["run_id"] + ".stderr.log")).open("wb") as stderr:
            start = time.monotonic()
            try:
                process = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=environment,
                                           start_new_session=True, preexec_fn=lambda: os.nice(limits["nice"]))
                process._discharge_owned_session = True
                identity = process_identity(process.pid)
                process._discharge_birth_ticks = identity[2] if identity else None
                entry.update(worker_pid=process.pid, owned_process_group=process.pid, command=command)
                write_json(root / "status.json", state)
                peak_rss = 0
                while process.poll() is None:
                    ensure_disk(root, limits)
                    elapsed = time.monotonic() - start
                    if elapsed > limits["max_wall_hours"] * 3600:
                        raise ResourceError("worker wall-time exceeded")
                    rss = owned_rss_bytes(process.pid)
                    peak_rss = max(peak_rss, rss)
                    if rss > limits["max_rss_gib"] * (1024 ** 3):
                        raise ResourceError("worker process-tree RSS limit exceeded")
                    heartbeat = output / "status.json"
                    if heartbeat.exists() and time.time() - heartbeat.stat().st_mtime > limits["heartbeat_wall_seconds"]:
                        # Closed native output may take longer than 900s to
                        # hash/export. Whole-job wall/RSS/disk guards remain.
                        if load_json(heartbeat).get("phase") != "finalizing":
                            raise ResourceError("worker progress heartbeat stale; no restart authorized")
                    if not heartbeat.exists() and elapsed > limits["heartbeat_wall_seconds"]:
                        raise ResourceError("worker initialization produced no progress heartbeat")
                    if (root / "STOP_ALL_OWN_WORKERS").exists():
                        raise ResourceError("own batch stop marker")
                    time.sleep(limits["check_seconds"])
                entry.update(returncode=process.returncode, wall_seconds=time.monotonic() - start,
                             peak_process_tree_rss_bytes=peak_rss)
                if stop_owned_group(process):
                    # A normal adapter closes/waits for native SUMO before
                    # returning. Remaining native children invalidate success.
                    entry["orphan_group_cleaned"] = True
                    raise QualityError("worker exited but left its own process group running")
            except BaseException as caught:
                error = caught
                if process is not None:
                    try:
                        stop_owned_group(process)
                    except BaseException as cleanup_error:
                        entry["cleanup_error"] = f"{type(cleanup_error).__name__}: {cleanup_error}"
                        error = ResourceError("own worker cleanup unconfirmed; original error: " + str(caught))
                entry.update(error=f"{type(error).__name__}: {error}", wall_seconds=time.monotonic() - start)
        entry["finished_utc"] = utc()
        if error is not None:
            entry["status"] = "resource_aborted" if isinstance(error, ResourceError) else "supervisor_failed"
            # A killed worker may not execute its finally; authoritative parent
            # quarantine prevents treating its files as successfully completed.
            write_json(root / (job["run_id"] + ".quarantine.json"), entry)
            try:
                quarantine_output(output, entry["error"])
            except (OSError, ValueError) as quarantine_error:
                entry["quarantine_write_error"] = str(quarantine_error)
            state["status"] = "resource_blocked" if isinstance(error, ResourceError) else "supervisor_failed"
            break
        if process.returncode == 0 and horizon is not None:
            entry["status"] = "diagnostic_only"
        elif process.returncode == 0:
            try:
                record_completed_run(state, entry, output)
            except (OSError, ValueError, QualityError) as caught:
                entry.update(status="quarantined", error=str(caught))
                quarantine_output(output, str(caught))
        else:
            entry["status"] = "resource_failed" if process.returncode == 3 else "failed_quarantined"
            if process.returncode == 3:
                state["status"] = "resource_blocked"
                break
        write_json(root / "status.json", state)
    if state["status"] == "running":
        state["status"] = "complete_with_quarantine" if any(job["status"] not in {"complete", "complete_no_valid_labels", "diagnostic_only"} for job in state["jobs"]) else "complete"
    state["finished_utc"] = utc()
    write_json(root / "status.json", state)
    write_json(root / "accepted_runs.json", {"schema": "discharge-accepted-runs-v1",
               "plan_sha256": readiness["plan_sha256"], "runs": state["training_eligible"],
               "headway_runs": state["headway_training_eligible"],
               "complete_no_valid_labels": state["complete_no_valid_labels"],
               "diagnostic_only": horizon is not None, "note": "separate domains must be trained separately"})
    return state


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="command", required=True)
    for name in ("validate-plan", "run", "pilot", "_worker"):
        item = commands.add_parser(name)
        item.add_argument("--plan", type=Path, required=True)
        item.add_argument("--output-root", type=Path, required=True)
        if name == "run":
            item.add_argument("--validation-receipt", type=Path, required=True)
        if name in {"pilot", "_worker"}:
            item.add_argument("--run-id", required=True)
            item.add_argument("--horizon", type=float, required=name == "pilot")
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "validate-plan":
            print(json.dumps(validate_plan(load_json(args.plan)), ensure_ascii=False, indent=2))
            return 0
        if args.command == "_worker":
            plan = load_json(args.plan)
            if args.horizon is None:
                verify_admission(args.plan.parent / "validation_receipt.json", plan)
            matches = [job for job in plan["jobs"] if job["run_id"] == args.run_id]
            if len(matches) != 1:
                raise ValueError("worker run_id must identify exactly one planned job")
            if args.horizon is not None:
                positive(args.horizon, "horizon")
            result = collect_job(matches[0], args.output_root, resource_limits(plan),
                {**DEFAULT_OBSERVATION, **plan.get("observation", {})}, horizon=args.horizon)
        else:
            if args.command == "pilot":
                positive(args.horizon, "horizon")
            result = run_batch(args.plan, args.output_root,
                pilot_id=getattr(args, "run_id", None), horizon=getattr(args, "horizon", None),
                receipt_path=getattr(args, "validation_receipt", None))
        print(json.dumps({key: result.get(key) for key in ("status", "complete", "run_id")}, ensure_ascii=False))
        return 0 if result["status"] in {"complete", "complete_no_valid_labels", "complete_with_quarantine", "diagnostic_complete"} else 2
    except ResourceError as error:
        print(f"RESOURCE_BLOCKED: {error}", file=sys.stderr)
        return 3
    except (OSError, ValueError, RuntimeError, ET.ParseError) as error:
        print(f"COLLECTION_FAILED: {type(error).__name__}: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
