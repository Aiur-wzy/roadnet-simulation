#!/usr/bin/env python3
"""Collect/replay independent physical-lane discharge sequences.

The collector never changes road prediction CSVs, models, or C++ settings.
Use replay for deterministic event fixtures; collect starts its own SUMO process.
Existing output directories are never overwritten.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import sys
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from roadnet_paths import source_path
from typing import Iterable

from features.discharge_sequences import (
    DATASET_SCHEMA, RULE_VERSION, SegmentationConfig, SequenceExtractor, SQLiteStore)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def read_events(path: Path) -> Iterable[dict]:
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key: {key}")
            result[key] = value
        return result

    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8-sig") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                yield json.loads(line, object_pairs_hook=unique_object,
                                 parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            except (json.JSONDecodeError, ValueError) as error:
                raise ValueError(f"{path.name}:{line_number}: invalid event JSON: {error}") from error


def export_jsonl(path: Path, rows: Iterable[dict]) -> int:
    count = 0
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            count += 1
    return count


def canonical_hash(value: dict) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
                                     separators=(",", ":"), allow_nan=False).encode("utf-8")).hexdigest()


def dataset_domain(args: argparse.Namespace, manifest: dict) -> dict:
    domain = {"rule_version": RULE_VERSION, "segmentation": manifest["config"],
              "measurement_method": "normalized-event-replay" if args.command == "replay" else "E1-front-entry"}
    if args.command == "replay":
        domain["physical_configuration"] = "unverified_replay"
        return domain
    metadata = manifest.get("adapter_metadata", {})
    effective = metadata.get("effective_options", {})
    domain.update({"detector_offset": args.detector_offset, "stop_duration": args.stop_duration,
                   "cohort_distance": args.cohort_distance, "queue_gap": args.queue_gap,
                   "max_start_distance": args.max_start_distance,
                   "gate_policy_version": metadata.get("gate_policy_version", "unknown"),
                   "observed_vehicle_type_physics": metadata.get("vehicle_types", {}),
                   "sampling": {key: effective.get(key, "unknown") for key in
                                ("step_length", "integrator", "action_step_length")}})
    return domain


def simulation_identity(args: argparse.Namespace, manifest: dict, output: Path) -> tuple[str, str]:
    if args.command == "replay":
        digest = hashlib.sha256()
        for event in read_events(output / "raw_events.jsonl"):
            event.pop("run_id", None)
            digest.update(json.dumps(event, sort_keys=True, ensure_ascii=False,
                                     separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n")
        return digest.hexdigest(), "normalized_observations_not_physical_configuration"
    metadata = manifest.get("adapter_metadata", {})
    sources = metadata.get("input_files", manifest["source_files"])
    inputs = sorted((entry.get("role", "input"), entry["sha256"]) for entry in sources
                    if entry.get("role") != "sumo_config")
    effective = metadata.get("effective_options", {})
    physical_options = {key: value for key, value in effective.items()
                        if key not in {"end", "verified", "verification_limit"}}
    config_physics = {key: value for key, value in metadata.get("config_values", {}).items()
                      if key not in {"begin", "end", "net-file", "route-files", "additional-files",
                                     "output-prefix", "log", "error-log", "message-log"}
                      and "output" not in key and not key.startswith("save-state")}
    payload = {"inputs": inputs, "sumo_version": metadata.get("sumo_version", "unknown"),
               "effective_options": physical_options, "other_config_options": config_physics,
               "domain": manifest["domain"]}
    quality = "verified_configuration" if effective.get("verified") is True else "declared_and_sampled_configuration"
    return canonical_hash(payload), quality


def positive(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="command", required=True)
    replay = commands.add_parser("replay", help="extract from a normalized observation JSONL[.gz]")
    replay.add_argument("--events", required=True, type=Path)
    collect = commands.add_parser("collect", help="run an independent SUMO observer")
    network = collect.add_mutually_exclusive_group(required=True)
    network.add_argument("--net", type=Path)
    network.add_argument("--sumo-config", type=Path)
    collect.add_argument("--routes", type=Path, nargs="+")
    collect.add_argument("--additional-files", type=Path, nargs="*", default=[])
    collect.add_argument("--sumo-binary", default="sumo")
    collect.add_argument("--end", type=positive, required=True, help="bounded observation horizon in simulation seconds")
    collect.add_argument("--seed", type=int, default=None, help="only overrides the SUMO seed when supplied")
    collect.add_argument("--step-length", type=positive, default=None, help="explicit override; changes simulation dynamics")
    collect.add_argument("--stop-duration", type=positive, default=1.0,
                         help="new-start qualification, never delays a re-stop cut")
    collect.add_argument("--cohort-distance", type=positive, default=100.0)
    collect.add_argument("--queue-gap", type=positive, default=12.0)
    collect.add_argument("--max-start-distance", type=positive, default=15.0)
    collect.add_argument("--detector-offset", type=float, default=0.0,
                         help="meters before lane end; a nonzero value is a different observation section")
    for command in (collect, replay):
        command.add_argument("--output-dir", required=True, type=Path)
        command.add_argument("--run-id", required=True)
        command.add_argument("--epsilon-zero", type=float, default=1e-7)
        command.add_argument("--progress-epsilon", type=float, default=1e-4)
    return result


def run(args: argparse.Namespace) -> dict:
    config = SegmentationConfig(epsilon_zero=args.epsilon_zero, progress_epsilon=args.progress_epsilon)
    if not args.run_id.strip():
        raise ValueError("run-id cannot be empty")
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(f"output directory already exists: {output}")
    if args.command == "replay":
        source = args.events.resolve(strict=True)
        source_files = [{"name": source.name, "sha256": sha256_file(source)}]
    else:
        if args.net and not args.routes:
            raise ValueError("--net requires --routes")
        if args.sumo_config and args.routes:
            raise ValueError("--routes cannot silently override a --sumo-config")
        if not math.isfinite(args.detector_offset) or args.detector_offset < 0:
            raise ValueError("detector-offset must be finite and nonnegative")
        source_files = []
        for source in ([args.net] if args.net else [args.sumo_config]) + (args.routes or []) + args.additional_files:
            source = source.resolve(strict=True)
            source_files.append({"name": source.name, "sha256": sha256_file(source)})
    output.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema": DATASET_SCHEMA, "complete": False, "status": "running", "run_id": args.run_id,
        "mode": args.command, "created_utc": datetime.now(timezone.utc).isoformat(),
        "rule_version": RULE_VERSION, "config": asdict(config), "source_files": source_files,
        "observation_contract": "physical lane exit; known re-stop ends the clean sequence",
        "code_sha256": {Path(__file__).name: sha256_file(Path(__file__)),
                        "discharge_sequences.py": sha256_file(source_path("scripts/discharge_sequences.py"))},
    }
    write_json(output / "manifest.json", manifest)
    store = SQLiteStore(output / "provisional.sqlite")
    events = None
    try:
        with (output / "raw_events.jsonl").open("w", encoding="utf-8", newline="\n") as raw:
            def sink(event: dict) -> None:
                raw.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
            extractor = SequenceExtractor(args.run_id, config=config, store=store, raw_sink=sink)
            if args.command == "replay":
                events = read_events(args.events)
            else:
                from collection.discharge_sumo import iter_sumo_events
                adapter_config = dict(vars(args))
                adapter_config["output_dir"] = output
                adapter_config["metadata"] = {}
                manifest["adapter_metadata"] = adapter_config["metadata"]
                manifest["code_sha256"]["discharge_sumo.py"] = sha256_file(source_path("scripts/discharge_sumo.py"))
                events = iter_sumo_events(adapter_config)
            for event in events:
                extractor.feed(event)
            extractor.finish()
        sequence_count = export_jsonl(output / "sequences.jsonl", store.iter_sequences())
        passage_count = export_jsonl(output / "passages.jsonl", store.iter_passages())
        coverage = extractor.summary()
        write_json(output / "coverage.json", coverage)
        manifest["domain"] = dataset_domain(args, manifest)
        identity, identity_quality = simulation_identity(args, manifest, output)
        manifest["simulation_identity"] = identity
        manifest["simulation_identity_quality"] = identity_quality
        manifest.update(complete=True, status="complete", sequence_count=sequence_count,
                        passage_count=passage_count, totals=coverage["totals"],
                        finished_utc=datetime.now(timezone.utc).isoformat())
        manifest["file_sha256"] = {name: sha256_file(output / name) for name in
                                   ("raw_events.jsonl", "sequences.jsonl", "passages.jsonl", "coverage.json")}
        write_json(output / "manifest.json", manifest)
        return manifest
    except BaseException as error:
        manifest.update(complete=False, status="failed", error=f"{type(error).__name__}: {error}")
        write_json(output / "manifest.json", manifest)
        raise
    finally:
        if events is not None and hasattr(events, "close"):
            events.close()
        store.close()


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        manifest = run(args)
    except (ValueError, OSError, RuntimeError) as error:
        print(f"collection failed: {error}", file=sys.stderr)
        return 2
    print(json.dumps({"status": manifest["status"], "run_id": manifest["run_id"],
                      "sequences": manifest["sequence_count"], "passages": manifest["passage_count"],
                      "output_dir": str(args.output_dir)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
