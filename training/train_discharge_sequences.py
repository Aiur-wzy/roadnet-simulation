#!/usr/bin/env python3
"""Train an auditable per-lane discharge table from closed collector datasets.

This script never invokes SUMO or changes the road travel-time model.  It uses
only the Python standard library.  All records are validated before fitting;
temporary SQLite storage keeps observation memory independent of dataset size.
The startup-delay table is an offline target, NOT permission to add that delay
to the current C++ road prediction (whose timing boundary is different).
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import shutil
import uuid
from collections import Counter
from contextlib import closing, contextmanager
from datetime import datetime, timezone


DATASET_SCHEMA = "lane-discharge-dataset-v1"
SEQUENCE_SCHEMA = "lane-discharge-sequence-v1"
MODEL_SCHEMA = "lane-discharge-model-v1"
RULE_VERSION = "lane-exit-hard-cut-v2.1"
START_KINDS = {"stopped_green_start", "stopped_unblock_start"}
PERMISSION_CLASSES = {"protected", "yield", "unsignalized", "unspecified"}
FEATURES = ("permission_class", "start_kind", "turn", "type_id")
LEVELS = (FEATURES[:1], FEATURES[:2], FEATURES[:3], FEATURES)
TOLERANCE = 1e-7


class DatasetError(ValueError):
    """The dataset cannot be interpreted without silently changing its labels."""


@contextmanager
def temporary_workspace(parent, prefix=".discharge_train_"):
    """Windows sandbox-compatible spool; delete only the exact created child."""
    parent = Path(parent).resolve()
    parent.mkdir(parents=True, exist_ok=True)
    workspace = parent / (prefix + uuid.uuid4().hex)
    workspace.mkdir(mode=0o777)
    try:
        yield workspace
    finally:
        resolved = workspace.resolve()
        if resolved.parent != parent or not resolved.name.startswith(prefix) or resolved != workspace:
            raise RuntimeError("refusing cleanup outside the created training workspace")
        shutil.rmtree(resolved)


def _json_error(value):
    raise DatasetError("non-finite JSON constant: " + value)


def _unique_json_keys(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise DatasetError("duplicate JSON object key: " + key)
        value[key] = item
    return value


def read_json(path):
    try:
        with Path(path).open("r", encoding="utf-8-sig") as stream:
            return json.load(stream, parse_constant=_json_error, object_pairs_hook=_unique_json_keys)
    except (OSError, json.JSONDecodeError) as exc:
        raise DatasetError("cannot read {}: {}".format(path, exc)) from exc


def json_lines(path):
    try:
        with Path(path).open("r", encoding="utf-8-sig") as stream:
            for number, text in enumerate(stream, 1):
                if not text.strip():
                    raise DatasetError("{}:{}: blank JSONL record".format(path, number))
                try:
                    value = json.loads(text, parse_constant=_json_error, object_pairs_hook=_unique_json_keys)
                except (ValueError, json.JSONDecodeError) as exc:
                    raise DatasetError("{}:{}: {}".format(path, number, exc)) from exc
                if not isinstance(value, dict):
                    raise DatasetError("{}:{}: record must be an object".format(path, number))
                yield number, value
    except OSError as exc:
        raise DatasetError("cannot read {}: {}".format(path, exc)) from exc


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _string(value, field):
    if not isinstance(value, str) or not value.strip():
        raise DatasetError(field + " must be a nonempty string")
    return value


def _number(value, field, minimum=0.0, strict=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DatasetError(field + " must be a finite number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite or value < minimum or (strict and value == minimum):
        raise DatasetError(field + " is outside its finite allowed range")
    return float(value)


def _integer(value, field, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise DatasetError(field + " must be an integer >= " + str(minimum))
    return value


def _same_number(actual, expected, field):
    actual = _number(actual, field)
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=TOLERANCE):
        raise DatasetError("{} does not match passage times".format(field))


def validate_sequence(sequence, run_id):
    """Validate one final safe prefix; do not repair, drop or renumber records."""
    if sequence.get("schema") != SEQUENCE_SCHEMA:
        raise DatasetError("unsupported sequence schema")
    if sequence.get("rule_version") != RULE_VERSION:
        raise DatasetError("unsupported sequence rule_version")
    if sequence.get("run_id") != run_id:
        raise DatasetError("sequence run_id differs from manifest")
    for field in ("lane_id", "section_id", "sequence_id", "start_kind", "cut_reason"):
        _string(sequence.get(field), field)
    if sequence["start_kind"] not in START_KINDS:
        raise DatasetError("unsupported start_kind; rolling/unknown cannot become startup prefixes")
    permission_class = sequence.get("permission_class", "unspecified")
    _string(permission_class, "permission_class")
    if permission_class not in PERMISSION_CLASSES:
        raise DatasetError("unsupported permission_class")
    if sequence.get("closed") is not True:
        raise DatasetError("sequence must be closed")
    if type(sequence.get("valid_start")) is not bool:
        raise DatasetError("valid_start must be a boolean")
    anchor = _number(sequence.get("anchor_time"), "anchor_time")
    cut = _number(sequence.get("cut_time"), "cut_time")
    cohort_size = _integer(sequence.get("cohort_size"), "cohort_size")
    passages = sequence.get("passages")
    headways = sequence.get("headways")
    if not isinstance(passages, list) or not isinstance(headways, list):
        raise DatasetError("passages and headways must be lists")
    if not sequence["valid_start"]:
        if passages or headways or sequence.get("startup_delay") is not None:
            raise DatasetError("invalid start may only contain an empty audit segment")
        return sequence
    if cut < anchor:
        raise DatasetError("valid sequence cut precedes its anchor")
    if len(passages) > cohort_size:
        raise DatasetError("passages exceed the initial stopped cohort")
    cohort_keys = sequence.get("cohort_vehicle_keys")
    if cohort_keys is not None:
        if not isinstance(cohort_keys, list) or len(cohort_keys) != cohort_size:
            raise DatasetError("cohort_vehicle_keys must match cohort_size")
        for key in cohort_keys:
            _string(key, "cohort vehicle_key")
        if len(set(cohort_keys)) != len(cohort_keys):
            raise DatasetError("cohort_vehicle_keys contains a repeated visit")
        if [p.get("vehicle_key") for p in passages if isinstance(p, dict)] != cohort_keys[:len(passages)]:
            raise DatasetError("retained passages must be a prefix of the original cohort")
    if len(headways) != max(0, len(passages) - 1):
        raise DatasetError("headway count must equal the full adjacent prefix")
    seen = set()
    previous = None
    for local_j, passage in enumerate(passages, 1):
        if not isinstance(passage, dict):
            raise DatasetError("passage must be an object")
        for field in ("vehicle_key", "vehicle_id", "turn", "type_id"):
            _string(passage.get(field), "passage." + field)
        for field in ("run_id", "lane_id", "section_id"):
            if field in passage and passage[field] != sequence[field]:
                raise DatasetError("passage belongs to a different " + field)
        if passage.get("permission_class", "unspecified") != permission_class:
            raise DatasetError("passage crosses the sequence permission class")
        if passage["vehicle_key"] in seen:
            raise DatasetError("repeated vehicle visit inside sequence")
        seen.add(passage["vehicle_key"])
        if _integer(passage.get("local_j"), "local_j", 1) != local_j:
            raise DatasetError("local_j must be the unbroken prefix 1..n")
        ordinal = _integer(passage.get("lane_passage_ordinal"), "lane_passage_ordinal", 1)
        time = _number(passage.get("time"), "passage.time")
        if time < anchor or time > cut:
            raise DatasetError("passage is outside its anchor/cut interval")
        if time == cut and sequence["cut_reason"] not in ("cohort_end", "observation_end"):
            raise DatasetError("passage touches an interference boundary")
        if previous is not None:
            if time <= previous["time"]:
                raise DatasetError("same-lane passage times must strictly increase")
            if ordinal != previous["lane_passage_ordinal"] + 1:
                raise DatasetError("headway bridges a missing physical passage")
            headway = headways[local_j - 2]
            if not isinstance(headway, dict):
                raise DatasetError("headway must be an object")
            expected = {
                "local_j": local_j,
                "prev_vehicle_key": previous["vehicle_key"],
                "vehicle_key": passage["vehicle_key"],
                "prev_ordinal": previous["lane_passage_ordinal"],
                "ordinal": ordinal,
                "prev_turn": previous["turn"],
                "turn": passage["turn"],
                "prev_type_id": previous["type_id"],
                "type_id": passage["type_id"],
            }
            for field, value in expected.items():
                if headway.get(field) != value or (isinstance(value, int) and type(headway.get(field)) is not int):
                    raise DatasetError("headway endpoint mismatch: " + field)
            _same_number(headway.get("prev_time"), previous["time"], "headway.prev_time")
            _same_number(headway.get("time"), time, "headway.time")
            _number(headway.get("headway"), "headway", strict=True)
            _same_number(headway["headway"], time - previous["time"], "headway")
        previous = passage
    if passages:
        _same_number(sequence.get("startup_delay"), passages[0]["time"] - anchor, "startup_delay")
    elif sequence.get("startup_delay") is not None:
        raise DatasetError("zero-passage sequence cannot have a startup target")
    return sequence


def _create_database(connection):
    connection.executescript("""
        PRAGMA journal_mode=OFF;
        CREATE TABLE sequences (
            id INTEGER PRIMARY KEY, run_id TEXT, lane_id TEXT, section_id TEXT,
            sequence_id TEXT, start_kind TEXT, permission_class TEXT, cut_reason TEXT,
            anchor REAL, cut REAL, valid_start INTEGER, cohort_size INTEGER,
            passage_count INTEGER, split TEXT,
            UNIQUE(run_id,lane_id,section_id,sequence_id));
        CREATE TABLE passages (
            sequence_fk INTEGER, run_id TEXT, lane_id TEXT, section_id TEXT,
            ordinal INTEGER, vehicle_key TEXT, local_j INTEGER, time REAL,
            turn TEXT, type_id TEXT,
            UNIQUE(run_id,lane_id,section_id,ordinal),
            UNIQUE(run_id,lane_id,section_id,vehicle_key));
        CREATE TABLE samples (
            sequence_fk INTEGER, target TEXT, j INTEGER, bucket TEXT,
            value REAL, weight REAL, fit_weight REAL, permission_class TEXT, start_kind TEXT, turn TEXT, type_id TEXT);
        CREATE TABLE audit_passages (
            run_id TEXT,lane_id TEXT,section_id TEXT,ordinal INTEGER,vehicle_key TEXT,
            time REAL,turn TEXT,type_id TEXT,permission_class TEXT,
            PRIMARY KEY(run_id,lane_id,section_id,ordinal),
            UNIQUE(run_id,lane_id,section_id,vehicle_key));
    """)


def choose_split(run_ids, seed=0, validation_fraction=0.2, holdout_runs=None):
    if not 0.0 < validation_fraction < 1.0:
        raise DatasetError("validation_fraction must be strictly between zero and one")
    ordered = sorted(set(run_ids))
    if len(ordered) != len(run_ids):
        raise DatasetError("duplicate run_id: separate directories cannot split one run")
    if holdout_runs:
        held_out = set(holdout_runs)
        if len(held_out) != len(holdout_runs):
            raise DatasetError("duplicate requested holdout run")
        if not held_out.issubset(ordered):
            raise DatasetError("unknown holdout run")
        if held_out == set(ordered):
            raise DatasetError("holdout leaves no training run")
    elif len(ordered) < 2:
        held_out = set()
    else:
        ranked = sorted(ordered, key=lambda run: (
            hashlib.sha256((str(seed) + "\0" + run).encode("utf-8")).hexdigest(), run))
        count = min(len(ordered) - 1, max(1, int(math.ceil(len(ordered) * validation_fraction))))
        held_out = set(ranked[:count])
    return {run: ("validation" if run in held_out else "train") for run in ordered}


def ingest_datasets(connection, directories, prefix_length, seed=0,
                    validation_fraction=0.2, holdout_runs=None):
    manifests = []
    for directory in directories:
        directory = Path(directory).resolve()
        manifest = read_json(directory / "manifest.json")
        if not isinstance(manifest, dict) or manifest.get("schema") != DATASET_SCHEMA:
            raise DatasetError("unsupported dataset manifest: " + str(directory))
        if manifest.get("complete") is not True:
            raise DatasetError("dataset manifest must declare complete=true: " + str(directory))
        _string(manifest.get("run_id"), "manifest.run_id")
        if manifest.get("rule_version") != RULE_VERSION:
            raise DatasetError("unsupported segmentation rule_version")
        if not isinstance(manifest.get("config"), dict):
            raise DatasetError("manifest.config must be an object")
        if not (directory / "passages.jsonl").is_file() or not (directory / "raw_events.jsonl").is_file():
            raise DatasetError("dataset needs passages.jsonl and raw_events.jsonl audit files")
        manifests.append((directory, manifest))
    if not manifests:
        raise DatasetError("at least one dataset is required")
    split = choose_split([m["run_id"] for _, m in manifests], seed, validation_fraction, holdout_runs)
    provenance = []
    domains = set()
    raw_fingerprints = set()
    simulation_identities = set()
    for directory, manifest in manifests:
        run_id = manifest["run_id"]
        adapter = manifest.get("adapter_metadata", {})
        if not isinstance(adapter, dict):
            raise DatasetError("adapter_metadata must be an object")
        domain = manifest.get("domain") or {"rule_version": manifest.get("rule_version", "unspecified"),
                  "segmentation": manifest["config"], "mode": manifest.get("mode", "unspecified"),
                  "training_domain": adapter.get("training_domain")}
        if not isinstance(domain, dict):
            raise DatasetError("manifest.domain must be an object")
        domains.add(json.dumps(domain, sort_keys=True, ensure_ascii=False, allow_nan=False))
        if len(domains) > 1:
            raise DatasetError("incompatible collection domains; train separate tables instead of silently pooling")
        simulation_identity = manifest.get("simulation_identity", adapter.get("simulation_identity"))
        if simulation_identity is not None:
            _string(simulation_identity, "simulation_identity")
            if simulation_identity in simulation_identities:
                raise DatasetError("same physical simulation appears under multiple run IDs")
            simulation_identities.add(simulation_identity)
        hashes = {name: sha256_file(directory / name) for name in (
            "manifest.json", "sequences.jsonl", "passages.jsonl", "raw_events.jsonl")}
        declared_hashes = manifest.get("file_sha256", {})
        if not isinstance(declared_hashes, dict):
            raise DatasetError("file_sha256 must be an object")
        for name, value in declared_hashes.items():
            if name not in ("sequences.jsonl", "passages.jsonl", "raw_events.jsonl", "coverage.json"):
                raise DatasetError("unknown dataset hash filename: " + name)
            if value != sha256_file(directory / name):
                raise DatasetError("dataset artifact does not match declared SHA256: " + name)
        raw_digest = hashlib.sha256()
        raw_count = 0
        for _, event in json_lines(directory / "raw_events.jsonl"):
            if event.get("run_id") != run_id:
                raise DatasetError("raw event run_id differs from manifest")
            canonical = dict(event)
            canonical.pop("run_id", None)
            raw_digest.update(json.dumps(canonical, sort_keys=True, separators=(",", ":"),
                                         ensure_ascii=False, allow_nan=False).encode("utf-8") + b"\n")
            raw_count += 1
        fingerprint = raw_digest.hexdigest()
        if raw_count and fingerprint in raw_fingerprints:
            raise DatasetError("identical raw observation stream appears under different run IDs")
        if raw_count:
            raw_fingerprints.add(fingerprint)
        audit_count = 0
        for line_number, passage in json_lines(directory / "passages.jsonl"):
            try:
                if passage.get("run_id") != run_id:
                    raise DatasetError("audit passage run_id differs from manifest")
                for field in ("lane_id", "section_id", "vehicle_key", "turn", "type_id"):
                    _string(passage.get(field), "audit." + field)
                _integer(passage.get("lane_passage_ordinal"), "audit.ordinal", 1)
                _number(passage.get("time"), "audit.time")
                _string(passage.get("permission_class", "unspecified"), "audit.permission_class")
                connection.execute("INSERT INTO audit_passages VALUES (?,?,?,?,?,?,?,?,?)", (
                    run_id, passage["lane_id"], passage["section_id"], passage["lane_passage_ordinal"],
                    passage["vehicle_key"], passage["time"], passage["turn"], passage["type_id"],
                    passage.get("permission_class", "unspecified")))
                audit_count += 1
            except (DatasetError, sqlite3.IntegrityError) as exc:
                raise DatasetError("{}:{}: {}".format(directory / "passages.jsonl", line_number, exc)) from exc
        rows = 0
        for line_number, sequence in json_lines(directory / "sequences.jsonl"):
            try:
                validate_sequence(sequence, run_id)
                permission_class = sequence.get("permission_class", "unspecified")
                if manifest.get("mode") == "collect" and permission_class == "unspecified":
                    raise DatasetError("live SUMO sequences require an explicit permission_class")
                cursor = connection.execute("""INSERT INTO sequences
                    (run_id,lane_id,section_id,sequence_id,start_kind,permission_class,cut_reason,
                     anchor,cut,valid_start,cohort_size,passage_count,split)
                    VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)""", (
                    run_id, sequence["lane_id"], sequence["section_id"], sequence["sequence_id"],
                    sequence["start_kind"], permission_class, sequence["cut_reason"], sequence["anchor_time"],
                    sequence["cut_time"], int(sequence["valid_start"]), sequence["cohort_size"],
                    len(sequence["passages"]), split[run_id]))
                sequence_fk = cursor.lastrowid
                n_headways = len(sequence["headways"])
                n_tail = sum(h["local_j"] > prefix_length for h in sequence["headways"])
                for passage in sequence["passages"]:
                    connection.execute("INSERT INTO passages VALUES (?,?,?,?,?,?,?,?,?,?)", (
                        sequence_fk, run_id, sequence["lane_id"], sequence["section_id"],
                        passage["lane_passage_ordinal"], passage["vehicle_key"], passage["local_j"],
                        passage["time"], passage["turn"], passage["type_id"]))
                if sequence["passages"]:
                    first = sequence["passages"][0]
                    connection.execute("INSERT INTO samples VALUES (?,?,?,?,?,?,?,?,?,?,?)", (
                        sequence_fk, "startup", 1, "startup", sequence["startup_delay"], 1.0, 1.0,
                        permission_class, sequence["start_kind"], first["turn"], first["type_id"]))
                for h in sequence["headways"]:
                    j = h["local_j"]
                    connection.execute("INSERT INTO samples VALUES (?,?,?,?,?,?,?,?,?,?,?)", (
                        sequence_fk, "headway", j, str(j) if j <= prefix_length else "tail",
                        h["headway"], 1.0 / n_headways, 1.0 if j <= prefix_length else 1.0 / n_tail,
                        permission_class, sequence["start_kind"], h["turn"], h["type_id"]))
                rows += 1
            except (DatasetError, sqlite3.IntegrityError) as exc:
                raise DatasetError("{}:{}: {}".format(directory / "sequences.jsonl", line_number, exc)) from exc
        # Detect source modification during ingestion before accepting its labels.
        if (rows or audit_count) and not raw_count:
            raise DatasetError("nonempty dataset has no original observation events")
        if any(sha256_file(directory / name) != value for name, value in hashes.items()):
            raise DatasetError("dataset changed while it was being read: " + str(directory))
        provenance.append({"run_id": run_id, "split": split[run_id], "directory": str(directory),
                           "sha256": hashes, "sequence_count": rows, "manifest": manifest,
                           "raw_event_count": raw_count, "audit_passage_count": audit_count,
                           "raw_stream_sha256_without_run_id": fingerprint,
                           "domain_verification": "declared_domain_not_physical_reproof" if manifest.get("domain") or adapter.get("training_domain")
                           else "segmentation_only_not_dynamics"})
    connection.executescript("""
        CREATE INDEX samples_lookup ON samples(target,bucket,permission_class,start_kind,turn,type_id);
        CREATE INDEX samples_sequence ON samples(sequence_fk,j);
        CREATE INDEX sequences_group ON sequences(run_id,lane_id,section_id,anchor);
        CREATE INDEX passages_sequence ON passages(sequence_fk,local_j);
    """)
    mismatch = connection.execute("""SELECT p.run_id,p.lane_id,p.ordinal FROM passages p
        JOIN sequences q ON q.id=p.sequence_fk
        LEFT JOIN audit_passages a ON a.run_id=p.run_id AND a.lane_id=p.lane_id
        AND a.section_id=p.section_id AND a.ordinal=p.ordinal
        WHERE a.ordinal IS NULL OR a.vehicle_key<>p.vehicle_key OR ABS(a.time-p.time)>?
        OR a.turn<>p.turn OR a.type_id<>p.type_id OR a.permission_class<>q.permission_class
        LIMIT 1""", (TOLERANCE,)).fetchone()
    if mismatch:
        raise DatasetError("safe-prefix passage disagrees with original audit stream: " + repr(mismatch))
    previous_key, previous_time, previous_ordinal = None, None, None
    for run, lane, section, ordinal, time in connection.execute("""SELECT run_id,lane_id,section_id,ordinal,time
            FROM audit_passages ORDER BY run_id,lane_id,section_id,ordinal"""):
        key = (run, lane, section)
        if key == previous_key:
            if time < previous_time or ordinal != previous_ordinal + 1:
                raise DatasetError("raw physical passage stream has a time reversal or missing ordinal")
        elif ordinal != 1:
            raise DatasetError("raw physical passage stream must begin at ordinal 1")
        previous_key, previous_time, previous_ordinal = key, time, ordinal
    previous_key = None
    previous_cut = None
    for run, lane, section, anchor, cut in connection.execute("""SELECT run_id,lane_id,section_id,anchor,cut
            FROM sequences WHERE valid_start=1 ORDER BY run_id,lane_id,section_id,anchor,cut"""):
        key = (run, lane, section)
        if key == previous_key and anchor < previous_cut:
            raise DatasetError("overlapping valid startup sequences for " + repr(key))
        previous_key, previous_cut = key, cut
    previous_key, previous_time, previous_ordinal = None, None, None
    for run, lane, section, ordinal, time in connection.execute("""SELECT run_id,lane_id,section_id,ordinal,time
            FROM passages ORDER BY run_id,lane_id,section_id,ordinal"""):
        key = (run, lane, section)
        if key == previous_key and (time <= previous_time or ordinal <= previous_ordinal):
            raise DatasetError("physical lane passage order is inconsistent across sequences")
        previous_key, previous_time, previous_ordinal = key, time, ordinal
    connection.commit()
    return provenance, split


def _key(context, fields):
    return tuple(context[field] for field in fields)


def fit_table(connection, prefix_length=8, min_samples=5, min_sequences=3,
              estimator="mean", fixed_headway=2.0):
    if estimator not in ("mean", "median"):
        raise DatasetError("estimator must be mean or median")
    rows = []
    for fields in LEVELS:
        context_sql = "," + ",".join("s." + field for field in fields) if fields else ""
        query = """SELECT s.target,s.bucket{context},COUNT(*),COUNT(DISTINCT s.sequence_fk),
            SUM(s.fit_weight),SUM(s.value*s.fit_weight)/SUM(s.fit_weight),MIN(s.value),MAX(s.value)
            FROM samples s JOIN sequences q ON q.id=s.sequence_fk
            WHERE q.split='train' GROUP BY s.target,s.bucket{context}
            ORDER BY s.target,s.bucket{context}""".format(context=context_sql)
        for record in connection.execute(query):
            target, bucket = record[:2]
            values = record[2:2 + len(fields)]
            count, sequence_count, weight, mean, low, high = record[2 + len(fields):]
            if count < min_samples or sequence_count < min_sequences:
                continue
            estimate = mean
            if estimator == "median":
                conditions = "".join(" AND s." + field + "=?" for field in fields)
                ordered = connection.execute("""SELECT s.value,s.fit_weight FROM samples s
                    JOIN sequences q ON q.id=s.sequence_fk WHERE q.split='train'
                    AND s.target=? AND s.bucket=?""" + conditions + " ORDER BY s.value", (target, bucket) + values)
                cumulative = 0.0
                for value, sample_weight in ordered:
                    cumulative += sample_weight
                    if cumulative >= weight / 2.0:
                        estimate = value
                        break
            if not math.isfinite(estimate) or (target == "headway" and estimate <= 0):
                raise DatasetError("non-finite or non-positive fitted service value")
            rows.append({"target": target, "bucket": bucket, "fields": list(fields),
                         "context": dict(zip(fields, values)), "value": estimate,
                         "samples": count, "sequences": sequence_count,
                         "effective_episode_weight": weight, "observed_min": low, "observed_max": high})
    return {
        "schema": MODEL_SCHEMA, "prefix_length": prefix_length, "estimator": estimator,
        "feature_order": list(FEATURES), "hierarchy": [list(level) for level in LEVELS],
        "weighting": {"fit": "startup and each explicit j: weight 1 per episode; tail: weight 1 per episode distributed over its tail pairs",
                      "metrics": "all headways of an episode jointly weight 1; startup weight 1; unweighted errors also reported"},
        "min_samples": min_samples, "min_sequences": min_sequences,
        "fallback_headway": fixed_headway, "fallback_startup": None,
        "startup_deployment": "offline_only_until_road_timing_contract_is_resolved",
        "semantics": {"unit": "seconds", "sequence_index": "per physical lane or service channel",
                      "lane_count_scaling": "none", "tail": "j > prefix_length; no extrapolation",
                      "missing_prefix": "same-j hierarchy within permission class, then same-class tail, then fixed",
                      "permission_class": "never pool protected, yield and unsignalized; unspecified is replay-only",
                      "missing_startup": "unavailable; never silently substitute zero"},
        "rows": rows,
    }


class Predictor:
    """Reference lookup contract; no lane-count division or future-state inputs."""

    def __init__(self, model):
        if not isinstance(model, dict) or model.get("schema") != MODEL_SCHEMA:
            raise DatasetError("unsupported model schema")
        _integer(model.get("prefix_length"), "model.prefix_length", 2)
        _number(model.get("fallback_headway"), "model.fallback_headway", strict=True)
        if model.get("fallback_startup") is not None:
            raise DatasetError("startup fallback must remain unavailable")
        if model.get("feature_order") != list(FEATURES) or model.get("hierarchy") != [list(level) for level in LEVELS]:
            raise DatasetError("unsupported model feature/hierarchy contract")
        if not isinstance(model.get("rows"), list):
            raise DatasetError("model.rows must be a list")
        self.model = model
        self.lookup = {}
        for row in model["rows"]:
            if not isinstance(row, dict) or not isinstance(row.get("fields"), list):
                raise DatasetError("invalid model table row")
            fields = tuple(row["fields"])
            if fields not in LEVELS or not isinstance(row.get("context"), dict) or set(row["context"]) != set(fields):
                raise DatasetError("invalid model row feature fields")
            for field in fields:
                _string(row["context"][field], "model.context." + field)
            if row["context"]["permission_class"] not in PERMISSION_CLASSES:
                raise DatasetError("invalid model permission class")
            target, bucket = row.get("target"), row.get("bucket")
            if target == "startup":
                if bucket != "startup":
                    raise DatasetError("invalid startup bucket")
                _number(row.get("value"), "startup value")
            elif target == "headway":
                valid_prefix = (isinstance(bucket, str) and bucket.isascii() and bucket.isdecimal()
                                and str(int(bucket)) == bucket and 2 <= int(bucket) <= model["prefix_length"])
                if bucket != "tail" and not valid_prefix:
                    raise DatasetError("invalid headway bucket")
                _number(row.get("value"), "headway value", strict=True)
            else:
                raise DatasetError("invalid model target")
            key = (target, bucket, fields, _key(row["context"], fields))
            if key in self.lookup:
                raise DatasetError("duplicate model lookup row")
            self.lookup[key] = row

    def predict(self, target, local_j, context):
        if target not in ("startup", "headway"):
            raise DatasetError("unknown prediction target")
        _integer(local_j, "local_j", 1 if target == "startup" else 2)
        if target == "startup" and local_j != 1:
            raise DatasetError("startup prediction only accepts local_j=1")
        context = dict(context)
        context.setdefault("permission_class", "unspecified")
        for field in FEATURES:
            _string(context.get(field), "prediction." + field)
        if context["start_kind"] not in START_KINDS:
            return (self.model["fallback_headway"], "fixed_nonstartup") if target == "headway" else (None, "unavailable_nonstartup")
        bucket = "startup" if target == "startup" else (
            str(local_j) if local_j <= self.model["prefix_length"] else "tail")
        for fields in reversed(LEVELS):
            row = self.lookup.get((target, bucket, fields, _key(context, fields)))
            if row is not None:
                return row["value"], "level_" + str(len(fields))
        if target == "headway":
            row = self.lookup.get(("headway", "tail", ("permission_class",), (context["permission_class"],)))
            if row is not None:
                return row["value"], "permission_tail_for_missing_prefix"
            return self.model["fallback_headway"], "fixed"
        return None, "unavailable"


class Metrics:
    def __init__(self):
        self.n = 0
        self.absolute = 0.0
        self.signed = 0.0
        self.square = 0.0
        self.weight = 0.0
        self.weighted_absolute = 0.0
        self.weighted_signed = 0.0

    def add(self, prediction, truth, weight=1.0):
        error = prediction - truth
        self.n += 1
        self.absolute += abs(error)
        self.signed += error
        self.square += error * error
        self.weight += weight
        self.weighted_absolute += weight * abs(error)
        self.weighted_signed += weight * error

    def report(self):
        return {"n": self.n, "mae": self.absolute / self.n if self.n else None,
                "bias": self.signed / self.n if self.n else None,
                "rmse": math.sqrt(self.square / self.n) if self.n else None,
                "episode_weighted_mae": self.weighted_absolute / self.weight if self.weight else None,
                "episode_weighted_bias": self.weighted_signed / self.weight if self.weight else None}


def evaluate(connection, model, split):
    predictor = Predictor(model)
    strata = {}
    fallback = Counter()
    unavailable = Counter()
    baseline = Metrics()
    for target, j, value, weight, permission_class, start_kind, turn, type_id in connection.execute("""SELECT
            s.target,s.j,s.value,s.weight,s.permission_class,s.start_kind,s.turn,s.type_id
            FROM samples s JOIN sequences q ON q.id=s.sequence_fk WHERE q.split=?""", (split,)):
        context = {"permission_class": permission_class, "start_kind": start_kind, "turn": turn, "type_id": type_id}
        prediction, source = predictor.predict(target, j, context)
        fallback[target + ":" + source] += 1
        if target == "headway":
            baseline.add(model["fallback_headway"], value, weight)
        if prediction is None:
            unavailable[target] += 1
            continue
        for key in ((target, "all", "all"), (target, "j", str(j)),
                    (target, "start_kind", start_kind), (target, "turn", turn),
                    (target, "type_id", type_id), (target, "permission_class", permission_class)):
            strata.setdefault(key, Metrics()).add(prediction, value, weight)
    prefix_headway = {}
    prefix_full = {}
    sequence_rows = connection.execute("""SELECT id,start_kind,permission_class,anchor FROM sequences
        WHERE split=? AND passage_count>0 ORDER BY id""", (split,))
    for sequence_fk, start_kind, permission_class, anchor in sequence_rows:
        predicted_h_sum = 0.0
        predicted_s = None
        first_time = None
        for local_j, time, turn, type_id in connection.execute("""SELECT local_j,time,turn,type_id
                FROM passages WHERE sequence_fk=? ORDER BY local_j""", (sequence_fk,)):
            context = {"permission_class": permission_class, "start_kind": start_kind, "turn": turn, "type_id": type_id}
            if local_j == 1:
                first_time = time
                predicted_s, _ = predictor.predict("startup", 1, context)
            else:
                prediction, _ = predictor.predict("headway", local_j, context)
                predicted_h_sum += prediction
                prefix_headway.setdefault(local_j, Metrics()).add(predicted_h_sum, time - first_time)
            if predicted_s is not None:
                prefix_full.setdefault(local_j, Metrics()).add(predicted_s + predicted_h_sum, time - anchor)
    target_status = {}
    for target in ("startup", "headway"):
        selected = {key.split(":", 1)[1]: count for key, count in fallback.items() if key.startswith(target + ":")}
        learned = sum(count for key, count in selected.items() if key.startswith("level_") or key == "permission_tail_for_missing_prefix")
        target_status[target] = {"labels": sum(selected.values()), "learned_predictions": learned,
                                 "status": "available" if learned else (
                                     "insufficient_training_support" if selected else "no_labels")}
    overall_status = "available" if any(row["learned_predictions"] for row in target_status.values()) else (
        "fallback_only_insufficient_training_support" if fallback else "no_predictable_labels")
    return {"split": split, "status": overall_status, "targets": target_status,
            "strata": [{"target": key[0], "dimension": key[1], "value": key[2], **metric.report()}
                       for key, metric in sorted(strata.items())],
            "fallback_counts": dict(sorted(fallback.items())), "unavailable_targets": dict(unavailable),
            "fixed_headway_fallback_fraction": (
                fallback["headway:fixed"] / sum(n for key, n in fallback.items() if key.startswith("headway:"))
                if any(key.startswith("headway:") for key in fallback) else None),
            "fixed_headway_baseline": baseline.report(),
            "cumulative_headway_from_first_passage": {str(j): metric.report() for j, metric in sorted(prefix_headway.items())},
            "cumulative_with_offline_startup": {str(j): metric.report() for j, metric in sorted(prefix_full.items())},
            "limits": ["conditional clean-prefix errors, not full-green throughput or network travel-time accuracy",
                       "startup remains offline-only; real blocking/reset classifier is not evaluated here"]}


def coverage(connection):
    total = connection.execute("SELECT COUNT(*) FROM sequences").fetchone()[0]
    counts = dict(connection.execute("SELECT target,COUNT(*) FROM samples GROUP BY target"))
    return {"sequence_count": total, "valid_start_count": connection.execute(
        "SELECT COUNT(*) FROM sequences WHERE valid_start=1").fetchone()[0],
        "zero_passage_count": connection.execute("SELECT COUNT(*) FROM sequences WHERE passage_count=0").fetchone()[0],
        "startup_targets": counts.get("startup", 0), "headway_targets": counts.get("headway", 0),
        "cut_reasons": dict(connection.execute("SELECT cut_reason,COUNT(*) FROM sequences GROUP BY cut_reason")),
        "permission_classes": dict(connection.execute("SELECT permission_class,COUNT(*) FROM sequences GROUP BY permission_class")),
        "mixed_turn_sequences": connection.execute("""SELECT COUNT(*) FROM
            (SELECT sequence_fk FROM passages GROUP BY sequence_fk HAVING COUNT(DISTINCT turn)>1)""").fetchone()[0],
        "mixed_type_sequences": connection.execute("""SELECT COUNT(*) FROM
            (SELECT sequence_fk FROM passages GROUP BY sequence_fk HAVING COUNT(DISTINCT type_id)>1)""").fetchone()[0],
        "exclusions": "invalid-start and zero-passage segments remain in denominators; no minimum prefix-length exclusion",
        "risk_set_definition": "initial stopped cohort size >= j, including failed starts; observed label counts separate",
        "raw_audit": "files hashed and retained by input path; no claim of re-proving physical interference labels"}


def write_json(path, value):
    with Path(path).open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def train_datasets(directories, output, prefix_length=8, min_samples=5, min_sequences=3,
                   estimator="mean", fixed_headway=2.0, seed=0, validation_fraction=0.2,
                   holdout_runs=None):
    _integer(prefix_length, "prefix_length", 2)
    _integer(min_samples, "min_samples", 1)
    _integer(min_sequences, "min_sequences", 1)
    _number(fixed_headway, "fixed_headway", strict=True)
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise DatasetError("output directory must be absent or empty; existing artifacts are never overwritten")
    for directory in directories:
        source = Path(directory).resolve()
        if source == output or source in output.parents or output in source.parents:
            raise DatasetError("training output must be separate from each source dataset")
    with temporary_workspace(output.parent) as temporary:
        with closing(sqlite3.connect(str(temporary / "observations.sqlite"))) as connection:
            _create_database(connection)
            provenance, split = ingest_datasets(connection, directories, prefix_length, seed,
                                                validation_fraction, holdout_runs)
            train_labels = connection.execute("""SELECT COUNT(*) FROM samples s JOIN sequences q
                ON q.id=s.sequence_fk WHERE q.split='train'""").fetchone()[0]
            if not train_labels:
                raise DatasetError("training runs contain no completed startup or headway targets")
            model = fit_table(connection, prefix_length, min_samples, min_sequences, estimator, fixed_headway)
            metrics = {"train": evaluate(connection, model, "train"),
                       "validation": evaluate(connection, model, "validation")}
            if "validation" not in split.values():
                metrics["validation"]["status"] = "unavailable_insufficient_runs"
            coverage_report = coverage(connection)
            output.mkdir(parents=True, exist_ok=True)
            write_json(output / "manifest.json", {"schema": "lane-discharge-training-v1",
                       "complete": False, "status": "writing_artifacts"})
            write_json(output / "model.json", model)
            write_json(output / "metrics.json", metrics)
            write_json(output / "coverage.json", coverage_report)
            with (output / "coverage_by_lane.jsonl").open("w", encoding="utf-8", newline="\n") as stream:
                lanes = connection.execute("""SELECT run_id,lane_id,section_id,start_kind,permission_class,COUNT(*),
                    SUM(passage_count=0),SUM(valid_start),MAX(cohort_size) FROM sequences
                    GROUP BY run_id,lane_id,section_id,start_kind,permission_class
                    ORDER BY run_id,lane_id,section_id,start_kind,permission_class""")
                for run, lane, section, kind, permission, n, empty, valid, max_cohort in lanes:
                    # Only supported prefix indices plus one aggregated tail threshold;
                    # huge queues cannot create unbounded dense coverage vectors.
                    by_j = []
                    for j in range(1, min(max_cohort, prefix_length + 1) + 1):
                        risk, observed = connection.execute("""SELECT SUM(cohort_size>=?),SUM(passage_count>=?)
                            FROM sequences WHERE run_id=? AND lane_id=? AND section_id=? AND start_kind=?
                            AND permission_class=?""", (j, j, run, lane, section, kind, permission)).fetchone()
                        mean = connection.execute("""SELECT AVG(s.value) FROM samples s
                            JOIN sequences q ON q.id=s.sequence_fk WHERE q.run_id=? AND q.lane_id=?
                            AND q.section_id=? AND q.start_kind=? AND q.permission_class=? AND s.j=?""",
                            (run, lane, section, kind, permission, j)).fetchone()[0]
                        by_j.append({"j": j, "bucket": "tail_entry" if j > prefix_length else str(j),
                                     "risk_sequences": risk, "observed_prefix_sequences": observed,
                                     "conditional_observed_mean": mean,
                                     "target": "startup" if j == 1 else "headway"})
                    row = {"run_id": run, "split": split[run], "lane_id": lane, "section_id": section,
                           "start_kind": kind, "permission_class": permission,
                           "sequences": n, "zero_passages": empty,
                           "valid_starts": valid, "prefix_coverage": by_j}
                    stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            artifacts = {name: sha256_file(output / name) for name in (
                "model.json", "metrics.json", "coverage.json", "coverage_by_lane.jsonl")}
            manifest = {
                "schema": "lane-discharge-training-v1", "complete": True,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "trainer_sha256": sha256_file(Path(__file__)), "input_schema": DATASET_SCHEMA,
                "inputs": provenance, "split": {"unit": "entire run", "seed": seed,
                    "validation_fraction": validation_fraction, "assignments": split,
                    "explicit_holdout_runs": holdout_runs or []},
                "config": {"prefix_length": prefix_length, "min_samples": min_samples,
                    "min_sequences": min_sequences, "estimator": estimator, "fixed_headway": fixed_headway},
                "sha256": artifacts, "validation_status": metrics["validation"]["status"],
                "deployment": "new independent service artifact; no current C++ integration or road catch changes",
                "limitations": ["held-out runs may share routes/seeds; unseen-network generalization requires separately chosen runs",
                    "clean labels depend on collector evidence; raw files are fingerprinted, not physically reclassified",
                    "large-j support is conditional on surviving truncation; no censored targets are imputed",
                    "type_id meanings and simulator domains must be checked across input manifests before deployment"]}
            write_json(output / "manifest.json", manifest)
            return manifest


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("datasets", nargs="+", type=Path, help="complete collector output directories")
    parser.add_argument("--output", type=Path, required=True, help="new or empty artifact directory")
    parser.add_argument("--prefix-length", type=int, default=8, help="last explicit per-lane ordinal (default: 8)")
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--min-sequences", type=int, default=3)
    parser.add_argument("--estimator", choices=("mean", "median"), default="mean")
    parser.add_argument("--fixed-headway", type=float, default=2.0, help="explicit sparse/out-of-domain fallback, seconds")
    parser.add_argument("--seed", type=int, default=0, help="deterministic entire-run split seed")
    parser.add_argument("--validation-fraction", type=float, default=0.2)
    parser.add_argument("--holdout-run", action="append", dest="holdout_runs", help="explicit whole-run validation ID; repeatable")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        manifest = train_datasets(args.datasets, args.output, args.prefix_length, args.min_samples,
                                  args.min_sequences, args.estimator, args.fixed_headway, args.seed,
                                  args.validation_fraction, args.holdout_runs)
    except (DatasetError, OSError, sqlite3.Error) as exc:
        print("error: " + str(exc), file=sys.stderr)
        return 2
    print(json.dumps({"output": str(args.output.resolve()), "complete": True,
                      "validation_status": manifest["validation_status"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
