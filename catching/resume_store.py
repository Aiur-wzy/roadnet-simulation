"""Disk-backed native catch checkpoints; never turn partial replay into scores.

SQLite transactions commit prediction batches and verified replay receipts.
Two OS locks distinguish the coordinator from its possibly orphaned C++ child.
No collector, historical table, or model-fitting code is imported here.
"""
from __future__ import annotations

import csv
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import sqlite3
import time
import uuid


def atomic_bytes(path, data):
    path = Path(path)
    temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".partial")
    with temporary.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    # Windows readers must use share-delete, or close promptly. Bounded retry;
    # never overwrite the old committed checkpoint with a truncated file.
    for attempt in range(10):
        try:
            os.replace(temporary, path)
            break
        except PermissionError:
            if attempt == 9:
                raise
            time.sleep(.1)
    sync_directory(path.parent)


def sync_directory(path):
    if os.name != "nt":
        fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def atomic_json(path, value):
    from catching.catch_road_time import encoded_json
    atomic_bytes(path, encoded_json(value) + b"\n")


class FileLock:
    """Nonblocking interprocess lock, compatible with NativeReplayGuard."""
    def __init__(self, path):
        self.path = Path(path)
        self.handle = None

    def __enter__(self):
        if os.name == "nt":
            kernel = ctypes.WinDLL("kernel32", use_last_error=True)
            kernel.CreateFileW.restype = ctypes.c_void_p
            kernel.CreateFileW.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32,
                                          ctypes.c_void_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p]
            handle = kernel.CreateFileW(str(self.path.resolve()), 0xC0000000, 0, None, 4, 0x80, None)
            if handle == ctypes.c_void_p(-1).value:
                raise RuntimeError(f"Catch/replay lock busy or inaccessible: {self.path}")
            self.handle = handle
        else:
            import fcntl
            stream = self.path.open("a+b")
            try:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                stream.close()
                raise RuntimeError(f"Catch/replay lock busy: {self.path}") from None
            self.handle = stream
        return self

    def __exit__(self, *exc):
        if self.handle is not None:
            if os.name == "nt":
                kernel = ctypes.WinDLL("kernel32", use_last_error=True)
                kernel.CloseHandle.argtypes = [ctypes.c_void_p]
                kernel.CloseHandle(self.handle)
            else:
                self.handle.close()
            self.handle = None


def require_space(path, reserve, additional=0):
    if shutil.disk_usage(path).free < reserve + additional:
        raise RuntimeError("Output disk reserve reached; committed checkpoint retained; no valid new scores")


class Store:
    def __init__(self, path, binding):
        self.db = sqlite3.connect(path)
        self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.db.execute("PRAGMA cache_size=-32768")
        self.db.execute("PRAGMA temp_store=FILE")
        with self.db:
            self.db.execute("CREATE TABLE IF NOT EXISTS state (name TEXT PRIMARY KEY, value TEXT NOT NULL)")
            self.db.execute("CREATE TABLE IF NOT EXISTS nodes (g INTEGER, r INTEGER, value REAL, batch INTEGER, PRIMARY KEY(g,r)) WITHOUT ROWID")
            self.db.execute("CREATE INDEX IF NOT EXISTS pending ON nodes(g,r) WHERE value IS NULL")
            self.db.execute("CREATE INDEX IF NOT EXISTS batches_nodes ON nodes(batch,g,r) WHERE batch IS NOT NULL")
            self.db.execute("CREATE TABLE IF NOT EXISTS batches (id INTEGER PRIMARY KEY, n INTEGER, sha TEXT)")
            old = self.get("binding")
            if old is not None and old != binding:
                raise ValueError("Checkpoint scope/plan/source binding differs")
            self.put("binding", binding)
            if self.get("counts") is None:
                self.put("counts", self.actual_counts())

    def get(self, name):
        row = self.db.execute("SELECT value FROM state WHERE name=?", (name,)).fetchone()
        return None if row is None else json.loads(row[0])

    def put(self, name, value):
        from catching.catch_road_time import encoded_json
        self.db.execute("INSERT OR REPLACE INTO state VALUES (?,?)", (name, encoded_json(value).decode("utf-8")))

    def save(self, name, value):
        with self.db:
            self.put(name, value)

    def counts(self):
        return self.get("counts")

    def actual_counts(self):
        return dict(zip(("total", "predicted", "pending"), self.db.execute(
            "SELECT COUNT(*), COALESCE(SUM(value IS NOT NULL),0), COALESCE(SUM(value IS NULL),0) FROM nodes").fetchone()))

    def rows(self):
        return self.db.execute("SELECT g,r,value FROM nodes WHERE value IS NOT NULL ORDER BY g,r")

    def verify(self):
        from catching import catch_road_time as c
        if self.db.execute("PRAGMA integrity_check").fetchall() != [("ok",)]:
            raise ValueError("Checkpoint SQLite integrity failure")
        count = 0
        for batch, expected, sha in self.db.execute("SELECT id,n,sha FROM batches ORDER BY id"):
            digest, n = hashlib.sha256(), 0
            for row in self.db.execute("SELECT g,r,value FROM nodes WHERE batch=? ORDER BY g,r", (batch,)):
                digest.update(c.RECORD.pack(*row))
                n += 1
            if n != expected or digest.hexdigest() != sha:
                raise ValueError("Committed prediction batch digest differs")
            count += n
        if self.actual_counts() != self.counts() or count != self.counts()["predicted"] or self.db.execute(
                "SELECT COUNT(*) FROM nodes WHERE (value IS NULL) != (batch IS NULL)").fetchone()[0]:
            raise ValueError("Checkpoint prediction coverage differs")
        for name, raw in self.db.execute("SELECT name,value FROM state WHERE name LIKE 'receipt:%' OR name LIKE 'cache:%' OR name LIKE 'export:%'"):
            receipt = json.loads(raw)
            c.verify_sources(receipt["files"])

    def import_requests(self, path, plan, ceiling, expected=None):
        """SQL dedup instead of a Python set; transaction publishes all or none."""
        from catching import catch_road_time as c
        codec, rows = c.KeyCodec(plan), 0
        self.db.execute("DROP TABLE IF EXISTS temp.incoming")
        self.db.execute("CREATE TEMP TABLE incoming (g INTEGER,r INTEGER,PRIMARY KEY(g,r)) WITHOUT ROWID")
        with Path(path).open(newline="", encoding="utf-8-sig") as stream, self.db:
            reader = csv.DictReader(stream)
            if reader.fieldnames != ["group_index", "road_flow"]:
                raise ValueError("Requests require group_index,road_flow CSV")
            chunk = []
            for row in reader:
                if None in row or any(not re.fullmatch(r"[0-9]+", row.get(n) or "") for n in reader.fieldnames):
                    raise ValueError("Malformed native demand request")
                key = int(row["group_index"]), int(row["road_flow"])
                codec.decode(key)
                chunk.append(key)
                rows += 1
                if len(chunk) == 20000:
                    self.db.executemany("INSERT OR IGNORE INTO incoming VALUES (?,?)", chunk)
                    chunk.clear()
            self.db.executemany("INSERT OR IGNORE INTO incoming VALUES (?,?)", chunk)
            unique = self.db.execute("SELECT COUNT(*) FROM incoming").fetchone()[0]
            if expected is not None:
                if unique != expected["missing_unique"] or rows not in (unique, expected["missing_queries"]):
                    raise ValueError("Discovery requests/status disagree")
                if self.db.execute("SELECT 1 FROM incoming JOIN nodes USING(g,r) WHERE nodes.value IS NOT NULL LIMIT 1").fetchone():
                    raise ValueError("Discovery requested an already cached key")
            added = self.db.execute("INSERT OR IGNORE INTO nodes(g,r) SELECT g,r FROM incoming").rowcount
            counts = self.counts()
            counts["total"] += added
            counts["pending"] += added
            if ceiling and counts["total"] > ceiling:
                raise ValueError("Accumulated demand exceeds explicit key limit")
            self.put("counts", counts)
        self.db.execute("DROP TABLE incoming")
        return {"rows": rows, "unique": unique}

    def commit_predictions(self, values):
        from catching.catch_road_time import RECORD
        digest = hashlib.sha256()
        with self.db:
            batch = self.db.execute("SELECT COALESCE(MAX(id),0)+1 FROM batches").fetchone()[0]
            for key in sorted(values):
                value = values[key]
                if not math.isfinite(value) or value < 1:
                    raise ValueError("Invalid committed prediction")
                changed = self.db.execute("UPDATE nodes SET value=?,batch=? WHERE g=? AND r=? AND value IS NULL",
                                          (value, batch, *key)).rowcount
                if changed != 1:
                    raise ValueError("Prediction commit is not a pending key")
                digest.update(RECORD.pack(*key, value))
            self.db.execute("INSERT INTO batches VALUES (?,?,?)", (batch, len(values), digest.hexdigest()))
            counts = self.counts()
            counts["predicted"] += len(values)
            counts["pending"] -= len(values)
            self.put("counts", counts)

    def close(self):
        self.db.close()


def export_cache(store, path, plan, scope, reserve):
    from catching import catch_road_time as c
    name = "cache:" + path.name
    recorded = store.get(name)
    if recorded:
        c.verify_sources(recorded["files"])
        return path
    n = store.counts()["predicted"]
    require_space(path.parent, reserve, n * c.RECORD.size + c.MAX_HEADER)
    digest = hashlib.sha256()
    for row in store.rows():
        digest.update(c.RECORD.pack(*row))
    header = dict(plan, scope_sha256=scope, entry_count=n, payload_bytes=n*c.RECORD.size,
                  payload_sha256=digest.hexdigest())
    data = c.encoded_json(header)
    if len(data) > c.MAX_HEADER or n*c.RECORD.size > c.MAX_JSON_INTEGER:
        raise ValueError("Export exceeds exact transport range")
    if path.exists():
        existing, _ = c.read_cache(path, plan, scope, retain_values=False)
        if existing != header:
            raise ValueError("Uncommitted cache differs from current checkpoint")
    else:
        temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".partial")
        with temporary.open("xb") as stream:
            stream.write(c.MAGIC.encode("ascii") + b"\n" + data + b"\n")
            for index, row in enumerate(store.rows()):
                if index % 100000 == 0:
                    require_space(path.parent, reserve)
                stream.write(c.RECORD.pack(*row))
            stream.flush()
            os.fsync(stream.fileno())
        c.read_cache(temporary, plan, scope, retain_values=False)
        os.replace(temporary, path)
        sync_directory(path.parent)
    store.save(name, {"files": [c.signature(path)]})
    return path


def estimate(plan, scenarios, counts, rounds):
    from catching import catch_road_time as c
    representative = None
    if plan["road_flow_max"] < c.MAX_NODES and plan["lane_flow_max"] < c.MAX_NODES:
        representative = sum(len(c.flow_nodes(lane, plan["road_flow_max"], plan["road_flow_quantization"]))
                             for lane in range(plan["lane_flow_max"] + 1))
    factor = (len(plan["road_lengths"]) * (plan["free_speed_cap_grid"]["index_max"] -
              plan["free_speed_cap_grid"]["index_min"] + 1) * 6 * plan["signal_point_count"])
    full = None if representative is None else factor * representative
    return {"schema": "native-demand-size-estimate-v1", "record_bytes": c.RECORD.size,
            "cartesian_representative_keys": full,
            "cartesian_payload_bytes_not_allocated": None if full is None else full*c.RECORD.size,
            "planned_visits_per_replay": sum(s["road_count"] for s in scenarios),
            "observed_unique_keys": counts["total"], "committed_predictions": counts["predicted"],
            "current_union_payload_bytes": counts["total"]*c.RECORD.size,
            "discovery_rounds": rounds, "final_count_known": bool(rounds and not rounds[-1]["missing_unique"]),
            "note": "Not a full-grid export. Subsequent closed-loop rounds may discover additional keys; DB, traces, caches and model memory are extra."}


def execute(args, plan, scope, plan_sha, sources, model_files, scenarios, loader, runner):
    from catching import catch_road_time as c
    output = Path(args.output_dir).resolve()
    reserve = c.number(args.min_free_disk_gib, "min free disk GiB") * 1024**3
    binding = {"schema": "native-catch-sqlite-v1", "scope_sha256": scope, "plan_sha256": plan_sha}
    if not args.resume:
        output.mkdir(parents=True, exist_ok=False)
    with FileLock(output / ".catch.lock"):
        # The child itself holds this guard. A killed coordinator cannot hide an
        # active replay. Rotation also invalidates a delayed, old child launch.
        guard = output / ".replay.lock"
        with FileLock(guard):
            token = uuid.uuid4().hex
            atomic_bytes(Path(str(guard) + ".token"), token.encode("ascii"))
        frozen = output / "frozen.json"
        document = dict(binding, plan=plan, sources=sources, arguments={k: v for k, v in vars(args).items()
                                                                      if k not in ("resume", "dry_run")})
        if frozen.exists():
            if c.read_json(frozen) != document:
                raise ValueError("Resume scope/arguments/source fingerprint differs")
        else:
            if args.resume and (output / "checkpoint.sqlite3").exists():
                raise ValueError("Checkpoint lacks immutable frozen contract")
            atomic_json(frozen, document)
        for filename, content in (("plan.json", plan), ("scope.json", dict(binding, sources=sources))):
            destination = output / filename
            if destination.exists() and c.read_json(destination) != content:
                raise ValueError("Frozen plan/scope artifact differs")
            if not destination.exists():
                atomic_json(destination, content)
        require_space(output, reserve)
        store = Store(output / "checkpoint.sqlite3", binding)
        predictor = None
        try:
            if args.resume:
                store.verify()
            rounds = store.get("rounds") or []

            def progress(phase, **details):
                counts = store.counts()
                atomic_json(output / "progress.json", dict(binding, phase=phase, updated_unix=time.time(),
                                                            **counts, **details))
                atomic_json(output / "key_estimate.json", estimate(plan, scenarios, counts, rounds))

            complete = store.get("complete")
            if complete:
                c.verify_sources(complete["files"])
                progress("COMPLETE")
                return complete["summary"]

            def predict_pending():
                nonlocal predictor
                while True:
                    keys = list(store.db.execute("SELECT g,r FROM nodes WHERE value IS NULL ORDER BY g,r LIMIT ?",
                                                 (args.chunk_size,)))
                    if not keys:
                        return
                    require_space(output, reserve)
                    if predictor is None:
                        c.verify_sources(sources)
                        c.verify_model_tree(plan, model_files)
                        predictor = loader(plan["model"]["predictor_path"])
                        c.verify_sources(sources)
                        c.verify_model_tree(plan, model_files)
                    values = c.predict_keys(predictor, plan, keys, args.chunk_size)
                    require_space(output, reserve)
                    store.commit_predictions(values)
                    progress("PREDICTING")

            def replay(scenario, cache, label, discovery):
                name = "receipt:" + label + "/" + scenario["name"]
                recorded = store.get(name)
                if recorded:
                    c.verify_sources(recorded["files"])
                    return recorded
                base = output / label / scenario["name"]
                base.mkdir(parents=True, exist_ok=True)
                attempt = base / ("attempt_" + uuid.uuid4().hex)
                attempt.mkdir()
                command = c.command_for(args, scenario, cache, attempt, discovery)
                command += ["--demand-run-guard", str(guard), "--demand-run-token", token]
                c.write_json(attempt / "command.json", {"command": command, "discovery": discovery})
                progress("DISCOVERY" if discovery else "STRICT_REPLAY", scenario=scenario["name"], attempt=str(attempt))
                require_space(output, reserve)
                cache_signature = c.signature(cache)
                invoke = runner or (lambda cmd, log, timeout: c.run_process(cmd, log, timeout, reserve))
                if invoke(command, attempt / "simulator.log", args.simulation_timeout) != 0:
                    raise RuntimeError("Native replay failed; partial attempt retained; no valid scores")
                c.verify_sources([cache_signature])
                status = c.check_status(attempt / "status.json", scope, scenario, discovery)
                if discovery:
                    request = store.import_requests(attempt / "missing.csv", plan, args.max_cache_entries, status)
                    metrics = None
                else:
                    request, metrics = None, c.check_final(attempt, scenario)
                files = [c.signature(path) for path in sorted(attempt.iterdir()) if path.is_file()]
                receipt = {"files": files, "status": status, "request": request, "metrics": metrics,
                           "directory": str(attempt)}
                store.save(name, receipt)
                progress("SCENARIO_COMMITTED", scenario=scenario["name"])
                return receipt

            progress("VERIFIED_CHECKPOINT" if args.resume else "INITIALIZED")
            if args.requests:
                imported = store.get("requests")
                if imported is None:
                    imported = store.import_requests(args.requests, plan, args.max_cache_entries)
                    store.save("requests", imported)
                predict_pending()
                cache = export_cache(store, output / "table.demand", plan, scope, reserve)
                summary = {"schema": "native-road-time-catch-result-v2", "status": "COMPLETE",
                           "result_role": "request_scoped_table_not_closed_loop_scores", "request_rows": imported["rows"]}
            else:
                state = store.get("position") or {"round": 1, "phase": "discover"}
                while state["phase"] != "final":
                    number_round = state["round"]
                    if state["phase"] == "predict":
                        predict_pending()
                        state = {"round": number_round + 1, "phase": "discover"}
                        store.save("position", state)
                        continue
                    cache = export_cache(store, output / f"cache_{number_round:03d}.demand", plan, scope, reserve)
                    for scenario in scenarios:
                        replay(scenario, cache, f"round_{number_round:03d}", True)
                    counts = store.counts()
                    entry = {"round": number_round, "cached_entries": counts["predicted"],
                             "missing_unique": counts["pending"], "result_role": "provisional_only"}
                    rounds = [r for r in rounds if r["round"] < number_round] + [entry]
                    atomic_json(output / f"round_{number_round:03d}" / "round.json", entry)
                    if counts["pending"] and number_round == args.max_rounds:
                        store.save("rounds", rounds)
                        progress("ROUND_LIMIT")
                        raise RuntimeError("No zero-miss fixed point within max rounds; no valid scores")
                    state = {"round": number_round, "phase": "predict" if counts["pending"] else "final"}
                    with store.db:
                        store.put("rounds", rounds)
                        store.put("position", state)
                    progress("ROUND_COMMITTED")
                cache = output / f"cache_{state['round']:03d}.demand"
                results = [replay(s, cache, "final", False)["metrics"] for s in scenarios]
                count = sum(item["vehicle_count"] for item in results)
                summary = {"schema": "native-road-time-catch-result-v2", "status": "COMPLETE",
                           "result_role": "strict_scenario_replay", "discovery_rounds": rounds, "scenarios": results,
                           "final_strict_missing_queries": 0,
                           "combined": {"vehicle_count": count,
                             "mae_seconds": math.fsum(i["sum_absolute_error_seconds"] for i in results)/count,
                             "bias_seconds": math.fsum(i["sum_error_seconds"] for i in results)/count}}
            for extension, delimiter in (("csv", ","), ("txt", "\t")):
                if args.text_format not in (extension, "both") or store.get("export:" + extension):
                    continue
                require_space(output, reserve)
                path = output / ("predicted_nodes." + extension)
                temporary = path.with_name(path.name + "." + uuid.uuid4().hex + ".partial")
                codec = c.KeyCodec(plan)
                with temporary.open("x", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream, delimiter=delimiter)
                    writer.writerow(["group_index", *c.FEATURE_COLUMNS, c.MODEL_LABEL])
                    for n, (g, r, value) in enumerate(store.rows()):
                        if n % 20000 == 0:
                            require_space(output, reserve)
                        writer.writerow([g, *codec.decode((g, r)), value])
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, path)
                sync_directory(path.parent)
                store.save("export:" + extension, {"files": [c.signature(path)]})
            c.verify_sources(sources)
            c.verify_model_tree(plan, model_files)
            summary.update(scope_sha256=scope, source_label=plan["source_label"], entry_count=store.counts()["predicted"],
                           cache=c.signature(cache), input_semantics_equivalent=False,
                           coverage="only requested/scenario keys; strict missing-key failure elsewhere")
            atomic_json(output / "summary.json", summary)
            store.save("complete", {"summary": summary, "files": [c.signature(output / "summary.json"), c.signature(cache)]})
            progress("COMPLETE")
            return summary
        except BaseException as exc:
            failure = {"status": "FAILED", "exception": type(exc).__name__, "message": str(exc),
                       "formal_results_accepted": False, "committed_checkpoint_retained": True}
            atomic_json(output / "failure.json", failure)
            atomic_json(output / ("failure_" + uuid.uuid4().hex + ".json"), failure)
            atomic_json(output / "progress.json", dict(binding, phase="FAILED", updated_unix=time.time(),
                                                        **store.counts(), error=str(exc)))
            raise
        finally:
            store.close()
