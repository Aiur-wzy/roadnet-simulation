"""Predict every legal node in the declared native road-time feature grid.

This independent entry point never runs a simulator or discovers demand keys.
Prediction is chunked; complete binary shards are resumable commit units.
Use --dry-run to inspect the full size without loading AutoGluon or writing files.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import sqlite3
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from catching import catch_road_time as core
from catching import resume_store as io

SCHEMA = "native-full-grid-catch-v1"
HEADER_RESERVE = core.MAX_HEADER
PREFIX_BYTES = len(core.MAGIC.encode("ascii")) + 1 + HEADER_RESERVE + 1


class FullGrid:
    """Ordered key indexing without allocating the Cartesian product or flow grid."""

    def __init__(self, plan):
        self.plan = plan
        self.maximum = plan["road_flow_max"]
        self.lanes = plan["lane_flow_max"] + 1
        self.policy = plan["road_flow_quantization"]
        self.explicit = self.policy.get("nodes")
        if self.explicit is not None:
            self.sums = [0, *itertools.accumulate(self.explicit)]
        else:
            self.dense = min(self.maximum, self.policy["dense_until"])
            self.step = self.policy["step"]
            self.first_multiple = self.dense // self.step + 1
            self.multiple_count = max(0, self.maximum // self.step - self.first_multiple + 1)
        self.base_count = self.base_stats(self.maximum + 1)[0]
        self.flow_pairs = self.lane_prefix(self.lanes)
        self.blocks = plan["group_count"] // self.lanes
        self.total = self.blocks * self.flow_pairs

    def base_stats(self, stop):
        """Count and sum of global road-flow representatives strictly below stop."""
        if self.explicit is not None:
            count = bisect_left(self.explicit, stop)
            return count, self.sums[count]
        upper = min(self.maximum, stop - 1)
        if upper < 0:
            return 0, 0
        dense = min(self.dense, upper)
        count, total = dense + 1, dense * (dense + 1) // 2
        last = upper // self.step
        multiples = max(0, last - self.first_multiple + 1)
        count += multiples
        total += self.step * (self.first_multiple + last) * multiples // 2
        if upper == self.maximum and self.maximum > self.dense and self.maximum % self.step:
            count += 1
            total += self.maximum
        return count, total

    def lane_prefix(self, stop):
        # Each lane adds itself when absent from the global nodes. Summing all
        # lanes below stop gives this closed form, even for signed-int domains.
        below, total = self.base_stats(stop)
        return stop * (1 + self.base_count - below) + total

    def base_at(self, index):
        if self.explicit is not None:
            return self.explicit[index]
        if index <= self.dense:
            return index
        index -= self.dense + 1
        if index < self.multiple_count:
            return (self.first_multiple + index) * self.step
        return self.maximum

    def lane_nodes(self, lane, offset=0):
        below = self.base_stats(lane)[0]
        has_lane = self.base_at(below) == lane
        count = self.base_count - below + (not has_lane)
        for index in range(offset, count):
            if not has_lane and index == 0:
                yield lane
            else:
                yield self.base_at(below + index - (not has_lane))

    def keys(self, start=0, stop=None):
        stop = self.total if stop is None else stop
        if not 0 <= start <= stop <= self.total:
            raise ValueError("Full-grid ordinal range outside plan")
        if start == stop:
            return
        block, within = divmod(start, self.flow_pairs)
        lo, hi = 0, self.lanes
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if self.lane_prefix(mid) <= within:
                lo = mid
            else:
                hi = mid
        lane, offset, ordinal = lo, within - self.lane_prefix(lo), start
        while ordinal < stop:
            for road in self.lane_nodes(lane, offset):
                if ordinal == stop:
                    return
                yield block * self.lanes + lane, road
                ordinal += 1
            lane += 1
            offset = 0
            if lane == self.lanes:
                block += 1
                lane = 0

    def estimate(self, shard_rows, merge):
        shards = (self.total + shard_rows - 1) // shard_rows
        binary = self.total * core.RECORD.size + shards * PREFIX_BYTES
        return {
            "full_grid_keys": self.total,
            "factors": {
                "road_lengths": len(self.plan["road_lengths"]),
                "speed_nodes": self.plan["free_speed_cap_grid"]["index_max"]
                               - self.plan["free_speed_cap_grid"]["index_min"] + 1,
                "context_categories": self.plan["context_count"],
                "signal_nodes": self.plan["signal_point_count"],
                "legal_flow_representative_pairs": self.flow_pairs,
            },
            "payload_bytes": self.total * core.RECORD.size,
            "shard_count": shards,
            "binary_shards_bytes": binary,
            "merged_file_extra_bytes": self.total * core.RECORD.size + PREFIX_BYTES if merge else 0,
            "note": "Exact binary size; optional text, checkpoint, failed partials and model memory are extra.",
        }


def shard_name(index):
    return f"shards/part_{index:09d}.demand"


def header_bytes(plan, scope, start, count, digest):
    header = dict(plan, scope_sha256=scope, entry_count=count, payload_bytes=count * core.RECORD.size,
                  payload_sha256=digest, full_grid_export={"schema": SCHEMA, "ordinal_start": start,
                                                         "ordinal_stop": start + count})
    raw = core.encoded_json(header)
    if len(raw) > HEADER_RESERVE or count * core.RECORD.size > core.MAX_JSON_INTEGER:
        raise ValueError("Full-grid shard exceeds native transport bounds")
    return core.MAGIC.encode("ascii") + b"\n" + raw.ljust(HEADER_RESERVE, b" ") + b"\n"


def blank_header(stream):
    stream.write(core.MAGIC.encode("ascii") + b"\n" + b"{}".ljust(HEADER_RESERVE, b" ") + b"\n")


def verify_shard(path, plan, scope, grid, start, count, chunk_size):
    """Validate full ordinal coverage as well as schema, finite values and SHA."""
    payload_sha, file_sha = hashlib.sha256(), hashlib.sha256()
    with Path(path).open("rb") as stream:
        prefix = stream.read(PREFIX_BYTES)
        if not prefix.startswith(core.MAGIC.encode("ascii") + b"\n") or not prefix.endswith(b"\n"):
            raise ValueError("Invalid full-grid binary prefix")
        header = json.loads(prefix.split(b"\n", 1)[1], object_pairs_hook=core.unique_json)
        expected = header_bytes(plan, scope, start, count, header.get("payload_sha256"))
        if prefix != expected or Path(path).stat().st_size != PREFIX_BYTES + count * core.RECORD.size:
            raise ValueError("Full-grid shard header/size differs from frozen plan")
        file_sha.update(prefix)
        keys = grid.keys(start, start + count)
        remaining = count
        while remaining:
            rows = min(chunk_size, remaining)
            data = stream.read(rows * core.RECORD.size)
            if len(data) != rows * core.RECORD.size:
                raise ValueError("Truncated full-grid shard")
            for group, road, value in core.RECORD.iter_unpack(data):
                if (group, road) != next(keys) or not math.isfinite(value) or value < 1:
                    raise ValueError("Full-grid shard has missing/reordered keys or invalid predictions")
            payload_sha.update(data)
            file_sha.update(data)
            remaining -= rows
        if stream.read(1) or payload_sha.hexdigest() != header["payload_sha256"]:
            raise ValueError("Full-grid payload checksum differs")
    return {"bytes": PREFIX_BYTES + count * core.RECORD.size, "sha256": file_sha.hexdigest(),
            "payload_sha256": payload_sha.hexdigest()}


def publish_new(temporary, destination):
    """Never silently replace an uncommitted artifact with different content."""
    if destination.exists():
        if destination.stat().st_size != temporary.stat().st_size or core.sha256_file(destination) != core.sha256_file(temporary):
            raise ValueError(f"Existing uncommitted artifact differs: {destination}")
        temporary.unlink()  # Only this invocation's verified duplicate temporary.
    else:
        os.replace(temporary, destination)
        io.sync_directory(destination.parent)


def export_text(binary, plan, extension, chunk_size, reserve):
    destination = binary.with_suffix("." + extension)
    temporary = destination.with_name(destination.name + "." + uuid.uuid4().hex + ".partial")
    codec = core.KeyCodec(plan)
    with binary.open("rb") as source, temporary.open("x", newline="", encoding="utf-8") as target:
        source.seek(PREFIX_BYTES)
        writer = csv.writer(target, delimiter="," if extension == "csv" else "\t")
        writer.writerow(["group_index", *core.FEATURE_COLUMNS, core.MODEL_LABEL])
        while data := source.read(chunk_size * core.RECORD.size):
            io.require_space(binary.parent, reserve)
            for group, road, value in core.RECORD.iter_unpack(data):
                writer.writerow([group, *codec.decode((group, road)), value])
        target.flush()
        os.fsync(target.fileno())
    publish_new(temporary, destination)
    return {"name": destination.name, "bytes": destination.stat().st_size,
            "sha256": core.sha256_file(destination)}


def _sign(path):
    return {"name": path.name, "bytes": path.stat().st_size, "sha256": core.sha256_file(path)}


def merge_shards(output, connection, plan, scope, grid, chunk_size, reserve):
    """Optional streaming assembly; committed shards remain resumable inputs."""
    destination = output / "table.demand"
    if destination.exists():
        verify_shard(destination, plan, scope, grid, 0, grid.total, chunk_size)
        return _sign(destination)
    io.require_space(output, reserve, PREFIX_BYTES + grid.total * core.RECORD.size)
    temporary = output / ("table.demand." + uuid.uuid4().hex + ".partial")
    digest = hashlib.sha256()
    with temporary.open("xb+") as target:
        blank_header(target)
        for (raw,) in connection.execute("SELECT receipt FROM shards ORDER BY start"):
            receipt = json.loads(raw)
            with (output / receipt["binary"]).open("rb") as source:
                source.seek(PREFIX_BYTES)
                while data := source.read(chunk_size * core.RECORD.size):
                    io.require_space(output, reserve)
                    target.write(data)
                    digest.update(data)
        target.seek(0)
        target.write(header_bytes(plan, scope, 0, grid.total, digest.hexdigest()))
        target.flush()
        os.fsync(target.fileno())
    verify_shard(temporary, plan, scope, grid, 0, grid.total, chunk_size)
    publish_new(temporary, destination)
    return _sign(destination)


def run(args, predictor_loader=None):
    core.integer(args.shard_rows, "shard rows", 1, core.MAX_JSON_INTEGER // core.RECORD.size)
    reserve = core.number(args.min_free_disk_gib, "minimum free disk GiB") * 1024**3
    model_files = []
    plan = core.build_plan(args, source_files=model_files)
    plan["warnings"] = [message for message in plan["warnings"] if not message.startswith("Sparse coverage")]
    plan["warnings"].append("Full declared quantized grid coverage is not a training-coverage or accuracy claim.")
    grid = FullGrid(plan)
    estimate = grid.estimate(args.shard_rows, args.merge_output)
    if args.merge_output and grid.total * core.RECORD.size > core.MAX_JSON_INTEGER:
        raise ValueError("Merged file exceeds native exact-integer transport range; use shards")
    sources = [core.signature(path) for path in
               (args.net_file, __file__, core.__file__, io.__file__, core.native_contract_module.__file__)]
    sources += model_files
    arguments = {key: value for key, value in vars(args).items() if key not in ("resume", "dry_run")}
    frozen = {"schema": SCHEMA, "plan": plan, "sources": sources, "arguments": arguments}
    scope = core.canonical_sha(frozen)
    output = Path(args.output_dir).resolve()
    if args.dry_run:
        return {"schema": SCHEMA, "status": "dry_run", "scope_sha256": scope, "plan": plan,
                "size_estimate": estimate, "model_loaded": False, "files_written": False,
                "simulator_used": False}
    if args.resume:
        if not output.is_dir() or not (output / "frozen.json").is_file() or not (output / "checkpoint.sqlite3").is_file():
            raise ValueError("Full-grid resume requires its own existing frozen checkpoint")
    else:
        output.mkdir(parents=True, exist_ok=False)
    with io.FileLock(output / ".full-grid.lock"):
        if (output / "frozen.json").exists():
            if core.read_json(output / "frozen.json") != frozen:
                raise ValueError("Full-grid resume model/input/source/arguments changed")
        else:
            io.atomic_json(output / "frozen.json", frozen)
        io.require_space(output, reserve)
        (output / "shards").mkdir(exist_ok=True)
        connection = sqlite3.connect(output / "checkpoint.sqlite3")
        connection.execute("PRAGMA journal_mode=WAL")
        connection.execute("PRAGMA synchronous=FULL")
        connection.execute("CREATE TABLE IF NOT EXISTS shards(start INTEGER PRIMARY KEY, receipt TEXT NOT NULL)")
        connection.commit()
        committed, predictor = 0, None

        def progress(phase, **fields):
            io.atomic_json(output / "progress.json", {"schema": SCHEMA, "scope_sha256": scope, "phase": phase,
                "committed_rows": committed, "total_rows": grid.total, "updated_unix": time.time(), **fields})

        try:
            io.atomic_json(output / "plan.json", plan)
            io.atomic_json(output / "size_estimate.json", estimate)
            progress("VERIFYING_CHECKPOINT")
            for start, raw in connection.execute("SELECT start,receipt FROM shards ORDER BY start"):
                receipt = json.loads(raw)
                expected_count = min(args.shard_rows, grid.total - committed)
                if (start != committed or receipt["start"] != committed or receipt["count"] != expected_count
                        or receipt["binary"] != shard_name(committed // args.shard_rows)):
                    raise ValueError("Full-grid checkpoint has a gap or invalid shard range")
                binary = output / receipt["binary"]
                if binary.stat().st_size != receipt["bytes"] or core.sha256_file(binary) != receipt["sha256"]:
                    raise ValueError("Committed binary shard changed")
                for item in receipt["text_files"]:
                    path = binary.with_suffix("." + item["name"].rsplit(".", 1)[-1])
                    if path.name != item["name"] or _sign(path) != item:
                        raise ValueError("Committed readable shard changed")
                committed += expected_count
            complete = output / "completion.json"
            if complete.exists():
                result = core.read_json(complete)
                if committed != grid.total:
                    raise ValueError("Incomplete grid incorrectly marked complete")
                for item in result["files"]:
                    if _sign(output / item["name"]) != item:
                        raise ValueError("Completed full-grid output changed")
                progress("COMPLETE")
                return result["summary"]
            while committed < grid.total:
                count = min(args.shard_rows, grid.total - committed)
                binary = output / shard_name(committed // args.shard_rows)
                progress("PREDICTING", current_shard=binary.name, active_shard_rows=0)
                if not binary.exists():
                    io.require_space(output, reserve, PREFIX_BYTES + count * core.RECORD.size)
                    if predictor is None:
                        core.verify_sources(sources)
                        core.verify_model_tree(plan, model_files)
                        predictor = (predictor_loader or core.load_predictor)(plan["model"]["predictor_path"])
                    temporary = binary.with_name(binary.name + "." + uuid.uuid4().hex + ".partial")
                    digest = hashlib.sha256()
                    with temporary.open("xb+") as target:
                        blank_header(target)
                        for start in range(committed, committed + count, args.chunk_size):
                            io.require_space(output, reserve)
                            keys = list(grid.keys(start, min(start + args.chunk_size, committed + count)))
                            values = core.predict_keys(predictor, plan, keys, args.chunk_size)
                            data = b"".join(core.RECORD.pack(*key, values[key]) for key in keys)
                            io.require_space(output, reserve, len(data))
                            target.write(data)
                            digest.update(data)
                            progress("PREDICTING", current_shard=binary.name,
                                     active_shard_rows=start + len(keys) - committed)
                        target.seek(0)
                        target.write(header_bytes(plan, scope, committed, count, digest.hexdigest()))
                        target.flush()
                        os.fsync(target.fileno())
                    verified = verify_shard(temporary, plan, scope, grid, committed, count, args.chunk_size)
                    publish_new(temporary, binary)
                else:
                    # Crash after atomic rename but before SQLite commit: verify
                    # and adopt the complete shard without repeating inference.
                    verified = verify_shard(binary, plan, scope, grid, committed, count, args.chunk_size)
                text_files = [export_text(binary, plan, ext, args.chunk_size, reserve) for ext in ("csv", "txt")
                              if args.text_format in (ext, "both")]
                receipt = dict(verified, start=committed, count=count,
                               binary=binary.relative_to(output).as_posix(), text_files=text_files)
                with connection:
                    connection.execute("INSERT INTO shards VALUES (?,?)", (committed, core.encoded_json(receipt).decode("utf-8")))
                committed += count
                progress("SHARD_COMMITTED", current_shard=binary.name)
            progress("FINALIZING")
            index = output / "shards.jsonl"
            temporary = index.with_name(index.name + "." + uuid.uuid4().hex + ".partial")
            with temporary.open("xb") as stream:
                for (raw,) in connection.execute("SELECT receipt FROM shards ORDER BY start"):
                    io.require_space(output, reserve)
                    stream.write(raw.encode("utf-8") + b"\n")
                stream.flush()
                os.fsync(stream.fileno())
            publish_new(temporary, index)
            files = [_sign(index)]
            if args.merge_output:
                progress("MERGING")
                files.append(merge_shards(output, connection, plan, scope, grid, args.chunk_size, reserve))
            core.verify_sources(sources)
            core.verify_model_tree(plan, model_files)
            summary = {"schema": SCHEMA, "status": "COMPLETE", "scope_sha256": scope,
                "source_label": plan["source_label"], "coverage": "all-legal-keys-of-declared-quantized-grid",
                "entry_count": committed, "size_estimate": estimate, "shard_index": "shards.jsonl",
                "merged_table": "table.demand" if args.merge_output else None,
                "cpp_input": "table.demand" if args.merge_output else "shard index is not a C++ input; merge required",
                "simulation_run": False, "formal_mae_available": False, "input_semantics_equivalent": False}
            io.atomic_json(output / "summary.json", summary)
            files.append(_sign(output / "summary.json"))
            io.atomic_json(complete, {"schema": SCHEMA, "summary": summary, "files": files})
            progress("COMPLETE")
            return summary
        except BaseException as exc:
            progress("FAILED", error=f"{type(exc).__name__}: {exc}")
            raise
        finally:
            connection.close()


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--expected-threshold", choices=core.THRESHOLD_ORDER)
    parser.add_argument("--all-thresholds", action="store_true")
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--signal-remaining-step", type=float, required=True)
    parser.add_argument("--road-flow-step", type=int, default=1)
    parser.add_argument("--road-flow-dense-until", type=int, default=10)
    parser.add_argument("--road-flow-segments", nargs="+", metavar="UPPER:STEP")
    parser.add_argument("--cap-step", type=float, default=.1389)
    parser.add_argument("--cap-min", type=float)
    parser.add_argument("--cap-max", type=float)
    parser.add_argument("--cap-out-of-range", choices=("clamp", "error"), default="clamp")
    parser.add_argument("--first-road-position-correction", choices=("on", "off"), default="on")
    parser.add_argument("--max-road-flow", type=int, default=222)
    parser.add_argument("--max-lane-flow", type=int, default=80)
    parser.add_argument("--intersection-crossing-time", dest="crossing_time", type=float, default=3.)
    parser.add_argument("--allow-static-extrapolation", action="store_true")
    parser.add_argument("--chunk-size", type=int, default=50_000, help="Bound model prediction batch size")
    parser.add_argument("--shard-rows", type=int, default=1_000_000,
                        help="Rows per committed shard; only an interrupted unfinished shard may be recomputed")
    parser.add_argument("--min-free-disk-gib", type=float, default=50.)
    parser.add_argument("--text-format", choices=("none", "csv", "txt", "both"), default="none")
    parser.add_argument("--merge-output", action="store_true",
                        help="Also assemble table.demand for existing C++ reader; requires extra full-payload disk space")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.set_defaults(max_cache_entries=0)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        if args.all_thresholds:
            if args.expected_threshold:
                raise ValueError("all-thresholds conflicts with expected-threshold")
            configurations = []
            for threshold in core.THRESHOLD_ORDER:
                current = argparse.Namespace(**vars(args))
                name = core.model_directory_name(threshold)
                current.model_dir = str(Path(args.model_dir) / name)
                current.output_dir = str(Path(args.output_dir) / name)
                current.expected_threshold = threshold
                current.resume = args.resume and Path(current.output_dir).exists()
                configurations.append(current)
            # Preflight every model before writing any output.
            previews = [run(argparse.Namespace(**(vars(item) | {"dry_run": True}))) for item in configurations]
            result = previews if args.dry_run else [run(item) for item in configurations]
        else:
            result = run(args)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    except KeyboardInterrupt:
        print("Full catch interrupted; completed shards retained; resume with the same arguments", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError, ET.ParseError, sqlite3.Error) as exc:
        print(f"Full catch failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
