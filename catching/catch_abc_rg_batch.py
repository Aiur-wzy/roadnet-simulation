#!/usr/bin/env python3
"""Serial ABC/RG catch jobs; resume completed thresholds, never partial files.

Run under nohup using the same Python environment as the model. No training,
automatic coarsening, deletion, or background worker is started by this script.
"""
import argparse
from contextlib import contextmanager
from datetime import datetime
import importlib
import json
import os
from pathlib import Path
from roadnet_paths import ROOT
import shlex
import signal
import subprocess
import sys
import uuid


DEFAULT_THRESHOLDS = ("1", "3", "0p5", "1p5", "2")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--model-root", type=Path, required=True)
    result.add_argument("--net-file", type=Path, required=True)
    result.add_argument("--output-root", type=Path, required=True)
    result.add_argument("--thresholds", nargs="+", choices=("1", "3", "0p1", "0p5", "1p5", "2"),
                        default=list(DEFAULT_THRESHOLDS))
    result.add_argument("--signal-remaining-step", type=float, default=5.)
    result.add_argument("--road-flow-step", type=int, default=1,
                        help="Road-flow node spacing in vehicles above the dense range (default: 1)")
    result.add_argument("--road-flow-dense-until", type=int, default=10,
                        help="Keep every integer road-flow node through this count, inclusive (default: 10)")
    result.add_argument("--road-flow-segments", nargs="+", metavar="UPPER:STEP",
                        help="Optional piecewise road-flow grid, e.g. 10:1 60:5 100:20 222:50; "
                             "cannot combine with nondefault --road-flow-step/--road-flow-dense-until")
    result.add_argument("--max-rows", type=int, default=1_500_000_000)
    result.add_argument("--chunk-size", type=int, default=50_000)
    result.add_argument("--cap-step", type=float, default=.1389)
    result.add_argument("--max-road-flow", type=int, default=222)
    result.add_argument("--max-lane-flow", type=int, default=80)
    result.add_argument("--resume", action="store_true",
                        help="Verify and skip complete matching tables; incomplete output is an error")
    result.add_argument("--dry-run", action="store_true",
                        help="Validate ALL jobs without loading predictors, creating logs, or exporting")
    return result


def job_arguments(args, token):
    result = ["--model-dir", str(args.model_root / ("gt_" + token)),
            "--net-file", str(args.net_file),
            "--output-dir", str(args.output_root / ("gt_" + token)),
            "--expected-threshold", token,
            "--signal-remaining-step", str(args.signal_remaining_step),
            "--road-flow-step", str(args.road_flow_step),
            "--road-flow-dense-until", str(args.road_flow_dense_until),
            "--max-rows", str(args.max_rows), "--chunk-size", str(args.chunk_size),
            "--cap-step", str(args.cap_step), "--max-road-flow", str(args.max_road_flow),
            "--max-lane-flow", str(args.max_lane_flow),
            "--text-format", "none"]
    if args.road_flow_segments is not None:
        result.extend(["--road-flow-segments", *args.road_flow_segments])
    return result


def comparable_plan(plan):
    # Absolute locations, warnings, and a user-selected row budget are not
    # prediction identities. Contracts, grids, and content hashes still match.
    result = json.loads(json.dumps(plan))
    result.pop("warnings", None)
    result["network"].pop("path", None)
    for key in ("predictor_path", "contract_path"):
        result["model"].pop(key, None)
    result["resource_limits"].pop("requested_max_rows", None)
    return result


def verify_completed(catch, output, expected_plan):
    manifest_path = output / "manifest.json"
    if not manifest_path.is_file():
        raise ValueError(f"Incomplete output: {output}. Preserve/rename this directory before rerunning; "
                         "partial catch cannot resume within a threshold.")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"),
                          object_pairs_hook=catch.abc._unique_json,
                          parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    if (manifest.get("status") != "complete" or manifest.get("table") != "table.rle"
            or manifest.get("roundtrip_exact") is not True):
        raise ValueError(f"Incomplete/incompatible manifest: {manifest_path}")
    table = output / "table.rle"
    if (not table.is_file() or table.stat().st_size != manifest.get("file_bytes")
            or catch.abc.sha256_file(table) != manifest.get("table_sha256")):
        raise ValueError(f"Table size/SHA mismatch: {table}")
    # Stream the compressed bytes, never expand the hundreds of millions of rows.
    stream, header, _ = catch._open_table(table)
    stream.close()
    if manifest.get("plan") != header:
        raise ValueError(f"Manifest and table header disagree: {output}")
    if manifest.get("rows") != header["expanded_rows"] or manifest.get("run_count") != header["run_count"]:
        raise ValueError(f"Manifest row/run count mismatch: {output}")
    missing = sorted(set(expected_plan) - set(header))
    if missing:
        raise ValueError(f"Existing table lacks requested exporter metadata {missing}: {output}")
    if "prediction_rows" in header and manifest.get("prediction_rows") != header["prediction_rows"]:
        raise ValueError(f"Manifest prediction-row count mismatch: {output}")
    actual_plan = {key: header[key] for key in expected_plan}
    if comparable_plan(actual_plan) != comparable_plan(expected_plan):
        raise ValueError(f"Existing table differs from requested model/network/grid/exporter: {output}")


def preflight(args, catch):
    if len(set(args.thresholds)) != len(args.thresholds):
        raise ValueError("Duplicate thresholds are not allowed")
    jobs = []
    for token in args.thresholds:
        argv = job_arguments(args, token)
        options = catch.build_parser().parse_args(argv)
        output = Path(options.output_dir)
        # Core build_plan intentionally rejects existing output. A nonexistent
        # plan-only name lets us compare an existing table without creating it.
        if output.exists():
            options.output_dir = str(output.parent / (".plan_only_" + uuid.uuid4().hex))
        plan = catch.build_plan(options)
        catch.enforce_budget(plan)
        skip = output.exists()
        if skip:
            if not args.resume:
                raise ValueError(f"Output exists: {output}; use --resume to verify completed tables")
            verify_completed(catch, output, plan)
        policy = plan["road_flow_quantization"]
        road_grid = (f"road_flow_grid=explicit road_flow_nodes={len(policy['nodes'])} "
                     if "nodes" in policy else
                     f"road_flow_step={policy['step']}veh road_flow_dense_until={policy['dense_until']}veh ")
        print(f"[{'SKIP_VERIFIED' if skip else 'PLAN'}] gt_{token} "
              f"signal_step={plan['signal_remaining_step']:g}s signal_points={plan['signal_point_count']} "
              f"{road_grid}"
              f"prediction_rows={plan['prediction_rows']} expanded_rows={plan['expanded_rows']} "
              f"output={output}", flush=True)
        jobs.append((token, argv, output, plan, skip))
    return jobs


@contextmanager
def batch_lock(output_root):
    output_root.mkdir(parents=True, exist_ok=True)
    lock = output_root / ".catch_batch.lock"
    try:
        handle = lock.open("x", encoding="utf-8")
    except FileExistsError:
        raise ValueError(f"Batch lock exists: {lock}. Check whether the old job is still running; "
                         "only after confirming it has stopped may you remove a stale lock.") from None
    with handle:
        handle.write(json.dumps({"pid": os.getpid(), "started": datetime.now().isoformat()}))
    try:
        yield
    finally:
        lock.unlink()  # Only the exact lock created by this invocation.


def run_child(command, stream):
    process = None
    previous = signal.getsignal(signal.SIGTERM)

    def terminate(_signum, _frame):
        raise KeyboardInterrupt("batch terminated")

    signal.signal(signal.SIGTERM, terminate)
    try:
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(ROOT) + os.pathsep + environment.get("PYTHONPATH", "")
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, env=environment)
        return process.wait()
    except BaseException:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        raise
    finally:
        signal.signal(signal.SIGTERM, previous)


def run(args, catch):
    for name in ("model_root", "net_file", "output_root"):
        setattr(args, name, getattr(args, name).expanduser().resolve())
    if args.dry_run:
        preflight(args, catch)
        print("[DRY_RUN] All selected contracts/grids checked; no predictors loaded or outputs written", flush=True)
        return
    with batch_lock(args.output_root):
        jobs = preflight(args, catch)  # Fail ALL preflight issues before starting an expensive export.
        logs = args.output_root / "logs"
        logs.mkdir(exist_ok=True)
        for token, argv, output, plan, skip in jobs:
            if skip:
                continue
            log = logs / ("gt_" + token + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".log")
            command = [sys.executable, "-u", "-m", "catching.model_catching_sumo_cap_context_abc_rg", *argv]
            print(f"[START] gt_{token} log={log}\n[COMMAND] {shlex.join(command)}", flush=True)
            with log.open("x", encoding="utf-8") as stream:
                code = run_child(command, stream)
            if code != 0:
                raise RuntimeError(f"gt_{token} exited {code}; later thresholds NOT started. Log: {log}")
            verify_completed(catch, output, plan)
            print(f"[DONE] gt_{token} table={output / 'table.rle'}", flush=True)
        print("[COMPLETE] All selected thresholds exported or verified as complete", flush=True)


def main(argv=None):
    args = parser().parse_args(argv)
    catch = importlib.import_module('catching.model_catching_sumo_cap_context_abc_rg')
    try:
        run(args, catch)
    except KeyboardInterrupt:
        print("[INTERRUPTED] Current partial output retained; later thresholds NOT started", file=sys.stderr, flush=True)
        return 130
    except (ValueError, RuntimeError, OSError, KeyError) as exc:
        print(f"[FAILED] {exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
