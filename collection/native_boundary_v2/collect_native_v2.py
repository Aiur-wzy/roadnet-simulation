"""Streaming, quality-gated SUMO-native collection; original collectors untouched.

This entry only observes SUMO. Accepted labels use the verified no-internal
Euler boundary estimator and FIRST VISIBLE physical features, not a fabricated
exact entry state. Per-road eligibility is provisional until the entire closed
run passes identity, provenance, file and native-output checks.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import gzip
import math
import os
from pathlib import Path
import shutil
import time
import traceback
import xml.etree.ElementTree as ET

from collection.TraCI_Python_SUMO_Native import (
    FIELDS as OLD_FIELDS, atomic_json, load_traci, read_inputs, route_digest, sha256,
    tripinfo_audit,
)
from collection.sumo_native_state import FEATURES, LABELS, Observation
from collection.native_boundary_v2.sumo_native_path_v2 import NoInternalGeometry
from collection.native_boundary_v2.sumo_native_signals_v2 import (
    NativeStateV2, NativeSignalReaderV2, sampling_contract_v2, snapshot_clock,
)
from collection.native_boundary_v2.streaming_quality_v2 import (
    StreamingQualityV2, EXTRA_FIELDS, SCHEMA,
)


CLOCK_FIELDS = ("snapshot_clock_schema", "snapshot_clock_step_seconds", "signal_clock_schema",
                "signal_api_return_time", "signal_snapshot_time", "signal_step_seconds")
FIELDS = tuple(dict.fromkeys((*OLD_FIELDS, *CLOCK_FIELDS, *EXTRA_FIELDS)))


def contract():
    result = sampling_contract_v2()
    result.update(schema=SCHEMA,
        feature_anchor="first genuine visible SUMO snapshot; record lag to estimated entry; no synthetic features",
        labels="verified no-internal Euler geometry allocation; six strict speed > thresholds; unknown remains blank",
        boundary="road occurrence identity, odometer, geometry and elapsed budget checked; unknown contaminates all affected visits",
        quality="per-road candidate only; entire run released only after successful close, native identity and provenance audits",
        zero="valid zero and subsecond values retained; no target-magnitude exclusion",
        memory="active occurrences in memory; arrived terminal suffixes on disk until closed tripinfo reconciliation",
        limitations="existing Euler domain only; paused integrator/ABC/subsecond-consumer extensions not implemented")
    return result


def validate_snapshot_frame(expected_live, live_ids, snapshots, variables):
    """Missing observations could corrupt OTHER vehicles' features: fail the run."""
    if len(live_ids) != len(set(live_ids)):
        raise ValueError("duplicate native live vehicle ID")
    wanted = set(expected_live)
    if set(live_ids) != wanted or set(snapshots) != wanted:
        raise ValueError("incomplete physical snapshot: tracked/live/subscription ID sets differ")
    for vid, values in snapshots.items():
        if not isinstance(values, dict) or any(key not in values for key in variables):
            raise ValueError("incomplete vehicle subscription fields: " + vid)


def validate_snapshot_clock(clock, step_seconds, previous_snapshot=None):
    """A skipped whole frame invalidates simultaneous physical feature history."""
    if not math.isclose(clock["step_seconds"], step_seconds, rel_tol=0, abs_tol=1e-12):
        raise ValueError("SUMO step length changed")
    snapshot = clock["snapshot_time"]
    if previous_snapshot is not None and not math.isclose(
            snapshot - previous_snapshot, step_seconds, rel_tol=0, abs_tol=1e-8):
        raise ValueError("missing or nonconsecutive native snapshot step")


def validate_row(row):
    if row.get("schema") != SCHEMA:
        raise ValueError("wrong row schema")
    if row.get("trainable") not in (0, 1):
        raise ValueError("invalid row eligibility")
    if not row.get("trainable"):
        return
    if row.get("feature_valid") not in (1, "1") or row.get("label_valid") not in (1, "1"):
        raise ValueError("eligible row lacks complete feature/label")
    for name in FEATURES:
        value = row.get(name)
        if name in {"entry_signal_type", "next_signal_type"}:
            if str(value) not in {"R", "G"}:
                raise ValueError("invalid signal category")
        elif value is None or not math.isfinite(float(value)):
            raise ValueError("invalid formal feature: " + name)
    labels = [float(row[x]) for x in LABELS]
    if any(not math.isfinite(x) or x < 0 for x in labels):
        raise ValueError("invalid eligible label")
    if any(a + 1e-8 < b for a, b in zip(labels, labels[1:])):
        raise ValueError("threshold label monotonicity failed")


def audit_closed_csv(path, expected_count, training=False):
    count = 0
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or ()) != FIELDS:
            raise ValueError("closed CSV schema mismatch")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError("incomplete CSV row")
            row["trainable"] = int(row["trainable"])
            validate_row(row)
            if training and not row["trainable"]:
                raise ValueError("excluded row leaked into training output")
            count += 1
    if count != expected_count:
        raise ValueError("closed CSV count mismatch")
    return count


def audit_native_xml(path, tag):
    # Release only files parsed to EOF after SUMO's process has exited.
    opener = gzip.open if str(path).endswith(".gz") else open
    count = 0
    with opener(path, "rb") as stream:
        root = None
        for event, node in ET.iterparse(stream, events=("start", "end")):
            if root is None:
                root = node
            if event == "end" and node.tag == tag:
                count += 1
                node.clear()
                root.clear()
    return count


def safe_inputs(config, network):
    expected, route_files = read_inputs(config, network)
    options = {n.tag: n.get("value") for n in ET.parse(config).getroot().iter() if "value" in n.attrib}
    for key in ("mesosim", "step-method.ballistic"):
        if options.get(key, "false").lower() not in {"false", "0"}:
            raise ValueError("unsupported simulation/integration mode; inputs unchanged")
    # Retain the already-validated explicit domain checks. Do not expand the
    # integrator inspection paused by the user (T05-INT).
    if any("output" in key or key in {"log", "error-log", "save-state.times", "save-state.period"}
           for key in options):
        raise ValueError("input configuration must not name old output destinations")
    return expected, route_files


def run(args):
    started = time.time()
    config, network, out = map(lambda x: Path(x).resolve(), (args.sumo_config, args.net_file, args.output_dir))
    if out.exists():
        raise ValueError("fresh output directory required; no implicit resume or retry")
    expected, route_files = safe_inputs(config, network)
    graph = NoInternalGeometry(network)
    lanes = {}
    for _, node in ET.iterparse(network, events=("end",)):
        if node.tag == "edge":
            for lane in node.findall("lane"):
                lanes[lane.get("id")] = (float(lane.get("length")), float(lane.get("speed")))
            node.clear()
    from collection.run_manhattan_training_batch_v2 import collector_sources
    source_root = Path(__file__).resolve().parents[2]
    out.mkdir(parents=True, exist_ok=False)
    command = [args.sumo_binary, "-c", str(config),
        "--tripinfo-output", str(out / "tripinfo.xml"), "--tripinfo-output.write-unfinished", "true",
        "--vehroute-output", str(out / "vehroute.xml.gz"), "--vehroute-output.exit-times", "true",
        "--vehroute-output.write-unfinished", "true", "--vehroute-output.speedfactor", "true",
        "--statistic-output", str(out / "statistics.xml"), "--summary-output", str(out / "summary.xml.gz"),
        "--lanechange-output", str(out / "lanechanges.xml.gz"), "--precision", "12",
        "--log", str(out / "sumo.log"), "--error-log", str(out / "sumo.stderr.log"),
        "--no-step-log", "true", "--duration-log.statistics", "true"]
    manifest = dict(schema="sumo-native-collection-run-v2", contract=contract(),
        inputs={str(p): sha256(p) for p in (config, network, *route_files)},
        sources=collector_sources(source_root), command=command, expected_vehicles=len(expected),
        expected_road_occurrences=sum(x["roads"] for x in expected.values()), started_unix=started,
        no_hard_end=True, behavior_setters_used=False, stall_seconds=args.stall_seconds)
    atomic_json(out / "manifest.json", manifest)
    all_stream = train_stream = stdout = api = engine = state = None
    native_closed = False
    result, error, now = "FAILED", None, 0.0
    counts, modes = Counter(), Counter()
    routes, metadata, sf_values, progress = {}, {}, {}, {}
    departed_seen, arrived_seen = set(), set()
    feature_rows = []
    quality_audit = {}
    try:
        all_stream = gzip.open(out / "road_visits.partial.csv.gz", "wt", encoding="utf-8", newline="", compresslevel=1)
        train_stream = gzip.open(out / "training.partial.csv.gz", "wt", encoding="utf-8", newline="", compresslevel=1)
        all_writer, train_writer = [csv.DictWriter(s, FIELDS, extrasaction="ignore") for s in (all_stream, train_stream)]
        all_writer.writeheader()
        train_writer.writeheader()
        def emit(row):
            validate_row(row)
            all_writer.writerow(row)
            if row["trainable"]:
                train_writer.writerow(row)
        traci = load_traci()
        from traci import constants as tc
        label = "native_quality_v2_" + str(os.getpid())
        stdout = (out / "sumo.stdout.log").open("w", encoding="utf-8")
        traci.start(command, label=label, stdout=stdout)
        api = traci.getConnection(label)
        version, dt = api.getVersion(), float(api.simulation.getDeltaT())
        if not any(v in version[1] for v in ("1.27.0", "1.27.1")):
            raise ValueError("uncalibrated SUMO version")
        manifest.update(sumo_version=version, step_seconds=dt)
        atomic_json(out / "manifest.json", manifest)
        engine = StreamingQualityV2(graph, dt, out / "terminal_pending.sqlite", emit)
        state = NativeStateV2(routes, metadata, NativeSignalReaderV2(api, network), feature_rows.append)
        variables = (tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_ROUTE_INDEX, tc.VAR_LANEPOSITION,
                     tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_DISTANCE)
        next_status = 0.0
        previous_snapshot = None
        while api.simulation.getMinExpectedNumber() > 0:
            api.simulationStep()
            clock = snapshot_clock(api)
            now, state_time = clock["api_return_time"], clock["snapshot_time"]
            validate_snapshot_clock(clock, dt, previous_snapshot)
            previous_snapshot = state_time
            departed, arrived = list(api.simulation.getDepartedIDList()), list(api.simulation.getArrivedIDList())
            if len(departed) != len(set(departed)) or len(arrived) != len(set(arrived)):
                raise ValueError("duplicate departure/arrival event")
            if set(departed) & set(arrived):
                raise ValueError("unobservable same-step departure/arrival; retain but do not certify run")
            for vid in departed:
                if vid not in expected or vid in departed_seen:
                    raise ValueError("unexpected/duplicate departure identity")
                route = tuple(api.vehicle.getRoute(vid))
                sf = float(api.vehicle.getSpeedFactor(vid))
                if route_digest(route) != expected[vid]["route_sha256"] or not math.isclose(sf, expected[vid]["sf"], rel_tol=1e-12, abs_tol=1e-12):
                    raise ValueError("route or explicit SF runtime mismatch")
                routes[vid] = route
                metadata[vid] = dict(speed_factor=sf, actual_depart=float(api.vehicle.getDeparture(vid)),
                    planned_depart=expected[vid]["depart"], vehicle_type=api.vehicle.getTypeID(vid),
                    vehicle_max_speed=float(api.vehicle.getMaxSpeed(vid)))
                modes[int(api.vehicle.getLaneChangeMode(vid))] += 1
                sf_values[vid] = sf
                departed_seen.add(vid)
                engine.depart(vid, route, metadata[vid])
                api.vehicle.subscribe(vid, variables)
            if set(arrived) - departed_seen or set(arrived) & arrived_seen:
                raise ValueError("unexpected/duplicate arrival identity")
            counts["teleports"] += len(api.simulation.getStartingTeleportIDList())
            counts["collisions"] += int(api.simulation.getCollidingVehiclesNumber())
            if counts["teleports"] or counts["collisions"]:
                raise ValueError("teleport or collision detected; preserve evidence, do not publish")
            snapshots = api.vehicle.getAllSubscriptionResults()
            wanted = set(routes) - set(arrived)
            validate_snapshot_frame(wanted, list(api.vehicle.getIDList()), snapshots, variables)
            observations, raw, stalled = [], [], []
            for vid, values in snapshots.items():
                edge, lane = values[tc.VAR_ROAD_ID], values[tc.VAR_LANE_ID]
                index = int(values[tc.VAR_ROUTE_INDEX])
                if not edge or edge.startswith(":") or lane not in lanes or not 0 <= index < len(routes[vid]) or routes[vid][index] != edge:
                    raise ValueError("snapshot lane/route/external-road identity failed")
                length, limit = lanes[lane]
                value = dict(id=vid, edge=edge, lane=lane, route_index=index,
                    position=float(values[tc.VAR_LANEPOSITION]), speed=float(values[tc.VAR_SPEED]),
                    cap=float(values[tc.VAR_ALLOWED_SPEED]), distance=float(values[tc.VAR_DISTANCE]), time=state_time)
                if (graph.lane_roads.get(lane) != edge or
                    any(not math.isfinite(value[k]) for k in ("position", "speed", "cap", "distance")) or
                    min(value["position"], value["speed"], value["distance"]) < 0 or value["position"] > length or value["cap"] <= 0):
                    raise ValueError("invalid physical snapshot; no dependent feature may be trusted")
                raw.append(value)
                observations.append(Observation(vid, edge, index, lane, value["position"], value["speed"], value["cap"], length, limit))
                old = progress.get(vid)
                if old is None or old[0] != index or abs(value["position"] - old[1]) >= 1.0:
                    progress[vid] = (index, value["position"], state_time)
                elif args.stall_seconds > 0 and state_time - old[2] >= args.stall_seconds:
                    stalled.append(dict(vehicle_id=vid, road=edge, lane=lane, no_progress_seconds=state_time-old[2]))
            feature_rows.clear()
            state.step_from_api(api, observations, arrived)
            engine.step(state_time, raw, arrived=arrived, feature_rows=feature_rows, expected_live_ids=wanted)
            state.discard_arrived(arrived)
            arrived_seen.update(arrived)
            for vid in arrived:
                progress.pop(vid, None)
            counts["departed"] = len(departed_seen)
            counts["arrived"] = len(arrived_seen)
            if now >= next_status:
                all_stream.flush()
                train_stream.flush()
                atomic_json(out / "status.json", dict(status="RUNNING", simulation_time=now,
                    wall_seconds=time.time()-started, counts=dict(counts), running=len(wanted), training_released=False))
                next_status = now + args.status_period
            if stalled or (out / "STOP").exists():
                api.simulation.saveState(str(out / "stopped_state.xml.gz"))
                atomic_json(out / "stop_evidence.json", dict(simulation_time=now, stalled=stalled,
                    reason="sampled_no_progress_not_proven_deadlock" if stalled else "STOP_requested"))
                result = "CENSORED"
                break
        else:
            result = "COMPLETE"
    except Exception as exc:
        result, error = "FAILED", f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        if api is not None:
            try:
                api.simulation.saveState(str(out / "failed_state.xml.gz"))
            except Exception:
                pass
    finally:
        if api is not None:
            try:
                api.close()
                native_closed = True
            except Exception as exc:
                result, error = "FAILED", f"native close failed: {type(exc).__name__}: {exc}"
        if stdout is not None:
            try:
                stdout.close()
            except Exception as exc:
                result, error = "FAILED", f"stdout close failed: {type(exc).__name__}: {exc}"
    # Closed native outputs only: pending terminal rows must never use live XML.
    try:
        if result == "COMPLETE" and error is None and native_closed:
            if arrived_seen != set(expected) or departed_seen != set(expected):
                raise ValueError("incomplete final vehicle identity sets")
            counts.update(tripinfo_audit(out / "tripinfo.xml", expected, sf_values))
            if counts["normal_arrived"] != len(expected) or counts["missing_ids"] or counts["missing_runtime_sf"]:
                raise ValueError("native terminal audit failed")
            engine.finalize_tripinfo(out / "tripinfo.xml")
            quality_audit = engine.finish()
            counts.update(quality_audit["counts"])
            if counts["road_occurrences"] != manifest["expected_road_occurrences"]:
                raise ValueError("road occurrence ledger does not match supplied routes")
            if counts["trainable"] + counts["excluded"] != counts["road_occurrences"]:
                raise ValueError("eligibility ledger does not close")
    except Exception as exc:
        result, error = "FAILED_AUDIT", f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    finally:
        for obj in (engine, all_stream, train_stream):
            if obj is not None:
                try:
                    obj.close()
                except Exception as exc:
                    result, error = "FAILED", f"output close failed: {type(exc).__name__}: {exc}"
    counts["expected_vehicles"] = len(expected)
    released, closed_hashes = False, {}
    try:
        if result == "COMPLETE" and error is None and native_closed:
            for name, digest in manifest["inputs"].items():
                if sha256(name) != digest:
                    raise ValueError("input changed during collection")
            if collector_sources(source_root) != manifest["sources"]:
                raise ValueError("source changed during collection")
            audit_closed_csv(out / "road_visits.partial.csv.gz", counts["road_occurrences"])
            audit_closed_csv(out / "training.partial.csv.gz", counts["trainable"], training=True)
            if audit_native_xml(out / "vehroute.xml.gz", "vehicle") != len(expected):
                raise ValueError("closed vehroute count mismatch")
            audit_native_xml(out / "lanechanges.xml.gz", "change")
            audit_native_xml(out / "statistics.xml", "statistics")
            audit_native_xml(out / "summary.xml.gz", "step")
            for name in ("road_visits.csv.gz", "training.csv.gz", "tripinfo.xml", "vehroute.xml.gz",
                         "lanechanges.xml.gz", "summary.xml.gz", "statistics.xml", "terminal_pending.sqlite"):
                candidate_name = name.replace(".csv.gz", ".partial.csv.gz") if name.endswith(".csv.gz") else name
                closed_hashes[name] = sha256(out / candidate_name)
            # Keep training under its provisional name until every closed
            # artifact has passed both parsing and hashing. Audit failures
            # must not leave a final-named training file behind.
            (out / "road_visits.partial.csv.gz").replace(out / "road_visits.csv.gz")
            (out / "training.partial.csv.gz").replace(out / "training.csv.gz")
            released = True
    except Exception as exc:
        result, error = "FAILED_AUDIT", f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
    audit = dict(status=result, error=error, counts=dict(counts), quality=quality_audit,
        simulation_time=now, wall_seconds=time.time()-started, native_outputs_closed=native_closed,
        training_released=released, explicit_speed_factor_audited=len(sf_values)==len(expected),
        lane_change_modes=dict(modes), manifest_sha256=sha256(out / "manifest.json"),
        closed_outputs_sha256=closed_hashes, contract=contract())
    atomic_json(out / "audit.json", audit)
    atomic_json(out / "status.json", audit)
    print(__import__("json").dumps(audit, ensure_ascii=False, allow_nan=False))
    return 0 if result == "COMPLETE" and released else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo-config", required=True)
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--sumo-binary", default=shutil.which("sumo") or "sumo")
    parser.add_argument("--stall-seconds", type=float, default=3600.0)
    parser.add_argument("--status-period", type=int, default=60)
    args = parser.parse_args(argv)
    if not math.isfinite(args.stall_seconds) or args.stall_seconds < 0 or args.status_period < 1:
        parser.error("invalid stall/status interval")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
