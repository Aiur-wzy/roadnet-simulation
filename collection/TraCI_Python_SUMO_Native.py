"""Streaming SUMO-grounded nine-feature / six-label collector.

Run as a module from the repository root. Does not set vehicle lanes, speed,
route, lane-change mode or traffic lights. Config/route/seed are input truth.
Only observations and native output destinations are added to the run.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import sys
import time
import traceback
import xml.etree.ElementTree as ET

from collection.sumo_native_state import (FEATURES, LABELS, Observation, NativeState, sampling_contract)
from collection.sumo_native_signals import NativeSignalReader

FIELDS = ("schema", "vehicle_id", "route_index", "road_id", "is_first_road", "is_final_road",
          *FEATURES, "travel_time_label", *LABELS, "quality", "trainable", "label_valid",
          "feature_valid", "feature_error", "actual_depart", "planned_depart", "speed_factor",
          "vehicle_type", "vehicle_max_speed", "speed_limit", "entry_lane_id", "entry_position",
          "entry_speed", "observed_remaining_distance", "nominal_free_finish_time",
          "observed_enter_time", "observed_last_time", "observed_exit_time", "sampled_seconds",
          "sample_intervals", "lane_changes_within_road", "exit_boundary_unallocated_seconds",
          "P", "E", "legacy_platoon_context", "context0_subclass", "candidate_count",
          "max_deficit_seconds", "sum_deficit_seconds", "horizon_seconds",
          "max_overdue_seconds", "sum_overdue_seconds", "overdue_count",
          "signal_status", "signal_tls_id", "signal_link_indices", "signal_program_id",
          "signal_anchor_time", "signal_anchor_phase", "signal_anchor_next_switch_time",
          "signal_reference_time", "signal_projection_seconds", "signal_state_at_reference",
          "signal_yellow_skipped", "signal_selection_method", "signal_actual_lane_id",
          "signal_pair_connection_count", "signal_selected_link_count", "signal_selection_fallback_reason")


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8") as f:
        json.dump(value, f, indent=2, ensure_ascii=False, allow_nan=False)
        f.write("\n")
    for attempt in range(12):
        try:
            temp.replace(path)
            return
        except PermissionError:
            if attempt == 11:
                raise
            time.sleep(0.05 * (attempt + 1))


def route_digest(route):
    return hashlib.sha256(" ".join(route).encode("utf-8")).hexdigest()


def read_inputs(config, network):
    cfg = ET.parse(config).getroot()
    configured = cfg.find("./input/net-file")
    if configured is None or (config.parent / configured.get("value")).resolve() != network.resolve():
        raise ValueError("--net-file must match the configuration network")
    if cfg.find("./time/end") is not None:
        raise ValueError("full collection must drain without a hard simulation end")
    extra = cfg.find("./input/additional-files")
    if extra is not None and extra.get("value", "").strip():
        raise ValueError("native-v1 requires an explicit route/net config without additional behavior files")
    paths = [(config.parent / p.strip()).resolve() for p in cfg.find("./input/route-files").get("value").split(",")]
    expected = {}
    for path in paths:
        root = None
        for event, node in ET.iterparse(path, events=("start", "end")):
            if root is None:
                root = node
            if event != "end":
                continue
            if node.tag in {"flow", "trip", "routeDistribution", "vTypeDistribution"}:
                raise ValueError("native-v1 requires expanded vehicles with explicit routes and SF")
            if node.tag == "vehicle":
                vehicle = node.get("id")
                route = node.find("route")
                if route is None or not route.get("edges") or vehicle in expected:
                    raise ValueError("missing route or duplicate vehicle")
                sf, depart = float(node.attrib["speedFactor"]), float(node.attrib["depart"])
                if not math.isfinite(sf) or sf <= 0 or not math.isfinite(depart) or depart < 0:
                    raise ValueError("invalid SF/depart")
                edges = tuple(route.get("edges").split())
                expected[vehicle] = {"sf": sf, "depart": depart, "route_sha256": route_digest(edges), "roads": len(edges)}
                root.remove(node)
            elif node.tag == "vType":
                if any(x.startswith("lc") or x in {"laneChangeMode", "carFollowModel"} for x in node.attrib):
                    raise ValueError("training batch requires full original lane-change/default dynamics")
                root.remove(node)
    if not expected:
        raise ValueError("empty route demand")
    options = {node.tag: node.get("value") for node in cfg.iter() if "value" in node.attrib}
    for key in ("time-to-teleport", "time-to-teleport.highways", "time-to-teleport.disconnected",
                "time-to-teleport.ride", "time-to-teleport.bidi", "time-to-teleport.railsignal-deadlock"):
        if float(options.get(key, "0")) >= 0:
            raise ValueError(f"must explicitly disable {key} in config")
    if options.get("collision.action") != "warn" or options.get("intermodal-collision.action") != "warn":
        raise ValueError("collision actions must warn, not teleport/remove")
    return expected, paths


def tripinfo_audit(path, expected, actual_sf):
    seen, normal, total = set(), 0, 0.0
    missing_sf = set(expected) - set(actual_sf)
    for _, node in ET.iterparse(path, events=("end",)):
        if node.tag != "tripinfo":
            continue
        vehicle = node.get("id")
        if vehicle not in expected or vehicle in seen:
            raise ValueError("unexpected/duplicate tripinfo ID")
        seen.add(vehicle)
        depart, arrival, duration = (float(node.get(k)) for k in ("depart", "arrival", "duration"))
        if arrival >= 0 and depart >= 0 and node.get("vaporized", "") in {"", "false"}:
            if abs((arrival - depart) - duration) > .011:
                raise ValueError("tripinfo duration boundary mismatch")
            normal += 1
            total += duration
        node.clear()
    return dict(tripinfo_count=len(seen), normal_arrived=normal, missing_ids=len(set(expected) - seen),
                runtime_sf_count=len(actual_sf), missing_runtime_sf=len(missing_sf),
                mean_duration_seconds=total / normal if normal else None)


def load_traci():
    try:
        import traci
    except ImportError:
        tools = Path(os.environ.get("SUMO_HOME", "")) / "tools"
        if not tools.is_dir():
            raise RuntimeError("install matching traci or set SUMO_HOME")
        sys.path.insert(0, str(tools))
        import traci
    return traci


def run(args):
    started = time.time()
    config, network, output = Path(args.sumo_config).resolve(), Path(args.net_file).resolve(), Path(args.output_dir).resolve()
    if output.exists():
        raise ValueError("output directory must not already exist; no implicit resume/overwrite")
    expected, route_files = read_inputs(config, network)
    output.mkdir(parents=True)
    traci = load_traci()
    from traci import constants as tc
    lanes = {}
    for _, element in ET.iterparse(network, events=("end",)):
        if element.tag == "edge":
            if not element.get("id", "").startswith(":"):
                for lane in element.findall("lane"):
                    lanes[lane.get("id")] = float(lane.get("length")), float(lane.get("speed"))
            element.clear()
    command = [args.sumo_binary, "-c", str(config), "--tripinfo-output", str(output / "tripinfo.xml"),
               "--tripinfo-output.write-unfinished", "true", "--vehroute-output", str(output / "vehroute.xml.gz"),
               "--vehroute-output.exit-times", "true", "--vehroute-output.write-unfinished", "true",
               "--vehroute-output.speedfactor", "true", "--statistic-output", str(output / "statistics.xml"),
               "--summary-output", str(output / "summary.xml.gz"), "--lanechange-output", str(output / "lanechanges.xml.gz"),
               "--log", str(output / "sumo.log"), "--error-log", str(output / "sumo.stderr.log"),
               "--no-step-log", "true", "--duration-log.statistics", "true"]
    sources = {str(path): sha256(path) for folder in (Path(__file__).parent, Path(__file__).parents[1] / "features")
               for path in folder.glob("*.py")}
    manifest = dict(schema="sumo-native-collection-run-v1", inputs={str(p): sha256(p) for p in (config, network, *route_files)},
                    source_sha256=sources, command=command, contract=sampling_contract(), expected_vehicles=len(expected),
                    expected_road_occurrences=sum(v["roads"] for v in expected.values()), started_unix=started,
                    stall_seconds=args.stall_seconds, no_hard_end=True, behavior_setters_used=False)
    atomic_json(output / "manifest.json", manifest)
    all_stream = gzip.open(output / "road_visits.partial.csv.gz", "wt", encoding="utf-8", newline="", compresslevel=1)
    train_stream = gzip.open(output / "training.partial.csv.gz", "wt", encoding="utf-8", newline="", compresslevel=1)
    all_writer, train_writer = [csv.DictWriter(f, FIELDS, extrasaction="ignore") for f in (all_stream, train_stream)]
    all_writer.writeheader()
    train_writer.writeheader()
    def emit(row):
        all_writer.writerow(row)
        if row.get("trainable"):
            train_writer.writerow(row)
    routes, metadata, sf_values, departed_seen, arrived_seen = {}, {}, {}, set(), set()
    counts, now, state, api, result, error = Counter(), 0.0, None, None, "FAILED", None
    native_closed = False
    sumo_stdout = None
    observed_progress = {}
    lanechange_modes = Counter()
    try:
        label = f"native_{os.getpid()}"
        sumo_stdout = (output / "sumo.stdout.log").open("w", encoding="utf-8")
        traci.start(command, label=label, stdout=sumo_stdout)
        api = traci.getConnection(label)
        manifest["sumo_version"] = api.getVersion()
        signals = NativeSignalReader(api, network)
        state = NativeState(routes, metadata, signals, emit)
        variables = (tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_ROUTE_INDEX, tc.VAR_LANEPOSITION,
                     tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED)
        while api.simulation.getMinExpectedNumber() > 0:
            api.simulationStep()
            now = float(api.simulation.getTime())
            departed, arrived = api.simulation.getDepartedIDList(), api.simulation.getArrivedIDList()
            for vehicle in departed:
                if vehicle not in expected or vehicle in departed_seen:
                    raise ValueError("unexpected/duplicate actual departure")
                departed_seen.add(vehicle)
                sf = float(api.vehicle.getSpeedFactor(vehicle))
                if not math.isclose(sf, expected[vehicle]["sf"], rel_tol=1e-12, abs_tol=1e-12):
                    raise ValueError(f"runtime explicit SF mismatch: {vehicle}")
                route = tuple(api.vehicle.getRoute(vehicle))
                if route_digest(route) != expected[vehicle]["route_sha256"]:
                    raise ValueError("SUMO route differs from supplied demand")
                routes[vehicle] = route
                metadata[vehicle] = dict(speed_factor=sf, actual_depart=float(api.vehicle.getDeparture(vehicle)),
                                         planned_depart=expected[vehicle]["depart"], vehicle_type=api.vehicle.getTypeID(vehicle),
                                         vehicle_max_speed=float(api.vehicle.getMaxSpeed(vehicle)))
                sf_values[vehicle] = sf
                lanechange_modes[int(api.vehicle.getLaneChangeMode(vehicle))] += 1
                api.vehicle.subscribe(vehicle, variables)
            teleport = api.simulation.getStartingTeleportIDList()
            counts["teleports"] += len(teleport)
            counts["collisions"] += int(api.simulation.getCollidingVehiclesNumber())
            if teleport:
                raise ValueError("teleport observed despite disabled contract")
            snapshots = api.vehicle.getAllSubscriptionResults()
            observations = []
            internal = []
            stalled = []
            for vehicle, values in snapshots.items():
                road, lane = values[tc.VAR_ROAD_ID], values[tc.VAR_LANE_ID]
                if not road or road.startswith(":"):
                    if road.startswith(":"):
                        internal.append(vehicle)
                    continue
                length, limit = lanes[lane]
                o = Observation(vehicle, road, int(values[tc.VAR_ROUTE_INDEX]), lane,
                                float(values[tc.VAR_LANEPOSITION]), float(values[tc.VAR_SPEED]),
                                float(values[tc.VAR_ALLOWED_SPEED]), length, limit)
                observations.append(o)
                last = observed_progress.get(vehicle)
                # Only a diagnostic guard. Does not delete or free blocked cars.
                if last is None or last[0] != o.route_index or o.position - last[1] >= 1.0 or o.position < last[1] - 1.0:
                    observed_progress[vehicle] = (o.route_index, o.position, now)
                elif args.stall_seconds > 0 and now - last[2] >= args.stall_seconds:
                    stalled.append(dict(vehicle_id=vehicle, road=road, lane=lane, position=o.position, no_progress_seconds=now - last[2]))
            state.step(now, observations, arrived, internal=internal)
            arrived_seen.update(arrived)
            for vehicle in arrived:
                observed_progress.pop(vehicle, None)
            state.discard_arrived(arrived)
            counts["arrived"] = len(arrived_seen)
            counts["departed"] = len(departed_seen)
            if int(now) % args.status_period == 0:
                all_stream.flush()
                train_stream.flush()
                atomic_json(output / "status.json", dict(status="RUNNING", simulation_time=now, wall_seconds=time.time() - started,
                            counts=dict(counts), road_counts=dict(state.counts), running=len(snapshots)))
            if stalled or (output / "STOP").exists():
                api.simulation.saveState(str(output / "stopped_state.xml.gz"))
                atomic_json(output / "stop_evidence.json", dict(simulation_time=now, stalled=stalled,
                            reason="sampled_no_progress_not_proven_deadlock" if stalled else "STOP_requested"))
                result = "CENSORED"
                break
        else:
            result = "COMPLETE"
        if result != "COMPLETE":
            state.censor(now)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        traceback.print_exc()
        if api is not None:
            try:
                api.simulation.saveState(str(output / "failed_state.xml.gz"))
            except Exception:
                pass
        if state is not None:
            state.censor(now)
    finally:
        try:
            if api is not None:
                api.close()
                native_closed = True
        except Exception as exc:
            result, error = "FAILED", f"native close failed: {type(exc).__name__}: {exc}"
        finally:
            all_stream.close()
            train_stream.close()
            if sumo_stdout is not None:
                sumo_stdout.close()
    counts.update({"expected_vehicles": len(expected), "valid_feature_rows": state.counts["trainable"] if state else 0,
                   "feature_errors": state.counts["unusable_feature_rows"] if state else 0})
    counts.update(state.counts if state else {})
    audit = dict(status=result, error=error, counts=dict(counts), simulation_time=now, wall_seconds=time.time() - started,
                 explicit_speed_factor_audited=(len(sf_values) == len(expected)), lane_change_modes=dict(lanechange_modes),
                 contract=sampling_contract(), manifest_sha256=sha256(output / "manifest.json"))
    if result == "COMPLETE":
        try:
            counts.update(tripinfo_audit(output / "tripinfo.xml", expected, sf_values))
            if (arrived_seen != set(expected) or counts["normal_arrived"] != len(expected)
                    or counts["teleports"] or counts["collisions"] or counts["feature_errors"] or counts["unexplained_disappearances"]
                    or counts["road_occurrences"] != manifest["expected_road_occurrences"]):
                raise ValueError("full termination/identity/feature/collision gate failed; inspect counts")
            for p, digest in manifest["inputs"].items():
                if sha256(p) != digest:
                    raise ValueError("input changed during collection")
            (output / "road_visits.partial.csv.gz").replace(output / "road_visits.csv.gz")
            (output / "training.partial.csv.gz").replace(output / "training.csv.gz")
        except Exception as exc:
            audit.update(status="FAILED_AUDIT", error=f"{type(exc).__name__}: {exc}")
    audit["counts"] = dict(counts)
    audit["observed_label_coverage"] = counts["valid_label"] / counts["road_occurrences"] if counts["road_occurrences"] else 0
    audit["native_outputs_closed"] = native_closed
    atomic_json(output / "manifest.json", manifest)
    audit["manifest_sha256"] = sha256(output / "manifest.json")
    atomic_json(output / "audit.json", audit)
    atomic_json(output / "status.json", audit)
    print(json.dumps(audit, ensure_ascii=False))
    return 0 if audit["status"] == "COMPLETE" else 2


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
