"""Independent BOUNDED observer for boundary-v2 validation, not a batch launcher.

The original collectors are imported read-only for feature comparison. No
vehicle/TLS behavior setters, no changed step length, and no implicit resume.
The explicit limits prevent this diagnostic from launching the full dataset.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source-root", required=True, help="Existing canonical source, read only")
    p.add_argument("--sumo-config", required=True)
    p.add_argument("--net-file", required=True)
    p.add_argument("--output-dir", required=True)
    p.add_argument("--sumo-binary", required=True)
    p.add_argument("--maximum-vehicles", type=int, default=128)
    p.add_argument("--maximum-seconds", type=float, default=20000)
    args = p.parse_args(argv)
    if not 1 <= args.maximum_vehicles <= 128 or not 0 < args.maximum_seconds <= 20000:
        p.error("bounded diagnostic requires <=128 vehicles and <=20000 simulated seconds")
    sys.path.insert(0, str(Path(args.source_root).resolve()))
    from collection.TraCI_Python_SUMO_Native import (read_inputs, load_traci, sha256,
                                                    atomic_json, route_digest, FIELDS)
    from collection.sumo_native_state import Observation
    try:
        from .sumo_native_signals_v2 import NativeStateV2, NativeSignalReaderV2
        from .sumo_native_path_v2 import NoInternalGeometry
    except ImportError:
        from sumo_native_signals_v2 import NativeStateV2, NativeSignalReaderV2
        from sumo_native_path_v2 import NoInternalGeometry
    cfg, net, out = (Path(x).resolve() for x in (args.sumo_config, args.net_file, args.output_dir))
    expected, route_files = read_inputs(cfg, net)
    if len(expected) > args.maximum_vehicles:
        raise ValueError("input exceeds diagnostic vehicle bound; no SUMO started")
    NoInternalGeometry(net)  # Refuse unsupported networks BEFORE launching SUMO.
    options = {n.tag: n.get("value") for n in ET.parse(cfg).getroot().iter() if "value" in n.attrib}
    if options.get("step-method.ballistic", "false").lower() not in {"false", "0"}:
        raise ValueError("only semi-implicit Euler supported; input left unchanged")
    for behavior_option in ("mesosim", "step-method.ballistic"):
        if options.get(behavior_option, "false").lower() not in {"false", "0"}:
            raise ValueError("unsupported integration/simulation mode")
    # An arbitrary output from the input config must never overwrite old truth.
    if any("output" in key or key in {"log", "error-log", "save-state.times", "save-state.period"}
           for key in options):
        raise ValueError("make an isolated input-only config before using this probe")
    lanes = {}
    for _, edge in ET.iterparse(net, events=("end",)):
        if edge.tag == "edge":
            for lane in edge.findall("lane"):
                lanes[lane.get("id")] = float(lane.get("length")), float(lane.get("speed"))
            edge.clear()
    out.mkdir(parents=True, exist_ok=False)
    command = [args.sumo_binary, "-c", str(cfg), "--tripinfo-output", str(out / "tripinfo.xml"),
               "--tripinfo-output.write-unfinished", "true", "--vehroute-output", str(out / "vehroute.xml.gz"),
               "--vehroute-output.exit-times", "true", "--vehroute-output.write-unfinished", "true",
               "--vehroute-output.speedfactor", "true", "--lanechange-output", str(out / "lanechanges.xml.gz"),
               "--statistic-output", str(out / "statistics.xml"), "--precision", "12",
               "--log", str(out / "sumo.log"), "--error-log", str(out / "sumo.stderr.log"), "--no-step-log", "true"]
    dependencies = set(Path(__file__).parent.glob("*.py"))
    for package in ("collection", "features"):
        dependencies.update((Path(args.source_root).resolve()/package).rglob("*.py"))
    manifest = dict(schema="sumo-boundary-v2-bounded-probe", input_sha256={str(x): sha256(x) for x in (cfg, net, *route_files)},
                    network_sha256=sha256(net),
                    source_sha256={str(x.resolve()): sha256(x) for x in sorted(dependencies)}, command=command,
                    expected_vehicles=len(expected), maximum_seconds=args.maximum_seconds,
                    behavior_setters_used=False, original_collectors_modified=False,
                    feature_snapshot_contract="physical post-step snapshot clock; v1 label counter retained only for comparison")
    started, api, status, error = time.monotonic(), None, "FAILED", None
    arrived_set, sf_values, routes, metadata = set(), {}, {}, {}
    collisions = teleports = 0
    traci = load_traci()
    from traci import constants as tc
    state = None
    try:
        with (out / "stdout.log").open("w", encoding="utf-8") as stdout, \
                gzip.open(out / "trace.jsonl.gz", "wt", encoding="utf-8", compresslevel=1) as trace, \
                gzip.open(out / "snapshot_feature_rows.csv.gz", "wt", encoding="utf-8", newline="", compresslevel=1) as old:
            fields = (*FIELDS, "snapshot_clock_schema", "snapshot_clock_step_seconds", "signal_clock_schema",
                      "signal_api_return_time", "signal_snapshot_time", "signal_step_seconds")
            old_writer = csv.DictWriter(old, fields, extrasaction="ignore")
            old_writer.writeheader()
            label = f"boundary_v2_probe_{os.getpid()}"
            traci.start(command, label=label, stdout=stdout)
            api = traci.getConnection(label)
            version, dt = api.getVersion(), float(api.simulation.getDeltaT())
            if not ("1.27.0" in version[1] or "1.27.1" in version[1]):
                raise ValueError("uncalibrated SUMO version; validate snapshot/terminal clock before extending support")
            manifest.update(sumo_version=version, step_seconds=dt,
                            position_clock="simulation.getTime() minus step: verified Euler/FCD clock")
            atomic_json(out / "manifest.json", manifest)
            def record(value):
                trace.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
            record(dict(event="contract", step_seconds=dt, version=version, integration="euler",
                        feature_time_clock="state_time"))
            state = NativeStateV2(routes, metadata, NativeSignalReaderV2(api, net), old_writer.writerow)
            variables = (tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_ROUTE_INDEX, tc.VAR_LANEPOSITION,
                         tc.VAR_SPEED, tc.VAR_ALLOWED_SPEED, tc.VAR_DISTANCE)
            while api.simulation.getMinExpectedNumber() > 0:
                api.simulationStep()
                now = float(api.simulation.getTime())
                departed, arrived = api.simulation.getDepartedIDList(), api.simulation.getArrivedIDList()
                for vehicle in departed:
                    if vehicle in sf_values or vehicle not in expected:
                        raise ValueError("departure identity mismatch")
                    if vehicle in arrived:
                        raise ValueError("same-step departure/arrival lacks observable features: explicit unsupported case")
                    route = tuple(api.vehicle.getRoute(vehicle))
                    sf = float(api.vehicle.getSpeedFactor(vehicle))
                    if route_digest(route) != expected[vehicle]["route_sha256"] or abs(sf - expected[vehicle]["sf"]) > 1e-12:
                        raise ValueError("route/SF runtime mismatch")
                    routes[vehicle] = route
                    sf_values[vehicle] = sf
                    metadata[vehicle] = dict(speed_factor=sf, actual_depart=float(api.vehicle.getDeparture(vehicle)),
                                            planned_depart=expected[vehicle]["depart"], vehicle_type=api.vehicle.getTypeID(vehicle),
                                            vehicle_max_speed=float(api.vehicle.getMaxSpeed(vehicle)))
                    record(dict(event="depart", vehicle=vehicle, route=route, metadata=metadata[vehicle],
                                lane_change_mode=api.vehicle.getLaneChangeMode(vehicle)))
                    api.vehicle.subscribe(vehicle, variables)
                teleports += len(api.simulation.getStartingTeleportIDList())
                collisions += int(api.simulation.getCollidingVehiclesNumber())
                if teleports or collisions:
                    raise ValueError("teleport/collision: retain evidence but do not certify this probe")
                observations, raw, internal = [], [], []
                for vehicle, values in api.vehicle.getAllSubscriptionResults().items():
                    edge, lane = values[tc.VAR_ROAD_ID], values[tc.VAR_LANE_ID]
                    value = dict(id=vehicle, edge=edge, lane=lane, route_index=int(values[tc.VAR_ROUTE_INDEX]),
                                 position=float(values[tc.VAR_LANEPOSITION]), speed=float(values[tc.VAR_SPEED]),
                                 cap=float(values[tc.VAR_ALLOWED_SPEED]), distance=float(values[tc.VAR_DISTANCE]),
                                 time=now - dt, api_time=now)
                    raw.append(value)
                    if edge.startswith(":"):
                        internal.append(vehicle)
                    elif edge:
                        length, limit = lanes[lane]
                        observations.append(Observation(vehicle, edge, value["route_index"], lane,
                                            value["position"], value["speed"], value["cap"], length, limit))
                record(dict(event="step", api_time=now, state_time=now - dt, observations=raw, arrived=list(arrived)))
                state.step_from_api(api, observations, arrived, internal=internal)
                state.discard_arrived(arrived)
                arrived_set.update(arrived)
                if now >= args.maximum_seconds or (out / "STOP").exists():
                    if api.simulation.getMinExpectedNumber() > 0:
                        status = "BOUNDED_CENSORED"
                        api.simulation.saveState(str(out / "bounded_state.xml.gz"))
                        break
            else:
                status = "COMPLETE"
            if status != "COMPLETE":
                state.censor(now-dt)
            api.close()
            api = None
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        if api is not None:
            try:
                api.close()
            except Exception as closing:
                error += f"; close: {closing}"
    if status == "COMPLETE" and arrived_set != set(expected):
        status, error = "FAILED_AUDIT", "incomplete arrival ID set"
    for path, digest in manifest["input_sha256"].items():
        if sha256(path) != digest:
            status, error = "FAILED_AUDIT", "input changed during probe"
    for path, original_digest in manifest["source_sha256"].items():
        if sha256(path) != original_digest:
            status, error = "FAILED_AUDIT", "source changed during probe; retain, do not certify"
    closed_outputs = {}
    if status == "COMPLETE":
        for name in ("trace.jsonl.gz", "tripinfo.xml", "snapshot_feature_rows.csv.gz",
                     "vehroute.xml.gz", "lanechanges.xml.gz", "statistics.xml"):
            closed_outputs[name] = sha256(out/name)
    audit = dict(status=status, error=error, arrivals=len(arrived_set), expected=len(expected),
                 collisions=collisions, teleports=teleports, wall_seconds=time.monotonic()-started,
                 native_closed=api is None, old_road_counts=dict(state.counts) if state else {},
                 closed_outputs_sha256=closed_outputs,
                 manifest_sha256=sha256(out / "manifest.json") if (out / "manifest.json").exists() else None)
    atomic_json(out / "audit.json", audit)
    print(json.dumps(audit, ensure_ascii=False))
    return 0 if status == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
