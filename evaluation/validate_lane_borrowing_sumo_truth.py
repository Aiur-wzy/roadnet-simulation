#!/usr/bin/env python3
"""Create isolated full-lane-change truth for the three explicit-SF baselines.

This runner changes only lane-change input controls and output locations. TraCI
is used only to step and read; no vehicle, lane, signal, or simulation setter is
called. Original inputs and historical ground truth are never overwritten.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time
import xml.etree.ElementTree as ET


SCENARIOS = ("free", "tight", "congested")
LC_KEYS = ("lcStrategic", "lcCooperative", "lcSpeedGain", "lcKeepRight")


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def option(root, name, default=None):
    node = root.find(".//" + name)
    return node.get("value") if node is not None else default


def set_option(root, group_name, name, value):
    group = root.find(group_name)
    if group is None:
        group = ET.SubElement(root, group_name)
    node = group.find(name)
    if node is None:
        node = ET.SubElement(group, name)
    node.set("value", value)


def relative(path, directory):
    return Path(os.path.relpath(path, directory)).as_posix()


def signature(node):
    return [node.tag, sorted(node.attrib.items()), (node.text or "").strip(),
            [signature(child) for child in node]]


def input_paths(source, scenario):
    config = source / "sumocfg" / f"{scenario}.sumocfg"
    root = ET.parse(config).getroot()
    return config, (config.parent / option(root, "route-files")).resolve(), (
        config.parent / option(root, "net-file")).resolve()


def prepare(source, output):
    if output.exists():
        raise RuntimeError(f"Refusing to overwrite existing output: {output}")
    output.mkdir(parents=True)
    manifest = {"schema": "lane-borrowing-full-lc-truth-v1", "scenarios": {}}
    for scenario in SCENARIOS:
        source_config, source_route, source_net = input_paths(source, scenario)
        config = output / "sumocfg" / f"{scenario}.sumocfg"
        route = output / "routes" / source_route.name
        net = output / "inputs" / source_net.name
        run = output / "runs" / scenario
        for parent in (config.parent, route.parent, net.parent, run):
            parent.mkdir(parents=True, exist_ok=True)
        if not net.exists():
            shutil.copy2(source_net, net)
        assert sha(net) == sha(source_net)
        original = ET.parse(source_route).getroot()
        updated = copy.deepcopy(original)
        changes = []
        for node in updated.iter():
            if node.tag == "vType":
                for key in LC_KEYS:
                    before = node.get(key)
                    node.set(key, "1")
                    if before != "1":
                        changes.append({"tag": node.tag, "id": node.get("id"),
                                        "attribute": key, "before": before, "after": "1"})
            if "laneChangeMode" in node.attrib:
                changes.append({"tag": node.tag, "id": node.get("id"),
                                "attribute": "laneChangeMode", "before": node.get("laneChangeMode"),
                                "after": None})
                del node.attrib["laneChangeMode"]
            if node.tag == "param" and "laneChangeMode" in node.get("key", ""):
                raise RuntimeError("Unreviewed nested laneChangeMode override")
        # Prove that no other route semantics changed.
        normalized_original, normalized_updated = copy.deepcopy(original), copy.deepcopy(updated)
        for root in (normalized_original, normalized_updated):
            for node in root.iter():
                if node.tag == "vType":
                    for key in LC_KEYS:
                        node.attrib.pop(key, None)
                node.attrib.pop("laneChangeMode", None)
        assert signature(normalized_original) == signature(normalized_updated)
        ET.indent(updated)
        ET.ElementTree(updated).write(route, encoding="utf-8", xml_declaration=True)
        cfg_original = ET.parse(source_config).getroot()
        cfg = copy.deepcopy(cfg_original)
        set_option(cfg, "input", "net-file", relative(net, config.parent))
        set_option(cfg, "input", "route-files", relative(route, config.parent))
        for key, filename in (("tripinfo-output", "tripinfo.xml"),
                              ("statistic-output", "statistics.xml"),
                              ("vehroute-output", "vehroute.xml")):
            set_option(cfg, "output", key, relative(run / filename, config.parent))
        set_option(cfg, "output", "vehroute-output.exit-times", "true")
        set_option(cfg, "output", "vehroute-output.write-unfinished", "true")
        set_option(cfg, "output", "tripinfo-output.write-unfinished", "true")
        set_option(cfg, "report", "log", relative(run / "sumo.log", config.parent))
        set_option(cfg, "report", "error-log", relative(run / "sumo.errors.log", config.parent))
        before_options = {child.tag: child.get("value") for group in cfg_original for child in group}
        after_options = {child.tag: child.get("value") for group in cfg for child in group}
        allowed = {"net-file", "route-files", "tripinfo-output", "statistic-output",
                   "vehroute-output", "vehroute-output.exit-times", "vehroute-output.write-unfinished",
                   "tripinfo-output.write-unfinished", "log", "error-log"}
        cfg_diff = {key: [before_options.get(key), after_options.get(key)]
                    for key in sorted(before_options.keys() | after_options.keys())
                    if before_options.get(key) != after_options.get(key)}
        assert set(cfg_diff) <= allowed
        ET.indent(cfg)
        ET.ElementTree(cfg).write(config, encoding="utf-8", xml_declaration=True)
        write_json(output / "manifests" / f"{scenario}_semantic_diff.json", {
            "route_changes": changes, "config_changes": cfg_diff,
            "route_equivalent_except_lc_controls": True,
            "vehicle_count": len(updated.findall("vehicle")),
            "explanation": "Remove XML laneChangeMode attributes and verify actual default 1621; no TraCI setter."})
        manifest["scenarios"][scenario] = {
            "config": str(config), "route": str(route), "net": str(net),
            "source_config": str(source_config), "source_route": str(source_route), "source_net": str(source_net),
            "source_hashes": {str(p): sha(p) for p in (source_config, source_route, source_net)},
            "prepared_hashes": {str(p): sha(p) for p in (config, route, net)},
            "seed": option(cfg, "seed"), "step_length_input": option(cfg, "step-length", "SUMO default"),
            "vehicle_count": len(updated.findall("vehicle")),
        }
    write_json(output / "manifests" / "preparation.json", manifest)
    return manifest


def load_traci(binary):
    sys.path.insert(0, str(Path(binary).resolve().parent.parent / "tools"))
    import traci
    return traci


def run_scenario(output, scenario, spec, binary):
    traci = load_traci(binary)
    run = output / "runs" / scenario
    result_path = output / "manifests" / f"{scenario}_result.json"
    if result_path.exists():
        raise RuntimeError(f"Refusing to overwrite run: {scenario}")
    expected_nodes = ET.parse(spec["route"]).getroot().findall("vehicle")
    expected = {node.get("id"): node.attrib for node in expected_nodes}
    command = [str(binary), "-c", spec["config"]]
    write_json(run / "command.json", {"command": command, "binary_sha256": sha(binary),
                                       "runner_sha256": sha(Path(__file__)), "traci": traci.__file__})
    seen, departed, arrived = set(), set(), set()
    teleports, collisions = [], []
    start = time.monotonic()
    with (run / "stdout.log").open("w", encoding="utf-8") as stdout, (
            run / "vehicle_getters.csv").open("w", encoding="utf-8", newline="") as audit_stream:
        audit = csv.writer(audit_stream)
        audit.writerow(["vehicle_id", "observed_at", "speed_factor", "route_speed_factor",
                        "lane_change_mode", *LC_KEYS])
        traci.start(command, stdout=stdout)
        try:
            version = traci.getVersion()
            options = {key: traci.simulation.getOption(key) for key in (
                "seed", "step-length", "time-to-teleport", "time-to-teleport.highways",
                "collision.action", "lanechange.duration", "lateral-resolution", "route-steps")}
            assert traci.simulation.getDeltaT() == 1.0
            assert options["seed"] == spec["seed"]

            def loaded():
                now = traci.simulation.getTime()
                for identifier in traci.simulation.getLoadedIDList():
                    if identifier in seen:
                        continue
                    assert identifier in expected, identifier
                    sf = traci.vehicle.getSpeedFactor(identifier)
                    lc_mode = traci.vehicle.getLaneChangeMode(identifier)
                    params = [traci.vehicle.getParameter(identifier, "laneChangeModel." + key) for key in LC_KEYS]
                    assert sf == float(expected[identifier]["speedFactor"]), (identifier, sf)
                    assert lc_mode == 1621, (identifier, lc_mode)
                    assert all(float(value) == 1.0 for value in params), (identifier, params)
                    audit.writerow([identifier, now, repr(sf), expected[identifier]["speedFactor"], lc_mode, *params])
                    seen.add(identifier)

            loaded()
            while traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                loaded()
                new_departed = set(traci.simulation.getDepartedIDList())
                new_arrived = set(traci.simulation.getArrivedIDList())
                assert not new_departed & departed
                assert not new_arrived & arrived
                departed.update(new_departed)
                arrived.update(new_arrived)
                now = traci.simulation.getTime()
                for identifier in traci.simulation.getStartingTeleportIDList():
                    teleports.append({"time": now, "vehicle_id": identifier})
                for identifier in traci.simulation.getCollidingVehiclesIDList():
                    collisions.append({"time": now, "vehicle_id": identifier})
                if int(now) % 1000 == 0:
                    print(f"{scenario}: t={now:g}, arrived={len(arrived)}/{len(expected)}, "
                          f"teleports={len(teleports)}, collisions={len(collisions)}", flush=True)
                if now > 100000:
                    raise RuntimeError(f"Safety bound reached without clearing: {scenario}")
            simulation_end = traci.simulation.getTime()
            active_final = list(traci.vehicle.getIDList())
        finally:
            traci.close()
    trip_rows = ET.parse(run / "tripinfo.xml").getroot().findall("tripinfo")
    trip_by_id = {node.get("id"): node.attrib for node in trip_rows}
    route_rows = ET.parse(run / "vehroute.xml").getroot().findall("vehicle")
    route_by_id = {node.get("id"): node for node in route_rows}
    expected_ids = set(expected)
    assert expected_ids == seen == departed == arrived == set(trip_by_id) == set(route_by_id)
    assert len(trip_rows) == len(trip_by_id) and len(route_rows) == len(route_by_id)
    assert not active_final
    incomplete = [identifier for identifier, row in trip_by_id.items() if float(row["arrival"]) < 0]
    assert not incomplete
    missing_exit_times = [identifier for identifier, node in route_by_id.items()
                          if node.find("route") is None or "exitTimes" not in node.find("route").attrib]
    assert not missing_exit_times
    route_definitions = {node.get("id"): node.get("edges", "").split()
                         for node in ET.parse(spec["route"]).getroot().findall("route")}
    route_mismatches, exit_count_mismatches = [], []
    for identifier, node in route_by_id.items():
        actual_route = node.find("route")
        edges = actual_route.get("edges").split()
        if edges != route_definitions[expected[identifier]["route"]]:
            route_mismatches.append(identifier)
        if len(edges) != len(actual_route.get("exitTimes").split()):
            exit_count_mismatches.append(identifier)
    assert not route_mismatches and not exit_count_mismatches
    for path, digest in {**spec["source_hashes"], **spec["prepared_hashes"]}.items():
        assert sha(path) == digest, path
    result = {
        "status": "success", "scenario": scenario, "version": version, "command": command,
        "options_getter": options, "vehicles": len(expected), "seen": len(seen),
        "departed": len(departed), "arrived": len(arrived), "tripinfo_rows": len(trip_rows),
        "vehroute_rows": len(route_rows), "active_final": active_final, "unfinished": incomplete,
        "all_vehicle_ids_exact": True, "all_sf_exact": True, "all_lc_mode_1621": True,
        "all_four_lc_parameters_one": True, "no_traci_setter": True,
        "all_routes_unchanged": True, "all_route_exit_times_complete": True,
        "teleports": teleports, "collisions": collisions, "simulation_end": simulation_end,
        "elapsed_seconds": time.monotonic() - start,
        "binary_sha256": sha(binary), "source_and_prepared_hashes_unchanged": True,
        "artifact_hashes": {relative(p, output): sha(p) for p in run.iterdir() if p.is_file()},
    }
    write_json(result_path, result)
    print(f"{scenario}: complete, {len(expected)} vehicles, t={simulation_end}, "
          f"{result['elapsed_seconds']:.2f}s wall, teleports={len(teleports)}, collisions={len(collisions)}", flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("data/variants/speed_factor"))
    parser.add_argument("--output", type=Path, default=Path("eval/lane_borrowing_20260929/sumo_full_lc"))
    parser.add_argument("--sumo-binary", type=Path, default=Path(shutil.which("sumo") or "sumo"))
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--run-prepared", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    if args.run_prepared:
        manifest = json.loads((output / "manifests" / "preparation.json").read_text(encoding="utf-8"))
    else:
        manifest = prepare(args.source.resolve(), output)
    if args.prepare_only:
        print(json.dumps(manifest, indent=2))
        return
    results = [run_scenario(output, name, manifest["scenarios"][name], args.sumo_binary.resolve())
               for name in SCENARIOS]
    write_json(output / "manifests" / "run_summary.json", {"results": results})


if __name__ == "__main__":
    main()
