#!/usr/bin/env python3
"""Independently audit complete high-precision trajectories and table inputs."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
from roadnet_paths import ROOT
import statistics
import xml.etree.ElementTree as ET


OUTPUT = ROOT / "eval/lane_borrowing_20260929/cpp_eval"
TELEPORT_ID = "congested_veh_013800"


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def metric(rows):
    errors = [float(row["predDuration"]) - float(row["truthDuration"]) for row in rows]
    return {"count": len(rows), "mae": statistics.mean(abs(x) for x in errors),
            "bias": statistics.mean(errors), "rmse": math.sqrt(statistics.mean(x*x for x in errors))}


def main():
    manifest = json.loads((OUTPUT / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "success" and len(manifest["runs"]) == 30
    audits = []
    for run in manifest["runs"]:
        directory = OUTPUT / "runs" / run["run_id"]
        command = run["command"]
        route_path = Path(command[command.index("--sumo-route") + 1])
        truth_path = Path(command[command.index("--sumo-tripinfo") + 1])
        route_root = ET.parse(route_path).getroot()
        definitions = {node.get("id"): node.get("edges").split() for node in route_root.findall("route")}
        expected = {node.get("id"): definitions[node.get("route")] for node in route_root.findall("vehicle")}
        truths = {node.get("id"): node.attrib for node in ET.parse(truth_path).getroot().findall("tripinfo")}
        summary = read_csv(directory / "eval_vehicle_trajectory_summary.csv")
        assert len(summary) == len(expected)
        assert {row["vehicleID"] for row in summary} == expected.keys()
        assert all(row["valid"] == "true" and row["finished"] == "true" for row in summary)
        roads = read_csv(directory / "eval_vehicle_road_timeline.csv")
        movements = read_csv(directory / "eval_vehicle_movement_timeline.csv")
        assert len(roads) == sum(len(path) for path in expected.values())
        assert len(movements) == sum(len(path)-1 for path in expected.values())
        seen_roads, seen_movements = set(), set()
        road_lanes = {}
        road_sums, move_sums, crossing_sums = {}, {}, {}
        for row in roads:
            identifier, index = row["vehicleID"], int(row["routeRoadIndex"])
            assert 0 <= index < len(expected[identifier])
            key = (identifier, index)
            assert key not in seen_roads
            seen_roads.add(key)
            assert row["roadEdge"] == expected[identifier][index]
            assert abs(float(row["arrivalAtRoadEndTime"]) - float(row["enterTime"]) - float(row["roadTravelTime"])) < 1e-7
            road_lanes[key] = int(row["laneIndex"])
            road_sums[identifier] = road_sums.get(identifier, 0.0) + float(row["roadTravelTime"])
            crossing_sums[identifier] = crossing_sums.get(identifier, 0.0) + float(row["intersectionCrossingTime"])
        for row in movements:
            identifier, index = row["vehicleID"], int(row["routeMovementIndex"])
            assert 0 <= index < len(expected[identifier]) - 1
            key = (identifier, index)
            assert key not in seen_movements
            seen_movements.add(key)
            assert row["fromEdge"] == expected[identifier][index]
            assert row["toEdge"] == expected[identifier][index + 1]
            assert abs(float(row["dischargeTime"]) - float(row["arrivalAtWaitingBufferTime"]) - float(row["movementWaitingTime"])) < 1e-7
            assert float(row["movementWaitingTime"]) >= -1e-7
            move_sums[identifier] = move_sums.get(identifier, 0.0) + float(row["movementWaitingTime"])
        max_residual = 0.0
        for row in summary:
            identifier = row["vehicleID"]
            assert float(row["truthDuration"]) == float(truths[identifier]["duration"])
            assert int(row["roadRecordCount"]) == len(expected[identifier])
            assert int(row["movementRecordCount"]) == len(expected[identifier]) - 1
            duration = float(row["predDuration"])
            assert abs(float(row["predArrival"]) - float(row["actualDepart"]) - duration) < 1e-7
            residual = duration - road_sums[identifier] - move_sums.get(identifier, 0.0) - crossing_sums[identifier]
            assert abs(residual) < 1e-7, (run["run_id"], identifier, residual)
            max_residual = max(max_residual, abs(residual))
        audited = {"run_id": run["run_id"], "metrics_all_high_precision": metric(summary),
                   "vehicles": len(summary), "roads": len(roads), "movements": len(movements),
                   "routes_and_record_counts_complete": True,
                   "max_full_duration_accounting_residual": max_residual}
        if run["run_id"].startswith("full_lc/congested/"):
            audited["metrics_excluding_teleported_id"] = metric([row for row in summary if row["vehicleID"] != TELEPORT_ID])
        if run["lane_borrowing"]:
            assert int(run["lane_borrowing"]["storage_reservations"]) == len(roads)
            assert int(run["lane_borrowing"]["storage_releases"]) == len(roads)
            audited["reservation_release_counts_match_all_road_records"] = True
        if "/table/" in run["run_id"]:
            traces = read_csv(directory / "cap_context_trace.csv")
            assert len(traces) == len(roads) == run["table_hits"]
            seen = set()
            max_lane_flow = 0
            boundary_rows = 0
            for row in traces:
                key = (row["sumoVehicleID"], int(row["route_index"]))
                assert key not in seen
                seen.add(key)
                assert int(row["laneIndex"]) == road_lanes[key]
                assert row["sumoRoadID"] == expected[key[0]][key[1]]
                lane_flow = int(row["lane_flow"])
                # These inputs have one common car type: length=5, minGap=2.5.
                capacity = max(1, math.floor((float(row["road_length"]) + 2.5) / 7.5) + 1)
                assert 1 <= lane_flow <= capacity
                max_lane_flow = max(max_lane_flow, lane_flow)
                boundary_rows += lane_flow == capacity
            assert seen == seen_roads
            audited["table_trace"] = {"rows": len(traces), "logical_lane_identity_matches_timeline": True,
                "all_lane_flow_within_original_single_lane_capacity": True,
                "max_lane_flow": max_lane_flow, "rows_at_capacity": boundary_rows}
        audits.append(audited)
    result = {"status": "success", "run_count": len(audits), "audits": audits,
              "audit_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "metric_source": "15-digit trajectory summary CSV; predDuration minus matched SUMO truthDuration",
              "limitations": "Per-run final-state checks and causal traces do not prove every transient invariant or physical merge feasibility."}
    (OUTPUT / "independent_trajectory_audit.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    with (OUTPUT / "metrics_high_precision.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["run_id", "count", "mae", "bias", "rmse", "exclude_teleport_mae", "exclude_teleport_bias"])
        writer.writeheader()
        for audit in audits:
            row = {"run_id": audit["run_id"], **audit["metrics_all_high_precision"]}
            excluded = audit.get("metrics_excluding_teleported_id", {})
            row.update({"exclude_teleport_mae": excluded.get("mae"), "exclude_teleport_bias": excluded.get("bias")})
            writer.writerow(row)
    print(f"PASS: {len(audits)} complete trajectory audits, high-precision metrics saved")


if __name__ == "__main__":
    main()
