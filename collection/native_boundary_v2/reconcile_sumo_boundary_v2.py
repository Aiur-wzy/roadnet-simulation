"""Reconcile CLOSED bounded native observations; never invent entry features.

This research adapter accepts <=128 vehicles. It is deliberately separate from
the full-data collector pending approval of the unobserved-feature policy.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import replace
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

try:
    from .sumo_native_boundary_v2 import allocate_step, LABELS, THRESHOLDS
    from .sumo_native_path_v2 import NoInternalGeometry, GeometryUnknown
except ImportError:
    from sumo_native_boundary_v2 import allocate_step, LABELS, THRESHOLDS
    from sumo_native_path_v2 import NoInternalGeometry, GeometryUnknown

# Physical arrival acceptance used by SUMO 1.27.0/1.27.1, not a floating
# tolerance. Ordinary crossing geometry retains its strict numeric tolerance.
SUMO_POSITION_EPS_METRES = 0.1


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1048576), b""):
            h.update(block)
    return h.hexdigest()


def reconcile(net, trace_path, trip_path, feature_path, output, maximum_vehicles=128):
    output = Path(output).resolve()
    if output.exists():
        raise ValueError("fresh output directory required")
    graph = NoInternalGeometry(net)
    trips = {}
    for _, node in ET.iterparse(trip_path, events=("end",)):
        if node.tag != "tripinfo":
            continue
        vid = node.get("id")
        if not vid or vid in trips or node.get("vaporized", "") not in {"", "false"}:
            raise ValueError("unexpected duplicate/abnormal tripinfo identity")
        if any(not math.isfinite(float(node.get(k))) or float(node.get(k)) < 0
               for k in ("depart", "arrival", "duration", "arrivalSpeed", "arrivalPos", "routeLength")):
            raise ValueError("unfinished/nonfinite tripinfo")
        if abs(float(node.get("arrival"))-float(node.get("depart"))-float(node.get("duration"))) > 1e-6:
            raise ValueError("tripinfo time identity failed")
        trips[vid] = dict(node.attrib)
    if len(trips) > maximum_vehicles:
        raise ValueError("bounded research adapter refuses full dataset")
    vehicles, previous, vehicle_budgets, rows, failures = {}, {}, {}, {}, []
    counts, audit_totals = Counter(), Counter()
    arrival_acceptance = []
    def row_for(vid, index):
        key = vid, index
        if key not in rows:
            rows[key] = dict(vehicle_id=vid, route_index=index, road_id=vehicles[vid]["route"][index],
                             labels=[0.0]*len(THRESHOLDS), seconds=0.0, valid=True, touched=False,
                             reasons=set(), enter=None, exit=None, intervals=0)
        return rows[key]
    def unknown(vid, indices, reason, elapsed=0.0, details=None):
        for i in indices:
            row = row_for(vid, i)
            row["valid"] = False
            row["reasons"].add(reason)
        audit_totals["unresolved_seconds"] += elapsed
        vehicle_budgets[vid]["unresolved_seconds"] += elapsed
        counts["unresolved_"+reason] += 1
        if len(failures) < 500:
            failures.append(dict(vehicle=vid, occurrences=list(indices), reason=reason, elapsed=elapsed, details=details))
    def apply(vid, result, affected):
        if result.status != "RESOLVED":
            unknown(vid, affected, result.reason.value, result.budget.elapsed or 0.0,
                    dict(detail=result.detail, distance_residual=result.audit.distance_residual,
                         raw_time_residual=result.audit.raw_time_residual))
            return False
        audit_totals["allocated_seconds"] += result.audit.allocated_elapsed
        vehicle_budgets[vid]["allocated_seconds"] += result.audit.allocated_elapsed
        for part in result.parts:
            row = row_for(vid, part.occurrence)
            row["touched"] = True
            row["seconds"] += part.time
            row["intervals"] += 1
            row["enter"] = part.t_enter if row["enter"] is None else min(row["enter"], part.t_enter)
            row["exit"] = part.t_exit if row["exit"] is None else max(row["exit"], part.t_exit)
            row["labels"] = [a+b for a,b in zip(row["labels"], part.labels)]
        counts["resolved_steps"] += 1
        return True
    with gzip.open(trace_path, "rt", encoding="utf-8") as stream:
        for line in stream:
            event = json.loads(line)
            if event["event"] == "contract":
                if event["integration"] != "euler":
                    raise ValueError("unsupported integration")
                dt = event["step_seconds"]
                if event.get("feature_time_clock") != "state_time":
                    raise ValueError("feature clock must be calibrated, not API-next-step time")
            elif event["event"] == "depart":
                vid = event["vehicle"]
                if vid in vehicles or len(vehicles) >= maximum_vehicles:
                    raise ValueError("duplicate/excess departure")
                vehicles[vid] = event
                vehicle_budgets[vid] = Counter()
            elif event["event"] == "step":
                observed = {v["id"]: v for v in event["observations"]}
                if len(observed) != len(event["observations"]):
                    raise ValueError("duplicate snapshot vehicle")
                for vid, current in observed.items():
                    route = vehicles[vid]["route"]
                    if current["edge"].startswith(":"):
                        raise ValueError("no-internal adapter must not consume internal snapshots")
                    if current["edge"] != route[current["route_index"]]:
                        raise ValueError("route occurrence mismatch")
                    if not math.isclose(current["time"], event["api_time"]-dt, abs_tol=1e-9):
                        raise ValueError("snapshot clock mismatch")
                    if vid not in previous:
                        gap = current["time"] - vehicles[vid]["metadata"]["actual_depart"]
                        if abs(gap) > 1e-7 or current["route_index"] != 0:
                            unknown(vid, range(current["route_index"]+1), "unobserved_departure_interval", max(0,gap))
                        row_for(vid, current["route_index"])["touched"] = True
                        previous[vid] = current
                        continue
                    old = previous[vid]
                    affected = range(old["route_index"], current["route_index"]+1)
                    elapsed = current["time"] - old["time"]
                    if not math.isclose(elapsed, dt, abs_tol=1e-8):
                        unknown(vid, affected, "missing_snapshot_step", elapsed)
                    else:
                        try:
                            segments = graph.step_segments(route, old, current)
                            # Road/position geometry alone can be misleading after
                            # route distance jumps; independently check odometry.
                            if "distance" in old and "distance" in current and not math.isclose(
                                    current["distance"]-old["distance"], current["speed"]*elapsed,
                                    abs_tol=1e-6, rel_tol=1e-9):
                                raise GeometryUnknown("odometer_not_closed")
                            result = allocate_step(t0=old["time"], t1=current["time"], speed=current["speed"], segments=segments)
                            apply(vid, result, affected)
                        except GeometryUnknown as exc:
                            unknown(vid, affected, str(exc), elapsed)
                    previous[vid] = current
                for vid in event["arrived"]:
                    trip, old = trips[vid], previous.pop(vid)
                    route = vehicles[vid]["route"]
                    end, speed = float(trip["arrival"]), float(trip["arrivalSpeed"])
                    affected = range(old["route_index"], len(route))
                    elapsed = end-old["time"]
                    acceptance_record = None
                    if elapsed < -1e-8 or not math.isclose(end, event["state_time"], abs_tol=1e-7):
                        raise ValueError("terminal clock inconsistent with validated SUMO contract")
                    try:
                        if not math.isclose(elapsed, dt, abs_tol=1e-8):
                            raise GeometryUnknown("missing_terminal_snapshot_step")
                        segments = graph.terminal_segments(route, old, trip["arrivalLane"], float(trip["arrivalPos"]))
                        distance = math.fsum(s.distance for s in segments)
                        if "distance" not in old or not math.isclose(
                                float(trip["routeLength"])-old["distance"], distance,
                                abs_tol=1e-6, rel_tol=1e-9):
                            raise GeometryUnknown("terminal_odometer_not_closed")
                        shortfall = distance-speed*elapsed
                        if shortfall > 1e-7:
                            predicted_front = old["position"]+speed*elapsed
                            for route_index in range(old["route_index"], len(route)-1):
                                predicted_front -= graph.road_length(route[route_index])
                            native_boundary = min(graph.lane_length(trip["arrivalLane"]), float(trip["arrivalPos"]))
                            # Match the strict native POSITION comparison, not
                            # the algebraically similar (target-front)<0.1,
                            # which can differ at the floating-point boundary.
                            if (not predicted_front > native_boundary-SUMO_POSITION_EPS_METRES
                                    or shortfall > segments[-1].distance):
                                raise GeometryUnknown("terminal_distance_speed_not_closed")
                            # hasArrived() accepts front position up to 0.1 m
                            # BEFORE its nominal target; tripinfo still writes
                            # that target. Allocate actual Euler displacement,
                            # do not invent the small untravelled distance/time.
                            segments = (*segments[:-1], replace(segments[-1], distance=segments[-1].distance-shortfall))
                            distance = math.fsum(s.distance for s in segments)
                            acceptance_record = dict(vehicle_id=vid, route_index=len(route)-1,
                                nominal_arrival_position=float(trip["arrivalPos"]),
                                inferred_front_position=predicted_front,
                                untravelled_target_distance_m=shortfall, source_rule="SUMO hasArrived POSITION_EPS=0.1m")
                        if speed <= 0 or distance < 0 or distance > speed*elapsed+1e-7:
                            raise GeometryUnknown("terminal_distance_speed_not_closed")
                        moving = distance/speed
                        # SUMO tripinfo terminates at the discrete step end. The
                        # fractional overshoot remainder is NOT road motion.
                        remainder = max(0.0, elapsed-moving)
                        audit_totals["terminal_rounding_seconds"] += remainder
                        vehicle_budgets[vid]["terminal_rounding_seconds"] += remainder
                        if moving > 1e-9:
                            resolved = apply(vid, allocate_step(t0=old["time"], t1=old["time"]+moving,
                                                   speed=speed, segments=segments), affected)
                        else:
                            for i in affected:
                                row_for(vid,i)["touched"] = True
                            resolved = True
                        if resolved and acceptance_record is not None:
                            arrival_acceptance.append(acceptance_record)
                            counts["native_arrival_acceptance_band"] += 1
                        if float(trip["depart"]) != vehicles[vid]["metadata"]["actual_depart"]:
                            raise ValueError("departure audit mismatch")
                    except GeometryUnknown as exc:
                        unknown(vid, affected, str(exc), elapsed)
                    counts["arrivals"] += 1
    if previous or set(vehicles) != set(trips) or counts["arrivals"] != len(vehicles):
        raise ValueError("incomplete trace/terminal identities")
    total_duration = math.fsum(float(t["duration"]) for t in trips.values())
    accounted = math.fsum(audit_totals[k] for k in ("allocated_seconds","unresolved_seconds","terminal_rounding_seconds"))
    if not math.isclose(total_duration, accounted, abs_tol=1e-5, rel_tol=1e-10):
        raise ValueError(f"global time conservation failed: {total_duration} != {accounted}")
    per_vehicle = []
    for vid, totals in vehicle_budgets.items():
        duration = float(trips[vid]["duration"])
        residual = duration - math.fsum(totals.values())
        if abs(residual) > 1e-6:
            raise ValueError(f"vehicle time conservation failed: {vid}: {residual}")
        per_vehicle.append(dict(vehicle_id=vid, duration=duration, **dict(totals), residual=residual))
    feature_rows = {}
    with gzip.open(feature_path,"rt",encoding="utf-8",newline="") as f:
        for row in csv.DictReader(f):
            key = row["vehicle_id"], int(row["route_index"])
            if key in feature_rows:
                raise ValueError("duplicate feature occurrence")
            if (key[0] not in vehicles or not 0 <= key[1] < len(vehicles[key[0]]["route"])
                    or row.get("road_id") != vehicles[key[0]]["route"][key[1]]):
                raise ValueError("feature road identity disagrees with vehicle route occurrence")
            feature_rows[key] = row
    extra = ["v2_label_valid", "v2_quality", "v2_partial_seconds", "v2_enter", "v2_exit",
             "feature_snapshot_lag_seconds", "v2_trainable_observed_snapshot", "v2_exact_entry_features_known"]
    fields = list(next(iter(feature_rows.values())).keys()) + ["v1_"+k for k in LABELS] + extra
    counts["expected_occurrences"] = sum(len(v["route"]) for v in vehicles.values())
    if len(feature_rows) != counts["expected_occurrences"]:
        raise ValueError("feature ledger missing planned occurrences")
    output.mkdir(parents=True)
    with gzip.open(output / "road_visits_v2.csv.gz","wt",encoding="utf-8",newline="") as f:
        writer = csv.DictWriter(f, fields, extrasaction="ignore")
        writer.writeheader()
        for vid, event in vehicles.items():
            for index in range(len(event["route"])):
                row, feature = row_for(vid,index), dict(feature_rows[vid,index])
                valid = row["valid"] and row["touched"]
                observed = feature.get("feature_valid") == "1"
                lag = None
                if observed and row["enter"] is not None and feature.get("observed_enter_time"):
                    lag = float(feature["observed_enter_time"])-row["enter"]
                feature.update({"v1_"+name:feature.get(name, "") for name in LABELS})
                feature.update(zip(LABELS, row["labels"] if valid else [""]*len(LABELS)))
                feature.update(schema="road-time-native-euler-boundary-v2-research", travel_time_label=row["labels"][0] if valid else "",
                               v2_label_valid=int(valid), label_valid=int(valid), trainable=0,
                               v2_quality="estimated_euler_geometry" if valid else ";".join(sorted(row["reasons"])) or "unobserved_interval",
                               v2_partial_seconds=row["seconds"], v2_enter=row["enter"], v2_exit=row["exit"],
                               feature_snapshot_lag_seconds=lag,
                               v2_trainable_observed_snapshot=int(valid and observed), v2_exact_entry_features_known=0)
                writer.writerow(feature)
                counts["valid_v2_labels"] += int(valid)
                counts["candidate_snapshot_trainable"] += int(valid and observed)
                counts["valid_labels_without_features"] += int(valid and not observed)
                counts["v1_valid_labels"] += int(bool(feature.get("v1_"+LABELS[0])))
    report = dict(status="RECONCILED_RESEARCH_NOT_RELEASED", counts=dict(counts), seconds=dict(audit_totals),
                  tripinfo_total_duration=total_duration, global_time_residual=total_duration-accounted,
                  feature_policy="No fabricated flow/lane/ABC. trainable=0 until snapshot-anchor policy approval; candidate flag is not training release.",
                  limitation="no-internal Euler only; route-index labels reconstructed, dynamic features for fully skipped occurrences remain unknown",
                  inputs_sha256={str(Path(x).resolve()):digest(x) for x in (net,trace_path,trip_path,feature_path)},
                  analysis_source_sha256={str(x.resolve()):digest(x) for x in Path(__file__).parent.glob("*.py")},
                  failures=failures, native_arrival_acceptance=arrival_acceptance)
    (output / "audit.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    (output / "vehicle_budgets.json").write_text(json.dumps(per_vehicle,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--net-file", required=True)
    p.add_argument("--probe-dir", required=True)
    p.add_argument("--output-dir", required=True)
    args=p.parse_args(argv)
    root=Path(args.probe_dir)
    audit=json.loads((root/"audit.json").read_text(encoding="utf-8"))
    if audit.get("status")!="COMPLETE" or not audit.get("native_closed"):
        raise ValueError("only closed complete probe files may be reconciled")
    if digest(root/"manifest.json") != audit.get("manifest_sha256"):
        raise ValueError("manifest hash mismatch")
    manifest=json.loads((root/"manifest.json").read_text(encoding="utf-8"))
    if digest(args.net_file) != manifest.get("network_sha256"):
        raise ValueError("network must be identical to the observed network")
    hashes=audit.get("closed_outputs_sha256",{})
    if not all(name in hashes for name in ("trace.jsonl.gz","tripinfo.xml","snapshot_feature_rows.csv.gz")):
        raise ValueError("missing closed evidence binding")
    for name,expected in hashes.items():
        if Path(name).name != name or digest(root/name) != expected:
            raise ValueError("closed evidence path/hash mismatch")
    report=reconcile(args.net_file,root/"trace.jsonl.gz",root/"tripinfo.xml",root/"snapshot_feature_rows.csv.gz",args.output_dir)
    print(json.dumps({k:report[k] for k in ("status","counts","seconds","global_time_residual")},ensure_ascii=False))


if __name__ == "__main__":
    main()
