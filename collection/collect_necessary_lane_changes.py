#!/usr/bin/env python3
"""Trace route-required lane changes in a SUMO simulation via TraCI.

For a route triple ``previous_edge -> middle_edge -> next_edge``, the previous
movement constrains which lanes a vehicle may enter on ``middle_edge`` and the
next movement constrains which lanes may leave it.  This tool records cases in
which those lane sets differ, then observes whether and where the vehicle moves
into a lane accepted by the next movement.  The ``entry_*`` fields are the
first TraCI sample observed on the middle edge, not a zero-distance detector.
When disjoint lane sets imply a required change that is already complete in
that first sample, its position is explicitly marked as an upper bound.

The normalized position fields use 0 at the start of the middle edge and 1 at
its downstream stop line.  A vehicle is considered to require a lane change
when its actually observed entry lane is not in the next movement's from-lane
set.  The tool observes SUMO; it never calls ``changeLane``.
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


def import_traci():
    """Import TraCI from Python or from ``SUMO_HOME/tools``."""

    try:
        import traci  # type: ignore
        import traci.constants as tc  # type: ignore

        return traci, tc
    except ModuleNotFoundError:
        sumo_home = os.environ.get("SUMO_HOME")
        if not sumo_home:
            raise SystemExit(
                "TraCI is unavailable. Install the SUMO Python tools or set SUMO_HOME."
            )
        tools_path = os.path.join(sumo_home, "tools")
        if tools_path not in sys.path:
            sys.path.append(tools_path)
        import traci  # type: ignore
        import traci.constants as tc  # type: ignore

        return traci, tc


@dataclass(frozen=True)
class NetworkInfo:
    lanes_by_edge_index: Dict[str, Dict[int, str]]
    lane_lengths: Dict[str, float]
    movement_lanes: Dict[Tuple[str, str], Tuple[Tuple[str, ...], Tuple[str, ...]]]


@dataclass
class RequirementRecord:
    vehicle_id: str
    route_id: str
    route_index: int
    previous_edge: str
    middle_edge: str
    next_edge: str
    previous_to_lanes: str
    next_from_lanes: str
    lane_set_relation: str
    middle_edge_enter_time: float
    entry_lane_id: str
    entry_lane_index: int
    entry_position_m: float
    entry_position_fraction: float
    change_required_at_entry: int
    required_lane_reached: int = 0
    required_lane_reach_time: Optional[float] = None
    required_lane_id: str = ""
    required_lane_index: Optional[int] = None
    required_lane_position_m: Optional[float] = None
    required_lane_position_fraction: Optional[float] = None
    remaining_fraction_at_required_lane: Optional[float] = None
    completion_position_is_upper_bound: int = 0
    observed_lane_change_count: int = 0
    lane_path: str = ""
    middle_edge_exit_time: Optional[float] = None
    middle_edge_travel_time: Optional[float] = None
    status: str = "active"


@dataclass
class LaneChangeEvent:
    vehicle_id: str
    route_id: str
    route_index: int
    previous_edge: str
    middle_edge: str
    next_edge: str
    time: float
    from_lane_id: str
    from_lane_index: int
    to_lane_id: str
    to_lane_index: int
    position_m: float
    position_fraction: float
    remaining_fraction: float
    completes_route_requirement: int


@dataclass
class ActiveTracker:
    record: RequirementRecord
    required_lanes: Tuple[str, ...]
    last_lane_id: str
    last_lane_index: int
    last_position_m: float
    lane_path_parts: List[str]


def lane_index(lane_id: str) -> int:
    try:
        return int(lane_id.rsplit("_", 1)[1])
    except (IndexError, ValueError):
        return -1


def parse_network(net_file: str) -> NetworkInfo:
    root = ET.parse(net_file).getroot()
    lanes_by_edge_index: Dict[str, Dict[int, str]] = {}
    lane_lengths: Dict[str, float] = {}

    for edge in root.findall("edge"):
        edge_id = edge.attrib.get("id", "")
        if not edge_id or edge_id.startswith(":") or "function" in edge.attrib:
            continue
        indexed_lanes: Dict[int, str] = {}
        for lane in edge.findall("lane"):
            lane_id = lane.attrib["id"]
            index = lane_index(lane_id)
            if index >= 0:
                indexed_lanes[index] = lane_id
            lane_lengths[lane_id] = float(lane.attrib.get("length", "0"))
        lanes_by_edge_index[edge_id] = indexed_lanes

    connection_indices: Dict[Tuple[str, str], List[Tuple[int, int]]] = defaultdict(list)
    for connection in root.findall("connection"):
        from_edge = connection.attrib.get("from", "")
        to_edge = connection.attrib.get("to", "")
        if from_edge.startswith(":") or to_edge.startswith(":"):
            continue
        try:
            from_index = int(connection.attrib["fromLane"])
            to_index = int(connection.attrib["toLane"])
        except (KeyError, ValueError):
            continue
        connection_indices[(from_edge, to_edge)].append((from_index, to_index))

    movement_lanes: Dict[Tuple[str, str], Tuple[Tuple[str, ...], Tuple[str, ...]]] = {}
    for (from_edge, to_edge), pairs in connection_indices.items():
        from_map = lanes_by_edge_index.get(from_edge, {})
        to_map = lanes_by_edge_index.get(to_edge, {})
        from_lanes = tuple(sorted({from_map[i] for i, _ in pairs if i in from_map}))
        to_lanes = tuple(sorted({to_map[i] for _, i in pairs if i in to_map}))
        movement_lanes[(from_edge, to_edge)] = (from_lanes, to_lanes)

    return NetworkInfo(lanes_by_edge_index, lane_lengths, movement_lanes)


def position_fraction(position_m: float, lane_id: str, network: NetworkInfo) -> float:
    length = network.lane_lengths.get(lane_id, 0.0)
    if length <= 0:
        return 0.0
    return min(1.0, max(0.0, position_m / length))


def percentile(values: Sequence[float], probability: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = probability * (len(ordered) - 1)
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def format_float(value: Optional[float]) -> object:
    return "" if value is None else round(value, 6)


def finish_tracker(tracker: ActiveTracker, current_time: float, status: str) -> None:
    record = tracker.record
    record.middle_edge_exit_time = current_time
    record.middle_edge_travel_time = current_time - record.middle_edge_enter_time
    record.lane_path = ">".join(tracker.lane_path_parts)
    if record.change_required_at_entry:
        record.status = "completed" if record.required_lane_reached else status
    else:
        record.status = "aligned_on_entry"


def write_csv(path: str, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def requirement_rows(records: Sequence[RequirementRecord]) -> Iterable[Dict[str, object]]:
    for record in records:
        row = asdict(record)
        for name in (
            "middle_edge_enter_time",
            "entry_position_m",
            "entry_position_fraction",
            "required_lane_reach_time",
            "required_lane_position_m",
            "required_lane_position_fraction",
            "remaining_fraction_at_required_lane",
            "middle_edge_exit_time",
            "middle_edge_travel_time",
        ):
            row[name] = format_float(row[name])
        yield row


def event_rows(events: Sequence[LaneChangeEvent]) -> Iterable[Dict[str, object]]:
    for event in events:
        row = asdict(event)
        for name in ("time", "position_m", "position_fraction", "remaining_fraction"):
            row[name] = format_float(row[name])
        yield row


def build_transition_summary(records: Sequence[RequirementRecord]) -> List[Dict[str, object]]:
    grouped: Dict[Tuple[str, ...], List[RequirementRecord]] = defaultdict(list)
    for record in records:
        key = (
            record.previous_edge,
            record.middle_edge,
            record.next_edge,
            record.previous_to_lanes,
            record.next_from_lanes,
            record.lane_set_relation,
        )
        grouped[key].append(record)

    rows: List[Dict[str, object]] = []
    for key, group in sorted(grouped.items()):
        required = [record for record in group if record.change_required_at_entry]
        completed = [record for record in required if record.required_lane_reached]
        upper_bounds = [
            record for record in completed if record.completion_position_is_upper_bound
        ]
        fractions = [
            record.required_lane_position_fraction
            for record in completed
            if record.required_lane_position_fraction is not None
        ]
        entry_lanes = Counter(record.entry_lane_id for record in group)
        statuses = Counter(record.status for record in group)
        rows.append(
            {
                "previous_edge": key[0],
                "middle_edge": key[1],
                "next_edge": key[2],
                "previous_to_lanes": key[3],
                "next_from_lanes": key[4],
                "lane_set_relation": key[5],
                "vehicle_count": len(group),
                "aligned_on_entry_count": len(group) - len(required),
                "change_required_count": len(required),
                "required_lane_reached_count": len(completed),
                "completion_observed_count": len(completed) - len(upper_bounds),
                "completion_position_upper_bound_count": len(upper_bounds),
                "completion_rate": round(len(completed) / len(required), 6) if required else 1.0,
                "mean_completion_fraction": format_float(statistics.fmean(fractions) if fractions else None),
                "p10_completion_fraction": format_float(percentile(fractions, 0.10)),
                "p25_completion_fraction": format_float(percentile(fractions, 0.25)),
                "p50_completion_fraction": format_float(percentile(fractions, 0.50)),
                "p75_completion_fraction": format_float(percentile(fractions, 0.75)),
                "p90_completion_fraction": format_float(percentile(fractions, 0.90)),
                "mean_observed_lane_changes": round(
                    statistics.fmean(record.observed_lane_change_count for record in group), 6
                ),
                "entry_lane_counts": "|".join(f"{lane}:{count}" for lane, count in sorted(entry_lanes.items())),
                "status_counts": "|".join(f"{status}:{count}" for status, count in sorted(statuses.items())),
            }
        )
    return rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Observe where SUMO performs route-required lane changes."
    )
    parser.add_argument("--sumo-config", required=True, help="SUMO .sumocfg file")
    parser.add_argument("--net-file", required=True, help="SUMO .net.xml file")
    parser.add_argument("--sumo-binary", default="sumo", help="sumo or sumo-gui")
    parser.add_argument(
        "--requirement-output",
        default="necessary_lane_change_requirements.csv",
        help="One row per vehicle and mismatched movement-lane transition",
    )
    parser.add_argument(
        "--event-output",
        default="necessary_lane_change_events.csv",
        help="One row per observed lane change on a tracked middle edge",
    )
    parser.add_argument(
        "--summary-output",
        default="necessary_lane_change_summary.csv",
        help="Aggregated completion-position distribution per edge triple",
    )
    parser.add_argument(
        "--tripinfo-output",
        default=None,
        help="Optional SUMO tripinfo override, useful for avoiding changes to the configured truth file",
    )
    parser.add_argument(
        "--step-length",
        type=float,
        default=None,
        help="Optional SUMO step length. Omit to preserve the truth configuration.",
    )
    return parser.parse_args()


def run(args: argparse.Namespace) -> None:
    traci, tc = import_traci()
    network = parse_network(args.net_file)
    # Explicitly override the config network so TraCI and the XML lane mapping
    # always describe the same file.  This also enables paired old/new-network
    # controls with an unchanged route file and random seed.
    sumo_command = [
        args.sumo_binary,
        "-c",
        args.sumo_config,
        "--net-file",
        args.net_file,
    ]
    if args.tripinfo_output:
        Path(args.tripinfo_output).parent.mkdir(parents=True, exist_ok=True)
        sumo_command.extend(["--tripinfo-output", args.tripinfo_output])
    if args.step_length is not None:
        sumo_command.extend(["--step-length", str(args.step_length)])

    subscription_variables = (
        tc.VAR_ROAD_ID,
        tc.VAR_LANE_ID,
        tc.VAR_LANEPOSITION,
        tc.VAR_ROUTE_INDEX,
    )
    route_cache: Dict[str, Tuple[str, ...]] = {}
    route_id_cache: Dict[str, str] = {}
    active: Dict[Tuple[str, int], ActiveTracker] = {}
    records: List[RequirementRecord] = []
    events: List[LaneChangeEvent] = []

    traci.start(sumo_command)
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = float(traci.simulation.getTime())

            for vehicle_id in traci.simulation.getDepartedIDList():
                route_cache[vehicle_id] = tuple(traci.vehicle.getRoute(vehicle_id))
                route_id_cache[vehicle_id] = traci.vehicle.getRouteID(vehicle_id)
                traci.vehicle.subscribe(vehicle_id, subscription_variables)

            subscription_results = traci.vehicle.getAllSubscriptionResults() or {}
            observed_keys = set()

            for vehicle_id, values in subscription_results.items():
                road_id = values.get(tc.VAR_ROAD_ID, "")
                lane_id = values.get(tc.VAR_LANE_ID, "")
                route_index = int(values.get(tc.VAR_ROUTE_INDEX, -1))
                position_m = float(values.get(tc.VAR_LANEPOSITION, 0.0))
                route = route_cache.get(vehicle_id)
                if route is None:
                    route = tuple(traci.vehicle.getRoute(vehicle_id))
                    route_cache[vehicle_id] = route
                    route_id_cache[vehicle_id] = traci.vehicle.getRouteID(vehicle_id)

                # Leaving a tracked normal edge (including entry into an internal
                # junction edge) ends the opportunity for this lane change.
                for key, tracker in list(active.items()):
                    if key[0] == vehicle_id and road_id != tracker.record.middle_edge:
                        finish_tracker(tracker, current_time, "exited_without_observed_target_lane")
                        records.append(tracker.record)
                        del active[key]

                if (
                    road_id.startswith(":")
                    or not lane_id
                    or route_index <= 0
                    or route_index >= len(route) - 1
                    or road_id != route[route_index]
                ):
                    continue

                previous_edge = route[route_index - 1]
                middle_edge = route[route_index]
                next_edge = route[route_index + 1]
                previous_movement = network.movement_lanes.get((previous_edge, middle_edge))
                next_movement = network.movement_lanes.get((middle_edge, next_edge))
                if previous_movement is None or next_movement is None:
                    continue

                previous_to_lanes = previous_movement[1]
                next_from_lanes = next_movement[0]
                if not previous_to_lanes or not next_from_lanes:
                    continue
                if set(previous_to_lanes) == set(next_from_lanes):
                    continue

                key = (vehicle_id, route_index)
                observed_keys.add(key)
                fraction = position_fraction(position_m, lane_id, network)
                if key not in active:
                    relation = (
                        "overlap_not_equal"
                        if set(previous_to_lanes).intersection(next_from_lanes)
                        else "disjoint"
                    )
                    # Disjoint movement lane sets prove that a lane change is
                    # necessary even when it completed before the first normal-
                    # edge TraCI sample.  For partially overlapping sets, use
                    # the actually observed lane to decide necessity.
                    change_required = relation == "disjoint" or lane_id not in next_from_lanes
                    record = RequirementRecord(
                        vehicle_id=vehicle_id,
                        route_id=route_id_cache.get(vehicle_id, ""),
                        route_index=route_index,
                        previous_edge=previous_edge,
                        middle_edge=middle_edge,
                        next_edge=next_edge,
                        previous_to_lanes="|".join(previous_to_lanes),
                        next_from_lanes="|".join(next_from_lanes),
                        lane_set_relation=relation,
                        middle_edge_enter_time=current_time,
                        entry_lane_id=lane_id,
                        entry_lane_index=lane_index(lane_id),
                        entry_position_m=position_m,
                        entry_position_fraction=fraction,
                        change_required_at_entry=1 if change_required else 0,
                    )
                    tracker = ActiveTracker(
                        record=record,
                        required_lanes=next_from_lanes,
                        last_lane_id=lane_id,
                        last_lane_index=lane_index(lane_id),
                        last_position_m=position_m,
                        lane_path_parts=[f"{lane_id}@{fraction:.6f}"],
                    )
                    active[key] = tracker
                    if lane_id in next_from_lanes:
                        record.required_lane_reached = 1
                        record.required_lane_reach_time = current_time
                        record.required_lane_id = lane_id
                        record.required_lane_index = lane_index(lane_id)
                        record.required_lane_position_m = position_m
                        record.required_lane_position_fraction = fraction
                        record.remaining_fraction_at_required_lane = 1.0 - fraction
                        record.completion_position_is_upper_bound = 1 if change_required else 0

                tracker = active[key]
                if lane_id != tracker.last_lane_id:
                    completes_requirement = (
                        tracker.record.change_required_at_entry
                        and not tracker.record.required_lane_reached
                        and lane_id in tracker.required_lanes
                    )
                    events.append(
                        LaneChangeEvent(
                            vehicle_id=vehicle_id,
                            route_id=tracker.record.route_id,
                            route_index=route_index,
                            previous_edge=previous_edge,
                            middle_edge=middle_edge,
                            next_edge=next_edge,
                            time=current_time,
                            from_lane_id=tracker.last_lane_id,
                            from_lane_index=tracker.last_lane_index,
                            to_lane_id=lane_id,
                            to_lane_index=lane_index(lane_id),
                            position_m=position_m,
                            position_fraction=fraction,
                            remaining_fraction=1.0 - fraction,
                            completes_route_requirement=1 if completes_requirement else 0,
                        )
                    )
                    tracker.record.observed_lane_change_count += 1
                    tracker.lane_path_parts.append(f"{lane_id}@{fraction:.6f}")
                    tracker.last_lane_id = lane_id
                    tracker.last_lane_index = lane_index(lane_id)

                if (
                    tracker.record.change_required_at_entry
                    and not tracker.record.required_lane_reached
                    and lane_id in tracker.required_lanes
                ):
                    tracker.record.required_lane_reached = 1
                    tracker.record.required_lane_reach_time = current_time
                    tracker.record.required_lane_id = lane_id
                    tracker.record.required_lane_index = lane_index(lane_id)
                    tracker.record.required_lane_position_m = position_m
                    tracker.record.required_lane_position_fraction = fraction
                    tracker.record.remaining_fraction_at_required_lane = 1.0 - fraction

                tracker.last_position_m = position_m

            current_vehicle_ids = set(subscription_results)
            for key, tracker in list(active.items()):
                if key not in observed_keys and key[0] not in current_vehicle_ids:
                    finish_tracker(tracker, current_time, "vehicle_left_simulation")
                    records.append(tracker.record)
                    del active[key]

        final_time = float(traci.simulation.getTime())
        for tracker in active.values():
            finish_tracker(tracker, final_time, "simulation_ended_before_completion")
            records.append(tracker.record)
    finally:
        traci.close()

    requirement_fieldnames = list(RequirementRecord.__dataclass_fields__)
    event_fieldnames = list(LaneChangeEvent.__dataclass_fields__)
    summary_rows = build_transition_summary(records)
    summary_fieldnames = list(summary_rows[0]) if summary_rows else [
        "previous_edge",
        "middle_edge",
        "next_edge",
        "vehicle_count",
    ]
    write_csv(args.requirement_output, requirement_fieldnames, requirement_rows(records))
    write_csv(args.event_output, event_fieldnames, event_rows(events))
    write_csv(args.summary_output, summary_fieldnames, summary_rows)

    required = [record for record in records if record.change_required_at_entry]
    completed = [record for record in required if record.required_lane_reached]
    completion_upper_bounds = [
        record for record in completed if record.completion_position_is_upper_bound
    ]
    print(f"mismatched movement transitions: {len(records)}")
    print(f"lane changes required at entry: {len(required)}")
    print(f"required lane reached: {len(completed)}")
    print(f"completed before first middle-edge sample: {len(completion_upper_bounds)}")
    print(
        "completion rate: "
        + (f"{len(completed) / len(required):.2%}" if required else "n/a")
    )
    print(f"requirement output: {args.requirement_output}")
    print(f"lane-change event output: {args.event_output}")
    print(f"transition summary output: {args.summary_output}")


def main() -> None:
    run(parse_args())


if __name__ == "__main__":
    main()
