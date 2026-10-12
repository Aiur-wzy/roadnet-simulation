#!/usr/bin/env python3
"""Collect per-vehicle green-window usage from a SUMO TraCI run.

For each signalized movement, one green round is the green window in which the
vehicle finally crosses the stop line. Every eligible green window that ends
while the vehicle remains queued adds one missed round. Therefore:

    total_green_rounds = signalized_movements + missed_green_rounds

The collector intentionally queries only fields needed for this statistic. It
is substantially lighter than the full CAMS training/event extractor.
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics
import sys
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple


sumo_home = os.environ.get("SUMO_HOME")
if sumo_home:
    tools_path = os.path.join(sumo_home, "tools")
    if tools_path not in sys.path:
        sys.path.append(tools_path)

try:
    import traci
except ModuleNotFoundError as exc:
    raise SystemExit(
        "traci module not found; set SUMO_HOME to the SUMO installation directory"
    ) from exc


LinkKey = Tuple[str, int]
CycleToken = Tuple[str, int, int]


@dataclass
class Observation:
    road_id: str
    tls_id: str
    link_index: int
    distance: float
    signal_state: str
    speed: Optional[float]

    @property
    def link_key(self) -> LinkKey:
        return self.tls_id, self.link_index


@dataclass
class Approach:
    vehicle_id: str
    from_edge: str
    to_edge: str
    route_index: int
    tls_id: str
    link_index: int
    first_seen_time: float
    waiting_region_enter_time: Optional[float] = None
    last_distance: Optional[float] = None
    last_speed: Optional[float] = None
    queued_at_green_start: Set[CycleToken] = field(default_factory=set)
    green_exposure: Dict[CycleToken, float] = field(default_factory=dict)
    missed_cycle_ids: List[CycleToken] = field(default_factory=list)
    missed_cycle_set: Set[CycleToken] = field(default_factory=set)

    @property
    def link_key(self) -> LinkKey:
        return self.tls_id, self.link_index


@dataclass
class VehicleTotals:
    vehicle_id: str
    depart_time: float
    arrival_time: Optional[float] = None
    signalized_movements: int = 0
    missed_green_rounds: int = 0
    movements_with_missed_green: int = 0
    max_green_rounds_at_movement: int = 0
    max_missed_green_rounds_at_movement: int = 0

    @property
    def successful_green_rounds(self) -> int:
        return self.signalized_movements

    @property
    def total_green_rounds(self) -> int:
        return self.signalized_movements + self.missed_green_rounds


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo-config", required=True)
    parser.add_argument("--sumo-binary", default="sumo")
    parser.add_argument("--tripinfo-output", required=True)
    parser.add_argument("--vehicle-output", required=True)
    parser.add_argument("--event-output", required=True)
    parser.add_argument("--distribution-output", required=True)
    parser.add_argument("--summary-output", required=True)
    parser.add_argument(
        "--waiting-region-length",
        type=float,
        default=None,
        help=(
            "distance before the stop line in which queued vehicles are tracked; "
            "the default tracks the whole approach"
        ),
    )
    parser.add_argument("--queue-speed-threshold", type=float, default=2.0)
    parser.add_argument("--min-green-opportunity-time", type=float, default=2.0)
    return parser.parse_args()


def is_green(state: str) -> bool:
    return state in {"G", "g"}


def ensure_parent(paths: Iterable[str]) -> None:
    for raw_path in paths:
        Path(raw_path).resolve().parent.mkdir(parents=True, exist_ok=True)


def percentile(values: List[int], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def format_token(token: CycleToken) -> str:
    return f"{token[0]}:{token[1]}:{token[2]}"


def in_waiting_region(distance: float, waiting_region_length: Optional[float]) -> bool:
    return waiting_region_length is None or distance <= waiting_region_length


def get_observation(
    vehicle_id: str,
    waiting_region_length: Optional[float],
) -> Optional[Observation]:
    road_id = traci.vehicle.getRoadID(vehicle_id)
    if not road_id or road_id.startswith(":"):
        return None
    next_tls = traci.vehicle.getNextTLS(vehicle_id)
    if not next_tls:
        return None
    tls_id, link_index, distance, state = next_tls[0]
    distance = float(distance)
    speed = (
        traci.vehicle.getSpeed(vehicle_id)
        if in_waiting_region(distance, waiting_region_length)
        else None
    )
    return Observation(
        road_id=road_id,
        tls_id=str(tls_id),
        link_index=int(link_index),
        distance=distance,
        signal_state=str(state),
        speed=speed,
    )


def new_approach(vehicle_id: str, observation: Observation, current_time: float) -> Approach:
    route = traci.vehicle.getRoute(vehicle_id)
    route_index = traci.vehicle.getRouteIndex(vehicle_id)
    from_edge = observation.road_id
    to_edge = route[route_index + 1] if 0 <= route_index < len(route) - 1 else ""
    return Approach(
        vehicle_id=vehicle_id,
        from_edge=from_edge,
        to_edge=to_edge,
        route_index=route_index,
        tls_id=observation.tls_id,
        link_index=observation.link_index,
        first_seen_time=current_time,
    )


def write_event(
    writer: csv.DictWriter,
    approach: Approach,
    pass_time: float,
    totals: VehicleTotals,
) -> None:
    missed = len(approach.missed_cycle_ids)
    green_rounds = missed + 1
    totals.signalized_movements += 1
    totals.missed_green_rounds += missed
    totals.movements_with_missed_green += int(missed > 0)
    totals.max_green_rounds_at_movement = max(
        totals.max_green_rounds_at_movement, green_rounds
    )
    totals.max_missed_green_rounds_at_movement = max(
        totals.max_missed_green_rounds_at_movement, missed
    )
    writer.writerow(
        {
            "vehicle_id": approach.vehicle_id,
            "route_index": approach.route_index,
            "from_edge": approach.from_edge,
            "to_edge": approach.to_edge,
            "tls_id": approach.tls_id,
            "link_index": approach.link_index,
            "first_seen_time": round(approach.first_seen_time, 2),
            "waiting_region_enter_time": (
                round(approach.waiting_region_enter_time, 2)
                if approach.waiting_region_enter_time is not None
                else ""
            ),
            "pass_time": round(pass_time, 2),
            "missed_green_rounds": missed,
            "successful_green_rounds": 1,
            "total_green_rounds": green_rounds,
            "missed_green_cycle_ids": ";".join(
                format_token(token) for token in approach.missed_cycle_ids
            ),
        }
    )


def write_vehicle_rows(output_path: str, totals: Dict[str, VehicleTotals]) -> None:
    fieldnames = [
        "vehicle_id",
        "depart_time",
        "arrival_time",
        "trip_duration",
        "signalized_movements",
        "successful_green_rounds",
        "missed_green_rounds",
        "total_green_rounds",
        "movements_with_missed_green",
        "max_green_rounds_at_movement",
        "max_missed_green_rounds_at_movement",
    ]
    with open(output_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for vehicle_id in sorted(totals):
            row = totals[vehicle_id]
            duration = (
                row.arrival_time - row.depart_time
                if row.arrival_time is not None
                else None
            )
            writer.writerow(
                {
                    "vehicle_id": vehicle_id,
                    "depart_time": round(row.depart_time, 2),
                    "arrival_time": (
                        round(row.arrival_time, 2)
                        if row.arrival_time is not None
                        else ""
                    ),
                    "trip_duration": round(duration, 2) if duration is not None else "",
                    "signalized_movements": row.signalized_movements,
                    "successful_green_rounds": row.successful_green_rounds,
                    "missed_green_rounds": row.missed_green_rounds,
                    "total_green_rounds": row.total_green_rounds,
                    "movements_with_missed_green": row.movements_with_missed_green,
                    "max_green_rounds_at_movement": row.max_green_rounds_at_movement,
                    "max_missed_green_rounds_at_movement": row.max_missed_green_rounds_at_movement,
                }
            )


def write_distribution(output_path: str, totals: Dict[str, VehicleTotals]) -> None:
    distributions = {
        "total_green_rounds": Counter(
            row.total_green_rounds for row in totals.values()
        ),
        "missed_green_rounds": Counter(
            row.missed_green_rounds for row in totals.values()
        ),
        "max_missed_green_rounds_at_movement": Counter(
            row.max_missed_green_rounds_at_movement for row in totals.values()
        ),
        "signalized_movements": Counter(
            row.signalized_movements for row in totals.values()
        ),
    }
    vehicle_count = len(totals)
    with open(output_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle, fieldnames=["metric", "value", "vehicle_count", "vehicle_share"]
        )
        writer.writeheader()
        for metric, counts in distributions.items():
            for value in sorted(counts):
                count = counts[value]
                writer.writerow(
                    {
                        "metric": metric,
                        "value": value,
                        "vehicle_count": count,
                        "vehicle_share": (
                            round(count / vehicle_count, 8) if vehicle_count else 0.0
                        ),
                    }
                )


def write_summary(output_path: str, totals: Dict[str, VehicleTotals], event_count: int) -> None:
    total_rounds = [row.total_green_rounds for row in totals.values()]
    missed_rounds = [row.missed_green_rounds for row in totals.values()]
    max_missed_at_movement = [
        row.max_missed_green_rounds_at_movement for row in totals.values()
    ]
    movements = [row.signalized_movements for row in totals.values()]
    vehicles = len(totals)
    values = {
        "vehicles": vehicles,
        "movement_events": event_count,
        "mean_signalized_movements": statistics.fmean(movements) if movements else 0.0,
        "mean_total_green_rounds": statistics.fmean(total_rounds) if total_rounds else 0.0,
        "median_total_green_rounds": statistics.median(total_rounds) if total_rounds else 0.0,
        "p90_total_green_rounds": percentile(total_rounds, 0.90),
        "p95_total_green_rounds": percentile(total_rounds, 0.95),
        "p99_total_green_rounds": percentile(total_rounds, 0.99),
        "max_total_green_rounds": max(total_rounds, default=0),
        "mean_missed_green_rounds": statistics.fmean(missed_rounds) if missed_rounds else 0.0,
        "median_missed_green_rounds": statistics.median(missed_rounds) if missed_rounds else 0.0,
        "p90_missed_green_rounds": percentile(missed_rounds, 0.90),
        "p95_missed_green_rounds": percentile(missed_rounds, 0.95),
        "p99_missed_green_rounds": percentile(missed_rounds, 0.99),
        "max_missed_green_rounds": max(missed_rounds, default=0),
        "vehicles_with_missed_green": sum(value > 0 for value in missed_rounds),
        "share_with_missed_green": (
            sum(value > 0 for value in missed_rounds) / vehicles if vehicles else 0.0
        ),
        "vehicles_with_2plus_missed_green": sum(value >= 2 for value in missed_rounds),
        "share_with_2plus_missed_green": (
            sum(value >= 2 for value in missed_rounds) / vehicles if vehicles else 0.0
        ),
        "vehicles_with_3plus_missed_green": sum(value >= 3 for value in missed_rounds),
        "share_with_3plus_missed_green": (
            sum(value >= 3 for value in missed_rounds) / vehicles if vehicles else 0.0
        ),
        "vehicles_with_4plus_missed_green": sum(value >= 4 for value in missed_rounds),
        "share_with_4plus_missed_green": (
            sum(value >= 4 for value in missed_rounds) / vehicles if vehicles else 0.0
        ),
        "vehicles_with_5plus_missed_green": sum(value >= 5 for value in missed_rounds),
        "share_with_5plus_missed_green": (
            sum(value >= 5 for value in missed_rounds) / vehicles if vehicles else 0.0
        ),
        "vehicles_with_10plus_missed_green": sum(value >= 10 for value in missed_rounds),
        "vehicles_with_50plus_missed_green": sum(value >= 50 for value in missed_rounds),
        "max_missed_green_rounds_at_one_movement": max(
            max_missed_at_movement, default=0
        ),
    }
    with open(output_path, "w", encoding="utf-8", newline="\n") as handle:
        for key, value in values.items():
            handle.write(f"{key}={value}\n")


def main() -> None:
    args = parse_args()
    ensure_parent(
        [
            args.tripinfo_output,
            args.vehicle_output,
            args.event_output,
            args.distribution_output,
            args.summary_output,
        ]
    )
    command = [
        args.sumo_binary,
        "-c",
        args.sumo_config,
        "--tripinfo-output",
        args.tripinfo_output,
        "--no-step-log",
        "true",
    ]
    traci.start(command)

    active: Dict[str, Approach] = {}
    totals: Dict[str, VehicleTotals] = {}
    previous_green: Dict[LinkKey, bool] = {}
    cycle_counters: Dict[LinkKey, int] = {}
    current_tokens: Dict[LinkKey, CycleToken] = {}
    last_time = 0.0
    event_count = 0
    tls_ids = tuple(traci.trafficlight.getIDList())
    event_fields = [
        "vehicle_id",
        "route_index",
        "from_edge",
        "to_edge",
        "tls_id",
        "link_index",
        "first_seen_time",
        "waiting_region_enter_time",
        "pass_time",
        "missed_green_rounds",
        "successful_green_rounds",
        "total_green_rounds",
        "missed_green_cycle_ids",
    ]

    try:
        with open(args.event_output, "w", newline="", encoding="utf-8") as event_handle:
            event_writer = csv.DictWriter(event_handle, fieldnames=event_fields)
            event_writer.writeheader()

            while traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                current_time = float(traci.simulation.getTime())
                delta = max(0.0, current_time - last_time)
                last_time = current_time

                for vehicle_id in traci.simulation.getDepartedIDList():
                    totals.setdefault(
                        vehicle_id,
                        VehicleTotals(vehicle_id=vehicle_id, depart_time=current_time),
                    )
                for vehicle_id in traci.simulation.getArrivedIDList():
                    if vehicle_id in totals:
                        totals[vehicle_id].arrival_time = current_time

                started_tokens: Dict[LinkKey, CycleToken] = {}
                ended_tokens: Dict[LinkKey, CycleToken] = {}
                for tls_id in tls_ids:
                    state = traci.trafficlight.getRedYellowGreenState(tls_id)
                    for link_index, signal_state in enumerate(state):
                        key = (tls_id, link_index)
                        green = is_green(signal_state)
                        was_green = previous_green.get(key, False)
                        if green and not was_green:
                            cycle_id = cycle_counters.get(key, 0) + 1
                            cycle_counters[key] = cycle_id
                            token = (tls_id, link_index, cycle_id)
                            current_tokens[key] = token
                            started_tokens[key] = token
                        elif was_green and not green:
                            token = current_tokens.pop(key, None)
                            if token is not None:
                                ended_tokens[key] = token
                        previous_green[key] = green

                vehicle_ids = set(traci.vehicle.getIDList())
                observations: Dict[str, Observation] = {}
                for vehicle_id in vehicle_ids:
                    observation = get_observation(
                        vehicle_id, args.waiting_region_length
                    )
                    if observation is not None:
                        observations[vehicle_id] = observation

                for vehicle_id, approach in list(active.items()):
                    observation = observations.get(vehicle_id)
                    current_road = (
                        observation.road_id
                        if observation is not None
                        else (
                            traci.vehicle.getRoadID(vehicle_id)
                            if vehicle_id in vehicle_ids
                            else ""
                        )
                    )
                    passed = (
                        vehicle_id not in vehicle_ids
                        or not current_road
                        or current_road.startswith(":")
                        or current_road != approach.from_edge
                    )
                    if passed:
                        totals.setdefault(
                            vehicle_id,
                            VehicleTotals(vehicle_id=vehicle_id, depart_time=current_time),
                        )
                        write_event(
                            event_writer, approach, current_time, totals[vehicle_id]
                        )
                        event_count += 1
                        del active[vehicle_id]

                for vehicle_id, observation in observations.items():
                    approach = active.get(vehicle_id)
                    if approach is None:
                        approach = new_approach(vehicle_id, observation, current_time)
                        active[vehicle_id] = approach
                    elif (
                        approach.tls_id != observation.tls_id
                        or approach.from_edge != observation.road_id
                    ):
                        write_event(
                            event_writer, approach, current_time, totals[vehicle_id]
                        )
                        event_count += 1
                        approach = new_approach(vehicle_id, observation, current_time)
                        active[vehicle_id] = approach
                    elif approach.link_index != observation.link_index:
                        # Lane changes may select an equivalent controlled link.
                        approach.link_index = observation.link_index

                    approach.last_distance = observation.distance
                    approach.last_speed = observation.speed
                    inside_waiting_region = in_waiting_region(
                        observation.distance, args.waiting_region_length
                    )
                    queued = (
                        inside_waiting_region
                        and observation.speed is not None
                        and observation.speed <= args.queue_speed_threshold
                    )
                    if (
                        inside_waiting_region
                        and approach.waiting_region_enter_time is None
                    ):
                        approach.waiting_region_enter_time = current_time

                    token = current_tokens.get(observation.link_key)
                    if token is not None and inside_waiting_region:
                        approach.green_exposure[token] = (
                            approach.green_exposure.get(token, 0.0) + delta
                        )
                    start_token = started_tokens.get(observation.link_key)
                    if start_token is not None and queued:
                        approach.queued_at_green_start.add(start_token)

                for approach in active.values():
                    token = ended_tokens.get(approach.link_key)
                    if token is None or token in approach.missed_cycle_set:
                        continue
                    queued = (
                        approach.last_distance is not None
                        and in_waiting_region(
                            approach.last_distance, args.waiting_region_length
                        )
                        and approach.last_speed is not None
                        and approach.last_speed <= args.queue_speed_threshold
                    )
                    eligible = (
                        token in approach.queued_at_green_start
                        or approach.green_exposure.get(token, 0.0)
                        >= args.min_green_opportunity_time
                    )
                    if queued and eligible:
                        approach.missed_cycle_ids.append(token)
                        approach.missed_cycle_set.add(token)

            current_time = float(traci.simulation.getTime())
            for vehicle_id, approach in list(active.items()):
                totals.setdefault(
                    vehicle_id,
                    VehicleTotals(vehicle_id=vehicle_id, depart_time=current_time),
                )
                write_event(event_writer, approach, current_time, totals[vehicle_id])
                event_count += 1
    finally:
        traci.close()

    write_vehicle_rows(args.vehicle_output, totals)
    write_distribution(args.distribution_output, totals)
    write_summary(args.summary_output, totals, event_count)
    print(f"vehicles={len(totals)}")
    print(f"movement_events={event_count}")
    print(f"vehicle_output={args.vehicle_output}")
    print(f"event_output={args.event_output}")
    print(f"distribution_output={args.distribution_output}")
    print(f"summary_output={args.summary_output}")


if __name__ == "__main__":
    main()
