#!/usr/bin/env python3
"""Generate conservative, balanced SUMO training episodes.

This generator consumes the 90-OD balanced route catalog produced by
``build_balanced_training_routes.py`` and creates independent SUMO episodes.
It deliberately avoids the old pattern of sending one vehicle on every route
at the same departure instant.

Default full profile (``train-conservative-v1``):

* 16 demand levels from 600 to 3800 veh/h;
* 5 deterministic seeds per level;
* 7200 s of demand per independent episode;
* 300 s allocation blocks;
* exact/near-exact origin, destination and OD balance;
* stratified random departure times within each block;
* a 4000 veh/h hard demand guard and a static movement V/C guard.

The script uses only the Python standard library.  It writes route files,
SUMO configurations, detailed manifests, and a ready-to-run TraCI command
script.  It does not invoke SUMO itself.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import random
import re
import shlex
import shutil
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


DEFAULT_LEVELS_VPH = (
    600,
    800,
    1000,
    1200,
    1400,
    1600,
    1800,
    2000,
    2200,
    2400,
    2600,
    2800,
    3000,
    3200,
    3500,
    3800,
)

PILOT_LEVELS_VPH = (600, 1000, 1400, 1800, 2200, 2600, 3000, 3400, 3800)

PROFILE_DEFAULTS = {
    "pilot-conservative-v1": {
        "levels_vph": PILOT_LEVELS_VPH,
        "seeds_per_level": 3,
        "duration_seconds": 3600,
    },
    "train-conservative-v1": {
        "levels_vph": DEFAULT_LEVELS_VPH,
        "seeds_per_level": 5,
        "duration_seconds": 7200,
    },
}

TURN_NAMES = {
    "l": "left",
    "L": "left",
    "s": "straight",
    "r": "right",
    "R": "right",
    "t": "turnaround",
}


MovementKey = tuple[str, str]


@dataclass(frozen=True)
class EdgeInfo:
    edge_id: str
    from_node: str
    to_node: str


@dataclass(frozen=True)
class MovementInfo:
    from_edge: str
    to_edge: str
    tls_id: str
    link_index: int
    direction: str
    green_seconds: float
    cycle_seconds: float
    capacity_proxy_vph: float

    @property
    def key(self) -> MovementKey:
        return (self.from_edge, self.to_edge)


@dataclass(frozen=True)
class NetworkData:
    edges: dict[str, EdgeInfo]
    transition_pairs: frozenset[MovementKey]
    movements: dict[MovementKey, MovementInfo]
    signal_nodes: frozenset[str]
    inbound_edges: tuple[str, ...]
    outbound_edges: tuple[str, ...]


@dataclass(frozen=True)
class RouteInfo:
    route_id: str
    edges: tuple[str, ...]
    origin_edge: str
    destination_edge: str
    movement_keys: tuple[MovementKey, ...]


@dataclass(frozen=True)
class EpisodeSpec:
    episode_id: str
    level_index: int
    demand_vph: int
    seed_index: int
    seed: int
    split: str
    duration_seconds: int


@dataclass(frozen=True)
class VehiclePlan:
    depart: float
    route_id: str
    origin_edge: str
    destination_edge: str


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--net-file", required=True, type=Path, help="training SUMO .net.xml")
    parser.add_argument(
        "--route-catalog",
        required=True,
        type=Path,
        help="balanced 90-OD .rou.xml route catalog",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("train_balanced_v1"),
        help="dataset output directory (default: train_balanced_v1)",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_DEFAULTS),
        default="train-conservative-v1",
        help="built-in generation profile",
    )
    parser.add_argument(
        "--demand-levels",
        help="comma-separated veh/h levels overriding the selected profile",
    )
    parser.add_argument(
        "--seeds-per-level",
        type=int,
        help="override number of independent seeds per demand level",
    )
    parser.add_argument(
        "--duration-seconds",
        type=int,
        help="override demand duration of each independent episode",
    )
    parser.add_argument(
        "--block-seconds",
        type=int,
        default=300,
        help="balanced allocation block length (default: 300)",
    )
    parser.add_argument(
        "--seed-base",
        type=int,
        default=20260721,
        help="base used to derive reproducible episode seeds",
    )
    parser.add_argument(
        "--safety-cap-vph",
        type=float,
        default=4000.0,
        help="reject demand above this value (default: 4000)",
    )
    parser.add_argument(
        "--max-vc-proxy",
        type=float,
        default=0.65,
        help="reject an episode whose static maximum movement V/C proxy exceeds this value",
    )
    parser.add_argument(
        "--saturation-flow-vph",
        type=float,
        default=1800.0,
        help="per-movement saturation flow used only for the static safety proxy",
    )
    parser.add_argument(
        "--program-id",
        default="0",
        help="traffic-light programID used for static capacity checks (default: 0)",
    )
    parser.add_argument(
        "--lane-change-mode",
        choices=("no-change", "change"),
        default="no-change",
        help="no-change still permits route-required strategic/cooperative changes",
    )
    parser.add_argument("--depart-lane", default="best")
    parser.add_argument("--depart-speed", default="max")
    parser.add_argument("--depart-pos", default="base")
    parser.add_argument(
        "--traci-script",
        default="TraCI_Python_Adjusted.py",
        help="default TraCI collector path recorded in run_traci.sh",
    )
    parser.add_argument("--sumo-binary", default="sumo")
    parser.add_argument(
        "--traci-outputs",
        default=None,
        help=(
            "value passed to TraCI_Python_Adjusted.py --outputs "
            "(default: all for pilot, legacy for full training)"
        ),
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="validate inputs and print the dataset plan without writing files",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="overwrite files generated at the same paths; unrelated files are preserved",
    )
    args = parser.parse_args(argv)

    defaults = PROFILE_DEFAULTS[args.profile]
    args.levels_vph = (
        parse_demand_levels(args.demand_levels)
        if args.demand_levels
        else tuple(defaults["levels_vph"])
    )
    args.seeds_per_level = (
        args.seeds_per_level
        if args.seeds_per_level is not None
        else int(defaults["seeds_per_level"])
    )
    args.duration_seconds = (
        args.duration_seconds
        if args.duration_seconds is not None
        else int(defaults["duration_seconds"])
    )
    if args.traci_outputs is None:
        args.traci_outputs = "all" if args.profile == "pilot-conservative-v1" else "legacy"

    if not args.net_file.is_file():
        parser.error(f"--net-file does not exist: {args.net_file}")
    if not args.route_catalog.is_file():
        parser.error(f"--route-catalog does not exist: {args.route_catalog}")
    if args.seeds_per_level <= 0:
        parser.error("--seeds-per-level must be positive")
    if args.duration_seconds <= 0:
        parser.error("--duration-seconds must be positive")
    if args.block_seconds <= 0 or args.block_seconds > args.duration_seconds:
        parser.error("--block-seconds must be positive and no larger than the episode duration")
    if args.safety_cap_vph <= 0:
        parser.error("--safety-cap-vph must be positive")
    if args.max_vc_proxy <= 0:
        parser.error("--max-vc-proxy must be positive")
    if args.saturation_flow_vph <= 0:
        parser.error("--saturation-flow-vph must be positive")
    if max(args.levels_vph) > args.safety_cap_vph + 1e-9:
        parser.error(
            f"maximum demand {max(args.levels_vph)} veh/h exceeds safety cap "
            f"{args.safety_cap_vph:g} veh/h"
        )
    return args


def parse_demand_levels(value: str) -> tuple[int, ...]:
    levels: list[int] = []
    for raw in value.split(","):
        raw = raw.strip()
        if not raw:
            continue
        try:
            number = float(raw)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(f"invalid demand level: {raw}") from exc
        if number <= 0 or not number.is_integer():
            raise argparse.ArgumentTypeError(
                f"demand levels must be positive whole-number veh/h values: {raw}"
            )
        levels.append(int(number))
    if not levels:
        raise argparse.ArgumentTypeError("--demand-levels must contain at least one value")
    if len(set(levels)) != len(levels):
        raise argparse.ArgumentTypeError("--demand-levels contains duplicates")
    if levels != sorted(levels):
        raise argparse.ArgumentTypeError("--demand-levels must be strictly increasing")
    return tuple(levels)


def required_attr(element: ET.Element, name: str) -> str:
    value = element.get(name)
    if value is None:
        raise ValueError(f"missing required XML attribute {name!r} on <{element.tag}>")
    return value


def load_network(
    net_file: Path,
    program_id: str,
    saturation_flow_vph: float,
) -> NetworkData:
    root = ET.parse(net_file).getroot()
    edges: dict[str, EdgeInfo] = {}
    for edge in root.findall("edge"):
        if edge.get("function") is not None:
            continue
        edge_id = required_attr(edge, "id")
        edges[edge_id] = EdgeInfo(
            edge_id=edge_id,
            from_node=required_attr(edge, "from"),
            to_node=required_attr(edge, "to"),
        )

    signal_nodes = frozenset(
        required_attr(junction, "id")
        for junction in root.findall("junction")
        if junction.get("type") == "traffic_light"
    )
    if not signal_nodes:
        raise ValueError("network has no traffic-light junctions")

    programs: dict[str, tuple[float, list[tuple[float, str]]]] = {}
    for logic in root.findall("tlLogic"):
        if logic.get("programID") != program_id:
            continue
        tls_id = required_attr(logic, "id")
        phases: list[tuple[float, str]] = []
        for phase in logic.findall("phase"):
            duration = float(required_attr(phase, "duration"))
            phases.append((duration, required_attr(phase, "state")))
        if not phases:
            raise ValueError(f"tlLogic {tls_id!r} program {program_id!r} has no phases")
        programs[tls_id] = (sum(duration for duration, _ in phases), phases)

    transition_pairs: set[MovementKey] = set()
    movements: dict[MovementKey, MovementInfo] = {}
    for connection in root.findall("connection"):
        from_edge = connection.get("from")
        to_edge = connection.get("to")
        if from_edge not in edges or to_edge not in edges:
            continue
        key = (from_edge, to_edge)
        transition_pairs.add(key)
        tls_id = connection.get("tl")
        link_index_raw = connection.get("linkIndex")
        if tls_id is None or link_index_raw is None:
            continue
        if key in movements:
            raise ValueError(
                "route-level movement is ambiguous because multiple controlled "
                f"connections share {from_edge}->{to_edge}"
            )
        if tls_id not in programs:
            raise ValueError(
                f"connection {from_edge}->{to_edge} uses TLS {tls_id!r}, but program "
                f"{program_id!r} was not found"
            )
        link_index = int(link_index_raw)
        cycle_seconds, phases = programs[tls_id]
        green_seconds = 0.0
        for duration, state in phases:
            if link_index >= len(state):
                raise ValueError(
                    f"TLS {tls_id} state length {len(state)} does not cover linkIndex {link_index}"
                )
            if state[link_index] in {"G", "g"}:
                green_seconds += duration
        if green_seconds <= 0:
            raise ValueError(
                f"real movement {from_edge}->{to_edge} at {tls_id}:{link_index} never receives green"
            )
        capacity = saturation_flow_vph * green_seconds / cycle_seconds
        movements[key] = MovementInfo(
            from_edge=from_edge,
            to_edge=to_edge,
            tls_id=tls_id,
            link_index=link_index,
            direction=connection.get("dir", "unknown"),
            green_seconds=green_seconds,
            cycle_seconds=cycle_seconds,
            capacity_proxy_vph=capacity,
        )

    inbound_edges = tuple(
        sorted(
            (
                edge.edge_id
                for edge in edges.values()
                if edge.to_node in signal_nodes and edge.from_node not in signal_nodes
            ),
            key=natural_edge_key,
        )
    )
    outbound_edges = tuple(
        sorted(
            (
                edge.edge_id
                for edge in edges.values()
                if edge.from_node in signal_nodes and edge.to_node not in signal_nodes
            ),
            key=natural_edge_key,
        )
    )
    return NetworkData(
        edges=edges,
        transition_pairs=frozenset(transition_pairs),
        movements=movements,
        signal_nodes=signal_nodes,
        inbound_edges=inbound_edges,
        outbound_edges=outbound_edges,
    )


def load_route_catalog(route_catalog: Path, network: NetworkData) -> tuple[RouteInfo, ...]:
    root = ET.parse(route_catalog).getroot()
    routes: list[RouteInfo] = []
    seen_ids: set[str] = set()
    seen_ods: set[tuple[str, str]] = set()
    for route_element in root.findall("route"):
        route_id = required_attr(route_element, "id")
        if route_id in seen_ids:
            raise ValueError(f"duplicate route id: {route_id}")
        edge_ids = tuple(required_attr(route_element, "edges").split())
        if len(edge_ids) < 2:
            raise ValueError(f"route {route_id} must contain at least two edges")
        unknown = [edge_id for edge_id in edge_ids if edge_id not in network.edges]
        if unknown:
            raise ValueError(f"route {route_id} contains unknown edges: {unknown}")
        pairs = tuple(zip(edge_ids, edge_ids[1:]))
        invalid = [pair for pair in pairs if pair not in network.transition_pairs]
        if invalid:
            raise ValueError(f"route {route_id} contains illegal transitions: {invalid}")
        origin_edge = edge_ids[0]
        destination_edge = edge_ids[-1]
        if origin_edge not in network.inbound_edges:
            raise ValueError(f"route {route_id} does not start at a boundary inbound edge")
        if destination_edge not in network.outbound_edges:
            raise ValueError(f"route {route_id} does not end at a boundary outbound edge")
        od = (origin_edge, destination_edge)
        if od in seen_ods:
            raise ValueError(f"multiple catalog routes represent OD {origin_edge}->{destination_edge}")
        nodes = [network.edges[edge_ids[0]].from_node]
        nodes.extend(network.edges[edge_id].to_node for edge_id in edge_ids)
        if len(nodes) != len(set(nodes)):
            raise ValueError(f"route {route_id} repeats a junction and is not simple")
        movement_keys = tuple(pair for pair in pairs if pair in network.movements)
        routes.append(
            RouteInfo(
                route_id=route_id,
                edges=edge_ids,
                origin_edge=origin_edge,
                destination_edge=destination_edge,
                movement_keys=movement_keys,
            )
        )
        seen_ids.add(route_id)
        seen_ods.add(od)

    if not routes:
        raise ValueError("route catalog contains no top-level <route> definitions")
    validate_balanced_catalog(routes, network)
    return tuple(sorted(routes, key=lambda route: route.route_id))


def validate_balanced_catalog(routes: Sequence[RouteInfo], network: NetworkData) -> None:
    origins = sorted({route.origin_edge for route in routes}, key=natural_edge_key)
    destinations = sorted({route.destination_edge for route in routes}, key=natural_edge_key)
    if len(origins) != 10 or len(destinations) != 10:
        raise ValueError(
            f"expected 10 boundary origins and 10 destinations, found {len(origins)} and {len(destinations)}"
        )
    if len(routes) != 90:
        raise ValueError(f"expected exactly 90 non-return OD routes, found {len(routes)}")
    origin_bases = {boundary_key(edge) for edge in origins}
    destination_bases = {boundary_key(edge) for edge in destinations}
    if origin_bases != destination_bases:
        raise ValueError("origin/destination boundary identities do not match")
    route_ods = {(route.origin_edge, route.destination_edge) for route in routes}
    expected_ods = {
        (origin, destination)
        for origin in origins
        for destination in destinations
        if boundary_key(origin) != boundary_key(destination)
    }
    missing_ods = expected_ods - route_ods
    extra_ods = route_ods - expected_ods
    if missing_ods or extra_ods:
        raise ValueError(
            f"route catalog is not a complete non-return OD set; missing={sorted(missing_ods)}, "
            f"extra={sorted(extra_ods)}"
        )
    covered_movements = {
        movement
        for route in routes
        for movement in route.movement_keys
    }
    missing_movements = set(network.movements) - covered_movements
    if missing_movements:
        raise ValueError(
            f"route catalog does not cover all real controlled movements: {sorted(missing_movements)}"
        )


def build_perfect_matchings(routes: Sequence[RouteInfo]) -> tuple[tuple[RouteInfo, ...], ...]:
    origins_by_key = {boundary_key(route.origin_edge): route.origin_edge for route in routes}
    destinations_by_key = {
        boundary_key(route.destination_edge): route.destination_edge for route in routes
    }
    boundary_keys = sorted(origins_by_key, key=natural_boundary_key)
    route_by_od = {(route.origin_edge, route.destination_edge): route for route in routes}
    matchings: list[tuple[RouteInfo, ...]] = []
    for offset in range(1, len(boundary_keys)):
        matching: list[RouteInfo] = []
        for origin_index, origin_key in enumerate(boundary_keys):
            destination_key = boundary_keys[(origin_index + offset) % len(boundary_keys)]
            matching.append(
                route_by_od[(origins_by_key[origin_key], destinations_by_key[destination_key])]
            )
        if len({route.origin_edge for route in matching}) != len(boundary_keys):
            raise AssertionError("matching does not cover each origin exactly once")
        if len({route.destination_edge for route in matching}) != len(boundary_keys):
            raise AssertionError("matching does not cover each destination exactly once")
        matchings.append(tuple(matching))
    return tuple(matchings)


def build_episode_specs(args: argparse.Namespace) -> tuple[EpisodeSpec, ...]:
    specs: list[EpisodeSpec] = []
    for level_index, demand_vph in enumerate(args.levels_vph):
        for seed_index in range(1, args.seeds_per_level + 1):
            seed = args.seed_base + level_index * 1009 + seed_index * 37
            split = "validation" if args.seeds_per_level > 1 and seed_index == args.seeds_per_level else "train"
            episode_id = (
                f"level_{level_index:02d}_q{demand_vph:04d}_seed_{seed_index:02d}"
            )
            specs.append(
                EpisodeSpec(
                    episode_id=episode_id,
                    level_index=level_index,
                    demand_vph=demand_vph,
                    seed_index=seed_index,
                    seed=seed,
                    split=split,
                    duration_seconds=args.duration_seconds,
                )
            )
    return tuple(specs)


def plan_only_summary(
    args: argparse.Namespace,
    routes: Sequence[RouteInfo],
    network: NetworkData,
    specs: Sequence[EpisodeSpec],
) -> dict[str, object]:
    average_movements = statistics.fmean(len(route.movement_keys) for route in routes)
    total_vehicles = sum(
        round_half_up(spec.demand_vph * spec.duration_seconds / 3600.0)
        for spec in specs
    )
    estimated_rows = round(total_vehicles * average_movements)
    max_demand = max(spec.demand_vph for spec in specs)
    route_shares = Counter(
        movement
        for route in routes
        for movement in set(route.movement_keys)
    )
    max_vc = max(
        max_demand * count / len(routes) / network.movements[movement].capacity_proxy_vph
        for movement, count in route_shares.items()
    )
    return {
        "profile": args.profile,
        "demand_levels_vph": list(args.levels_vph),
        "seeds_per_level": args.seeds_per_level,
        "episode_count": len(specs),
        "duration_seconds_per_episode": args.duration_seconds,
        "block_seconds": args.block_seconds,
        "total_planned_vehicles": total_vehicles,
        "estimated_movement_training_rows": estimated_rows,
        "route_count": len(routes),
        "controlled_movement_count": len(network.movements),
        "maximum_demand_vph": max_demand,
        "static_max_vc_proxy_at_maximum_demand": round(max_vc, 6),
        "safety_cap_vph": args.safety_cap_vph,
        "max_vc_proxy_guard": args.max_vc_proxy,
    }


def generate_episode_vehicles(
    spec: EpisodeSpec,
    routes: Sequence[RouteInfo],
    matchings: Sequence[Sequence[RouteInfo]],
    block_seconds: int,
) -> list[VehiclePlan]:
    rng = random.Random(spec.seed)
    expected_total = round_half_up(spec.demand_vph * spec.duration_seconds / 3600.0)
    route_schedule = build_episode_route_schedule(expected_total, routes, matchings, rng)

    planned: list[VehiclePlan] = []
    assigned_total = 0
    schedule_cursor = 0
    block_start = 0
    while block_start < spec.duration_seconds:
        block_end = min(block_start + block_seconds, spec.duration_seconds)
        target_by_block_end = round_half_up(spec.demand_vph * block_end / 3600.0)
        block_vehicle_count = target_by_block_end - assigned_total
        assigned_total = target_by_block_end

        selected = route_schedule[
            schedule_cursor : schedule_cursor + block_vehicle_count
        ]
        schedule_cursor += block_vehicle_count

        if len(selected) != block_vehicle_count:
            raise AssertionError(
                f"block allocation mismatch: expected {block_vehicle_count}, got {len(selected)}"
            )

        by_origin_routes: dict[str, list[RouteInfo]] = defaultdict(list)
        for route in selected:
            by_origin_routes[route.origin_edge].append(route)

        block_duration = block_end - block_start
        for origin_edge in sorted(by_origin_routes, key=natural_edge_key):
            origin_routes = by_origin_routes[origin_edge]
            rng.shuffle(origin_routes)
            count = len(origin_routes)
            slot_seconds = block_duration / count
            used_departures: set[str] = set()
            for index, route in enumerate(origin_routes):
                # Keep jitter away from slot boundaries.  Even at the maximum
                # default demand this leaves several seconds between vehicles
                # from the same boundary entry.
                jitter = 0.15 + 0.70 * rng.random()
                depart = block_start + (index + jitter) * slot_seconds
                departure_key = f"{depart:.3f}"
                while departure_key in used_departures:
                    depart += 0.001
                    departure_key = f"{depart:.3f}"
                if depart >= block_end:
                    raise AssertionError("departure jitter escaped its allocation block")
                used_departures.add(departure_key)
                planned.append(
                    VehiclePlan(
                        depart=depart,
                        route_id=route.route_id,
                        origin_edge=route.origin_edge,
                        destination_edge=route.destination_edge,
                    )
                )
        block_start = block_end

    if len(planned) != expected_total:
        raise AssertionError(f"episode total mismatch: expected {expected_total}, got {len(planned)}")
    if schedule_cursor != len(route_schedule):
        raise AssertionError("not all routes in the episode schedule were consumed")
    planned.sort(key=lambda vehicle: (vehicle.depart, vehicle.origin_edge, vehicle.route_id))
    return planned


def build_episode_route_schedule(
    vehicle_count: int,
    routes: Sequence[RouteInfo],
    matchings: Sequence[Sequence[RouteInfo]],
    rng: random.Random,
) -> list[RouteInfo]:
    """Build a route sequence with balanced episode-level margins.

    The 90 OD graph decomposes into nine perfect 10-route matchings.  Every
    matching contributes exactly one vehicle to every origin and destination.
    Full 90-route cycles therefore cover every OD once.  Any final partial
    cycle is composed of full matchings plus at most one partial matching, so
    total origin and destination counts can differ by no more than one.

    The resulting sequence is later sliced into 300-second blocks.  A slice
    can start/end inside a matching, but the bulk of every block still consists
    of complete matchings rather than arbitrary independent OD draws.
    """

    if vehicle_count <= 0:
        return []
    full_route_cycles, remainder = divmod(vehicle_count, len(routes))
    schedule: list[RouteInfo] = []

    for _ in range(full_route_cycles):
        matching_order = list(range(len(matchings)))
        rng.shuffle(matching_order)
        for matching_index in matching_order:
            matching = list(matchings[matching_index])
            rng.shuffle(matching)
            schedule.extend(matching)

    full_matchings, partial_size = divmod(
        remainder,
        len({route.origin_edge for route in routes}),
    )
    matching_order = list(range(len(matchings)))
    rng.shuffle(matching_order)
    for matching_index in matching_order[:full_matchings]:
        matching = list(matchings[matching_index])
        rng.shuffle(matching)
        schedule.extend(matching)

    if partial_size:
        matching = list(matchings[matching_order[full_matchings]])
        rng.shuffle(matching)
        schedule.extend(matching[:partial_size])

    if len(schedule) != vehicle_count:
        raise AssertionError(
            f"route schedule mismatch: expected {vehicle_count}, got {len(schedule)}"
        )
    return schedule


def audit_episode(
    spec: EpisodeSpec,
    vehicles: Sequence[VehiclePlan],
    routes_by_id: dict[str, RouteInfo],
    network: NetworkData,
    args: argparse.Namespace,
) -> dict[str, object]:
    origin_counts = Counter(vehicle.origin_edge for vehicle in vehicles)
    destination_counts = Counter(vehicle.destination_edge for vehicle in vehicles)
    route_counts = Counter(vehicle.route_id for vehicle in vehicles)
    od_counts = Counter(
        f"{vehicle.origin_edge}->{vehicle.destination_edge}" for vehicle in vehicles
    )
    movement_counts: Counter[MovementKey] = Counter()
    edge_counts: Counter[str] = Counter()
    turn_counts: Counter[str] = Counter()
    estimated_rows = 0
    for vehicle in vehicles:
        route = routes_by_id[vehicle.route_id]
        edge_counts.update(route.edges)
        movement_counts.update(route.movement_keys)
        estimated_rows += len(route.movement_keys)
    for movement, count in movement_counts.items():
        turn_name = TURN_NAMES.get(network.movements[movement].direction, "other")
        turn_counts[turn_name] += count

    movement_details: dict[str, dict[str, object]] = {}
    max_vc = 0.0
    max_vc_movement = ""
    for key in sorted(network.movements):
        info = network.movements[key]
        count = movement_counts[key]
        flow_vph = count * 3600.0 / spec.duration_seconds
        vc_proxy = flow_vph / info.capacity_proxy_vph
        movement_name = f"{key[0]}->{key[1]}"
        movement_details[movement_name] = {
            "count": count,
            "flow_vph": round(flow_vph, 6),
            "capacity_proxy_vph": round(info.capacity_proxy_vph, 6),
            "vc_proxy": round(vc_proxy, 6),
            "tls_id": info.tls_id,
            "link_index": info.link_index,
            "turn": TURN_NAMES.get(info.direction, info.direction),
        }
        if vc_proxy > max_vc:
            max_vc = vc_proxy
            max_vc_movement = movement_name

    departure_strings = [f"{vehicle.depart:.3f}" for vehicle in vehicles]
    departure_counts = Counter(departure_strings)
    simultaneous_departure_excess = sum(count - 1 for count in departure_counts.values())
    max_same_timestamp = max(departure_counts.values(), default=0)

    by_origin_departures: dict[str, list[float]] = defaultdict(list)
    by_block_vehicles: dict[int, list[VehiclePlan]] = defaultdict(list)
    for vehicle in vehicles:
        by_origin_departures[vehicle.origin_edge].append(vehicle.depart)
        by_block_vehicles[int(vehicle.depart // args.block_seconds)].append(vehicle)
    per_origin_duplicate_count = 0
    min_origin_headway = math.inf
    for departures in by_origin_departures.values():
        formatted = [f"{depart:.3f}" for depart in departures]
        per_origin_duplicate_count += len(formatted) - len(set(formatted))
        ordered = sorted(departures)
        if len(ordered) > 1:
            min_origin_headway = min(
                min_origin_headway,
                min(right - left for left, right in zip(ordered, ordered[1:])),
            )

    expected_total = round_half_up(spec.demand_vph * spec.duration_seconds / 3600.0)
    realized_vph = len(vehicles) * 3600.0 / spec.duration_seconds
    origin_range = value_range(origin_counts.values())
    destination_range = value_range(destination_counts.values())
    movement_coverage = sum(count > 0 for count in movement_counts.values())
    route_coverage = sum(count > 0 for count in route_counts.values())
    maximum_block_origin_range = 0
    maximum_block_destination_range = 0
    for block_vehicles in by_block_vehicles.values():
        block_origins = Counter(vehicle.origin_edge for vehicle in block_vehicles)
        block_destinations = Counter(vehicle.destination_edge for vehicle in block_vehicles)
        maximum_block_origin_range = max(
            maximum_block_origin_range,
            value_range(block_origins.values()),
        )
        maximum_block_destination_range = max(
            maximum_block_destination_range,
            value_range(block_destinations.values()),
        )

    checks = {
        "vehicle_count_exact": len(vehicles) == expected_total,
        "origin_count_range_le_1": origin_range <= 1,
        "destination_count_range_le_1": destination_range <= 1,
        "block_origin_count_range_le_2": maximum_block_origin_range <= 2,
        "block_destination_count_range_le_2": maximum_block_destination_range <= 2,
        "all_routes_covered": route_coverage == len(routes_by_id),
        "all_movements_covered": movement_coverage == len(network.movements),
        "no_same_origin_duplicate_departure": per_origin_duplicate_count == 0,
        "demand_below_safety_cap": spec.demand_vph <= args.safety_cap_vph + 1e-9,
        "vc_proxy_below_guard": max_vc <= args.max_vc_proxy + 1e-9,
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise ValueError(f"episode {spec.episode_id} failed static checks: {failed}")

    return {
        "episode": {
            "episode_id": spec.episode_id,
            "profile": args.profile,
            "level_index": spec.level_index,
            "target_demand_vph": spec.demand_vph,
            "realized_demand_vph": round(realized_vph, 6),
            "duration_seconds": spec.duration_seconds,
            "block_seconds": args.block_seconds,
            "seed_index": spec.seed_index,
            "seed": spec.seed,
            "split": spec.split,
            "arrival_pattern": "balanced_block_stratified_jitter",
        },
        "counts": {
            "vehicle_count": len(vehicles),
            "estimated_movement_training_rows": estimated_rows,
            "route_coverage": route_coverage,
            "route_total": len(routes_by_id),
            "movement_coverage": movement_coverage,
            "movement_total": len(network.movements),
            "origin_count_range": origin_range,
            "destination_count_range": destination_range,
            "maximum_block_origin_count_range": maximum_block_origin_range,
            "maximum_block_destination_count_range": maximum_block_destination_range,
            "global_duplicate_departure_excess": simultaneous_departure_excess,
            "maximum_vehicles_at_one_timestamp": max_same_timestamp,
            "same_origin_duplicate_departures": per_origin_duplicate_count,
            "minimum_same_origin_headway_seconds": (
                round(min_origin_headway, 6) if math.isfinite(min_origin_headway) else None
            ),
        },
        "safety": {
            "maximum_movement_vc_proxy": round(max_vc, 6),
            "maximum_vc_movement": max_vc_movement,
            "vc_proxy_guard": args.max_vc_proxy,
            "demand_safety_cap_vph": args.safety_cap_vph,
            "note": (
                "Static V/C is only a generation-time guard. SUMO pilot results, "
                "clearance time, teleports, departDelay and occupancy remain authoritative."
            ),
        },
        "distribution": {
            "origin_vehicle_counts": dict(sorted(origin_counts.items(), key=lambda item: natural_edge_key(item[0]))),
            "destination_vehicle_counts": dict(
                sorted(destination_counts.items(), key=lambda item: natural_edge_key(item[0]))
            ),
            "od_vehicle_counts": dict(sorted(od_counts.items())),
            "route_vehicle_counts": dict(sorted(route_counts.items())),
            "edge_traversal_counts": dict(sorted(edge_counts.items(), key=lambda item: natural_edge_key(item[0]))),
            "turn_event_counts": dict(sorted(turn_counts.items())),
            "movement_traversals": movement_details,
        },
        "checks": checks,
    }


def write_route_file(
    path: Path,
    spec: EpisodeSpec,
    vehicles: Sequence[VehiclePlan],
    routes: Sequence[RouteInfo],
    args: argparse.Namespace,
) -> None:
    root = ET.Element("routes")
    root.append(ET.Comment(
        f" {spec.episode_id}; q={spec.demand_vph} veh/h; seed={spec.seed}; "
        "balanced 90-OD stratified departures "
    ))
    ET.SubElement(root, "vType", **vehicle_type_attributes(args.lane_change_mode))
    for route in routes:
        ET.SubElement(root, "route", id=route.route_id, edges=" ".join(route.edges))
    for vehicle_index, vehicle in enumerate(vehicles, start=1):
        attributes = {
            "id": f"{spec.episode_id}_veh_{vehicle_index:06d}",
            "type": "car",
            "route": vehicle.route_id,
            "depart": f"{vehicle.depart:.3f}",
            "departLane": args.depart_lane,
            "departSpeed": args.depart_speed,
            "departPos": args.depart_pos,
        }
        if args.lane_change_mode == "no-change":
            attributes["laneChangeMode"] = "517"
        ET.SubElement(root, "vehicle", **attributes)
    write_xml(path, root, args.force)


def vehicle_type_attributes(lane_change_mode: str) -> dict[str, str]:
    attributes = {
        "id": "car",
        "accel": "2.6",
        "decel": "4.5",
        "sigma": "0.5",
        "length": "5.0",
        "minGap": "2.5",
        "maxSpeed": "13.89",
        "guiShape": "passenger",
    }
    if lane_change_mode == "no-change":
        attributes.update(
            {
                "lcStrategic": "1",
                "lcCooperative": "1",
                "lcSpeedGain": "0",
                "lcKeepRight": "0",
                "lcSublane": "0",
            }
        )
    else:
        attributes.update(
            {
                "lcStrategic": "1",
                "lcCooperative": "1",
                "lcSpeedGain": "1",
                "lcKeepRight": "1",
                "lcSublane": "1",
            }
        )
    return attributes


def write_sumocfg(
    path: Path,
    spec: EpisodeSpec,
    route_path: Path,
    copied_net_path: Path,
    tripinfo_path: Path,
    force: bool,
) -> None:
    root = ET.Element("configuration")
    input_element = ET.SubElement(root, "input")
    ET.SubElement(input_element, "net-file", value=os.path.relpath(copied_net_path, path.parent))
    ET.SubElement(input_element, "route-files", value=os.path.relpath(route_path, path.parent))
    output_element = ET.SubElement(root, "output")
    ET.SubElement(
        output_element,
        "tripinfo-output",
        value=os.path.relpath(tripinfo_path, path.parent),
    )
    time_element = ET.SubElement(root, "time")
    ET.SubElement(time_element, "begin", value="0")
    random_element = ET.SubElement(root, "random_number")
    ET.SubElement(random_element, "seed", value=str(spec.seed))
    report_element = ET.SubElement(root, "report")
    ET.SubElement(report_element, "duration-log.statistics", value="true")
    ET.SubElement(report_element, "no-step-log", value="true")
    write_xml(path, root, force)


def write_xml(path: Path, root: ET.Element, force: bool) -> None:
    ensure_writable(path, force)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="    ")
    tree = ET.ElementTree(root)
    temp_path = path.with_name(path.name + ".tmp")
    tree.write(temp_path, encoding="utf-8", xml_declaration=True)
    temp_path.replace(path)


def write_json(path: Path, value: object, force: bool) -> None:
    ensure_writable(path, force)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(path.name + ".tmp")
    temp_path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    temp_path.replace(path)


def write_dataset(
    args: argparse.Namespace,
    network: NetworkData,
    routes: Sequence[RouteInfo],
    specs: Sequence[EpisodeSpec],
) -> dict[str, object]:
    output_dir = args.output_dir.resolve()
    directories = {
        "inputs": output_dir / "inputs",
        "routes": output_dir / "routes",
        "sumocfg": output_dir / "sumocfg",
        "manifests": output_dir / "manifests",
        "commands": output_dir / "commands",
        "tripinfo": output_dir / "outputs" / "tripinfo",
        "traci": output_dir / "outputs" / "traci",
        "logs": output_dir / "outputs" / "logs",
    }
    for directory in directories.values():
        directory.mkdir(parents=True, exist_ok=True)

    copied_net = directories["inputs"] / args.net_file.name
    copied_catalog = directories["inputs"] / args.route_catalog.name
    copy_input(args.net_file.resolve(), copied_net, args.force)
    copy_input(args.route_catalog.resolve(), copied_catalog, args.force)

    routes_by_id = {route.route_id: route for route in routes}
    matchings = build_perfect_matchings(routes)
    episode_rows: list[dict[str, object]] = []
    audits: list[dict[str, object]] = []

    for spec in specs:
        vehicles = generate_episode_vehicles(spec, routes, matchings, args.block_seconds)
        audit = audit_episode(spec, vehicles, routes_by_id, network, args)

        route_path = directories["routes"] / f"{spec.episode_id}.rou.xml"
        sumocfg_path = directories["sumocfg"] / f"{spec.episode_id}.sumocfg"
        manifest_path = directories["manifests"] / f"{spec.episode_id}.json"
        tripinfo_path = directories["tripinfo"] / f"tripinfo_{spec.episode_id}.xml"
        legacy_output = directories["traci"] / f"TraCI_output_adjusted_{spec.episode_id}.csv"
        training_output = directories["traci"] / f"cams_model_training_data_{spec.episode_id}.csv"
        event_output = directories["traci"] / f"cams_vehicle_events_{spec.episode_id}.csv"
        cycle_output = directories["traci"] / f"cams_cycle_summary_{spec.episode_id}.csv"
        log_output = directories["logs"] / f"traci_{spec.episode_id}.log"

        write_route_file(route_path, spec, vehicles, routes, args)
        write_sumocfg(sumocfg_path, spec, route_path, copied_net, tripinfo_path, args.force)
        audit["files"] = {
            "route_file": relative_to(route_path, output_dir),
            "sumocfg_file": relative_to(sumocfg_path, output_dir),
            "tripinfo_output": relative_to(tripinfo_path, output_dir),
            "legacy_training_output": relative_to(legacy_output, output_dir),
            "full_training_output": relative_to(training_output, output_dir),
            "vehicle_event_output": relative_to(event_output, output_dir),
            "cycle_summary_output": relative_to(cycle_output, output_dir),
            "run_log": relative_to(log_output, output_dir),
        }
        write_json(manifest_path, audit, args.force)
        audits.append(audit)

        episode_rows.append(
            {
                "episode_id": spec.episode_id,
                "profile": args.profile,
                "level_index": spec.level_index,
                "target_demand_vph": spec.demand_vph,
                "realized_demand_vph": audit["episode"]["realized_demand_vph"],
                "duration_seconds": spec.duration_seconds,
                "block_seconds": args.block_seconds,
                "seed_index": spec.seed_index,
                "seed": spec.seed,
                "split": spec.split,
                "vehicle_count": audit["counts"]["vehicle_count"],
                "estimated_movement_training_rows": audit["counts"]["estimated_movement_training_rows"],
                "movement_coverage": audit["counts"]["movement_coverage"],
                "maximum_movement_vc_proxy": audit["safety"]["maximum_movement_vc_proxy"],
                "maximum_vc_movement": audit["safety"]["maximum_vc_movement"],
                "route_file": relative_to(route_path, output_dir),
                "sumocfg_file": relative_to(sumocfg_path, output_dir),
                "tripinfo_output": relative_to(tripinfo_path, output_dir),
                "legacy_training_output": relative_to(legacy_output, output_dir),
                "full_training_output": relative_to(training_output, output_dir),
                "vehicle_event_output": relative_to(event_output, output_dir),
                "cycle_summary_output": relative_to(cycle_output, output_dir),
                "run_log": relative_to(log_output, output_dir),
            }
        )

    episodes_csv = directories["manifests"] / "episodes.csv"
    write_csv(episodes_csv, episode_rows, args.force)
    run_manifest = directories["commands"] / "traci_jobs.tsv"
    write_tsv(run_manifest, episode_rows, args.force)
    run_script = directories["commands"] / "run_traci.sh"
    write_run_script(run_script, args, output_dir, copied_net, run_manifest)

    summary = build_dataset_summary(args, network, routes, episode_rows, audits)
    summary["files"] = {
        "episodes_csv": relative_to(episodes_csv, output_dir),
        "traci_jobs_tsv": relative_to(run_manifest, output_dir),
        "run_traci_script": relative_to(run_script, output_dir),
        "copied_net_file": relative_to(copied_net, output_dir),
        "copied_route_catalog": relative_to(copied_catalog, output_dir),
    }
    write_json(directories["manifests"] / "dataset_summary.json", summary, args.force)
    return summary


def build_dataset_summary(
    args: argparse.Namespace,
    network: NetworkData,
    routes: Sequence[RouteInfo],
    episode_rows: Sequence[dict[str, object]],
    audits: Sequence[dict[str, object]],
) -> dict[str, object]:
    total_vehicles = sum(int(row["vehicle_count"]) for row in episode_rows)
    total_rows = sum(int(row["estimated_movement_training_rows"]) for row in episode_rows)
    split_episodes = Counter(str(row["split"]) for row in episode_rows)
    split_vehicles = Counter()
    for row in episode_rows:
        split_vehicles[str(row["split"])] += int(row["vehicle_count"])
    levels: dict[str, dict[str, object]] = {}
    for demand in args.levels_vph:
        rows = [row for row in episode_rows if int(row["target_demand_vph"]) == demand]
        levels[str(demand)] = {
            "episode_count": len(rows),
            "vehicle_count": sum(int(row["vehicle_count"]) for row in rows),
            "estimated_movement_training_rows": sum(
                int(row["estimated_movement_training_rows"]) for row in rows
            ),
            "maximum_movement_vc_proxy": max(
                float(row["maximum_movement_vc_proxy"]) for row in rows
            ),
        }
    checks = {
        "all_episode_checks_passed": all(
            all(bool(value) for value in audit["checks"].values()) for audit in audits
        ),
        "all_90_od_routes_present": len(routes) == 90,
        "all_72_controlled_movements_present": len(network.movements) == 72,
        "peak_demand_below_safety_cap": max(args.levels_vph) <= args.safety_cap_vph,
        "peak_static_vc_below_guard": max(
            float(row["maximum_movement_vc_proxy"]) for row in episode_rows
        ) <= args.max_vc_proxy,
    }
    return {
        "metadata": {
            "generator": Path(__file__).name,
            "profile": args.profile,
            "net_file_sha256": sha256_file(args.net_file),
            "route_catalog_sha256": sha256_file(args.route_catalog),
            "program_id": args.program_id,
            "seed_base": args.seed_base,
            "arrival_pattern": "balanced_block_stratified_jitter",
            "lane_change_mode": args.lane_change_mode,
            "traci_outputs": args.traci_outputs,
        },
        "design": {
            "demand_levels_vph": list(args.levels_vph),
            "seeds_per_level": args.seeds_per_level,
            "duration_seconds_per_episode": args.duration_seconds,
            "block_seconds": args.block_seconds,
            "episode_count": len(episode_rows),
            "route_count": len(routes),
            "origin_count": len(network.inbound_edges),
            "destination_count": len(network.outbound_edges),
            "controlled_movement_count": len(network.movements),
            "safety_cap_vph": args.safety_cap_vph,
            "max_vc_proxy_guard": args.max_vc_proxy,
            "saturation_flow_proxy_vph": args.saturation_flow_vph,
        },
        "size": {
            "total_vehicles": total_vehicles,
            "estimated_movement_training_rows": total_rows,
            "episodes_by_split": dict(sorted(split_episodes.items())),
            "vehicles_by_split": dict(sorted(split_vehicles.items())),
        },
        "levels": levels,
        "checks": checks,
        "post_sumo_acceptance_required": {
            "teleports": "must be 0",
            "unfinished_vehicles": "must be 0",
            "clearance_ratio": "simulation_end / demand_end should be <= 1.25",
            "depart_delay": "inspect P95 and maximum; stop escalation if queues block insertion",
            "lane_occupancy": "inspect P95 and maximum; stop escalation near persistent spillback",
            "density_escalation": (
                "Only after the top two levels pass all seeds; add at most 200 veh/h per round."
            ),
        },
    }


def write_csv(path: Path, rows: Sequence[dict[str, object]], force: bool) -> None:
    ensure_writable(path, force)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError("cannot write an empty CSV")
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    temp_path.replace(path)


def write_tsv(path: Path, episode_rows: Sequence[dict[str, object]], force: bool) -> None:
    ensure_writable(path, force)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "episode_id",
        "split",
        "sumocfg_file",
        "legacy_training_output",
        "full_training_output",
        "vehicle_event_output",
        "cycle_summary_output",
        "run_log",
    )
    temp_path = path.with_name(path.name + ".tmp")
    with temp_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, delimiter="\t")
        writer.writeheader()
        for row in episode_rows:
            writer.writerow({field: row[field] for field in fields})
    temp_path.replace(path)


def write_run_script(
    path: Path,
    args: argparse.Namespace,
    output_dir: Path,
    copied_net: Path,
    run_manifest: Path,
) -> None:
    ensure_writable(path, args.force)
    path.parent.mkdir(parents=True, exist_ok=True)
    script = f"""#!/usr/bin/env bash
set -euo pipefail

DATASET_DIR="$(cd "$(dirname "${{BASH_SOURCE[0]}}")/.." && pwd)"
TRACI_SCRIPT="${{TRACI_SCRIPT:-{shell_single_quote(args.traci_script)}}}"
SUMO_BINARY="${{SUMO_BINARY:-{shell_single_quote(args.sumo_binary)}}}"
TRACI_OUTPUTS="${{TRACI_OUTPUTS:-{shell_single_quote(args.traci_outputs)}}}"
NET_FILE="$DATASET_DIR/{relative_to(copied_net, output_dir)}"
JOB_FILE="$DATASET_DIR/{relative_to(run_manifest, output_dir)}"

tail -n +2 "$JOB_FILE" | while IFS=$'\\t' read -r episode_id split sumocfg legacy full events cycles log_file; do
    mkdir -p "$(dirname "$DATASET_DIR/$legacy")" "$(dirname "$DATASET_DIR/$log_file")"
    echo "[RUN] $episode_id demand-data split=$split"
    python3 "$TRACI_SCRIPT" \\
        --sumo-config "$DATASET_DIR/$sumocfg" \\
        --net-file "$NET_FILE" \\
        --sumo-binary "$SUMO_BINARY" \\
        --outputs "$TRACI_OUTPUTS" \\
        --legacy-edge-output "$DATASET_DIR/$legacy" \\
        --training-output "$DATASET_DIR/$full" \\
        --vehicle-event-output "$DATASET_DIR/$events" \\
        --cycle-summary-output "$DATASET_DIR/$cycles" \\
        2>&1 | tee "$DATASET_DIR/$log_file"
done
"""
    temp_path = path.with_name(path.name + ".tmp")
    temp_path.write_text(script, encoding="utf-8", newline="\n")
    temp_path.chmod(0o755)
    temp_path.replace(path)


def copy_input(source: Path, destination: Path, force: bool) -> None:
    if source == destination:
        return
    ensure_writable(destination, force)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temp_path = destination.with_name(destination.name + ".tmp")
    shutil.copy2(source, temp_path)
    temp_path.replace(destination)


def ensure_writable(path: Path, force: bool) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"refusing to overwrite without --force: {path}")


def relative_to(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def boundary_key(edge_id: str) -> str:
    return edge_id[1:] if edge_id.startswith("-") else edge_id


def natural_boundary_key(value: str) -> tuple[object, ...]:
    return tuple(int(part) if part.isdigit() else part for part in re.split(r"(\d+)", value))


def natural_edge_key(edge_id: str) -> tuple[object, ...]:
    return (natural_boundary_key(boundary_key(edge_id)), 0 if edge_id.startswith("-") else 1)


def round_half_up(value: float) -> int:
    return int(math.floor(value + 0.5))


def value_range(values: Iterable[int]) -> int:
    values = list(values)
    return max(values) - min(values) if values else 0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def shell_single_quote(value: str) -> str:
    # The value is embedded as a default inside a double-quoted shell
    # parameter expansion.  shlex.quote keeps spaces safe; remove its outer
    # quotes because the expansion already supplies double quotes.
    quoted = shlex.quote(value)
    return quoted[1:-1] if len(quoted) >= 2 and quoted[0] == quoted[-1] == "'" else quoted


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    network = load_network(args.net_file, args.program_id, args.saturation_flow_vph)
    routes = load_route_catalog(args.route_catalog, network)
    specs = build_episode_specs(args)
    plan = plan_only_summary(args, routes, network, specs)

    if plan["static_max_vc_proxy_at_maximum_demand"] > args.max_vc_proxy + 1e-9:
        raise ValueError(
            "planned maximum static movement V/C proxy "
            f"{plan['static_max_vc_proxy_at_maximum_demand']:.3f} exceeds guard "
            f"{args.max_vc_proxy:.3f}"
        )

    if args.plan_only:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return 0

    summary = write_dataset(args, network, routes, specs)
    print(f"Dataset written to: {args.output_dir.resolve()}")
    print(f"Profile: {args.profile}")
    print(f"Episodes: {summary['design']['episode_count']}")
    print(f"Vehicles: {summary['size']['total_vehicles']}")
    print(
        "Estimated movement training rows: "
        f"{summary['size']['estimated_movement_training_rows']}"
    )
    print(f"Demand levels: {', '.join(map(str, args.levels_vph))} veh/h")
    print(
        "Peak static movement V/C proxy: "
        f"{max(float(level['maximum_movement_vc_proxy']) for level in summary['levels'].values()):.3f}"
    )
    print("Static checks: PASS")
    print("SUMO pilot checks are still required before increasing the demand cap.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileExistsError, ValueError, ET.ParseError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
