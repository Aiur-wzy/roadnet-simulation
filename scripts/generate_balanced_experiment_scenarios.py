#!/usr/bin/env python3
"""Generate the three balanced, multi-cycle experiment route files.

The scenarios share one 90-OD catalog. Congestion is controlled with repeating
high-pressure and recovery windows instead of a permanently oversaturated OD.
"""

from __future__ import annotations

import argparse
import csv
import os
import random
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

SCRIPT_REPO_ROOT = Path(__file__).resolve().parents[1]
IMPORT_ROOT = next(
    (
        candidate
        for candidate in (SCRIPT_REPO_ROOT, Path.cwd().resolve())
        if (candidate / "generate_balanced_training_data.py").is_file()
    ),
    SCRIPT_REPO_ROOT,
)
if str(IMPORT_ROOT) not in sys.path:
    sys.path.insert(0, str(IMPORT_ROOT))

import generate_balanced_training_data as balanced


@dataclass(frozen=True)
class Scenario:
    name: str
    average_demand_vph: int
    seed: int
    demand_profile: tuple[tuple[int, float], ...]


SCENARIOS = (
    Scenario(
        "free",
        2800,
        20260764,
        ((10000, 2800.0),),
    ),
    Scenario(
        "tight",
        6400,
        20261773,
        ((1200, 8100.0), (1200, 4700.0)) * 4 + ((400, 6400.0),),
    ),
    Scenario(
        "congested",
        7000,
        20262782,
        ((1500, 8100.0), (700, 4700.0)) * 4
        + ((1200, 6866.6666666667),),
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--net-file", type=Path, default=Path("data/net/test_new.net.xml"))
    parser.add_argument(
        "--route-catalog",
        type=Path,
        default=Path("train_data/training_balanced_routes.rou.xml"),
    )
    parser.add_argument("--data-dir", type=Path, default=Path("data"))
    parser.add_argument("--block-seconds", type=int, default=300)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def demand_blocks(
    profile: Sequence[tuple[int, float]], block_seconds: int
) -> list[tuple[int, int, float]]:
    blocks: list[tuple[int, int, float]] = []
    current_time = 0
    for duration, demand_vph in profile:
        remaining = duration
        while remaining:
            span = min(block_seconds, remaining)
            blocks.append((current_time, current_time + span, demand_vph))
            current_time += span
            remaining -= span
    if current_time != 10000:
        raise ValueError(f"scenario duration must be 10000s, got {current_time}")
    return blocks


def generate_vehicles(
    scenario: Scenario,
    routes: Sequence[balanced.RouteInfo],
    matchings: Sequence[Sequence[balanced.RouteInfo]],
    block_seconds: int,
) -> list[balanced.VehiclePlan]:
    blocks = demand_blocks(scenario.demand_profile, block_seconds)
    exact_total = sum((end - start) * demand for start, end, demand in blocks) / 3600.0
    expected_total = balanced.round_half_up(exact_total)
    rng = random.Random(scenario.seed)
    schedule = balanced.build_episode_route_schedule(
        expected_total, routes, matchings, rng
    )

    vehicles: list[balanced.VehiclePlan] = []
    schedule_cursor = 0
    assigned_total = 0
    cumulative_exact = 0.0
    for start, end, demand_vph in blocks:
        cumulative_exact += demand_vph * (end - start) / 3600.0
        target_total = balanced.round_half_up(cumulative_exact)
        block_count = target_total - assigned_total
        assigned_total = target_total
        selected = schedule[schedule_cursor : schedule_cursor + block_count]
        schedule_cursor += block_count

        by_origin: dict[str, list[balanced.RouteInfo]] = defaultdict(list)
        for route in selected:
            by_origin[route.origin_edge].append(route)
        for origin in sorted(by_origin, key=balanced.natural_edge_key):
            origin_routes = by_origin[origin]
            rng.shuffle(origin_routes)
            slot_seconds = (end - start) / len(origin_routes)
            used_departures: set[str] = set()
            for index, route in enumerate(origin_routes):
                depart = start + (index + 0.15 + 0.70 * rng.random()) * slot_seconds
                departure_key = f"{depart:.3f}"
                while departure_key in used_departures:
                    depart += 0.001
                    departure_key = f"{depart:.3f}"
                used_departures.add(departure_key)
                vehicles.append(
                    balanced.VehiclePlan(
                        depart=depart,
                        route_id=route.route_id,
                        origin_edge=route.origin_edge,
                        destination_edge=route.destination_edge,
                    )
                )

    if len(vehicles) != expected_total or schedule_cursor != expected_total:
        raise AssertionError("profile vehicle allocation mismatch")
    vehicles.sort(key=lambda row: (row.depart, row.origin_edge, row.route_id))
    return vehicles


def add_profile_comment(route_path: Path, scenario: Scenario) -> None:
    tree = ET.parse(route_path)
    root = tree.getroot()
    profile_text = ", ".join(
        f"{duration}s@{demand:g}veh/h"
        for duration, demand in scenario.demand_profile
    )
    root.insert(1, ET.Comment(f" block demand profile: {profile_text} "))
    ET.indent(root, space="    ")
    tree.write(route_path, encoding="utf-8", xml_declaration=True)


def config_path_value(target: Path, config_dir: Path) -> str:
    try:
        value = os.path.relpath(target, config_dir)
    except ValueError:
        value = str(target)
    return value.replace("\\", "/")


def write_sumocfg(
    path: Path,
    scenario: Scenario,
    route_path: Path,
    net_path: Path,
    tripinfo_path: Path,
    force: bool,
) -> None:
    root = ET.Element("configuration")
    input_element = ET.SubElement(root, "input")
    ET.SubElement(
        input_element,
        "net-file",
        value=config_path_value(net_path, path.parent),
    )
    ET.SubElement(
        input_element,
        "route-files",
        value=config_path_value(route_path, path.parent),
    )
    output_element = ET.SubElement(root, "output")
    ET.SubElement(
        output_element,
        "tripinfo-output",
        value=config_path_value(tripinfo_path, path.parent),
    )
    time_element = ET.SubElement(root, "time")
    ET.SubElement(time_element, "begin", value="0")
    random_element = ET.SubElement(root, "random_number")
    ET.SubElement(random_element, "seed", value=str(scenario.seed))
    report_element = ET.SubElement(root, "report")
    ET.SubElement(report_element, "duration-log.statistics", value="true")
    ET.SubElement(report_element, "no-step-log", value="true")
    balanced.write_xml(path, root, force)


def main() -> None:
    args = parse_args()
    data_dir = args.data_dir.resolve()
    route_dir = data_dir / "route"
    config_dir = data_dir / "sumocfg"
    tripinfo_dir = data_dir / "tripinfo"
    for directory in (route_dir, config_dir, tripinfo_dir):
        directory.mkdir(parents=True, exist_ok=True)

    network = balanced.load_network(args.net_file.resolve(), "0", 1800.0)
    routes = balanced.load_route_catalog(args.route_catalog.resolve(), network)
    matchings = balanced.build_perfect_matchings(routes)
    write_args = argparse.Namespace(
        lane_change_mode="no-change",
        depart_lane="best",
        depart_speed="max",
        depart_pos="base",
        force=args.force,
    )
    manifest_rows: list[dict[str, object]] = []

    for scenario in SCENARIOS:
        vehicles = generate_vehicles(scenario, routes, matchings, args.block_seconds)
        spec = balanced.EpisodeSpec(
            episode_id=scenario.name,
            level_index=0,
            demand_vph=scenario.average_demand_vph,
            seed_index=1,
            seed=scenario.seed,
            split="experiment",
            duration_seconds=10000,
        )
        route_path = route_dir / f"{scenario.name}.rou.xml"
        config_path = config_dir / f"{scenario.name}.sumocfg"
        tripinfo_path = tripinfo_dir / f"tripinfo_{scenario.name}.xml"
        balanced.write_route_file(route_path, spec, vehicles, routes, write_args)
        add_profile_comment(route_path, scenario)
        write_sumocfg(
            config_path,
            scenario,
            route_path,
            args.net_file.resolve(),
            tripinfo_path,
            args.force,
        )
        manifest_rows.append(
            {
                "scenario": scenario.name,
                "average_demand_vph": scenario.average_demand_vph,
                "vehicles": len(vehicles),
                "seed": scenario.seed,
                "demand_profile": "; ".join(
                    f"{duration}s@{demand:g}veh/h"
                    for duration, demand in scenario.demand_profile
                ),
            }
        )
        print(f"{scenario.name}: {len(vehicles)} vehicles")

    manifest_path = route_dir / "experiment_profiles_free_tight_congested.csv"
    if manifest_path.exists() and not args.force:
        raise FileExistsError(f"refusing to overwrite {manifest_path}; pass --force")
    with manifest_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(manifest_rows[0]))
        writer.writeheader()
        writer.writerows(manifest_rows)
    print(f"profile manifest: {manifest_path}")


if __name__ == "__main__":
    main()
