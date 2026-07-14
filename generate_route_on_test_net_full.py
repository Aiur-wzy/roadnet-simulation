import argparse
import csv
import os
import random
import xml.etree.ElementTree as ET
from xml.dom import minidom


# Expanded route set generated from test.net.xml.
# Each route is an edge sequence that follows valid SUMO <connection from=... to=...> relations.
EXPANDED_ROUTES = [
    ("route_01_E9_to_E5_original", "E9 -E12 E20 E15 -E0 E5"),
    ("route_02_E9_to_E5_short", "E9 -E12 E13 E5"),
    ("route_03_E9_to_E4", "E9 -E11 E16 E15 E4"),
    ("route_04_E9_to_E14", "E9 -E12 -E14"),
    ("route_05_E9_to_E18", "E9 -E11 -E18"),
    ("route_06_E10_to_E19", "E10 -E11 E16 E19"),
    ("route_07_E10_to_E5", "E10 -E12 E13 E5"),
    ("route_08_E10_to_E17", "E10 -E11 -E17"),
    ("route_09_E10_to_E4", "E10 -E11 E16 E15 E4"),
    ("route_10_E14_to_E9", "E14 E12 -E9"),
    ("route_11_E14_to_E5", "E14 E13 E5"),
    ("route_12_E14_to_E19", "E14 E20 E19"),
    ("route_13_E14_to_E4", "E14 E13 E0 E4"),
    ("route_14_E2_to_E14", "-E2 -E13 -E14"),
    ("route_15_E2_to_E4", "-E2 E0 E4"),
    ("route_16_E2_to_E17", "-E2 -E13 E12 -E11 -E17"),
    ("route_17_E2_to_E19", "-E2 -E13 E20 E19"),
    ("route_18_E5_to_E14", "-E5 -E13 -E14"),
    ("route_19_E5_to_E3", "-E5 E0 E3"),
    ("route_20_E5_to_E18", "-E5 -E13 E12 -E11 -E18"),
    ("route_21_E5_to_E19", "-E5 -E13 E20 E19"),
    ("route_22_E4_to_E3", "-E4 E3"),
    ("route_23_E4_to_E14", "-E4 -E0 -E13 -E14"),
    ("route_24_E4_to_E18", "-E4 -E15 -E16 -E18"),
    ("route_25_E4_to_E2", "-E4 -E0 E2"),
    ("route_26_E3_to_E4", "-E3 E4"),
    ("route_27_E3_to_E2", "-E3 -E0 E2"),
    ("route_28_E3_to_E17", "-E3 -E15 -E16 -E17"),
    ("route_29_E3_to_E14", "-E3 -E0 -E13 -E14"),
    ("route_30_E19_to_E17", "-E19 -E16 -E17"),
    ("route_31_E19_to_E4", "-E19 E15 E4"),
    ("route_32_E19_to_E5", "-E19 -E20 E13 E5"),
    ("route_33_E19_to_E10", "-E19 -E16 E11 -E10"),
    ("route_34_E18_to_E9", "E18 E11 -E9"),
    ("route_35_E18_to_E4", "E18 E16 E15 E4"),
    ("route_36_E18_to_E14", "E18 E11 -E12 -E14"),
    ("route_37_E18_to_E5", "E18 E11 -E12 E13 E5"),
    ("route_38_E17_to_E10", "E17 E11 -E10"),
    ("route_39_E17_to_E3", "E17 E16 E15 E3"),
    ("route_40_E17_to_E14", "E17 E11 -E12 -E14"),
    ("route_41_E17_to_E5", "E17 E11 -E12 E13 E5"),
]

# Convenient route groups. You can add/remove routes here depending on the experiment.
ROUTE_OPTIONS = {
    "ORIGINAL": [
        ("fixed_route_e9", "E9 -E12 E20 E15 -E0 E5"),
        ("fixed_route_e10", "E10 -E12 E20 E15 -E0 E5"),
    ],
    "ALL": EXPANDED_ROUTES,
    "CORE": EXPANDED_ROUTES[:21],
    "WEST_IN": EXPANDED_ROUTES[0:9],      # Routes starting from E9 / E10
    "NORTH_IN": EXPANDED_ROUTES[9:17],    # Routes starting from E14 / -E2
    "EAST_IN": EXPANDED_ROUTES[17:25],    # Routes starting from -E5 / -E4
    "SOUTH_IN": EXPANDED_ROUTES[25:33],   # Routes starting from -E3 / -E19
    "SOUTHWEST_IN": EXPANDED_ROUTES[33:41], # Routes starting from E18 / E17
}


ALL_CONGESTION_SCENARIOS = [
    {
        "scenario_name": "free_all",
        "start_time": 0,
        "end_time": 10000,
        "route_option": "ALL",
        "mode": "uniform",
        "num_departures": 50,
    },
    {
        "scenario_name": "light_all_seed1",
        "start_time": 11000,
        "end_time": 21000,
        "route_option": "ALL",
        "mode": "poisson",
        "num_departures": 100,
        "seed": 1,
    },
    {
        "scenario_name": "light_all_seed2",
        "start_time": 22000,
        "end_time": 32000,
        "route_option": "ALL",
        "mode": "poisson",
        "num_departures": 100,
        "seed": 2,
    },
    {
        "scenario_name": "medium_core",
        "start_time": 34000,
        "end_time": 44000,
        "route_option": "CORE",
        "mode": "period",
        "period": 80,
    },
    {
        "scenario_name": "heavy_all",
        "start_time": 46000,
        "end_time": 56000,
        "route_option": "ALL",
        "mode": "period",
        "period": 50,
    },
    {
        "scenario_name": "oversat_all",
        "start_time": 58000,
        "end_time": 68000,
        "route_option": "ALL",
        "mode": "period",
        "period": 30,
    },
    {
        "scenario_name": "west_bottleneck",
        "start_time": 70000,
        "end_time": 80000,
        "route_option": "WEST_IN",
        "mode": "period",
        "period": 30,
    },
    {
        "scenario_name": "north_bottleneck",
        "start_time": 82000,
        "end_time": 92000,
        "route_option": "NORTH_IN",
        "mode": "period",
        "period": 30,
    },
    {
        "scenario_name": "east_bottleneck",
        "start_time": 94000,
        "end_time": 104000,
        "route_option": "EAST_IN",
        "mode": "period",
        "period": 30,
    },
    {
        "scenario_name": "south_bottleneck",
        "start_time": 106000,
        "end_time": 116000,
        "route_option": "SOUTH_IN",
        "mode": "period",
        "period": 30,
    },
    {
        "scenario_name": "southwest_bottleneck",
        "start_time": 118000,
        "end_time": 128000,
        "route_option": "SOUTHWEST_IN",
        "mode": "period",
        "period": 30,
    },
]


GROUP_DESCRIPTIONS = {
    "free_all": "Free-flow baseline across all expanded routes.",
    "light_all_seed1": "Light poisson demand across all expanded routes using seed 1.",
    "light_all_seed2": "Light poisson demand across all expanded routes using seed 2.",
    "medium_core": "Medium periodic demand on the core route subset.",
    "heavy_all": "Heavy periodic demand across all expanded routes.",
    "oversat_all": "Oversaturated periodic demand across all expanded routes.",
    "west_bottleneck": "Bottleneck demand focused on west inbound routes.",
    "north_bottleneck": "Bottleneck demand focused on north inbound routes.",
    "east_bottleneck": "Bottleneck demand focused on east inbound routes.",
    "south_bottleneck": "Bottleneck demand focused on south inbound routes.",
    "southwest_bottleneck": "Bottleneck demand focused on southwest inbound routes.",
}

GROUPS = {scenario["scenario_name"]: scenario for scenario in ALL_CONGESTION_SCENARIOS}

# Also allow selecting one route by its route id.
for route_id, route_edges in EXPANDED_ROUTES:
    ROUTE_OPTIONS[route_id] = [(route_id, route_edges)]


def build_vehicle_type_attributes(vehicle_type_id: str, lane_change_mode: str) -> dict:
    attributes = {
        "id": vehicle_type_id,
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
    elif lane_change_mode == "change":
        attributes.update(
            {
                "lcStrategic": "1",
                "lcCooperative": "1",
                "lcSpeedGain": "1",
                "lcKeepRight": "1",
                "lcSublane": "1",
            }
        )
    else:
        raise ValueError(f"Unsupported lane_change_mode: {lane_change_mode}")

    return attributes


def build_vehicle_attributes(
    lane_change_mode: str,
    *,
    vehicle_id: str,
    vehicle_type_id: str,
    route_id: str,
    depart: float,
    depart_lane: str,
    depart_speed: str,
    depart_pos: str,
) -> dict:
    attributes = {
        "id": vehicle_id,
        "type": vehicle_type_id,
        "route": route_id,
        "depart": f"{depart:.2f}",
        "departLane": depart_lane,
        "departSpeed": depart_speed,
        "departPos": depart_pos,
    }

    if lane_change_mode == "no-change":
        # Write laneChangeMode on each vehicle rather than the vType so this remains
        # compatible with SUMO versions that do not accept laneChangeMode on <vType>.
        # 517 = 512 + 1 + 4:
        # - keep SUMO safety-gap enforcement;
        # - allow strategic lane changes required to follow the route;
        # - allow cooperative lane changes so target-lane vehicles may help create gaps;
        # - disable speed-gain, keep-right/right-drive, and sublane lane changes.
        attributes["laneChangeMode"] = "517"

    return attributes


def prettify_xml(elem: ET.Element) -> str:
    rough_string = ET.tostring(elem, encoding="utf-8")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="    ")


def generate_uniform_departures(num_vehicles: int, start_time: float, end_time: float):
    if num_vehicles <= 0:
        return []
    if num_vehicles == 1:
        return [start_time]
    step = (end_time - start_time) / (num_vehicles - 1)
    return [start_time + i * step for i in range(num_vehicles)]


def generate_poisson_departures(num_vehicles: int, start_time: float, end_time: float, seed: int):
    duration = end_time - start_time
    if duration <= 0:
        raise ValueError("end_time must be greater than start_time for poisson generation.")
    if num_vehicles <= 0:
        return []

    # Poisson arrival process: exponential headways.
    # Here num_vehicles is the expected number of departure times over the window.
    rate = num_vehicles / duration
    rng = random.Random(seed)
    departures = []
    t = start_time
    while True:
        t += rng.expovariate(rate)
        if t > end_time + 1e-9:
            break
        departures.append(t)
    return departures


def generate_by_period(start_time: float, end_time: float, period: float):
    if period <= 0:
        raise ValueError("period must be > 0.")
    departures = []
    t = start_time
    while t <= end_time + 1e-9:
        departures.append(t)
        t += period
    return departures


def generate_departures_for_scenario(scenario):
    mode = scenario["mode"]
    start_time = scenario["start_time"]
    end_time = scenario["end_time"]

    if end_time < start_time:
        raise ValueError(
            f"Scenario {scenario['scenario_name']} end_time must be >= start_time."
        )

    if mode == "uniform":
        return generate_uniform_departures(
            scenario.get("num_departures", 0), start_time, end_time
        )
    if mode == "poisson":
        return generate_poisson_departures(
            scenario.get("num_departures", 0),
            start_time,
            end_time,
            scenario.get("seed", 42),
        )
    if mode == "period":
        return generate_by_period(start_time, end_time, scenario["period"])

    raise ValueError(f"Unsupported scenario mode: {mode}")


def build_all_congestion_routes_xml(
    scenarios,
    vehicle_type_id="car",
    depart_lane="best",
    depart_speed="max",
    depart_pos="base",
    lane_change_mode="no-change",
):
    root = ET.Element("routes")

    ET.SubElement(
        root,
        "vType",
        **build_vehicle_type_attributes(vehicle_type_id, lane_change_mode),
    )

    for route_id, route_edges in EXPANDED_ROUTES:
        ET.SubElement(root, "route", id=route_id, edges=route_edges)

    vehicles = []
    manifest_rows = []
    scenario_counts = {}

    for scenario in scenarios:
        scenario_name = scenario["scenario_name"]
        route_option = scenario["route_option"]
        selected_routes = ROUTE_OPTIONS[route_option]
        departures = generate_departures_for_scenario(scenario)
        scenario_counts[scenario_name] = len(departures) * len(selected_routes)

        vehicle_idx = 1
        for depart in departures:
            for route_id, _ in selected_routes:
                vehicles.append(
                    {
                        "depart": depart,
                        "scenario_name": scenario_name,
                        "vehicle_id": f"{scenario_name}_veh_{vehicle_idx:06d}",
                        "route_id": route_id,
                    }
                )
                vehicle_idx += 1

        manifest_rows.append(
            {
                "scenario_name": scenario_name,
                "start_time": scenario["start_time"],
                "end_time": scenario["end_time"],
                "route_option": route_option,
                "mode": scenario["mode"],
                "num_departures": scenario.get("num_departures", ""),
                "period": scenario.get("period", ""),
                "seed": scenario.get("seed", ""),
                "selected_route_count": len(selected_routes),
                "vehicle_count": scenario_counts[scenario_name],
                "first_depart": f"{departures[0]:.2f}" if departures else "",
                "last_depart": f"{departures[-1]:.2f}" if departures else "",
            }
        )

    vehicles.sort(
        key=lambda vehicle: (
            vehicle["depart"],
            vehicle["scenario_name"],
            vehicle["vehicle_id"],
        )
    )

    for vehicle in vehicles:
        ET.SubElement(
            root,
            "vehicle",
            **build_vehicle_attributes(
                lane_change_mode,
                vehicle_id=vehicle["vehicle_id"],
                vehicle_type_id=vehicle_type_id,
                route_id=vehicle["route_id"],
                depart=vehicle["depart"],
                depart_lane=depart_lane,
                depart_speed=depart_speed,
                depart_pos=depart_pos,
            ),
        )

    return root, manifest_rows, scenario_counts, vehicles



def scenario_density_label(scenario):
    mode = scenario["mode"]
    if mode in {"uniform", "poisson"}:
        return f"{mode}, num_departures={scenario.get('num_departures', '')}"
    if mode == "period":
        return f"period={scenario['period']}s"
    return mode


def scenario_time_window(scenario):
    return f"{scenario['start_time']}-{scenario['end_time']}"


def select_group_scenarios(group_names):
    scenarios = []
    for group_name in group_names:
        if group_name not in GROUPS:
            available = ", ".join(GROUPS)
            raise ValueError(f"Unknown group '{group_name}'. Available groups: {available}")
        scenarios.append(GROUPS[group_name])
    return scenarios


def default_group_route_path(output_dir, group_name, prefix):
    return os.path.join(output_dir, f"{group_name}_{prefix}.rou.xml")


def default_group_sumocfg_path(output_dir, group_name, prefix):
    return os.path.join(output_dir, f"{group_name}_{prefix}.sumocfg")


def ensure_can_write(path, force):
    if os.path.exists(path) and not force:
        raise FileExistsError(f"Refusing to overwrite existing file without --force: {path}")


def write_route_file(path, root, force=False):
    ensure_can_write(path, force)
    output_dir = os.path.dirname(path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(prettify_xml(root))


def relative_value_from_config(config_path, referenced_path):
    config_dir = os.path.dirname(os.path.abspath(config_path))
    abs_ref = os.path.abspath(referenced_path)
    return os.path.relpath(abs_ref, config_dir)


def build_sumocfg_xml(net_value, route_value, tripinfo_value):
    root = ET.Element("configuration")
    input_elem = ET.SubElement(root, "input")
    ET.SubElement(input_elem, "net-file", value=net_value)
    ET.SubElement(input_elem, "route-files", value=route_value)
    output_elem = ET.SubElement(root, "output")
    ET.SubElement(output_elem, "tripinfo-output", value=tripinfo_value)
    time_elem = ET.SubElement(root, "time")
    ET.SubElement(time_elem, "begin", value="0")
    return root


def write_sumocfg_file(path, group_name, prefix, net_file, route_file, tripinfo_output_dir, force=False):
    ensure_can_write(path, force)
    output_dir = os.path.dirname(path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    tripinfo_path = os.path.join(tripinfo_output_dir, f"tripinfo_{group_name}_{prefix}.xml")
    root = build_sumocfg_xml(
        relative_value_from_config(path, net_file),
        relative_value_from_config(path, route_file),
        relative_value_from_config(path, tripinfo_path),
    )
    with open(path, "w", encoding="utf-8") as f:
        f.write(prettify_xml(root))


def print_group_listing():
    print("Available groups:")
    for name, scenario in GROUPS.items():
        selected_routes = ROUTE_OPTIONS[scenario["route_option"]]
        print(
            f"{name}: {GROUP_DESCRIPTIONS.get(name, '')} "
            f"route_family={scenario['route_option']} routes={len(selected_routes)} "
            f"density={scenario_density_label(scenario)} "
            f"window={scenario_time_window(scenario)} seed={scenario.get('seed', '')}"
        )


def print_group_generation_summary(path, manifest_rows, vehicles):
    print(f"Route file written to: {path}")
    print(f"Number of scenarios: {len(manifest_rows)}")
    print(f"Total vehicles: {len(vehicles)}")
    if vehicles:
        print(f"First depart: {vehicles[0]['depart']:.2f}")
        print(f"Last depart:  {vehicles[-1]['depart']:.2f}")
    for row in manifest_rows:
        print(f"  {row['scenario_name']}: {row['vehicle_count']} vehicles")

def write_manifest(manifest_output, manifest_rows):
    fieldnames = [
        "scenario_name",
        "start_time",
        "end_time",
        "route_option",
        "mode",
        "num_departures",
        "period",
        "seed",
        "selected_route_count",
        "vehicle_count",
        "first_depart",
        "last_depart",
    ]
    with open(manifest_output, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(manifest_rows)


def build_routes_xml(
    departures,
    routes,
    vehicle_type_id="car",
    depart_lane="best",
    depart_speed="max",
    depart_pos="base",
    lane_change_mode="no-change",
):
    root = ET.Element("routes")

    ET.SubElement(
        root,
        "vType",
        **build_vehicle_type_attributes(vehicle_type_id, lane_change_mode),
    )

    for route_id, route_edges in routes:
        ET.SubElement(root, "route", id=route_id, edges=route_edges)
    vehicle_idx = 0
    for depart in departures:
        for route_id, _ in routes:
            ET.SubElement(
                root,
                "vehicle",
                **build_vehicle_attributes(
                    lane_change_mode,
                    vehicle_id=f"veh_{vehicle_idx}",
                    vehicle_type_id=vehicle_type_id,
                    route_id=route_id,
                    depart=depart,
                    depart_lane=depart_lane,
                    depart_speed=depart_speed,
                    depart_pos=depart_pos,
                ),
            )
            vehicle_idx += 1

    return root


def main():
    parser = argparse.ArgumentParser(
        description="Generate SUMO route files with expanded fixed route options."
    )
    parser.add_argument("--output", type=str, default="expanded_routes.rou.xml")
    parser.add_argument("--list-groups", action="store_true", help="Print available congestion groups and exit.")
    parser.add_argument("--group", type=str, help="Generate one named congestion group into a single route file.")
    parser.add_argument("--groups", type=str, help="Comma-separated congestion group names to generate as separate route files.")
    parser.add_argument("--all-groups", action="store_true", help="Generate each known congestion group as a separate route file.")
    parser.add_argument("--output-dir", type=str, default=".", help="Output directory for --groups or --all-groups route files.")
    parser.add_argument("--prefix", type=str, default="no_change_random_offset_seed20260708", help="Filename suffix used for per-group generated files.")
    parser.add_argument("--force", action="store_true", help="Allow overwriting generated route and sumocfg files.")
    parser.add_argument("--write-sumocfg-only", action="store_true", help="Generate matching .sumocfg files only, without route XML.")
    parser.add_argument("--sumocfg", action="store_true", help="Generate matching .sumocfg files alongside generated route files.")
    parser.add_argument("--net-file", type=str, default="data/test_random_offsets_seed20260708.net.xml", help="Network file referenced by generated sumocfg files.")
    parser.add_argument("--tripinfo-output-dir", type=str, default="data", help="Directory used for future tripinfo output paths in sumocfg files.")
    parser.add_argument("--sumocfg-output-dir", type=str, default="data", help="Directory where generated sumocfg files are written.")
    parser.add_argument(
        "--profile",
        type=str,
        choices=["all-congestion"],
        help="Generate a staged route file containing all congestion scenarios.",
    )
    parser.add_argument(
        "--manifest-output",
        type=str,
        default="all_congestion_manifest.csv",
        help="Manifest CSV path for --profile all-congestion.",
    )
    parser.add_argument(
        "--num-vehicles",
        type=int,
        default=20,
        help="Number of departure times. Total vehicles = departure times * number of selected routes. For poisson, this is expected departure count.",
    )
    parser.add_argument("--start-time", type=float, default=0.0)
    parser.add_argument("--end-time", type=float, default=300.0)
    parser.add_argument(
        "--mode",
        type=str,
        choices=["uniform", "poisson", "period"],
        default="uniform",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=10.0,
        help="Used only when mode=period. One departure time every period seconds.",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--route-option",
        type=str,
        choices=sorted(ROUTE_OPTIONS.keys()),
        default="CORE",
        help="Choose a route group such as ORIGINAL, CORE, ALL, WEST_IN, NORTH_IN, EAST_IN, SOUTH_IN, SOUTHWEST_IN, or one specific route id.",
    )
    parser.add_argument(
        "--lane-change-mode",
        type=str,
        choices=["no-change", "change"],
        default="no-change",
        help="Control lane changes: no-change = allow route-required and cooperative lane changes only; change = enable all lane-changing behaviors.",
    )
    parser.add_argument("--depart-lane", type=str, default="best")
    parser.add_argument("--depart-speed", type=str, default="max")
    parser.add_argument("--depart-pos", type=str, default="base")

    args = parser.parse_args()

    if args.list_groups:
        print_group_listing()
        return

    selected_group_names = []
    if args.group:
        selected_group_names.extend([args.group])
    if args.groups:
        selected_group_names.extend([name.strip() for name in args.groups.split(",") if name.strip()])
    if args.all_groups:
        selected_group_names.extend(GROUPS.keys())

    if selected_group_names:
        # Preserve declaration order while avoiding duplicate writes if options overlap.
        selected_group_names = list(dict.fromkeys(selected_group_names))
        scenarios = select_group_scenarios(selected_group_names)

        if args.write_sumocfg_only:
            route_paths = {
                name: default_group_route_path(args.sumocfg_output_dir, name, args.prefix)
                for name in selected_group_names
            }
        elif args.group and len(selected_group_names) == 1:
            route_paths = {selected_group_names[0]: args.output}
        else:
            route_paths = {
                name: default_group_route_path(args.output_dir, name, args.prefix)
                for name in selected_group_names
            }

        if args.write_sumocfg_only:
            for name in selected_group_names:
                sumocfg_path = default_group_sumocfg_path(args.sumocfg_output_dir, name, args.prefix)
                write_sumocfg_file(
                    sumocfg_path,
                    name,
                    args.prefix,
                    args.net_file,
                    route_paths[name],
                    args.tripinfo_output_dir,
                    force=args.force,
                )
                print(f"SUMO configuration written to: {sumocfg_path}")
            return

        for scenario in scenarios:
            name = scenario["scenario_name"]
            root, manifest_rows, scenario_counts, vehicles = build_all_congestion_routes_xml(
                [scenario],
                depart_lane=args.depart_lane,
                depart_speed=args.depart_speed,
                depart_pos=args.depart_pos,
                lane_change_mode=args.lane_change_mode,
            )
            write_route_file(route_paths[name], root, force=args.force)
            print_group_generation_summary(route_paths[name], manifest_rows, vehicles)
            if args.sumocfg:
                sumocfg_path = default_group_sumocfg_path(args.sumocfg_output_dir, name, args.prefix)
                write_sumocfg_file(
                    sumocfg_path,
                    name,
                    args.prefix,
                    args.net_file,
                    route_paths[name],
                    args.tripinfo_output_dir,
                    force=args.force,
                )
                print(f"SUMO configuration written to: {sumocfg_path}")
        return

    if args.profile == "all-congestion":
        root, manifest_rows, scenario_counts, vehicles = build_all_congestion_routes_xml(
            ALL_CONGESTION_SCENARIOS,
            depart_lane=args.depart_lane,
            depart_speed=args.depart_speed,
            depart_pos=args.depart_pos,
            lane_change_mode=args.lane_change_mode,
        )
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(prettify_xml(root))
        write_manifest(args.manifest_output, manifest_rows)

        print(f"Route file written to: {args.output}")
        print(f"Manifest file written to: {args.manifest_output}")
        print(f"Lane change mode: {args.lane_change_mode}")
        print(f"Number of scenarios: {len(ALL_CONGESTION_SCENARIOS)}")
        print(f"Total vehicles: {len(vehicles)}")
        if vehicles:
            print(f"First depart: {vehicles[0]['depart']:.2f}")
            print(f"Last depart:  {vehicles[-1]['depart']:.2f}")
        print("Vehicle count per scenario:")
        for scenario_name, vehicle_count in scenario_counts.items():
            print(f"  {scenario_name}: {vehicle_count}")
        return

    if args.end_time < args.start_time:
        raise ValueError("end-time must be >= start-time.")

    if args.mode == "uniform":
        departures = generate_uniform_departures(args.num_vehicles, args.start_time, args.end_time)
    elif args.mode == "poisson":
        departures = generate_poisson_departures(args.num_vehicles, args.start_time, args.end_time, args.seed)
    else:
        departures = generate_by_period(args.start_time, args.end_time, args.period)

    selected_routes = ROUTE_OPTIONS[args.route_option]
    root = build_routes_xml(
        departures=departures,
        routes=selected_routes,
        depart_lane=args.depart_lane,
        depart_speed=args.depart_speed,
        depart_pos=args.depart_pos,
        lane_change_mode=args.lane_change_mode,
    )

    with open(args.output, "w", encoding="utf-8") as f:
        f.write(prettify_xml(root))

    print(f"Route file written to: {args.output}")
    print(f"Lane change mode: {args.lane_change_mode}")
    print(f"Route option: {args.route_option}")
    print(f"Number of selected routes: {len(selected_routes)}")
    print(f"Number of departure times: {len(departures)}")
    print(f"Number of vehicles: {len(departures) * len(selected_routes)}")
    if departures:
        print(f"First depart: {departures[0]:.2f}")
        print(f"Last depart:  {departures[-1]:.2f}")
    print("Selected routes:")
    for route_id, route_edges in selected_routes:
        print(f"  {route_id}: {route_edges}")


if __name__ == "__main__":
    main()
