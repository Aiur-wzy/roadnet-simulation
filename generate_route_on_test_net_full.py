import argparse
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

# Also allow selecting one route by its route id.
for route_id, route_edges in EXPANDED_ROUTES:
    ROUTE_OPTIONS[route_id] = [(route_id, route_edges)]


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


def build_routes_xml(
    departures,
    routes,
    vehicle_type_id="car",
    depart_lane="best",
    depart_speed="max",
    depart_pos="base",
):
    root = ET.Element("routes")

    ET.SubElement(
        root,
        "vType",
        id=vehicle_type_id,
        accel="2.6",
        decel="4.5",
        sigma="0.5",
        length="5.0",
        minGap="2.5",
        maxSpeed="13.89",
        guiShape="passenger",
    )

    for route_id, route_edges in routes:
        ET.SubElement(root, "route", id=route_id, edges=route_edges)

    vehicle_idx = 0
    for depart in departures:
        for route_id, _ in routes:
            ET.SubElement(
                root,
                "vehicle",
                id=f"veh_{vehicle_idx}",
                type=vehicle_type_id,
                route=route_id,
                depart=f"{depart:.2f}",
                departLane=depart_lane,
                departSpeed=depart_speed,
                departPos=depart_pos,
            )
            vehicle_idx += 1

    return root


def main():
    parser = argparse.ArgumentParser(
        description="Generate SUMO route files with expanded fixed route options."
    )
    parser.add_argument("--output", type=str, default="expanded_routes.rou.xml")
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
    parser.add_argument("--depart-lane", type=str, default="best")
    parser.add_argument("--depart-speed", type=str, default="max")
    parser.add_argument("--depart-pos", type=str, default="base")

    args = parser.parse_args()

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
    )

    with open(args.output, "w", encoding="utf-8") as f:
        f.write(prettify_xml(root))

    print(f"Route file written to: {args.output}")
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
