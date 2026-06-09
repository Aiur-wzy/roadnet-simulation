import argparse
import random
import xml.etree.ElementTree as ET
from xml.dom import minidom


ROUTE_OPTIONS = {
    "E9": [("fixed_route_e9", "E9 -E12 E20 E15 -E0 E5")],
    "E9E10": [
        ("fixed_route_e9", "E9 -E12 E20 E15 -E0 E5"),
        ("fixed_route_e10", "E10 -E12 E20 E15 -E0 E5"),
    ],
}


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

    # 泊松到达过程对应指数分布的车头时距。
    # 这里将 num_vehicles 解释为时间区间内的期望车辆数。
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
        ET.SubElement(
            root,
            "route",
            id=route_id,
            edges=route_edges,
        )

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
        description="Generate SUMO routes for fixed paths ending with -E12 -> E20 -> E15."
    )

    parser.add_argument("--output", type=str, default="fixed_route.rou.xml",
                        help="Output route file name.")
    parser.add_argument("--num-vehicles", type=int, default=20,
                        help="Total number of vehicles to generate. For poisson mode, this is the expected number over the time window.")
    parser.add_argument("--start-time", type=float, default=0.0,
                        help="Start departure time in seconds.")
    parser.add_argument("--end-time", type=float, default=300.0,
                        help="End departure time in seconds.")
    parser.add_argument("--mode", type=str, choices=["uniform", "poisson", "period"], default="uniform",
                        help="Departure generation mode.")
    parser.add_argument("--period", type=float, default=5.0,
                        help="Used only when mode=period. One vehicle every 'period' seconds.")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for poisson mode.")
    parser.add_argument("--route-option", type=str, choices=list(ROUTE_OPTIONS.keys()), default="E9",
                        help="Select E9 only, or E9 and E10 together.")
    parser.add_argument("--depart-lane", type=str, default="best",
                        help="SUMO departLane setting.")
    parser.add_argument("--depart-speed", type=str, default="max",
                        help="SUMO departSpeed setting.")
    parser.add_argument("--depart-pos", type=str, default="base",
                        help="SUMO departPos setting.")

    args = parser.parse_args()

    if args.end_time < args.start_time:
        raise ValueError("end-time must be >= start-time.")

    if args.mode == "uniform":
        departures = generate_uniform_departures(
            num_vehicles=args.num_vehicles,
            start_time=args.start_time,
            end_time=args.end_time,
        )
    elif args.mode == "poisson":
        departures = generate_poisson_departures(
            num_vehicles=args.num_vehicles,
            start_time=args.start_time,
            end_time=args.end_time,
            seed=args.seed,
        )
    else:  # period
        departures = generate_by_period(
            start_time=args.start_time,
            end_time=args.end_time,
            period=args.period,
        )

    selected_routes = ROUTE_OPTIONS[args.route_option]

    root = build_routes_xml(
        departures=departures,
        routes=selected_routes,
        depart_lane=args.depart_lane,
        depart_speed=args.depart_speed,
        depart_pos=args.depart_pos,
    )

    xml_string = prettify_xml(root)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(xml_string)

    print(f"Route file written to: {args.output}")
    print(f"Route option: {args.route_option}")
    print(f"Number of routes: {len(selected_routes)}")
    print(f"Number of vehicles: {len(departures) * len(selected_routes)}")
    if departures:
        print(f"First depart: {departures[0]:.2f}")
        print(f"Last depart:  {departures[-1]:.2f}")
    for route_id, route_edges in selected_routes:
        print(f"{route_id}: {route_edges}")


if __name__ == "__main__":
    main()
