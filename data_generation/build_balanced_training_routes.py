#!/usr/bin/env python3
"""Build a capacity-balanced SUMO training route catalog for the supplied net.

The catalog contains exactly one simple path for every reachable, non-return
boundary OD pair.  All routes have equal sampling probability.  Path selection
is solved as a small mixed-integer problem with these priorities:

1. cover every real controlled movement at least once;
2. minimize the largest signal-capacity-normalized movement pressure;
3. balance both directions of every internal physical road and minimize the
   range of internal directed-edge use;
4. reduce remaining movement/edge imbalance while avoiding unnecessary detours.

Requires SciPy >= 1.9 (``scipy.optimize.milp``).
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    import numpy as np
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import csr_matrix, lil_matrix
except ImportError as exc:  # pragma: no cover - depends on the user's environment
    raise SystemExit(
        "This builder requires NumPy and SciPy >= 1.9. "
        "Install them with: python -m pip install 'scipy>=1.9' numpy"
    ) from exc


MovementKey = tuple[str, str]


@dataclass(frozen=True)
class EdgeInfo:
    edge_id: str
    from_node: str
    to_node: str
    lane_count: int


@dataclass(frozen=True)
class MovementInfo:
    from_edge: str
    to_edge: str
    tls_id: str
    link_index: int
    green_seconds: float
    cycle_seconds: float

    @property
    def key(self) -> MovementKey:
        return (self.from_edge, self.to_edge)

    @property
    def green_ratio(self) -> float:
        return self.green_seconds / self.cycle_seconds


@dataclass(frozen=True)
class Candidate:
    od_index: int
    candidate_rank: int
    edges: tuple[str, ...]
    shortest_edge_count: int
    movements: frozenset[MovementKey]
    internal_edges: frozenset[str]

    @property
    def detour_edges(self) -> int:
        return len(self.edges) - self.shortest_edge_count


@dataclass
class NetworkData:
    edges: dict[str, EdgeInfo]
    transitions: dict[str, tuple[str, ...]]
    movements: dict[MovementKey, MovementInfo]
    signal_nodes: set[str]
    inbound_edges: list[str]
    outbound_edges: list[str]

    @property
    def transition_pairs(self) -> set[MovementKey]:
        return {
            (from_edge, to_edge)
            for from_edge, next_edges in self.transitions.items()
            for to_edge in next_edges
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("net_file", type=Path, help="SUMO .net.xml used for training")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("."),
        help="directory for generated files (default: current directory)",
    )
    parser.add_argument(
        "--prefix",
        default="training_balanced_routes",
        help="output filename prefix (default: training_balanced_routes)",
    )
    parser.add_argument(
        "--program-id",
        default="0",
        help="traffic-light programID used for capacity normalization (default: 0)",
    )
    parser.add_argument(
        "--max-extra-edges",
        type=int,
        default=2,
        help="maximum selected/candidate detour above OD shortest path (default: 2)",
    )
    parser.add_argument(
        "--max-path-edges",
        type=int,
        default=10,
        help="hard path-length limit during enumeration (default: 10)",
    )
    parser.add_argument(
        "--saturation-flow-vph",
        type=float,
        default=1800.0,
        help="per-movement saturation-flow proxy for audit metrics (default: 1800)",
    )
    parser.add_argument(
        "--solver-time-limit",
        type=float,
        default=60.0,
        help="time limit in seconds for each MILP stage (default: 60)",
    )
    args = parser.parse_args()
    if args.max_extra_edges < 0:
        parser.error("--max-extra-edges must be non-negative")
    if args.max_path_edges < 2:
        parser.error("--max-path-edges must be at least 2")
    if args.saturation_flow_vph <= 0:
        parser.error("--saturation-flow-vph must be positive")
    return args


def load_network(net_file: Path, program_id: str) -> NetworkData:
    root = ET.parse(net_file).getroot()

    edges: dict[str, EdgeInfo] = {}
    for edge in root.findall("edge"):
        if edge.get("function") is not None:
            continue
        edge_id = required_attr(edge, "id")
        lanes = edge.findall("lane")
        lane_count = len(lanes) or int(edge.get("numLanes", "1"))
        edges[edge_id] = EdgeInfo(
            edge_id=edge_id,
            from_node=required_attr(edge, "from"),
            to_node=required_attr(edge, "to"),
            lane_count=lane_count,
        )

    signal_nodes = {
        required_attr(junction, "id")
        for junction in root.findall("junction")
        if junction.get("type") == "traffic_light"
    }
    if not signal_nodes:
        raise ValueError("No traffic-light junctions were found in the network")

    programs: dict[str, ET.Element] = {}
    for logic in root.findall("tlLogic"):
        if logic.get("programID") == program_id:
            programs[required_attr(logic, "id")] = logic

    transition_sets: dict[str, set[str]] = defaultdict(set)
    movements: dict[MovementKey, MovementInfo] = {}
    for connection in root.findall("connection"):
        from_edge = connection.get("from")
        to_edge = connection.get("to")
        if from_edge not in edges or to_edge not in edges:
            continue
        transition_sets[from_edge].add(to_edge)

        tls_id = connection.get("tl")
        link_index_text = connection.get("linkIndex")
        if tls_id is None or link_index_text is None:
            continue
        key = (from_edge, to_edge)
        if key in movements:
            raise ValueError(
                f"Movement {from_edge} -> {to_edge} has multiple controlled lane "
                "connections; this network-specific builder expects one linkIndex per edge pair"
            )
        logic = programs.get(tls_id)
        if logic is None:
            raise ValueError(
                f"No tlLogic programID={program_id!r} found for controlled junction {tls_id!r}"
            )
        link_index = int(link_index_text)
        phases = logic.findall("phase")
        cycle_seconds = sum(float(phase.get("duration", "0")) for phase in phases)
        green_seconds = 0.0
        for phase in phases:
            state = required_attr(phase, "state")
            if link_index >= len(state):
                raise ValueError(
                    f"tlLogic {tls_id!r} state is shorter than linkIndex {link_index}"
                )
            if state[link_index] in "Gg":
                green_seconds += float(required_attr(phase, "duration"))
        if cycle_seconds <= 0 or green_seconds <= 0:
            raise ValueError(
                f"Real connection {from_edge}->{to_edge} at {tls_id}[{link_index}] "
                "does not receive green in the selected program"
            )
        movements[key] = MovementInfo(
            from_edge=from_edge,
            to_edge=to_edge,
            tls_id=tls_id,
            link_index=link_index,
            green_seconds=green_seconds,
            cycle_seconds=cycle_seconds,
        )

    inbound_edges = sorted(
        edge_id
        for edge_id, edge in edges.items()
        if edge.from_node not in signal_nodes and edge.to_node in signal_nodes
    )
    outbound_edges = sorted(
        edge_id
        for edge_id, edge in edges.items()
        if edge.from_node in signal_nodes and edge.to_node not in signal_nodes
    )
    if not inbound_edges or not outbound_edges:
        raise ValueError(
            "Could not infer boundary edges. This builder expects boundary roads to "
            "connect a non-signal terminal node directly to a signalized junction."
        )

    return NetworkData(
        edges=edges,
        transitions={edge: tuple(sorted(next_edges)) for edge, next_edges in transition_sets.items()},
        movements=movements,
        signal_nodes=signal_nodes,
        inbound_edges=inbound_edges,
        outbound_edges=outbound_edges,
    )


def required_attr(element: ET.Element, name: str) -> str:
    value = element.get(name)
    if value is None:
        raise ValueError(f"Element <{element.tag}> is missing required attribute {name!r}")
    return value


def enumerate_simple_paths(
    network: NetworkData,
    origin: str,
    destination: str,
    max_path_edges: int,
) -> list[tuple[str, ...]]:
    first = network.edges[origin]
    stack: list[tuple[str, tuple[str, ...], frozenset[str]]] = [
        (origin, (origin,), frozenset((first.from_node, first.to_node)))
    ]
    paths: list[tuple[str, ...]] = []
    outbound = set(network.outbound_edges)

    while stack:
        current, path, visited_nodes = stack.pop()
        if current == destination:
            paths.append(path)
            continue
        if len(path) >= max_path_edges:
            continue
        for next_edge in reversed(network.transitions.get(current, ())):
            if next_edge in outbound and next_edge != destination:
                continue
            next_node = network.edges[next_edge].to_node
            if next_node in visited_nodes:
                continue
            stack.append(
                (
                    next_edge,
                    path + (next_edge,),
                    visited_nodes | {next_node},
                )
            )

    return sorted(set(paths), key=lambda path: (len(path), path))


def build_candidates(
    network: NetworkData,
    max_extra_edges: int,
    max_path_edges: int,
) -> tuple[list[tuple[str, str]], list[list[Candidate]]]:
    internal_edges = {
        edge_id
        for edge_id, edge in network.edges.items()
        if edge.from_node in network.signal_nodes and edge.to_node in network.signal_nodes
    }
    od_pairs: list[tuple[str, str]] = []
    candidates_by_od: list[list[Candidate]] = []

    for origin in network.inbound_edges:
        origin_boundary = network.edges[origin].from_node
        for destination in network.outbound_edges:
            destination_boundary = network.edges[destination].to_node
            if origin_boundary == destination_boundary:
                continue  # exclude return-to-the-same-boundary-arm trips
            paths = enumerate_simple_paths(network, origin, destination, max_path_edges)
            if not paths:
                continue
            shortest = len(paths[0])
            paths = [path for path in paths if len(path) <= shortest + max_extra_edges]
            od_index = len(od_pairs)
            od_pairs.append((origin, destination))
            od_candidates: list[Candidate] = []
            for rank, path in enumerate(paths, start=1):
                transition_pairs = frozenset(zip(path, path[1:]))
                missing_connections = transition_pairs - network.transition_pairs
                if missing_connections:
                    raise AssertionError(f"Invalid path transitions: {sorted(missing_connections)}")
                od_candidates.append(
                    Candidate(
                        od_index=od_index,
                        candidate_rank=rank,
                        edges=path,
                        shortest_edge_count=shortest,
                        movements=frozenset(
                            movement for movement in transition_pairs if movement in network.movements
                        ),
                        internal_edges=frozenset(edge for edge in path if edge in internal_edges),
                    )
                )
            candidates_by_od.append(od_candidates)

    if not od_pairs:
        raise ValueError("No reachable non-return boundary OD pairs were found")
    return od_pairs, candidates_by_od


def internal_directed_edges(network: NetworkData) -> list[str]:
    return sorted(
        edge_id
        for edge_id, edge in network.edges.items()
        if edge.from_node in network.signal_nodes and edge.to_node in network.signal_nodes
    )


def reverse_internal_pairs(network: NetworkData, internal_edges: Iterable[str]) -> list[tuple[str, str]]:
    by_nodes = {
        (network.edges[edge_id].from_node, network.edges[edge_id].to_node): edge_id
        for edge_id in internal_edges
    }
    pairs: set[tuple[str, str]] = set()
    for edge_id in internal_edges:
        edge = network.edges[edge_id]
        reverse = by_nodes.get((edge.to_node, edge.from_node))
        if reverse is not None:
            pairs.add(tuple(sorted((edge_id, reverse))))
    return sorted(pairs)


def flatten_candidates(
    candidates_by_od: list[list[Candidate]],
) -> tuple[list[Candidate], list[list[int]]]:
    flat: list[Candidate] = []
    indices_by_od: list[list[int]] = []
    for candidates in candidates_by_od:
        indices: list[int] = []
        for candidate in candidates:
            indices.append(len(flat))
            flat.append(candidate)
        indices_by_od.append(indices)
    return flat, indices_by_od


def add_base_constraints(
    rows: list[dict[int, float]],
    lower: list[float],
    upper: list[float],
    flat: list[Candidate],
    indices_by_od: list[list[int]],
    movement_keys: list[MovementKey],
    reverse_pairs: list[tuple[str, str]],
) -> None:
    for indices in indices_by_od:
        add_constraint(rows, lower, upper, {index: 1.0 for index in indices}, 1.0, 1.0)

    for movement in movement_keys:
        variables = {
            index: 1.0 for index, candidate in enumerate(flat) if movement in candidate.movements
        }
        if not variables:
            raise ValueError(
                f"Movement {movement[0]} -> {movement[1]} cannot be covered by the "
                "candidate paths. Increase --max-extra-edges or --max-path-edges."
            )
        add_constraint(rows, lower, upper, variables, 1.0, math.inf)

    for edge_a, edge_b in reverse_pairs:
        coefficients: dict[int, float] = {}
        for index, candidate in enumerate(flat):
            value = float(edge_a in candidate.internal_edges) - float(
                edge_b in candidate.internal_edges
            )
            if value:
                coefficients[index] = value
        add_constraint(rows, lower, upper, coefficients, 0.0, 0.0)


def add_constraint(
    rows: list[dict[int, float]],
    lower: list[float],
    upper: list[float],
    coefficients: dict[int, float],
    lb: float,
    ub: float,
) -> None:
    rows.append(coefficients)
    lower.append(lb)
    upper.append(ub)


def run_milp(
    objective: np.ndarray,
    integrality: np.ndarray,
    variable_upper: np.ndarray,
    rows: list[dict[int, float]],
    lower: list[float],
    upper: list[float],
    time_limit: float,
    stage_name: str,
):
    matrix = lil_matrix((len(rows), len(objective)), dtype=float)
    for row_index, row in enumerate(rows):
        for column, value in row.items():
            matrix[row_index, column] = value
    result = milp(
        objective,
        integrality=integrality,
        bounds=Bounds(np.zeros(len(objective)), variable_upper),
        constraints=LinearConstraint(csr_matrix(matrix), np.array(lower), np.array(upper)),
        options={"time_limit": time_limit, "mip_rel_gap": 0.0, "presolve": True},
    )
    if not result.success or result.x is None:
        raise RuntimeError(f"MILP {stage_name} failed: {result.message}")
    return result


def optimize_routes(
    network: NetworkData,
    candidates_by_od: list[list[Candidate]],
    time_limit: float,
) -> tuple[list[Candidate], dict[str, float]]:
    flat, indices_by_od = flatten_candidates(candidates_by_od)
    movement_keys = sorted(network.movements)
    internal_edges = internal_directed_edges(network)
    reverse_pairs = reverse_internal_pairs(network, internal_edges)
    candidate_count = len(flat)
    tolerance = 1e-7

    # Stage 1: minimize the maximum count / effective-green-ratio across movements.
    max_pressure_index = candidate_count
    variable_count = candidate_count + 1
    rows: list[dict[int, float]] = []
    lower: list[float] = []
    upper: list[float] = []
    add_base_constraints(
        rows, lower, upper, flat, indices_by_od, movement_keys, reverse_pairs
    )
    for movement in movement_keys:
        green_ratio = network.movements[movement].green_ratio
        coefficients = {
            index: 1.0 / green_ratio
            for index, candidate in enumerate(flat)
            if movement in candidate.movements
        }
        coefficients[max_pressure_index] = -1.0
        add_constraint(rows, lower, upper, coefficients, -math.inf, 0.0)
    objective = np.zeros(variable_count)
    objective[max_pressure_index] = 1.0
    integrality = np.zeros(variable_count)
    integrality[:candidate_count] = 1
    variable_upper = np.full(variable_count, math.inf)
    variable_upper[:candidate_count] = 1.0
    stage1 = run_milp(
        objective,
        integrality,
        variable_upper,
        rows,
        lower,
        upper,
        time_limit,
        "stage 1 (movement minimax)",
    )
    max_pressure = float(stage1.x[max_pressure_index])

    # Stage 2: at the optimal movement limit, minimize max(edge)-min(edge).
    max_edge_index = candidate_count
    min_edge_index = candidate_count + 1
    variable_count = candidate_count + 2
    rows = []
    lower = []
    upper = []
    add_base_constraints(
        rows, lower, upper, flat, indices_by_od, movement_keys, reverse_pairs
    )
    for movement in movement_keys:
        green_ratio = network.movements[movement].green_ratio
        coefficients = {
            index: 1.0 / green_ratio
            for index, candidate in enumerate(flat)
            if movement in candidate.movements
        }
        add_constraint(
            rows, lower, upper, coefficients, -math.inf, max_pressure + tolerance
        )
    for edge_id in internal_edges:
        coefficients = {
            index: 1.0
            for index, candidate in enumerate(flat)
            if edge_id in candidate.internal_edges
        }
        upper_row = dict(coefficients)
        upper_row[max_edge_index] = -1.0
        add_constraint(rows, lower, upper, upper_row, -math.inf, 0.0)
        lower_row = {index: -value for index, value in coefficients.items()}
        lower_row[min_edge_index] = 1.0
        add_constraint(rows, lower, upper, lower_row, -math.inf, 0.0)
    objective = np.zeros(variable_count)
    objective[max_edge_index] = 1.0
    objective[min_edge_index] = -1.0
    for index, candidate in enumerate(flat):
        objective[index] = 1e-5 * candidate.detour_edges + index * 1e-11
    integrality = np.zeros(variable_count)
    integrality[:candidate_count] = 1
    variable_upper = np.full(variable_count, math.inf)
    variable_upper[:candidate_count] = 1.0
    stage2 = run_milp(
        objective,
        integrality,
        variable_upper,
        rows,
        lower,
        upper,
        time_limit,
        "stage 2 (internal-edge range)",
    )
    max_edge_use = float(stage2.x[max_edge_index])
    min_edge_use = float(stage2.x[min_edge_index])

    # Stage 3: minimize residual L1 imbalance within the two optimal limits.
    movement_deviation_start = candidate_count
    edge_deviation_start = movement_deviation_start + len(movement_keys)
    movement_center_index = edge_deviation_start + len(internal_edges)
    edge_center_index = movement_center_index + 1
    variable_count = edge_center_index + 1
    rows = []
    lower = []
    upper = []
    add_base_constraints(
        rows, lower, upper, flat, indices_by_od, movement_keys, reverse_pairs
    )
    for movement in movement_keys:
        green_ratio = network.movements[movement].green_ratio
        coefficients = {
            index: 1.0 / green_ratio
            for index, candidate in enumerate(flat)
            if movement in candidate.movements
        }
        add_constraint(
            rows, lower, upper, coefficients, -math.inf, max_pressure + tolerance
        )
    for edge_id in internal_edges:
        coefficients = {
            index: 1.0
            for index, candidate in enumerate(flat)
            if edge_id in candidate.internal_edges
        }
        add_constraint(
            rows, lower, upper, coefficients, math.ceil(min_edge_use - tolerance),
            math.floor(max_edge_use + tolerance)
        )
    for movement_offset, movement in enumerate(movement_keys):
        green_ratio = network.movements[movement].green_ratio
        load = {
            index: 1.0 / green_ratio
            for index, candidate in enumerate(flat)
            if movement in candidate.movements
        }
        positive = dict(load)
        positive[movement_center_index] = -1.0
        positive[movement_deviation_start + movement_offset] = -1.0
        add_constraint(rows, lower, upper, positive, -math.inf, 0.0)
        negative = {index: -value for index, value in load.items()}
        negative[movement_center_index] = 1.0
        negative[movement_deviation_start + movement_offset] = -1.0
        add_constraint(rows, lower, upper, negative, -math.inf, 0.0)
    for edge_offset, edge_id in enumerate(internal_edges):
        load = {
            index: 1.0
            for index, candidate in enumerate(flat)
            if edge_id in candidate.internal_edges
        }
        positive = dict(load)
        positive[edge_center_index] = -1.0
        positive[edge_deviation_start + edge_offset] = -1.0
        add_constraint(rows, lower, upper, positive, -math.inf, 0.0)
        negative = {index: -value for index, value in load.items()}
        negative[edge_center_index] = 1.0
        negative[edge_deviation_start + edge_offset] = -1.0
        add_constraint(rows, lower, upper, negative, -math.inf, 0.0)

    objective = np.zeros(variable_count)
    objective[
        movement_deviation_start:edge_deviation_start
    ] = 100.0
    objective[edge_deviation_start:movement_center_index] = 1.0
    for index, candidate in enumerate(flat):
        objective[index] = 0.001 * candidate.detour_edges + index * 1e-10
    integrality = np.zeros(variable_count)
    integrality[:candidate_count] = 1
    variable_upper = np.full(variable_count, math.inf)
    variable_upper[:candidate_count] = 1.0
    stage3 = run_milp(
        objective,
        integrality,
        variable_upper,
        rows,
        lower,
        upper,
        time_limit,
        "stage 3 (residual balance)",
    )

    selected: list[Candidate] = []
    for indices in indices_by_od:
        selected_index = max(indices, key=lambda index: stage3.x[index])
        if stage3.x[selected_index] < 0.5:
            raise AssertionError("MILP returned a non-integral OD selection")
        selected.append(flat[selected_index])

    solver_metrics = {
        "optimal_max_capacity_normalized_pressure": max_pressure,
        "optimal_internal_edge_min": min_edge_use,
        "optimal_internal_edge_max": max_edge_use,
        "optimal_internal_edge_range": max_edge_use - min_edge_use,
        "stage1_objective": float(stage1.fun),
        "stage2_objective": float(stage2.fun),
        "stage3_objective": float(stage3.fun),
    }
    return selected, solver_metrics


def validate_selection(
    network: NetworkData,
    od_pairs: list[tuple[str, str]],
    candidates_by_od: list[list[Candidate]],
    selected: list[Candidate],
    max_extra_edges: int,
) -> dict[str, bool]:
    checks: dict[str, bool] = {}
    checks["one_route_per_od"] = len(selected) == len(od_pairs) and all(
        candidate.od_index == index for index, candidate in enumerate(selected)
    )
    checks["all_routes_follow_connections"] = all(
        next_edge in network.transitions.get(edge, ())
        for candidate in selected
        for edge, next_edge in zip(candidate.edges, candidate.edges[1:])
    )
    checks["all_routes_have_simple_junction_paths"] = all(
        route_has_unique_nodes(network, candidate.edges) for candidate in selected
    )
    checks["all_routes_match_od"] = all(
        candidate.edges[0] == od_pairs[index][0]
        and candidate.edges[-1] == od_pairs[index][1]
        for index, candidate in enumerate(selected)
    )
    checks["detour_limit_respected"] = all(
        candidate.detour_edges <= max_extra_edges for candidate in selected
    )
    covered_movements = set().union(*(candidate.movements for candidate in selected))
    checks["all_real_controlled_movements_covered"] = covered_movements == set(
        network.movements
    )
    checks["all_candidate_sets_nonempty"] = all(candidates_by_od)

    origin_counts = Counter(od_pairs[index][0] for index in range(len(selected)))
    destination_counts = Counter(od_pairs[index][1] for index in range(len(selected)))
    checks["origins_exactly_balanced"] = len(set(origin_counts.values())) == 1
    checks["destinations_exactly_balanced"] = len(set(destination_counts.values())) == 1

    internal_counts = Counter(
        edge_id for candidate in selected for edge_id in candidate.internal_edges
    )
    reverse_pairs = reverse_internal_pairs(network, internal_directed_edges(network))
    checks["reverse_internal_directions_exactly_balanced"] = all(
        internal_counts[edge_a] == internal_counts[edge_b] for edge_a, edge_b in reverse_pairs
    )
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise AssertionError(f"Generated route catalog failed validation: {', '.join(failed)}")
    return checks


def route_has_unique_nodes(network: NetworkData, route: tuple[str, ...]) -> bool:
    nodes = [network.edges[route[0]].from_node]
    nodes.extend(network.edges[edge_id].to_node for edge_id in route)
    return len(nodes) == len(set(nodes))


def route_id(index: int) -> str:
    return f"train_balanced_od_{index + 1:03d}"


def write_route_xml(path: Path, selected: list[Candidate]) -> None:
    root = ET.Element("routes")
    root.append(
        ET.Comment(
            " Equal-weight, capacity-balanced training catalog. "
            "Use route='train_balanced_all' for probabilistic sampling. "
        )
    )
    for index, candidate in enumerate(selected):
        ET.SubElement(
            root,
            "route",
            {"id": route_id(index), "edges": " ".join(candidate.edges)},
        )
    distribution = ET.SubElement(root, "routeDistribution", {"id": "train_balanced_all"})
    for index in range(len(selected)):
        ET.SubElement(
            distribution,
            "route",
            {"refId": route_id(index), "probability": "1"},
        )
    ET.indent(root, space="    ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def write_manifest_csv(
    path: Path,
    network: NetworkData,
    od_pairs: list[tuple[str, str]],
    candidates_by_od: list[list[Candidate]],
    selected: list[Candidate],
) -> None:
    probability = 1.0 / len(selected)
    fieldnames = [
        "route_id",
        "origin_edge",
        "destination_edge",
        "origin_boundary_node",
        "destination_boundary_node",
        "probability",
        "candidate_rank",
        "candidate_count_for_od",
        "edge_count",
        "shortest_edge_count",
        "detour_edges",
        "controlled_movement_count",
        "internal_edge_count",
        "edges",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, candidate in enumerate(selected):
            origin, destination = od_pairs[index]
            writer.writerow(
                {
                    "route_id": route_id(index),
                    "origin_edge": origin,
                    "destination_edge": destination,
                    "origin_boundary_node": network.edges[origin].from_node,
                    "destination_boundary_node": network.edges[destination].to_node,
                    "probability": f"{probability:.12f}",
                    "candidate_rank": candidate.candidate_rank,
                    "candidate_count_for_od": len(candidates_by_od[index]),
                    "edge_count": len(candidate.edges),
                    "shortest_edge_count": candidate.shortest_edge_count,
                    "detour_edges": candidate.detour_edges,
                    "controlled_movement_count": len(candidate.movements),
                    "internal_edge_count": len(candidate.internal_edges),
                    "edges": " ".join(candidate.edges),
                }
            )


def build_audit(
    net_file: Path,
    network: NetworkData,
    od_pairs: list[tuple[str, str]],
    candidates_by_od: list[list[Candidate]],
    selected: list[Candidate],
    checks: dict[str, bool],
    solver_metrics: dict[str, float],
    args: argparse.Namespace,
) -> dict:
    route_count = len(selected)
    route_probability = 1.0 / route_count
    origin_counts = Counter(origin for origin, _ in od_pairs)
    destination_counts = Counter(destination for _, destination in od_pairs)
    movement_counts = Counter(
        movement for candidate in selected for movement in candidate.movements
    )
    internal_counts = Counter(
        edge_id for candidate in selected for edge_id in candidate.internal_edges
    )
    internal_edges = internal_directed_edges(network)
    internal_count_values = [internal_counts[edge_id] for edge_id in internal_edges]

    movement_details = []
    capacity_limited_total_demands = []
    normalized_pressures = []
    for movement in sorted(network.movements):
        info = network.movements[movement]
        count = movement_counts[movement]
        capacity_vph = args.saturation_flow_vph * info.green_ratio
        normalized_pressure = count / info.green_ratio
        normalized_pressures.append(normalized_pressure)
        total_demand_at_vc1 = route_count * capacity_vph / count
        capacity_limited_total_demands.append(total_demand_at_vc1)
        movement_details.append(
            {
                "from_edge": info.from_edge,
                "to_edge": info.to_edge,
                "tls_id": info.tls_id,
                "link_index": info.link_index,
                "route_count": count,
                "route_probability_share": count * route_probability,
                "green_seconds": info.green_seconds,
                "cycle_seconds": info.cycle_seconds,
                "green_ratio": info.green_ratio,
                "capacity_proxy_vph": capacity_vph,
                "capacity_normalized_pressure": normalized_pressure,
                "uniform_pool_total_demand_vph_at_proxy_vc1": total_demand_at_vc1,
            }
        )

    junction_summary: dict[str, dict] = {}
    for tls_id in sorted({info.tls_id for info in network.movements.values()}):
        details = [item for item in movement_details if item["tls_id"] == tls_id]
        junction_summary[tls_id] = {
            "movement_count": len(details),
            "covered_movement_count": sum(item["route_count"] > 0 for item in details),
            "min_route_count": min(item["route_count"] for item in details),
            "max_route_count": max(item["route_count"] for item in details),
            "mean_route_count": statistics.fmean(item["route_count"] for item in details),
        }

    physical_segments: dict[str, dict] = {}
    for edge_a, edge_b in reverse_internal_pairs(network, internal_edges):
        nodes = sorted(
            (network.edges[edge_a].from_node, network.edges[edge_a].to_node)
        )
        segment_id = f"{nodes[0]}--{nodes[1]}"
        physical_segments[segment_id] = {
            "directions": {edge_a: internal_counts[edge_a], edge_b: internal_counts[edge_b]},
            "total_route_traversals": internal_counts[edge_a] + internal_counts[edge_b],
            "direction_difference": abs(internal_counts[edge_a] - internal_counts[edge_b]),
        }

    route_lengths = [len(candidate.edges) for candidate in selected]
    detours = [candidate.detour_edges for candidate in selected]
    audit = {
        "metadata": {
            "source_net": net_file.name,
            "source_net_sha256": sha256_file(net_file),
            "builder": Path(__file__).name,
            "program_id": args.program_id,
            "max_extra_edges": args.max_extra_edges,
            "max_path_edges": args.max_path_edges,
            "saturation_flow_proxy_vph_per_movement": args.saturation_flow_vph,
            "scipy_version": __import__("scipy").__version__,
        },
        "network": {
            "signalized_junction_count": len(network.signal_nodes),
            "inbound_boundary_edges": network.inbound_edges,
            "outbound_boundary_edges": network.outbound_edges,
            "real_controlled_movement_count": len(network.movements),
            "internal_directed_edges": internal_edges,
        },
        "route_pool": {
            "route_count": route_count,
            "reachable_non_return_od_count": len(od_pairs),
            "candidate_path_count": sum(len(items) for items in candidates_by_od),
            "equal_route_probability": route_probability,
            "route_distribution_id": "train_balanced_all",
            "origin_route_counts": dict(sorted(origin_counts.items())),
            "destination_route_counts": dict(sorted(destination_counts.items())),
            "origin_probability_shares": {
                edge: count * route_probability for edge, count in sorted(origin_counts.items())
            },
            "destination_probability_shares": {
                edge: count * route_probability
                for edge, count in sorted(destination_counts.items())
            },
            "route_edge_count": {
                "min": min(route_lengths),
                "max": max(route_lengths),
                "mean": statistics.fmean(route_lengths),
                "distribution": dict(sorted(Counter(route_lengths).items())),
            },
            "detour_edge_count_distribution": dict(sorted(Counter(detours).items())),
        },
        "balance": {
            "movement_coverage": {
                "covered": len(movement_counts),
                "total": len(network.movements),
                "ratio": len(movement_counts) / len(network.movements),
            },
            "movement_route_count": {
                "min": min(movement_counts.values()),
                "max": max(movement_counts.values()),
                "mean": statistics.fmean(movement_counts.values()),
                "distribution": dict(sorted(Counter(movement_counts.values()).items())),
            },
            "capacity_normalized_movement_pressure": {
                "min": min(normalized_pressures),
                "max": max(normalized_pressures),
                "mean": statistics.fmean(normalized_pressures),
                "population_stddev": statistics.pstdev(normalized_pressures),
                "coefficient_of_variation": coefficient_of_variation(normalized_pressures),
            },
            "internal_directed_edge_route_count": {
                "counts": {edge: internal_counts[edge] for edge in internal_edges},
                "min": min(internal_count_values),
                "max": max(internal_count_values),
                "mean": statistics.fmean(internal_count_values),
                "population_stddev": statistics.pstdev(internal_count_values),
                "coefficient_of_variation": coefficient_of_variation(internal_count_values),
            },
            "physical_segments": physical_segments,
            "junction_summary": junction_summary,
            "uniform_pool_capacity_proxy": {
                "total_demand_vph_at_first_movement_vc1": min(
                    capacity_limited_total_demands
                ),
                "interpretation": (
                    "Theoretical screening value only: equal route probabilities, "
                    "1800 veh/h saturation-flow proxy, effective green ratio, no spillback."
                ),
            },
        },
        "solver": solver_metrics,
        "validation": checks,
        "movement_details": movement_details,
    }
    return audit


def coefficient_of_variation(values: Iterable[float]) -> float:
    materialized = list(values)
    mean = statistics.fmean(materialized)
    return statistics.pstdev(materialized) / mean if mean else 0.0


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_report(
    path: Path,
    audit: dict,
    route_xml_name: str,
    manifest_name: str,
    audit_name: str,
) -> None:
    route_pool = audit["route_pool"]
    balance = audit["balance"]
    internal = balance["internal_directed_edge_route_count"]
    movement = balance["movement_route_count"]
    capacity = balance["capacity_normalized_movement_pressure"]
    lines = [
        "# 训练专用均衡路线集合校验报告",
        "",
        "## 设计口径",
        "",
        "- 对每个可达、非原路返回的边界 OD 选择一条简单路径。",
        "- 所有路线等权；入口、出口以及 OD 采样分布均由路线池直接控制。",
        "- 72 个真实受控转向必须全部被至少一条路线覆盖。",
        "- 转向压力按有效绿灯比例归一化；内部双向道路要求两个方向使用次数完全一致。",
        f"- 候选路径最多比对应 OD 的最短路径多 {audit['metadata']['max_extra_edges']} 条 edge，且不得重复经过 junction。",
        "",
        "## 核心结果",
        "",
        "| 指标 | 结果 |",
        "|---|---:|",
        f"| 入口 / 出口 | {len(audit['network']['inbound_boundary_edges'])} / {len(audit['network']['outbound_boundary_edges'])} |",
        f"| 有效非回返 OD / 路线数 | {route_pool['reachable_non_return_od_count']} / {route_pool['route_count']} |",
        f"| 每条路线概率 | {route_pool['equal_route_probability']:.6%} |",
        f"| 每个入口概率 | {next(iter(route_pool['origin_probability_shares'].values())):.2%} |",
        f"| 每个出口概率 | {next(iter(route_pool['destination_probability_shares'].values())):.2%} |",
        f"| 真实受控转向覆盖 | {balance['movement_coverage']['covered']} / {balance['movement_coverage']['total']} ({balance['movement_coverage']['ratio']:.1%}) |",
        f"| 单个转向被路线覆盖次数 | {movement['min']}–{movement['max']} |",
        f"| 内部有向 edge 使用次数 | {internal['min']}–{internal['max']}，CV={internal['coefficient_of_variation']:.3f} |",
        f"| 容量归一化转向压力 CV | {capacity['coefficient_of_variation']:.3f} |",
        f"| 路线 edge 数 | {route_pool['route_edge_count']['min']}–{route_pool['route_edge_count']['max']}，均值 {route_pool['route_edge_count']['mean']:.3f} |",
        "",
        "## 边界分布",
        "",
        "| 入口 edge | 路线数 | 概率 | 出口 edge | 路线数 | 概率 |",
        "|---|---:|---:|---|---:|---:|",
    ]
    origins = list(route_pool["origin_route_counts"].items())
    destinations = list(route_pool["destination_route_counts"].items())
    for (origin, origin_count), (destination, destination_count) in zip(origins, destinations):
        lines.append(
            f"| {origin} | {origin_count} | {route_pool['origin_probability_shares'][origin]:.2%} "
            f"| {destination} | {destination_count} | {route_pool['destination_probability_shares'][destination]:.2%} |"
        )

    lines.extend(
        [
            "",
            "## 内部有向路段压力",
            "",
            "| edge | 路线使用次数 |",
            "|---|---:|",
        ]
    )
    for edge_id, count in internal["counts"].items():
        lines.append(f"| {edge_id} | {count} |")

    lines.extend(
        [
            "",
            "## 各信号路口转向覆盖",
            "",
            "| TLS | 覆盖 | 路线次数范围 | 平均次数 |",
            "|---|---:|---:|---:|",
        ]
    )
    for tls_id, summary in balance["junction_summary"].items():
        lines.append(
            f"| {tls_id} | {summary['covered_movement_count']}/{summary['movement_count']} "
            f"| {summary['min_route_count']}–{summary['max_route_count']} "
            f"| {summary['mean_route_count']:.3f} |"
        )

    capacity_limit = balance["uniform_pool_capacity_proxy"][
        "total_demand_vph_at_first_movement_vc1"
    ]
    lines.extend(
        [
            "",
            "## 使用方式",
            "",
            f"- `{route_xml_name}` 中已定义 {route_pool['route_count']} 条路线和等权 `train_balanced_all` routeDistribution。车辆可直接设置 `route=\"train_balanced_all\"`。",
            "- 若需要短时间窗内也严格均衡，不要只依赖随机抽样：将 CSV 中 90 个 route_id 每轮洗牌后各使用一次，并给不同入口独立生成发车时刻。",
            "- 不要像旧生成器的 ALL 模式那样，让所有路线在同一时间戳同时发车；这会制造非自然的 90 车脉冲。",
            f"- 在等权假设、每个转向饱和流率 {audit['metadata']['saturation_flow_proxy_vph_per_movement']:.0f} veh/h、仅按有效绿灯比例折算且不考虑溢出时，首个转向达到代理 v/c=1 的全网总需求约为 {capacity_limit:.0f} veh/h。该值只用于需求档位初筛，最终档位必须由 SUMO 实测标定。",
            "",
            "## 输出文件",
            "",
            f"- `{route_xml_name}`：SUMO 路线定义与等权路线分布。",
            f"- `{manifest_name}`：逐路线 OD、路径、权重和绕行信息。",
            f"- `{audit_name}`：逐 movement 容量、覆盖次数及全部机器可读校验结果。",
            "",
            "所有 validation 项均已通过。",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    net_file = args.net_file.resolve()
    if not net_file.is_file():
        raise SystemExit(f"Network file does not exist: {net_file}")
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    network = load_network(net_file, args.program_id)
    od_pairs, candidates_by_od = build_candidates(
        network, args.max_extra_edges, args.max_path_edges
    )
    selected, solver_metrics = optimize_routes(
        network, candidates_by_od, args.solver_time_limit
    )
    checks = validate_selection(
        network, od_pairs, candidates_by_od, selected, args.max_extra_edges
    )

    route_xml = output_dir / f"{args.prefix}.rou.xml"
    manifest_csv = output_dir / f"{args.prefix}.csv"
    audit_json = output_dir / f"{args.prefix}_audit.json"
    report_md = output_dir / f"{args.prefix}_report.md"
    write_route_xml(route_xml, selected)
    write_manifest_csv(
        manifest_csv, network, od_pairs, candidates_by_od, selected
    )
    audit = build_audit(
        net_file,
        network,
        od_pairs,
        candidates_by_od,
        selected,
        checks,
        solver_metrics,
        args,
    )
    audit_json.write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    write_report(
        report_md,
        audit,
        route_xml.name,
        manifest_csv.name,
        audit_json.name,
    )

    print(f"Generated {len(selected)} balanced routes for {len(od_pairs)} OD pairs")
    print(
        f"Movement coverage: {audit['balance']['movement_coverage']['covered']}/"
        f"{audit['balance']['movement_coverage']['total']}"
    )
    print(
        "Internal directed-edge range: "
        f"{audit['balance']['internal_directed_edge_route_count']['min']}.."
        f"{audit['balance']['internal_directed_edge_route_count']['max']}"
    )
    for output in (route_xml, manifest_csv, audit_json, report_md):
        print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
