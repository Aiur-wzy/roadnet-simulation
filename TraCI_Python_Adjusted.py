import os
import sys
import csv
import argparse
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple
 
'''
# 替换以下路径为你的 SUMO 的 tools 目录路径
sumo_tools_path = '/$SUMO_HOME/tools'

# 检查 SUMO tools 目录是否已经在 sys.path 中
if sumo_tools_path not in sys.path:
    sys.path.append(sumo_tools_path)

# 现在尝试导入 traci
try:
    import traci
except ModuleNotFoundError:
    print("traci module not found. Ensure SUMO tools path is correctly added to Python path.")
    sys.exit(1)
'''

# 获取 SUMO_HOME 环境变量的值
sumo_home = os.environ.get('SUMO_HOME')
if sumo_home is None:
    print("SUMO_HOME environment variable not set.")
    sys.exit(1)

# 构建 SUMO tools 目录路径
sumo_tools_path = os.path.join(sumo_home, 'tools')

# 检查并添加到 sys.path
if sumo_tools_path not in sys.path:
    sys.path.append(sumo_tools_path)

# 尝试导入 traci
try:
    import traci
except ModuleNotFoundError:
    print("traci module not found. Ensure SUMO tools path is correctly added to Python path.")
    sys.exit(1)

import xml.etree.ElementTree as ET

# 获取当前 edge 和下一个 edge 的转向方向
def get_next_edge_direction(vehicle_id, current_edge, connections):
    route = traci.vehicle.getRoute(vehicle_id)
    current_edge_index = route.index(current_edge)
    # print(f"current_edge_index={current_edge_index}")

    if current_edge_index < len(route) - 1:
        next_edge = route[current_edge_index + 1]
        # 查找和下一个 edge 的连接方向
        if current_edge in connections:
            for conn in connections[current_edge]:
                if conn['to'] == next_edge:
                    return conn['dir']                    
        else:
            print(f"Error. {current_edge} does not be found in dic.")
            return "unknown"
    else:
        return "end"
    
    # 如果没有找到
    print(f"Error. current edge {current_edge} does not find next edge {next_edge} in dic.")
    return "unknown"
    
# 提取 edge 的限速，长度，两个 edge 之间的连接关系
def parse_sumo_network(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # 映射1: Edge属性
    edge_attributes = {}
    for edge in root.findall('edge'):
        if 'function' not in edge.attrib:  # 忽略内部edge
            edge_id = edge.attrib['id']
            lanes = edge.findall('lane')
            total_speed = sum(float(lane.attrib['speed']) for lane in lanes)
            total_length = sum(float(lane.attrib['length']) for lane in lanes)
            avg_speed = total_speed / len(lanes) if lanes else 0
            avg_length = total_length / len(lanes) if lanes else 0
            edge_attributes[edge_id] = {'lanes': len(lanes), 'avg_speed': avg_speed, 'avg_length': avg_length}

    # 映射2: Edge连接
    edge_connections = {}
    for connection in root.findall('connection'):
        from_edge = connection.attrib['from']
        to_edge = connection.attrib['to']
        direction = connection.attrib.get('dir', 'unknown')  # 默认值为'unknown'
        if from_edge not in edge_connections:
            edge_connections[from_edge] = []
        edge_connections[from_edge].append({'to': to_edge, 'dir': direction})

    return edge_attributes, edge_connections

# 更新字典 traffic_data 下的信息
def update_vehicle_data(traffic_data, v_id, current_edge, data):
    if v_id not in traffic_data:
        traffic_data[v_id] = {}
    traffic_data[v_id][current_edge] = data

def process_single_vehicle_data(row_data, turn_stat_col_name):
    
    # 转换方向代码
    direction_mapping = {
        's': 'straight',
        't': 'turn',
        'l': 'left',
        'r': 'right',
        'L': 'partially_left',
        'R': 'partially_right',
        'end': 'end_road'
    }

    # 初始化新的转向统计字段
    for direction in direction_mapping.values():
        row_data[direction] = 0
        
    # 假设 Turn_Stat 是一个列表
    turns = row_data[turn_stat_col_name]
    for turn in turns:
        turn_col = direction_mapping.get(turn, 'invalid')
        row_data[turn_col] += 1

    return row_data

# 创建类储存车辆的信息
class Vehicle:
    def __init__(self, id):
        self.id = id
        self.current_edge = None
        self.edge_enter_time = None
        # 行驶时间
        self.driving_time = 0
        # 停止 + 红灯 -> 正常等待时间
        self.is_waiting = False
        self.red_light_waiting_time = 0
        # 停止 + 绿灯 -> 异常等待时间
        self.delay_time = 0
        # 低速 -> 加减速等待时间
        self.lowSpee_time = 0
        #
        self.wait_sum = 0


MovementKey = Tuple[str, int]


@dataclass
class CamsConfig:
    """Configurable assumptions for CAMS-oriented SUMO data extraction.

    The most important manual setting is ``waiting_region_length``. For example,
    if it is 80 m, a vehicle on ``-E12`` approaching ``E20`` is treated as
    entering the downstream waiting region once its distance to the signalized
    stop line is <= 80 m. Before that point, it belongs to the running region.
    """

    sumo_config: str
    net_file: str
    sumo_binary: str
    vehicle_event_output: str
    cycle_summary_output: str
    legacy_edge_output: str
    waiting_region_length: float
    stop_speed_threshold: float
    queue_speed_threshold: float
    downstream_block_occupancy_threshold: float


@dataclass
class NetworkInfo:
    """Static road-network information parsed from the SUMO net.xml file."""

    edge_lanes: Dict[str, List[str]]
    lane_lengths: Dict[str, float]
    connection_dirs: Dict[Tuple[str, str], str]


@dataclass
class VehicleSnapshot:
    """One vehicle's current signalized movement at the current SUMO step."""

    vehicle_id: str
    route_id: str
    route_edges: Tuple[str, ...]
    route_index: int
    from_edge: str
    to_edge: str
    lane_id: str
    tls_id: str
    link_index: int
    signal_state: str
    tls_distance: float
    speed: float
    downstream_occupancy: float
    available_storage_meter: float
    turn_direction: str
    vehicle_length: float


@dataclass
class VehicleMovementEvent:
    """Ground-truth event for one vehicle passing one signalized movement.

    Example:
        For route ``E9 -E12 E20 E15 -E0 E5``, when a vehicle is on ``-E12``
        and will enter ``E20``, the event has:
        ``from_edge=-E12``, ``to_edge=E20``, and ``link_index`` equal to the
        SUMO signal index controlling that exact connection.
    """

    vehicle_id: str
    route_id: str
    route_edges: Tuple[str, ...]
    route_index: int
    from_edge: str
    to_edge: str
    lane_id: str
    tls_id: str
    link_index: int
    edge_enter_time: float
    turn_direction: str
    vehicle_length: float
    signal_state_at_arrival: Optional[str] = None
    waiting_region_enter_time: Optional[float] = None
    pass_stopline_time: Optional[float] = None
    downstream_enter_time: Optional[float] = None
    red_wait_time: float = 0.0
    green_queue_wait_time: float = 0.0
    downstream_block_wait_time: float = 0.0
    cycles_waited: int = 0
    queue_rank_at_arrival: Optional[int] = None
    queue_at_green_start: Optional[int] = None
    pass_cycle_id: Optional[int] = None
    downstream_occupancy_at_arrival: Optional[float] = None
    available_storage_meter_at_arrival: Optional[float] = None
    last_update_time: Optional[float] = None


@dataclass
class MovementCycleSummary:
    """Summary of one green wave for a specific signalized movement."""

    tls_id: str
    link_index: int
    cycle_id: int
    green_start_time: float
    queue_at_green_start: int
    green_end_time: Optional[float] = None
    arrivals_during_green: int = 0
    discharged_in_green: int = 0
    residual_queue_after_green: int = 0
    downstream_blocked_vehicle_ids: Set[str] = field(default_factory=set)
    max_queue_length_in_cycle: int = 0
    queue_length_sum: int = 0
    queue_sample_count: int = 0

    def update_queue(self, queue_length: int) -> None:
        """Accumulate queue length samples inside this green window."""

        self.max_queue_length_in_cycle = max(self.max_queue_length_in_cycle, queue_length)
        self.queue_length_sum += queue_length
        self.queue_sample_count += 1

    @property
    def mean_queue_length_in_cycle(self) -> float:
        """Return the sampled mean queue length for this green window."""

        if self.queue_sample_count == 0:
            return 0.0
        return self.queue_length_sum / self.queue_sample_count

    @property
    def downstream_blocked_count(self) -> int:
        """Return unique vehicles blocked by downstream storage in this green."""

        return len(self.downstream_blocked_vehicle_ids)


def parse_args() -> CamsConfig:
    """Parse command-line parameters for CAMS data extraction."""

    parser = argparse.ArgumentParser(
        description="Collect signal-aware SUMO ground truth for CAMS validation."
    )
    parser.add_argument("--sumo-config", default="map.sumo.cfg", help="SUMO cfg file.")
    parser.add_argument("--net-file", default="test.net.xml", help="SUMO network file.")
    parser.add_argument("--sumo-binary", default="sumo", help="SUMO binary, e.g., sumo or sumo-gui.")
    parser.add_argument(
        "--vehicle-event-output",
        default="cams_vehicle_movement_events.csv",
        help="Output CSV for vehicle-movement ground-truth events.",
    )
    parser.add_argument(
        "--cycle-summary-output",
        default="cams_movement_cycle_summary.csv",
        help="Output CSV for movement-level green-window summaries.",
    )
    parser.add_argument(
        "--legacy-edge-output",
        default="TraCI_output_adjusted.csv",
        help="Compatibility output containing a compact vehicle-edge summary.",
    )
    parser.add_argument(
        "--waiting-region-length",
        type=float,
        default=80.0,
        help=(
            "Meters before a signalized stop line treated as the waiting region. "
            "Example: 80 means the last 80 m of -E12 before E20 is no longer "
            "running region, but intersection waiting region."
        ),
    )
    parser.add_argument(
        "--stop-speed-threshold",
        type=float,
        default=0.1,
        help="Speed <= this value is treated as stopped, in m/s.",
    )
    parser.add_argument(
        "--queue-speed-threshold",
        type=float,
        default=2.0,
        help="Speed <= this value inside the waiting region is treated as queued, in m/s.",
    )
    parser.add_argument(
        "--downstream-block-occupancy-threshold",
        type=float,
        default=85.0,
        help="Downstream lane occupancy percentage above which storage is treated as blocked.",
    )
    args = parser.parse_args()
    return CamsConfig(
        sumo_config=args.sumo_config,
        net_file=args.net_file,
        sumo_binary=args.sumo_binary,
        vehicle_event_output=args.vehicle_event_output,
        cycle_summary_output=args.cycle_summary_output,
        legacy_edge_output=args.legacy_edge_output,
        waiting_region_length=args.waiting_region_length,
        stop_speed_threshold=args.stop_speed_threshold,
        queue_speed_threshold=args.queue_speed_threshold,
        downstream_block_occupancy_threshold=args.downstream_block_occupancy_threshold,
    )


def parse_network_info(file_path: str) -> NetworkInfo:
    """Parse lane lists, lane lengths, and connection directions from net.xml."""

    tree = ET.parse(file_path)
    root = tree.getroot()
    edge_lanes: Dict[str, List[str]] = {}
    lane_lengths: Dict[str, float] = {}
    connection_dirs: Dict[Tuple[str, str], str] = {}

    for edge in root.findall("edge"):
        if "function" in edge.attrib:
            continue
        edge_id = edge.attrib["id"]
        edge_lanes[edge_id] = []
        for lane in edge.findall("lane"):
            lane_id = lane.attrib["id"]
            edge_lanes[edge_id].append(lane_id)
            lane_lengths[lane_id] = float(lane.attrib.get("length", 0.0))

    for connection in root.findall("connection"):
        from_edge = connection.attrib.get("from")
        to_edge = connection.attrib.get("to")
        if from_edge is None or to_edge is None:
            continue
        connection_dirs[(from_edge, to_edge)] = connection.attrib.get("dir", "unknown")

    return NetworkInfo(
        edge_lanes=edge_lanes,
        lane_lengths=lane_lengths,
        connection_dirs=connection_dirs,
    )


def is_green_state(signal_state: str) -> bool:
    """Return True when a SUMO signal state allows this movement to go."""

    return signal_state in {"G", "g"}


def is_red_or_yellow_state(signal_state: str) -> bool:
    """Return True for states that should not be counted as an active green."""

    return signal_state in {"r", "R", "y", "Y"}


def get_downstream_storage(to_edge: str, network_info: NetworkInfo) -> Tuple[float, float]:
    """Estimate downstream occupancy and available storage for an edge.

    SUMO gives lane occupancy as a percentage. We aggregate all lanes on
    ``to_edge`` by averaging occupancy and estimating remaining meters as
    ``lane_length * (1 - occupancy / 100)``. This is an approximation, so the
    threshold is exposed as a hyperparameter.
    """

    lanes = network_info.edge_lanes.get(to_edge, [])
    if not lanes:
        return 0.0, 0.0

    occupancies: List[float] = []
    available_storage = 0.0
    for lane_id in lanes:
        occupancy = traci.lane.getLastStepOccupancy(lane_id)
        lane_length = network_info.lane_lengths.get(lane_id, 0.0)
        occupancies.append(occupancy)
        available_storage += lane_length * max(0.0, 1.0 - occupancy / 100.0)

    return sum(occupancies) / len(occupancies), available_storage


def get_vehicle_snapshot(
    vehicle_id: str,
    network_info: NetworkInfo,
) -> Optional[VehicleSnapshot]:
    """Read the current signalized movement of one vehicle.

    ``from_edge`` is the current road that the vehicle is leaving.
    ``to_edge`` is the next road in its route. ``link_index`` is the index into
    the traffic-light state string for the exact movement
    ``from_lane -> to_lane``. For example, if ``link_index=15``, then
    ``traci.trafficlight.getRedYellowGreenState(tls_id)[15]`` is the signal
    state controlling this vehicle's movement.
    """

    current_edge = traci.vehicle.getRoadID(vehicle_id)
    if current_edge.startswith(":"):
        return None

    route = tuple(traci.vehicle.getRoute(vehicle_id))
    route_index = traci.vehicle.getRouteIndex(vehicle_id)
    if route_index < 0 or route_index >= len(route) - 1:
        return None

    from_edge = route[route_index]
    to_edge = route[route_index + 1]
    if current_edge != from_edge:
        return None

    next_tls = traci.vehicle.getNextTLS(vehicle_id)
    if not next_tls:
        return None

    tls_id, link_index, tls_distance, signal_state = next_tls[0]
    downstream_occupancy, available_storage_meter = get_downstream_storage(to_edge, network_info)
    return VehicleSnapshot(
        vehicle_id=vehicle_id,
        route_id=traci.vehicle.getRouteID(vehicle_id),
        route_edges=route,
        route_index=route_index,
        from_edge=from_edge,
        to_edge=to_edge,
        lane_id=traci.vehicle.getLaneID(vehicle_id),
        tls_id=tls_id,
        link_index=int(link_index),
        signal_state=signal_state,
        tls_distance=float(tls_distance),
        speed=traci.vehicle.getSpeed(vehicle_id),
        downstream_occupancy=downstream_occupancy,
        available_storage_meter=available_storage_meter,
        turn_direction=network_info.connection_dirs.get((from_edge, to_edge), "unknown"),
        vehicle_length=traci.vehicle.getLength(vehicle_id),
    )


def movement_key(snapshot: VehicleSnapshot) -> MovementKey:
    """Return the movement key used for cycle-aware aggregation."""

    return snapshot.tls_id, snapshot.link_index


def compute_queue_counts(
    snapshots: Dict[str, VehicleSnapshot],
    config: CamsConfig,
) -> Dict[MovementKey, int]:
    """Count queued vehicles per signalized movement at the current step."""

    queue_counts: Dict[MovementKey, int] = {}
    for snapshot in snapshots.values():
        if (
            snapshot.tls_distance <= config.waiting_region_length
            and snapshot.speed <= config.queue_speed_threshold
        ):
            key = movement_key(snapshot)
            queue_counts[key] = queue_counts.get(key, 0) + 1
    return queue_counts


def compute_queue_rank_at_arrival(
    target: VehicleSnapshot,
    snapshots: Dict[str, VehicleSnapshot],
    config: CamsConfig,
) -> int:
    """Approximate FIFO queue rank by counting queued vehicles ahead.

    A smaller ``tls_distance`` means the vehicle is closer to the stop line.
    Therefore, vehicles with the same ``tls_id + link_index`` and a smaller
    distance are ahead of the target vehicle.
    """

    rank = 0
    target_key = movement_key(target)
    for snapshot in snapshots.values():
        if snapshot.vehicle_id == target.vehicle_id:
            continue
        if movement_key(snapshot) != target_key:
            continue
        if snapshot.tls_distance > config.waiting_region_length:
            continue
        if snapshot.speed > config.queue_speed_threshold:
            continue
        if snapshot.tls_distance < target.tls_distance:
            rank += 1
    return rank


def make_vehicle_event(snapshot: VehicleSnapshot, current_time: float) -> VehicleMovementEvent:
    """Create a new vehicle-movement event when a vehicle enters an edge."""

    return VehicleMovementEvent(
        vehicle_id=snapshot.vehicle_id,
        route_id=snapshot.route_id,
        route_edges=snapshot.route_edges,
        route_index=snapshot.route_index,
        from_edge=snapshot.from_edge,
        to_edge=snapshot.to_edge,
        lane_id=snapshot.lane_id,
        tls_id=snapshot.tls_id,
        link_index=snapshot.link_index,
        edge_enter_time=current_time,
        turn_direction=snapshot.turn_direction,
        vehicle_length=snapshot.vehicle_length,
        last_update_time=current_time,
    )


def update_waiting_region_entry(
    event: VehicleMovementEvent,
    snapshot: VehicleSnapshot,
    snapshots: Dict[str, VehicleSnapshot],
    active_cycles: Dict[MovementKey, MovementCycleSummary],
    config: CamsConfig,
    current_time: float,
) -> None:
    """Set the first time the vehicle reaches the downstream waiting region."""

    if event.waiting_region_enter_time is not None:
        return
    if snapshot.tls_distance > config.waiting_region_length:
        return

    event.waiting_region_enter_time = current_time
    event.signal_state_at_arrival = snapshot.signal_state
    event.queue_rank_at_arrival = compute_queue_rank_at_arrival(snapshot, snapshots, config)
    event.downstream_occupancy_at_arrival = snapshot.downstream_occupancy
    event.available_storage_meter_at_arrival = snapshot.available_storage_meter

    key = movement_key(snapshot)
    if key in active_cycles:
        active_cycles[key].arrivals_during_green += 1
        event.queue_at_green_start = active_cycles[key].queue_at_green_start
        if event.cycles_waited == 0:
            # The vehicle arrived during an already-green wave. We still count
            # this green as its first opportunity if it has not passed yet.
            event.cycles_waited = 1


def accumulate_waiting_time(
    event: VehicleMovementEvent,
    snapshot: VehicleSnapshot,
    config: CamsConfig,
    current_time: float,
) -> None:
    """Accumulate red, green-queue, and downstream-block waiting times."""

    if event.waiting_region_enter_time is None or event.pass_stopline_time is not None:
        event.last_update_time = current_time
        return

    previous_time = event.last_update_time if event.last_update_time is not None else current_time
    delta = max(0.0, current_time - previous_time)
    event.last_update_time = current_time

    if delta == 0.0:
        return

    is_stopped = snapshot.speed <= config.stop_speed_threshold
    is_waiting = is_stopped or snapshot.speed <= config.queue_speed_threshold
    if not is_waiting:
        return

    if is_red_or_yellow_state(snapshot.signal_state):
        event.red_wait_time += delta
    elif (
        is_green_state(snapshot.signal_state)
        and snapshot.downstream_occupancy >= config.downstream_block_occupancy_threshold
    ):
        event.downstream_block_wait_time += delta
    else:
        event.green_queue_wait_time += delta


def finalize_event(
    event: VehicleMovementEvent,
    current_time: float,
    completed_events: List[VehicleMovementEvent],
) -> None:
    """Close an event when the vehicle reaches the downstream edge or exits."""

    if event.pass_stopline_time is None:
        event.pass_stopline_time = current_time
    if event.downstream_enter_time is None:
        event.downstream_enter_time = current_time
    if event.waiting_region_enter_time is None:
        # If a vehicle never entered the configured waiting region, keep the
        # event complete by treating the stop-line pass as the waiting entry.
        event.waiting_region_enter_time = event.pass_stopline_time
    completed_events.append(event)


def update_cycle_states(
    snapshots: Dict[str, VehicleSnapshot],
    active_events: Dict[str, VehicleMovementEvent],
    queue_counts: Dict[MovementKey, int],
    previous_green_states: Dict[MovementKey, bool],
    cycle_counters: Dict[MovementKey, int],
    active_cycles: Dict[MovementKey, MovementCycleSummary],
    completed_cycles: List[MovementCycleSummary],
    config: CamsConfig,
    current_time: float,
) -> None:
    """Track green windows for every movement observed in the simulation."""

    observed_keys = set(queue_counts) | set(active_cycles)
    observed_keys.update(movement_key(snapshot) for snapshot in snapshots.values())

    for key in observed_keys:
        tls_id, link_index = key
        state = traci.trafficlight.getRedYellowGreenState(tls_id)
        signal_state = state[link_index] if link_index < len(state) else "?"
        is_green = is_green_state(signal_state)
        was_green = previous_green_states.get(key, False)
        queue_length = queue_counts.get(key, 0)

        if is_green and not was_green:
            cycle_id = cycle_counters.get(key, 0) + 1
            cycle_counters[key] = cycle_id
            active_cycles[key] = MovementCycleSummary(
                tls_id=tls_id,
                link_index=link_index,
                cycle_id=cycle_id,
                green_start_time=current_time,
                queue_at_green_start=queue_length,
                max_queue_length_in_cycle=queue_length,
            )

            for event in active_events.values():
                if (event.tls_id, event.link_index) != key:
                    continue
                if event.waiting_region_enter_time is None or event.pass_stopline_time is not None:
                    continue
                event.cycles_waited += 1
                if event.queue_at_green_start is None:
                    event.queue_at_green_start = queue_length

        if is_green and key in active_cycles:
            active_cycles[key].update_queue(queue_length)

        if was_green and not is_green and key in active_cycles:
            cycle = active_cycles.pop(key)
            cycle.green_end_time = current_time
            cycle.residual_queue_after_green = queue_length
            completed_cycles.append(cycle)

        previous_green_states[key] = is_green


def write_vehicle_events(
    output_path: str,
    completed_events: List[VehicleMovementEvent],
    cycle_lookup: Dict[Tuple[str, int, int], MovementCycleSummary],
) -> None:
    """Write CAMS vehicle-movement events to CSV."""

    fieldnames = [
        "vehicle_id",
        "route_id",
        "route_edges",
        "route_index",
        "from_edge",
        "to_edge",
        "lane_id",
        "tls_id",
        "link_index",
        "edge_enter_time",
        "waiting_region_enter_time",
        "pass_stopline_time",
        "downstream_enter_time",
        "running_time",
        "signal_wait_time",
        "red_wait_time",
        "green_queue_wait_time",
        "downstream_block_wait_time",
        "cycles_waited",
        "queue_rank_at_arrival",
        "queue_at_green_start",
        "discharged_in_green",
        "residual_queue_after_green",
        "downstream_occupancy",
        "available_storage_meter",
        "signal_state_at_arrival",
        "turn_direction",
        "vehicle_length",
    ]
    with open(output_path, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for event in completed_events:
            cycle = None
            if event.pass_cycle_id is not None:
                cycle = cycle_lookup.get((event.tls_id, event.link_index, event.pass_cycle_id))

            running_time = event.waiting_region_enter_time - event.edge_enter_time
            signal_wait_time = event.pass_stopline_time - event.waiting_region_enter_time
            writer.writerow(
                {
                    "vehicle_id": event.vehicle_id,
                    "route_id": event.route_id,
                    "route_edges": " ".join(event.route_edges),
                    "route_index": event.route_index,
                    "from_edge": event.from_edge,
                    "to_edge": event.to_edge,
                    "lane_id": event.lane_id,
                    "tls_id": event.tls_id,
                    "link_index": event.link_index,
                    "edge_enter_time": round(event.edge_enter_time, 2),
                    "waiting_region_enter_time": round(event.waiting_region_enter_time, 2),
                    "pass_stopline_time": round(event.pass_stopline_time, 2),
                    "downstream_enter_time": round(event.downstream_enter_time, 2),
                    "running_time": round(running_time, 2),
                    "signal_wait_time": round(signal_wait_time, 2),
                    "red_wait_time": round(event.red_wait_time, 2),
                    "green_queue_wait_time": round(event.green_queue_wait_time, 2),
                    "downstream_block_wait_time": round(event.downstream_block_wait_time, 2),
                    "cycles_waited": event.cycles_waited,
                    "queue_rank_at_arrival": event.queue_rank_at_arrival,
                    "queue_at_green_start": event.queue_at_green_start,
                    "discharged_in_green": cycle.discharged_in_green if cycle else "",
                    "residual_queue_after_green": cycle.residual_queue_after_green if cycle else "",
                    "downstream_occupancy": round(event.downstream_occupancy_at_arrival or 0.0, 4),
                    "available_storage_meter": round(event.available_storage_meter_at_arrival or 0.0, 4),
                    "signal_state_at_arrival": event.signal_state_at_arrival,
                    "turn_direction": event.turn_direction,
                    "vehicle_length": event.vehicle_length,
                }
            )


def write_cycle_summaries(output_path: str, completed_cycles: List[MovementCycleSummary]) -> None:
    """Write movement-level green-window summaries to CSV."""

    fieldnames = [
        "tls_id",
        "link_index",
        "cycle_id",
        "green_start_time",
        "green_end_time",
        "green_duration",
        "queue_at_green_start",
        "arrivals_during_green",
        "discharged_in_green",
        "residual_queue_after_green",
        "downstream_blocked_count",
        "max_queue_length_in_cycle",
        "mean_queue_length_in_cycle",
    ]
    with open(output_path, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for cycle in completed_cycles:
            green_end_time = cycle.green_end_time if cycle.green_end_time is not None else cycle.green_start_time
            writer.writerow(
                {
                    "tls_id": cycle.tls_id,
                    "link_index": cycle.link_index,
                    "cycle_id": cycle.cycle_id,
                    "green_start_time": round(cycle.green_start_time, 2),
                    "green_end_time": round(green_end_time, 2),
                    "green_duration": round(green_end_time - cycle.green_start_time, 2),
                    "queue_at_green_start": cycle.queue_at_green_start,
                    "arrivals_during_green": cycle.arrivals_during_green,
                    "discharged_in_green": cycle.discharged_in_green,
                    "residual_queue_after_green": cycle.residual_queue_after_green,
                    "downstream_blocked_count": cycle.downstream_blocked_count,
                    "max_queue_length_in_cycle": cycle.max_queue_length_in_cycle,
                    "mean_queue_length_in_cycle": round(cycle.mean_queue_length_in_cycle, 4),
                }
            )

def main():
    """Run SUMO and collect CAMS-oriented signal-aware ground truth.

    The output focuses on the minimum fields needed by CAMS:
    vehicle movement events, signal-cycle discharge summaries, and a compact
    legacy edge summary for quick sanity checks.
    """

    config = parse_args()
    network_info = parse_network_info(config.net_file)
    traci.start([config.sumo_binary, "-c", config.sumo_config])

    print("simulation start")
    print(f"waiting_region_length={config.waiting_region_length} m")
    print(f"queue_speed_threshold={config.queue_speed_threshold} m/s")
    print(f"downstream_block_occupancy_threshold={config.downstream_block_occupancy_threshold}%")

    active_events: Dict[str, VehicleMovementEvent] = {}
    completed_events: List[VehicleMovementEvent] = []
    completed_cycles: List[MovementCycleSummary] = []
    active_cycles: Dict[MovementKey, MovementCycleSummary] = {}
    previous_green_states: Dict[MovementKey, bool] = {}
    cycle_counters: Dict[MovementKey, int] = {}
    previous_vehicle_edges: Dict[str, str] = {}

    # Compact compatibility table: one row per completed signalized movement.
    # It is intentionally smaller than the old table; the detailed CAMS fields
    # are stored in cams_vehicle_movement_events.csv.
    legacy_fieldnames = [
        "Vehicle_ID",
        "Edge_ID",
        "Route_Index",
        "To_Edge",
        "TLS_ID",
        "Link_Index",
        "Edge_Enter_Time",
        "Edge_Leave_Time",
        "Travel_Time",
        "Signal_Wait_Time",
        "Cycles_Waited",
    ]

    with open(config.legacy_edge_output, mode="w", newline="", encoding="utf-8") as legacy_file:
        legacy_writer = csv.DictWriter(legacy_file, fieldnames=legacy_fieldnames)
        legacy_writer.writeheader()

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = traci.simulation.getTime()
            vehicle_ids = set(traci.vehicle.getIDList())

            # First, close events whose vehicles have left the simulation. This
            # avoids losing the last edge when a vehicle reaches its destination.
            for vehicle_id in list(active_events.keys()):
                if vehicle_id not in vehicle_ids:
                    finalize_event(active_events.pop(vehicle_id), current_time, completed_events)

            snapshots: Dict[str, VehicleSnapshot] = {}
            for vehicle_id in vehicle_ids:
                snapshot = get_vehicle_snapshot(vehicle_id, network_info)
                if snapshot is not None:
                    snapshots[vehicle_id] = snapshot

            queue_counts = compute_queue_counts(snapshots, config)

            # Update signal-cycle states before vehicle discharge is counted.
            # A green wave is movement-specific: same traffic light, but a
            # different link_index means a different controlled movement.
            update_cycle_states(
                snapshots=snapshots,
                active_events=active_events,
                queue_counts=queue_counts,
                previous_green_states=previous_green_states,
                cycle_counters=cycle_counters,
                active_cycles=active_cycles,
                completed_cycles=completed_cycles,
                config=config,
                current_time=current_time,
            )

            for vehicle_id in vehicle_ids:
                current_edge = traci.vehicle.getRoadID(vehicle_id)
                previous_edge = previous_vehicle_edges.get(vehicle_id)

                # If the vehicle has moved from from_edge to an internal lane
                # such as :J21_12_0, it has crossed the stop line. If it moved
                # directly to to_edge, both stop-line passing and downstream
                # entry happen at this simulation step.
                if vehicle_id in active_events and previous_edge != current_edge:
                    event = active_events[vehicle_id]
                    if current_edge.startswith(":") and event.pass_stopline_time is None:
                        event.pass_stopline_time = current_time
                        active_cycle = active_cycles.get((event.tls_id, event.link_index))
                        if active_cycle is not None:
                            active_cycle.discharged_in_green += 1
                            event.pass_cycle_id = active_cycle.cycle_id
                    elif current_edge == event.to_edge:
                        if event.pass_stopline_time is None:
                            event.pass_stopline_time = current_time
                            active_cycle = active_cycles.get((event.tls_id, event.link_index))
                            if active_cycle is not None:
                                active_cycle.discharged_in_green += 1
                                event.pass_cycle_id = active_cycle.cycle_id
                        event.downstream_enter_time = current_time
                        finalize_event(active_events.pop(vehicle_id), current_time, completed_events)

                        signal_wait_time = (
                            event.pass_stopline_time - event.waiting_region_enter_time
                            if event.waiting_region_enter_time is not None
                            else 0.0
                        )
                        legacy_writer.writerow(
                            {
                                "Vehicle_ID": event.vehicle_id,
                                "Edge_ID": event.from_edge,
                                "Route_Index": event.route_index,
                                "To_Edge": event.to_edge,
                                "TLS_ID": event.tls_id,
                                "Link_Index": event.link_index,
                                "Edge_Enter_Time": round(event.edge_enter_time, 2),
                                "Edge_Leave_Time": round(event.downstream_enter_time, 2),
                                "Travel_Time": round(event.downstream_enter_time - event.edge_enter_time, 2),
                                "Signal_Wait_Time": round(signal_wait_time, 2),
                                "Cycles_Waited": event.cycles_waited,
                            }
                        )

                previous_vehicle_edges[vehicle_id] = current_edge

            for vehicle_id, snapshot in snapshots.items():
                if vehicle_id not in active_events:
                    active_events[vehicle_id] = make_vehicle_event(snapshot, current_time)

                event = active_events[vehicle_id]
                update_waiting_region_entry(
                    event=event,
                    snapshot=snapshot,
                    snapshots=snapshots,
                    active_cycles=active_cycles,
                    config=config,
                    current_time=current_time,
                )
                accumulate_waiting_time(
                    event=event,
                    snapshot=snapshot,
                    config=config,
                    current_time=current_time,
                )

                active_cycle = active_cycles.get((event.tls_id, event.link_index))
                if (
                    active_cycle is not None
                    and event.waiting_region_enter_time is not None
                    and event.pass_stopline_time is None
                    and snapshot.downstream_occupancy >= config.downstream_block_occupancy_threshold
                    and is_green_state(snapshot.signal_state)
                ):
                    active_cycle.downstream_blocked_vehicle_ids.add(vehicle_id)

    current_time = traci.simulation.getTime()
    for event in list(active_events.values()):
        finalize_event(event, current_time, completed_events)

    for cycle in list(active_cycles.values()):
        cycle.green_end_time = current_time
        completed_cycles.append(cycle)

    traci.close()

    cycle_lookup = {
        (cycle.tls_id, cycle.link_index, cycle.cycle_id): cycle
        for cycle in completed_cycles
    }
    write_vehicle_events(config.vehicle_event_output, completed_events, cycle_lookup)
    write_cycle_summaries(config.cycle_summary_output, completed_cycles)

    print(f"vehicle movement events written to: {config.vehicle_event_output}")
    print(f"movement cycle summaries written to: {config.cycle_summary_output}")
    print(f"legacy edge summary written to: {config.legacy_edge_output}")
    print("simulation done")

if __name__ == '__main__':
    main()
