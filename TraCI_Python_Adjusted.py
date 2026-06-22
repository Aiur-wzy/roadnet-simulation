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
    training_output: str
    low_speed_threshold: float
    outputs: Set[str]


@dataclass
class NetworkInfo:
    """Static road-network information parsed from the SUMO net.xml file."""

    edge_lanes: Dict[str, List[str]]
    lane_lengths: Dict[str, float]
    connection_dirs: Dict[Tuple[str, str], str]
    edge_speed_limits: Dict[str, float]


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
    turn_type: int
    vehicle_length: float
    vehicle_min_gap: float


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
    vehicle_min_gap: float
    signal_state_at_arrival: Optional[str] = None
    waiting_region_enter_time: Optional[float] = None
    pass_stopline_time: Optional[float] = None
    downstream_enter_time: Optional[float] = None
    driving_time: float = 0.0
    low_speed_time: float = 0.0
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
    sampled_time: Optional[float] = None
    road_length: Optional[float] = None
    lane_index: Optional[int] = None
    lane_num: Optional[int] = None
    speed_limit: Optional[float] = None
    road_flow: Optional[int] = None
    lane_flow: Optional[int] = None
    entry_speed: Optional[float] = None
    entry_is_low_speed: int = 0
    entry_is_stopped: int = 0
    lane_capacity: Optional[float] = None
    lane_occupied_length: Optional[float] = None
    incoming_from_edge: str = ""
    # Main model feature: downstream outgoing movement from current edge/from_edge
    # to outgoing_to_edge/to_edge. Incoming movement is kept separately below.
    turn_type: int = 0
    incoming_turn_type: int = 0
    previous_has_waiting: int = 0
    previous_waiting_duration: float = 0.0


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



def normalize_outputs(outputs_arg: str) -> Set[str]:
    """Normalize and validate the comma-separated CSV output selection."""

    valid_outputs = {"legacy", "training", "events", "cycles", "all", "none"}
    requested = {item.strip().lower() for item in outputs_arg.split(",") if item.strip()}
    if not requested:
        raise argparse.ArgumentTypeError("--outputs must contain at least one output name")

    invalid = sorted(requested - valid_outputs)
    if invalid:
        raise argparse.ArgumentTypeError(
            f"unsupported output(s): {', '.join(invalid)}. "
            f"Choose from: {', '.join(sorted(valid_outputs))}"
        )
    if "none" in requested and len(requested) > 1:
        raise argparse.ArgumentTypeError("--outputs none cannot be combined with other output names")
    if "none" in requested:
        return set()
    if "all" in requested:
        return {"legacy", "training", "events", "cycles"}
    return requested

def parse_args() -> CamsConfig:
    """Parse command-line parameters for CAMS data extraction."""

    parser = argparse.ArgumentParser(
        description="Collect signal-aware SUMO ground truth for CAMS validation.",
        epilog=(
            "Examples:\n"
            "  # Lightweight training table only\n"
            "  python3 TraCI_Python_Adjusted.py --sumo-config full.sumocfg "
            "--net-file test_fixed.net.xml --sumo-binary sumo --outputs legacy\n"
            "  # Lightweight table + full training table\n"
            "  python3 TraCI_Python_Adjusted.py --sumo-config full.sumocfg "
            "--net-file test_fixed.net.xml --sumo-binary sumo --outputs legacy,training\n"
            "  # All debug outputs\n"
            "  python3 TraCI_Python_Adjusted.py --sumo-config full.sumocfg "
            "--net-file test_fixed.net.xml --sumo-binary sumo --outputs all"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--sumo-config", default="map.sumo.cfg", help="SUMO cfg file.")
    parser.add_argument("--net-file", default="test.net.xml", help="SUMO network file.")
    parser.add_argument("--sumo-binary", default="sumo", help="SUMO binary, e.g., sumo or sumo-gui.")
    parser.add_argument(
        "--vehicle-event-output",
        default="cams_vehicle_movement_events.csv",
        help="Debug/validation CSV for vehicle-movement ground-truth events (enabled by --outputs events/all).",
    )
    parser.add_argument(
        "--cycle-summary-output",
        default="cams_movement_cycle_summary.csv",
        help="Debug/validation CSV for movement-level green-window summaries (enabled by --outputs cycles/all).",
    )
    parser.add_argument(
        "--legacy-edge-output",
        default="TraCI_output_adjusted.csv",
        help="Lightweight model-training input CSV (enabled by --outputs legacy/all).",
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
    parser.add_argument(
        "--training-output",
        default="cams_model_training_data.csv",
        help="Fuller debug/training-compatible CSV (enabled by --outputs training/all).",
    )
    parser.add_argument(
        "--low-speed-threshold",
        type=float,
        default=2.0,
        help="Speed <= this value and > stop threshold is counted as low-speed time, in m/s.",
    )
    parser.add_argument(
        "--outputs",
        default="legacy",
        type=normalize_outputs,
        help=(
            "Comma-separated CSV outputs to write. Supported values: legacy "
            "(TraCI_output_adjusted.csv slim model-training table), training "
            "(cams_model_training_data.csv full debug/training-compatible output), "
            "events, cycles, all, none. Default: legacy."
        ),
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
        training_output=args.training_output,
        low_speed_threshold=args.low_speed_threshold,
        outputs=args.outputs,
    )


def parse_network_info(file_path: str) -> NetworkInfo:
    """Parse lane lists, lane lengths, and connection directions from net.xml."""

    tree = ET.parse(file_path)
    root = tree.getroot()
    edge_lanes: Dict[str, List[str]] = {}
    lane_lengths: Dict[str, float] = {}
    connection_dirs: Dict[Tuple[str, str], str] = {}
    edge_speed_limits: Dict[str, float] = {}

    for edge in root.findall("edge"):
        if "function" in edge.attrib:
            continue
        edge_id = edge.attrib["id"]
        edge_lanes[edge_id] = []
        lane_speeds: List[float] = []
        for lane in edge.findall("lane"):
            lane_id = lane.attrib["id"]
            edge_lanes[edge_id].append(lane_id)
            lane_lengths[lane_id] = float(lane.attrib.get("length", 0.0))
            lane_speeds.append(float(lane.attrib.get("speed", 0.0)))
        edge_speed_limits[edge_id] = sum(lane_speeds) / len(lane_speeds) if lane_speeds else 0.0

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
        edge_speed_limits=edge_speed_limits,
    )



def encode_turn_direction(direction: str) -> int:
    """Encode SUMO connection directions for BasicRoadModelFeatures.

    For model-training rows, turn_type is the SUMO connection direction from
    current_edge/from_edge to outgoing_to_edge/to_edge.
    """

    if direction in {"l", "L"}:
        return 1
    if direction == "s":
        return 2
    if direction in {"r", "R"}:
        return 3
    if direction == "t":
        return 4
    return 0


def lane_index_from_id(lane_id: str) -> Optional[int]:
    """Return the numeric SUMO lane suffix, if present."""

    try:
        return int(lane_id.rsplit("_", 1)[1])
    except (IndexError, ValueError):
        return None


def compute_flow_snapshot(network_info: NetworkInfo) -> Tuple[Dict[str, int], Dict[str, int], Dict[str, float]]:
    """Count vehicles and occupied length on each normal edge/lane."""

    road_flow: Dict[str, int] = {}
    lane_flow: Dict[str, int] = {}
    lane_occupied_length: Dict[str, float] = {}
    for vehicle_id in traci.vehicle.getIDList():
        road_id = traci.vehicle.getRoadID(vehicle_id)
        if road_id.startswith(":"):
            continue
        lane_id = traci.vehicle.getLaneID(vehicle_id)
        vehicle_length = traci.vehicle.getLength(vehicle_id)
        try:
            min_gap = traci.vehicle.getMinGap(vehicle_id)
        except Exception:
            min_gap = 2.5
        road_flow[road_id] = road_flow.get(road_id, 0) + 1
        lane_flow[lane_id] = lane_flow.get(lane_id, 0) + 1
        lane_occupied_length[lane_id] = lane_occupied_length.get(lane_id, 0.0) + vehicle_length + min_gap
    return road_flow, lane_flow, lane_occupied_length


def sample_training_features(
    event: VehicleMovementEvent,
    snapshot: VehicleSnapshot,
    network_info: NetworkInfo,
    previous_release_waiting: Dict[str, Tuple[int, float]],
    config: CamsConfig,
) -> None:
    """Sample road-model features immediately when an event starts."""

    road_flow, lane_flow, lane_occupied_length = compute_flow_snapshot(network_info)
    lanes = network_info.edge_lanes.get(snapshot.from_edge, [])
    selected_lane = snapshot.lane_id if snapshot.lane_id in lanes else (lanes[0] if lanes else snapshot.lane_id)
    selected_lane_flow = lane_flow.get(selected_lane)
    if selected_lane_flow is None and lanes:
        selected_lane_flow = max((lane_flow.get(lane, 0) for lane in lanes), default=0)
    lane_length = network_info.lane_lengths.get(selected_lane, 0.0)
    min_gap = snapshot.vehicle_min_gap or 2.5
    occupied_length = lane_occupied_length.get(selected_lane, 0.0)
    has_waiting, waiting_duration = previous_release_waiting.pop(snapshot.vehicle_id, (0, 0.0))

    event.sampled_time = event.edge_enter_time
    event.road_length = lane_length
    event.lane_index = lane_index_from_id(selected_lane)
    event.lane_num = len(lanes)
    event.speed_limit = network_info.edge_speed_limits.get(snapshot.from_edge, 0.0)
    event.road_flow = road_flow.get(snapshot.from_edge, 0)
    event.lane_flow = selected_lane_flow or 0
    event.entry_speed = snapshot.speed
    event.entry_is_low_speed = int(config.stop_speed_threshold < snapshot.speed <= config.low_speed_threshold)
    event.entry_is_stopped = int(snapshot.speed <= config.stop_speed_threshold)
    event.lane_capacity = lane_length / max(snapshot.vehicle_length + min_gap, 0.1) if lane_length > 0 else 0.0
    event.lane_occupied_length = occupied_length
    event.vehicle_min_gap = min_gap
    event.turn_type = encode_turn_direction(event.turn_direction)
    event.incoming_from_edge = snapshot.route_edges[snapshot.route_index - 1] if snapshot.route_index > 0 else ""
    event.incoming_turn_type = encode_turn_direction(
        network_info.connection_dirs.get((event.incoming_from_edge, snapshot.from_edge), "unknown")
    )
    event.previous_has_waiting = has_waiting
    event.previous_waiting_duration = waiting_duration

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
        turn_type=encode_turn_direction(network_info.connection_dirs.get((from_edge, to_edge), "unknown")),
        vehicle_length=traci.vehicle.getLength(vehicle_id),
        vehicle_min_gap=traci.vehicle.getMinGap(vehicle_id),
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


def make_vehicle_event(
    snapshot: VehicleSnapshot,
    current_time: float,
    network_info: NetworkInfo,
    previous_release_waiting: Dict[str, Tuple[int, float]],
    config: CamsConfig,
) -> VehicleMovementEvent:
    """Create a new vehicle-movement event when a vehicle enters an edge."""

    event = VehicleMovementEvent(
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
        turn_type=snapshot.turn_type,
        vehicle_length=snapshot.vehicle_length,
        vehicle_min_gap=snapshot.vehicle_min_gap,
        last_update_time=current_time,
    )
    sample_training_features(event, snapshot, network_info, previous_release_waiting, config)
    return event


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
    """Accumulate road driving/low-speed/waiting time before the stop line."""

    if event.pass_stopline_time is not None:
        event.last_update_time = current_time
        return

    previous_time = event.last_update_time if event.last_update_time is not None else current_time
    delta = max(0.0, current_time - previous_time)
    event.last_update_time = current_time

    if delta == 0.0:
        return

    if snapshot.speed > config.low_speed_threshold:
        event.driving_time += delta
    elif snapshot.speed > config.stop_speed_threshold:
        event.low_speed_time += delta
    elif is_red_or_yellow_state(snapshot.signal_state):
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
        "time_to_waiting_region",
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

            time_to_waiting_region = event.waiting_region_enter_time - event.edge_enter_time
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
                    "time_to_waiting_region": round(time_to_waiting_region, 2),
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



def write_lightweight_training_rows(output_path: str, completed_events: List[VehicleMovementEvent]) -> int:
    """Write TraCI_output_adjusted.csv as the slim model-training table.

    This is a minimal projection of the same VehicleMovementEvent feature and
    label semantics used by write_training_rows(). Keep fuller debug/training
    columns in cams_model_training_data.csv, and use the movement/cycle CSVs
    for debugging and validation.
    """

    fieldnames = [
        "has_waiting",
        "road_length",
        "turn_type",
        "road_flow",
        "lane_flow",
        "travel_time_label",
    ]
    row_count = 0
    with open(output_path, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for event in completed_events:
            travel_time_label = event.driving_time + event.low_speed_time
            if travel_time_label <= 0 or not event.road_length or event.road_length <= 0 or not event.from_edge:
                continue
            pass_stopline_time = event.pass_stopline_time if event.pass_stopline_time is not None else event.downstream_enter_time
            if pass_stopline_time is None:
                continue

            writer.writerow({
                "has_waiting": event.previous_has_waiting,
                "road_length": round(event.road_length, 4),
                "turn_type": event.turn_type,
                "road_flow": event.road_flow if event.road_flow is not None else 0,
                "lane_flow": event.lane_flow if event.lane_flow is not None else 0,
                "travel_time_label": round(travel_time_label, 4),
            })
            row_count += 1
    return row_count

def write_training_rows(output_path: str, completed_events: List[VehicleMovementEvent]) -> None:
    """Write CAMS BasicRoadModelFeatures-compatible training rows.

    The main turn_type feature is the downstream outgoing movement from
    current_edge/from_edge to outgoing_to_edge/to_edge. incoming_turn_type is
    retained only as an explicit previous-edge debugging/future-feature column.
    """

    fieldnames = [
        "time", "vehicleID", "roadID", "movementID", "laneIndex",
        "road_length", "turn_direction", "turn_type", "incoming_turn_type", "road_flow", "lane_flow", "lane_num",
        "speed_limit", "vehicle_length", "vehicle_min_gap", "lane_capacity",
        "lane_occupied_length", "has_waiting", "waiting_duration",
        "travel_time_label", "edge_enter_time", "pass_stopline_time",
        "downstream_enter_time", "driving_time", "low_speed_time",
        "red_light_waiting_time", "green_queue_wait_time",
        "downstream_block_wait_time", "total_before_stopline_time",
        "waiting_region_enter_time", "incoming_from_edge", "current_edge",
        "outgoing_to_edge", "tls_id", "link_index",
    ]
    with open(output_path, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for event in completed_events:
            travel_time_label = event.driving_time + event.low_speed_time
            if travel_time_label <= 0 or not event.road_length or event.road_length <= 0 or not event.from_edge:
                continue
            pass_stopline_time = event.pass_stopline_time if event.pass_stopline_time is not None else event.downstream_enter_time
            if pass_stopline_time is None:
                continue
            total_before_stopline_time = pass_stopline_time - event.edge_enter_time
            writer.writerow({
                "time": round(event.sampled_time if event.sampled_time is not None else event.edge_enter_time, 2),
                "vehicleID": event.vehicle_id,
                "roadID": event.from_edge,
                "movementID": f"{event.tls_id}:{event.link_index}",
                "laneIndex": event.lane_index if event.lane_index is not None else "",
                "road_length": round(event.road_length, 4),
                "turn_direction": event.turn_direction,
                "turn_type": event.turn_type,
                "incoming_turn_type": event.incoming_turn_type,
                "road_flow": event.road_flow if event.road_flow is not None else 0,
                "lane_flow": event.lane_flow if event.lane_flow is not None else 0,
                "lane_num": event.lane_num if event.lane_num is not None else 0,
                "speed_limit": round(event.speed_limit or 0.0, 4),
                "vehicle_length": round(event.vehicle_length, 4),
                "vehicle_min_gap": round(event.vehicle_min_gap, 4),
                "lane_capacity": round(event.lane_capacity or 0.0, 4),
                "lane_occupied_length": round(event.lane_occupied_length or 0.0, 4),
                "has_waiting": event.previous_has_waiting,
                "waiting_duration": round(event.previous_waiting_duration, 4),
                "travel_time_label": round(travel_time_label, 4),
                "edge_enter_time": round(event.edge_enter_time, 2),
                "pass_stopline_time": round(pass_stopline_time, 2),
                "downstream_enter_time": round(event.downstream_enter_time, 2) if event.downstream_enter_time is not None else "",
                "driving_time": round(event.driving_time, 4),
                "low_speed_time": round(event.low_speed_time, 4),
                "red_light_waiting_time": round(event.red_wait_time, 4),
                "green_queue_wait_time": round(event.green_queue_wait_time, 4),
                "downstream_block_wait_time": round(event.downstream_block_wait_time, 4),
                "total_before_stopline_time": round(total_before_stopline_time, 4),
                "waiting_region_enter_time": round(event.waiting_region_enter_time, 2) if event.waiting_region_enter_time is not None else "",
                "incoming_from_edge": event.incoming_from_edge,
                "current_edge": event.from_edge,
                "outgoing_to_edge": event.to_edge,
                "tls_id": event.tls_id,
                "link_index": event.link_index,
            })

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
    slim model-training table for large-scale training.
    """

    config = parse_args()
    want_legacy = "legacy" in config.outputs
    want_training = "training" in config.outputs
    want_events = "events" in config.outputs
    want_cycles = "cycles" in config.outputs
    need_cycle_debug = want_events or want_cycles
    need_waiting_region_debug = want_events or want_cycles

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
    previous_release_waiting: Dict[str, Tuple[int, float]] = {}

    # CSV outputs are selected by --outputs. TraCI_output_adjusted.csv is the
    # slim model-training table, cams_model_training_data.csv is the
    # fuller debug/training-compatible output, and movement/cycle files are
    # debugging and validation outputs.
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        current_time = traci.simulation.getTime()
        vehicle_ids = set(traci.vehicle.getIDList())

        # First, close events whose vehicles have left the simulation. This
        # avoids losing the last edge when a vehicle reaches its destination.
        for vehicle_id in list(active_events.keys()):
            if vehicle_id not in vehicle_ids:
                event = active_events.pop(vehicle_id)
                release_waiting_duration = event.red_wait_time + event.green_queue_wait_time + event.downstream_block_wait_time
                previous_release_waiting[vehicle_id] = (1 if release_waiting_duration > 0 else 0, release_waiting_duration)
                finalize_event(event, current_time, completed_events)

        snapshots: Dict[str, VehicleSnapshot] = {}
        for vehicle_id in vehicle_ids:
            snapshot = get_vehicle_snapshot(vehicle_id, network_info)
            if snapshot is not None:
                snapshots[vehicle_id] = snapshot

        if need_cycle_debug:
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
        else:
            queue_counts = {}

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
                    if need_cycle_debug:
                        active_cycle = active_cycles.get((event.tls_id, event.link_index))
                        if active_cycle is not None:
                            active_cycle.discharged_in_green += 1
                            event.pass_cycle_id = active_cycle.cycle_id
                elif current_edge == event.to_edge:
                    if event.pass_stopline_time is None:
                        event.pass_stopline_time = current_time
                        if need_cycle_debug:
                            active_cycle = active_cycles.get((event.tls_id, event.link_index))
                            if active_cycle is not None:
                                active_cycle.discharged_in_green += 1
                                event.pass_cycle_id = active_cycle.cycle_id
                    event.downstream_enter_time = current_time
                    release_waiting_duration = event.red_wait_time + event.green_queue_wait_time + event.downstream_block_wait_time
                    previous_release_waiting[vehicle_id] = (1 if release_waiting_duration > 0 else 0, release_waiting_duration)
                    finalize_event(active_events.pop(vehicle_id), current_time, completed_events)


            previous_vehicle_edges[vehicle_id] = current_edge

        for vehicle_id, snapshot in snapshots.items():
            if vehicle_id not in active_events:
                active_events[vehicle_id] = make_vehicle_event(snapshot, current_time, network_info, previous_release_waiting, config)

            event = active_events[vehicle_id]
            if need_waiting_region_debug:
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

            if need_cycle_debug:
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

    if need_cycle_debug:
        for cycle in list(active_cycles.values()):
            cycle.green_end_time = current_time
            completed_cycles.append(cycle)

    traci.close()

    cycle_lookup = {}
    if want_events:
        cycle_lookup = {
            (cycle.tls_id, cycle.link_index, cycle.cycle_id): cycle
            for cycle in completed_cycles
        }
        write_vehicle_events(config.vehicle_event_output, completed_events, cycle_lookup)
        print(f"vehicle movement events written to: {config.vehicle_event_output}")
    if want_cycles:
        write_cycle_summaries(config.cycle_summary_output, completed_cycles)
        print(f"movement cycle summaries written to: {config.cycle_summary_output}")
    if want_training:
        write_training_rows(config.training_output, completed_events)
        print(f"training data written to: {config.training_output}")
    if want_legacy:
        row_count = write_lightweight_training_rows(config.legacy_edge_output, completed_events)
        print(f"lightweight training data written to: {config.legacy_edge_output}")
        print("lightweight training columns: has_waiting,road_length,turn_type,road_flow,lane_flow,travel_time_label")
        print(f"lightweight training rows: {row_count}")
    print("simulation done")

if __name__ == '__main__':
    main()
