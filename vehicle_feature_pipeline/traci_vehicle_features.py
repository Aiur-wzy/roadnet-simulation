#!/usr/bin/env python3
"""Run the existing CAMS TraCI extractor with vehicle attributes enabled.

This variant deliberately leaves ``TraCI_Python_Adjusted.py`` unchanged. It
reuses its movement, waiting, and label accounting, then records the numeric
vehicle attributes represented by the C++ ``VehicleType`` structure.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Dict, List


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import TraCI_Python_Adjusted as base  # noqa: E402


VEHICLE_FEATURES = [
    "vehicle_length",
    "vehicle_min_gap",
    "vehicle_max_speed",
    "vehicle_accel",
    "vehicle_decel",
    "vehicle_sigma",
]
LIGHTWEIGHT_FEATURES = [
    "has_waiting",
    "road_length",
    "turn_type",
    "road_flow",
    "lane_flow",
] + VEHICLE_FEATURES


_original_make_vehicle_event = base.make_vehicle_event


def _vehicle_value(vehicle_id: str, getter_name: str, fallback: float) -> float:
    """Read one per-vehicle value, falling back to its vType when necessary."""

    try:
        return float(getattr(base.traci.vehicle, getter_name)(vehicle_id))
    except Exception:
        try:
            type_id = base.traci.vehicle.getTypeID(vehicle_id)
            return float(getattr(base.traci.vehicletype, getter_name)(type_id))
        except Exception:
            return float(fallback)


def make_vehicle_event(*args, **kwargs):
    """Create a normal event and snapshot its immutable vehicle attributes."""

    event = _original_make_vehicle_event(*args, **kwargs)
    vehicle_id = event.vehicle_id
    event.vehicle_length = _vehicle_value(vehicle_id, "getLength", event.vehicle_length)
    event.vehicle_min_gap = _vehicle_value(vehicle_id, "getMinGap", event.vehicle_min_gap)
    event.vehicle_max_speed = _vehicle_value(vehicle_id, "getMaxSpeed", 13.89)
    event.vehicle_accel = _vehicle_value(vehicle_id, "getAccel", 2.6)
    event.vehicle_decel = _vehicle_value(vehicle_id, "getDecel", 4.5)
    event.vehicle_sigma = _vehicle_value(vehicle_id, "getImperfection", 0.5)
    return event


def _valid_training_events(completed_events):
    for event in completed_events:
        travel_time_label = event.driving_time + event.low_speed_time
        pass_stopline_time = (
            event.pass_stopline_time
            if event.pass_stopline_time is not None
            else event.downstream_enter_time
        )
        if (
            travel_time_label <= 0
            or not event.road_length
            or event.road_length <= 0
            or not event.from_edge
            or pass_stopline_time is None
        ):
            continue
        yield event, travel_time_label, pass_stopline_time


def _vehicle_feature_row(event) -> Dict[str, float]:
    return {
        "vehicle_length": round(float(event.vehicle_length), 6),
        "vehicle_min_gap": round(float(event.vehicle_min_gap), 6),
        "vehicle_max_speed": round(float(event.vehicle_max_speed), 6),
        "vehicle_accel": round(float(event.vehicle_accel), 6),
        "vehicle_decel": round(float(event.vehicle_decel), 6),
        "vehicle_sigma": round(float(event.vehicle_sigma), 6),
    }


def write_lightweight_training_rows(output_path: str, completed_events) -> int:
    """Write the compact schema consumed by the vehicle-feature trainer."""

    fieldnames = LIGHTWEIGHT_FEATURES + ["travel_time_label"]
    row_count = 0
    with open(output_path, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for event, travel_time_label, _pass_stopline_time in _valid_training_events(
            completed_events
        ):
            row = {
                "has_waiting": event.previous_has_waiting,
                "road_length": round(event.road_length, 4),
                "turn_type": event.turn_type,
                "road_flow": event.road_flow if event.road_flow is not None else 0,
                "lane_flow": event.lane_flow if event.lane_flow is not None else 0,
                "travel_time_label": round(travel_time_label, 4),
            }
            row.update(_vehicle_feature_row(event))
            writer.writerow(row)
            row_count += 1
    return row_count


def write_training_rows(output_path: str, completed_events) -> None:
    """Write the metadata-rich CAMS CSV with the same vehicle feature contract."""

    fieldnames: List[str] = [
        "time", "vehicleID", "roadID", "movementID", "laneIndex",
        "road_length", "turn_direction", "turn_type", "incoming_turn_type",
        "road_flow", "lane_flow", "lane_num", "speed_limit",
        "vehicle_length", "vehicle_min_gap", "vehicle_max_speed",
        "vehicle_accel", "vehicle_decel", "vehicle_sigma",
        "lane_capacity", "lane_occupied_length", "has_waiting",
        "waiting_duration", "travel_time_label", "edge_enter_time",
        "pass_stopline_time", "downstream_enter_time", "driving_time",
        "low_speed_time", "red_light_waiting_time", "green_queue_wait_time",
        "downstream_block_wait_time", "total_before_stopline_time",
        "waiting_region_enter_time", "incoming_from_edge", "current_edge",
        "outgoing_to_edge", "tls_id", "link_index", "missed_green_rounds",
        "missed_green_due_to_queue", "missed_green_due_to_downstream_block",
    ]
    with open(output_path, mode="w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        for event, travel_time_label, pass_stopline_time in _valid_training_events(
            completed_events
        ):
            total_before_stopline_time = pass_stopline_time - event.edge_enter_time
            row = {
                "time": round(
                    event.sampled_time
                    if event.sampled_time is not None
                    else event.edge_enter_time,
                    2,
                ),
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
                "lane_capacity": round(event.lane_capacity or 0.0, 4),
                "lane_occupied_length": round(event.lane_occupied_length or 0.0, 4),
                "has_waiting": event.previous_has_waiting,
                "waiting_duration": round(event.previous_waiting_duration, 4),
                "travel_time_label": round(travel_time_label, 4),
                "edge_enter_time": round(event.edge_enter_time, 2),
                "pass_stopline_time": round(pass_stopline_time, 2),
                "downstream_enter_time": (
                    round(event.downstream_enter_time, 2)
                    if event.downstream_enter_time is not None
                    else ""
                ),
                "driving_time": round(event.driving_time, 4),
                "low_speed_time": round(event.low_speed_time, 4),
                "red_light_waiting_time": round(event.red_wait_time, 4),
                "green_queue_wait_time": round(event.green_queue_wait_time, 4),
                "downstream_block_wait_time": round(event.downstream_block_wait_time, 4),
                "total_before_stopline_time": round(total_before_stopline_time, 4),
                "waiting_region_enter_time": (
                    round(event.waiting_region_enter_time, 2)
                    if event.waiting_region_enter_time is not None
                    else ""
                ),
                "incoming_from_edge": event.incoming_from_edge,
                "current_edge": event.from_edge,
                "outgoing_to_edge": event.to_edge,
                "tls_id": event.tls_id,
                "link_index": event.link_index,
                "missed_green_rounds": event.missed_green_rounds,
                "missed_green_due_to_queue": event.missed_green_due_to_queue,
                "missed_green_due_to_downstream_block": (
                    event.missed_green_due_to_downstream_block
                ),
            }
            row.update(_vehicle_feature_row(event))
            writer.writerow(row)


def main() -> None:
    base.make_vehicle_event = make_vehicle_event
    base.write_lightweight_training_rows = write_lightweight_training_rows
    base.write_training_rows = write_training_rows
    print("[Vehicle Features] " + ",".join(VEHICLE_FEATURES))
    base.main()


if __name__ == "__main__":
    main()
