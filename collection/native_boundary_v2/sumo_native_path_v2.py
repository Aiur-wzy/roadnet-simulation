"""Conservative road-level geometry for an explicitly no-internal SUMO net.

No TraCI calls, time calibration, velocity inference, or entry features. A
unique road-level displacement does not identify the vehicle's unobserved lane.
Crossing endpoints and intermediate roads require equal lengths on ALL their
lanes: a lane change may occur between samples, including before an exit.
"""
from __future__ import annotations

import math
from numbers import Integral, Real
import xml.etree.ElementTree as ET

try:
    from .sumo_native_boundary_v2 import Segment
except ImportError:  # Direct execution/import beside the standalone reconciler.
    from sumo_native_boundary_v2 import Segment


class GeometryUnknown(ValueError):
    def __init__(self, reason, detail=""):
        self.reason = str(reason)
        self.detail = detail
        super().__init__(self.reason + (": " + detail if detail else ""))


def _finite(value):
    if not isinstance(value, Real) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


class NoInternalGeometry:
    """Read once; each query visits only the occurrences crossed by that step.

    `net_file` may be a path or an Element for a small in-memory test fixture.
    Connections are checked at ROAD level. We never require the previously
    observed lane to directly connect to the next road (SUMO may change lanes).
    Equal road-lane lengths identify longitudinal geometry, not a chosen lane.
    """

    def __init__(self, net_file):
        try:
            root = net_file if isinstance(net_file, ET.Element) else ET.parse(net_file).getroot()
        except (OSError, ET.ParseError) as exc:
            raise GeometryUnknown("invalid_network_xml", str(exc)) from exc
        if root.tag != "net":
            raise GeometryUnknown("invalid_network", "root must be net")
        self.lane_lengths, self.lane_roads, self.road_lanes = {}, {}, {}
        lane_indices = {}
        for edge in root.findall("edge"):
            road = edge.get("id", "")
            if edge.get("function") == "internal" or road.startswith(":"):
                raise GeometryUnknown("internal_network_unsupported", road)
            if not road or road in self.road_lanes:
                raise GeometryUnknown("invalid_network", "empty/duplicate edge ID")
            lanes = []
            for lane in edge.findall("lane"):
                identity = lane.get("id", "")
                try:
                    index = int(lane.attrib["index"])
                    length = float(lane.attrib["length"])
                except (KeyError, ValueError, OverflowError) as exc:
                    raise GeometryUnknown("invalid_network", "invalid lane index/length") from exc
                if (not identity or identity in self.lane_lengths or index < 0
                        or (road, index) in lane_indices or not math.isfinite(length) or length <= 0):
                    raise GeometryUnknown("invalid_network", "duplicate/invalid lane identity or length")
                self.lane_lengths[identity] = length
                self.lane_roads[identity] = road
                lane_indices[road, index] = identity
                lanes.append(identity)
            if not lanes:
                raise GeometryUnknown("invalid_network", "external edge without lanes: " + road)
            self.road_lanes[road] = tuple(lanes)
        if not self.road_lanes:
            raise GeometryUnknown("invalid_network", "empty external network")
        for junction in root.findall("junction"):
            if junction.get("intLanes", "").strip():
                raise GeometryUnknown("internal_network_unsupported", "junction intLanes")
        self.connections = {}
        for connection in root.findall("connection"):
            if connection.get("via", "").strip():
                raise GeometryUnknown("internal_network_unsupported", "connection via")
            road, target = connection.get("from"), connection.get("to")
            try:
                from_index, to_index = int(connection.attrib["fromLane"]), int(connection.attrib["toLane"])
                from_lane, to_lane = lane_indices[road, from_index], lane_indices[target, to_index]
            except (KeyError, ValueError, OverflowError) as exc:
                raise GeometryUnknown("invalid_network", "connection references unknown lane") from exc
            self.connections.setdefault((road, target), set()).add((from_lane, to_lane))
        self.connections = {pair: tuple(sorted(items)) for pair, items in self.connections.items()}
        # Exact equality is intentional. Unequal declared lengths are physical
        # ambiguity, not a floating calculation we may normalize away.
        self.road_lengths = {}
        for road, lanes in self.road_lanes.items():
            lengths = {self.lane_lengths[lane] for lane in lanes}
            self.road_lengths[road] = next(iter(lengths)) if len(lengths) == 1 else None

    def road_length(self, road):
        if road not in self.road_lengths:
            raise GeometryUnknown("unknown_road", str(road))
        length = self.road_lengths[road]
        if length is None:
            raise GeometryUnknown("unequal_lane_lengths", road)
        return length

    def lane_length(self, lane):
        try:
            return self.lane_lengths[lane]
        except (KeyError, TypeError) as exc:
            raise GeometryUnknown("unknown_lane", str(lane)) from exc

    @staticmethod
    def _route(route):
        if not isinstance(route, (tuple, list)) or not route:
            raise GeometryUnknown("invalid_route", "nonempty list/tuple required")

    def _road_at(self, route, index):
        if not isinstance(index, Integral) or isinstance(index, bool) or not 0 <= index < len(route):
            raise GeometryUnknown("invalid_route_index")
        road = route[index]
        if not isinstance(road, str) or road not in self.road_lanes:
            raise GeometryUnknown("unknown_road", str(road))
        return road

    def _observation(self, route, observation):
        if not isinstance(observation, dict):
            raise GeometryUnknown("invalid_observation", "dict required")
        index = observation.get("route_index")
        road = self._road_at(route, index)
        lane, position = observation.get("lane"), observation.get("position")
        if not isinstance(lane, str) or self.lane_roads.get(lane) != road or observation.get("edge") != road:
            raise GeometryUnknown("observation_road_lane_mismatch")
        if not _finite(position) or not 0 <= position <= self.lane_lengths[lane]:
            raise GeometryUnknown("invalid_position", lane)
        for name in ("speed", "time"):
            value = observation.get(name)
            if not _finite(value) or name == "speed" and value < 0:
                raise GeometryUnknown("invalid_observation", name)
        if "distance" in observation and observation["distance"] is not None:
            if not _finite(observation["distance"]) or observation["distance"] < 0:
                raise GeometryUnknown("invalid_odometer")
        return int(index), road, lane, float(position)

    @staticmethod
    def _odometer_delta(previous, current):
        old, new = previous.get("distance"), current.get("distance")
        if old is None or new is None:
            return None
        delta = new - old
        if not math.isfinite(delta) or delta < 0:
            raise GeometryUnknown("odometer_regression")
        return float(delta)

    def _crossing(self, route, start, finish):
        first, first_road, _, first_position = start
        last, last_road, _, last_position = finish
        if last < first:
            raise GeometryUnknown("route_index_regression")
        for index in range(first, last):
            pair = self._road_at(route, index), self._road_at(route, index + 1)
            if pair not in self.connections:
                raise GeometryUnknown("missing_roadpair_connection", " -> ".join(pair))
        # Even observed endpoint lanes cannot prove no lane change occurred
        # before/after an unobserved boundary. Require equal length there too.
        self.road_length(last_road)
        first_length = self.road_length(first_road)
        if first == last:
            distance = last_position - first_position
            if distance < 0:
                raise GeometryUnknown("negative_terminal_displacement")
            return (Segment(first, first_road, "external", distance),)
        result = [Segment(first, first_road, "external", first_length - first_position)]
        for index in range(first + 1, last):
            road = self._road_at(route, index)
            result.append(Segment(index, road, "external", self.road_length(road)))
        result.append(Segment(last, last_road, "external", last_position))
        return tuple(result)

    def step_segments(self, route, prev_obs, curr_obs):
        self._route(route)
        old, new = self._observation(route, prev_obs), self._observation(route, curr_obs)
        if curr_obs["time"] <= prev_obs["time"]:
            raise GeometryUnknown("nonmonotone_observation_time")
        if new[0] < old[0]:
            raise GeometryUnknown("route_index_regression")
        odometer = self._odometer_delta(prev_obs, curr_obs)
        if old[0] == new[0]:
            # A same-road lane change does not reset ownership. Without both
            # odometers, distance is explicitly unknown, not a fabricated zero.
            return (Segment(old[0], old[1], "external", odometer),)
        return self._crossing(route, old, new)

    def terminal_segments(self, route, last, arrivalLane, arrivalPos):
        """Only distance segments; the adapter owns terminal time/speed clipping."""
        self._route(route)
        old = self._observation(route, last)
        index = len(route) - 1
        road = self._road_at(route, index)
        if not isinstance(arrivalLane, str) or self.lane_roads.get(arrivalLane) != road:
            raise GeometryUnknown("arrival_lane_not_final_road")
        if not _finite(arrivalPos) or not 0 <= arrivalPos <= self.lane_lengths[arrivalLane]:
            raise GeometryUnknown("invalid_arrival_position")
        return self._crossing(route, old, (index, road, arrivalLane, float(arrivalPos)))

    def contract(self):
        return {
            "schema": "sumo-native-no-internal-geometry-v2",
            "supported_network": "no internal edge, connection via, or junction intLanes",
            "crossing": "previous remainder + intermediate road lengths + current/arrival position",
            "lane_ambiguity": "all road lanes must have exactly equal declared length for crossings; no chosen lane inferred",
            "connectivity": "roadpair exists; do not require observed previous lane to directly connect",
            "same_occurrence": "single owner; odometer delta if available, otherwise unknown distance",
            "terminal": "distance only; no inferred arrival/departure time or speed",
            "features": "none; road-level identifiable geometry does not identify entry lane/flow/ABC",
        }
