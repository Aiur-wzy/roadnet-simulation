"""Read-only, causal outgoing-movement R/G features for fixed SUMO plans.

No signal setter or future simulation step is used. The live phase/end-time
anchors the static calendar, so configured offsets need not be reinterpreted.
Only a final route occurrence gets the G0G0 sentinel; uncontrolled exits use
constant G,H,G,H with a network-derived green horizon. Unknown or
ambiguous connections/programs are errors, not missing-signal observations.
"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from features.signal_rg_features import (
    RGSignalProgram, SIGNAL_FEATURE_COLS, no_signal_features,
    signal_feature_contract,
    always_green_features, network_green_horizon,
    signal_runtime_support_rule,
    signal_entry_reference_rule,
    network_without_internal_links,
)


SIGNAL_AUDIT_FIELDS = tuple(SIGNAL_FEATURE_COLS) + (
    "signal_status", "signal_tls_id", "signal_link_indices", "signal_program_id",
    "signal_anchor_time", "signal_anchor_phase", "signal_anchor_next_switch_time",
    "signal_reference_time", "signal_projection_seconds",
    "signal_state_at_reference", "signal_yellow_skipped",
)
UNSIGNALIZED_JUNCTION_TYPES = frozenset({
    "priority", "priority_stop", "right_before_left", "allway_stop", "zipper",
    "unregulated", "dead_end", "district",
})


class TraCISignalRG:
    """Cache one clock per TLS/observation step and one program per TLS.

    Connections and cycle segments are preprocessed, not scanned over vehicles.
    Sequential cyclic static programs are supported. Branching, actuated plans,
    WAUT switching, off/blinking states and changes on queried controllers fail
    closed. The network green horizon is frozen once; external signal control
    after initialization is unsupported, not polled globally for every vehicle.
    """

    def __init__(self, api, net_file, source_paths=()):
        self.api = api
        self.net_file = Path(net_file)
        self.unsignalized_horizon = None
        self.horizon_movements = {}
        root = ET.parse(net_file).getroot()
        self.network_without_internal_links = network_without_internal_links(root)
        node_types = {node.get("id"): node.get("type") for node in root.findall("junction")}
        edge_targets = {edge.get("id"): edge.get("to") for edge in root.findall("edge")}
        self.incoming_junction_types = {}
        self.lanes = {}
        for edge in root.findall("edge"):
            if edge.get("function") != "internal" and not edge.get("id", "").startswith(":"):
                for lane in edge.findall("lane"):
                    self.lanes[(edge.get("id"), int(lane.get("index", "0")))] = lane.get("id")
        self.connections = {}
        for connection in root.findall("connection"):
            edge, target = connection.get("from"), connection.get("to")
            from_lane = int(connection.get("fromLane", "0"))
            to_lane = int(connection.get("toLane", "0"))
            if (edge, from_lane) not in self.lanes or (target, to_lane) not in self.lanes:
                continue  # Internal connections are not the current external exit.
            tls = connection.get("tl", "")
            link = int(connection.get("linkIndex", "-1"))
            if tls and link < 0:
                raise ValueError("controlled connection lacks a valid linkIndex")
            item = (tls, link, self.lanes[(edge, from_lane)], self.lanes[(target, to_lane)])
            self.connections.setdefault((edge, target, from_lane), set()).add(item)
            self.incoming_junction_types.setdefault((edge, target), set()).add(
                "signalized_node" if tls or str(node_types.get(edge_targets.get(edge), "")).startswith("traffic_light")
                else "unsignalized_node" if node_types.get(edge_targets.get(edge)) in UNSIGNALIZED_JUNCTION_TYPES
                else "unknown_node")
        # Source files are small config/additional/route XML already used by the
        # base collector. Streaming avoids retaining all vehicle route elements.
        for path in (Path(net_file), *map(Path, source_paths)):
            for _, element in ET.iterparse(path, events=("end",)):
                if element.tag in ("WAUT", "wautJunction", "wautSwitch"):
                    raise ValueError("R/G v1 requires a fixed program: WAUT is unsupported")
                element.clear()
        self.plans = {}
        self.clocks = {}
        self.initial_origins = {}
        self.clock_queries = 0

    def incoming_junction_status(self, route, route_index):
        if route_index == 0:
            return "route_start"
        if not 0 < route_index < len(route):
            raise ValueError("invalid incoming route occurrence")
        statuses = self.incoming_junction_types.get((route[route_index - 1], route[route_index]), set())
        if not statuses:
            raise ValueError("missing incoming connection for crossing policy")
        # Match the edge-pair movement abstraction: any TLS retains crossing.
        if "signalized_node" in statuses:
            return "signalized_node"
        if statuses == {"unsignalized_node"}:
            return "unsignalized_node"
        # Unknown topology never becomes a silent zero-crossing connector.
        return "unknown_node"

    def _clock(self, tls, now):
        if tls in self.clocks and self.clocks[tls]["time"] == now:
            return self.clocks[tls]
        trafficlight = self.api.trafficlight
        program_id = str(trafficlight.getProgram(tls))
        if tls not in self.plans:
            matches = [p for p in trafficlight.getAllProgramLogics(tls)
                       if str(p.programID) == program_id]
            if len(matches) != 1 or int(matches[0].type) != 0:
                raise ValueError(f"R/G requires one active STATIC program: {tls}/{program_id}")
            phases = tuple(matches[0].phases)
            if not phases:
                raise ValueError(f"empty signal program: {tls}")
            durations = tuple(float(p.duration) for p in phases)
            if any(not math.isfinite(v) or v <= 0 for v in durations):
                raise ValueError(f"invalid static phase durations: {tls}")
            for index, phase in enumerate(phases):
                successors = tuple(getattr(phase, "next", ()))
                if successors and successors != ((index + 1) % len(phases),):
                    raise ValueError(f"R/G v1 requires sequential static phases: {tls}/{index}")
            states = tuple(str(p.state) for p in phases)
            if not states[0] or any(len(s) != len(states[0]) for s in states):
                raise ValueError(f"inconsistent static signal-state width: {tls}")
            ends, total = [], 0.0
            for duration in durations:
                total += duration
                ends.append(total)
            self.plans[tls] = {
                "id": program_id, "durations": durations, "states": states,
                "cycle": total, "ends": tuple(ends), "movements": {},
                "controlled_links": trafficlight.getControlledLinks(tls),
            }
        plan = self.plans[tls]
        if program_id != plan["id"]:
            raise ValueError(f"static program changed during collection: {tls}")
        phase_index = int(trafficlight.getPhase(tls))
        end = float(trafficlight.getNextSwitch(tls))
        if not 0 <= phase_index < len(plan["durations"]):
            raise ValueError(f"invalid runtime phase index: {tls}")
        remaining = end - now
        # SUMO may still expose the just-executed phase when its end equals
        # simulation.getTime(). The pure calendar uses [start,end) intervals.
        if not math.isfinite(end) or remaining < -1e-7 or remaining > plan["durations"][phase_index] + 1e-7:
            raise ValueError(f"runtime phase duration differs from fixed plan: {tls}")
        state = str(trafficlight.getRedYellowGreenState(tls))
        if state != plan["states"][phase_index]:
            raise ValueError(f"runtime signal state differs from static program: {tls}")
        origin = end - plan["ends"][phase_index]
        if tls in self.initial_origins:
            delta = origin - self.initial_origins[tls]
            if not math.isclose(delta, round(delta / plan["cycle"]) * plan["cycle"], abs_tol=1e-6, rel_tol=0):
                raise ValueError(f"runtime static signal calendar changed: {tls}")
        else:
            self.initial_origins[tls] = origin
        clock = {"time": now, "phase": phase_index, "end": end,
                 "remaining": min(plan["durations"][phase_index], max(0.0, remaining)), "plan": plan}
        self.clocks[tls] = clock
        self.clock_queries += 1
        return clock

    def _movement(self, plan, item):
        tls, link, from_lane, to_lane = item
        if link >= len(plan["states"][0]) or link >= len(plan["controlled_links"]):
            raise ValueError(f"outgoing linkIndex outside live program: {tls}/{link}")
        live_links = plan["controlled_links"][link]
        if not any(len(pair) >= 2 and pair[0] == from_lane and pair[1] == to_lane for pair in live_links):
            raise ValueError(f"net/live outgoing connection mismatch: {tls}/{link}")
        if link not in plan["movements"]:
            characters = tuple(state[link] for state in plan["states"])
            plan["movements"][link] = RGSignalProgram(plan["durations"], characters)
        return plan["movements"][link]

    def features(self, route, route_index, logical_lane, decision_time, entry_time):
        now, entry = float(decision_time), float(entry_time)
        if not all(math.isfinite(t) for t in (now, entry)) or entry < now:
            raise ValueError("R/G requires a finite causal planned entry time >= decision time")
        if not math.isclose(float(self.api.simulation.getTime()), now, abs_tol=1e-9, rel_tol=0):
            raise ValueError("signal anchor must be the current SUMO decision time")
        if not 0 <= route_index < len(route):
            raise ValueError("invalid signal route occurrence")
        audit = {
            "signal_status": "", "signal_tls_id": "", "signal_link_indices": "",
            "signal_program_id": "", "signal_anchor_time": now,
            "signal_anchor_phase": "", "signal_anchor_next_switch_time": "",
            "signal_reference_time": entry, "signal_projection_seconds": entry - now,
        }
        if route_index == len(route) - 1:
            audit["signal_status"] = "no_outgoing_route"
            return dict(no_signal_features(), **audit)
        key = route[route_index], route[route_index + 1], int(logical_lane)
        connections = sorted(self.connections.get(key, ()))
        if not connections:
            raise ValueError(f"no outgoing connection for logical lane/route: {key}")
        controls = {item[0] for item in connections}
        if controls == {""}:
            audit["signal_status"] = "unsignalized_connection"
            if self.unsignalized_horizon is None:
                horizon, network_programs = network_green_horizon(self.net_file)
                for tls, static in network_programs.items():
                    live = self._clock(tls, now)["plan"]
                    if (live["id"] != static["program_id"] or live["durations"] != static["durations"]
                            or live["states"] != static["states"]):
                        raise ValueError(f"green horizon net/active program mismatch: {tls}")
                self.horizon_movements = {tls: p["links"] for tls, p in network_programs.items()}
                self.unsignalized_horizon = horizon
            return dict(always_green_features(self.unsignalized_horizon), **audit)
        if "" in controls or len(controls) != 1:
            raise ValueError(f"ambiguous outgoing signal controllers: {key}")
        tls = next(iter(controls))
        clock = self._clock(tls, now)
        plan = clock["plan"]
        # Even equal values at this instant are not enough to resolve multiple
        # connections: require their complete raw movement-state sequences to
        # agree. The lane selector itself is unchanged from ABC.
        sequences = {tuple(s[item[1]] for s in plan["states"])
                     for item in connections if 0 <= item[1] < len(plan["states"][0])}
        if len(sequences) != 1:
            raise ValueError(f"ambiguous outgoing movement programs: {key}")
        movements = [self._movement(plan, item) for item in connections]
        values = movements[0].query(clock["phase"], clock["remaining"], entry - now)
        audit.update(signal_status="controlled_static", signal_tls_id=tls,
                     signal_link_indices=";".join(str(i) for i in sorted({item[1] for item in connections})),
                     signal_program_id=plan["id"], signal_anchor_phase=clock["phase"],
                     signal_anchor_next_switch_time=clock["end"])
        return dict(values, **audit)

    def contract(self):
        # Keep existing has-internal collections byte-compatible in semantics.
        # The new clock must not masquerade as a v3 controlled crossing.
        version = "static-movement-rg-segments-v4" if self.network_without_internal_links else "static-movement-rg-segments-v3"
        result = dict(signal_feature_contract(version=version))
        result.update(
            movement="current_external_road -> next_route_road; ABC logical fromLane; live linkIndex checked",
            reference_time=signal_entry_reference_rule(result["version"]),
            anchor="observed active STATIC program/phase/absolute nextSwitch; half-open phase boundaries",
            causality="known fixed plan at decision; no future actual entry, trajectory or release information",
            supported_programs=signal_runtime_support_rule(),
            runtime_programs={tls: {"program_id": p["id"], "durations": p["durations"], "states": p["states"],
                                    "initial_cycle_origin": self.initial_origins[tls]}
                              for tls, p in sorted(self.plans.items())},
            clock_queries=self.clock_queries,
        )
        if self.unsignalized_horizon is not None:
            result["runtime_unsignalized_green_seconds"] = self.unsignalized_horizon
            result["runtime_green_horizon_movements"] = self.horizon_movements
        if version == "static-movement-rg-segments-v4":
            result["runtime_network_without_internal_links"] = self.network_without_internal_links
        return result
