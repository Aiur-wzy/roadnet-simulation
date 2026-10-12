"""Route-occurrence turn metadata shared by collectors and diagnostics.

This module defines audit metadata, not additional model features. Incoming and
outgoing always refer to the road at ``route_index``; never use ``route.index``
because a route can visit the same edge more than once.
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence


TURN_CONTEXT_SCHEMA = "road-turn-context-v1"
TURN_CODE_LABELS = {0: "unknown", 1: "left", 2: "straight", 3: "right", 4: "uturn"}
TURN_DIRECTION_CODES = {"l": 1, "L": 1, "s": 2, "S": 2, "r": 3, "R": 3, "t": 4, "U": 4}
# Compatibility import name for the legacy preprocessing entry points.
TURN_TYPE_MAP = TURN_DIRECTION_CODES
TURN_CONTEXT_COLUMNS = [
    "incoming_turn_type", "outgoing_turn_type",
    "incoming_turn_direction", "outgoing_turn_direction",
    "incoming_turn_present", "outgoing_turn_present", "turn_context_schema",
]
TURN_CONTEXT_CONTRACT = {
    "schema": TURN_CONTEXT_SCHEMA,
    "reference": "current external road occurrence at route_index",
    "incoming": "previous route edge -> current route edge",
    "outgoing": "current route edge -> next route edge",
    "codes": {str(code): name for code, name in TURN_CODE_LABELS.items()},
    "raw_direction_codes": dict(TURN_DIRECTION_CODES),
    "legacy_raw_aliases": {"S": "s", "U": "t"},
    "slight_turns": "L/R merge with l/r numerically; original raw direction retained",
    "present": "adjacent route edge exists, even if its movement mapping is missing",
    "absent_or_missing_raw_direction": "unknown",
    "legacy_python_turn_type": "outgoing_turn_type",
    "model_feature_change": False,
}


def encode_turn_direction(direction: str) -> int:
    """Map SUMO raw codes and explicit legacy aliases; unknown is never straight."""
    return TURN_DIRECTION_CODES.get(direction, 0) if isinstance(direction, str) else 0


def resolve_turn_context(
    route_edges: Sequence[str], route_index: int,
    connection_dirs: Mapping[tuple[str, str], str],
) -> dict:
    """Return both turns for one valid route occurrence, including boundaries.

    ``connection_dirs`` contains raw direction strings, not already encoded
    integers. The edge-pair representation assumes lane connections for that
    pair agree; callers must resolve or reject conflicting raw directions.
    Missing connections on a real adjacent pair remain present with code 0.
    """
    if isinstance(route_edges, (str, bytes)) or not route_edges:
        raise ValueError("route_edges must be a nonempty edge sequence")
    if isinstance(route_index, bool) or not isinstance(route_index, int):
        raise ValueError("route_index must be an integer occurrence index")
    if route_index < 0 or route_index >= len(route_edges):
        raise ValueError("route_index outside route_edges")
    incoming_present = route_index > 0
    outgoing_present = route_index + 1 < len(route_edges)
    current = route_edges[route_index]
    previous = route_edges[route_index - 1] if incoming_present else ""
    following = route_edges[route_index + 1] if outgoing_present else ""
    # Validate just this local neighborhood so each event stays O(1).
    adjacent = [current] + ([previous] if incoming_present else []) + ([following] if outgoing_present else [])
    if not all(isinstance(edge, str) and edge for edge in adjacent):
        raise ValueError("route_edges must contain nonempty edge identifiers")

    def raw_direction(source, target, present):
        if not present:
            return "unknown"
        raw = connection_dirs.get((source, target), "unknown")
        if not isinstance(raw, str):
            raise ValueError("connection_dirs must contain raw direction strings, not numeric codes")
        return raw

    incoming_raw = raw_direction(previous, current, incoming_present)
    outgoing_raw = raw_direction(current, following, outgoing_present)
    return {
        "incoming_turn_type": encode_turn_direction(incoming_raw),
        "outgoing_turn_type": encode_turn_direction(outgoing_raw),
        "incoming_turn_direction": incoming_raw,
        "outgoing_turn_direction": outgoing_raw,
        "incoming_turn_present": incoming_present,
        "outgoing_turn_present": outgoing_present,
        "turn_context_schema": TURN_CONTEXT_SCHEMA,
        "incoming_from_edge": previous, "current_edge": current, "outgoing_to_edge": following,
    }


def event_turn_metadata(event) -> dict:
    """Project sampled event metadata; legacy outgoing fields stay compatible.

    Event writers also accept older programmatic fixtures. Their existing
    ``turn_type``/``turn_direction`` are the authoritative outgoing aliases;
    incoming raw direction is unknown if it was never collected.
    """
    return {
        "incoming_turn_type": event.incoming_turn_type,
        "outgoing_turn_type": event.turn_type,
        "incoming_turn_direction": getattr(event, "incoming_turn_direction", "unknown"),
        "outgoing_turn_direction": event.turn_direction,
        "incoming_turn_present": int(event.route_index > 0),
        "outgoing_turn_present": int(event.route_index + 1 < len(event.route_edges)),
        "turn_context_schema": TURN_CONTEXT_SCHEMA,
    }
