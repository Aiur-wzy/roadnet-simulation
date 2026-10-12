"""Pure static, cyclic movement-signal features; no SUMO calls or vehicle state.

The four formal values describe two finite non-yellow segments. Yellow time is
used when advancing the reference clock, but is not included in either duration
and is deliberately not exposed as a start delta or an additional input.
"""
from __future__ import annotations

from bisect import bisect_right
import math
from numbers import Integral, Real
import xml.etree.ElementTree as ET


SIGNAL_FEATURE_COLS = [
    "entry_signal_type", "entry_signal_remaining_seconds",
    "next_signal_type", "next_signal_duration_seconds",
]
SIGNAL_FEATURE_SCHEMA = "road-time-cap-context-abc-rg-v1"
SIGNAL_AUDIT_COLS = ["signal_state_at_reference", "signal_yellow_skipped"]


def signal_runtime_support_rule(legacy=False):
    if legacy:
        return "fixed sequential cyclic STATIC; reject WAUT, switching, calendar edits and unknown mappings"
    return ("fixed sequential cyclic STATIC; reject WAUT and queried-controller calendar changes; "
            "network green horizon frozen after initialization; external signal overrides unsupported")


def signal_feature_contract(legacy=False, *, version=None):
    """Return a fresh, JSON-serializable description of the exact feature rules."""
    version = version or ("static-movement-rg-segments-v1" if legacy else "static-movement-rg-segments-v3")
    if version not in {"static-movement-rg-segments-v1", "static-movement-rg-segments-v2", "static-movement-rg-segments-v3", "static-movement-rg-segments-v4"}:
        raise ValueError("unsupported signal_contract version")
    legacy = version == "static-movement-rg-segments-v1"
    result = {
        "version": "static-movement-rg-segments-v1",
        "feature_schema": SIGNAL_FEATURE_SCHEMA,
        "feature_columns": list(SIGNAL_FEATURE_COLS),
        "audit_columns": list(SIGNAL_AUDIT_COLS),
        "signal_types": ["R", "G"],
        "state_normalization": {"r": "R", "R": "R", "g": "G", "G": "G", "y": "Y", "Y": "Y"},
        "supported_program": "finite positive-duration phases; static sequential cyclic program; one outgoing movement",
        "reference_time": "planned road entry e; query current decision d with lookahead e-d >= 0",
        "phase_clock_input": "current phase index and its remaining seconds at decision d",
        "phase_boundary": "half-open phase intervals; remaining == 0 advances to the next phase",
        "same_color_merge": "merge consecutive normalized same-color phases, including across the cycle boundary; never merge across yellow",
        "reference_rg": "entry is current R/G segment remainder; next is full next non-yellow segment after current segment ends",
        "reference_yellow": "skip current yellow; entry is full first following R/G segment; next is full following non-yellow segment",
        "yellow_duration_policy": "advance clock through yellow, but exclude yellow from both formal durations; no start_delta or yellow-duration feature",
        "second_segment_color": "may equal entry color when yellow separates two segments",
        "duration_unit": "seconds; no fixed cycle length or green duration",
        "lookahead": "nonnegative finite seconds; any number of full cycles supported",
        "unavailable_finite_segments": "raise ValueError for constant-color or all-yellow cycles; never emit infinite or invented finite durations",
        "no_tls_placeholder": {
            "entry_signal_type": "G", "entry_signal_remaining_seconds": 0.0,
            "next_signal_type": "G", "next_signal_duration_seconds": 0.0,
            "interpretation": "G0G0 is an explicit no-TLS placeholder, not measured green or an always-green timed program",
            "signal_state_at_reference": "G", "signal_yellow_skipped": 0,
        },
        "complexity": "precompute O(phases log phases), O(phases) memory; query O(log phases), no per-vehicle phase scan",
    }
    if not legacy:
        result["version"] = "static-movement-rg-segments-v2"
        result["no_tls_placeholder"]["interpretation"] = "G0G0 identifies a final route occurrence with no outgoing movement only"
        result["unsignalized_connection"] = {
            "types": ["G", "G"],
            "durations": "both equal network longest finite continuous movement green segment H",
            "green_merge": "normalize G/g; merge adjacent phases and across cycle boundary; exclude yellow",
            "network_scope": "external controlled connections; one fixed sequential static program per TLS",
            "clock": "constant H at every reference time; no countdown or artificial phase event",
            "missing_finite_green": "error when an unsignalized movement needs H; never invent a duration",
        }
    if version in {"static-movement-rg-segments-v3", "static-movement-rg-segments-v4"}:
        result["version"] = version
        result["crossing_policy"] = {
            "first_road": "zero",
            "unsignalized_entry": "zero only for known nonsignalized incoming junction without TLS connection",
            "other_entry": "base context crossing_time_seconds; controlled junction or TLS connection retains crossing",
            "capacity": "unchanged waiting buffers, movement service interval and downstream storage",
        }
        if version == "static-movement-rg-segments-v4":
            result["crossing_policy"]["network_without_internal_links"] = (
                "zero for every incoming junction when loaded SUMO network has no internal edges or via references")
    return result


def signal_entry_reference_rule(version):
    if version == "static-movement-rg-segments-v4":
        return "planned_enter_time: decision_time plus incoming junction crossing; first/known unsignalized/no-internal network=0; otherwise context crossing seconds"
    if version == "static-movement-rg-segments-v3":
        return "planned_enter_time: decision_time plus incoming junction crossing; first/known unsignalized=0; otherwise context crossing seconds"
    return "planned_enter_time: first=decision_time; later=decision_time+context crossing seconds"


def network_without_internal_links(net):
    """Use actual net topology, not optional netconvert command comments.

    This is a crossing-clock rule, NOT a signal-state or right-of-way rule.
    A net with via references but missing internal edges is inconsistent and
    must not silently become a zero-crossing network.
    """
    root = net if isinstance(net, ET.Element) else ET.parse(net).getroot()
    edges = root.findall("edge")
    if root.tag != "net" or not edges:
        raise ValueError("crossing policy requires a SUMO net with edges")
    internal = any(e.get("function") == "internal" or e.get("id", "").startswith(":") for e in edges)
    internal_lanes = {lane.get("id") for e in edges
                      if e.get("function") == "internal" or e.get("id", "").startswith(":")
                      for lane in e.findall("lane") if lane.get("id")}
    has_via = any(c.get("via", "").strip() for c in root.findall("connection"))
    if has_via and not internal:
        raise ValueError("network has via references but no internal edges")
    for connection in root.findall("connection"):
        via = connection.get("via", "").strip()
        if via and via not in internal_lanes:
            raise ValueError(f"network references missing internal via lane: {via}")
    for junction in root.findall("junction"):
        if set(junction.get("intLanes", "").split()) - internal_lanes:
            raise ValueError("network junction references missing internal lane")
    return not internal and not has_via


def no_signal_features():
    """G0G0 sentinel for final route occurrences (legacy v1 also used no-TLS)."""
    return {
        "entry_signal_type": "G", "entry_signal_remaining_seconds": 0.0,
        "next_signal_type": "G", "next_signal_duration_seconds": 0.0,
        "signal_state_at_reference": "G", "signal_yellow_skipped": 0,
    }


def always_green_features(horizon):
    """Finite encoding of an assumed always-open exit, not a green countdown."""
    horizon = _number(horizon, "unsignalized green horizon", positive=True)
    return dict(no_signal_features(), entry_signal_remaining_seconds=horizon,
                next_signal_duration_seconds=horizon)


def longest_finite_green_seconds(durations, states):
    """Longest bounded green run for a movement; constant colors contribute 0."""
    durations, states = tuple(durations), tuple(states)
    if not durations or len(durations) != len(states):
        raise ValueError("green horizon requires matching phase durations/states")
    values = tuple(_number(v, "phase duration", positive=True) for v in durations)
    total = 0.0
    for value in values:
        advanced = total + value
        if not math.isfinite(advanced) or advanced <= total:
            raise ValueError("green horizon cycle cannot be represented at finite precision")
        total = advanced
    if any(not isinstance(s, str) or len(s) != 1 or s not in "rRgGyY" for s in states):
        raise ValueError("green horizon requires r/R/g/G/y/Y states")
    if len(set(s.upper() for s in states)) == 1:
        return 0.0
    program = RGSignalProgram(values, states)
    return max((d for s, d in zip(program._segment_states, program._segment_durations) if s == "G"), default=0.0)


def network_green_horizon(net_file):
    """Return H and the exact static network programs/movements defining it.

    Called once when an uncontrolled external movement actually needs H.
    Live collectors must verify these programs against their active programs.
    """
    root = ET.parse(net_file).getroot()
    if any(e.tag in ("WAUT", "waut", "wautJunction", "wautSwitch") for e in root.iter()):
        raise ValueError("green horizon requires fixed programs; WAUT unsupported")
    lanes = {(e.get("id"), int(l.get("index", "0"))) for e in root.findall("edge")
             if e.get("function") != "internal" and not e.get("id", "").startswith(":") for l in e.findall("lane")}
    used = {}
    for c in root.findall("connection"):
        if ((c.get("from"), int(c.get("fromLane", "0"))) not in lanes
                or (c.get("to"), int(c.get("toLane", "0"))) not in lanes or not c.get("tl")):
            continue
        index = int(c.get("linkIndex", "-1"))
        if index < 0:
            raise ValueError("green horizon controlled connection lacks linkIndex")
        used.setdefault(c.get("tl"), set()).add(index)
    programs = {}
    for tls in root.findall("tlLogic"):
        key = tls.get("id")
        if key not in used:
            continue
        if key in programs or tls.get("type") != "static":
            raise ValueError("green horizon requires one fixed static program per TLS")
        if not math.isfinite(float(tls.get("offset", "0"))):
            raise ValueError("green horizon requires finite offset")
        phases = tls.findall("phase")
        if not phases:
            raise ValueError("green horizon TLS has no phases")
        for i, phase in enumerate(phases):
            successor = phase.get("next", "").strip()
            if successor and successor != str((i + 1) % len(phases)):
                raise ValueError("green horizon requires sequential phases")
        programs[key] = {"program_id": tls.get("programID", "0"),
                         "durations": tuple(float(p.attrib["duration"]) for p in phases),
                         "states": tuple(p.attrib["state"] for p in phases),
                         "links": sorted(used[key])}
    if set(programs) != set(used):
        raise ValueError("green horizon missing controlled program")
    horizon = 0.0
    for program in programs.values():
        states = program["states"]
        if not states[0] or len({len(s) for s in states}) != 1:
            raise ValueError("green horizon inconsistent state widths")
        for index in program["links"]:
            if index >= len(states[0]):
                raise ValueError("green horizon linkIndex outside program")
            horizon = max(horizon, longest_finite_green_seconds(program["durations"], [s[index] for s in states]))
    if horizon <= 0 or not math.isfinite(horizon):
        raise ValueError("No finite network green segment available for unsignalized G,H,G,H")
    return horizon, programs


def _number(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number, not boolean/string")
    try:
        result = float(value)
    except (OverflowError, ValueError, TypeError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(result) or result < 0 or (positive and result == 0):
        constraint = "positive" if positive else "nonnegative"
        raise ValueError(f"{name} must be finite and {constraint}")
    return result


class RGSignalProgram:
    """Static timing for one movement, with cross-phase/cycle segment merging.

    The caller must establish that the loaded SUMO program is static and uses
    sequential phase succession. This helper deliberately has no dynamic-program
    fallback. Character case does not encode permissive/protected priority here.
    """

    def __init__(self, durations, states):
        try:
            durations, states = tuple(durations), tuple(states)
        except TypeError as exc:
            raise ValueError("durations and states must be nonempty sequences") from exc
        if not durations or len(durations) != len(states):
            raise ValueError("durations and states must have the same positive length")
        self.durations = tuple(_number(value, "phase duration", positive=True) for value in durations)
        if any(type(state) is not str or len(state) != 1 or state not in "rRgGyY" for state in states):
            raise ValueError("movement states must be single r/R/g/G/y/Y characters")
        self.states = tuple(state.upper() for state in states)
        if len(set(self.states)) == 1:
            raise ValueError("A constant-color or all-yellow cycle has no two finite non-yellow segments")
        try:
            self.cycle_seconds = math.fsum(self.durations)
        except OverflowError as exc:
            raise ValueError("Cycle duration exceeds finite floating-point range") from exc
        if not math.isfinite(self.cycle_seconds):
            raise ValueError("Cycle duration must be finite")

        # Keep one shared numerical phase partition for the binary search.
        phase_starts, phase_ends = [], []
        elapsed = 0.0
        for duration in self.durations:
            phase_starts.append(elapsed)
            next_elapsed = elapsed + duration
            if not math.isfinite(next_elapsed) or next_elapsed <= elapsed:
                raise ValueError("Phase boundaries cannot be resolved at floating-point precision")
            phase_ends.append(next_elapsed)
            elapsed = next_elapsed
        # The partition's final boundary is authoritative for modular lookup.
        self.cycle_seconds = elapsed
        self._phase_starts = tuple(phase_starts)
        self._phase_ends = tuple(phase_ends)

        # Start at a real color boundary so that a first/last same-color run is
        # represented once, not cut artificially at phase zero.
        count = len(self.states)
        start = next(index for index in range(count) if self.states[index] != self.states[index - 1])
        segment_phases, segment_states = [], []
        for offset in range(count):
            phase = (start + offset) % count
            state = self.states[phase]
            if not segment_states or state != segment_states[-1]:
                segment_states.append(state)
                segment_phases.append([])
            segment_phases[-1].append(phase)
        self._segment_states = tuple(segment_states)
        segment_durations, phase_segment, following_in_segment = [], [None] * count, [0.0] * count
        for segment, phases in enumerate(segment_phases):
            segment_durations.append(math.fsum(self.durations[phase] for phase in phases))
            tail = 0.0
            for phase in reversed(phases):
                phase_segment[phase] = segment
                following_in_segment[phase] = tail
                tail += self.durations[phase]
        self._segment_durations = tuple(segment_durations)
        self._phase_segment = tuple(phase_segment)
        self._following_in_segment = tuple(following_in_segment)
        nonyellow = [index for index, state in enumerate(segment_states) if state != "Y"]
        if not nonyellow:
            raise ValueError("Program has no finite non-yellow segment")
        # With one non-yellow run separated by yellow, its next cycle occurrence
        # is a valid second finite segment; it is not an always-green/red program.
        self._next_nonyellow = tuple(
            nonyellow[bisect_right(nonyellow, segment) % len(nonyellow)]
            for segment in range(len(segment_states))
        )

    def query(self, phase_index, phase_remaining_seconds, lookahead_seconds=0.0):
        if isinstance(phase_index, bool) or not isinstance(phase_index, Integral):
            raise ValueError("phase_index must be an integer")
        if not 0 <= phase_index < len(self.durations):
            raise ValueError("phase_index outside program")
        phase_index = int(phase_index)
        remaining = _number(phase_remaining_seconds, "phase_remaining_seconds")
        if remaining > self.durations[phase_index]:
            raise ValueError("phase_remaining_seconds exceeds the current phase duration")
        lookahead = _number(lookahead_seconds, "lookahead_seconds")
        # Reduce the potentially huge lookahead first: adding it to a small
        # phase offset directly would unnecessarily discard the offset's bits.
        position = (self._phase_starts[phase_index]
                    + (self.durations[phase_index] - remaining)
                    + (lookahead % self.cycle_seconds)) % self.cycle_seconds
        reference_phase = bisect_right(self._phase_ends, position)
        reference_remaining = self._phase_ends[reference_phase] - position
        reference_state = self.states[reference_phase]
        segment = self._phase_segment[reference_phase]
        yellow_skipped = int(reference_state == "Y")
        if yellow_skipped:
            entry_segment = self._next_nonyellow[segment]
            entry_remaining = self._segment_durations[entry_segment]
        else:
            entry_segment = segment
            entry_remaining = reference_remaining + self._following_in_segment[reference_phase]
        next_segment = self._next_nonyellow[entry_segment]
        return {
            "entry_signal_type": self._segment_states[entry_segment],
            "entry_signal_remaining_seconds": entry_remaining,
            "next_signal_type": self._segment_states[next_segment],
            "next_signal_duration_seconds": self._segment_durations[next_segment],
            "signal_state_at_reference": reference_state,
            "signal_yellow_skipped": yellow_skipped,
        }
