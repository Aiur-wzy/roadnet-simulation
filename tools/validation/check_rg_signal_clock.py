#!/usr/bin/env python3
"""Real empty-network SUMO clock check against independent XML signal calendars.

Never edits network/routes/configuration; launches separate no-vehicle SUMO runs.
The XML oracle scans phases directly and does not call RGSignalProgram.query.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
from roadnet_paths import source_path
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from features.signal_rg_features import RGSignalProgram, SIGNAL_FEATURE_COLS  # noqa: E402


TOLERANCE = 1e-7


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_network(path):
    root = ET.parse(path).getroot()
    programs = defaultdict(dict)
    for logic in root.findall("tlLogic"):
        if logic.get("type") != "static":
            raise ValueError(f"Nonstatic XML program {logic.get('id')}/{logic.get('programID')}")
        phases = logic.findall("phase")
        if not phases:
            raise ValueError("Empty XML signal program")
        for index, phase in enumerate(phases):
            if phase.get("next") and phase.get("next").split() != [str((index + 1) % len(phases))]:
                raise ValueError("Nonsequential phase succession is outside this clock check")
        durations = [float(phase.attrib["duration"]) for phase in phases]
        states = [phase.attrib["state"] for phase in phases]
        offset = float(logic.get("offset", "0"))
        if not all(math.isfinite(duration) and duration > 0 for duration in durations) or not math.isfinite(offset):
            raise ValueError("Invalid XML timing")
        programs[logic.attrib["id"]][logic.attrib["programID"]] = {
            "durations": durations, "states": states, "offset": offset, "cycle": sum(durations),
        }
    connections = defaultdict(lambda: defaultdict(list))
    for connection in root.findall("connection"):
        tls = connection.get("tl")
        if not tls:
            continue
        link = int(connection.attrib["linkIndex"])
        connections[tls][link].append({
            "from_lane": connection.attrib["from"] + "_" + connection.attrib["fromLane"],
            "to_lane": connection.attrib["to"] + "_" + connection.attrib["toLane"],
            "via": connection.get("via", ""), "direction": connection.get("dir", ""),
        })
    return dict(programs), {tls: dict(links) for tls, links in connections.items()}


def calendar_phase(program, absolute_time, *, plus_offset=False):
    """Independent NET calendar, using SUMO's offset reference convention."""
    sign = 1 if plus_offset else -1
    position = (absolute_time + sign * program["offset"]) % program["cycle"]
    if math.isclose(position, program["cycle"], abs_tol=TOLERANCE, rel_tol=0):
        position = 0.0
    start = 0.0
    for index, duration in enumerate(program["durations"]):
        end = start + duration
        if position < end - TOLERANCE:
            return index, end - position
        start = end
    return 0, program["durations"][0]


def calendar_features(program, link, absolute_time):
    """Naive phase-by-phase oracle; no helper segment arrays or helper calls."""
    index, remainder = calendar_phase(program, absolute_time)
    durations = program["durations"]
    colors = [state[link].upper() for state in program["states"]]
    original_color = colors[index]
    phase_count = len(durations)
    if len(set(colors)) == 1:
        raise ValueError("Movement has no two finite non-yellow segments")
    if original_color == "Y":
        for _ in range(phase_count):
            index = (index + 1) % phase_count
            remainder = durations[index]
            if colors[index] != "Y":
                break
        else:
            raise ValueError("All-yellow movement")
    first_color, first_duration = colors[index], remainder
    for _ in range(phase_count):
        following = (index + 1) % phase_count
        if colors[following] != first_color:
            index = following
            break
        first_duration += durations[following]
        index = following
    else:
        raise ValueError("Nonfinite first signal segment")
    for _ in range(phase_count):
        if colors[index] != "Y":
            break
        index = (index + 1) % phase_count
    else:
        raise ValueError("Missing next non-yellow segment")
    second_color, second_duration = colors[index], durations[index]
    for _ in range(phase_count):
        following = (index + 1) % phase_count
        if colors[following] != second_color:
            break
        second_duration += durations[following]
        index = following
    else:
        raise ValueError("Nonfinite next signal segment")
    return {
        "entry_signal_type": first_color, "entry_signal_remaining_seconds": first_duration,
        "next_signal_type": second_color, "next_signal_duration_seconds": second_duration,
        "signal_state_at_reference": original_color, "signal_yellow_skipped": int(original_color == "Y"),
    }


def equivalent(first, second):
    return set(first) == set(second) and all(
        math.isclose(first[key], second[key], rel_tol=0, abs_tol=TOLERANCE)
        if isinstance(first[key], float) else first[key] == second[key]
        for key in first
    )


def run_probe(args):
    output = Path(args.output).expanduser().resolve()
    if output.exists():
        raise ValueError("Output exists; choose a new JSON filename")
    net = Path(args.net_file).expanduser().resolve(strict=True)
    binary = Path(shutil.which(args.sumo_binary) or args.sumo_binary).expanduser().resolve(strict=True)
    tools_dir = binary.parent.parent / "tools"
    if not (tools_dir / "traci" / "__init__.py").is_file():
        raise ValueError(f"SUMO tools/traci not found beside {binary}")
    sys.path.insert(0, str(tools_dir))
    import traci
    import traci.constants as tc

    programs, links = load_network(net)
    if not programs or set(programs) != set(links):
        raise ValueError("Every XML TLS must have controlled movement connections")
    version = subprocess.run([str(binary), "--version"], check=True, text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout.strip()
    report = {
        "schema": "real-sumo-rg-clock-probe-v1", "status": "RUNNING",
        "network": {"path": str(net), "sha256": sha256(net)},
        "sumo": {"path": str(binary), "sha256": sha256(binary), "version": version},
        "traci_module": traci.__file__, "helper_sha256": sha256(source_path('signal_rg_features.py')),
        "xml_tls_count": len(programs), "xml_controlled_link_count": sum(map(len, links.values())),
        "duration_seconds": args.duration, "lookaheads_seconds": [0.0, 3.0],
        "checks": {}, "errors": [], "error_count": 0, "runs": [],
        "interpretation": "runtime getter describes the last executed step; features project to planned calendar time e=d or d+3; no vehicles, route/config writes or behavior mutation",
    }
    counters = defaultdict(lambda: {"checks": 0, "failed": 0})

    def check(condition, name, **context):
        counters[name]["checks"] += 1
        if not condition:
            counters[name]["failed"] += 1
            report["error_count"] += 1
            if len(report["errors"]) < 100:
                report["errors"].append({"check": name, **context})

    for step in (1.0, 0.1):
        started = time.monotonic()
        label = "rg_clock_" + uuid.uuid4().hex
        command = [str(binary), "--net-file", str(net), "--step-length", str(step),
                   "--begin", "0", "--end", str(args.duration + 1), "--seed", "42",
                   "--no-step-log", "true", "--duration-log.disable", "true"]
        run = {"step_length": step, "command": command, "snapshots": 0, "movement_queries": 0,
               "examples": [], "tls": {}, "reference_state_counts": {}, "direction_query_counts": {},
               "offset_plus_hypothesis_mismatches": 0, "runtime_calendar_boundary_differences": 0,
               "red_spans_multiple_tls_phases": 0, "green_spans_multiple_tls_phases": 0}
        state_counts, direction_counts = Counter(), Counter()
        connection = None
        try:
            traci.start(command, label=label)
            connection = traci.getConnection(label)
            run["runtime_version"] = list(connection.getVersion())
            runtime_ids = set(connection.trafficlight.getIDList())
            check(runtime_ids == set(programs), "runtime_tls_set", step=step, runtime=sorted(runtime_ids), expected=sorted(programs))
            selected, helpers = {}, {}
            for tls in sorted(runtime_ids & set(programs)):
                current = connection.trafficlight.getProgram(tls)
                check(current in programs[tls], "runtime_program_exists_in_original_net", tls=tls, program=current)
                if current not in programs[tls]:
                    continue
                program = programs[tls][current]
                selected[tls] = program
                logics = [logic for logic in connection.trafficlight.getAllProgramLogics(tls) if logic.programID == current]
                valid = len(logics) == 1 and logics[0].type == 0 and len(logics[0].phases) == len(program["durations"])
                if valid:
                    valid = all(
                        math.isclose(phase.duration, duration, rel_tol=0, abs_tol=TOLERANCE)
                        and phase.state == state
                        and (not phase.next or tuple(phase.next) == ((index + 1) % len(logics[0].phases),))
                        for index, (phase, duration, state) in enumerate(zip(logics[0].phases, program["durations"], program["states"]))
                    )
                check(valid, "runtime_program_matches_original_net_no_override", tls=tls, program=current)
                runtime_links = connection.trafficlight.getControlledLinks(tls)
                expected_links = {
                    index: sorted((item["from_lane"], item["to_lane"], item["via"]) for item in records)
                    for index, records in links[tls].items()
                }
                actual_links = {index: sorted(tuple(item) for item in records)
                                for index, records in enumerate(runtime_links) if records}
                check(actual_links == expected_links, "runtime_links_match_original_net", tls=tls)
                helpers[tls] = {index: RGSignalProgram(program["durations"], [state[index] for state in program["states"]])
                                for index in links[tls]}
                run["tls"][tls] = {"program_id": current, "offset": program["offset"], "cycle": program["cycle"],
                                     "links": sorted(links[tls]), "directions": sorted({item["direction"] for records in links[tls].values() for item in records})}
                connection.trafficlight.subscribe(tls, [tc.TL_CURRENT_PHASE, tc.TL_NEXT_SWITCH, tc.TL_RED_YELLOW_GREEN_STATE])
            sample_targets = {23.0, 24.0, 24.1, 25.0, 27.0, 28.0, 89.0, 90.0, 91.0, 114.0, 118.0}
            for _ in range(int(round(args.duration / step))):
                connection.simulationStep()
                decision = connection.simulation.getTime()
                run["snapshots"] += 1
                check(connection.vehicle.getIDCount() == 0, "no_vehicles", step=step, time=decision)
                for tls, program in selected.items():
                    result = connection.trafficlight.getSubscriptionResults(tls)
                    phase = result[tc.TL_CURRENT_PHASE]
                    switch = result[tc.TL_NEXT_SWITCH]
                    state = result[tc.TL_RED_YELLOW_GREEN_STATE]
                    remaining = switch - decision
                    executed_phase, executed_remaining = calendar_phase(program, decision - step)
                    executed_switch = decision - step + executed_remaining
                    check(phase == executed_phase and math.isclose(switch, executed_switch, rel_tol=0, abs_tol=TOLERANCE),
                          "runtime_anchor_minus_offset_matches_last_executed_step", step=step, tls=tls,
                          time=decision, actual=[phase, switch], expected=[executed_phase, executed_switch])
                    check(state == program["states"][phase], "runtime_state_matches_selected_phase", step=step, tls=tls, time=decision)
                    plus_phase, plus_remaining = calendar_phase(program, decision - step, plus_offset=True)
                    if plus_phase != phase or not math.isclose(decision - step + plus_remaining, switch, rel_tol=0, abs_tol=TOLERANCE):
                        run["offset_plus_hypothesis_mismatches"] += 1
                    calendar_current_phase = calendar_phase(program, decision)[0]
                    if calendar_current_phase != phase:
                        run["runtime_calendar_boundary_differences"] += 1
                    for link, helper in helpers[tls].items():
                        for lookahead in (0.0, 3.0):
                            expected = calendar_features(program, link, decision + lookahead)
                            try:
                                actual = helper.query(phase, remaining, lookahead)
                            except Exception as exc:
                                check(False, "helper_runtime_query", step=step, tls=tls, link=link, time=decision,
                                      phase=phase, remaining=remaining, lookahead=lookahead, error=repr(exc))
                                continue
                            check(equivalent(actual, expected), "all_movements_calendar_feature_match", step=step, tls=tls,
                                  link=link, time=decision, lookahead=lookahead, actual=actual, expected=expected)
                            run["movement_queries"] += 1
                            state_counts[actual["signal_state_at_reference"]] += 1
                            for item in links[tls][link]:
                                direction_counts[item["direction"]] += 1
                            if (lookahead == 0 and calendar_current_phase == phase
                                    and actual["signal_state_at_reference"] in "RG"
                                    and actual["entry_signal_remaining_seconds"] > remaining + TOLERANCE):
                                field = "red_spans_multiple_tls_phases" if actual["signal_state_at_reference"] == "R" else "green_spans_multiple_tls_phases"
                                run[field] += 1
                            if tls == "J17" and link == 0 and lookahead == 0 and any(math.isclose(decision, target, abs_tol=1e-8) for target in sample_targets):
                                # Confirm subscriptions are exactly the ordinary getter view at diagnostic points.
                                check(phase == connection.trafficlight.getPhase(tls)
                                      and switch == connection.trafficlight.getNextSwitch(tls)
                                      and state == connection.trafficlight.getRedYellowGreenState(tls),
                                      "subscription_equals_getters", step=step, tls=tls, time=decision)
                                run["examples"].append({"time": decision, "tls": tls, "link": link,
                                                        "runtime_phase": phase, "runtime_state": state[link],
                                                        "runtime_phase_remaining": remaining, "features": actual})
            check(run["offset_plus_hypothesis_mismatches"] > 0, "offset_sign_discriminated_by_nonzero_offsets", step=step)
            check(run["runtime_calendar_boundary_differences"] > 0, "half_open_boundary_projected_beyond_stale_getter", step=step)
            check(run["red_spans_multiple_tls_phases"] > 0, "movement_red_not_tls_phase_remaining", step=step)
            check(set(state_counts) == {"R", "G", "Y"}, "all_reference_colors_exercised", step=step, counts=dict(state_counts))
            expected_directions = {item["direction"] for tls_links in links.values() for records in tls_links.values() for item in records}
            check(set(direction_counts) == expected_directions, "all_xml_movement_directions_exercised", step=step)
        except Exception as exc:
            check(False, "run_exception", step=step, error=repr(exc))
        finally:
            if connection is not None:
                connection.close()
        run["reference_state_counts"] = dict(state_counts)
        run["direction_query_counts"] = dict(direction_counts)
        run["elapsed_wall_seconds"] = time.monotonic() - started
        report["runs"].append(run)
    check(sha256(net) == report["network"]["sha256"], "source_network_unchanged")
    report["checks"] = dict(counters)
    report["status"] = "PASS" if report["error_count"] == 0 else "FAIL"
    report["limitations"] = [
        "No additional/config inputs loaded: any runtime program/connection mismatch with the original NET fails.",
        "This network need not contain adjacent green TLS phases; synthetic unit tests cover green cross-phase and cycle merging.",
        "Clock conformance only, not model accuracy or vehicle/candidate movement semantics.",
        "Half-open calendar state intentionally differs from stale SUMO getter at an exact switch boundary.",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({"status": report["status"], "output": str(output), "errors": report["error_count"],
                      "feature_queries": sum(run["movement_queries"] for run in report["runs"])}, ensure_ascii=False))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--net-file", required=True)
    parser.add_argument("--sumo-binary", default="sumo")
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=190.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 180 or args.duration != int(args.duration):
        parser.error("--duration must be an integer >= 180 seconds")
    return 0 if run_probe(args)["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
