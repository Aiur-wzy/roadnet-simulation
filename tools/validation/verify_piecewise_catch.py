"""Small real-network C++ smoke using a synthetic model, not an MAE experiment."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import catching.model_catching_sumo_cap_context_abc_rg as catch
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, mock_predictor


def formula(frame):
    return (frame.road_length.to_numpy() / frame.free_speed_cap.to_numpy()
            + frame.road_flow.to_numpy() * .125 + frame.lane_flow.to_numpy() * .0625
            + frame.platoon_context.astype(int).to_numpy() * .25
            + frame.entry_signal_remaining_seconds.to_numpy() * .005
            + frame.next_signal_duration_seconds.to_numpy() * .003)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--net-file", type=Path, required=True)
    parser.add_argument("--small-route", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = ET.parse(args.small_route).getroot()
    vehicles = source.findall("vehicle")
    if not 1 <= len(vehicles) <= 64 or source.findall("flow"):
        raise ValueError("Only a bounded fixture of 1..64 explicit vehicles is allowed")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    model, net, contract = prepare_fixture(output)
    net.write_bytes(args.net_file.read_bytes())
    pairs, _ = catch.abc.network_pairs(net)
    lengths = sorted({pair[0] for pair in pairs})
    ranges = contract["training_features"]
    ranges["road_length"] = {"min": min(lengths), "max": max(lengths)}
    ranges["free_speed_cap"] = {"min": 13.89, "max": 13.89}
    ranges["road_flow"] = {"min": 0, "max": 40}
    ranges["lane_flow"] = {"min": 0, "max": 30}
    catalog = catch.signal_catalog(net, 1)
    for column, key in [("entry_signal_remaining_seconds", "entry_max_seconds"),
                        ("next_signal_duration_seconds", "next_duration_seconds")]:
        ranges[column] = {"min": 0, "max": max(item[key] for item in catalog)}
    (model / "training_contract.json").write_text(json.dumps(contract), encoding="utf-8")
    options = catch.build_parser().parse_args([
        "--model-dir", str(model), "--net-file", str(net), "--output-dir", str(output / "table"),
        "--signal-remaining-step", "1", "--road-flow-segments", "10:1", "20:5", "40:20",
        "--max-road-flow", "40", "--max-lane-flow", "30", "--chunk-size", "50000"])
    plan = catch.build_plan(options)
    manifest = catch.export_table(options, plan, mock_predictor(formula))
    trace = output / "trace.csv"
    command = [str(args.binary.resolve()), "--use-sumo", "--sumo-net", str(net),
               "--sumo-route", str(args.small_route.resolve()), "--travel-time-mode", "table",
               "--travel-time-table", str(output / "table/table.rle"),
               "--expected-signal-remaining-step", "1", "--no-eval", "--cap-context-trace", str(trace)]
    process = subprocess.run(command, capture_output=True, text=True, errors="replace", timeout=60)
    (output / "cpp.log").write_text(process.stdout + process.stderr, encoding="utf-8")
    if process.returncode or f"Finished vehicles: {len(vehicles)} / {len(vehicles)}" not in process.stdout:
        raise AssertionError("Simulator did not finish the fixture; see cpp.log")
    with trace.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    routes = {item.get("id"): item.get("edges").split() for item in source.findall("route")}
    expected_queries = sum(len(routes[vehicle.get("route")]) for vehicle in vehicles)
    assert len(rows) == expected_queries
    assert len({(row["sumoVehicleID"], row["route_index"]) for row in rows}) == expected_queries
    # Independent brute-force representative choice, not production road helper.
    global_nodes = [*range(11), 15, 20, 40]
    errors, remapped = [], 0
    for row in rows:
        road, lane = int(row["road_flow"]), int(row["lane_flow"])
        legal = {lane, *(node for node in global_nodes if node >= lane)}
        representative = min(legal, key=lambda node: (abs(node - road), -node))
        remapped += representative != road
        signal = (row["entry_signal_type"], float(row["entry_signal_remaining_seconds"]),
                  row["next_signal_type"], float(row["next_signal_duration_seconds"]))
        _, quantized = catch.quantize_signal(plan["signal_catalog"], *signal)
        case = (float(row["road_length"]), representative, lane, 13.89,
                int(row["platoon_context"]), *quantized)
        expected = max(1., float(formula(catch.feature_frame([case]))[0]))
        actual = float(row["predicted_time"])
        assert math.isclose(actual, expected, abs_tol=1e-10, rel_tol=0), (row, expected)
        errors.append(abs(actual - expected))
    assert remapped > 0, "Fixture must exercise non-node road queries"
    report = {"status": "PASS", "synthetic_model_only": True, "no_accuracy_claim": True,
              "vehicles": len(vehicles), "queries": len(rows), "remapped_queries": remapped,
              "max_lookup_error_seconds": max(errors), "prediction_rows": plan["prediction_rows"],
              "expanded_rows": plan["expanded_rows"], "table_bytes": manifest["file_bytes"],
              "binary_sha256": catch.abc.sha256_file(args.binary), "command": command}
    (output / "results.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
