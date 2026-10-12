#!/usr/bin/env python3
"""Small, explicitly MOCK nine-feature integration test, not an accuracy experiment.

Reuses recorded real collector rows/audits, mocks only AutoGluon fit/predict,
exports the real RLE format and checks every C++ query against the Python clock,
quantizer and exported values. Never replace a trained model with this fixture.
"""
import argparse
import contextlib
import json
import math
from pathlib import Path
import subprocess
import sys
import types
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np
import pandas as pd
import training.model_training_sumo_cap_context_abc_rg as train
import catching.model_catching_sumo_cap_context_abc_rg as catch
from features.signal_rg_features import RGSignalProgram, network_green_horizon


class MockPredictor:
    """Deliberately depends on all nine columns, including unordered categories."""
    label = train.LABEL_COL
    problem_type = "regression"
    feature_metadata = types.SimpleNamespace(get_features=lambda: train.FEATURE_COLS)
    feature_metadata_in = types.SimpleNamespace(
        get_features=lambda: train.FEATURE_COLS,
        get_feature_type_raw=lambda name: "category" if name in train.CATEGORICAL_FEATURES else "float")

    def __init__(self, path=None, **kwargs):
        if path is not None:
            output = Path(path)
            output.mkdir(parents=True, exist_ok=False)
            (output / "predictor.pkl").write_bytes(b"MOCK FIXTURE ONLY - NOT AN AUTOGLUON PICKLE\n")

    def fit(self, train_data, **kwargs):
        assert list(train_data) == train.FEATURE_COLS + [train.LABEL_COL]
        self.check(train_data)
        return self

    @staticmethod
    def check(frame):
        for name, values in train.CATEGORICAL_FEATURES.items():
            assert isinstance(frame[name].dtype, pd.CategoricalDtype)
            assert frame[name].cat.categories.tolist() == values
            assert frame[name].cat.ordered is False

    def predict(self, frame):
        self.check(frame)
        return (frame.road_length.to_numpy() / frame.free_speed_cap.to_numpy()
                + .05 * (frame.road_flow.to_numpy() // 3) + .07 * frame.lane_flow.to_numpy()
                + .02 * frame.platoon_context.astype(int).to_numpy()
                + .03 * (frame.entry_signal_type == "R").to_numpy()
                + .001 * frame.entry_signal_remaining_seconds.to_numpy()
                + .04 * (frame.next_signal_type == "R").to_numpy()
                + .002 * frame.next_signal_duration_seconds.to_numpy())


def prepare(args):
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    model = output / "MOCK_NOT_A_TRAINED_MODEL"
    training = train.load_frame(Path(args.training_csv), train.DEFAULT_LABEL_COLUMN)
    validation = train.load_frame(Path(args.validation_csv), train.DEFAULT_LABEL_COLUMN)
    options = train.build_parser().parse_args([
        "--save-path", str(model), "--sample-weighting", "none", "--num-gpus", "0"])
    options.label_column = train.DEFAULT_LABEL_COLUMN
    package, tabular = types.ModuleType("autogluon"), types.ModuleType("autogluon.tabular")
    tabular.TabularPredictor = MockPredictor
    with mock.patch.dict(sys.modules, {"autogluon": package, "autogluon.tabular": tabular}):
        predictor = train.fit_model(training, validation, options, {"MOCK_ONLY": True})
    catch_args = catch.build_parser().parse_args([
        "--model-dir", str(model), "--net-file", args.net,
        "--output-dir", str(output / "mock_catch"), "--signal-remaining-step", str(args.signal_remaining_step),
        "--cap-min", str(args.cap_min), "--cap-max", str(args.cap_max),
        "--cap-step", str(args.cap_step),
        "--max-road-flow", str(args.max_road_flow), "--max-lane-flow", str(args.max_lane_flow),
        "--dense-reference", "--max-rows", "10000000"])
    plan = catch.build_plan(catch_args)
    print("[MOCK PLAN]", plan["expanded_rows"], "rows", flush=True)
    manifest = catch.export_table(catch_args, plan, predictor)
    print("[MOCK EXPORTED]", manifest["rows"], manifest["run_count"], flush=True)


def run_verify(args):
    output = Path(args.output).resolve()
    table = output / "mock_catch" / "table.rle"
    trace = output / "cpp_trace.csv"
    tripinfo = output / "cpp_tripinfo.xml"
    if not args.reuse_cpp_outputs and (trace.exists() or tripinfo.exists()):
        raise ValueError("Verification output exists; choose a new output directory")
    command = [args.binary, "--use-sumo", "--sumo-net", args.net, "--sumo-route", args.route,
               "--travel-time-mode", "table", "--travel-time-table", str(table),
               "--cap-context-trace", str(trace), "--prediction-tripinfo-output", str(tripinfo),
               "--sumo-tripinfo", args.truth, "--eval-output", str(output / "mock_eval.csv")]
    if not args.reuse_cpp_outputs:
        result = subprocess.run(command, text=True, capture_output=True, errors="replace")
        (output / "cpp.log").write_text(result.stdout + result.stderr, encoding="utf-8")
        if result.returncode:
            raise RuntimeError(f"C++ failed: {result.returncode}; see {output / 'cpp.log'}")
    header, values = catch.read_table(table)
    reference = np.fromfile(output / "mock_catch" / "reference.f64", dtype="<f8")
    assert np.array_equal(values, reference)
    net = ET.parse(args.net).getroot()
    tls = {item.attrib["id"]: item for item in net.findall("tlLogic")}
    edge_nodes = {edge.get("id"): edge.get("to") for edge in net.findall("edge")}
    node_types = {node.get("id"): node.get("type") for node in net.findall("junction")}
    routes_root = ET.parse(args.route).getroot()
    routes = {item.attrib["id"]: item.attrib["edges"].split() for item in routes_root.findall("route")}
    vehicles = {item.attrib["id"]: routes[item.attrib["route"]] for item in routes_root.findall("vehicle")}
    rows = pd.read_csv(trace, keep_default_na=False)
    controlled = terminal = yellow = 0
    raw_values, node_values, signal_errors, lookup_errors = [], [], [], []
    max_clock_error = 0.
    grid = header["free_speed_cap_grid"]
    caps = grid["index_max"] - grid["index_min"] + 1
    for row in rows.to_dict("records"):
        idx, route = int(row["route_index"]), vehicles[str(row["sumoVehicleID"])]
        e, d = float(row["planned_enter_time"]), float(row["decision_time"])
        crossing = header["context_contract"]["base_context_contract"]["crossing_time_seconds"] if idx else 0.
        if idx and header["signal_contract"]["version"] in {"static-movement-rg-segments-v3", "static-movement-rg-segments-v4"}:
            incoming = [c for c in net.findall("connection")
                        if c.get("from") == route[idx - 1] and c.get("to") == route[idx]]
            assert incoming
            node = edge_nodes.get(route[idx - 1])
            if node_types.get(node) in {"priority", "priority_stop", "right_before_left", "allway_stop", "zipper", "unregulated", "dead_end", "district"} and not any(c.get("tl") for c in incoming):
                crossing = 0.
        if (header["signal_contract"]["version"] == "static-movement-rg-segments-v4"
                and not any(edge.get("function") == "internal" or edge.get("id", "").startswith(":") for edge in net.findall("edge"))):
            assert not any(c.get("via") for c in net.findall("connection"))
            crossing = 0.
        assert abs(e - d - crossing) < 1e-8
        assert route[idx] == row["sumoRoadID"]
        if idx == len(route) - 1:
            expected = ("G", 0., "G", 0.)
            terminal += 1
        else:
            candidates = [c for c in net.findall("connection")
                          if c.get("from") == route[idx] and c.get("to") == route[idx + 1]
                          and int(c.get("fromLane", -1)) == int(row["laneIndex"])]
            assert candidates
            c = candidates[0]
            if "tl" not in c.attrib:
                horizon, _ = network_green_horizon(args.net)
                expected = ("G", horizon, "G", horizon)
            else:
                program = tls[c.attrib["tl"]]
                link = int(c.attrib["linkIndex"])
                phases = program.findall("phase")
                p = RGSignalProgram([float(x.attrib["duration"]) for x in phases],
                                    [x.attrib["state"][link] for x in phases])
                local = (e - float(program.get("offset", 0))) % p.cycle_seconds
                actual = p.query(0, p.durations[0], local)
                expected = tuple(actual[name] for name in train.SIGNAL_FEATURE_COLS)
                controlled += 1
                yellow += int(actual["signal_yellow_skipped"])
                assert row["signal_tls_id"] == c.attrib["tl"]
        raw = tuple(row[name] for name in train.SIGNAL_FEATURE_COLS)
        assert raw[0] == expected[0] and raw[2] == expected[2]
        clock_error = max(abs(raw[1] - expected[1]), abs(raw[3] - expected[3]))
        max_clock_error = max(max_clock_error, clock_error)
        assert clock_error < 1e-8
        point, node = catch.quantize_signal(header["signal_catalog"], *raw)
        assert abs(node[1] - row["table_entry_signal_remaining_seconds"]) < 1e-10
        length = header["road_lengths"].index(row["road_length"])
        cap_idx = math.floor(row["free_speed_cap"] / grid["step"] + .5 + 1e-9)
        lane, road = int(row["lane_flow"]), int(row["road_flow"])
        block = (((point * len(header["road_lengths"]) + length) * caps
                  + cap_idx - grid["index_min"]) * 6 + int(row["platoon_context"]))
        flow_offset = lane * (header["road_flow_max"] + 1) - lane * (lane - 1) // 2 + road - lane
        table_value = values[block * header["flow_count"] + flow_offset]
        lookup_errors.append(abs(table_value - row["predicted_time"]))
        assert lookup_errors[-1] < 1e-10
        base = [row[name] for name in train.FEATURE_COLS[:5]]
        base[3] = cap_idx * grid["step"]
        raw_values.append(base + list(raw))
        node_values.append(base + list(node))
        signal_errors.append(node[1] - raw[1])
    raw_prediction = MockPredictor().predict(catch.feature_frame(raw_values))
    node_prediction = MockPredictor().predict(catch.feature_frame(node_values))
    assert len(ET.parse(tripinfo).getroot().findall("tripinfo")) == len(vehicles)
    assert len(rows) == sum(map(len, vehicles.values()))
    report = {"status": "PASS", "MOCK_ONLY": True, "vehicles": len(vehicles), "road_queries": len(rows),
              "controlled_queries": controlled, "terminal_queries": terminal, "yellow_queries": yellow,
              "signal_step_seconds": header["signal_remaining_step"], "dense_rle_exact": True,
              "max_clock_difference_seconds": max_clock_error,
              "max_cpp_python_lookup_difference_seconds": max(lookup_errors),
              "signal_remaining_quantization_mae_seconds": float(np.abs(signal_errors).mean()),
              "signal_remaining_quantization_max_seconds": float(np.abs(signal_errors).max()),
              "mock_same_input_quantization_prediction_mae_seconds": float(np.abs(node_prediction - raw_prediction).mean()),
              "accuracy_claim": "None: synthetic predictor; no real training or closed-loop MAE improvement evaluated",
              "command": command}
    (output / "verification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["prepare", "verify"])
    for name in ("output", "net", "route", "training-csv", "validation-csv", "binary", "truth"):
        parser.add_argument("--" + name, required=name in ("output", "net"))
    parser.add_argument("--cap-min", type=float, default=11.)
    parser.add_argument("--cap-max", type=float, default=14.)
    parser.add_argument("--cap-step", type=float, default=.1389)
    parser.add_argument("--max-road-flow", type=int, default=12)
    parser.add_argument("--max-lane-flow", type=int, default=8)
    parser.add_argument("--signal-remaining-step", type=float, default=1.,
                        help="Mock export remaining-time node spacing in seconds; verification reads the table header")
    parser.add_argument("--reuse-cpp-outputs", action="store_true")
    args = parser.parse_args()
    (prepare if args.action == "prepare" else run_verify)(args)


if __name__ == "__main__":
    main()
