#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Train an AutoGluon Tabular model on SUMO/TraCI edge-level records, following the
same baseline training logic as `model_training6.py` (CityFlow version), but
with SUMO net.xml + TraCI_output_adjusted.csv as inputs.

Expected inputs (sample):
- net xml: roadnet_example.txt (SUMO net.xml)  fileciteturn1file7L34-L36
- records csv: track_example.txt (TraCI_output_adjusted.csv) fileciteturn1file14L1-L3

Notes:
- We map SUMO edge -> "intersection" using the downstream junction (edge 'to' attribute).
- Graph neighbor features (competing/downstream/upstream + in/out degree) are computed in the same way
  as model_training6.py's `add_neighbor_features` pipeline. fileciteturn1file1L25-L47
- The TraCI script that produces the CSV writes Wait/Travel/Delay/LowSpeed times per (vehicle, edge). fileciteturn1file4L42-L47
"""



import argparse
import warnings
from collections import defaultdict
from typing import Dict, Set, Tuple

import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder

from autogluon.tabular import TabularPredictor

warnings.filterwarnings("ignore", message="load_learner` uses Python's insecure pickle module")


# ==========================================================
# 1) Parse SUMO net.xml into a "road graph" (downstream/upstream + intersection inbound)
# ==========================================================
def build_sumo_road_graph(net_xml_path: str):
    """
    Build:
      - downstream[edge] = set(next_edges) based on <connection from=... to=...>
      - upstream[edge]   = set(prev_edges)
      - inter_in_roads[junction] = set(inbound_edges) based on edge.to == junction
      - edge2inter[edge] = junction id (edge.to)
    """
    tree = ET.parse(net_xml_path)
    root = tree.getroot()

    downstream: Dict[str, Set[str]] = defaultdict(set)
    upstream: Dict[str, Set[str]] = defaultdict(set)

    # edge -> (from, to) junction mapping
    edge_from = {}
    edge_to = {}

    for edge in root.findall("edge"):
        # skip internal edges
        if "function" in edge.attrib:
            continue
        eid = edge.attrib.get("id")
        if not eid:
            continue
        edge_from[eid] = edge.attrib.get("from")
        edge_to[eid] = edge.attrib.get("to")

    # inbound edges for each junction (same idea as CityFlow inter_in_roads) fileciteturn1file1L65-L70
    inter_in_roads: Dict[str, Set[str]] = defaultdict(set)
    for eid, j_to in edge_to.items():
        if j_to is None:
            continue
        inter_in_roads[j_to].add(eid)

    # connections define legal transitions
    for conn in root.findall("connection"):
        s = conn.attrib.get("from")
        e = conn.attrib.get("to")
        if not s or not e:
            continue
        downstream[s].add(e)
        upstream[e].add(s)

    edge2inter = {eid: j_to for eid, j_to in edge_to.items() if j_to is not None}
    return downstream, upstream, edge2inter, inter_in_roads


def export_degree_combinations(downstream, upstream, out_csv="road_degree_combinations.csv"):
    """
    Same idea as model_training6.py: export per-road degrees + unique (in,out) combos with counts.
    fileciteturn1file11L41-L64
    """
    all_roads = set(downstream.keys()) | set(upstream.keys())
    rows = []
    for rid in sorted(all_roads):
        rows.append(
            {
                "road_id": rid,
                "in_degree": len(upstream.get(rid, [])),
                "out_degree": len(downstream.get(rid, [])),
            }
        )

    df_deg = pd.DataFrame(rows)
    df_deg.to_csv(out_csv.replace(".csv", "_per_road.csv"), index=False)

    df_combo = (
        df_deg.groupby(["in_degree", "out_degree"], as_index=False)
        .size()
        .rename(columns={"size": "count"})
        .sort_values(["in_degree", "out_degree"])
    )
    df_combo.to_csv(out_csv, index=False)
    print(f"[INFO] saved degree combos -> {out_csv}")
    print(f"[INFO] saved per-road degrees -> {out_csv.replace('.csv','_per_road.csv')}")


# ==========================================================
# 2) Build "enter_time-road_id" road-level table to avoid duplication
# ==========================================================
def build_road_time_table(df: pd.DataFrame):
    """
    Align with model_training6.py's expected schema:
      need: enter_time, road_id, inter_id, road_flow, road_waiting_flow
    fileciteturn1file1L1-L19
    """
    need_cols = ["enter_time", "road_id", "inter_id", "road_flow", "road_waiting_flow"]
    for c in need_cols:
        if c not in df.columns:
            raise ValueError(f"Missing required column: {c}")

    road_t = (
        df.groupby(["enter_time", "road_id"], as_index=False)
        .agg(
            {
                "inter_id": "first",
                "road_flow": "mean",
                "road_waiting_flow": "mean",
            }
        )
    )

    eps = 1e-6
    road_t["road_wait_ratio"] = road_t["road_waiting_flow"] / (road_t["road_flow"] + eps)
    return road_t


# ==========================================================
# 3) Neighbor features: competing / downstream / upstream + degrees
# ==========================================================
def add_neighbor_features(
    df: pd.DataFrame,
    road_t: pd.DataFrame,
    downstream: dict,
    upstream: dict,
    inter_in_roads: dict,
):
    """
    Ported from model_training6.py (same feature names).
    fileciteturn1file1L25-L47
    """
    rt_map = {}
    for r in road_t.itertuples(index=False):
        rt_map[(int(r.enter_time), r.road_id)] = {
            "inter_id": r.inter_id,
            "flow": float(r.road_flow),
            "wait": float(r.road_waiting_flow),
            "ratio": float(r.road_wait_ratio),
        }

    eps = 1e-6

    comp_flow, comp_wait, comp_ratio = [], [], []
    down_wait_mean, down_flow_mean, down_ratio_mean = [], [], []
    up_flow_sum, up_wait_sum, up_pressure = [], [], []
    out_deg, in_deg = [], []

    for row in df.itertuples(index=False):
        t = int(getattr(row, "enter_time"))
        rid = getattr(row, "road_id")
        inter_id = getattr(row, "inter_id")

        # A) competing inbound roads at same junction
        inbound_roads = inter_in_roads.get(inter_id, set())
        c_flow = 0.0
        c_wait = 0.0
        for other in inbound_roads:
            if other == rid:
                continue
            v = rt_map.get((t, other))
            if v is None:
                continue
            c_flow += v["flow"]
            c_wait += v["wait"]
        comp_flow.append(c_flow)
        comp_wait.append(c_wait)
        comp_ratio.append(c_wait / (c_flow + eps))

        # B) downstream congestion
        ds = list(downstream.get(rid, []))
        vals = [rt_map.get((t, d)) for d in ds]
        vals = [v for v in vals if v is not None]
        if not vals:
            down_wait_mean.append(0.0)
            down_flow_mean.append(0.0)
            down_ratio_mean.append(0.0)
        else:
            down_wait_mean.append(float(np.mean([v["wait"] for v in vals])))
            down_flow_mean.append(float(np.mean([v["flow"] for v in vals])))
            down_ratio_mean.append(float(np.mean([v["ratio"] for v in vals])))

        # C) upstream pressure (sum flow / |upstream|)
        us = list(upstream.get(rid, []))
        vals = [rt_map.get((t, u)) for u in us]
        vals = [v for v in vals if v is not None]
        if not vals:
            up_flow_sum.append(0.0)
            up_wait_sum.append(0.0)
            up_pressure.append(0.0)
        else:
            sflow = float(np.sum([v["flow"] for v in vals]))
            swait = float(np.sum([v["wait"] for v in vals]))
            up_flow_sum.append(sflow)
            up_wait_sum.append(swait)
            up_pressure.append(sflow / (len(us) + eps))

        out_deg.append(len(downstream.get(rid, [])))
        in_deg.append(len(upstream.get(rid, [])))

    out = df.copy()
    out["competing_road_flow"] = comp_flow
    out["competing_waiting_flow"] = comp_wait
    out["competing_wait_ratio"] = comp_ratio

    out["downstream_wait_mean"] = down_wait_mean
    out["downstream_flow_mean"] = down_flow_mean
    out["downstream_cong_mean"] = down_ratio_mean

    out["upstream_flow_sum"] = up_flow_sum
    out["upstream_wait_sum"] = up_wait_sum
    out["upstream_pressure"] = up_pressure

    out["out_degree"] = out_deg
    out["in_degree"] = in_deg
    return out


# ==========================================================
# 4) SUMO dataset -> baseline schema (enter_time/road_id/inter_id + features/label)
# ==========================================================
def prepare_sumo_training_frame(
    csv_path: str,
    net_xml_path: str,
) -> Tuple[pd.DataFrame, dict, dict, dict]:
    """
    Convert TraCI_output_adjusted.csv into the "baseline" training DataFrame:
      enter_time, road_id, inter_id,
      has_waiting, road_length, turn_type, road_flow, lane_flow,
      road_waiting_flow (for road_t table),
      travel_time_no_waiting (label)

    Source of available columns: TraCI script writes these columns. fileciteturn1file3L14-L24
    """
    raw = pd.read_csv(csv_path)

    # Basic cleaning: drop rows with missing essentials
    must = ["Edge_ID", "Time", "E_Length", "Turn", "Driving_Num", "Wait_Time", "Travel_Time", "Delay_Time", "LowSpee_Time", "Lanes_Net"]
    for c in must:
        if c not in raw.columns:
            raise ValueError(f"[SUMO] Missing required column in {csv_path}: {c}")
    df = raw.dropna(subset=["Edge_ID", "Time", "E_Length", "Turn", "Driving_Num", "Lanes_Net"]).copy()

    # Build graph + edge -> junction mapping
    downstream, upstream, edge2inter, inter_in_roads = build_sumo_road_graph(net_xml_path)

    # Map schema names
    df["enter_time"] = df["Time"].astype(float).round().astype(int)
    df["road_id"] = df["Edge_ID"].astype(str)
    df["inter_id"] = df["road_id"].map(edge2inter).fillna("UNKNOWN_INTER")

    # Base features (match baseline names as much as possible)
    df["road_length"] = df["E_Length"].astype(float)

    # road_flow: use Driving_Num (vehicles on edge at last step) as a flow proxy
    df["road_flow"] = df["Driving_Num"].astype(float)

    # lane_flow: normalize by lanes
    eps = 1e-6
    df["lane_flow"] = df["road_flow"] / (df["Lanes_Net"].astype(float) + eps)

    # turn_type: from Turn (s/t/l/r/L/R/end)
    df["turn_type"] = df["Turn"].astype(str)
    le = LabelEncoder()
    df["turn_type"] = le.fit_transform(df["turn_type"])

    # has_waiting: based on red-light waiting time
    df["has_waiting"] = (df["Wait_Time"].fillna(0).astype(float) > 0).astype("category")

    # Label construction:
    # We only have components, so define:
    #   total_time = Travel_Time + Wait_Time + Delay_Time + LowSpee_Time
    #   travel_time_no_waiting = total_time - Wait_Time = Travel + Delay + LowSpeed
    df["Wait_Time"] = df["Wait_Time"].fillna(0).astype(float)
    df["Travel_Time"] = df["Travel_Time"].fillna(0).astype(float)
    df["Delay_Time"] = df["Delay_Time"].fillna(0).astype(float)
    df["LowSpee_Time"] = df["LowSpee_Time"].fillna(0).astype(float)

    df["travel_time_no_waiting"] = df["Travel_Time"] + df["Delay_Time"] + df["LowSpee_Time"]
    if (df["travel_time_no_waiting"] < 0).any():
        raise ValueError("[SUMO] travel_time_no_waiting has negative values, please check input.")

    # road_waiting_flow: proxy "congestion/wait" at edge level.
    # Use Wait_Sum if present (counts low-speed steps), else use (Wait+Delay+LowSpeed)
    if "Wait_Sum" in df.columns:
        df["road_waiting_flow"] = df["Wait_Sum"].fillna(0).astype(float)
    else:
        df["road_waiting_flow"] = df["Wait_Time"] + df["Delay_Time"] + df["LowSpee_Time"]

    return df, downstream, upstream, inter_in_roads


# ==========================================================
# 5) Main training pipeline (same structure as baseline)
# ==========================================================
def train(
    csv_path: str,
    net_xml_path: str,
    save_path: str,
    time_limit: int,
    test_size: float,
    random_state: int,
):
    df_processed, downstream, upstream, inter_in_roads = prepare_sumo_training_frame(csv_path, net_xml_path)
    print("data process finished")
    # (f) remove extreme outliers (99th percentile), same baseline idea fileciteturn1file2L22-L24
    q99 = df_processed["travel_time_no_waiting"].quantile(0.99)
    df_processed = df_processed[df_processed["travel_time_no_waiting"] <= q99].reset_index(drop=True)

    # Export degree combos for caching/table lookup like the baseline fileciteturn1file11L41-L64
    export_degree_combinations(downstream, upstream, out_csv="road_degree_combinations.csv")
    print("degree finished")

    # Add graph neighbor features
    road_t = build_road_time_table(df_processed)
    print("build road finished")
    #df_processed = add_neighbor_features(df_processed, road_t, downstream, upstream, inter_in_roads)
    print("have add neighbor's feature")
    # Feature set: keep aligned with model_training6.py's chosen subset fileciteturn1file2L35-L47

    base_feats = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
    #graph_feats = ["competing_wait_ratio", "downstream_cong_mean", "upstream_pressure", "out_degree", "in_degree"]
    graph_feats=[]
    label = "travel_time_no_waiting"

    feature_cols = [c for c in (base_feats + graph_feats) if c in df_processed.columns]
    df_selected = df_processed[feature_cols + [label]]

    train_data, test_data = train_test_split(df_selected, test_size=test_size, random_state=random_state)
    print("training start")
    predictor = TabularPredictor(
        label=label,
        problem_type="regression",
        eval_metric="mae",
        path=save_path,
    ).fit(
        train_data,
        presets="best_quality",
        num_bag_folds=10,
        num_stack_levels=2,
        excluded_model_types=["NN_TORCH"],
        time_limit=time_limit,
        ag_args_fit={'num_gpus': 4},
    )

    print("feature metadata:\n", predictor.feature_metadata)
    print("test eval:\n", predictor.evaluate(test_data))
    print("feature importance:\n", predictor.feature_importance(test_data))


def _build_argparser():
    p = argparse.ArgumentParser(description="AutoGluon training on SUMO TraCI edge records (baseline-compatible).")
    p.add_argument("--csv", required=True, help="TraCI_output_adjusted.csv path")
    p.add_argument("--net", required=True, help="SUMO net.xml path")
    p.add_argument("--save_path", default="AutogluonModels_SUMO", help="AutoGluon model directory")
    p.add_argument("--time_limit", type=int, default=43200, help="Training time limit (seconds)")
    p.add_argument("--test_size", type=float, default=0.2, help="Test split ratio")
    p.add_argument("--random_state", type=int, default=42, help="Random seed")
    return p


def main():
    '''args = _build_argparser().parse_args()
    train(
        csv_path=args.csv,
        net_xml_path=args.net,
        save_path=args.save_path,
        time_limit=args.time_limit,
        test_size=args.test_size,
        random_state=args.random_state,
    )'''
    train(
        csv_path="TraCI_output_adjusted_no_change.csv",
        net_xml_path='test.net.xml',
        save_path='models_v1_no_change',
        time_limit=18000,
        test_size=0.2,
        random_state=42,
    )

if __name__ == "__main__":
    main()
