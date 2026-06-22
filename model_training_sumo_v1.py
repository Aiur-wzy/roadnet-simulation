#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Train an AutoGluon Tabular model on the slim SUMO/CAMS TraCI training schema.

Default input is TraCI_output_adjusted.csv with exactly these model columns:
    has_waiting, road_length, turn_type, road_flow, lane_flow, travel_time_label

The model label is travel_time_no_waiting, copied from travel_time_label, so it
predicts road running time only (driving_time + low_speed_time). Signal and
queue delay are handled by the macro simulator instead of this model.
"""



import argparse
import warnings
from collections import defaultdict
from typing import Dict, Set, Tuple

import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
from sklearn.model_selection import train_test_split

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
SLIM_COLS = [
    "has_waiting",
    "road_length",
    "turn_type",
    "road_flow",
    "lane_flow",
    "travel_time_label",
]
FULL_CAMS_MARKERS = {"vehicleID", "roadID", "movementID", "driving_time", "low_speed_time"}
LEGACY_REQUIRED = ["E_Length", "Driving_Num", "Turn", "Travel_Time", "LowSpee_Time"]
TURN_TYPE_MAP = {"l": 1, "L": 1, "s": 2, "r": 3, "R": 3, "t": 4}


def _encode_legacy_turn(value) -> int:
    if pd.isna(value):
        return 0
    return TURN_TYPE_MAP.get(str(value), 0)


def _detect_schema(csv_path: str):
    columns = list(pd.read_csv(csv_path, nrows=0).columns)
    column_set = set(columns)
    if set(SLIM_COLS).issubset(column_set):
        if FULL_CAMS_MARKERS & column_set:
            print("[INFO] detected full CAMS training schema; using slim column projection")
            return "full_cams", SLIM_COLS
        print("[INFO] detected slim TraCI training schema")
        return "slim", SLIM_COLS
    if set(LEGACY_REQUIRED).issubset(column_set):
        print("[WARN] detected older legacy TraCI schema; using compatibility conversion")
        usecols = sorted(set(LEGACY_REQUIRED + ["Wait_Time", "Delay_Time", "Lanes_Net", "lane_flow", "turn_type", "travel_time_label", "has_waiting"]))
        return "legacy", [c for c in usecols if c in column_set]
    raise ValueError(
        f"[SUMO] Unsupported CSV schema in {csv_path}. Expected slim columns {SLIM_COLS}."
    )


def prepare_sumo_training_frame(
    csv_path: str,
    net_xml_path: str = "",
) -> Tuple[pd.DataFrame, dict, dict, dict]:
    """Load the slim SUMO/CAMS-aligned base-feature training frame."""
    schema, usecols = _detect_schema(csv_path)
    df = pd.read_csv(csv_path, usecols=usecols)

    if schema in {"slim", "full_cams"}:
        missing = [c for c in SLIM_COLS if c not in df.columns]
        if missing:
            raise ValueError(f"[SUMO] Missing required slim columns in {csv_path}: {missing}")
    else:
        df["road_length"] = df["E_Length"].astype(float)
        df["road_flow"] = df["Driving_Num"].fillna(0).astype(float)
        if "lane_flow" not in df.columns:
            raise ValueError("[SUMO] Legacy fallback requires a lane_flow column; refusing to recompute lane_flow from Lanes_Net.")
        if "turn_type" not in df.columns:
            df["turn_type"] = df["Turn"].map(_encode_legacy_turn)
        if "has_waiting" not in df.columns:
            df["has_waiting"] = (df["Wait_Time"].fillna(0).astype(float) > 0).astype(int) if "Wait_Time" in df.columns else 0
        if "travel_time_label" not in df.columns:
            df["travel_time_label"] = df["Travel_Time"].fillna(0).astype(float) + df["LowSpee_Time"].fillna(0).astype(float)

    essential = SLIM_COLS.copy()
    before_drop = len(df)
    df = df.dropna(subset=essential).copy()
    dropped_missing = before_drop - len(df)
    if dropped_missing:
        print(f"[INFO] dropped rows with missing essential columns: {dropped_missing}")

    df["has_waiting"] = df["has_waiting"].fillna(0).astype(int).astype("category")
    df["road_length"] = df["road_length"].astype(float)
    df["turn_type"] = df["turn_type"].fillna(0).astype(int)
    df["road_flow"] = df["road_flow"].fillna(0).astype(float)
    df["lane_flow"] = df["lane_flow"].fillna(0).astype(float)
    df["travel_time_no_waiting"] = df["travel_time_label"].astype(float)

    invalid = (
        (df["travel_time_no_waiting"] <= 0)
        | (df["road_length"] <= 0)
        | (df["road_flow"] < 0)
        | (df["lane_flow"] < 0)
    )
    invalid_count = int(invalid.sum())
    if invalid_count:
        print(f"[INFO] dropped invalid rows: {invalid_count}")
        df = df.loc[~invalid].copy()

    lane_gt_road = int((df["lane_flow"] > df["road_flow"]).sum())
    if lane_gt_road:
        print(f"[WARN] lane_flow > road_flow in {lane_gt_road} rows")

    q99 = df["travel_time_no_waiting"].quantile(0.99)
    before_q99 = len(df)
    df = df[df["travel_time_no_waiting"] <= q99].reset_index(drop=True)
    print(f"[INFO] q99 travel_time_no_waiting threshold: {q99}")
    print(f"[INFO] q99 rows removed: {before_q99 - len(df)}")

    base_feats = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
    print(f"[INFO] selected feature columns: {base_feats}")
    print(f"[INFO] final row count after cleaning: {len(df)}")
    for col in ["road_length", "road_flow", "lane_flow", "travel_time_no_waiting"]:
        print(f"[INFO] {col} min/max: {df[col].min()} / {df[col].max()}")
    print(f"[INFO] unique turn_type values: {sorted(df['turn_type'].dropna().astype(int).unique().tolist())}")
    print(f"[INFO] has_waiting distribution: {df['has_waiting'].value_counts(dropna=False).to_dict()}")

    return df[base_feats + ["travel_time_no_waiting"]], {}, {}, {}

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
    df_processed, _downstream, _upstream, _inter_in_roads = prepare_sumo_training_frame(csv_path, net_xml_path)
    print("data process finished")

    base_feats = [
        "has_waiting",
        "road_length",
        "turn_type",
        "road_flow",
        "lane_flow",
    ]
    label = "travel_time_no_waiting"
    feature_cols = base_feats
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
    p = argparse.ArgumentParser(description="AutoGluon training on slim SUMO/CAMS TraCI records.")
    p.add_argument("--csv", default="TraCI_output_adjusted.csv", help="Slim TraCI training CSV path")
    p.add_argument("--net", default="", help="Optional SUMO net.xml path (unused by the default slim base-feature model)")
    p.add_argument("--save_path", default="models_v1", help="AutoGluon model directory")
    p.add_argument("--time_limit", type=int, default=43200, help="Training time limit (seconds)")
    p.add_argument("--test_size", type=float, default=0.2, help="Test split ratio")
    p.add_argument("--random_state", type=int, default=42, help="Random seed")
    return p


def main():
    args = _build_argparser().parse_args()
    train(
        csv_path=args.csv,
        net_xml_path=args.net,
        save_path=args.save_path,
        time_limit=args.time_limit,
        test_size=args.test_size,
        random_state=args.random_state,
    )

if __name__ == "__main__":
    main()
