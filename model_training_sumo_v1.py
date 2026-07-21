#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Train one AutoGluon model from one or many SUMO/CAMS TraCI CSV files.

Every input CSV must contain these model columns (additional metadata columns
are ignored):

    has_waiting, road_length, turn_type, road_flow, lane_flow,
    travel_time_label

The model label is ``travel_time_no_waiting``, copied from
``travel_time_label``.  It predicts road running time only
(``driving_time + low_speed_time``); signal and queue delay remain the macro
simulator's responsibility.

Input examples::

    # One group
    python3 model_training_sumo_v1.py --csv one_group.csv

    # Several explicitly selected groups, merged into one training set/model
    python3 model_training_sumo_v1.py --csv group_a.csv --csv group_b.csv

    # All generated groups (quote the pattern so Python expands it)
    python3 model_training_sumo_v1.py \
        --csv-glob 'train_data/outputs/traci/TraCI_output_adjusted_*.csv' \
        --expected-csv-count 80

The old unconditional P99 deletion is disabled by default.  P99 filtering and
tail sample-weight redistribution are independent opt-in switches.
"""

import argparse
import glob
import os
import warnings
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple, Union

import numpy as np
import pandas as pd
import xml.etree.ElementTree as ET
from sklearn.model_selection import train_test_split

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
BASE_FEATURES = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
LABEL_COL = "travel_time_no_waiting"
SAMPLE_WEIGHT_COL = "__tail_sample_weight"
DEFAULT_CSV = "TraCI_output_adjusted.csv"

FULL_CAMS_MARKERS = {
    "vehicleID",
    "roadID",
    "movementID",
    "vehicle_id",
    "from_edge",
    "to_edge",
    "movement_id",
    "driving_time",
    "low_speed_time",
}
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
            print(f"[INFO] {csv_path}: detected metadata-rich CAMS schema; using slim projection")
            return "full_cams", SLIM_COLS
        print(f"[INFO] {csv_path}: detected slim TraCI training schema")
        return "slim", SLIM_COLS
    if set(LEGACY_REQUIRED).issubset(column_set):
        print(f"[WARN] {csv_path}: detected older legacy schema; using compatibility conversion")
        usecols = sorted(set(LEGACY_REQUIRED + ["Wait_Time", "Delay_Time", "Lanes_Net", "lane_flow", "turn_type", "travel_time_label", "has_waiting"]))
        return "legacy", [c for c in usecols if c in column_set]
    raise ValueError(
        f"[SUMO] Unsupported CSV schema in {csv_path}. Expected slim columns {SLIM_COLS}."
    )


def _logical_abspath(path: str, base_dir: str = "") -> str:
    """Return an absolute path without resolving symlink targets."""
    expanded = os.path.expandvars(os.path.expanduser(path.strip()))
    if not os.path.isabs(expanded):
        expanded = os.path.join(base_dir or os.getcwd(), expanded)
    return os.path.abspath(expanded)


def resolve_csv_inputs(
    csv_paths: Sequence[str],
    csv_globs: Sequence[str],
    csv_lists: Sequence[str],
    expected_count: int = 0,
) -> List[str]:
    """Resolve literal, glob and newline-list inputs into a deterministic file list."""
    candidates: List[str] = []

    for csv_path in csv_paths:
        candidates.append(_logical_abspath(csv_path))

    for pattern in csv_globs:
        expanded_pattern = _logical_abspath(pattern)
        matches = sorted(path for path in glob.glob(expanded_pattern, recursive=True) if os.path.isfile(path))
        if not matches:
            raise ValueError(f"CSV glob matched no files: {pattern}")
        candidates.extend(matches)

    for list_path_raw in csv_lists:
        list_path = _logical_abspath(list_path_raw)
        if not os.path.isfile(list_path):
            raise FileNotFoundError(f"CSV list file not found: {list_path}")
        list_dir = os.path.dirname(list_path)
        with open(list_path, "r", encoding="utf-8-sig") as handle:
            for line_number, raw_line in enumerate(handle, start=1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                path = _logical_abspath(line, base_dir=list_dir)
                if not path:
                    raise ValueError(f"Empty CSV path at {list_path}:{line_number}")
                candidates.append(path)

    if not candidates:
        candidates.append(_logical_abspath(DEFAULT_CSV))

    unique_paths = sorted(set(candidates))
    duplicate_count = len(candidates) - len(unique_paths)
    if duplicate_count:
        print(f"[WARN] ignored duplicate CSV inputs: {duplicate_count}")

    missing = [path for path in unique_paths if not os.path.isfile(path)]
    if missing:
        preview = "\n  ".join(missing[:10])
        suffix = "" if len(missing) <= 10 else f"\n  ... and {len(missing) - 10} more"
        raise FileNotFoundError(f"CSV input files not found:\n  {preview}{suffix}")

    if expected_count < 0:
        raise ValueError("--expected-csv-count must be >= 0")
    if expected_count and len(unique_paths) != expected_count:
        raise ValueError(
            f"Expected {expected_count} CSV files, resolved {len(unique_paths)}. "
            "Refusing to start incomplete or over-broad training."
        )

    print(f"[INFO] resolved CSV files: {len(unique_paths)}")
    for index, path in enumerate(unique_paths, start=1):
        print(f"[INFO] input [{index}/{len(unique_paths)}]: {path}")
    return unique_paths


def _load_one_sumo_training_frame(csv_path: str) -> pd.DataFrame:
    """Load and clean one SUMO/CAMS CSV without performing quantile filtering."""
    schema, usecols = _detect_schema(csv_path)
    df = pd.read_csv(csv_path, usecols=usecols)
    rows_read = len(df)

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
    df[LABEL_COL] = df["travel_time_label"].astype(float)

    invalid = (
        (df[LABEL_COL] <= 0)
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
        print(f"[WARN] {csv_path}: lane_flow > road_flow in {lane_gt_road} rows")

    if df.empty:
        raise ValueError(f"[SUMO] No valid training rows remain in {csv_path}")

    print(f"[INFO] {csv_path}: rows read={rows_read}, valid={len(df)}")
    return df[BASE_FEATURES + [LABEL_COL]].reset_index(drop=True)


def prepare_sumo_training_frame(
    csv_paths: Union[str, Sequence[str]],
    net_xml_path: str = "",
    q99_filter: bool = False,
    q99_quantile: float = 0.99,
) -> Tuple[pd.DataFrame, dict, dict, dict]:
    """Load and merge one or many slim SUMO/CAMS-aligned training frames."""
    del net_xml_path  # Kept for backward-compatible callers; base features do not use the net.

    if isinstance(csv_paths, str):
        normalized_paths = [csv_paths]
    else:
        normalized_paths = list(csv_paths)
    if not normalized_paths:
        raise ValueError("At least one CSV input is required")
    if not 0.0 < q99_quantile < 1.0:
        raise ValueError("--q99-quantile must be between 0 and 1")

    frames = [_load_one_sumo_training_frame(path) for path in normalized_paths]
    df = pd.concat(frames, ignore_index=True, copy=False)
    df["has_waiting"] = df["has_waiting"].astype(int).astype("category")
    print(f"[INFO] merged valid rows before optional P99 filtering: {len(df)}")

    if q99_filter:
        threshold = float(df[LABEL_COL].quantile(q99_quantile))
        before_filter = len(df)
        df = df[df[LABEL_COL] <= threshold].reset_index(drop=True)
        print(f"[INFO] P99 filter: ENABLED (quantile={q99_quantile})")
        print(f"[INFO] P99 threshold for {LABEL_COL}: {threshold}")
        print(f"[INFO] P99 rows removed: {before_filter - len(df)}")
    else:
        df = df.reset_index(drop=True)
        print("[INFO] P99 filter: DISABLED (all valid tail rows retained)")

    if df.empty:
        raise ValueError("No training rows remain after preprocessing")

    print(f"[INFO] selected feature columns: {BASE_FEATURES}")
    print(f"[INFO] final row count after cleaning: {len(df)}")
    for col in ["road_length", "road_flow", "lane_flow", LABEL_COL]:
        print(f"[INFO] {col} min/max: {df[col].min()} / {df[col].max()}")
    print(f"[INFO] unique turn_type values: {sorted(df['turn_type'].dropna().astype(int).unique().tolist())}")
    print(f"[INFO] has_waiting distribution: {df['has_waiting'].value_counts(dropna=False).to_dict()}")

    return df[BASE_FEATURES + [LABEL_COL]], {}, {}, {}

# ==========================================================
# 5) Main training pipeline (same structure as baseline)
# ==========================================================
def add_tail_sample_weights(
    train_data: pd.DataFrame,
    label: str,
    tail_quantile: float,
    tail_weight_factor: float,
) -> Tuple[pd.DataFrame, float]:
    """Upweight the label tail while normalizing weights to sum to row count."""
    if not 0.0 < tail_quantile < 1.0:
        raise ValueError("--tail-quantile must be between 0 and 1")
    if tail_weight_factor < 1.0:
        raise ValueError("--tail-weight-factor must be >= 1")

    threshold = float(train_data[label].quantile(tail_quantile))
    tail_mask = train_data[label] >= threshold
    tail_count = int(tail_mask.sum())
    if tail_count == 0:
        raise ValueError("Tail weighting selected zero rows; check --tail-quantile")

    raw_weights = np.ones(len(train_data), dtype=float)
    raw_weights[tail_mask.to_numpy()] = tail_weight_factor
    normalized_weights = raw_weights * (len(raw_weights) / raw_weights.sum())

    weighted = train_data.copy()
    weighted[SAMPLE_WEIGHT_COL] = normalized_weights
    non_tail_count = len(weighted) - tail_count
    tail_weight = float(weighted.loc[tail_mask, SAMPLE_WEIGHT_COL].iloc[0])
    non_tail_weight = (
        float(weighted.loc[~tail_mask, SAMPLE_WEIGHT_COL].iloc[0])
        if non_tail_count
        else tail_weight
    )

    print("[INFO] tail reweighting: ENABLED")
    print(f"[INFO] tail quantile/threshold: {tail_quantile} / {threshold}")
    print(f"[INFO] tail rows: {tail_count}/{len(weighted)}")
    print(f"[INFO] normalized non-tail/tail weights: {non_tail_weight} / {tail_weight}")
    print(f"[INFO] sample weight sum: {weighted[SAMPLE_WEIGHT_COL].sum()}")
    return weighted, threshold


def train(
    csv_paths: Sequence[str],
    net_xml_path: str,
    save_path: str,
    time_limit: int,
    test_size: float,
    random_state: int,
    q99_filter: bool = False,
    q99_quantile: float = 0.99,
    tail_reweight: bool = False,
    tail_quantile: float = 0.95,
    tail_weight_factor: float = 3.0,
    num_gpus: int = 4,
):
    if time_limit <= 0:
        raise ValueError("--time-limit must be positive")
    if not 0.0 < test_size < 1.0:
        raise ValueError("--test-size must be between 0 and 1")
    if num_gpus < 0:
        raise ValueError("--num-gpus must be >= 0")

    df_processed, _downstream, _upstream, _inter_in_roads = prepare_sumo_training_frame(
        csv_paths,
        net_xml_path,
        q99_filter=q99_filter,
        q99_quantile=q99_quantile,
    )
    print("[INFO] data processing finished")

    df_selected = df_processed[BASE_FEATURES + [LABEL_COL]]
    if len(df_selected) < 2:
        raise ValueError("At least two valid rows are required for train/test splitting")

    train_data, test_data = train_test_split(df_selected, test_size=test_size, random_state=random_state)
    print(f"[INFO] train/test rows: {len(train_data)} / {len(test_data)}")

    predictor_kwargs = {
        "label": LABEL_COL,
        "problem_type": "regression",
        "eval_metric": "mae",
        "path": save_path,
    }
    if tail_reweight:
        train_data, _tail_threshold = add_tail_sample_weights(
            train_data,
            label=LABEL_COL,
            tail_quantile=tail_quantile,
            tail_weight_factor=tail_weight_factor,
        )
        predictor_kwargs["sample_weight"] = SAMPLE_WEIGHT_COL
        predictor_kwargs["weight_evaluation"] = False
    else:
        print("[INFO] tail reweighting: DISABLED (uniform training weights)")

    try:
        from autogluon.tabular import TabularPredictor
    except ImportError as exc:
        raise RuntimeError(
            "AutoGluon is required for training. Install a compatible autogluon.tabular "
            "package in the training environment."
        ) from exc

    Path(save_path).expanduser().parent.mkdir(parents=True, exist_ok=True)
    print("[INFO] training start")
    predictor = TabularPredictor(**predictor_kwargs).fit(
        train_data,
        presets="best_quality",
        num_bag_folds=10,
        num_stack_levels=2,
        excluded_model_types=["NN_TORCH"],
        time_limit=time_limit,
        ag_args_fit={"num_gpus": num_gpus},
    )

    print("feature metadata:\n", predictor.feature_metadata)
    print("test eval:\n", predictor.evaluate(test_data))
    print("feature importance:\n", predictor.feature_importance(test_data))
    return predictor


def _build_argparser():
    p = argparse.ArgumentParser(
        description="AutoGluon training on one or many slim SUMO/CAMS TraCI CSV files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--csv",
        action="append",
        default=[],
        metavar="PATH",
        help="Input CSV path. Repeat --csv to merge multiple explicit files into one model.",
    )
    p.add_argument(
        "--csv-glob",
        "--csv_glob",
        action="append",
        default=[],
        metavar="PATTERN",
        help="Quoted glob pattern for multiple CSV files; may be repeated.",
    )
    p.add_argument(
        "--csv-list",
        "--csv_list",
        action="append",
        default=[],
        metavar="PATH",
        help="Text file containing one CSV path per line; relative entries use the list file directory.",
    )
    p.add_argument(
        "--expected-csv-count",
        "--expected_csv_count",
        type=int,
        default=0,
        help="Fail unless exactly this many unique CSV inputs are resolved; 0 disables the check.",
    )
    p.add_argument("--net", default="", help="Optional SUMO net.xml path (unused by the default slim base-feature model)")
    p.add_argument("--save-path", "--save_path", default="models_v1", help="AutoGluon model directory")
    p.add_argument("--time-limit", "--time_limit", type=int, default=43200, help="Training time limit (seconds)")
    p.add_argument("--test-size", "--test_size", type=float, default=0.2, help="Test split ratio")
    p.add_argument("--random-state", "--random_state", type=int, default=42, help="Random seed")
    p.add_argument("--num-gpus", "--num_gpus", type=int, default=4, help="GPU count passed to AutoGluon model fits; use 0 for CPU")

    q99_group = p.add_mutually_exclusive_group()
    q99_group.add_argument(
        "--q99-filter",
        "--enable-q99-filter",
        dest="q99_filter",
        action="store_true",
        help="Remove label values above the configured upper quantile before splitting.",
    )
    q99_group.add_argument(
        "--no-q99-filter",
        "--disable-q99-filter",
        dest="q99_filter",
        action="store_false",
        help="Keep the full valid label tail.",
    )
    p.set_defaults(q99_filter=False)
    p.add_argument(
        "--q99-quantile",
        "--q99_quantile",
        type=float,
        default=0.99,
        help="Upper-label quantile used only when --q99-filter is enabled.",
    )

    tail_group = p.add_mutually_exclusive_group()
    tail_group.add_argument(
        "--tail-reweight",
        "--enable-tail-reweight",
        dest="tail_reweight",
        action="store_true",
        help="Upweight high-label training rows and renormalize all training weights.",
    )
    tail_group.add_argument(
        "--no-tail-reweight",
        "--disable-tail-reweight",
        dest="tail_reweight",
        action="store_false",
        help="Use uniform sample weights.",
    )
    p.set_defaults(tail_reweight=False)
    p.add_argument(
        "--tail-quantile",
        "--tail_quantile",
        type=float,
        default=0.95,
        help="Training-label quantile defining the tail when reweighting is enabled.",
    )
    p.add_argument(
        "--tail-weight-factor",
        "--tail_weight_factor",
        type=float,
        default=3.0,
        help="Raw tail/non-tail weight ratio before normalization.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve and preprocess all inputs, print settings, then stop before AutoGluon training.",
    )
    return p


def _validate_cli_options(args: argparse.Namespace) -> None:
    if args.expected_csv_count < 0:
        raise ValueError("--expected-csv-count must be >= 0")
    if not 0.0 < args.q99_quantile < 1.0:
        raise ValueError("--q99-quantile must be between 0 and 1")
    if not 0.0 < args.tail_quantile < 1.0:
        raise ValueError("--tail-quantile must be between 0 and 1")
    if args.tail_weight_factor < 1.0:
        raise ValueError("--tail-weight-factor must be >= 1")
    if args.time_limit <= 0:
        raise ValueError("--time-limit must be positive")
    if not 0.0 < args.test_size < 1.0:
        raise ValueError("--test-size must be between 0 and 1")
    if args.num_gpus < 0:
        raise ValueError("--num-gpus must be >= 0")


def main():
    args = _build_argparser().parse_args()
    _validate_cli_options(args)
    csv_paths = resolve_csv_inputs(
        csv_paths=args.csv,
        csv_globs=args.csv_glob,
        csv_lists=args.csv_list,
        expected_count=args.expected_csv_count,
    )

    print(f"[CONFIG] input mode: {'single' if len(csv_paths) == 1 else 'multi-file merge'}")
    print(f"[CONFIG] P99 filter: {'ENABLED' if args.q99_filter else 'DISABLED'}")
    print(f"[CONFIG] tail reweighting: {'ENABLED' if args.tail_reweight else 'DISABLED'}")
    print(f"[CONFIG] save path: {args.save_path}")

    if args.dry_run:
        processed, _downstream, _upstream, _inter_in_roads = prepare_sumo_training_frame(
            csv_paths,
            args.net,
            q99_filter=args.q99_filter,
            q99_quantile=args.q99_quantile,
        )
        if args.tail_reweight:
            dry_train, _dry_test = train_test_split(
                processed,
                test_size=args.test_size,
                random_state=args.random_state,
            )
            add_tail_sample_weights(
                dry_train,
                label=LABEL_COL,
                tail_quantile=args.tail_quantile,
                tail_weight_factor=args.tail_weight_factor,
            )
        else:
            print("[INFO] tail reweighting: DISABLED (uniform training weights)")
        print("[DRY-RUN] input and preprocessing validation passed; AutoGluon was not started")
        return

    train(
        csv_paths=csv_paths,
        net_xml_path=args.net,
        save_path=args.save_path,
        time_limit=args.time_limit,
        test_size=args.test_size,
        random_state=args.random_state,
        q99_filter=args.q99_filter,
        q99_quantile=args.q99_quantile,
        tail_reweight=args.tail_reweight,
        tail_quantile=args.tail_quantile,
        tail_weight_factor=args.tail_weight_factor,
        num_gpus=args.num_gpus,
    )

if __name__ == "__main__":
    main()
