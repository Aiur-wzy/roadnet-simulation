#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import os
import json
import argparse
from pathlib import Path
from typing import Dict

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

from autogluon.tabular import TabularPredictor

# Reuse the EXACT same preprocessing / feature engineering from training script
import model_training_sumo_v1 as mts


# -----------------------------
# Metrics + plotting
# -----------------------------
def safe_mape(y_true: np.ndarray, y_pred: np.ndarray, eps: float = 1e-6) -> float:
    denom = np.maximum(np.abs(y_true), eps)
    return float(np.mean(np.abs((y_pred - y_true) / denom))) * 100.0


def regression_metrics(y_true: np.ndarray, y_pred: np.ndarray) -> Dict[str, float]:
    mae = float(mean_absolute_error(y_true, y_pred))
    rmse = float(mean_squared_error(y_true, y_pred, squared=False))
    r2 = float(r2_score(y_true, y_pred))
    mape = safe_mape(y_true, y_pred)
    err = y_pred - y_true
    return {
        "MAE": mae,
        "RMSE": rmse,
        "R2": r2,
        "MAPE_%": mape,
        "ErrMean": float(np.mean(err)),
        "ErrStd": float(np.std(err)),
        "ErrMedian": float(np.median(err)),
        "AbsErrP90": float(np.quantile(np.abs(err), 0.90)),
        "AbsErrP95": float(np.quantile(np.abs(err), 0.95)),
        "AbsErrP99": float(np.quantile(np.abs(err), 0.99)),
        "N": int(len(y_true)),
    }


def save_plots(df_out: pd.DataFrame, plot_dir: Path) -> None:
    plot_dir.mkdir(parents=True, exist_ok=True)

    y = df_out["y_true"].to_numpy()
    p = df_out["y_pred"].to_numpy()
    e = df_out["error"].to_numpy()

    # 1) True vs Pred
    plt.figure()
    plt.scatter(y, p, s=6, alpha=0.5)
    mn = float(min(y.min(), p.min()))
    mx = float(max(y.max(), p.max()))
    plt.plot([mn, mx], [mn, mx])
    plt.xlabel("True travel_time_no_waiting")
    plt.ylabel("Pred travel_time_no_waiting")
    plt.title("True vs Pred")
    plt.tight_layout()
    plt.savefig(plot_dir / "scatter_true_vs_pred.png", dpi=200)
    plt.close()

    # 2) Residual histogram
    plt.figure()
    plt.hist(e, bins=60)
    plt.xlabel("Residual (pred - true)")
    plt.ylabel("Count")
    plt.title("Residual Histogram")
    plt.tight_layout()
    plt.savefig(plot_dir / "residual_hist.png", dpi=200)
    plt.close()

    # 3) MAE by true deciles
    plt.figure()
    q = pd.qcut(df_out["y_true"], q=10, duplicates="drop")
    byq = df_out.groupby(q)["abs_error"].mean()
    plt.plot(range(len(byq)), byq.to_numpy(), marker="o")
    plt.xlabel("True-value decile (low -> high)")
    plt.ylabel("Mean Abs Error")
    plt.title("MAE by True Deciles")
    plt.tight_layout()
    plt.savefig(plot_dir / "mae_by_true_deciles.png", dpi=200)
    plt.close()

    # 4) Residual vs true
    plt.figure()
    plt.scatter(y, e, s=6, alpha=0.5)
    plt.axhline(0.0)
    plt.xlabel("True travel_time_no_waiting")
    plt.ylabel("Residual (pred - true)")
    plt.title("Residual vs True")
    plt.tight_layout()
    plt.savefig(plot_dir / "residual_vs_true.png", dpi=200)
    plt.close()


def grouped_metrics(df_out: pd.DataFrame, group_col: str) -> pd.DataFrame:
    rows = []
    for key, g in df_out.groupby(group_col):
        y = g["y_true"].to_numpy()
        p = g["y_pred"].to_numpy()
        m = regression_metrics(y, p)
        m[group_col] = key
        rows.append(m)
    return pd.DataFrame(rows).sort_values("MAE", ascending=False)

def _feature_health_report(df: pd.DataFrame, cols: list) -> pd.DataFrame:
    rows = []
    for c in cols:
        s = df[c]
        row = {"col": c, "dtype": str(s.dtype), "null_rate": float(s.isna().mean())}
        # inf rate for numeric
        if pd.api.types.is_numeric_dtype(s):
            sv = pd.to_numeric(s, errors="coerce")
            row["inf_rate"] = float(np.isinf(sv.to_numpy()).mean())
            row["min"] = float(np.nanmin(sv.to_numpy()))
            row["p1"] = float(np.nanpercentile(sv.to_numpy(), 1))
            row["p50"] = float(np.nanpercentile(sv.to_numpy(), 50))
            row["p99"] = float(np.nanpercentile(sv.to_numpy(), 99))
            row["max"] = float(np.nanmax(sv.to_numpy()))
        else:
            # show top categories
            vc = s.astype(str).value_counts(dropna=False).head(10)
            row["top_values"] = "; ".join([f"{k}:{v}" for k, v in vc.items()])
        rows.append(row)
    return pd.DataFrame(rows)


def _topk_bad_cases(df_out: pd.DataFrame, k: int) -> pd.DataFrame:
    # O(N) 取 Top-K abs_error，避免 nlargest 的开销
    abs_e = df_out["abs_error"].to_numpy()
    k = int(min(k, len(abs_e)))
    if k <= 0:
        return df_out.head(0).copy()
    idx = np.argpartition(-abs_e, k - 1)[:k]
    top_df = df_out.iloc[idx].copy()
    top_df = top_df.sort_values("abs_error", ascending=False)
    return top_df


def _group_worst_table(
    df_out: pd.DataFrame,
    group_col: str,
    min_group_size: int,
    top_n: int,
) -> pd.DataFrame:
    # 先用“均值 MAE”粗筛出最差组，再对 top_n 组计算 P95/P99（两段式，避免全量 groupby quantile 太慢）
    g = df_out.groupby(group_col, sort=False)
    base = g.agg(
        count=("abs_error", "size"),
        mae=("abs_error", "mean"),
        max_abs_error=("abs_error", "max"),
        mean_true=("y_true", "mean"),
        mean_pred=("y_pred", "mean"),
    ).reset_index()

    base = base[base["count"] >= int(min_group_size)]
    base = base.sort_values("mae", ascending=False).head(int(top_n))

    if base.empty:
        return base

    keys = set(base[group_col].tolist())
    sub = df_out[df_out[group_col].isin(keys)]
    q = sub.groupby(group_col)["abs_error"].quantile([0.95, 0.99]).unstack().reset_index()
    q.columns = [group_col, "abs_err_p95", "abs_err_p99"]

    out = base.merge(q, on=group_col, how="left")
    out = out.sort_values("mae", ascending=False)
    return out


def run_diagnostics(
    df_out: pd.DataFrame,
    out_dir: Path,
    feature_cols: list,
    topk: int = 2000,
    min_group_size: int = 5000,
    top_group_n: int = 200,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    # --- A) y_true / abs_error 基础分布（解释 MAPE 爆炸：y_true≈0 时会爆）
    y = df_out["y_true"].to_numpy()
    abs_e = df_out["abs_error"].to_numpy()
    eps1 = 1.0
    eps01 = 0.1
    summary = {
        "N": int(len(df_out)),
        "y_true_min": float(np.min(y)),
        "y_true_p1": float(np.quantile(y, 0.01)),
        "y_true_p50": float(np.quantile(y, 0.50)),
        "y_true_p99": float(np.quantile(y, 0.99)),
        "y_true_max": float(np.max(y)),
        "frac_y_true_lt_1": float(np.mean(y < eps1)),
        "frac_y_true_lt_0p1": float(np.mean(y < eps01)),
        "abs_err_p90": float(np.quantile(abs_e, 0.90)),
        "abs_err_p95": float(np.quantile(abs_e, 0.95)),
        "abs_err_p99": float(np.quantile(abs_e, 0.99)),
        "abs_err_max": float(np.max(abs_e)),
    }
    with open(out_dir / "diagnostics_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    # --- B) Top-K 最差样本（直接定位具体行：road_id / inter_id / enter_time）
    top_df = _topk_bad_cases(df_out, topk)
    top_df.to_csv(out_dir / f"bad_cases_top{topk}.csv", index=False)

    # --- C) 最差 road_id / inter_id（你最关心的“哪里误差大”）
    if "road_id" in df_out.columns:
        worst_roads = _group_worst_table(df_out, "road_id", min_group_size, top_group_n)
        worst_roads.to_csv(out_dir / "worst_road_id.csv", index=False)

    if "inter_id" in df_out.columns:
        worst_inters = _group_worst_table(df_out, "inter_id", min_group_size, top_group_n)
        worst_inters.to_csv(out_dir / "worst_inter_id.csv", index=False)

        # 特别检查 UNKNOWN_INTER（训练脚本里 inter_id 会 fillna 为 UNKNOWN_INTER）:contentReference[oaicite:3]{index=3}
        unk = df_out[df_out["inter_id"] == "UNKNOWN_INTER"]
        if len(unk) > 0:
            unk_stats = {
                "count": int(len(unk)),
                "mae": float(unk["abs_error"].mean()),
                "abs_err_p95": float(unk["abs_error"].quantile(0.95)),
                "abs_err_p99": float(unk["abs_error"].quantile(0.99)),
                "ratio": float(len(unk) / max(len(df_out), 1)),
            }
            with open(out_dir / "unknown_inter_stats.json", "w", encoding="utf-8") as f:
                json.dump(unk_stats, f, ensure_ascii=False, indent=2)

    # --- D) turn_type / has_waiting 组合（看看是不是“某类转向 + 等灯”崩）
    cols = []
    if "turn_type" in df_out.columns:
        cols.append("turn_type")
    if "has_waiting" in df_out.columns:
        cols.append("has_waiting")
    if len(cols) > 0:
        comb = df_out.groupby(cols).agg(
            count=("abs_error", "size"),
            mae=("abs_error", "mean"),
            max_abs_error=("abs_error", "max"),
        ).reset_index()
        comb = comb.sort_values("mae", ascending=False)
        comb.to_csv(out_dir / "worst_turn_wait_combo.csv", index=False)

    # --- E) 特征健康检查（NaN/inf/异常范围/类别分布）
    health = _feature_health_report(df_out, feature_cols + ["y_true", "y_pred", "error", "abs_error"])
    health.to_csv(out_dir / "feature_health.csv", index=False)

    # --- F) 轻量打印：Top 10 最差 road_id / inter_id
    def _print_top(path: Path, name: str):
        if path.exists():
            d = pd.read_csv(path)
            print(f"\n[DIAG] Top 10 worst {name} (by MAE, count>={min_group_size}):")
            print(d.head(10).to_string(index=False))

    _print_top(out_dir / "worst_road_id.csv", "road_id")
    _print_top(out_dir / "worst_inter_id.csv", "inter_id")

    print(f"\n[DIAG] Saved diagnostics to: {out_dir.resolve()}")
    print(f"  - diagnostics_summary.json")
    print(f"  - bad_cases_top{topk}.csv")
    print(f"  - worst_road_id.csv / worst_inter_id.csv")
    print(f"  - worst_turn_wait_combo.csv")
    print(f"  - feature_health.csv")
# -----------------------------
# Core evaluation
# -----------------------------
def evaluate(model_dir: str, net_xml: str, csv_path: str, out_dir: str) -> None:
    out_dir_p = Path(out_dir)
    out_dir_p.mkdir(parents=True, exist_ok=True)

    # 1) Build validation features exactly like training
    df_base, downstream, upstream, inter_in_roads = mts.prepare_sumo_training_frame(csv_path, net_xml)
    road_t = mts.build_road_time_table(df_base)
    df_feat = mts.add_neighbor_features(df_base, road_t, downstream, upstream, inter_in_roads)

    # 2) Feature columns consistent with training
    base_feats = ["has_waiting", "road_length", "turn_type", "road_flow", "lane_flow"]
    graph_feats =[]
    #graph_feats = ["competing_wait_ratio", "downstream_cong_mean", "upstream_pressure", "out_degree", "in_degree"]
    label = "travel_time_no_waiting"
    feature_cols = [c for c in (base_feats + graph_feats) if c in df_feat.columns]

    # sanity checks
    missing = [c for c in feature_cols + [label] if c not in df_feat.columns]
    if missing:
        raise ValueError(f"Missing columns after preprocessing: {missing}")

    # 3) Load model + predict
    predictor = TabularPredictor.load(model_dir)
    X = df_feat[feature_cols].copy()
    y_true = df_feat[label].astype(float).to_numpy()
    y_pred = predictor.predict(X).astype(float).to_numpy()

    # 4) Build output table
    df_out = df_feat.copy()
    df_out["y_true"] = y_true
    df_out["y_pred"] = y_pred
    df_out["error"] = df_out["y_pred"] - df_out["y_true"]
    df_out["abs_error"] = df_out["error"].abs()

    # 5) Metrics
    metrics = regression_metrics(y_true, y_pred)
    print("[RESULT] Overall metrics:")
    for k, v in metrics.items():
        print(f"  {k}: {v}")

    with open(out_dir_p / "metrics.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    # 诊断输出：定位“哪里误差大”
    run_diagnostics(
        df_out=df_out,
        out_dir=out_dir_p / "diagnostics",
        feature_cols=feature_cols,
        topk=2000,
        min_group_size=5000,
        top_group_n=200,
    )

    '''# 6) Save artifacts
    df_out.to_csv(out_dir_p / "predictions.csv", index=False)
    with open(out_dir_p / "metrics.json", "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    # grouped reports (only if columns exist)
    for col in ["has_waiting", "turn_type", "road_id", "in_degree", "out_degree"]:
        if col in df_out.columns:
            gdf = grouped_metrics(df_out, col)
            gdf.to_csv(out_dir_p / f"group_metrics_by_{col}.csv", index=False)

    # plots
    save_plots(df_out, out_dir_p / "plots")

    print(f"[DONE] Saved to: {out_dir_p.resolve()}")
    print(f"  - predictions.csv")
    print(f"  - metrics.json")
    print(f"  - group_metrics_by_*.csv")
    print(f"  - plots/*.png")'''





if __name__ == "__main__":
    #args = build_argparser().parse_args()
    evaluate(
        model_dir="models_v1_no_change/",
        net_xml="test.net.xml",
        csv_path="TraCI_output_adjusted_no_change.csv",
        out_dir="eval_out_v1_no_change",
    )
