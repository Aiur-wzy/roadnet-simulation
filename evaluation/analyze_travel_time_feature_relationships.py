#!/usr/bin/env python3
"""Analyze feature relationships with pre-waiting-buffer travel-time labels.

The script is designed for the full CSVs written by
``TraCI_Python_Threshold_Sweep.py --outputs training``.  It analyzes every
``travel_time_label_gt_*`` column by default and produces five focused views:

1. Pearson feature/label correlation;
2. Spearman feature/label correlation;
3. mutual information;
4. within-stratum correlations after controlling for road length, turn type,
   and movement; and
5. label dispersion among rows with the exact same five input features.

Example::

    python -m evaluation.analyze_travel_time_feature_relationships \
      --csv-glob "train_data/threshold_sweep/cams_model_training_data_*.csv" \
      --output-dir "eval/feature_label_analysis"

The analysis intentionally does not train a prediction model or create plots.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import numpy as np
    import pandas as pd
    from sklearn.feature_selection import mutual_info_regression
except ImportError as exc:  # pragma: no cover - depends on the local environment
    raise SystemExit(
        "Missing analysis dependency. Install with: "
        "python -m pip install pandas numpy scikit-learn"
    ) from exc


DEFAULT_FEATURE_COLUMNS = (
    "has_waiting",
    "road_length",
    "turn_type",
    "road_flow",
    "lane_flow",
)
DISCRETE_FEATURES = {"has_waiting", "turn_type"}
MOVEMENT_ALIASES = ("movementID", "movement_id", "movementId", "movement")
LABEL_PREFIX = "travel_time_label_gt_"
LEGACY_LABEL_ALIASES = ("travel_time_label", "travel_time_no_waiting")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure Pearson, Spearman, mutual information, conditional "
            "correlations, and exact-feature-cell label variability."
        )
    )
    parser.add_argument(
        "--csv",
        action="append",
        default=[],
        help="Input CSV path. Repeat the option to provide multiple files.",
    )
    parser.add_argument(
        "--csv-glob",
        action="append",
        default=[],
        help=(
            "Glob for input CSVs, for example 'train_data/run/*.csv'. "
            "Repeatable; recursive '**' patterns are supported."
        ),
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory in which analysis CSVs and the manifest are written.",
    )
    parser.add_argument(
        "--label-columns",
        default="",
        help=(
            "Comma-separated labels to analyze. By default all "
            "travel_time_label_gt_* columns are used; legacy labels are a "
            "fallback when threshold-sweep labels are absent."
        ),
    )
    parser.add_argument(
        "--feature-columns",
        default=",".join(DEFAULT_FEATURE_COLUMNS),
        help=(
            "Comma-separated feature subset. Available features: {}. "
            "Custom subsets currently require --skip-conditional."
        ).format(", ".join(DEFAULT_FEATURE_COLUMNS)),
    )
    parser.add_argument(
        "--movement-column",
        default="",
        help="Movement column name. Common names are detected automatically.",
    )
    parser.add_argument(
        "--skip-conditional",
        action="store_true",
        help=(
            "Skip Movement-controlled conditional correlations. Use this "
            "explicitly for legacy six-column CSVs that contain no Movement."
        ),
    )
    parser.add_argument(
        "--min-conditional-group-size",
        type=int,
        default=20,
        help="Minimum rows per road-length/turn/movement stratum (default: 20).",
    )
    parser.add_argument(
        "--mi-max-samples",
        type=int,
        default=200000,
        help="Maximum deterministic sample used for mutual information.",
    )
    parser.add_argument(
        "--random-state",
        type=int,
        default=42,
        help="Random seed used only when mutual-information sampling is needed.",
    )
    args = parser.parse_args(argv)

    if args.min_conditional_group_size < 2:
        parser.error("--min-conditional-group-size must be at least 2")
    if args.mi_max_samples < 10:
        parser.error("--mi-max-samples must be at least 10")
    if not args.csv and not args.csv_glob:
        parser.error("provide at least one --csv or --csv-glob")
    feature_columns = [
        part.strip() for part in args.feature_columns.split(",") if part.strip()
    ]
    if not feature_columns:
        parser.error("--feature-columns must contain at least one feature")
    if len(feature_columns) != len(set(feature_columns)):
        parser.error("--feature-columns must not contain duplicates")
    unknown_features = [
        feature for feature in feature_columns if feature not in DEFAULT_FEATURE_COLUMNS
    ]
    if unknown_features:
        parser.error(
            "unknown --feature-columns values: {}".format(", ".join(unknown_features))
        )
    if tuple(feature_columns) != DEFAULT_FEATURE_COLUMNS and not args.skip_conditional:
        parser.error("custom --feature-columns currently require --skip-conditional")
    args.feature_columns = tuple(feature_columns)
    return args


def resolve_input_files(paths: Sequence[str], patterns: Sequence[str]) -> List[Path]:
    resolved: Dict[str, Path] = {}
    for raw_path in paths:
        path = Path(raw_path).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError("input CSV does not exist: {}".format(path))
        resolved[str(path).casefold()] = path

    for pattern in patterns:
        matches = glob.glob(str(Path(pattern).expanduser()), recursive=True)
        if not matches:
            raise FileNotFoundError("CSV glob matched no files: {}".format(pattern))
        for match in matches:
            path = Path(match).resolve()
            if path.is_file():
                resolved[str(path).casefold()] = path

    files = sorted(resolved.values(), key=lambda item: str(item).casefold())
    if not files:
        raise FileNotFoundError("no input CSV files were found")
    return files


def parse_threshold_from_label(column: str) -> float:
    token = column[len(LABEL_PREFIX):].replace("m", "-").replace("p", ".")
    try:
        return float(token)
    except ValueError:
        return math.inf


def detect_label_columns(columns: Iterable[str], requested: str) -> List[str]:
    available = list(columns)
    available_set = set(available)
    if requested.strip():
        labels = [part.strip() for part in requested.split(",") if part.strip()]
        missing = [label for label in labels if label not in available_set]
        if missing:
            raise ValueError("requested label columns are missing: {}".format(missing))
        return labels

    threshold_labels = [
        column for column in available if column.startswith(LABEL_PREFIX)
    ]
    if threshold_labels:
        return sorted(threshold_labels, key=parse_threshold_from_label)

    for fallback in LEGACY_LABEL_ALIASES:
        if fallback in available_set:
            return [fallback]
    raise ValueError(
        "no travel-time labels found; expected travel_time_label_gt_* or {}".format(
            ", ".join(LEGACY_LABEL_ALIASES)
        )
    )


def detect_movement_column(columns: Iterable[str], requested: str) -> str:
    available = set(columns)
    if requested:
        if requested not in available:
            raise ValueError(
                "requested movement column {!r} is missing".format(requested)
            )
        return requested
    for alias in MOVEMENT_ALIASES:
        if alias in available:
            return alias
    raise ValueError(
        "a Movement column is required for conditional analysis. Use the full "
        "training CSV produced by TraCI_Python_Threshold_Sweep.py with "
        "--outputs training, or pass --movement-column."
    )


def inspect_inputs(
    files: Sequence[Path],
    feature_columns: Sequence[str],
    requested_labels: str,
    requested_movement: str,
    skip_conditional: bool,
) -> Tuple[List[str], Optional[str]]:
    first_columns = list(pd.read_csv(files[0], nrows=0).columns)
    labels = detect_label_columns(first_columns, requested_labels)
    movement_column = (
        None
        if skip_conditional
        else detect_movement_column(first_columns, requested_movement)
    )
    required = set(feature_columns) | set(labels)
    if movement_column is not None:
        required.add(movement_column)

    for path in files:
        columns = set(pd.read_csv(path, nrows=0).columns)
        missing = sorted(required - columns)
        if missing:
            raise ValueError("{} is missing columns: {}".format(path, missing))
    return labels, movement_column


def load_inputs(
    files: Sequence[Path],
    feature_columns: Sequence[str],
    labels: Sequence[str],
    movement_column: Optional[str],
) -> Tuple[pd.DataFrame, Dict[str, object]]:
    usecols = list(feature_columns) + list(labels)
    if movement_column is not None:
        usecols.append(movement_column)
    frames: List[pd.DataFrame] = []
    file_rows: Dict[str, int] = {}
    for path in files:
        frame = pd.read_csv(path, usecols=usecols, low_memory=False)
        file_rows[str(path)] = int(len(frame))
        frames.append(frame)

    data = pd.concat(frames, ignore_index=True)
    input_rows = int(len(data))
    for column in list(feature_columns) + list(labels):
        data[column] = pd.to_numeric(data[column], errors="coerce")

    if movement_column is not None:
        movement_text = data[movement_column].astype("string").str.strip()
        invalid_movement = movement_text.isna() | movement_text.eq("")
        data["__movement"] = movement_text
    else:
        invalid_movement = pd.Series(False, index=data.index)

    finite_features = np.isfinite(data[list(feature_columns)].to_numpy(dtype=float))
    valid_feature_mask = finite_features.all(axis=1) & ~invalid_movement.to_numpy()
    invalid_base_rows = int((~valid_feature_mask).sum())
    data = data.loc[valid_feature_mask].copy()

    label_invalid_rows = {}
    for label in labels:
        label_invalid_rows[label] = int((~np.isfinite(data[label])).sum())

    metadata = {
        "input_rows": input_rows,
        "rows_after_feature_and_movement_validation": int(len(data)),
        "invalid_feature_or_movement_rows": invalid_base_rows,
        "invalid_label_rows_after_base_validation": label_invalid_rows,
        "input_file_rows": file_rows,
    }
    return data, metadata


def safe_correlation(x: pd.Series, y: pd.Series, method: str) -> float:
    if len(x) < 2 or x.nunique(dropna=True) < 2 or y.nunique(dropna=True) < 2:
        return math.nan
    return float(x.corr(y, method=method))


def compute_feature_associations(
    data: pd.DataFrame,
    feature_columns: Sequence[str],
    labels: Sequence[str],
    mi_max_samples: int,
    random_state: int,
) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    discrete_mask = [feature in DISCRETE_FEATURES for feature in feature_columns]

    for label in labels:
        working = data[list(feature_columns) + [label]].dropna()
        working = working.loc[np.isfinite(working[label])]
        if len(working) < 4:
            raise ValueError("label {} has fewer than four valid rows".format(label))

        if len(working) > mi_max_samples:
            mi_data = working.sample(n=mi_max_samples, random_state=random_state)
        else:
            mi_data = working

        mi_values = np.full(len(feature_columns), np.nan, dtype=float)
        nonconstant_indices = [
            index
            for index, feature in enumerate(feature_columns)
            if mi_data[feature].nunique(dropna=True) > 1
        ]
        if mi_data[label].nunique(dropna=True) > 1 and nonconstant_indices:
            selected_features = [feature_columns[index] for index in nonconstant_indices]
            selected_discrete = [discrete_mask[index] for index in nonconstant_indices]
            estimated = mutual_info_regression(
                mi_data[selected_features].to_numpy(dtype=float),
                mi_data[label].to_numpy(dtype=float),
                discrete_features=selected_discrete,
                random_state=random_state,
            )
            for index, value in zip(nonconstant_indices, estimated):
                mi_values[index] = float(value)

        label_rows: List[Dict[str, object]] = []
        for index, feature in enumerate(feature_columns):
            label_rows.append(
                {
                    "label": label,
                    "feature": feature,
                    "feature_kind": (
                        "categorical_numeric_code"
                        if feature in DISCRETE_FEATURES
                        else "continuous"
                    ),
                    "sample_count": int(len(working)),
                    "unique_feature_values": int(working[feature].nunique()),
                    "pearson": safe_correlation(working[feature], working[label], "pearson"),
                    "spearman": safe_correlation(working[feature], working[label], "spearman"),
                    "mutual_information_nats": mi_values[index],
                    "mi_sample_count": int(len(mi_data)),
                }
            )

        finite_mi = [
            float(row["mutual_information_nats"])
            for row in label_rows
            if math.isfinite(float(row["mutual_information_nats"]))
        ]
        max_mi = max(finite_mi, default=0.0)
        for row in label_rows:
            mi_value = float(row["mutual_information_nats"])
            row["mi_relative_to_label_max"] = (
                mi_value / max_mi if max_mi > 0.0 and math.isfinite(mi_value) else math.nan
            )

        ranked = sorted(
            label_rows,
            key=lambda item: (
                -abs(float(item["spearman"]))
                if math.isfinite(float(item["spearman"]))
                else math.inf
            ),
        )
        for rank, row in enumerate(ranked, start=1):
            row["absolute_spearman_rank"] = rank
        rows.extend(label_rows)

    return pd.DataFrame(rows)


def weighted_finite_mean(values: pd.Series, weights: pd.Series) -> float:
    valid = np.isfinite(values.to_numpy(dtype=float)) & np.isfinite(
        weights.to_numpy(dtype=float)
    )
    if not valid.any():
        return math.nan
    return float(np.average(values.to_numpy(dtype=float)[valid], weights=weights.to_numpy(dtype=float)[valid]))


def compute_conditional_correlations(
    data: pd.DataFrame,
    labels: Sequence[str],
    min_group_size: int,
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    controls = ["__movement", "road_length", "turn_type"]
    tested_features = ["has_waiting", "road_flow", "lane_flow"]
    detail_rows: List[Dict[str, object]] = []
    summary_rows: List[Dict[str, object]] = []

    for label in labels:
        working = data[controls + tested_features + [label]].dropna()
        working = working.loc[np.isfinite(working[label])].copy()

        for feature in tested_features:
            group_stats = working.groupby(controls, observed=True, sort=False).agg(
                group_size=(label, "size"),
                feature_unique=(feature, "nunique"),
                label_unique=(label, "nunique"),
            )
            eligible_stats = group_stats.loc[
                (group_stats["group_size"] >= min_group_size)
                & (group_stats["feature_unique"] >= 2)
                & (group_stats["label_unique"] >= 2)
            ]

            if eligible_stats.empty:
                summary_rows.append(
                    {
                        "label": label,
                        "feature": feature,
                        "controls": "movement+road_length+turn_type",
                        "minimum_group_size": min_group_size,
                        "eligible_group_count": 0,
                        "eligible_row_count": 0,
                        "eligible_row_fraction": 0.0,
                        "pooled_within_group_pearson": math.nan,
                        "pooled_within_group_spearman": math.nan,
                        "row_weighted_group_pearson": math.nan,
                        "row_weighted_group_spearman": math.nan,
                        "median_group_pearson": math.nan,
                        "median_group_spearman": math.nan,
                        "q25_group_spearman": math.nan,
                        "q75_group_spearman": math.nan,
                    }
                )
                continue

            eligible_keys = eligible_stats.index
            row_keys = pd.MultiIndex.from_frame(working[controls])
            eligible = working.loc[row_keys.isin(eligible_keys)].copy()
            grouped = eligible.groupby(controls, observed=True, sort=False)

            x_centered = eligible[feature] - grouped[feature].transform("mean")
            y_centered = eligible[label] - grouped[label].transform("mean")
            pooled_pearson = safe_correlation(x_centered, y_centered, "pearson")

            x_rank = grouped[feature].rank(method="average")
            y_rank = grouped[label].rank(method="average")
            rank_frame = eligible[controls].copy()
            rank_frame["__x_rank"] = x_rank
            rank_frame["__y_rank"] = y_rank
            rank_groups = rank_frame.groupby(controls, observed=True, sort=False)
            x_rank_centered = x_rank - rank_groups["__x_rank"].transform("mean")
            y_rank_centered = y_rank - rank_groups["__y_rank"].transform("mean")
            pooled_spearman = safe_correlation(
                x_rank_centered, y_rank_centered, "pearson"
            )

            current_detail_rows: List[Dict[str, object]] = []
            for keys, group in grouped:
                movement, road_length, turn_type = keys
                current_detail_rows.append(
                    {
                        "label": label,
                        "feature": feature,
                        "movement": movement,
                        "road_length": road_length,
                        "turn_type": turn_type,
                        "group_size": int(len(group)),
                        "pearson": safe_correlation(group[feature], group[label], "pearson"),
                        "spearman": safe_correlation(group[feature], group[label], "spearman"),
                    }
                )
            detail_rows.extend(current_detail_rows)
            detail_frame = pd.DataFrame(current_detail_rows)

            summary_rows.append(
                {
                    "label": label,
                    "feature": feature,
                    "controls": "movement+road_length+turn_type",
                    "minimum_group_size": min_group_size,
                    "eligible_group_count": int(len(detail_frame)),
                    "eligible_row_count": int(len(eligible)),
                    "eligible_row_fraction": float(len(eligible) / len(working)),
                    "pooled_within_group_pearson": pooled_pearson,
                    "pooled_within_group_spearman": pooled_spearman,
                    "row_weighted_group_pearson": weighted_finite_mean(
                        detail_frame["pearson"], detail_frame["group_size"]
                    ),
                    "row_weighted_group_spearman": weighted_finite_mean(
                        detail_frame["spearman"], detail_frame["group_size"]
                    ),
                    "median_group_pearson": float(detail_frame["pearson"].median()),
                    "median_group_spearman": float(detail_frame["spearman"].median()),
                    "q25_group_spearman": float(detail_frame["spearman"].quantile(0.25)),
                    "q75_group_spearman": float(detail_frame["spearman"].quantile(0.75)),
                }
            )

    return pd.DataFrame(summary_rows), pd.DataFrame(detail_rows)


def weighted_column_mean(frame: pd.DataFrame, value: str, weight: str) -> float:
    return weighted_finite_mean(frame[value], frame[weight])


def compute_feature_cell_variability(
    data: pd.DataFrame,
    feature_columns: Sequence[str],
    labels: Sequence[str],
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    detail_frames: List[pd.DataFrame] = []
    summaries: List[Dict[str, object]] = []

    for label in labels:
        working = data[list(feature_columns) + [label]].dropna().copy()
        working = working.loc[np.isfinite(working[label])]
        grouped = working.groupby(list(feature_columns), observed=True, sort=False)[label]

        cells = grouped.agg(
            sample_count="size",
            label_mean="mean",
            label_std="std",
            label_min="min",
            label_max="max",
        ).reset_index()
        quantiles = grouped.quantile([0.25, 0.5, 0.75]).unstack().reset_index()
        quantiles = quantiles.rename(
            columns={0.25: "label_q25", 0.5: "label_median", 0.75: "label_q75"}
        )
        cells = cells.merge(quantiles, on=list(feature_columns), how="left")
        cells["label_iqr"] = cells["label_q75"] - cells["label_q25"]
        cells["label_range"] = cells["label_max"] - cells["label_min"]
        cells.insert(0, "label", label)
        detail_frames.append(cells)

        group_mean = grouped.transform("mean")
        group_median = grouped.transform("median")
        label_values = working[label]
        total_sse = float(((label_values - label_values.mean()) ** 2).sum())
        within_sse = float(((label_values - group_mean) ** 2).sum())
        repeated = cells.loc[cells["sample_count"] >= 2]

        summaries.append(
            {
                "label": label,
                "valid_row_count": int(len(working)),
                "exact_feature_cell_count": int(len(cells)),
                "singleton_cell_count": int((cells["sample_count"] == 1).sum()),
                "repeated_cell_count": int(len(repeated)),
                "rows_in_repeated_cells": int(repeated["sample_count"].sum()),
                "repeated_cell_row_fraction": float(
                    repeated["sample_count"].sum() / len(working)
                ),
                "weighted_mean_repeated_cell_std": weighted_column_mean(
                    repeated, "label_std", "sample_count"
                ),
                "median_repeated_cell_std": float(repeated["label_std"].median()),
                "p90_repeated_cell_std": float(repeated["label_std"].quantile(0.90)),
                "weighted_mean_repeated_cell_iqr": weighted_column_mean(
                    repeated, "label_iqr", "sample_count"
                ),
                "median_repeated_cell_iqr": float(repeated["label_iqr"].median()),
                "p90_repeated_cell_iqr": float(repeated["label_iqr"].quantile(0.90)),
                "within_cell_variance_ratio": (
                    within_sse / total_sse if total_sse > 0.0 else math.nan
                ),
                "same_data_cell_mean_mae": float(
                    (label_values - group_mean).abs().mean()
                ),
                "same_data_cell_median_mae": float(
                    (label_values - group_median).abs().mean()
                ),
            }
        )

    detail = pd.concat(detail_frames, ignore_index=True) if detail_frames else pd.DataFrame()
    return pd.DataFrame(summaries), detail


def write_outputs(
    output_dir: Path,
    associations: pd.DataFrame,
    conditional_summary: pd.DataFrame,
    conditional_detail: pd.DataFrame,
    cell_summary: pd.DataFrame,
    cell_detail: pd.DataFrame,
    manifest: Dict[str, object],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = {
        "feature_label_association.csv": associations,
        "conditional_correlation_summary.csv": conditional_summary,
        "conditional_correlation_by_group.csv": conditional_detail,
        "feature_cell_variability_summary.csv": cell_summary,
        "feature_cell_variability.csv": cell_detail,
    }
    for filename, frame in outputs.items():
        frame.to_csv(output_dir / filename, index=False, float_format="%.10g")

    manifest["output_files"] = list(outputs) + ["analysis_manifest.json"]
    with (output_dir / "analysis_manifest.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    files = resolve_input_files(args.csv, args.csv_glob)
    labels, movement_column = inspect_inputs(
        files,
        args.feature_columns,
        args.label_columns,
        args.movement_column,
        args.skip_conditional,
    )
    data, load_metadata = load_inputs(
        files, args.feature_columns, labels, movement_column
    )

    associations = compute_feature_associations(
        data,
        args.feature_columns,
        labels,
        args.mi_max_samples,
        args.random_state,
    )
    if args.skip_conditional:
        conditional_summary = pd.DataFrame(
            columns=[
                "label",
                "feature",
                "controls",
                "minimum_group_size",
                "eligible_group_count",
                "eligible_row_count",
                "eligible_row_fraction",
                "pooled_within_group_pearson",
                "pooled_within_group_spearman",
                "row_weighted_group_pearson",
                "row_weighted_group_spearman",
                "median_group_pearson",
                "median_group_spearman",
                "q25_group_spearman",
                "q75_group_spearman",
            ]
        )
        conditional_detail = pd.DataFrame(
            columns=[
                "label",
                "feature",
                "movement",
                "road_length",
                "turn_type",
                "group_size",
                "pearson",
                "spearman",
            ]
        )
    else:
        conditional_summary, conditional_detail = compute_conditional_correlations(
            data, labels, args.min_conditional_group_size
        )
    cell_summary, cell_detail = compute_feature_cell_variability(
        data, args.feature_columns, labels
    )

    manifest: Dict[str, object] = {
        "analysis": "travel_time_feature_relationships",
        "input_files": [str(path) for path in files],
        "labels": list(labels),
        "features": list(args.feature_columns),
        "discrete_features_for_mutual_information": sorted(DISCRETE_FEATURES),
        "movement_source_column": movement_column,
        "conditional_analysis_status": (
            "skipped_by_explicit_option"
            if args.skip_conditional
            else "completed"
        ),
        "conditional_analysis_skip_reason": (
            "Input legacy CSVs contain no Movement identifier."
            if args.skip_conditional
            else None
        ),
        "conditional_controls": (
            []
            if args.skip_conditional
            else [movement_column, "road_length", "turn_type"]
        ),
        "conditional_tested_features": ["has_waiting", "road_flow", "lane_flow"],
        "minimum_conditional_group_size": args.min_conditional_group_size,
        "mutual_information_max_samples": args.mi_max_samples,
        "random_state": args.random_state,
        **load_metadata,
    }
    output_dir = Path(args.output_dir).expanduser().resolve()
    write_outputs(
        output_dir,
        associations,
        conditional_summary,
        conditional_detail,
        cell_summary,
        cell_detail,
        manifest,
    )

    print("Analyzed {:,} validated rows from {} CSV file(s).".format(len(data), len(files)))
    print("Labels: {}".format(", ".join(labels)))
    if args.skip_conditional:
        print("Conditional correlations: skipped explicitly (no Movement column).")
    print("Results: {}".format(output_dir))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, ValueError) as exc:
        raise SystemExit("error: {}".format(exc)) from exc
