#!/usr/bin/env python3
"""Summarize SUMO evaluation CSV outputs without third-party dependencies."""

from __future__ import annotations

import argparse
import collections
import csv
import math
from pathlib import Path


def as_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def print_threshold_counts(rows: list[dict[str, str]]) -> None:
    checks = [
        ("absDurationError", 300),
        ("absDurationError", 600),
        ("absDurationError", 1000),
        ("relativeDurationError", 1.0),
        ("relativeDurationError", 3.0),
        ("relativeDurationError", 5.0),
        ("predDuration", 1000),
        ("predDuration", 2000),
        ("truthDuration", 1000),
        ("truthDuration", 2000),
    ]
    print("[vehicle-threshold-counts]")
    for column, threshold in checks:
        count = sum(as_float(row[column]) > threshold for row in rows)
        print(f"{column}>{threshold}={count}")
    print()


def print_top(rows: list[dict[str, str]], title: str, key: str, limit: int) -> None:
    columns = [
        "vehicleID",
        "predDepart",
        "truthDepart",
        "predArrival",
        "truthArrival",
        "predDuration",
        "truthDuration",
        "durationErrorSigned",
        "absDurationError",
        "relativeDurationError",
        "truthWaitingTime",
        "truthTimeLoss",
        "truthRouteLength",
        "numRoads",
        "numMovements",
    ]
    print(f"[{title}]")
    print(",".join(columns))
    for row in sorted(rows, key=lambda item: as_float(item[key]), reverse=True)[:limit]:
        print(",".join(row.get(column, "") for column in columns))
    print()


def print_movement_summary(directory: Path) -> None:
    timeline = directory / "eval_extreme_vehicle_movement_timeline.csv"
    summary = directory / "eval_extreme_movement_time_summary.csv"
    if not timeline.exists() or not summary.exists():
        return

    timeline_rows = read_csv(timeline)
    print("[movement-wait-threshold-counts]")
    if timeline_rows:
        print(f"maxMovementWaitingTime={max(as_float(row['movementWaitingTime']) for row in timeline_rows)}")
    for threshold in [300, 500, 1000, 2000]:
        count = sum(as_float(row["movementWaitingTime"]) > threshold for row in timeline_rows)
        print(f"movementWaitingTime>{threshold}={count}")
    print()

    summary_rows = read_csv(summary)
    print("[top-movements-by-max-wait]")
    for row in sorted(summary_rows, key=lambda item: as_float(item["maxMovementWaitingTime"]), reverse=True)[:20]:
        print(
            row["movementID"],
            row["movementEdge"],
            "count", row["count"],
            "max", row["maxMovementWaitingTime"],
            "p95", row["p95MovementWaitingTime"],
            "mean", row["meanMovementWaitingTime"],
        )
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="Evaluation output directory")
    parser.add_argument("--limit", type=int, default=30, help="Rows to show for top vehicle tables")
    args = parser.parse_args()

    rows = read_csv(args.directory / "sumo_eval.csv")
    print(f"vehicles={len(rows)}")
    print_threshold_counts(rows)
    print_top(rows, "top-abs-duration-error", "absDurationError", args.limit)
    print_top(rows, "top-relative-duration-error", "relativeDurationError", args.limit)

    slow = [row for row in rows if as_float(row["durationErrorSigned"]) > 0]
    fast = [row for row in rows if as_float(row["durationErrorSigned"]) < 0]
    print_top(slow, "top-too-slow", "durationErrorSigned", args.limit)
    print_top(fast, "top-too-fast-absolute", "absDurationError", args.limit)
    print_movement_summary(args.directory)


if __name__ == "__main__":
    main()
