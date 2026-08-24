#!/usr/bin/env python3
"""Collect one TraCI dataset that can reconstruct several running-time labels.

This script deliberately leaves ``TraCI_Python_Adjusted.py`` unchanged.  It
reuses the original simulation/event implementation and augments both the
lightweight and full training CSVs with:

* time accumulated in disjoint speed bins; and
* one ``travel_time_label_gt_*`` column per candidate speed threshold.

For a threshold ``c``, the reconstructed label is the accumulated pre-stopline
time for which ``speed > c``.  The default thresholds are
``0.1,0.5,1.0,1.5,2.0,3.0`` m/s.

Example (Linux or Windows shell syntax adjusted as needed)::

    python TraCI_Python_Threshold_Sweep.py \
      --sumo-config scenario.sumocfg \
      --net-file data/net/test_new.net.xml \
      --outputs legacy,training \
      --legacy-edge-output threshold_sweep.csv \
      --training-output threshold_sweep_full.csv

The original ``--stop-speed-threshold`` and ``--low-speed-threshold`` still
control the legacy columns.  They do not affect the new speed-bin columns.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import sys
import tempfile
from typing import Iterable, List, Optional, Sequence, Tuple

import TraCI_Python_Adjusted as base


DEFAULT_LABEL_SPEED_THRESHOLDS = (0.1, 0.5, 1.0, 1.5, 2.0, 3.0)
_label_speed_thresholds: Tuple[float, ...] = DEFAULT_LABEL_SPEED_THRESHOLDS

_original_accumulate_waiting_time = base.accumulate_waiting_time
_original_write_lightweight_training_rows = base.write_lightweight_training_rows
_original_write_training_rows = base.write_training_rows


def parse_label_speed_thresholds(value: str) -> Tuple[float, ...]:
    """Parse a sorted, unique list of non-negative speed thresholds."""

    parts = [part.strip() for part in str(value).split(",") if part.strip()]
    if not parts:
        raise argparse.ArgumentTypeError("at least one label speed threshold is required")
    try:
        thresholds = sorted(set(float(part) for part in parts))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "label speed thresholds must be comma-separated numbers"
        ) from exc
    if any(not math.isfinite(value) or value < 0.0 for value in thresholds):
        raise argparse.ArgumentTypeError(
            "label speed thresholds must be finite and non-negative"
        )
    return tuple(thresholds)


def threshold_token(value: float) -> str:
    """Return a stable CSV-safe token such as ``0p5`` or ``3``."""

    text = format(float(value), ".12g")
    return text.replace("-", "m").replace(".", "p").replace("+", "")


def speed_bin_columns(thresholds: Sequence[float]) -> List[str]:
    """Build names for ``<= first``, intermediate, and ``> last`` bins."""

    columns = ["speed_time_le_{}".format(threshold_token(thresholds[0]))]
    for lower, upper in zip(thresholds, thresholds[1:]):
        columns.append(
            "speed_time_gt_{}_le_{}".format(
                threshold_token(lower), threshold_token(upper)
            )
        )
    columns.append("speed_time_gt_{}".format(threshold_token(thresholds[-1])))
    return columns


def candidate_label_columns(thresholds: Sequence[float]) -> List[str]:
    return [
        "travel_time_label_gt_{}".format(threshold_token(value))
        for value in thresholds
    ]


def _event_speed_bins(event) -> List[float]:
    bins = getattr(event, "threshold_sweep_speed_bins", None)
    expected = len(_label_speed_thresholds) + 1
    if bins is None:
        bins = [0.0] * expected
        event.threshold_sweep_speed_bins = bins
    elif len(bins) != expected:
        raise RuntimeError(
            "event speed-bin width does not match configured thresholds: "
            "{} != {}".format(len(bins), expected)
        )
    return bins


def accumulate_waiting_time(event, snapshot, config, current_time: float) -> None:
    """Run the original accounting and independently accumulate a speed bin."""

    should_accumulate = event.pass_stopline_time is None
    previous_time = (
        event.last_update_time
        if event.last_update_time is not None
        else current_time
    )
    delta = max(0.0, current_time - previous_time) if should_accumulate else 0.0

    _original_accumulate_waiting_time(event, snapshot, config, current_time)

    if delta <= 0.0:
        return
    bin_index = bisect.bisect_left(_label_speed_thresholds, float(snapshot.speed))
    _event_speed_bins(event)[bin_index] += delta


def _valid_training_events(completed_events: Iterable) -> Iterable:
    """Mirror the original writer's row filter and ordering."""

    for event in completed_events:
        travel_time_label = event.driving_time + event.low_speed_time
        pass_stopline_time = (
            event.pass_stopline_time
            if event.pass_stopline_time is not None
            else event.downstream_enter_time
        )
        if (
            travel_time_label <= 0
            or not event.road_length
            or event.road_length <= 0
            or not event.from_edge
            or pass_stopline_time is None
        ):
            continue
        yield event


def _event_extra_values(event) -> dict:
    bins = list(_event_speed_bins(event))
    values = {
        column: round(value, 4)
        for column, value in zip(speed_bin_columns(_label_speed_thresholds), bins)
    }

    # Bin i contains speeds <= threshold i (and > threshold i-1), so the
    # label for threshold i is the sum of all bins strictly above bin i.
    label_values = [sum(bins[index + 1 :]) for index in range(len(_label_speed_thresholds))]
    values.update(
        {
            column: round(value, 4)
            for column, value in zip(
                candidate_label_columns(_label_speed_thresholds), label_values
            )
        }
    )

    speed_bin_time_total = sum(bins)
    original_accounted_time = (
        event.driving_time
        + event.low_speed_time
        + event.red_wait_time
        + event.green_queue_wait_time
        + event.downstream_block_wait_time
    )
    values["speed_bin_time_total"] = round(speed_bin_time_total, 4)
    values["speed_bin_conservation_residual"] = round(
        speed_bin_time_total - original_accounted_time, 6
    )
    return values


def _augment_training_csv(output_path: str, completed_events: Sequence) -> None:
    """Append threshold-sweep columns to a CSV written by the original code."""

    extra_columns = (
        speed_bin_columns(_label_speed_thresholds)
        + candidate_label_columns(_label_speed_thresholds)
        + ["speed_bin_time_total", "speed_bin_conservation_residual"]
    )
    source_path = os.path.abspath(output_path)
    output_dir = os.path.dirname(source_path) or os.curdir
    os.makedirs(output_dir, exist_ok=True)
    temp_handle = tempfile.NamedTemporaryFile(
        mode="w",
        newline="",
        encoding="utf-8",
        prefix=os.path.basename(source_path) + ".",
        suffix=".threshold-sweep.tmp",
        dir=output_dir,
        delete=False,
    )
    temp_path = temp_handle.name

    try:
        events = iter(_valid_training_events(completed_events))
        with open(source_path, mode="r", newline="", encoding="utf-8") as source, temp_handle:
            reader = csv.DictReader(source)
            original_columns = list(reader.fieldnames or [])
            duplicates = sorted(set(original_columns).intersection(extra_columns))
            if duplicates:
                raise RuntimeError(
                    "threshold-sweep columns already exist in output: {}".format(duplicates)
                )
            writer = csv.DictWriter(
                temp_handle,
                fieldnames=original_columns + extra_columns,
            )
            writer.writeheader()
            row_count = 0
            for row in reader:
                try:
                    event = next(events)
                except StopIteration as exc:
                    raise RuntimeError(
                        "original CSV has more rows than valid completed events"
                    ) from exc
                row.update(_event_extra_values(event))
                writer.writerow(row)
                row_count += 1

            try:
                next(events)
            except StopIteration:
                pass
            else:
                raise RuntimeError(
                    "valid completed events remain after the original CSV ended"
                )
        os.replace(temp_path, source_path)
        print("threshold-sweep rows augmented: {}".format(row_count))
    except Exception:
        try:
            temp_handle.close()
        finally:
            if os.path.exists(temp_path):
                os.remove(temp_path)
        raise


def write_lightweight_training_rows(output_path: str, completed_events: Sequence) -> int:
    row_count = _original_write_lightweight_training_rows(output_path, completed_events)
    _augment_training_csv(output_path, completed_events)
    return row_count


def write_training_rows(output_path: str, completed_events: Sequence) -> None:
    _original_write_training_rows(output_path, completed_events)
    _augment_training_csv(output_path, completed_events)


def _extract_custom_args(argv: Sequence[str]):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--label-speed-thresholds",
        default=os.environ.get(
            "LABEL_SPEED_THRESHOLDS",
            ",".join(
                format(value, ".12g")
                for value in DEFAULT_LABEL_SPEED_THRESHOLDS
            ),
        ),
        help=(
            "Comma-separated speed cutoffs used to create disjoint time bins and "
            "travel_time_label_gt_* columns. Can also be set with the "
            "LABEL_SPEED_THRESHOLDS environment variable. "
            "Default: 0.1,0.5,1.0,1.5,2.0,3.0"
        ),
    )
    custom_args, remaining = parser.parse_known_args(list(argv))
    thresholds = parse_label_speed_thresholds(custom_args.label_speed_thresholds)
    return parser, thresholds, remaining


def main(argv: Optional[Sequence[str]] = None) -> None:
    global _label_speed_thresholds

    raw_argv = list(sys.argv[1:] if argv is None else argv)
    custom_parser, thresholds, remaining = _extract_custom_args(raw_argv)
    _label_speed_thresholds = thresholds

    if "-h" in raw_argv or "--help" in raw_argv:
        print("Threshold-sweep extension options:")
        custom_parser.print_help()
        print("Original TraCI options:")

    original_argv = sys.argv
    sys.argv = [original_argv[0]] + remaining
    base.accumulate_waiting_time = accumulate_waiting_time
    base.write_lightweight_training_rows = write_lightweight_training_rows
    base.write_training_rows = write_training_rows
    try:
        print(
            "label_speed_thresholds={} m/s".format(
                ",".join(format(value, ".12g") for value in thresholds)
            )
        )
        print(
            "threshold-sweep label columns={}".format(
                ",".join(candidate_label_columns(thresholds))
            )
        )
        base.main()
    finally:
        base.accumulate_waiting_time = _original_accumulate_waiting_time
        base.write_lightweight_training_rows = _original_write_lightweight_training_rows
        base.write_training_rows = _original_write_training_rows
        sys.argv = original_argv


if __name__ == "__main__":
    main()
