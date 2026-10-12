#!/usr/bin/env python3
"""Create a SUMO network copy with deterministic random tlLogic offsets."""

from __future__ import annotations

import argparse
import csv
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

CSV_COLUMNS = [
    "tlID",
    "programID",
    "type",
    "phaseCount",
    "cycleLength",
    "oldOffset",
    "newOffset",
    "status",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Randomize traffic-light offsets in a copy of a SUMO .net.xml file."
    )
    parser.add_argument("--input-net", required=True, help="Source SUMO network XML path")
    parser.add_argument("--output-net", required=True, help="New SUMO network XML output path")
    parser.add_argument("--seed", required=True, type=int, help="Deterministic random seed")
    parser.add_argument("--offset-csv", required=True, help="CSV path for generated offsets")
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow overwriting output files",
    )
    parser.add_argument(
        "--integer-offsets",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use integer-second offsets by default; use --no-integer-offsets for floats",
    )
    parser.add_argument(
        "--min-offset",
        type=float,
        default=0.0,
        help="Minimum generated offset, inclusive (default: 0)",
    )
    parser.add_argument(
        "--avoid-offsets-csv",
        help="Optional previous offset CSV whose tlID/programID newOffset values should be avoided when possible",
    )
    return parser.parse_args()


def canonical_path(path: Path) -> Path:
    """Return an absolute path without requiring the path to exist."""
    return path.expanduser().resolve(strict=False)


def format_number(value: float) -> str:
    """Format numeric XML/CSV values compactly while preserving fractional values."""
    if value.is_integer():
        return str(int(value))
    return f"{value:.6f}".rstrip("0").rstrip(".")


def phase_cycle_length(phases: Iterable[ET.Element]) -> float:
    total = 0.0
    for phase in phases:
        total += float(phase.get("duration", "0"))
    return total


def read_offsets_to_avoid(path: Path | None) -> dict[tuple[str, str], str]:
    if path is None:
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return {
            (row.get("tlID", ""), row.get("programID", "")): row.get("newOffset", "")
            for row in reader
        }


def random_offset(
    rng: random.Random,
    cycle_length: float,
    min_offset: float,
    integer_offsets: bool,
    avoid: str | None,
) -> str:
    if integer_offsets:
        low = int(min_offset)
        high = int(cycle_length)
        if low >= high:
            low = 0
        candidates = list(range(low, high))
        if avoid is not None and len(candidates) > 1:
            candidates = [candidate for candidate in candidates if str(candidate) != avoid]
        return str(rng.choice(candidates))

    for _ in range(10):
        value = rng.uniform(min_offset, cycle_length)
        formatted = format_number(value)
        if formatted != avoid:
            return formatted
    return format_number(rng.uniform(min_offset, cycle_length))


def main() -> int:
    args = parse_args()
    input_net = Path(args.input_net)
    output_net = Path(args.output_net)
    offset_csv = Path(args.offset_csv)
    avoid_csv = Path(args.avoid_offsets_csv) if args.avoid_offsets_csv else None

    if args.min_offset < 0:
        print("error: --min-offset must be >= 0", file=sys.stderr)
        return 2

    if canonical_path(input_net) == canonical_path(output_net):
        print("error: --output-net must not be the same path as --input-net", file=sys.stderr)
        return 2

    if output_net.exists() and not args.force:
        print(f"error: output network already exists: {output_net}", file=sys.stderr)
        return 2
    if offset_csv.exists() and not args.force:
        print(f"error: offset CSV already exists: {offset_csv}", file=sys.stderr)
        return 2

    tree = ET.parse(input_net)
    root = tree.getroot()
    rng = random.Random(args.seed)
    offsets_to_avoid = read_offsets_to_avoid(avoid_csv)

    rows: list[dict[str, str]] = []
    randomized_count = 0
    skipped_count = 0

    tls = root.findall("tlLogic")
    for tl in tls:
        phases = tl.findall("phase")
        cycle_length = phase_cycle_length(phases)
        old_offset = tl.get("offset", "")
        key = (tl.get("id", ""), tl.get("programID", ""))

        if cycle_length > 0:
            new_offset = random_offset(
                rng,
                cycle_length,
                args.min_offset,
                args.integer_offsets,
                offsets_to_avoid.get(key),
            )
            status = "randomized"
            randomized_count += 1
        else:
            new_offset = old_offset if old_offset != "" else "0"
            status = "skipped_invalid_cycle"
            skipped_count += 1

        tl.set("offset", new_offset)
        rows.append(
            {
                "tlID": tl.get("id", ""),
                "programID": tl.get("programID", ""),
                "type": tl.get("type", ""),
                "phaseCount": str(len(phases)),
                "cycleLength": format_number(cycle_length),
                "oldOffset": old_offset,
                "newOffset": new_offset,
                "status": status,
            }
        )

    output_net.parent.mkdir(parents=True, exist_ok=True)
    offset_csv.parent.mkdir(parents=True, exist_ok=True)

    tree.write(output_net, encoding="UTF-8", xml_declaration=True)
    with offset_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    print(f"input path: {input_net}")
    print(f"output path: {output_net}")
    print(f"seed: {args.seed}")
    print(f"tlLogic count: {len(tls)}")
    print(f"randomized count: {randomized_count}")
    print(f"skipped count: {skipped_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
