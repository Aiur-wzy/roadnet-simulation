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
        "--integer-offsets",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use integer-second offsets by default; use --no-integer-offsets for floats",
    )
    parser.add_argument("--force", action="store_true", help="Allow overwriting output files")
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


def main() -> int:
    args = parse_args()
    input_net = Path(args.input_net)
    output_net = Path(args.output_net)
    offset_csv = Path(args.offset_csv)

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

    rows: list[dict[str, str]] = []
    randomized_count = 0
    skipped_count = 0

    tls = root.findall("tlLogic")
    for tl in tls:
        phases = tl.findall("phase")
        cycle_length = phase_cycle_length(phases)
        old_offset = tl.get("offset", "")

        if cycle_length > 0:
            if args.integer_offsets:
                new_offset_value = rng.randrange(int(cycle_length))
                new_offset = str(new_offset_value)
            else:
                new_offset = format_number(rng.uniform(0, cycle_length))
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
