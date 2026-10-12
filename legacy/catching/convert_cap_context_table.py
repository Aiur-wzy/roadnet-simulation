#!/usr/bin/env python3
"""Convert a trusted, strictly validated cap/context RLE to readable TXT/CSV.

No predictor or AutoGluon is loaded. The original RLE and its manifest are read
only; C++ continues to consume the original RLE, not these analysis tables.
"""
import argparse
import json
from pathlib import Path

from roadnet_paths import source_path

from legacy.catching.model_catching_sumo_cap_context import (THRESHOLDS, _unique_json,
                                            scan_table, sha256_file,
                                            write_readable_tables)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--table", required=True, help="Existing CAMS_CAP_CONTEXT_RLE_V1 table")
    parser.add_argument("--output-dir", required=True, help="New directory; no overwrite/resume")
    parser.add_argument("--text-format", choices=("txt", "csv", "both"), default="both")
    parser.add_argument("--expected-threshold", choices=THRESHOLDS)
    parser.add_argument("--dry-run", action="store_true", help="Validate RLE/source manifest without output writes")
    return parser


def convert(args):
    source = Path(args.table).expanduser().resolve(strict=True)
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise ValueError("Output already exists; choose a new directory (no overwrite/resume)")
    header, dense_sha = scan_table(source)
    if args.expected_threshold and header["source_label"] != "travel_time_label_gt_" + args.expected_threshold:
        raise ValueError("Source table threshold differs from requested threshold")
    source_sha = sha256_file(source)
    source_manifest = source.parent / "manifest.json"
    provenance = {"table": str(source), "table_sha256": source_sha,
                  "manifest": None, "manifest_sha256": None,
                  "source_manifest_authenticated": False}
    if source_manifest.exists():
        manifest = json.loads(source_manifest.read_text(encoding="utf-8-sig"), object_pairs_hook=_unique_json)
        if (manifest.get("status") != "complete" or manifest.get("table") != source.name
                or manifest.get("table_sha256") != source_sha
                or manifest.get("dense_sha256") != dense_sha
                or manifest.get("rows") != header["expanded_rows"]
                or manifest.get("plan") != header):
            raise ValueError("Sibling manifest does not authenticate this complete RLE table")
        provenance.update(manifest=str(source_manifest), manifest_sha256=sha256_file(source_manifest),
                          source_manifest_authenticated=True)
    print(f"[PLAN] {header['source_label']} rows={header['expanded_rows']} text={args.text_format}")
    if args.dry_run:
        print("[DRY-RUN] Strict RLE/source-manifest validation passed; no output writes")
        return {"status": "dry_run", "source": provenance, "rows": header["expanded_rows"]}
    output.mkdir(parents=True, exist_ok=False)
    tables = write_readable_tables(source, output, args.text_format, expected_dense_sha=dense_sha)
    if sha256_file(source) != source_sha:
        raise ValueError("Source RLE changed during conversion")
    if provenance["manifest_sha256"] is not None and sha256_file(source_manifest) != provenance["manifest_sha256"]:
        raise ValueError("Source manifest changed during conversion")
    report = {"status": "complete", "schema": "cap-context-readable-conversion-v1",
              "source": provenance, "source_header": header,
              "rows": header["expanded_rows"], "dense_sha256": dense_sha,
              "readable_tables": tables, "cpp_input": False,
              "converter_sha256": sha256_file(__file__),
              "decoder_exporter_sha256": sha256_file(source_path("model_catching_sumo_cap_context.py")),
              "note": "No model inference or fit. Decimal strings round-trip all six float64 columns; downstream application display/import precision is separate."}
    with (output / "conversion_manifest.json").open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(f"[DONE] {output} rows={report['rows']} readable_roundtrip_exact=true")
    return report


def main(argv=None):
    return convert(build_parser().parse_args(argv))


if __name__ == "__main__":
    main()
