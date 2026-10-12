"""Readable exports use mocked predictions or existing RLE, never real training."""
from contextlib import redirect_stdout
import csv
import hashlib
import io
import json
from pathlib import Path
import struct
import unittest
from unittest import mock

import numpy as np

import legacy.catching.convert_cap_context_table as converter
import legacy.catching.model_catching_sumo_cap_context as catch
import tests.python.test_model_catching_sumo_cap_context as fixtures


class ReadableTableTests(unittest.TestCase):
    setUp = fixtures.CapContextCatchTests.setUp
    tearDown = fixtures.CapContextCatchTests.tearDown
    save_contract = fixtures.CapContextCatchTests.save_contract
    args = fixtures.CapContextCatchTests.args
    formula = staticmethod(fixtures.CapContextCatchTests.formula)
    predictor = fixtures.CapContextCatchTests.predictor
    export = fixtures.CapContextCatchTests.export
    mutated = fixtures.CapContextCatchTests.mutated

    def convert(self, source, *extra):
        args = converter.build_parser().parse_args([
            "--table", str(source), "--output-dir", str(self.root / "readable"), *extra])
        with redirect_stdout(io.StringIO()):
            return converter.convert(args)

    def test_default_none_preserves_rle_only_manifest(self):
        args, report = self.export()
        self.assertEqual(args.text_format, "none")
        self.assertNotIn("readable_tables", report)
        self.assertFalse((Path(args.output_dir) / "table.txt").exists())
        self.assertFalse((Path(args.output_dir) / "table.csv").exists())

    def test_each_text_option_and_exact_all_columns(self):
        for fmt, expected_formats in (("txt", {"txt"}), ("csv", {"csv"}), ("both", {"txt", "csv"})):
            with self.subTest(fmt=fmt):
                args = self.args("--text-format", fmt, "--output-dir", str(self.root / fmt))
                calls = []

                def predict(frame):
                    calls.append(len(frame))
                    return self.formula(frame)

                args, report = self.export(predict, args)
                self.assertEqual(sum(calls), report["rows"])
                self.assertEqual(set(report["readable_tables"]), expected_formats)
                path = Path(args.output_dir) / "table.rle"
                header, dense = catch.read_table(path)
                expected = np.array([(*row, value) for row, value in zip(catch.iter_cases(header), dense)], dtype="<f8")
                for kind in expected_formats:
                    readable = Path(args.output_dir) / ("table." + kind)
                    with readable.open(newline="", encoding="utf-8") as stream:
                        rows = list(csv.reader(stream, delimiter="," if kind == "csv" else " "))
                    self.assertEqual(rows[0], catch.FEATURE_COLS + [catch.LABEL_COL])
                    actual = np.array([[float(value) for value in row] for row in rows[1:]], dtype="<f8")
                    self.assertEqual(actual.tobytes(), expected.tobytes())
                    metadata = report["readable_tables"][kind]
                    self.assertEqual(metadata["sha256"], hashlib.sha256(readable.read_bytes()).hexdigest())
                    self.assertEqual(metadata["row_float64_sha256"], hashlib.sha256(expected.tobytes()).hexdigest())
                    self.assertFalse(metadata["cpp_input"])

    def test_stream_scan_equals_dense_without_numpy_allocation(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        header, dense = catch.read_table(path)
        callbacks = []
        with mock.patch.object(catch.np, "empty", side_effect=AssertionError("dense allocation")):
            streamed, digest = catch.scan_table(path, lambda h, group, lo, hi, value: callbacks.append((hi - lo + 1, value)))
        self.assertEqual(header, streamed)
        self.assertEqual(digest, hashlib.sha256(dense.tobytes()).hexdigest())
        rebuilt = b"".join(struct.pack("<d", value) * count for count, value in callbacks)
        self.assertEqual(rebuilt, dense.tobytes())

    def test_converter_no_autogluon_or_dense_and_preserves_sources(self):
        args, original = self.export()
        source = Path(args.output_dir) / "table.rle"
        before = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.parent.iterdir()}
        with mock.patch.dict("sys.modules", {"autogluon": None, "autogluon.tabular": None}), \
                mock.patch.object(catch.np, "empty", side_effect=AssertionError("dense allocation")):
            report = self.convert(source)
        self.assertEqual(report["status"], "complete")
        self.assertEqual(set(report["readable_tables"]), {"txt", "csv"})
        self.assertTrue(report["source"]["source_manifest_authenticated"])
        self.assertEqual(report["source_header"]["exporter_sha256"], original["plan"]["exporter_sha256"])
        self.assertEqual(before, {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source.parent.iterdir()})

    def test_standalone_rle_without_manifest_supported(self):
        args, _ = self.export()
        standalone = self.root / "standalone.rle"
        standalone.write_bytes((Path(args.output_dir) / "table.rle").read_bytes())
        report = self.convert(standalone, "--text-format", "txt")
        self.assertFalse(report["source"]["source_manifest_authenticated"])
        self.assertIsNone(report["source"]["manifest"])
        self.assertEqual(set(report["readable_tables"]), {"txt"})

    def test_converter_dryrun_no_output_and_threshold_check(self):
        args, _ = self.export()
        source = Path(args.output_dir) / "table.rle"
        with mock.patch.dict("sys.modules", {"autogluon": None, "autogluon.tabular": None}):
            report = self.convert(source, "--dry-run", "--expected-threshold", "0p1")
        self.assertEqual(report["status"], "dry_run")
        self.assertFalse((self.root / "readable").exists())
        with self.assertRaisesRegex(ValueError, "threshold"):
            self.convert(source, "--expected-threshold", "3")

    def test_no_overwrite_existing_converter_directory(self):
        args, _ = self.export()
        target = self.root / "readable"
        target.mkdir()
        sentinel = target / "table.csv"
        sentinel.write_bytes(b"KEEP")
        with self.assertRaisesRegex(ValueError, "already exists"):
            self.convert(Path(args.output_dir) / "table.rle")
        self.assertEqual(sentinel.read_bytes(), b"KEEP")

    def test_helper_no_overwrite_final_or_partial(self):
        args, _ = self.export()
        source = Path(args.output_dir) / "table.rle"
        for name in ("table.csv", "table.csv.partial"):
            output = self.root / name.replace(".", "_")
            output.mkdir()
            sentinel = output / name
            sentinel.write_bytes(b"KEEP")
            with self.assertRaisesRegex(ValueError, "already exists"):
                catch.write_readable_tables(source, output, "both")
            self.assertEqual(sentinel.read_bytes(), b"KEEP")
            self.assertFalse((output / "table.txt.partial").exists())

    def test_converter_bad_manifest_rejected_before_mkdir(self):
        args, _ = self.export()
        source = Path(args.output_dir) / "table.rle"
        manifest = source.parent / "manifest.json"
        data = json.loads(manifest.read_text())
        data["table_sha256"] = "0" * 64
        manifest.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "manifest"):
            self.convert(source)
        self.assertFalse((self.root / "readable").exists())

    def test_text_failure_never_completes_export_manifest(self):
        with mock.patch.object(catch, "validate_readable_table", side_effect=OSError("injected read failure")):
            with self.assertRaises(OSError):
                self.export(args=self.args("--text-format", "both"))
        output = self.root / "export"
        self.assertTrue((output / "table.rle").exists())
        self.assertFalse((output / "manifest.json").exists())
        self.assertFalse((output / "table.txt").exists())
        self.assertTrue((output / "table.txt.partial").exists())

    def test_converter_failure_never_completes_manifest(self):
        args, _ = self.export()
        with mock.patch.object(catch, "validate_readable_table", side_effect=ValueError("injected mismatch")):
            with self.assertRaises(ValueError):
                self.convert(Path(args.output_dir) / "table.rle")
        self.assertFalse((self.root / "readable" / "conversion_manifest.json").exists())
        self.assertFalse((self.root / "readable" / "table.csv").exists())

    def test_text_tamper_truncation_extra_and_precision_loss_rejected(self):
        args, _ = self.export(args=self.args("--text-format", "txt"))
        path = Path(args.output_dir) / "table.txt"
        source = Path(args.output_dir) / "table.rle"
        lines = path.read_text().splitlines(keepends=True)
        bad = self.root / "bad.txt"
        tokens = lines[1].split()
        tokens[-1] = str(np.nextafter(float(tokens[-1]), np.inf))
        mutations = ["wrong header\n" + "".join(lines[1:]), "".join(lines[:-1]),
                     "".join(lines) + lines[-1], lines[0] + " ".join(tokens) + "\n" + "".join(lines[2:])]
        for text in mutations:
            with self.subTest(text=text[:60]):
                bad.write_text(text, encoding="utf-8")
                with self.assertRaises(ValueError):
                    catch.validate_readable_table(bad, source, "txt")

    def test_adjacent_float64_and_extreme_values_exact(self):
        values = np.array([1.0, np.nextafter(1.0, np.inf), np.pi, np.finfo(np.float64).max])
        def predict(frame):
            return values[frame["platoon_context"].astype(int).to_numpy()]
        args, report = self.export(predict, self.args("--text-format", "both"))
        self.assertEqual(report["predictions_floored_to_one_second"], 0)
        for fmt in ("txt", "csv"):
            self.assertTrue(report["readable_tables"][fmt]["all_columns_float64_roundtrip_exact"])

    def test_zero_flow_grid_and_nonaligned_cap_decimal_exact(self):
        args, report = self.export(args=self.args("--text-format", "both", "--max-road-flow", "0",
                                                   "--max-lane-flow", "0", "--cap-step", ".3"))
        self.assertEqual(report["rows"], len(list(catch.iter_cases(report["plan"]))))
        for info in report["readable_tables"].values():
            self.assertTrue(info["all_columns_float64_roundtrip_exact"])

    def test_streaming_rejects_corrupt_rle_before_output(self):
        args, _ = self.export()
        source = Path(args.output_dir) / "table.rle"
        edits = [lambda h, b: struct.pack_into("<I", b, 0, 1),
                 lambda h, b: struct.pack_into("<I", b, h["group_count"] * 4, h["run_count"] - 1),
                 lambda h, b: struct.pack_into("<H", b, 4 * (h["group_count"] + 1), 1),
                 lambda h, b: struct.pack_into("<d", b, 4 * (h["group_count"] + 1) + 4, float("nan"))]
        for edit in edits:
            with self.subTest(edit=edit):
                changed = self.mutated(source, edit_payload=edit)
                with self.assertRaises(ValueError):
                    self.convert(changed)
                self.assertFalse((self.root / "readable").exists())

    def test_all_text_outputs_share_same_key_value_digest(self):
        args, report = self.export(args=self.args("--text-format", "both"))
        self.assertEqual(report["readable_tables"]["txt"]["row_float64_sha256"],
                         report["readable_tables"]["csv"]["row_float64_sha256"])

    def test_strict_stream_duplicate_json_eof_unsigned_and_canonical_runs(self):
        args, _ = self.export()
        source = Path(args.output_dir) / "table.rle"
        magic, header_line, payload = source.read_bytes().split(b"\n", 2)
        invalid = self.root / "invalid.rle"
        header = json.loads(header_line)
        for contents in (magic + b"\n" + b'{"format":"duplicate",' + header_line[1:] + b"\n" + payload,
                         source.read_bytes() + b"\n", source.read_bytes()[:-1]):
            with self.subTest(contents=contents[:60]):
                invalid.write_bytes(contents)
                with self.assertRaises(ValueError):
                    catch.scan_table(invalid)
        def noncanonical(h, b):
            start = 4 * (h["group_count"] + 1)
            first = struct.unpack_from("<d", b, start + 4)[0]
            struct.pack_into("<d", b, start + catch.RUN.size + 4, first)
        edits = [lambda h, b: struct.pack_into("<I", b, 4, 2**32 - 1),
                 lambda h, b: struct.pack_into("<H", b, 4 * (h["group_count"] + 1) + 2, 65535),
                 noncanonical]
        for edit in edits:
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                catch.scan_table(self.mutated(source, edit_payload=edit))

    def test_complete_file_recheck_detects_changes(self):
        args, _ = self.export()
        path = Path(args.output_dir) / "table.rle"
        with mock.patch.object(catch, "sha256_file", return_value="0" * 64):
            with self.assertRaisesRegex(ValueError, "changed during strict"):
                catch.scan_table(path)

    def test_bom_sibling_manifest_is_accepted(self):
        args, _ = self.export()
        source = Path(args.output_dir) / "table.rle"
        manifest = source.parent / "manifest.json"
        text = manifest.read_text(encoding="utf-8")
        manifest.write_text(text, encoding="utf-8-sig")
        self.assertTrue(self.convert(source, "--text-format", "csv")["source"]["source_manifest_authenticated"])


if __name__ == "__main__":
    unittest.main()
