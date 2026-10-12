"""Complete grid, output parity and crash recovery; no real model or traffic run."""
import argparse
from contextlib import closing
import csv
import hashlib
import itertools
import json
import os
from pathlib import Path
import random
import sqlite3
import subprocess
import tempfile
import unittest
from unittest import mock

from catching import catch_road_time as core
from catching import catch_road_time_full as full
from tests.python.test_native_catching_v2 import FakePredictor, NETWORK, fake_model


class CountingPredictor(FakePredictor):
    def __init__(self, fail_call=None):
        self.calls, self.rows, self.fail_call = 0, 0, fail_call

    def predict(self, frame):
        self.calls += 1
        if self.calls == self.fail_call:
            raise RuntimeError("injected inference interruption")
        self.rows += len(frame)
        super().predict(frame)  # Check shared column and categorical contract.
        return [float(r.road_length) / float(r.free_speed_cap) + r.road_flow * .1 + r.lane_flow * .01
                + int(r.platoon_context) * .03 + (r.entry_signal_type == "R") * .04
                + r.entry_signal_remaining_seconds * .005 + (r.next_signal_type == "R") * .006
                + r.next_signal_duration_seconds * .007 for r in frame.itertuples()]


class FullGridTests(unittest.TestCase):
    def test_arithmetic_grid_matches_brute_force(self):
        for maximum in (0, 1, 7, 22):
            for step in (1, 3, 5, 99):
                for dense in (0, 2, 10, 30):
                    for lane_max in {0, maximum // 2, maximum}:
                        policy = core.flow_policy(step, dense)
                        plan = {"road_flow_max": maximum, "lane_flow_max": lane_max,
                                "road_flow_quantization": policy, "group_count": 3 * (lane_max + 1)}
                        expected = [(group, road) for group in range(plan["group_count"])
                                    for road in core.flow_nodes(group % (lane_max + 1), maximum, policy)]
                        grid = full.FullGrid(plan)
                        self.assertEqual(grid.total, len(expected))
                        self.assertEqual(list(grid.keys()), expected)
                        for start in {0, 1, len(expected) // 2, len(expected) - 1, len(expected)}:
                            self.assertEqual(list(grid.keys(start, min(start + 11, grid.total))), expected[start:start + 11])

    def test_piecewise_and_signed_integer_domain(self):
        policy = core.flow_policy(segments=["10:1", "20:5", "31:10"], maximum=31)
        plan = {"road_flow_max": 31, "lane_flow_max": 30, "road_flow_quantization": policy, "group_count": 62}
        grid = full.FullGrid(plan)
        self.assertEqual(list(grid.keys()), [(g, r) for g in range(62) for r in core.flow_nodes(g % 31, 31, policy)])
        maximum = 2**31 - 1
        huge = full.FullGrid({"road_flow_max": maximum, "lane_flow_max": maximum,
                             "road_flow_quantization": core.flow_policy(), "group_count": maximum + 1})
        self.assertEqual(huge.total, (maximum + 1) * (maximum + 2) // 2)
        self.assertEqual(list(huge.keys(huge.total - 3)), [(maximum - 1, maximum - 1),
                                                        (maximum - 1, maximum), (maximum, maximum)])


class FullCatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.network = self.root / "network.net.xml"
        self.network.write_text(NETWORK, encoding="utf-8")
        model = fake_model(self.root)
        self.args = full.build_parser().parse_args([
            "--model-dir", str(model), "--net-file", str(self.network),
            "--output-dir", str(self.root / "full"), "--signal-remaining-step", "5",
            "--cap-step", "1", "--cap-min", "2", "--cap-max", "2",
            "--max-road-flow", "2", "--max-lane-flow", "1", "--shard-rows", "137",
            "--chunk-size", "47", "--min-free-disk-gib", "0"])

    def run_full(self, predictor=None):
        predictor = predictor or CountingPredictor()
        return full.run(self.args, predictor_loader=lambda _: predictor)

    def records(self):
        records = {}
        for path in sorted((self.root / "full" / "shards").glob("*.demand")):
            _, values = core.read_cache(path)
            self.assertFalse(records.keys() & values.keys())
            records.update(values)
        return records

    def test_full_output_matches_sparse_prediction_at_every_node(self):
        self.args.merge_output = True
        self.args.text_format = "both"
        predictor = CountingPredictor()
        result = self.run_full(predictor)
        self.assertEqual(result["status"], "COMPLETE")
        self.assertFalse(result["formal_mae_available"])
        plan = core.read_json(self.root / "full" / "plan.json")
        grid = full.FullGrid(plan)
        self.assertEqual(predictor.rows, grid.total)
        expected = core.predict_keys(CountingPredictor(), plan, list(grid.keys()), 100)
        self.assertEqual(self.records(), expected)
        _, merged = core.read_cache(self.root / "full" / "table.demand")
        self.assertEqual(merged, expected)
        for ext, delimiter in (("csv", ","), ("txt", "\t")):
            rows = []
            for path in sorted((self.root / "full" / "shards").glob("*." + ext)):
                with path.open(newline="", encoding="utf-8") as stream:
                    rows.extend(csv.DictReader(stream, delimiter=delimiter))
            self.assertEqual(len(rows), grid.total)
            actual = {(int(r["group_index"]), int(r["road_flow"])): float(r[core.MODEL_LABEL]) for r in rows}
            self.assertEqual(actual, expected)
        self.args.resume = True
        with mock.patch.object(core, "load_predictor", side_effect=AssertionError("completed run loaded model")):
            self.assertEqual(full.run(self.args), result)

    def test_dry_run_has_no_model_or_files_or_simulation(self):
        self.args.dry_run = True
        with mock.patch.object(core, "load_predictor", side_effect=AssertionError("model loaded")), \
             mock.patch.object(core, "run_process", side_effect=AssertionError("simulator started")):
            result = full.run(self.args)
        self.assertFalse((self.root / "full").exists())
        self.assertFalse(result["model_loaded"])
        self.assertFalse(result["simulator_used"])
        self.assertEqual(result["size_estimate"]["full_grid_keys"], 420)
        self.assertFalse(hasattr(self.args, "max_rounds"))

    def test_declared_grid_quantization_not_route_filter(self):
        self.args.max_road_flow = 22
        self.args.max_lane_flow = 3
        self.args.road_flow_step = 5
        self.args.dry_run = True
        preview = self.run_full()
        plan = preview["plan"]
        grid, codec = full.FullGrid(plan), core.KeyCodec(plan)
        keys = set(grid.keys())
        for group in range(plan["group_count"]):
            lane = group % 4
            for road in range(lane, 23):
                representative = core.representative_road_flow(road, lane, 22, plan["road_flow_quantization"])
                self.assertIn((group, representative), keys)
                self.assertEqual(codec.encode(codec.decode((group, representative))), (group, representative))

    def test_resume_reuses_committed_shards_after_inference_failure(self):
        # Three calls finish 137 rows; the fifth call interrupts the next shard.
        with self.assertRaisesRegex(RuntimeError, "injected"):
            self.run_full(CountingPredictor(fail_call=5))
        progress = core.read_json(self.root / "full" / "progress.json")
        self.assertEqual(progress["committed_rows"], 137)
        first = self.root / "full" / "shards" / "part_000000000.demand"
        signature = core.signature(first)
        self.args.resume = True
        predictor = CountingPredictor()
        result = self.run_full(predictor)
        self.assertEqual(predictor.rows, result["entry_count"] - 137)
        self.assertEqual(core.signature(first), signature)
        self.assertEqual(len(self.records()), 420)

    def test_resume_adopts_complete_uncommitted_binary(self):
        self.run_full()
        output = self.root / "full"
        (output / "completion.json").unlink()
        with closing(sqlite3.connect(output / "checkpoint.sqlite3")) as connection:
            with connection:
                connection.execute("DELETE FROM shards WHERE start=(SELECT max(start) FROM shards)")
        self.args.resume = True
        with mock.patch.object(core, "load_predictor", side_effect=AssertionError("complete orphan re-predicted")):
            self.assertEqual(full.run(self.args)["status"], "COMPLETE")

    def test_changed_shard_or_arguments_rejected(self):
        self.run_full()
        self.args.resume = True
        self.args.cap_max = 3
        with self.assertRaisesRegex(ValueError, "changed"):
            self.run_full()
        self.args.cap_max = 2
        binary = next((self.root / "full" / "shards").glob("*.demand"))
        with binary.open("r+b") as stream:
            stream.seek(-1, os.SEEK_END)
            stream.write(b"X")
        with self.assertRaisesRegex(ValueError, "shard changed"):
            self.run_full()

    def test_gap_in_checkpoint_rejected(self):
        self.run_full()
        with closing(sqlite3.connect(self.root / "full" / "checkpoint.sqlite3")) as connection:
            with connection:
                connection.execute("DELETE FROM shards WHERE start=137")
        self.args.resume = True
        with self.assertRaisesRegex(ValueError, "gap"):
            self.run_full()

    def test_nonfinite_model_and_disk_failure_never_mark_complete(self):
        predictor = CountingPredictor()
        predictor.predict = lambda frame: [float("nan")] * len(frame)
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            self.run_full(predictor)
        self.assertFalse((self.root / "full" / "completion.json").exists())
        self.args.resume = True
        with mock.patch.object(full.io, "require_space", side_effect=RuntimeError("disk reserve")):
            with self.assertRaisesRegex(RuntimeError, "disk reserve"):
                self.run_full()
        self.assertFalse((self.root / "full" / "completion.json").exists())

    def test_second_writer_refused(self):
        self.run_full()
        self.args.resume = True
        with full.io.FileLock(self.root / "full" / ".full-grid.lock"):
            with self.assertRaisesRegex(RuntimeError, "lock busy"):
                self.run_full()

    def test_uncommitted_shard_with_valid_hash_but_wrong_grid_rejected(self):
        self.run_full()
        output = self.root / "full"
        (output / "completion.json").unlink()
        with closing(sqlite3.connect(output / "checkpoint.sqlite3")) as connection:
            with connection:
                connection.execute("DELETE FROM shards")
        binary = output / "shards" / "part_000000000.demand"
        plan = core.read_json(output / "plan.json")
        scope = core.read_json(output / "summary.json")["scope_sha256"]
        data = bytearray(binary.read_bytes()[full.PREFIX_BYTES:])
        data[:core.RECORD.size] = data[core.RECORD.size:2 * core.RECORD.size]
        binary.write_bytes(full.header_bytes(plan, scope, 0, 137, hashlib.sha256(data).hexdigest()) + data)
        self.args.resume = True
        with self.assertRaisesRegex(ValueError, "missing/reordered"):
            self.run_full()

    def test_merge_failure_resume_never_repeats_inference(self):
        self.args.merge_output = True
        with mock.patch.object(full, "merge_shards", side_effect=RuntimeError("injected merge interruption")):
            with self.assertRaisesRegex(RuntimeError, "merge interruption"):
                self.run_full()
        self.args.resume = True
        with mock.patch.object(core, "load_predictor", side_effect=AssertionError("merge recovery loaded model")):
            result = full.run(self.args)
        self.assertEqual(result["status"], "COMPLETE")
        self.assertTrue((self.root / "full" / "table.demand").is_file())

    def test_six_threshold_order_preflight(self):
        observed = []
        def record(args):
            observed.append((args.expected_threshold, args.dry_run))
            return {"status": "dry_run"}
        with mock.patch.object(full, "run", side_effect=record), mock.patch("builtins.print"):
            self.assertEqual(full.main(["--model-dir", str(self.root), "--net-file", str(self.network),
                "--output-dir", str(self.root / "six"), "--signal-remaining-step", "1", "--all-thresholds"]), 0)
        expected = [(x, True) for x in ("1", "3", "0p1", "0p5", "1p5", "2")]
        expected += [(x, False) for x in ("1", "3", "0p1", "0p5", "1p5", "2")]
        self.assertEqual(observed, expected)

    def test_native_cpp_reads_merged_complete_grid(self):
        directory = os.environ.get("ROADNET_TEST_BUILD_DIR")
        if not directory:
            self.skipTest("Set ROADNET_TEST_BUILD_DIR for production C++ reader verification")
        probe = Path(directory) / ("NativeRoadTimeTableTest.exe" if os.name == "nt" else "NativeRoadTimeTableTest")
        self.assertTrue(probe.is_file())
        self.args.merge_output = True
        self.run_full()
        plan = core.read_json(self.root / "full" / "plan.json")
        codec = core.KeyCodec(plan)
        values = self.records()
        keys = list(values)
        rows = [codec.decode(key) for key in keys]
        result = subprocess.run([str(probe), "--probe", str(self.root / "full" / "table.demand")],
            input="".join(" ".join(map(str, row)) + "\n" for row in rows),
            capture_output=True, text=True, check=True, timeout=60)
        actual = {(int(g), int(r)): float(v) for g, r, v in (line.split(",") for line in result.stdout.splitlines())}
        self.assertEqual(actual, values)


if __name__ == "__main__":
    unittest.main()
