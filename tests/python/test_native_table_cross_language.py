"""Python writer / production C++ reader parity, without training or simulation."""
import os
from pathlib import Path
import random
import subprocess
import tempfile
import unittest

from catching import catch_road_time as catch
from roadnet_paths import ROOT
from tests.python.test_native_catching_v2 import fixture_args


class NativeTableCrossLanguageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        directory = os.environ.get("ROADNET_TEST_BUILD_DIR")
        if not directory:
            raise unittest.SkipTest("Set ROADNET_TEST_BUILD_DIR to a fresh native-table build")
        cls.probe = Path(directory) / ("NativeRoadTimeTableTest.exe" if os.name == "nt" else "NativeRoadTimeTableTest")
        if not cls.probe.is_file():
            raise unittest.SkipTest("NativeRoadTimeTableTest not built")

    def check_plan(self, args, directory, require_large=False):
        plan = catch.build_plan(args)
        codec = catch.KeyCodec(plan)
        rng = random.Random(20261001)
        rows = []
        grid = plan["free_speed_cap_grid"]
        caps = [grid["min"], grid["max"], (grid["index_min"] + .5) * grid["step"]]
        for index in range(500):
            lane = rng.randrange(plan["lane_flow_max"] + 1)
            road = rng.randint(lane, plan["road_flow_max"])
            signal = rng.choice(plan["signal_catalog"])
            remaining = 0. if signal["sentinel"] else rng.uniform(.00001, signal["entry_max_seconds"])
            rows.append((rng.choice(plan["road_lengths"]), road, lane,
                         caps[index % len(caps)], rng.randrange(6), signal["entry_type"], remaining,
                         signal["next_type"], signal["next_duration_seconds"]))
        # Exact half-node ties and endpoint rows, including final-road sentinel.
        for signal in plan["signal_catalog"]:
            remaining_values = signal["nodes"][:1] + signal["nodes"][-1:]
            if len(signal["nodes"]) > 1:
                remaining_values.append((signal["nodes"][0] + signal["nodes"][1]) / 2)
            for remaining in remaining_values:
                rows.append((plan["road_lengths"][-1], plan["road_flow_max"], plan["lane_flow_max"],
                             grid["max"], 5, signal["entry_type"], remaining,
                             signal["next_type"], signal["next_duration_seconds"]))
        if args.cap_out_of_range == "clamp":
            for raw in (grid["min"] / 100, grid["max"] * 100, 1.0874801041404398):
                rows.append((plan["road_lengths"][0], 1, 1, raw, 0, "G", 0., "G", 0.))
        keys = [codec.encode(row) for row in rows]
        if require_large:
            self.assertGreater(max(key[0] for key in keys), 2**32)
            self.assertGreater(plan["expanded_rows"], 2**32)
        values = {key: 1. + key[0] % 997 + key[1] / 1000 for key in keys[::2]}
        destination = directory / "cross-language.demand"
        header = catch.write_cache(destination, plan, "a" * 64, values)
        self.assertEqual(header["payload_bytes"], 20 * len(values))
        output = subprocess.run([str(self.probe), "--probe", str(destination)],
                                input="".join(" ".join(str(value) for value in row) + "\n" for row in rows),
                                capture_output=True, text=True, timeout=60, check=True)
        lines = output.stdout.splitlines()
        self.assertEqual(len(lines), len(keys), output.stderr)
        for key, line in zip(keys, lines):
            group, road, prediction = line.split(",")
            self.assertEqual((int(group), int(road)), key)
            if key in values:
                self.assertEqual(float(prediction), values[key])
            else:
                self.assertEqual(prediction, "MISSING")

    def test_large_index_and_quantization_parity(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            args = fixture_args(directory)
            args.cap_max = 1_000_000_001.
            self.check_plan(args, directory, require_large=True)

    def test_piecewise_flow_parity(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            args = fixture_args(directory)
            args.road_flow_segments = ["10:1", "20:5"]
            args.road_flow_step = 1
            args.road_flow_dense_until = 10
            self.check_plan(args, directory)

    def test_strict_cap_policy_parity(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            args = fixture_args(directory)
            args.cap_out_of_range = "error"
            args.first_road_position_correction = "off"
            self.check_plan(args, directory)

    def test_small_network_catalog_parity(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            args = fixture_args(directory)
            args.net_file = str(ROOT / "data/net/test_new.net.xml")
            if not Path(args.net_file).is_file():
                self.skipTest("Local small network unavailable")
            self.check_plan(args, directory)

    def test_manhattan_catalog_parity(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            args = fixture_args(directory)
            args.net_file = str(ROOT / "baselines/ICDE2025/manhattan/network.net.xml")
            if not Path(args.net_file).is_file():
                self.skipTest("Local Manhattan network unavailable")
            args.signal_remaining_step = 5.
            args.cap_max = 100.
            self.check_plan(args, directory, require_large=True)


if __name__ == "__main__":
    unittest.main()
