"""SF collection/output regressions; no SUMO run or model training required."""

import contextlib
import csv
import io
import math
from pathlib import Path
import types
import unittest
from unittest import mock
import uuid

from tests.python.test_final_road_training import (
    cams, config, event, network_info, snapshot, sweep, vehicle_features,
)


class SpeedFactorCollectionTests(unittest.TestCase):
    def test_actual_vehicle_value_not_type_mean_and_no_rounding(self):
        actual = 0.8123456789012345
        api = types.SimpleNamespace(getSpeedFactor=mock.Mock(return_value=actual))
        type_api = types.SimpleNamespace(getSpeedFactor=mock.Mock(return_value=1.0))
        with mock.patch.object(cams, "traci", types.SimpleNamespace(
            vehicle=api, vehicletype=type_api,
        )):
            self.assertEqual(cams.read_vehicle_speed_factor("v"), actual)
        api.getSpeedFactor.assert_called_once_with("v")
        type_api.getSpeedFactor.assert_not_called()

    def test_each_first_middle_final_road_records_resolved_value(self):
        factors = (0.8123456789012345, 0.8123456789012345, 0.8123456789012345)
        get_factor = mock.Mock(side_effect=factors)
        traci = types.SimpleNamespace(vehicle=types.SimpleNamespace(getSpeedFactor=get_factor))
        events = []
        with mock.patch.object(cams, "traci", traci), mock.patch.object(
            cams, "compute_flow_snapshot", return_value=({}, {}, {}),
        ):
            for index in range(3):
                s = snapshot(final=index == 2, route=("a", "b", "a"), route_index=index)
                events.append(cams.make_vehicle_event(s, index * 10.0, network_info(), {}, config()))
        self.assertEqual([e.route_index for e in events], [0, 1, 2])
        self.assertEqual([e.is_final_road for e in events], [False, False, True])
        self.assertEqual([e.speed_factor for e in events], list(factors))
        self.assertEqual(get_factor.call_args_list, [mock.call("v")] * 3)

    def test_runtime_query_failure_does_not_fall_back_to_type_or_one(self):
        type_getter = mock.Mock(return_value=1.0)
        for api in (types.SimpleNamespace(), types.SimpleNamespace(
            getSpeedFactor=mock.Mock(side_effect=RuntimeError("vehicle unavailable")),
        )):
            with self.subTest(api=api), mock.patch.object(cams, "traci", types.SimpleNamespace(
                vehicle=api, vehicletype=types.SimpleNamespace(getSpeedFactor=type_getter),
            )):
                with self.assertRaisesRegex(RuntimeError, "cannot read actual speed_factor.*'v'"):
                    cams.read_vehicle_speed_factor("v")
        type_getter.assert_not_called()

    def test_invalid_runtime_values_fail_instead_of_training(self):
        for value in (None, "invalid", 0.0, -1.0, float("nan"), float("inf"), -float("inf")):
            with self.subTest(value=value), mock.patch.object(cams, "traci", types.SimpleNamespace(
                vehicle=types.SimpleNamespace(getSpeedFactor=lambda _vehicle_id: value),
            )):
                with self.assertRaisesRegex(ValueError, "invalid speed_factor.*'v'"):
                    cams.read_vehicle_speed_factor("v")

    def test_factor_above_one_is_preserved_not_capped(self):
        self.assertEqual(cams.validate_speed_factor(1.2345678901234567, "v"), 1.2345678901234567)

    def test_read_once_at_entry_not_each_time_sample(self):
        getter = mock.Mock(return_value=0.9123456789012345)
        s = snapshot()
        with mock.patch.object(cams, "traci", types.SimpleNamespace(
            vehicle=types.SimpleNamespace(getSpeedFactor=getter),
        )), mock.patch.object(cams, "compute_flow_snapshot", return_value=({}, {}, {})):
            e = cams.make_vehicle_event(s, 1.0, network_info(), {}, config())
            for now in (2.0, 3.0, 4.0):
                cams.accumulate_waiting_time(e, s, config(), now)
        getter.assert_called_once_with("v")
        self.assertEqual(e.driving_time, 3.0)


class SpeedFactorWriterTests(unittest.TestCase):
    def setUp(self):
        self.directory = Path.cwd() / ("speed-factor-training-test-" + uuid.uuid4().hex)
        self.directory.mkdir()
        self.addCleanup(self.cleanup_outputs)

    def cleanup_outputs(self):
        for path in self.directory.iterdir():
            path.unlink()
        self.directory.rmdir()

    def rows(self, path):
        with path.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            return list(reader), reader.fieldnames

    def writers(self):
        for module in (cams, sweep, vehicle_features):
            for writer in (module.write_lightweight_training_rows, module.write_training_rows):
                yield module.__name__ + "-" + writer.__name__, writer

    def test_all_six_writers_preserve_precision_labels_and_terminal_rows(self):
        factor = math.nextafter(0.8123456789012345, 1.0)
        events = [event(final=False, completed=True), event(final=False, completed=True),
                  event(final=True, completed=True)]
        for index, e in enumerate(events):
            e.route_index = index
            e.speed_factor = factor
            e.threshold_sweep_speed_bins = [0.0] * 6 + [2.0]
        for name, writer in self.writers():
            with self.subTest(writer=name):
                path = self.directory / (name + ".csv")
                with contextlib.redirect_stdout(io.StringIO()):
                    writer(str(path), events)
                rows, fields = self.rows(path)
                self.assertEqual(len(rows), 3)
                self.assertEqual(fields.count("speed_factor"), 1)
                self.assertEqual([float(row["speed_factor"]) for row in rows], [factor] * 3)
                self.assertEqual([float(row["travel_time_label"]) for row in rows], [2.0] * 3)
                if "is_final_road" in fields:
                    self.assertEqual([row["is_final_road"] for row in rows], ["0", "0", "1"])
                if "Threshold_Sweep" in name:
                    for threshold in sweep.DEFAULT_LABEL_SPEED_THRESHOLDS:
                        label = "travel_time_label_gt_" + sweep.threshold_token(threshold)
                        self.assertEqual([float(row[label]) for row in rows], [2.0] * 3)
                    self.assertTrue(all(float(row["speed_bin_conservation_residual"]) == 0 for row in rows))

    def test_missing_event_factor_is_error_for_all_writers(self):
        e = event(completed=True)
        e.speed_factor = None
        for name, writer in self.writers():
            with self.subTest(writer=name), self.assertRaisesRegex(ValueError, "invalid speed_factor"):
                writer(str(self.directory / (name + ".csv")), [e])

    def test_sf_addition_does_not_change_any_existing_output_value(self):
        left, right = event(completed=True), event(completed=True)
        left.speed_factor, right.speed_factor = 0.7, 1.3
        for e in (left, right):
            e.threshold_sweep_speed_bins = [0.0] * 6 + [2.0]
        for name, writer in self.writers():
            with self.subTest(writer=name):
                path = self.directory / (name + "-invariance.csv")
                with contextlib.redirect_stdout(io.StringIO()):
                    writer(str(path), [left, right])
                rows, _ = self.rows(path)
                self.assertNotEqual(rows[0].pop("speed_factor"), rows[1].pop("speed_factor"))
                self.assertEqual(rows[0], rows[1])

    def test_legacy_vehicle_training_contract_is_not_silently_expanded(self):
        self.assertEqual(len(vehicle_features.VEHICLE_FEATURES), 6)
        self.assertEqual(len(vehicle_features.LIGHTWEIGHT_FEATURES), 11)
        self.assertNotIn("speed_factor", vehicle_features.VEHICLE_FEATURES)
        self.assertNotIn("speed_factor", vehicle_features.LIGHTWEIGHT_FEATURES)


if __name__ == "__main__":
    unittest.main()
