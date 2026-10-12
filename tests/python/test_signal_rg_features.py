"""Pure timing tests, including an independent per-second reference oracle."""
import itertools
import json
import math
import random
import unittest

from features.signal_rg_features import (
    RGSignalProgram, SIGNAL_FEATURE_COLS, SIGNAL_FEATURE_SCHEMA,
    no_signal_features, signal_feature_contract,
)


def formal(result):
    return tuple(result[column] for column in SIGNAL_FEATURE_COLS)


class RGSignalProgramTests(unittest.TestCase):
    def test_j17_red_is_movement_segment_not_tls_phase_remainder(self):
        program = RGSignalProgram([24, 3, 15, 3, 24, 3, 15, 3], "Gyrrrrrr")
        result = program.query(2, 14)  # t=28, this TLS phase ends at t=42.
        self.assertEqual(formal(result), ("R", 62.0, "G", 24.0))
        self.assertEqual(result["signal_state_at_reference"], "R")
        self.assertEqual(result["signal_yellow_skipped"], 0)

    def test_yellow_skips_to_two_complete_nonyellow_segments(self):
        program = RGSignalProgram([24, 3, 63], "Gyr")
        result = program.query(1, 2)
        self.assertEqual(formal(result), ("R", 63.0, "G", 24.0))
        self.assertEqual(result["signal_state_at_reference"], "Y")
        self.assertEqual(result["signal_yellow_skipped"], 1)
        self.assertNotIn("start_delta", result)
        self.assertEqual(len(result), 6)

    def test_green_remainder_and_next_red_exclude_yellow(self):
        self.assertEqual(formal(RGSignalProgram([24, 3, 63], "Gyr").query(0, 5)),
                         ("G", 5.0, "R", 63.0))

    def test_adjacent_case_variants_merge(self):
        program = RGSignalProgram([2, 5, 3, 4, 6], ["r", "R", "y", "g", "G"])
        self.assertEqual(formal(program.query(0, 1)), ("R", 6.0, "G", 10.0))
        self.assertEqual(formal(program.query(3, 2)), ("G", 8.0, "R", 7.0))

    def test_same_color_across_cycle_boundary_merges(self):
        program = RGSignalProgram([2, 3, 4, 5], "rGyr")
        self.assertEqual(formal(program.query(3, 4)), ("R", 6.0, "G", 3.0))
        self.assertEqual(formal(program.query(0, 1)), ("R", 1.0, "G", 3.0))
        self.assertEqual(formal(program.query(2, 2)), ("R", 7.0, "G", 3.0))

    def test_green_yellow_green_stays_two_distinct_segments(self):
        program = RGSignalProgram([5, 2, 7, 3], "GyGr")
        self.assertEqual(formal(program.query(0, 4)), ("G", 4.0, "G", 7.0))
        self.assertEqual(formal(program.query(1, 1)), ("G", 7.0, "R", 3.0))

    def test_one_finite_green_run_repeats_after_yellow(self):
        program = RGSignalProgram([5, 2], "Gy")
        self.assertEqual(formal(program.query(0, 3)), ("G", 3.0, "G", 5.0))
        self.assertEqual(formal(program.query(1, 1)), ("G", 5.0, "G", 5.0))

    def test_no_yellow_program(self):
        program = RGSignalProgram([9, 4], "rG")
        self.assertEqual(formal(program.query(0, 6)), ("R", 6.0, "G", 4.0))

    def test_arbitrary_fractional_durations(self):
        program = RGSignalProgram([1.25, 0.5, 2.75, 0.25], "rGyr")
        self.assertEqual(program.cycle_seconds, 4.75)
        self.assertEqual(formal(program.query(3, 0.125)), ("R", 1.375, "G", 0.5))
        self.assertEqual(formal(program.query(2, 1.5)), ("R", 1.5, "G", 0.5))

    def test_lookahead_crosses_yellow_to_red(self):
        program = RGSignalProgram([24, 3, 63], "Gyr")
        self.assertEqual(formal(program.query(0, 1, 3)), ("R", 63.0, "G", 24.0))
        self.assertEqual(program.query(0, 1, 3)["signal_state_at_reference"], "Y")
        self.assertEqual(formal(program.query(0, 1, 5)), ("R", 62.0, "G", 24.0))

    def test_lookahead_crosses_cycle_and_many_cycles(self):
        program = RGSignalProgram([24, 3, 63], "Gyr")
        expected = program.query(0, 23)
        self.assertEqual(program.query(2, 2, 3), expected)
        self.assertEqual(program.query(2, 2, 90 * 10**9 + 3), expected)
        self.assertEqual(program.query(2, 2, 90 * 10**9), program.query(2, 2))

    def test_half_open_exact_phase_boundaries(self):
        program = RGSignalProgram([24, 3, 63], "Gyr")
        self.assertEqual(program.query(0, 0), program.query(1, 3))
        self.assertEqual(program.query(1, 0), program.query(2, 63))
        self.assertEqual(program.query(2, 0), program.query(0, 24))
        self.assertEqual(program.query(0, 1, 1), program.query(1, 3))
        self.assertEqual(program.query(0, 1, 4), program.query(2, 63))

    def test_zero_remaining_with_additional_lookahead(self):
        program = RGSignalProgram([4, 1, 9], "Gyr")
        self.assertEqual(program.query(0, 0, 3), program.query(2, 7))

    def test_invalid_program_inputs(self):
        cases = [([], []), ([1], []), ([1], "rr"), ([0, 1], "rG"),
                 ([-1, 2], "rG"), ([math.nan, 1], "rG"), ([math.inf, 1], "rG"),
                 ([True, 1], "rG"), (["1", 2], "rG"), ([1, 2], "ro"),
                 ([1, 2], "ru"), ([1, 2], ["rr", "G"]), ([1, 2], [None, "G"]),
                 (None, "rG"), ([1, 2], None), ([1e308, 1e308], "rG")]
        for durations, states in cases:
            with self.subTest(durations=durations, states=states), self.assertRaises(ValueError):
                RGSignalProgram(durations, states)

    def test_uniform_or_all_yellow_cycles_are_not_invented_durations(self):
        for states in ("r", "G", "y", "rRr", "gGG", "yYy"):
            with self.subTest(states=states), self.assertRaises(ValueError):
                RGSignalProgram([5] * len(states), states)

    def test_invalid_query_inputs(self):
        program = RGSignalProgram([24, 3, 63], "Gyr")
        cases = [(True, 1, 0), (1.0, 1, 0), (-1, 1, 0), (3, 1, 0),
                 (0, -1, 0), (0, 25, 0), (0, math.nan, 0), (0, math.inf, 0),
                 (0, True, 0), (0, "1", 0), (0, 1, -1), (0, 1, math.inf),
                 (0, 1, math.nan), (0, 1, True)]
        for args in cases:
            with self.subTest(args=args), self.assertRaises(ValueError):
                program.query(*args)

    def test_no_signal_placeholder_is_fresh_and_explicit(self):
        expected = ("G", 0.0, "G", 0.0)
        self.assertEqual(formal(no_signal_features()), expected)
        result = no_signal_features()
        result["entry_signal_type"] = "R"
        self.assertEqual(formal(no_signal_features()), expected)

    def test_contract_is_json_fresh_and_documents_excluded_inputs(self):
        contract = signal_feature_contract()
        self.assertEqual(json.loads(json.dumps(contract)), contract)
        self.assertEqual(contract["feature_schema"], SIGNAL_FEATURE_SCHEMA)
        self.assertEqual(contract["feature_columns"], SIGNAL_FEATURE_COLS)
        self.assertEqual(len(contract["feature_columns"]), 4)
        self.assertIn("no start_delta", contract["yellow_duration_policy"])
        contract["feature_columns"].append("bad")
        self.assertEqual(signal_feature_contract()["feature_columns"], SIGNAL_FEATURE_COLS)

    def test_independent_discrete_oracle_many_static_programs(self):
        rng = random.Random(27419)
        for count in range(2, 6):
            for states in itertools.product("RGY", repeat=count):
                if len(set(states)) == 1:
                    continue
                durations = [rng.randint(1, 5) for _ in states]
                ticks = [state for state, duration in zip(states, durations) for _ in range(duration)]
                cycle = len(ticks)
                program = RGSignalProgram(durations, states)
                for _ in range(8):
                    phase = rng.randrange(count)
                    remaining = rng.randint(0, durations[phase])
                    lookahead = rng.randint(0, 10 * cycle)
                    offset = (sum(durations[:phase]) + durations[phase] - remaining + lookahead) % cycle
                    reference = ticks[offset]
                    position = offset
                    if reference == "Y":
                        while ticks[position % cycle] == "Y":
                            position += 1
                    color = ticks[position % cycle]
                    first = 0
                    while ticks[position % cycle] == color:
                        first += 1
                        position += 1
                    while ticks[position % cycle] == "Y":
                        position += 1
                    next_color = ticks[position % cycle]
                    second = 0
                    while ticks[position % cycle] == next_color:
                        second += 1
                        position += 1
                    actual = program.query(phase, remaining, lookahead)
                    self.assertEqual(formal(actual), (color, float(first), next_color, float(second)))
                    self.assertEqual(actual["signal_state_at_reference"], reference)
                    self.assertEqual(actual["signal_yellow_skipped"], int(reference == "Y"))


if __name__ == "__main__":
    unittest.main()
