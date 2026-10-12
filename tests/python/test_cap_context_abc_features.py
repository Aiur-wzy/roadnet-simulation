import copy
import json
import math
import random
import struct
import unittest

from features.cap_context_features import CONTEXT_CONTRACT, ContextConfig, ShadowLedger, ShadowRecord
from features.cap_context_abc_features import (
    ABC_AUDIT_FIELDS, ABC_CONTEXT_IDS, ABC_FEATURE_SCHEMA, ABCShadowLedger, abc_context_contract,
    validate_abc_context_contract,
)


def record(vid="ego", enter=10.0, distance=100.0, cap=10.0, lane=0, road="a", index=0):
    return ShadowRecord(vid, index, road, lane, enter, distance, cap)


class ABCFeaturesTest(unittest.TestCase):
    def assert_base_identical(self, legacy, abc):
        for key, value in legacy.items():
            actual = abc["legacy_platoon_context"] if key == "platoon_context" else abc[key]
            if isinstance(value, float):
                self.assertEqual(struct.pack(">d", value), struct.pack(">d", actual), key)
            else:
                self.assertEqual(value, actual, key)

    def test_schema_and_independent_contract(self):
        self.assertEqual(ABC_FEATURE_SCHEMA, "road-time-cap-context-abc-v1")
        self.assertEqual(ABC_CONTEXT_IDS, (0, 1, 2, 3, 4, 5))
        contract = abc_context_contract()
        self.assertEqual(contract["base_context_contract"], CONTEXT_CONTRACT)
        self.assertNotEqual(contract["version"], CONTEXT_CONTRACT["version"])
        self.assertEqual(contract["context_mapping"]["4"], "B")
        self.assertEqual(contract["context_mapping"]["5"], "C")
        self.assertEqual(contract["context0_overdue_threshold_seconds"], 0)
        contract["base_context_contract"]["crossing_time_seconds"] = 100
        self.assertEqual(CONTEXT_CONTRACT["crossing_time_seconds"], 3)
        self.assertEqual(abc_context_contract(ContextConfig(crossing_time_seconds=0), 2)["base_context_contract"]["crossing_time_seconds"], 0)

    def test_invalid_threshold_and_config(self):
        for bad in (True, False, None, "0", "1.0", math.nan, math.inf, -math.inf, -1, -1e-300):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    ABCShadowLedger(overdue_threshold_seconds=bad)
                with self.assertRaises(ValueError):
                    abc_context_contract(threshold=bad)
        for good in (0, 0.0, 0.5, 30, 1e300):
            self.assertEqual(ABCShadowLedger(overdue_threshold_seconds=good).overdue_threshold_seconds, good)
        with self.assertRaises(ValueError):
            abc_context_contract(None, 0)

    def test_six_states_and_legacy_mapping(self):
        cases = [([], 0, 0, "A"),
                 ([record("a", 0, 150)], 4, 0, "B"),
                 ([record("a", 0, 50)], 5, 0, "C"),
                 ([record("a", 10, 104)], 1, 1, "not_context0"),
                 ([record("a", 10, 105)], 2, 2, "not_context0"),
                 ([record("a", 10, 105), record("b", 10, 105)], 3, 3, "not_context0")]
        for records, context, old, subclass in cases:
            with self.subTest(context=context):
                ledger = ABCShadowLedger()
                for other in records:
                    ledger.reserve(other)
                result = ledger.query(record(), 10)
                self.assertEqual(result["platoon_context"], context)
                self.assertEqual(result["legacy_platoon_context"], old)
                self.assertEqual(result["context0_subclass"], subclass)
                self.assertTrue(set(ABC_AUDIT_FIELDS).issubset(result))

    def test_empty_and_ego_only_A(self):
        ledger, ego = ABCShadowLedger(), record()
        self.assertEqual(ledger.query(ego, 10)["platoon_context"], 0)
        ledger.reserve(ego)
        result = ledger.query(ego, 10)
        self.assertEqual(result["candidate_count"], 0)
        self.assertEqual(result["scanned_count"], 1)
        for key in ("max_overdue_seconds", "sum_overdue_seconds", "overdue_count", "overdue_fraction", "mean_overdue_seconds"):
            self.assertEqual(result[key], 0)

    def test_overdue_boundary_uses_epsilon_only_for_comparison(self):
        # decision=0 permits exact small differences without a large-time ulp.
        ego = record(enter=0, distance=10, cap=1)
        eps = ContextConfig().epsilon_seconds
        for overdue, expected in ((0, 4), (eps, 4), (math.nextafter(eps, math.inf), 5)):
            # f_j=-overdue exactly via planned=-1-overdue and horizon1 loses
            # tiny bits. A tiny fixed horizon prevents that representation loss.
            horizon = 2 ** -60
            other = record("a", enter=-overdue-horizon, distance=horizon, cap=1)
            measured = max(0.0, -other.free_finish_time)
            ledger = ABCShadowLedger()
            ledger.reserve(other)
            result = ledger.query(ego, 0)
            self.assertEqual(measured, overdue)
            self.assertEqual(result["platoon_context"], expected)
            self.assertEqual(result["overdue_count"], int(overdue > eps))

    def test_positive_threshold_boundary_and_count_are_separate(self):
        eps = ContextConfig().epsilon_seconds
        threshold = 2.0
        for overdue in (2.0, threshold + eps, math.nextafter(threshold + eps, math.inf)):
            ledger = ABCShadowLedger(overdue_threshold_seconds=threshold)
            other = record("a", enter=-overdue-0.25, distance=0.25, cap=1)
            ledger.reserve(other)
            result = ledger.query(record(enter=0), 0)
            self.assertEqual(result["max_overdue_seconds"], overdue)
            self.assertEqual(result["overdue_count"], 1)
            self.assertEqual(result["platoon_context"], 4 if overdue <= threshold + eps else 5)

    def test_max_drives_classification_not_sum_or_mean(self):
        ledger = ABCShadowLedger(overdue_threshold_seconds=3)
        for vid in ("a", "b", "c"):
            ledger.reserve(record(vid, 0, 80))  # each overdue2s at decision10
        r = ledger.query(record(), 10)
        self.assertEqual(r["platoon_context"], 4)
        self.assertEqual(r["max_overdue_seconds"], 2)
        self.assertEqual(r["sum_overdue_seconds"], 6)
        self.assertEqual(r["mean_overdue_seconds"], 2)
        self.assertEqual(r["overdue_count"], 3)
        self.assertEqual(r["overdue_fraction"], 1)

    def test_mean_and_fraction_include_nonoverdue_eligible_denominator(self):
        ledger = ABCShadowLedger()
        ledger.reserve(record("old", 0, 50))  # finish5, overdue5
        ledger.reserve(record("other", 0, 150))  # finish15, no overdue
        r = ledger.query(record(), 10)
        self.assertEqual(r["candidate_count"], 2)
        self.assertEqual(r["overdue_count"], 1)
        self.assertEqual(r["overdue_fraction"], .5)
        self.assertEqual(r["mean_overdue_seconds"], 2.5)

    def test_nonzero_classes_keep_continuous_audit_even_if_overdue_exists(self):
        ledger = ABCShadowLedger()
        ledger.reserve(record("old", 0, 50))
        ledger.reserve(record("slow", 10, 120))
        r = ledger.query(record(), 10)
        self.assertEqual(r["platoon_context"], 2)
        self.assertEqual(r["context0_subclass"], "not_context0")
        self.assertEqual(r["max_overdue_seconds"], 5)
        self.assertEqual(r["sum_overdue_seconds"], 5)
        self.assertEqual(r["overdue_count"], 1)

    def test_future_ego_lane_road_filters_and_exact_activation(self):
        ledger, ego = ABCShadowLedger(), record()
        ledger.reserve(ego)
        ledger.reserve(record("pending", math.nextafter(10, math.inf), 1000))
        ledger.reserve(record("other_lane", 0, 1, lane=1))
        ledger.reserve(record("other_road", 0, 1, road="b"))
        r = ledger.query(ego, 10)
        self.assertEqual(r["platoon_context"], 0)
        self.assertEqual(r["pending_count"], 1)
        self.assertEqual(r["candidate_count"], 0)
        self.assertEqual(r["scanned_count"], 2)
        ledger.reserve(record("exact", 10, 50))
        r = ledger.query(ego, 10)
        self.assertEqual(r["platoon_context"], 4)
        self.assertEqual(r["candidate_count"], 1)
        self.assertEqual(r["overdue_count"], 0)

    def test_release_and_revisit_occurrence(self):
        ledger = ABCShadowLedger()
        ledger.reserve(record("a", 0, 50, index=2))
        self.assertEqual(ledger.query(record(), 10)["platoon_context"], 5)
        with self.assertRaises(ValueError):
            ledger.release("a", 1)
        self.assertTrue(ledger.release("a", 2))
        self.assertFalse(ledger.release("a", 2))
        self.assertEqual(ledger.query(record(), 10)["platoon_context"], 0)
        ledger.reserve(record("a", 10, 105, index=3))
        self.assertEqual(ledger.query(record(), 10)["platoon_context"], 2)

    def test_single_scan_and_old_counters(self):
        class CountItems(dict):
            calls = 0
            def items(self):
                self.calls += 1
                return super().items()
        ledger, ego = ABCShadowLedger(), record()
        for other in (ego, record("a", 0, 50), record("b", 10, 150), record("p", 11, 5)):
            ledger.reserve(other)
        lane = CountItems(ledger._lanes[("a", 0)])
        ledger._lanes[("a", 0)] = lane
        r = ledger.query(ego, 10)
        self.assertEqual(lane.calls, 1)
        self.assertEqual(r["scanned_count"], 4)
        self.assertEqual(ledger.scanned_records, 4)
        self.assertEqual(ledger.query_count, 1)
        self.assertEqual(r["candidate_count"], 2)

    def test_old_P_E_and_metadata_bit_identical_randomized(self):
        rng = random.Random(20260913)
        for iteration in range(1200):
            config = ContextConfig(crossing_time_seconds=rng.choice((0.0, 3.0, 7.0)))
            legacy = ShadowLedger(config)
            abc = ABCShadowLedger(config, overdue_threshold_seconds=rng.choice((0.0, .5, 5.0, 30.0)))
            decision = rng.uniform(0, 10000)
            ego = record(enter=decision + config.crossing_time_seconds, distance=rng.choice((320.0, 540.5, 545.6)), cap=rng.uniform(5, 20))
            records = [record(str(j), decision + rng.uniform(-100, 10), rng.uniform(.1, 1000), rng.uniform(3, 20),
                              lane=rng.randrange(3), road=rng.choice(("a", "a", "b")), index=rng.randrange(5)) for j in range(rng.randrange(50))]
            if iteration % 2 == 0:
                records.append(ego)
            for other in records:
                legacy.reserve(other)
                abc.reserve(other)
            if records and iteration % 3 == 0:
                other = records[rng.randrange(len(records))]
                self.assertEqual(legacy.release(other.vehicle_id, other.route_index), abc.release(other.vehicle_id, other.route_index))
            before, after = legacy.query(ego, decision), abc.query(ego, decision)
            self.assert_base_identical(before, after)
            self.assertEqual(legacy.scanned_records, abc.scanned_records)
            self.assertEqual(legacy.query_count, abc.query_count)
            self.assertEqual(legacy.lane_load("a", 0), abc.lane_load("a", 0))
            self.assertEqual(len(legacy), len(abc))

    def test_neumaier_order_remains_identical(self):
        old, new = ShadowLedger(), ABCShadowLedger()
        ego = record(enter=0, distance=1, cap=1)
        for j, distance in enumerate((1e16, 2.0, 2.0, 2.0, 1e15, 2.0)):
            other = record(str(j), enter=0, distance=distance, cap=1)
            old.reserve(other)
            new.reserve(other)
        self.assert_base_identical(old.query(ego, 0), new.query(ego, 0))

    def test_invalid_decision_and_past_ego_match_old_rejection(self):
        for ledger in (ShadowLedger(), ABCShadowLedger()):
            for decision in (math.nan, math.inf, True, 11):
                with self.subTest(ledger=type(ledger).__name__, decision=decision), self.assertRaises(ValueError):
                    ledger.query(record(), decision)


class ABCContractValidationTest(unittest.TestCase):
    def test_default_and_json_roundtrip(self):
        original = abc_context_contract()
        parsed = json.loads(json.dumps(original, allow_nan=False))
        canonical = validate_abc_context_contract(parsed)
        self.assertEqual(canonical, original)
        self.assertIsNot(canonical, parsed)
        self.assertIsNot(canonical["base_context_contract"], parsed["base_context_contract"])
        self.assertIsNot(canonical["context_mapping"], parsed["context_mapping"])
        self.assertIsNot(canonical["context_ids"], parsed["context_ids"])
        canonical["context_ids"].append(6)
        self.assertEqual(parsed, original)

    def test_custom_crossing_and_threshold_roundtrip(self):
        for crossing in (0, 0.125, 3, 7.5, 1e300):
            for threshold in (0, 1e-300, 0.5, 30, 1e300):
                with self.subTest(crossing=crossing, threshold=threshold):
                    value = abc_context_contract(ContextConfig(crossing_time_seconds=crossing), threshold)
                    result = validate_abc_context_contract(json.loads(json.dumps(value)))
                    self.assertEqual(result, value)
                    self.assertIs(type(result["base_context_contract"]["crossing_time_seconds"]), float)
                    self.assertIs(type(result["context0_overdue_threshold_seconds"]), float)
        a = validate_abc_context_contract(abc_context_contract(threshold=0))
        b = validate_abc_context_contract(abc_context_contract(threshold=2))
        # The training loader must compare these, not silently mix valid files.
        self.assertNotEqual(a, b)

    def test_reject_old_contract_and_wrong_root_types(self):
        for value in (CONTEXT_CONTRACT, None, [], "free-clock-context-abc-v1", True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_abc_context_contract(value)

    def test_exact_fields_at_every_dictionary_level(self):
        for path in ((), ("base_context_contract",), ("context_mapping",), ("context0_subclass_rules",)):
            for mutation in ("extra", "missing", "numeric_key"):
                value = abc_context_contract()
                target = value
                for key in path:
                    target = target[key]
                if mutation == "missing":
                    target.pop(next(iter(target)))
                elif mutation == "numeric_key":
                    target[1] = "not a JSON object key"
                else:
                    target["unknown"] = "unversioned extension"
                with self.subTest(path=path, mutation=mutation), self.assertRaises(ValueError):
                    validate_abc_context_contract(value)

    def test_all_string_fields_and_rules_are_fixed_strings(self):
        paths = [(key,) for key, item in abc_context_contract().items() if isinstance(item, str)]
        for group in ("base_context_contract", "context_mapping", "context0_subclass_rules"):
            paths.extend((group, key) for key, item in abc_context_contract()[group].items() if isinstance(item, str))
        for path in paths:
            for wrong in ("different-v2-rule", 0, True, None, ["same-looking"]):
                value = abc_context_contract()
                target = value
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = wrong
                with self.subTest(path=path, wrong=wrong), self.assertRaises(ValueError):
                    validate_abc_context_contract(value)

    def test_ids_require_exact_ordered_integers(self):
        bad_ids = [(0, 1, 2, 3, 4, 5), [0, 1, 2, 3], [0, 1, 2, 3, 4, 5, 6],
                   [0, 1, 2, 3, 5, 4], [0, 1, 2, 3, 4, 4],
                   [False, 1, 2, 3, 4, 5], [0, True, 2, 3, 4, 5],
                   [0.0, 1, 2, 3, 4, 5], [0, "1", 2, 3, 4, 5]]
        for ids in bad_ids:
            value = abc_context_contract()
            value["context_ids"] = ids
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                validate_abc_context_contract(value)
        value = abc_context_contract()
        value["context_mapping"].update({"4": "C", "5": "B"})
        with self.assertRaises(ValueError):
            validate_abc_context_contract(value)

    def test_all_numeric_fields_reject_invalid_types_and_nonfinite(self):
        paths = [("context0_overdue_threshold_seconds",), ("overdue_epsilon_seconds",)]
        paths.extend(("base_context_contract", key) for key in
                     ("p_threshold", "e_threshold", "epsilon_seconds", "crossing_time_seconds"))
        for path in paths:
            for wrong in (True, False, None, "0", math.nan, math.inf, -math.inf, -1, 10 ** 1000):
                value = abc_context_contract()
                target = value
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = wrong
                with self.subTest(path=path, wrong=repr(wrong)[:30]), self.assertRaises(ValueError):
                    validate_abc_context_contract(value)

    def test_fixed_P_E_and_epsilon_exact_boundaries(self):
        for key in ("p_threshold", "e_threshold", "epsilon_seconds"):
            exact = CONTEXT_CONTRACT[key]
            for wrong in (0, math.nextafter(exact, -math.inf), math.nextafter(exact, math.inf)):
                value = abc_context_contract()
                value["base_context_contract"][key] = wrong
                if key == "epsilon_seconds":
                    value["overdue_epsilon_seconds"] = wrong
                with self.subTest(key=key, wrong=wrong), self.assertRaises(ValueError):
                    validate_abc_context_contract(value)
        for wrong in (0, math.nextafter(1e-9, -math.inf), math.nextafter(1e-9, math.inf)):
            value = abc_context_contract()
            value["overdue_epsilon_seconds"] = wrong
            with self.subTest(wrong=wrong), self.assertRaises(ValueError):
                validate_abc_context_contract(value)

    def test_validate_does_not_mutate_input_on_rejection(self):
        value = abc_context_contract(ContextConfig(crossing_time_seconds=7), threshold=2)
        value["context0_subclass_rules"]["B"] = "uses mean instead of max"
        before = copy.deepcopy(value)
        with self.assertRaises(ValueError):
            validate_abc_context_contract(value)
        self.assertEqual(value, before)


if __name__ == "__main__":
    unittest.main()
