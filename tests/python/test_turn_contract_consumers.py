"""Cross-entrypoint contract checks; never import or train AutoGluon."""
import unittest

import legacy.training.model_training_sumo_v1 as train_v1
import legacy.training.model_training_sumo_v2 as train_v2
import legacy.catching.model_catching_sumo_v1 as catch_v1
import training._shared.model_training_sumo_sf as train_sf
import catching._shared.model_catching_sumo_sf as catch_sf
import legacy.training.model_training_sumo_cap_context as train_cap
import legacy.catching.model_catching_sumo_cap_context as catch_cap
from features.turn_context import TURN_TYPE_MAP, encode_turn_direction


class TurnContractConsumersTest(unittest.TestCase):
    def test_legacy_raw_turn_converters_share_canonical_mapping(self):
        expected = {"l": 1, "L": 1, "s": 2, "S": 2, "r": 3, "R": 3,
                    "t": 4, "U": 4, "unknown": 0, "end": 0, "": 0}
        for module in (train_v1, train_v2, catch_v1):
            self.assertIs(module.TURN_TYPE_MAP, TURN_TYPE_MAP)
            for raw, value in expected.items():
                with self.subTest(module=module.__name__, direction=raw):
                    self.assertEqual(module._encode_legacy_turn(raw), value)
                    self.assertEqual(module._encode_legacy_turn(raw), encode_turn_direction(raw))
            self.assertEqual(module._encode_legacy_turn(None), 0)
            self.assertEqual(module._encode_legacy_turn(float("nan")), 0)

    def test_active_five_feature_schemas_remain_turn_free(self):
        sf = ["road_length", "road_flow", "lane_flow", "speed_factor", "speed_limit"]
        cap = ["road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context"]
        for module, expected in ((train_sf, sf), (catch_sf, sf), (train_cap, cap), (catch_cap, cap)):
            with self.subTest(module=module.__name__):
                self.assertEqual(module.FEATURE_COLS, expected)
                self.assertFalse(any("turn" in name for name in module.FEATURE_COLS))


if __name__ == "__main__":
    unittest.main()
