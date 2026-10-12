"""Current entry points must not route through historical model implementations."""
import ast
import subprocess
import sys
import unittest

from roadnet_paths import ROOT


class CurrentModelEntrypointTests(unittest.TestCase):
    def test_public_help_without_training_framework_or_legacy_imports(self):
        for module in ("training.train_road_time", "catching.catch_road_time"):
            with self.subTest(module=module):
                # A separate process prevents other regression modules' legacy
                # imports and fake AutoGluon modules from contaminating this check.
                code = (
                    "import importlib,sys; "
                    f"importlib.import_module({module!r}); "
                    "bad=[m for m in sys.modules if m.startswith('autogluon') "
                    "or m.startswith(('training._shared','catching._shared','legacy.')) "
                    "or m in ('training.model_training_sumo_cap_context_abc_rg',"
                    "'catching.model_catching_sumo_cap_context_abc_rg',"
                    "'catching.catch_abc_rg_demand','catching.catch_abc_rg_batch')]; "
                    "assert not bad,bad"
                )
                checked = subprocess.run([sys.executable, "-B", "-c", code],
                                         cwd=ROOT, capture_output=True, text=True, timeout=60)
                self.assertEqual(checked.returncode, 0, checked.stdout + checked.stderr)
                helped = subprocess.run([sys.executable, "-B", "-m", module, "--help"],
                                        cwd=ROOT, capture_output=True, text=True, timeout=60)
                self.assertEqual(helped.returncode, 0, helped.stdout + helped.stderr)
                self.assertIn("usage:", helped.stdout)

    def test_current_sources_do_not_import_legacy_pipeline(self):
        sources = [ROOT / "training/train_road_time.py", ROOT / "catching/catch_road_time.py"]
        sources += sorted((ROOT / "training").glob("native_*.py"))
        sources += sorted((ROOT / "catching").glob("native_*.py"))
        forbidden = ("legacy", "training._shared", "catching._shared",
                     "training.model_training_sumo", "catching.model_catching_sumo",
                     "catching.catch_abc_rg")
        for path in sources:
            with self.subTest(path=path.name):
                tree = ast.parse(path.read_text(encoding="utf-8-sig"))
                for node in ast.walk(tree):
                    modules = ([alias.name for alias in node.names] if isinstance(node, ast.Import)
                               else [node.module or ""] if isinstance(node, ast.ImportFrom) else [])
                    for name in modules:
                        self.assertFalse(name.startswith(forbidden), (path.name, name))

    def test_shared_native_schema_and_six_threshold_order(self):
        from training.native_contract import FEATURE_COLUMNS, MODEL_SCHEMA, SOURCE_SCHEMA, THRESHOLD_ORDER
        self.assertEqual(SOURCE_SCHEMA, "road-time-sumo-native-euler-streaming-quality-v2")
        self.assertEqual(MODEL_SCHEMA, "road-time-native-model-v2")
        self.assertEqual(THRESHOLD_ORDER, ["1", "3", "0p1", "0p5", "1p5", "2"])
        self.assertEqual(len(FEATURE_COLUMNS), 9)
        self.assertNotIn("vehicle_id", FEATURE_COLUMNS)
        self.assertNotIn("route_index", FEATURE_COLUMNS)


if __name__ == "__main__":
    unittest.main()
