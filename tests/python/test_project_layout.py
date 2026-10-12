"""Keep the functional source layout complete and unambiguous."""
import json
import unittest

from roadnet_paths import ROOT, SOURCE_PATHS, source_path


class ProjectLayoutTests(unittest.TestCase):
    def test_manifest_matches_runtime_locator(self):
        manifest = json.loads((ROOT / 'docs/source_layout.json').read_text(encoding='utf-8'))
        self.assertEqual(manifest, SOURCE_PATHS)

    def test_all_relocated_sources_exist_without_duplicate_targets(self):
        self.assertEqual(len(SOURCE_PATHS), len(set(SOURCE_PATHS.values())))
        for old, new in SOURCE_PATHS.items():
            with self.subTest(old=old):
                self.assertEqual(source_path(old), ROOT / new)
                self.assertTrue(source_path(old).is_file())

    def test_current_training_and_catch_are_separate_from_shell(self):
        self.assertEqual(source_path('model_training_sumo_cap_context_abc_rg.py').parent, ROOT / 'training')
        self.assertEqual(source_path('model_catching_sumo_cap_context_abc_rg.py').parent, ROOT / 'catching')
        for new in SOURCE_PATHS.values():
            if new.endswith('.py'):
                self.assertFalse(new.startswith('scripts/'))
            if new.endswith('.sh'):
                self.assertTrue(new.startswith('scripts/'))

    def test_history_and_required_shared_modules_are_distinct(self):
        self.assertEqual(source_path('model_training_sumo_v1.py').parent, ROOT / 'legacy/training')
        self.assertEqual(source_path('model_training_sumo_sf.py').parent, ROOT / 'training/_shared')
        self.assertEqual(source_path('TraCI_Python_Adjusted.py').parent, ROOT / 'collection/_shared')


if __name__ == '__main__':
    unittest.main()
