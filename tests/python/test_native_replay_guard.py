"""Cross-language coordinator/C++ guard tests; no traffic simulation."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from catching.resume_store import FileLock, atomic_bytes


class NativeReplayGuardTest(unittest.TestCase):
    def setUp(self):
        build = os.environ.get("ROADNET_TEST_BUILD_DIR")
        if not build:
            self.skipTest("Fresh native build required")
        self.binary = Path(build)/("Simulation_Prediction.exe" if os.name == "nt" else "Simulation_Prediction")
        if not self.binary.is_file():
            self.skipTest("Native simulator binary missing")
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.guard = self.root/"replay.lock"
        atomic_bytes(Path(str(self.guard)+".token"), b"new-token")

    def run_guard(self, token):
        return subprocess.run([str(self.binary), "--use-sumo", "--smoke-test",
                               "--sumo-net", str(self.root/"intentionally-missing.net.xml"),
                               "--demand-run-guard", str(self.guard), "--demand-run-token", token],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=15, text=True)

    def test_cpp_cannot_enter_python_owned_replay(self):
        with FileLock(self.guard):
            result = self.run_guard("new-token")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("guard busy", result.stdout)

    def test_delayed_old_child_is_rejected(self):
        result = self.run_guard("old-token")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("token stale", result.stdout)

    def test_correct_token_reaches_normal_loader_and_releases_lock(self):
        result = self.run_guard("new-token")
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("guard busy", result.stdout)
        self.assertNotIn("token stale", result.stdout)
        with FileLock(self.guard):
            pass


if __name__ == "__main__":
    unittest.main()
