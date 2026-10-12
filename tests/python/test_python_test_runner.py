"""Test runner planning and isolation without launching the regression suite."""

import os
from contextlib import redirect_stdout
import io
from types import SimpleNamespace
import unittest
from unittest import mock

from tools.validation import run_python_tests as runner


class PythonTestRunnerTests(unittest.TestCase):
    def test_default_plan_has_only_isolated_unit_discovery(self):
        self.assertEqual(runner.build_plan(), [[runner.sys.executable, "-B", "-m",
                                               runner.RUNNER_MODULE, "--unit-worker"]])

    def test_include_sumo_uses_fresh_interpreter_for_each_live_class(self):
        commands = runner.build_plan(include_sumo=True)
        self.assertEqual(commands[0], runner.build_plan()[0])
        self.assertEqual(len(commands), 1 + len(runner.LIVE_SUMO_CLASSES))
        for command, name in zip(commands[1:], runner.LIVE_SUMO_CLASSES):
            self.assertEqual(command, [runner.sys.executable, "-B", "-m", "unittest",
                                       "-v", name])
        self.assertIn("tests.python.test_discharge_e1_adapter.LiveParityTests",
                      runner.LIVE_SUMO_CLASSES)

    def test_nested_filter_keeps_ordinary_tests_and_import_failures(self):
        class NamedTest(unittest.TestCase):
            def __init__(self, name):
                super().__init__()
                self.name = name

            def id(self):
                return self.name

        unit = NamedTest("tests.python.test_discharge_sumo.GateTests.test_gate")
        live = [NamedTest(name + ".test_fixture") for name in runner.LIVE_SUMO_CLASSES]
        similar = NamedTest(runner.LIVE_SUMO_CLASSES[0] + "Extra.test_unit")
        failed_import = NamedTest("unittest.loader._FailedTest.test_broken_import")
        nested = unittest.TestSuite([unit, unittest.TestSuite(live + [similar, failed_import])])
        self.assertEqual(list(runner.unit_tests_only(nested)), [unit, similar, failed_import])

    def test_dry_run_and_list_never_launch_or_import_tests(self):
        for arguments in (["--dry-run", "--include-sumo"], ["--list"]):
            with self.subTest(arguments=arguments), mock.patch.object(runner.subprocess, "run") as run, \
                    mock.patch.object(runner, "run_unit_worker") as worker, \
                    redirect_stdout(io.StringIO()) as output:
                self.assertEqual(runner.main(arguments), 0)
                run.assert_not_called()
                worker.assert_not_called()
                for name in runner.LIVE_SUMO_CLASSES:
                    self.assertIn(name, output.getvalue())

    def test_environment_and_working_directory_are_inherited_by_all_children(self):
        with mock.patch.dict(os.environ, {"ROADNET_TEST_BUILD_DIR": "R:/build"}), \
                mock.patch.object(runner.subprocess, "run", return_value=SimpleNamespace(returncode=0)) as run:
            self.assertEqual(runner.main(["--include-sumo"]), 0)
        self.assertEqual(run.call_count, 1 + len(runner.LIVE_SUMO_CLASSES))
        for call in run.call_args_list:
            self.assertEqual(call.kwargs["cwd"], runner.ROOT)
            self.assertEqual(call.kwargs["env"]["ROADNET_TEST_BUILD_DIR"], "R:/build")
            self.assertFalse(call.kwargs["check"])

    def test_failures_preserve_first_nonzero_status_and_run_all_phases(self):
        for statuses, expected in (([7, 0, 0], 7), ([0, 3, 0], 3),
                                   ([0, 0, 4], 4), ([7, 3, 4], 7)):
            with self.subTest(statuses=statuses), mock.patch.object(
                    runner.subprocess, "run",
                    side_effect=[SimpleNamespace(returncode=value) for value in statuses]) as run:
                self.assertEqual(runner.main(["--include-sumo"]), expected)
                self.assertEqual(run.call_count, 1 + len(runner.LIVE_SUMO_CLASSES))


if __name__ == "__main__":
    unittest.main()
