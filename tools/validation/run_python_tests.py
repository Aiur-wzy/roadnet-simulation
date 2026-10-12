"""Run Python regressions with live SUMO fixtures in a separate process."""

import argparse
import os
import subprocess
import sys
import unittest

from roadnet_paths import ROOT


RUNNER_MODULE = "tools.validation.run_python_tests"
LIVE_SUMO_CLASSES = (
    "tests.python.test_discharge_sumo.LiveFixtureTests",
    "tests.python.test_discharge_e1_adapter.LiveParityTests",
)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--include-sumo", action="store_true",
                        help="Also run the private live SUMO fixtures in a fresh Python process.")
    inspection = parser.add_mutually_exclusive_group()
    inspection.add_argument("--list", action="store_true",
                            help="List discovered test modules and exclusions without importing tests.")
    inspection.add_argument("--dry-run", action="store_true",
                            help="Print subprocess commands without running or importing tests.")
    parser.add_argument("--unit-worker", action="store_true", help=argparse.SUPPRESS)
    return parser


def build_plan(include_sumo=False):
    commands = [[sys.executable, "-B", "-m", RUNNER_MODULE, "--unit-worker"]]
    if include_sumo:
        commands.extend([sys.executable, "-B", "-m", "unittest", "-v", name]
                        for name in LIVE_SUMO_CLASSES)
    return commands


def unit_tests_only(suite):
    """Keep discovery failures and ordinary tests; isolate registered live classes."""
    result = unittest.TestSuite()
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            result.addTests(unit_tests_only(item))
        elif not any(item.id().startswith(name + ".") for name in LIVE_SUMO_CLASSES):
            result.addTest(item)
    return result


def run_unit_worker():
    suite = unittest.TestLoader().discover(
        str(ROOT / "tests/python"), pattern="test_*.py", top_level_dir=str(ROOT))
    result = unittest.TextTestRunner(verbosity=2).run(unit_tests_only(suite))
    return 0 if result.wasSuccessful() else 1


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.unit_worker:
        return run_unit_worker()
    if args.list:
        for path in sorted((ROOT / "tests/python").glob("test_*.py")):
            print("tests.python." + path.stem)
        print("Excluded from the unit process: " + ", ".join(LIVE_SUMO_CLASSES))
        print("Separate live SUMO process: " + ("enabled" if args.include_sumo else "disabled"))
        return 0
    commands = build_plan(args.include_sumo)
    if args.dry_run:
        print("Working directory: " + str(ROOT))
        for command in commands:
            print(subprocess.list2cmdline(command))
        print("Unit discovery excludes " + ", ".join(LIVE_SUMO_CLASSES))
        return 0
    environment = os.environ.copy()
    exit_code = 0
    for command in commands:
        completed = subprocess.run(command, cwd=ROOT, env=environment, check=False)
        if completed.returncode and not exit_code:
            exit_code = completed.returncode
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
