"""Execute generated launchers with an argument recorder; never run SUMO."""

import importlib
import os
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace
import unittest
import uuid


DEFAULT_BASH = r"C:\Program Files\Git\bin\bash.exe" if os.name == "nt" else "bash"
BASH = os.environ.get("BASH_BIN", DEFAULT_BASH)
GENERATORS = (
    "data_generation.generate_balanced_training_data",
    "legacy.data_generation.generate_balanced_training_data",
)


def shell_path(path):
    value = path.resolve().as_posix()
    if os.name == "nt" and len(value) > 1 and value[1] == ":":
        return "/" + value[0].lower() + value[2:]
    return value


@unittest.skipUnless(shutil.which(BASH), "Bash is required for generated launcher checks")
class GeneratedCollectionLauncherTests(unittest.TestCase):
    def setUp(self):
        self.root = Path.cwd() / ("generated-collection-launcher-test-" + uuid.uuid4().hex)
        self.root.mkdir()
        self.addCleanup(self.cleanup_fixture)

    def cleanup_fixture(self):
        target = self.root.resolve()
        if target.parent != Path.cwd().resolve() or not target.name.startswith("generated-collection-launcher-test-"):
            raise RuntimeError("Refusing cleanup outside the test workspace")
        shutil.rmtree(target)

    def run_launcher(self, module_name, *, collector="", binary="sumo", outputs="legacy", overrides=None):
        generator = importlib.import_module(module_name)
        dataset = self.root / (uuid.uuid4().hex + " dataset with spaces")
        commands = dataset / "commands"
        commands.mkdir(parents=True)
        script = commands / "run_traci.sh"
        jobs = commands / "jobs.tsv"
        net = dataset / "inputs" / "mock net.xml"
        jobs.write_text(
            "episode_id\tsplit\tsumocfg_file\tlegacy_training_output\tfull_training_output\t"
            "vehicle_event_output\tcycle_summary_output\trun_log\n"
            "mock\ttrain\tconfigs/mock.sumocfg\tresults/legacy training.csv\tresults/full.csv\t"
            "results/events.csv\tresults/cycles.csv\tlogs/collector run.log\n",
            encoding="utf-8",
        )
        generator.write_run_script(
            script, SimpleNamespace(force=False, traci_script=collector,
                                    sumo_binary=binary, traci_outputs=outputs),
            dataset, net, jobs,
        )
        capture = dataset / "captured argv.bin"
        environment = os.environ.copy()
        for name in ("TRACI_SCRIPT", "SUMO_BINARY", "TRACI_OUTPUTS", "ROADNET_ROOT"):
            environment.pop(name, None)
        environment.update({"CAPTURE": capture.as_posix(), "EXPAND_ME": "unexpected expansion"})
        environment.update(overrides or {})
        # This function replaces python3 for the sourced launcher. It records every
        # argument with a NUL separator, including any erroneous empty/quote argument.
        recorder = r'''python3() { printf '%s\0' "$@" > "$CAPTURE"; }; source "$1"'''
        result = subprocess.run(
            [BASH, "-c", recorder, "generated-launcher-test", str(script)],
            cwd=self.root, env=environment, capture_output=True,
            text=True, encoding="utf-8", errors="replace", timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(capture.is_file(), result.stdout)
        actual = capture.read_bytes().decode("utf-8").split("\0")
        self.assertEqual(actual.pop(), "")
        selected = {"TRACI_SCRIPT": collector, "SUMO_BINARY": binary, "TRACI_OUTPUTS": outputs}
        for name, value in (overrides or {}).items():
            if name in selected and value:
                selected[name] = value
        expected = ([selected["TRACI_SCRIPT"]] if selected["TRACI_SCRIPT"] else
                    ["-m", "collection._shared.TraCI_Python_Adjusted"])
        expected += [
            "--sumo-config", shell_path(dataset / "configs/mock.sumocfg"),
            "--net-file", shell_path(net),
            "--sumo-binary", selected["SUMO_BINARY"],
            "--outputs", selected["TRACI_OUTPUTS"],
            "--legacy-edge-output", shell_path(dataset / "results/legacy training.csv"),
            "--training-output", shell_path(dataset / "results/full.csv"),
            "--vehicle-event-output", shell_path(dataset / "results/events.csv"),
            "--cycle-summary-output", shell_path(dataset / "results/cycles.csv"),
        ]
        self.assertEqual(actual, expected)
        self.assertNotIn("''", actual)
        return script.read_text(encoding="utf-8")

    def test_default_dispatches_module_and_all_arguments_exactly(self):
        for module_name in GENERATORS:
            with self.subTest(generator=module_name):
                script = self.run_launcher(module_name)
                self.assertIn("DEFAULT_TRACI_SCRIPT=''\n", script)
                self.assertIn('TRACI_SCRIPT="${TRACI_SCRIPT:-$DEFAULT_TRACI_SCRIPT}"', script)

    def test_custom_defaults_preserve_spaces_quotes_and_shell_metacharacters(self):
        for module_name in GENERATORS:
            with self.subTest(generator=module_name):
                self.run_launcher(
                    module_name,
                    collector="custom collector's ${EXPAND_ME} $(printf expanded) `printf expanded`.py",
                    binary="sumo binary's ${EXPAND_ME} $(printf expanded)",
                    outputs="output's ${EXPAND_ME} `printf expanded`",
                )

    def test_environment_overrides_remain_single_literal_arguments(self):
        for module_name in GENERATORS:
            with self.subTest(generator=module_name):
                self.run_launcher(
                    module_name, collector="unused collector.py", binary="unused sumo", outputs="unused",
                    overrides={"TRACI_SCRIPT": "environment collector's path.py",
                               "SUMO_BINARY": "environment sumo's path", "TRACI_OUTPUTS": "all"},
                )

    def test_empty_environment_values_use_generated_defaults(self):
        for module_name in GENERATORS:
            with self.subTest(generator=module_name):
                self.run_launcher(module_name, overrides={"TRACI_SCRIPT": "", "SUMO_BINARY": "", "TRACI_OUTPUTS": ""})


if __name__ == "__main__":
    unittest.main()
