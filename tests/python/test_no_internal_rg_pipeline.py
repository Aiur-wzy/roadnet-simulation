"""No-internal topology must change crossing clocks, not outgoing signals."""
import copy
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import catching.model_catching_sumo_cap_context_abc_rg as catch
import training.model_training_sumo_cap_context_abc_rg as training
from features.signal_rg_features import network_without_internal_links
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, fixture_args, mock_predictor
import tests.python.test_model_training_sumo_cap_context_abc_rg as train_tests
import tests.python.test_traci_cap_context_abc_rg as observer_tests
import tests.python.test_traci_cap_context as cap_tests
import collection._shared.TraCI_Python_Cap_Context as shared

V4 = "static-movement-rg-segments-v4"


class NoInternalPipelineTest(unittest.TestCase):
    def setUp(self):
        self.helper = train_tests.ABCRGTrainingTests()
        self.helper.setUp()
        self.root = self.helper.root
        self.addCleanup(self.helper.doCleanups)

    def no_internal_net(self):
        _, net, contract = prepare_fixture(self.root)
        root = ET.parse(net).getroot()
        for edge in list(root.findall("edge")):
            if edge.get("id", "").startswith(":"):
                root.remove(edge)
        ET.ElementTree(root).write(net)
        return net, contract

    def test_topology_not_netconvert_comment(self):
        for xml, expected in (
                ('<net><!-- no-internal-links=true --><edge id=":a"/><edge id="a"/></net>', False),
                ('<net><edge id="a"/></net>', True),
                ('<net><edge id="a"/><edge id=":a"><lane id=":a_0"/></edge><connection via=":a_0"/></net>', False)):
            self.assertEqual(network_without_internal_links(ET.fromstring(xml)), expected)
        for xml in ('<net/>', '<routes><edge id="a"/></routes>',
                    '<net><edge id="a"/><connection via=":missing"/></net>',
                    '<net><edge id="a"/><junction intLanes=":missing"/></net>'):
            with self.assertRaises(ValueError):
                network_without_internal_links(ET.fromstring(xml))

    def test_v4_collection_audit_requires_zero_even_controlled_incoming(self):
        signal = self.helper.signal_contract()
        signal.update(training.expected_signal_contract(version=V4))
        signal["runtime_network_without_internal_links"] = True
        def zero(frame):
            frame["incoming_crossing_seconds"] = 0.
            frame["planned_enter_time"] = frame["decision_time"]
            frame["signal_reference_time"] = frame["decision_time"]
            frame["signal_projection_seconds"] = 0.
        frame = self.helper.load(self.helper.write("zero.csv", signal=signal, audit_update=zero))
        fitted = self.helper.fake_fit(frame, frame.copy())
        self.assertEqual(fitted["contract"]["signal_contract"]["version"], V4)
        self.assertNotIn("runtime_network_without_internal_links", fitted["contract"]["signal_contract"])
        with self.assertRaisesRegex(ValueError, "incoming crossing"):
            self.helper.load(self.helper.write("wrong.csv", signal=signal))
        for flag in (None, 1, "true"):
            broken = copy.deepcopy(signal)
            broken["runtime_network_without_internal_links"] = flag
            with self.assertRaisesRegex(ValueError, "boolean"):
                training.validate_signal_contract(broken)

    def test_v4_has_internal_audit_retains_crossing(self):
        signal = self.helper.signal_contract()
        signal.update(training.expected_signal_contract(version=V4))
        signal["runtime_network_without_internal_links"] = False
        self.assertEqual(len(self.helper.load(self.helper.write("internal.csv", signal=signal))), 6)

    def test_legacy_export_refuses_positive_crossing_for_no_internal_net(self):
        net, contract = self.no_internal_net()
        with self.assertRaisesRegex(ValueError, "no-internal-links"):
            catch.build_plan(fixture_args(self.root))
        contract["context_contract"]["base_context_contract"]["crossing_time_seconds"] = 0.
        catch.validate_network_crossing_contract(net, contract)

    def test_mock_v4_export_preserves_red_green_and_steps(self):
        _, contract = self.no_internal_net()
        contract["signal_contract"] = training.expected_signal_contract(version=V4)
        (self.root / "model/training_contract.json").write_text(json.dumps(contract))
        for step in (1, 5):
            args = fixture_args(self.root, "--signal-remaining-step", str(step),
                                "--output-dir", str(self.root / f"export{step}"),
                                "--max-road-flow", "2", "--max-lane-flow", "1", "--cap-min", "8", "--cap-max", "8")
            plan = catch.build_plan(args)
            catch.export_table(args, plan, mock_predictor())
            header, values = catch.read_table(Path(args.output_dir) / "table.rle")
            self.assertEqual(header["signal_contract"]["version"], V4)
            self.assertEqual(len(values), plan["expanded_rows"])
            self.assertTrue(any(item["entry_type"] == "R" for item in header["signal_catalog"]))

    def test_live_observer_zero_crossing_does_not_turn_controlled_signal_green(self):
        helper = observer_tests.ABCRGObserverTest()
        helper.setUp()
        self.addCleanup(helper.doCleanups)
        net = helper.root / "net.xml"
        root = ET.parse(net).getroot()
        for edge in list(root.findall("edge")):
            if edge.get("id", "").startswith(":"):
                root.remove(edge)
        ET.ElementTree(root).write(net)
        observer = helper.make_observer()
        helper.api.step(20., {"ego": ("a", 0)})
        observer.on_step()
        helper.api.step(23., {"ego": ("b", 1)})
        observer.on_step()
        row = observer.frozen[("ego", 1)]
        self.assertEqual(row["incoming_junction_status"], "signalized_node")
        self.assertEqual(row["incoming_crossing_seconds"], 0.)
        self.assertEqual(row["planned_enter_time"], 23.)
        self.assertEqual(row["signal_status"], "controlled_static")
        self.assertEqual(row["entry_signal_type"], "R")
        self.assertEqual(observer.feature_signal_contract()["version"], V4)

    def test_collector_uses_sumo_default_not_unrelated_car(self):
        helper = cap_tests.TraCICapContextTest()
        helper.setUp()
        self.addCleanup(helper.doCleanups)
        observer = helper.observer
        del observer.metadata.vehicle_options["ego"]["type"]
        # The existing named car remains explicitly capped at 10; no type
        # means DEFAULT_VEHTYPE, never borrowing that car's settings.
        helper.api.vehicle.getTypeID = lambda _: "DEFAULT_VEHTYPE"
        helper.api.vehicle.getMaxSpeed = lambda _: 200. / 3.6
        values = observer._vehicle_data("ego")
        self.assertEqual(values["vehicle_type"], "DEFAULT_VEHTYPE")
        self.assertEqual(values["vehicle_max_speed"], 200. / 3.6)

    def test_collector_honors_default_type_override(self):
        helper = cap_tests.TraCICapContextTest()
        helper.setUp()
        self.addCleanup(helper.doCleanups)
        observer = helper.observer
        del observer.metadata.vehicle_options["ego"]["type"]
        observer.metadata.type_options["DEFAULT_VEHTYPE"] = {"maxSpeed": "20", "desiredMaxSpeed": "15"}
        helper.api.vehicle.getTypeID = lambda _: "DEFAULT_VEHTYPE"
        helper.api.vehicle.getMaxSpeed = lambda _: 20.
        values = observer._vehicle_data("ego")
        self.assertEqual((values["vehicle_max_speed"], values["desired_max_speed"]), (20., 15.))

    def test_collector_class_defaults_and_unsupported_are_explicit(self):
        self.assertEqual(shared.sumo_type_defaults("bus")["maxSpeed"], 100. / 3.6)
        self.assertEqual(shared.sumo_type_defaults("bicycle")["desiredMaxSpeed"], 20. / 3.6)
        with self.assertRaisesRegex(ValueError, "unsupported"):
            shared.sumo_type_defaults("future_unknown_class")


if __name__ == "__main__":
    unittest.main()
