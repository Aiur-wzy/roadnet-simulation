"""Isolated GPU-probe regressions; neither AutoGluon nor GPU fitting is invoked."""

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import sys
import types
import unittest
from unittest import mock
import uuid

import numpy as np
import pandas as pd

SCRIPTS_DIR = str(Path(__file__).resolve().parent)
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import tools.environment.check_autogluon_gpu as check


def load_helper_with_stock_stubs():
    """Import observation helpers without importing the actual AutoGluon package."""
    modules = {}
    for name in ("autogluon", "autogluon.tabular", "autogluon.tabular.models",
                 "autogluon.tabular.models.catboost", "autogluon.tabular.models.xgboost",
                 "autogluon.tabular.models.lgb"):
        modules[name] = types.ModuleType(name)
        modules[name].__path__ = []

    class StockModel:
        def _fit(self, X, y, **kwargs):
            self.stock_calls.append((X, y, kwargs))
            if getattr(self, "stock_error", False):
                raise RuntimeError("stock learner failed")
            self.model = self.fitted_native
            self.stock_finished = True
            return "stock-result"

    for suffix, name in (("catboost.catboost_model", "CatBoostModel"),
                         ("xgboost.xgboost_model", "XGBoostModel"),
                         ("lgb.lgb_model", "LGBModel")):
        module_name = "autogluon.tabular.models." + suffix
        module = types.ModuleType(module_name)
        setattr(module, name, StockModel)
        modules[module_name] = module
    path = Path(check.__file__).with_name("ag_gpu_probe_models.py")
    spec = importlib.util.spec_from_file_location("gpu_probe_fixture_" + uuid.uuid4().hex, path)
    helper = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, modules):
        spec.loader.exec_module(helper)
    return helper


class GPUProbeTests(unittest.TestCase):
    def setUp(self):
        self.directory = Path.cwd() / ("gpu-probe-test-" + uuid.uuid4().hex)
        self.directory.mkdir()
        self.addCleanup(self.cleanup_fixture)

    def cleanup_fixture(self):
        target = self.directory.resolve()
        if target.parent != Path.cwd().resolve() or not target.name.startswith("gpu-probe-test-"):
            raise RuntimeError("Refusing to clean a directory outside this test workspace")
        shutil.rmtree(target)

    def args(self, *arguments):
        return check.build_parser().parse_args(list(map(str, arguments)))

    def evidence(self, family="CAT", status="GPU_PASS", **changes):
        device = "cpu" if status == "CPU_FALLBACK" else ("gpu" if family == "CAT" else "cuda:0")
        details = {"task_type": "CPU" if device == "cpu" else "GPU"} if family == "CAT" else {"device": device}
        result = {"schema": "autogluon-live-gpu-fit-evidence-v1", "status": status,
                  "family": family, "device": device, "native_details": details,
                  "captured_before_save": True, "sample_weight_received": True,
                  "weight_count": 12, "rows": 12, "weight_min": 0.5,
                  "weight_max": 1.5, "weight_mean": 1.0}
        result.update(changes)
        return result

    def write_leaf(self, name, evidence=None):
        path = self.directory / name
        path.mkdir(parents=True)
        if evidence is not None:
            item = dict(evidence, model_path=str(path.resolve()))
            check.write_json(path / check.EVIDENCE_FILE, item)
        return types.SimpleNamespace(path=str(path))

    def predictor(self, models):
        loads = []

        def load_model(name):
            loads.append(name)
            return models[name]

        return types.SimpleNamespace(model_names=lambda: list(models),
                                     _trainer=types.SimpleNamespace(load_model=load_model)), loads

    def test_default_scope_is_small_synthetic_and_stacking_is_optional(self):
        args = self.args()
        self.assertEqual(args.rows, 2000)
        self.assertEqual(args.rounds, 20)
        self.assertEqual(args.families, ["CAT", "XGB", "GBM"])
        self.assertEqual(args.stages, ["single", "bagged"])
        self.assertNotIn("stacked", args.stages)
        self.assertEqual(check.AUDITED_VERSION, "1.6.1")

    def test_synthetic_data_exact_features_strict_boundary_and_weight_mean(self):
        frame = check.make_data()
        self.assertEqual(len(frame), 2000)
        self.assertEqual(list(frame.columns), check.FEATURES + [check.LABEL, check.WEIGHT])
        self.assertEqual(frame.road_flow.iloc[:3].tolist(), [4, 5, 6])
        self.assertAlmostEqual(frame[check.WEIGHT].mean(), 1.0)
        self.assertAlmostEqual(frame[check.WEIGHT].iloc[0] / frame[check.WEIGHT].iloc[1], 3.0)
        self.assertAlmostEqual(frame[check.WEIGHT].iloc[1] / frame[check.WEIGHT].iloc[2], 1.0)
        self.assertTrue((frame.lane_flow <= frame.road_flow).all())
        self.assertTrue(np.isfinite(frame.to_numpy()).all())
        self.assertNotIn(check.WEIGHT, check.FEATURES)
        pd.testing.assert_frame_equal(frame, check.make_data())
        self.assertFalse(frame.equals(check.make_data(seed=43)))

    def test_case_resource_arguments_are_explicit_at_both_levels(self):
        for family in ("CAT", "XGB", "GBM"):
            for stage, folds, stacks in (("single", 0, 0), ("bagged", 2, 0), ("stacked", 2, 1)):
                with self.subTest(family=family, stage=stage):
                    params, options = check.case_parameters(family, stage, self.args())
                    self.assertEqual(options["num_gpus"], 1)
                    self.assertEqual(options["ag_args_fit"]["num_gpus"], 1)
                    self.assertEqual(options["num_bag_folds"], folds)
                    self.assertEqual(options["num_stack_levels"], stacks)
                    self.assertEqual(options["num_bag_sets"], 1)
                    self.assertEqual(options["fit_strategy"], "sequential")
                    self.assertIs(options["dynamic_stacking"], False)
                    self.assertIs(options["fit_weighted_ensemble"], False)
                    if folds:
                        self.assertEqual(options["ag_args_ensemble"]["num_folds_parallel"], 2)
                    else:
                        self.assertNotIn("ag_args_ensemble", options)
                    if family == "CAT":
                        self.assertEqual(params["devices"], "0")

    def test_lgb_device_ids_are_explicit_and_not_assumed_cuda_ids(self):
        params, options = check.case_parameters("GBM", "bagged", self.args(
            "--lgb-gpu-platform-id", 1, "--lgb-gpu-device-id", 2,
            "--fold-strategy", "sequential_local"))
        self.assertEqual(params["gpu_platform_id"], 1)
        self.assertEqual(params["gpu_device_id"], 2)
        self.assertEqual(options["ag_args_ensemble"]["fold_fitting_strategy"], "sequential_local")

    def test_classification_requires_complete_successful_model_counts(self):
        for stage, names, leaves in (("single", ["one"], 1),
                                     ("bagged", ["one"], 2),
                                     ("stacked", ["one", "two"], 4)):
            with self.subTest(stage=stage):
                evidence = [self.evidence() for _ in range(leaves)]
                self.assertEqual(check.classify_case(stage, "CAT", names, evidence), "GPU_PASS")
                self.assertEqual(check.classify_case(stage, "CAT", names, evidence[:-1]), "INCOMPLETE")
                self.assertEqual(check.classify_case(stage, "CAT", names + ["extra"], evidence), "INCOMPLETE")
        self.assertEqual(check.classify_case("single", "CAT", [], []), "INCOMPLETE")

    def test_gpu_status_alone_is_insufficient_without_native_or_weight_evidence(self):
        variants = [
            {"status": "UNKNOWN"}, {"family": "GBM"}, {"captured_before_save": False},
            {"sample_weight_received": False}, {"weight_count": 11}, {"rows": 0},
            {"weight_min": None}, {"weight_min": 0}, {"device": ""},
            {"device": "cpu"}, {"native_details": None},
            {"weight_min": float("nan")}, {"weight_max": float("inf")},
            {"weight_min": 2.0, "weight_max": 1.0},
            {"native_details": {"task_type": "CPU"}},
        ]
        for change in variants:
            with self.subTest(change=change):
                result = check.classify_case("single", "CAT", ["one"], [self.evidence(**change)])
                self.assertNotEqual(result, "GPU_PASS")

    def test_mixed_gpu_cpu_children_are_not_gpu_pass(self):
        evidence = [self.evidence(), self.evidence(status="CPU_FALLBACK")]
        self.assertEqual(check.classify_case("bagged", "CAT", ["bag"], evidence), "CPU_FALLBACK")

    def test_evidence_loader_only_visits_successful_models_not_failed_artifacts(self):
        live = self.write_leaf("success", self.evidence())
        self.write_leaf("failed-unused", self.evidence(status="CPU_FALLBACK"))
        predictor, loads = self.predictor({"success": live})
        names, evidence = check.read_successful_evidence(predictor)
        self.assertEqual(names, ["success"])
        self.assertEqual(loads, ["success"])
        self.assertEqual(len(evidence), 1)
        self.assertEqual(evidence[0]["status"], "GPU_PASS")
        self.assertNotIn("failed-unused", evidence[0]["evidence_path"])

    def test_bagged_children_are_loaded_from_model_membership_and_deduplicated(self):
        children = {"S1F1": self.write_leaf("bag/S1F1", self.evidence()),
                    "S1F2": self.write_leaf("bag/S1F2", self.evidence())}
        self.write_leaf("bag/failed-S1F3", self.evidence(status="CPU_FALLBACK"))
        loaded = []

        def load_child(name):
            loaded.append(name)
            return children[name]

        bag = types.SimpleNamespace(models=list(children), load_child=load_child)
        predictor, _ = self.predictor({"bag": bag, "alias": children["S1F1"]})
        _, evidence = check.read_successful_evidence(predictor)
        self.assertEqual(loaded, ["S1F1", "S1F2"])
        self.assertEqual(len(evidence), 2)

    def test_missing_mismatched_or_empty_child_evidence_is_not_success(self):
        missing = self.write_leaf("missing")
        mismatch = self.write_leaf("mismatch", self.evidence())
        path = Path(mismatch.path) / check.EVIDENCE_FILE
        data = json.loads(path.read_text(encoding="utf-8"))
        data["model_path"] = str(self.directory / "different")
        check.write_json(path, data)
        empty_bag = types.SimpleNamespace(models=[], load_child=lambda name: None)
        predictor, _ = self.predictor({"missing": missing, "mismatch": mismatch, "empty": empty_bag})
        _, evidence = check.read_successful_evidence(predictor)
        self.assertEqual(len(evidence), 3)
        self.assertTrue(all(item["status"] == "UNKNOWN" for item in evidence))

    def fake_predictor_class(self, evidence, captured, leaked=False, prediction_error=False):
        class FakePredictor:
            def __init__(self, **kwargs):
                captured["constructor"] = kwargs
                self.path = Path(kwargs["path"])
                columns = check.FEATURES + ([check.WEIGHT] if leaked else [])
                self.feature_metadata = types.SimpleNamespace(get_features=lambda: columns)

            def fit(self, **kwargs):
                captured["fit"] = kwargs
                path = self.path / "models" / "success"
                path.mkdir(parents=True)
                if evidence is not None:
                    check.write_json(path / check.EVIDENCE_FILE,
                                     dict(evidence, model_path=str(path.resolve())))
                model = types.SimpleNamespace(path=str(path))
                self._trainer = types.SimpleNamespace(load_model=lambda name: model)
                return self

            def model_names(self):
                return ["success"]

            def predict(self, frame):
                captured["prediction_input"] = frame.copy()
                return np.full(len(frame), np.nan if prediction_error else 1.0)

        return FakePredictor

    def test_run_case_gpu_cpu_missing_evidence_and_predict_five_features(self):
        train, heldout = check.make_data(200), check.make_data(200, 43)
        for index, (evidence, expected) in enumerate(((self.evidence(), "GPU_PASS"),
                                                    (self.evidence(status="CPU_FALLBACK"), "CPU_FALLBACK"),
                                                    (None, "UNKNOWN"))):
            with self.subTest(expected=expected), contextlib.redirect_stdout(io.StringIO()):
                output = self.directory / f"run-{index}"
                output.mkdir()
                captured = {}
                predictor = self.fake_predictor_class(evidence, captured)
                result = check.run_case("CAT", "single", self.args(), output, train, heldout,
                                        predictor, type("DummyCAT", (), {}))
                self.assertEqual(result["status"], expected)
                self.assertEqual(captured["constructor"]["sample_weight"], check.WEIGHT)
                self.assertIs(captured["constructor"]["weight_evaluation"], True)
                pd.testing.assert_frame_equal(captured["fit"]["train_data"], train)
                self.assertNotIn("tuning_data", captured["fit"])
                self.assertNotIn("validation_data", captured["fit"])
                self.assertEqual(list(captured["prediction_input"].columns), check.FEATURES)
                report = json.loads((output / "cat_single" / "case_result.json").read_text(encoding="utf-8"))
                self.assertEqual(report["status"], expected)

    def test_run_case_rejects_feature_leakage_and_nonfinite_predictions(self):
        for index, (leaked, invalid) in enumerate(((True, False), (False, True))):
            with self.subTest(leaked=leaked, invalid=invalid), contextlib.redirect_stdout(io.StringIO()):
                output = self.directory / f"bad-{index}"
                output.mkdir()
                result = check.run_case("CAT", "single", self.args(), output,
                    check.make_data(200), check.make_data(200, 43),
                    self.fake_predictor_class(self.evidence(), {}, leaked, invalid), object)
                self.assertEqual(result["status"], "ERROR")

    def test_missing_or_wrong_autogluon_version_is_blocked_without_fitting(self):
        for index, version in enumerate((None, "1.5.0", "1.7.0")):
            with self.subTest(version=version):
                output = self.directory / f"blocked-{index}"
                with mock.patch.object(check, "collect_inventory", return_value={"versions": {"autogluon.tabular": version}}), \
                        mock.patch.object(check, "run_case", side_effect=AssertionError("must not fit")) as run, \
                        mock.patch.dict(sys.modules, {"autogluon": None, "autogluon.tabular": None}), \
                        contextlib.redirect_stdout(io.StringIO()):
                    status = check.main(["--output-dir", str(output)])
                self.assertEqual(status, 2)
                run.assert_not_called()
                report = json.loads((output / "gpu_report.json").read_text(encoding="utf-8"))
                self.assertEqual(report["status"], "ENVIRONMENT_BLOCKED")
                self.assertEqual(report["cases"], [])

    def test_inventory_only_does_not_claim_gpu_pass(self):
        output = self.directory / "inventory"
        with mock.patch.object(check, "collect_inventory", return_value={"versions": {}}), \
                mock.patch.object(check, "run_case", side_effect=AssertionError("must not fit")), \
                contextlib.redirect_stdout(io.StringIO()):
            status = check.main(["--output-dir", str(output), "--inventory-only"])
        self.assertEqual(status, 0)
        report = json.loads((output / "gpu_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["status"], "INVENTORY_ONLY_NOT_VERIFIED")
        self.assertEqual(report["cases"], [])

    def fake_runtime_modules(self, gpu_count):
        modules = {}
        for name in ("autogluon", "autogluon.tabular", "autogluon.common",
                     "autogluon.common.utils", "autogluon.common.utils.resource_utils",
                     'tools.environment.ag_gpu_probe_models'):
            modules[name] = types.ModuleType(name)
            modules[name].__path__ = []
        modules["autogluon.tabular"].TabularPredictor = mock.Mock(
            side_effect=AssertionError("real predictor construction is forbidden"))
        # Match the pinned AutoGluon 1.6.1 API; an unrestricted mock would also
        # accept nonexistent methods and hide preflight compatibility failures.
        class ResourceManagerAPI:
            @staticmethod
            def get_gpu_count():
                return gpu_count

        manager = mock.Mock(spec_set=ResourceManagerAPI)
        manager.get_gpu_count.return_value = gpu_count
        modules["autogluon.common.utils.resource_utils"].ResourceManager = manager
        for name in ("ProbeCatBoostModel", "ProbeXGBoostModel", "ProbeLGBModel"):
            setattr(modules['tools.environment.ag_gpu_probe_models'], name, type(name, (), {}))
        return modules

    def test_autogluon_161_resource_api_reaches_fit_but_count_is_not_gpu_proof(self):
        output = self.directory / "resource-api"
        modules = self.fake_runtime_modules(1)
        manager = modules["autogluon.common.utils.resource_utils"].ResourceManager
        self.assertFalse(hasattr(manager, "get_gpu_count_all"))
        with mock.patch.dict(sys.modules, modules), \
                mock.patch.dict(os.environ), \
                mock.patch.object(check, "collect_inventory", return_value={"versions": {"autogluon.tabular": "1.6.1"}}), \
                mock.patch.object(check, "run_case", return_value={"status": "UNKNOWN"}) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            status = check.main(["--output-dir", str(output), "--rows", "200",
                                 "--families", "CAT", "--stages", "single"])
        manager.get_gpu_count.assert_called_once_with()
        run.assert_called_once()
        self.assertEqual(status, 1)
        report = json.loads((output / "gpu_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["autogluon_detected_gpu_count"], 1)
        self.assertEqual(report["autogluon_gpu_count_api"], "ResourceManager.get_gpu_count")
        self.assertEqual(report["status"], "NOT_ALL_GPU_CASES_PASSED")

    def test_resource_api_failure_is_recorded_without_fit_or_inventory_fallback(self):
        output = self.directory / "resource-api-error"
        modules = self.fake_runtime_modules(1)
        manager = modules["autogluon.common.utils.resource_utils"].ResourceManager
        manager.get_gpu_count.side_effect = RuntimeError("resource query failed")
        with mock.patch.dict(sys.modules, modules), \
                mock.patch.dict(os.environ), \
                mock.patch.object(check, "collect_inventory", return_value={
                    "versions": {"autogluon.tabular": "1.6.1"}, "torch_cuda_device_count": 1}), \
                mock.patch.object(check, "run_case", side_effect=AssertionError("must not fit")) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            status = check.main(["--output-dir", str(output)])
        manager.get_gpu_count.assert_called_once_with()
        run.assert_not_called()
        self.assertEqual(status, 2)
        report = json.loads((output / "gpu_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["status"], "ENVIRONMENT_BLOCKED")
        self.assertEqual(report["error"], "RuntimeError: resource query failed")
        self.assertEqual(report["autogluon_gpu_count_api"], "ResourceManager.get_gpu_count")
        self.assertNotIn("autogluon_detected_gpu_count", report)
        self.assertEqual(report["cases"], [])

    def test_autogluon_zero_detected_gpus_blocks_without_smoke_fit(self):
        output = self.directory / "zero-gpus"
        with mock.patch.dict(sys.modules, self.fake_runtime_modules(0)), \
                mock.patch.dict(os.environ), \
                mock.patch.object(check, "collect_inventory", return_value={"versions": {"autogluon.tabular": "1.6.1"}}), \
                mock.patch.object(check, "run_case", side_effect=AssertionError("must not fit")) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            status = check.main(["--output-dir", str(output)])
        self.assertEqual(status, 2)
        run.assert_not_called()
        report = json.loads((output / "gpu_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["status"], "ENVIRONMENT_BLOCKED")
        self.assertEqual(report["autogluon_detected_gpu_count"], 0)

    def test_main_exit_code_tracks_proved_gpu_status_and_only_synthetic_data(self):
        for index, case_status in enumerate(("GPU_PASS", "CPU_FALLBACK", "UNKNOWN", "ERROR")):
            with self.subTest(case_status=case_status):
                output = self.directory / f"main-{index}"
                with mock.patch.dict(sys.modules, self.fake_runtime_modules(1)), \
                        mock.patch.dict(os.environ), \
                        mock.patch.object(check, "collect_inventory", return_value={"versions": {"autogluon.tabular": "1.6.1"}}), \
                        mock.patch.object(check, "run_case", return_value={"status": case_status}) as run, \
                        contextlib.redirect_stdout(io.StringIO()):
                    status = check.main(["--output-dir", str(output), "--rows", "200",
                                         "--families", "CAT", "--stages", "single"])
                self.assertEqual(status, 0 if case_status == "GPU_PASS" else 1)
                run.assert_called_once()
                train, heldout = run.call_args.args[4:6]
                self.assertEqual((len(train), len(heldout)), (200, 400))
                self.assertEqual(list(train.columns), check.FEATURES + [check.LABEL, check.WEIGHT])
                self.assertAlmostEqual(train[check.WEIGHT].mean(), 1.0)
                report = json.loads((output / "gpu_report.json").read_text(encoding="utf-8"))
                expected = "ALL_REQUESTED_GPU_CASES_PASS" if case_status == "GPU_PASS" else "NOT_ALL_GPU_CASES_PASSED"
                self.assertEqual(report["status"], expected)
                self.assertIn("synthetic only", report["data"])

    def test_existing_output_and_invalid_scope_fail_before_environment_or_fit(self):
        existing = self.directory / "existing"
        existing.mkdir()
        (existing / "keep.txt").write_text("preserve", encoding="utf-8")
        variants = [[], ["--rows", "199"], ["--rows", "10001"], ["--rounds", "101"],
                    ["--time-limit", "0"], ["--num-cpus", "0"],
                    ["--families", "CAT", "CAT"], ["--stages", "single", "single"],
                    ["--lgb-gpu-platform-id", "-1"]]
        for options in variants:
            with self.subTest(options=options), \
                    mock.patch.object(check, "collect_inventory", side_effect=AssertionError("must not inspect")), \
                    self.assertRaises(ValueError):
                check.main(["--output-dir", str(existing), *options])
        self.assertEqual((existing / "keep.txt").read_text(encoding="utf-8"), "preserve")


class ProbeHelperTests(unittest.TestCase):
    setUp = GPUProbeTests.setUp
    cleanup_fixture = GPUProbeTests.cleanup_fixture

    @classmethod
    def setUpClass(cls):
        cls.helper = load_helper_with_stock_stubs()

    def test_native_cat_uses_fitted_all_params_gpu_cpu_unknown(self):
        for task, expected in (("GPU", "GPU_PASS"), ("CPU", "CPU_FALLBACK"), (None, "UNKNOWN")):
            with self.subTest(task=task):
                native = types.SimpleNamespace(get_all_params=lambda: {"task_type": task},
                                               get_params=lambda: {"task_type": "GPU"})
                self.assertEqual(self.helper.native_device_evidence("CAT", native)["status"], expected)

    def test_native_xgb_uses_live_save_config_not_requested_get_params(self):
        for device, expected in (("cuda:0", "GPU_PASS"), ("cuda", "GPU_PASS"),
                                 ("cpu", "CPU_FALLBACK"), (None, "UNKNOWN")):
            with self.subTest(device=device):
                config = json.dumps({"learner": {"generic_param": {"device": device}}})
                save_config = mock.Mock(return_value=config)
                native = types.SimpleNamespace(get_booster=lambda: types.SimpleNamespace(save_config=save_config),
                                               get_params=mock.Mock(return_value={"device": "cuda"}))
                result = self.helper.native_device_evidence("XGB", native)
                self.assertEqual(result["status"], expected)
                save_config.assert_called_once_with()
                native.get_params.assert_not_called()

    def test_native_lgb_uses_fitted_booster_device_and_missing_evidence_unknown(self):
        for params, expected in (({"device_type": "gpu"}, "GPU_PASS"),
                                 ({"device": "cuda"}, "GPU_PASS"),
                                 ({"device": "cpu"}, "CPU_FALLBACK"), ({}, "UNKNOWN")):
            with self.subTest(params=params):
                native = types.SimpleNamespace(params=params)
                self.assertEqual(self.helper.native_device_evidence("GBM", native)["status"], expected)
        for family in ("CAT", "XGB", "GBM", "unsupported"):
            with self.subTest(family=family):
                self.assertEqual(self.helper.native_device_evidence(family, object())["status"], "UNKNOWN")

    def model(self, name="model", stock_error=False):
        model = self.helper.ProbeCatBoostModel()
        model.name = name
        model.path = str(self.directory / name)
        model.stock_calls = []
        model.stock_error = stock_error
        model.fitted_native = types.SimpleNamespace(get_all_params=lambda: {"task_type": "GPU"})
        return model

    def test_mixin_delegates_stock_fit_before_capturing_and_preserves_result(self):
        model = self.model()
        frame = check.make_data(200)
        features, label, weights = frame[check.FEATURES], frame[check.LABEL], frame[check.WEIGHT]
        original = self.helper.native_device_evidence

        def after_stock(family, native):
            self.assertTrue(model.stock_finished)
            self.assertIs(native, model.fitted_native)
            self.assertFalse((Path(model.path) / check.EVIDENCE_FILE).exists())
            return original(family, native)

        with mock.patch.object(self.helper, "native_device_evidence", side_effect=after_stock):
            result = model._fit(X=features, y=label, sample_weight=weights, num_gpus=1)
        self.assertEqual(result, "stock-result")
        self.assertEqual(len(model.stock_calls), 1)
        self.assertIs(model.stock_calls[0][0], features)
        self.assertIs(model.stock_calls[0][2]["sample_weight"], weights)
        evidence = json.loads((Path(model.path) / check.EVIDENCE_FILE).read_text(encoding="utf-8"))
        self.assertEqual(evidence["status"], "GPU_PASS")
        self.assertTrue(evidence["captured_before_save"])
        self.assertEqual(evidence["weight_count"], 200)
        self.assertEqual(evidence["requested_num_gpus"], 1)
        self.assertEqual(evidence["input_columns"], check.FEATURES)
        self.assertAlmostEqual(evidence["weight_mean"], 1.0)

    def test_stock_fit_failure_does_not_write_success_evidence(self):
        model = self.model(stock_error=True)
        frame = check.make_data(200)
        with mock.patch.object(self.helper, "native_device_evidence", side_effect=AssertionError("must not inspect failed fit")) as inspect:
            with self.assertRaisesRegex(RuntimeError, "stock learner failed"):
                model._fit(X=frame[check.FEATURES], y=frame[check.LABEL], sample_weight=frame[check.WEIGHT])
        inspect.assert_not_called()
        self.assertFalse((Path(model.path) / check.EVIDENCE_FILE).exists())

    def test_missing_weight_is_recorded_and_cannot_prove_weighted_gpu_case(self):
        model = self.model()
        frame = check.make_data(200)
        model._fit(X=frame[check.FEATURES], y=frame[check.LABEL], num_gpus=1)
        evidence = json.loads((Path(model.path) / check.EVIDENCE_FILE).read_text(encoding="utf-8"))
        self.assertFalse(evidence["sample_weight_received"])
        self.assertEqual(evidence["weight_count"], 0)
        self.assertIsNone(evidence["weight_min"])
        self.assertNotEqual(check.classify_case("single", "CAT", ["model"], [evidence]), "GPU_PASS")


if __name__ == "__main__":
    unittest.main()
