"""Failure/restart tests with exact mock predictions and real checkpoint files."""
import copy
import os
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from catching import catch_road_time as c
from catching import resume_store as rs
from tests.python import test_native_catching_v2 as fixtures
FakePredictor = fixtures.FakePredictor


class ResumeTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.args = fixtures.fixture_args(self.root)
        self.args.min_free_disk_gib = 0

    def plan(self):
        return c.build_plan(self.args)

    _closed_loop = fixtures.NativeCatchingTest._closed_loop

    def requests(self):
        path = self.root / "requests.csv"
        path.write_text("group_index,road_flow\n0,0\n1,1\n2,2\n3,3\n", encoding="utf-8")
        self.args.requests = str(path)
        self.args.chunk_size = 2
        self.args.requests_plan_sha256 = c.canonical_sha(self.plan())

    def test_predict_batch_resume_without_repredicting_committed(self):
        self.requests()
        calls = []
        class Break(FakePredictor):
            def predict(self, frame):
                calls.append(len(frame))
                if len(calls) == 2:
                    raise RuntimeError("synthetic interrupt")
                return super().predict(frame)
        with self.assertRaisesRegex(RuntimeError, "synthetic"):
            c.run(self.args, predictor_loader=lambda _: Break())
        db = sqlite3.connect(Path(self.args.output_dir)/"checkpoint.sqlite3")
        self.assertEqual(db.execute("SELECT COUNT(*) FROM nodes WHERE value IS NOT NULL").fetchone()[0], 2)
        db.close()
        self.args.resume = True
        calls.clear()
        class Resume(FakePredictor):
            def predict(self, frame):
                calls.append(len(frame))
                return super().predict(frame)
        result = c.run(self.args, predictor_loader=lambda _: Resume())
        self.assertEqual(result["entry_count"], 4)
        self.assertEqual(calls, [2])
        self.assertEqual(c.run(self.args, predictor_loader=lambda _: self.fail("already complete")), result)

    def test_process_kill_recovers_committed_wal_batch(self):
        self.requests()
        program = '''import sys,json,argparse,time
sys.path[:0]=json.loads(sys.argv[1])
from catching import catch_road_time as c
from tests.python.test_native_catching_v2 import FakePredictor
class Slow(FakePredictor):
    calls=0
    def predict(self,frame):
        self.calls+=1
        if self.calls==2:
            print("READY_TO_KILL",flush=True)
            time.sleep(60)
        return super().predict(frame)
c.run(argparse.Namespace(**json.loads(sys.argv[2])),predictor_loader=lambda _:Slow())
'''
        import json
        child = subprocess.Popen([sys.executable, "-B", "-c", program, json.dumps(sys.path), json.dumps(vars(self.args))],
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), "READY_TO_KILL")
            child.kill()
            child.wait(timeout=15)
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=15)
            child.stdout.close()
        self.args.resume = True
        counts = []
        class Count(FakePredictor):
            def predict(self, frame):
                counts.append(len(frame))
                return super().predict(frame)
        result = c.run(self.args, predictor_loader=lambda _: Count())
        self.assertEqual(result["entry_count"], 4)
        self.assertEqual(counts, [2])

    def test_interrupted_replay_is_rerun_completed_replay_is_reused(self):
        calls, delegate = self._closed_loop()
        attempts = []
        def interrupt(command, log, timeout):
            attempts.append(Path(log).parent)
            if len(attempts) == 2:
                (Path(log).parent/"partial").write_text("interrupted", encoding="utf-8")
                raise RuntimeError("incomplete replay")
            return delegate(command, log, timeout)
        with self.assertRaisesRegex(RuntimeError, "incomplete"):
            c.run(self.args, predictor_loader=lambda _: FakePredictor(), runner=interrupt)
        self.args.resume = True
        result = c.run(self.args, predictor_loader=lambda _: self.fail("committed predictions must survive"), runner=delegate)
        self.assertEqual(calls, [True, True, False])
        self.assertEqual(result["combined"]["mae_seconds"], 2)
        self.assertTrue((attempts[-1]/"partial").exists())

    def test_strict_completed_replay_reused_after_summary_failure(self):
        calls, runner = self._closed_loop()
        original = rs.atomic_json
        def fail(path, data):
            if Path(path).name == "summary.json":
                raise RuntimeError("summary failure")
            return original(path, data)
        with patch.object(rs, "atomic_json", fail), self.assertRaisesRegex(RuntimeError, "summary failure"):
            c.run(self.args, predictor_loader=lambda _: FakePredictor(), runner=runner)
        self.args.resume = True
        result = c.run(self.args, predictor_loader=lambda _: self.fail("prediction"), runner=lambda *_: self.fail("replay"))
        self.assertEqual(result["combined"]["mae_seconds"], 2)

    def test_prediction_tampering_is_detected(self):
        self.requests()
        c.run(self.args, predictor_loader=lambda _: FakePredictor())
        with sqlite3.connect(Path(self.args.output_dir)/"checkpoint.sqlite3") as db:
            db.execute("UPDATE nodes SET value=value+1 WHERE g=0")
        db.close()
        self.args.resume = True
        with self.assertRaisesRegex(ValueError, "batch digest"):
            c.run(self.args, predictor_loader=lambda _: self.fail("loaded"))

    def test_output_tampering_is_detected_before_replay(self):
        _, runner = self._closed_loop()
        c.run(self.args, predictor_loader=lambda _: FakePredictor(), runner=runner)
        path = next((Path(self.args.output_dir)/"final").rglob("trace.csv"))
        path.write_text("tampered", encoding="utf-8")
        self.args.resume = True
        with self.assertRaisesRegex(ValueError, "Source changed"):
            c.run(self.args, runner=lambda *_: self.fail("replay"))

    def test_scope_change_rejected_on_resume(self):
        self.requests()
        c.run(self.args, predictor_loader=lambda _: FakePredictor())
        self.args.resume = True
        self.args.chunk_size += 1
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            c.run(self.args)

    def test_active_replay_blocks_resume(self):
        self.requests()
        c.run(self.args, predictor_loader=lambda _: FakePredictor())
        self.args.resume = True
        with rs.FileLock(Path(self.args.output_dir)/".replay.lock"):
            with self.assertRaisesRegex(RuntimeError, "lock busy"):
                c.run(self.args)

    def test_no_ceiling_default_and_streamed_cache_bit_identity(self):
        self.requests()
        self.assertEqual(self.args.max_cache_entries, 0)
        result = c.run(self.args, predictor_loader=lambda _: FakePredictor())
        header, values = c.read_cache(result["cache"]["path"])
        self.assertEqual(header["resource_limits"]["max_payload_bytes"], 0)
        reference = self.root/"reference.demand"
        c.write_cache(reference, self.plan(), result["scope_sha256"], values)
        self.assertEqual(reference.read_bytes(), Path(result["cache"]["path"]).read_bytes())

    def test_sql_import_deduplicates_and_ceiling_transaction_rolls_back(self):
        plan = self.plan()
        requests = self.root/"dupes.csv"
        requests.write_text("group_index,road_flow\n0,0\n0,0\n1,1\n", encoding="utf-8")
        store = rs.Store(self.root/"store.db", {"id": 1})
        try:
            with self.assertRaisesRegex(ValueError, "key limit"):
                store.import_requests(requests, plan, 1)
            self.assertEqual(store.counts()["total"], 0)
            self.assertEqual(store.import_requests(requests, plan, 0), {"rows": 3, "unique": 2})
            self.assertEqual(store.counts()["total"], 2)
        finally:
            store.close()

    def test_disk_guard_preserves_model_unloaded(self):
        self.requests()
        with patch.object(rs, "require_space", side_effect=RuntimeError("disk reserve")):
            with self.assertRaisesRegex(RuntimeError, "disk reserve"):
                c.run(self.args, predictor_loader=lambda _: self.fail("loaded"))
        self.assertFalse((Path(self.args.output_dir)/"summary.json").exists())

    def test_discovery_does_not_write_full_trace_but_strict_does(self):
        self._closed_loop()
        scenario = c.prepare_scenarios(self.args)[0]
        self.assertNotIn("--cap-context-trace", c.command_for(self.args, scenario, Path("c"), Path("d"), True))
        self.assertIn("--cap-context-trace", c.command_for(self.args, scenario, Path("c"), Path("d"), False))


if __name__ == "__main__":
    unittest.main()
