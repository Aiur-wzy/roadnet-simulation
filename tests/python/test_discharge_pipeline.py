"""Small end-to-end fixtures for collector replay and independent table training."""

from __future__ import annotations

import json
import shutil
import unittest
import uuid
from pathlib import Path
from roadnet_paths import ROOT

import collection.collect_discharge_sequences as collector
import training.train_discharge_sequences as trainer
from features.discharge_sequences import EVENT_SCHEMA


class DischargePipelineTests(unittest.TestCase):
    def setUp(self):
        self.workspace = ROOT
        self.root = self.workspace / (".test_discharge_pipeline_" + uuid.uuid4().hex)
        self.root.mkdir(mode=0o777)

    def tearDown(self):
        resolved = self.root.resolve()
        if resolved.parent != self.workspace or not resolved.name.startswith(".test_discharge_pipeline_"):
            raise ValueError("unexpected cleanup path")
        shutil.rmtree(resolved)

    def event(self, run, kind, time, **extra):
        return {"schema": EVENT_SCHEMA, "run_id": run, "event": kind, "time": time,
                "lane_id": "lane_0", "section_id": "lane-end", **extra}

    def make_dataset(self, run, offset=0.0, empty=False):
        events = []
        for segment in range(4):
            anchor = segment * 20.0
            keys = [f"v{segment}_{i}" for i in range(3)]
            events.append(self.event(run, "start", anchor, anchor_time=anchor,
                                     valid_start=True, start_kind="stopped_green_start",
                                     cohort=[{"vehicle_key": key, "vehicle_id": key} for key in keys]))
            if empty:
                events.append(self.event(run, "cut", anchor + 1, cut_time=anchor,
                                         reason="fixture_known_interruption"))
                continue
            for index, time in enumerate((1.1, 3.6, 5.7)):
                events.append(self.event(run, "passage", anchor + time + offset,
                                         vehicle_key=keys[index], vehicle_id=keys[index],
                                         type_id="car", turn="s" if index != 1 else "l",
                                         movement_id="straight" if index != 1 else "left"))
        path = self.root / (run + ".jsonl")
        with path.open("w", encoding="utf-8") as handle:
            for event in events:
                handle.write(json.dumps(event) + "\n")
        output = self.root / run
        args = collector.parser().parse_args([
            "replay", "--events", str(path), "--run-id", run, "--output-dir", str(output)])
        collector.run(args)
        return output

    def test_four_runs_replay_train_and_whole_run_holdout(self):
        inputs = [self.make_dataset("r" + str(i), i / 10) for i in range(4)]
        output = self.root / "model"
        manifest = trainer.train_datasets(inputs, output, prefix_length=3,
                                          min_samples=2, min_sequences=2, holdout_runs=["r3"])
        self.assertTrue(manifest["complete"])
        self.assertEqual(manifest["split"]["assignments"]["r3"], "validation")
        self.assertEqual(set(manifest["split"]["assignments"].values()), {"train", "validation"})
        model = trainer.read_json(output / "model.json")
        predictor = trainer.Predictor(model)
        value, source = predictor.predict("headway", 2, {"start_kind": "stopped_green_start",
                                                        "turn": "l", "type_id": "car"})
        self.assertAlmostEqual(value, 2.5)
        self.assertNotEqual(source, "fixed")
        tail, _ = predictor.predict("headway", 30, {"start_kind": "stopped_green_start",
                                                  "turn": "s", "type_id": "car"})
        self.assertEqual(tail, 2.0)
        rows = list(collector.read_events(inputs[0] / "sequences.jsonl"))
        self.assertEqual(rows[0]["passages"][-1]["time"], rows[0]["cut_time"])
        self.assertEqual(rows[0]["headways"][1]["prev_turn"], "l")
        self.assertEqual(len(rows), 4)

    def test_changed_sequence_file_hash_rejected(self):
        dataset = self.make_dataset("hash-run")
        with (dataset / "sequences.jsonl").open("a", encoding="utf-8") as handle:
            handle.write("{}\n")
        with self.assertRaises(trainer.DatasetError):
            trainer.train_datasets([dataset], self.root / "bad-model")

    def test_zero_passage_starts_survive_collection_but_cannot_fit(self):
        dataset = self.make_dataset("empty-run", empty=True)
        manifest = trainer.read_json(dataset / "manifest.json")
        self.assertTrue(manifest["complete"])
        self.assertEqual(manifest["totals"]["zero_passage_sequences"], 4)
        with self.assertRaises(trainer.DatasetError):
            trainer.train_datasets([dataset], self.root / "no-model")

    def test_invalid_replay_leaves_failed_manifest_and_never_overwrites(self):
        source = self.root / "invalid.jsonl"
        source.write_text(json.dumps(self.event("broken", "unknown", 0)) + "\n", encoding="utf-8")
        output = self.root / "broken"
        args = collector.parser().parse_args([
            "replay", "--events", str(source), "--run-id", "broken", "--output-dir", str(output)])
        with self.assertRaises(ValueError):
            collector.run(args)
        manifest = trainer.read_json(output / "manifest.json")
        self.assertFalse(manifest["complete"])
        self.assertEqual(manifest["status"], "failed")
        before = (output / "manifest.json").read_bytes()
        with self.assertRaises(FileExistsError):
            collector.run(args)
        self.assertEqual(before, (output / "manifest.json").read_bytes())

    def test_same_observations_renamed_run_cannot_leak_to_validation(self):
        first = self.make_dataset("original")
        duplicate = self.make_dataset("alias")
        self.assertEqual(trainer.read_json(first / "manifest.json")["simulation_identity"],
                         trainer.read_json(duplicate / "manifest.json")["simulation_identity"])
        with self.assertRaises(trainer.DatasetError):
            trainer.train_datasets([first, duplicate], self.root / "alias-model")

    def test_duplicate_json_fields_are_not_silently_overwritten(self):
        source = self.root / "duplicate-key.jsonl"
        source.write_text('{"time": 1, "time": 2}\n', encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "duplicate JSON key"):
            list(collector.read_events(source))


if __name__ == "__main__":
    unittest.main()
