#!/usr/bin/env python3
"""Read-only independent audit of completed explicit-SF datasets.

Does not import the materializer or invoke SUMO. Reads each dataset's active
manifest, verifies files and recorded replay evidence, and writes one NEW JSON
report. Original exact SF cannot be reconstructed from rounded tripinfo: the
report explicitly distinguishes replay assertions from independent file checks.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


SCHEMA = "explicit-speed-factor-v1"


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def read_xml(path):
    return ET.parse(path, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))


def within_dataset(dataset, value):
    raw = Path(value)
    if raw.is_absolute():
        raise ValueError(f"Expected portable relative path: {value}")
    path = (dataset / raw).resolve()
    if dataset not in path.parents:
        raise ValueError(f"Path escapes dataset: {value}")
    return path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def options(tree):
    values = {}
    for group in tree.getroot():
        if not isinstance(group.tag, str):
            continue
        for node in group:
            if not isinstance(node.tag, str):
                continue
            if node.tag in values:
                raise ValueError(f"Duplicate SUMO option: {node.tag}")
            values[node.tag] = {"group": group.tag, "attributes": dict(node.attrib)}
    return values


def get_option(values, name, default=None):
    return values.get(name, {}).get("attributes", {}).get("value", default)


def motion_options(values):
    return {key: value for key, value in values.items()
            if key not in ("net-file", "route-files", "tripinfo-output", "statistic-output", "seed")}


def without_vehicle_sf(tree):
    root = copy.deepcopy(tree.getroot())
    for vehicle in root.findall("vehicle"):
        vehicle.attrib.pop("speedFactor", None)
    return ET.tostring(root)


def vehicles(tree, require_sf=False):
    result = {}
    for vehicle in tree.getroot().findall("vehicle"):
        name = vehicle.get("id")
        if not name or name in result:
            raise ValueError(f"Missing/duplicate route vehicle ID: {name!r}")
        value = vehicle.get("speedFactor")
        if require_sf and value is None:
            raise ValueError(f"Missing explicit speedFactor: {name}")
        if value is not None:
            value = float(value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"Nonpositive/nonfinite speedFactor: {name}")
        result[name] = value
    if not result:
        raise ValueError("No explicit vehicles in route")
    return result


def tripinfo(path):
    result = {}
    for _, node in ET.iterparse(path, events=("end",)):
        if node.tag == "tripinfo":
            identifier = node.get("id")
            if not identifier or identifier in result:
                raise ValueError(f"Missing/duplicate tripinfo ID: {identifier!r}")
            result[identifier] = dict(node.attrib)
            node.clear()
    return result


class Audit:
    def __init__(self):
        self.checks = []
        self.hash_cache = {}

    def check(self, scope, name, passed, **details):
        self.checks.append({"scope": scope, "check": name, "passed": bool(passed), **details})

    def hash(self, path):
        path = path.resolve()
        stat = path.stat()
        key = (str(path), stat.st_size, stat.st_mtime_ns)
        if key not in self.hash_cache:
            self.hash_cache[key] = sha256(path)
        return self.hash_cache[key]

    def check_hashes(self, scope, items, dataset=None):
        for name, expected in items.items():
            try:
                path = within_dataset(dataset, name) if dataset else Path(name).resolve()
                actual = self.hash(path)
                self.check(scope, "artifact_hash" if dataset else "source_hash", actual == expected,
                           path=str(path), expected=expected, actual=actual)
            except (OSError, ValueError) as exc:
                self.check(scope, "artifact_hash" if dataset else "source_hash", False,
                           path=name, error=f"{type(exc).__name__}: {exc}")

    def episode(self, dataset, row, marker_job, binary_digest):
        identifier = row["episode_id"]
        if not identifier or any(character in identifier for character in "/\\:\t\n\r") or identifier in (".", ".."):
            raise ValueError(f"Unsafe episode ID: {identifier!r}")
        scope = f"{dataset.name}/{identifier}"
        record = read_json(dataset / "manifests" / f"{identifier}.json")
        self.check(scope, "success_schema", record.get("schema") == SCHEMA and record.get("status") == "success")
        self.check(scope, "episode_and_split_preserved",
                   identifier == record.get("episode_id") == marker_job.get("episode_id")
                   and row["split"] == record.get("split") == marker_job.get("split"))
        self.check(scope, "source_hashes_match_run_signature", record.get("source_hashes") == marker_job.get("source_hashes"))
        self.check_hashes(scope, record["source_hashes"])
        paths = {name: within_dataset(dataset, value) for name, value in record["paths"].items()}
        self.check(scope, "path_artifacts_have_hashes",
                   set(record["paths"].values()) <= set(record["artifact_hashes"]))
        self.check_hashes(scope, record["artifact_hashes"], dataset)

        original_tree, explicit_tree = read_xml(paths["source_route"]), read_xml(paths["route"])
        original, factors = vehicles(original_tree), vehicles(explicit_tree, require_sf=True)
        expected = set(factors)
        count = len(factors)
        self.check(scope, "vehicle_count_and_ids", count == record["vehicle_count"] and expected == set(original), count=count)
        self.check(scope, "route_changed_only_by_vehicle_sf",
                   without_vehicle_sf(original_tree) == without_vehicle_sf(explicit_tree))
        self.check(scope, "preexisting_explicit_sf_preserved",
                   all(value is None or value == factors[name] for name, value in original.items()))
        self.check(scope, "original_route_byte_copy", self.hash(paths["source_route"]) == record["source_hashes"][record["route"]])
        received = dataset / "original" / "config_as_received" / f"{identifier}.sumocfg"
        self.check(scope, "original_config_byte_copy", self.hash(received) == record["source_hashes"][record["config"]])
        self.check(scope, "network_byte_copy", self.hash(paths["net"]) == record["source_hashes"][record["net"]])
        original_options = options(read_xml(received))
        original_seed = int(get_option(original_options, "seed", "23423"))
        self.check(scope, "seed_record_preserved", record["seed"] == original_seed)
        for role in ("reference", "explicit"):
            config = paths[role + "_config"]
            effective = options(read_xml(config))
            self.check(scope, role + "_motion_options_preserved", motion_options(effective) == motion_options(original_options))
            self.check(scope, role + "_seed_preserved", int(get_option(effective, "seed")) == original_seed)
            self.check(scope, role + "_random_disabled", str(get_option(effective, "random", "false")).lower() in ("false", "0"))
            required_paths = {"net-file": paths["net"],
                              "route-files": paths["source_route"] if role == "reference" else paths["route"],
                              "tripinfo-output": paths[role + "_tripinfo"],
                              "statistic-output": paths[role + "_statistics"]}
            for name, expected_path in required_paths.items():
                value = get_option(effective, name)
                raw = Path(value)
                resolved = (config.parent / raw).resolve()
                self.check(scope, role + "_portable_" + name,
                           not raw.is_absolute() and dataset in resolved.parents
                           and resolved == expected_path and resolved.is_file(), value=value)
            run = record[role + "_run"]
            self.check(scope, role + "_recorded_complete_coverage", run["vehicles"] == count and run["teleports"] == 0)
            if binary_digest:
                self.check(scope, role + "_sumo_binary_hash_preserved",
                           self.hash(Path(run["command"][0])) == binary_digest)
        self.check(scope, "recorded_exact_sf_replay", record.get("factors_exactly_preserved") is True)
        self.check(scope, "recorded_replay_versions_equal", record["reference_run"]["version"] == record["explicit_run"]["version"])
        self.check(scope, "job_manifest_points_to_explicit_config", within_dataset(dataset, row["sumocfg_file"]) == paths["explicit_config"])

        first, second = tripinfo(paths["reference_tripinfo"]), tripinfo(paths["explicit_tripinfo"])
        self.check(scope, "independent_tripinfo_complete_ids", set(first) == set(second) == expected)
        self.check(scope, "independent_tripinfo_all_attributes_equal", first == second)
        self.check(scope, "independent_tripinfo_all_arrived",
                   all(float(item.get("arrival", "-1")) >= 0 for item in first.values())
                   and all(float(item.get("arrival", "-1")) >= 0 for item in second.values()))
        self.check(scope, "recorded_tripinfo_matches_independent_check",
                   record["reference_vs_explicit"]["all_attributes_equal"] == (first == second)
                   and record["reference_vs_explicit"]["vehicle_count"] == len(set(first) & set(second)))
        self.check(scope, "rounded_tripinfo_sf_consistent_with_full_precision_route",
                   all(name in second and abs(float(second[name]["speedFactor"]) - factor) <= 0.005000001
                       for name, factor in factors.items()))
        values = list(factors.values())
        mean = math.fsum(values) / count
        self.check(scope, "recorded_sf_statistics_match", record["sf_range"] == [min(values), max(values)]
                   and math.isclose(record["sf_mean"], mean, rel_tol=1e-12, abs_tol=1e-12))
        result = {"episode_id": identifier, "split": row["split"], "seed": original_seed,
                  "vehicles": count, "sf_min": min(values), "sf_max": max(values), "sf_mean": mean,
                  "sf_unique": len(set(values)), "sf_below_one": sum(value < 1 for value in values),
                  "sf_equal_one": sum(value == 1 for value in values), "sf_above_one": sum(value > 1 for value in values),
                  "sumo_version": record["explicit_run"]["version"],
                  "recorded_loaded_departure_changes": {role: record[role + "_run"]["loaded_departure_changes"]
                                                        for role in ("reference", "explicit")},
                  "tripinfo_attributes_equal": first == second,
                  "historical_comparison": record.get("historical_vs_reference")}
        return result, Counter(values)

    def dataset(self, directory):
        dataset = directory.resolve(strict=True)
        scope = dataset.name
        start_check = len(self.checks)
        summary = read_json(dataset / "manifests" / "materialization_summary.json")
        marker = read_json(dataset / "manifests" / "materialization_run.json")
        with (dataset / "commands" / "traci_jobs.tsv").open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream, delimiter="\t"))
        marker_jobs = {item["episode_id"]: item for item in marker["jobs"]}
        identifiers = [row["episode_id"] for row in rows]
        self.check(scope, "active_manifest_coverage", bool(rows) and len(identifiers) == len(set(identifiers))
                   and set(identifiers) == set(marker_jobs) and len(marker_jobs) == len(marker["jobs"]))
        self.check(scope, "dataset_success_summary", summary.get("schema") == SCHEMA and marker.get("schema") == SCHEMA
                   and summary["expected_episodes"] == summary["successful_episodes"] == len(rows)
                   and not summary["failures"] and summary["all_factors_preserved"] is True)
        self.check(scope, "binary_fingerprint_recorded", len(marker.get("sumo_binary_sha256", "")) == 64)
        episodes, distribution = [], Counter()
        for row in rows:
            try:
                episode, counts = self.episode(dataset, row, marker_jobs[row["episode_id"]],
                                               marker.get("sumo_binary_sha256"))
                episodes.append(episode)
                distribution.update(counts)
            except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as exc:
                self.check(f"{scope}/{row.get('episode_id', '?')}", "episode_audit_exception", False,
                           error=f"{type(exc).__name__}: {exc}")
        count = sum(distribution.values())
        self.check(scope, "summary_total_vehicles", summary["vehicles"] == count)
        failures = [item for item in self.checks[start_check:] if not item["passed"]]
        return {"dataset_dir": str(dataset), "passed": not failures, "episodes": len(episodes),
                "expected_episodes": len(rows), "vehicles": count,
                "sf_min": min(distribution) if distribution else None,
                "sf_max": max(distribution) if distribution else None,
                "sf_mean": math.fsum(value * n for value, n in distribution.items()) / count if count else None,
                "sf_unique": len(distribution),
                "split_episodes": dict(Counter(item["split"] for item in episodes)),
                "split_vehicles": {split: sum(item["vehicles"] for item in episodes if item["split"] == split)
                                   for split in sorted({item["split"] for item in episodes})},
                "checks": len(self.checks) - start_check, "failed_checks": len(failures),
                "episode_results": episodes}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", action="append", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="New JSON report; refuses to overwrite any existing file")
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if output.exists():
        parser.error("Report output already exists; choose a new path")
    audit = Audit()
    results = []
    for dataset in args.dataset_dir:
        try:
            result = audit.dataset(dataset)
            results.append(result)
            print(json.dumps({key: value for key, value in result.items() if key != "episode_results"}, ensure_ascii=False), flush=True)
        except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as exc:
            audit.check(str(dataset), "dataset_audit_exception", False, error=f"{type(exc).__name__}: {exc}")
    failures = [item for item in audit.checks if not item["passed"]]
    report = {"schema": "explicit-speed-factor-independent-audit-v1", "passed": not failures,
              "dataset_count": len(results), "requested_dataset_count": len(args.dataset_dir),
              "vehicles": sum(item["vehicles"] for item in results),
              "checks": len(audit.checks), "failed_checks": len(failures),
              "evidence_limit": "No SUMO was run. Exact original-vs-explicit SF equality is the materializer's recorded runtime assertion; this audit independently checks file hashes, route values, route/config invariants, and full tripinfo equality. Rounded tripinfo cannot reconstruct original full-precision SF.",
              "datasets": results, "failures": failures, "all_checks": audit.checks}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps({"passed": report["passed"], "checks": report["checks"],
                      "failed_checks": report["failed_checks"], "report": str(output)}, ensure_ascii=False), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
