"""Surgically relabel audited historical turn groups; never recompute table values.

The command accepts only the three original, SHA-pinned diagnosis files. It
creates a new directory and refuses overwrite/reversal of already fixed data.
Only selected string tokens "1"/"3" are exchanged. All other source bytes,
including floating-point spellings, formatting, provenance and totals, survive.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

from features.turn_context import TURN_CONTEXT_CONTRACT, resolve_turn_context


SOURCE_SHA256 = {
    "fit_summary.json": "dcc992879228ff15a49b6dade566dbb384d2b4a16adb25466cfa385e869420f1",
    "benchmark_early.json": "07c38fcb6f7eb4b83394bcc03608cf15c548b36367f2a08d6a2619979de1a630",
    "closed_loop_input_comparison.json": "b66ce366a5cda2389aeb55ee1663f66b93677112b8b1cd303d6187430b61749d",
}
EXPECTED_CHANGES = {
    "fit_summary.json": {"metrics": 48, "threshold_decline": 8, "matched_free_cells": 16},
    "benchmark_early.json": {"metrics": 48, "threshold_decline": 8},
    "closed_loop_input_comparison.json": {"metrics": 16},
}
SWAP = {"1": "3", "3": "1"}


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def json_paths_and_spans(text):
    """Parse JSON while retaining every scalar's original character span."""
    decoder = json.JSONDecoder(parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    spans = {}

    def whitespace(pos):
        while pos < len(text) and text[pos] in " \t\r\n":
            pos += 1
        return pos

    def visit(pos, path):
        pos = whitespace(pos)
        start = pos
        if text[pos] == "{":
            result = {}
            pos = whitespace(pos + 1)
            if text[pos] == "}":
                return result, pos + 1
            while True:
                key, pos = decoder.raw_decode(text, pos)
                if not isinstance(key, str) or key in result:
                    raise ValueError("Non-string or duplicate JSON key")
                pos = whitespace(pos)
                if text[pos] != ":":
                    raise ValueError("Expected colon")
                result[key], pos = visit(pos + 1, path + (key,))
                pos = whitespace(pos)
                if text[pos] == "}":
                    return result, pos + 1
                if text[pos] != ",":
                    raise ValueError("Expected object comma")
                pos = whitespace(pos + 1)
        if text[pos] == "[":
            result = []
            pos = whitespace(pos + 1)
            if text[pos] == "]":
                return result, pos + 1
            while True:
                value, pos = visit(pos, path + (len(result),))
                result.append(value)
                pos = whitespace(pos)
                if text[pos] == "]":
                    return result, pos + 1
                if text[pos] != ",":
                    raise ValueError("Expected array comma")
                pos = whitespace(pos + 1)
        value, pos = decoder.raw_decode(text, pos)
        spans[path] = (start, pos)
        return value, pos

    document, end = visit(1 if text.startswith("\ufeff") else 0, ())
    if whitespace(end) != len(text):
        raise ValueError("Unexpected trailing JSON content")
    return document, spans


def target_paths(document, name):
    if name not in SOURCE_SHA256:
        raise ValueError(f"Unsupported historical document: {name}")
    paths = []
    sections = ("metrics",) if name == "closed_loop_input_comparison.json" else ("metrics", "threshold_decline")
    for section in sections:
        for index, row in enumerate(document[section]):
            population_matches = name == "closed_loop_input_comparison.json" or row.get("population") == "benchmark"
            if population_matches and row.get("dimension") == "incoming_turn_type" and row.get("group") in SWAP:
                paths.append((section, index, "group"))
    if name == "fit_summary.json":
        for cell_index, cell in enumerate(document["matched_free_cells"]):
            for group_index, row in enumerate(cell["free_hidden_groups"]):
                if row.get("dimension") == "incoming_turn_type" and row.get("group") in SWAP:
                    paths.append(("matched_free_cells", cell_index, "free_hidden_groups", group_index, "group"))
    return paths


def at_path(document, path):
    for component in path:
        document = document[component]
    return document


def overall_rows(document):
    return {section: [row for row in document.get(section, []) if row.get("dimension") == "overall"]
            for section in ("metrics", "threshold_decline")}


def repair_document(raw, name, verify_inverse=True):
    text = raw.decode("utf-8")
    document, spans = json_paths_and_spans(text)
    paths = target_paths(document, name)
    expected = copy.deepcopy(document)
    replacements = []
    changes = []
    for path in paths:
        before = at_path(document, path)
        after = SWAP[before]
        start, end = spans[path]
        if text[start:end] != json.dumps(before):
            raise ValueError("Audited group must be a plain single-digit JSON string")
        at_path(expected, path[:-1])[path[-1]] = after
        replacements.append((start + 1, after))
        changes.append({"path": list(path), "before": before, "after": after})
    characters = list(text)
    for position, value in replacements:
        characters[position] = value
    corrected_text = "".join(characters)
    corrected = corrected_text.encode("utf-8")
    actual, corrected_spans = json_paths_and_spans(corrected_text)
    if actual != expected or spans != corrected_spans:
        raise AssertionError("Unexpected structural or scalar change")
    if len(raw) != len(corrected) or sum(a != b for a, b in zip(raw, corrected)) != len(paths):
        raise AssertionError("Non-group source bytes changed")
    if overall_rows(document) != overall_rows(actual):
        raise AssertionError("Overall metrics changed")
    numeric_paths = [path for path in spans if isinstance(at_path(document, path), (int, float))
                     and not isinstance(at_path(document, path), bool)]
    numeric_tokens = [text[slice(*spans[path])] for path in numeric_paths]
    if numeric_tokens != [corrected_text[slice(*corrected_spans[path])] for path in numeric_paths]:
        raise AssertionError("Original numeric spelling changed")
    if verify_inverse and repair_document(corrected, name, verify_inverse=False)[0] != raw:
        raise AssertionError("Group permutation is not byte-exact self-inverse")
    evidence = {
        "source_sha256": sha256(raw), "corrected_sha256": sha256(corrected),
        "source_bytes": len(raw), "corrected_bytes": len(corrected),
        "changed_byte_count": len(paths), "changed_group_count": len(paths),
        "changed_sections": dict(Counter(path[0] for path in paths)), "changes": changes,
        "all_other_bytes_unchanged": True, "all_other_structural_fields_unchanged": True,
        "all_numeric_tokens_unchanged": True, "numeric_token_count": len(numeric_tokens),
        "numeric_token_sha256": sha256("\n".join(numeric_tokens).encode("utf-8")),
        "overall_metrics_unchanged": True, "byte_exact_self_inverse_verified": bool(verify_inverse),
    }
    return corrected, evidence


def audit_benchmark_turns(work, repo):
    """Read recorded identities and static routes only; no tables/SUMO/labels."""
    net_path = repo / "data/variants/speed_factor/inputs/dfbf38f23154_test_new.net.xml"
    directions = {}
    for connection in ET.parse(net_path).getroot().findall("connection"):
        source, target, direction = (connection.attrib[key] for key in ("from", "to", "dir"))
        if source.startswith(":") or target.startswith(":"):
            continue
        if (source, target) in directions and directions[source, target] != direction:
            raise ValueError("Conflicting lane-level raw directions for one edge pair")
        directions[source, target] = direction
    datasets = []
    for dataset in ("free", "tight", "congested"):
        feature_path = work / f"outputs/fit_audit_20260908/features/{dataset}/entry_features.csv"
        route_path = repo / f"data/variants/speed_factor/routes/{dataset}.rou.xml"
        root = ET.parse(route_path).getroot()
        routes = {route.attrib["id"]: route.attrib["edges"].split() for route in root.findall("route")}
        vehicles = {vehicle.attrib["id"]: routes[vehicle.attrib["route"]] for vehicle in root.findall("vehicle")}
        counters = {key: Counter() for key in ("canonical_incoming", "old_incoming", "canonical_outgoing", "old_to_new_incoming")}
        seen = set()
        first = final = changed = rows = 0
        with feature_path.open(encoding="utf-8-sig", newline="") as stream:
            for row in csv.DictReader(stream):
                vehicle, index = row["vehicle_id"], int(row["route_index"])
                edges = vehicles[vehicle]
                if (vehicle, index) in seen or edges[index] != row["edge_id"]:
                    raise AssertionError("Duplicate or route-edge mismatch")
                seen.add((vehicle, index))
                context = resolve_turn_context(edges, index, directions)
                old_incoming = 0 if index == 0 else {"r": 1, "s": 2, "l": 3}[directions[edges[index - 1], edges[index]]]
                incoming, outgoing = context["incoming_turn_type"], context["outgoing_turn_type"]
                if str(incoming) != SWAP.get(str(old_incoming), str(old_incoming)):
                    raise AssertionError("Correction is not solely the audited 1/3 permutation")
                if int(row["turn_type"]) != outgoing:
                    raise AssertionError("Historical feature turn_type is not canonical outgoing")
                if int(row["is_final"]) != int(not context["outgoing_turn_present"]):
                    raise AssertionError("Final boundary mismatch")
                if (context["incoming_turn_present"] and incoming == 0) or (context["outgoing_turn_present"] and outgoing == 0):
                    raise AssertionError("Unresolved current benchmark movement")
                first += not context["incoming_turn_present"]
                final += not context["outgoing_turn_present"]
                rows += 1
                changed += incoming != old_incoming
                counters["canonical_incoming"][str(incoming)] += 1
                counters["old_incoming"][str(old_incoming)] += 1
                counters["canonical_outgoing"][str(outgoing)] += 1
                counters["old_to_new_incoming"][f"{old_incoming}->{incoming}"] += 1
        if rows != sum(map(len, vehicles.values())) or len({vehicle for vehicle, _ in seen}) != len(vehicles):
            raise AssertionError("Incomplete vehicle x route occurrence coverage")
        datasets.append({"dataset": dataset, "rows": rows, "vehicles": len(vehicles), "first_rows": first,
                         "final_rows": final, "incoming_changed_rows": changed,
                         "historical_outgoing_all_correct": True, "unknown_present_rows": 0,
                         "features": str(feature_path), "feature_sha256": sha256(feature_path.read_bytes()),
                         "route": str(route_path), "route_sha256": sha256(route_path.read_bytes()),
                         **{key: dict(sorted(value.items())) for key, value in counters.items()}})
    return {"scope": "static routes plus recorded entry identity/legacy outgoing; no table loading or simulation",
            "net": str(net_path), "net_sha256": sha256(net_path.read_bytes()),
            "raw_direction_set": sorted(set(directions.values())), "datasets": datasets,
            "rows": sum(item["rows"] for item in datasets),
            "incoming_changed_rows": sum(item["incoming_changed_rows"] for item in datasets),
            "all_historical_outgoing_correct": True, "canonical_contract": TURN_CONTEXT_CONTRACT}


def validate_corrected_group_counts(documents, audit):
    """A source-independent count check; all current benchmark labels are positive."""
    expected = {item["dataset"]: item["canonical_incoming"] for item in audit["datasets"]}
    pooled = Counter()
    for value in expected.values():
        pooled.update(value)
    expected["pooled"] = dict(pooled)
    checked = 0
    for name in ("fit_summary.json", "benchmark_early.json"):
        for section in ("metrics", "threshold_decline"):
            for row in documents[name][section]:
                if row.get("population") == "benchmark" and row["dimension"] == "incoming_turn_type":
                    if row["n"] != expected[row["split"]][row["group"]]:
                        raise AssertionError(f"Route reconstruction disagrees: {name}, {row}")
                    checked += 1
    for row in documents["closed_loop_input_comparison.json"]["metrics"]:
        if row["dimension"] == "incoming_turn_type":
            for component in ("teacher_minus_truth", "closed_minus_truth", "closed_minus_teacher"):
                if row[component]["n"] != expected[row["dataset"]][row["group"]]:
                    raise AssertionError("Closed-loop incoming group count disagrees with static routes")
                checked += 1
    return checked


def report_erratum(source_dir, summary):
    path = source_dir / "REPORT.md"
    cells = [cell for cell in summary["matched_free_cells"] if cell["five_features"] == {
        "road_length": 320.0, "road_flow": 7.0, "lane_flow": 3.0, "speed_factor": 1.02, "speed_limit": 13.89}]
    if len(cells) != 1:
        raise AssertionError("Audited report neighborhood not found exactly once")
    groups = {row["group"]: row for row in cells[0]["train_hidden_groups"] if row["dimension"] == "incoming_turn_type"}
    if [groups[key]["gt_0p1"]["n"] for key in ("1", "2", "3")] != [246, 309, 240]:
        raise AssertionError("Audited training neighborhood changed")
    report = path.read_text(encoding="utf-8")
    if "incoming right/straight/left的gt0.1均值分别为28.354/28.262/29.771" not in report:
        raise AssertionError("Historical report statement changed; review instead of automatic erratum")
    return {"report": str(path), "report_sha256": sha256(path.read_bytes()),
            "section": "另一普通路邻域 length320/road_flow7/lane_flow3/SF1.02/limit13.89",
            "old_direction_names": ["right", "straight", "left"],
            "correct_direction_names": ["left", "straight", "right"],
            "training_json_already_correct_and_not_swapped": True,
            "training_group_details": groups}


def repair_directory(source_dir, output_dir, work=None, repo=None):
    source_dir, output_dir = source_dir.resolve(), output_dir.resolve()
    if output_dir.exists():
        raise FileExistsError("Output directory already exists; never overwrite or re-permute historical results")
    pending, evidence, documents = {}, [], {}
    for name, expected_sha in SOURCE_SHA256.items():
        path = source_dir / name
        raw = path.read_bytes()
        if sha256(raw) != expected_sha:
            raise ValueError(f"Historical source SHA mismatch: {path}; do not apply a reversible permutation blindly")
        corrected, proof = repair_document(raw, name)
        if proof["changed_sections"] != EXPECTED_CHANGES[name]:
            raise AssertionError("Unexpected number of changed historical groups")
        pending[name] = corrected
        documents[name] = json.loads(corrected.decode("utf-8-sig"))
        evidence.append({"source": str(path), "corrected": str(output_dir / name), **proof})
    erratum = report_erratum(source_dir, documents["fit_summary.json"])
    audit = audit_benchmark_turns(work, repo) if work is not None and repo is not None else None
    group_checks = validate_corrected_group_counts(documents, audit) if audit is not None else 0
    result = {"schema": "t04-turn-diagnostic-repair-v1", "status": "PASS", "files": evidence,
              "total_changed_group_tokens": sum(row["changed_group_count"] for row in evidence),
              "turn_context_contract": TURN_CONTEXT_CONTRACT, "report_erratum": erratum,
              "benchmark_route_audit": audit, "independent_incoming_group_count_checks": group_checks,
              "limitations": ["No table predictions, labels, waits or simulations were recomputed.",
                              "Only affected benchmark incoming groups are permuted; training/outgoing groups are untouched.",
                              "Historical provenance remains byte-identical; this certificate describes the correction.",
                              "The old independent_validation.json certifies old file hashes, not these corrected copies."]}
    output_dir.mkdir(parents=True)
    for name, raw in pending.items():
        (output_dir / name).write_bytes(raw)
    (output_dir / "repair_evidence.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    errata_text = """# T04 转向分组勘误（2026-09-11）

历史报告和 JSON 保留，不回写历史结论；纠正后的同名 JSON 位于本目录。
修复及静态路线复核证据见 [repair_evidence.json](repair_evidence.json)。

统一编号为 0=unknown、1=left、2=straight、3=right、4=uturn。
incoming 指上一条道路进入当前道路，outgoing 指当前道路驶往下一条道路。
首路无 incoming，末路无 outgoing，缺失与 unknown 通过 present 字段区分。

## JSON 的实际影响

- fit_summary.json：仅三基准 incoming 的 metrics 48 项、threshold_decline 8 项、matched_free_cells.free_hidden_groups 16 项交换 group 字符 1/3。
- benchmark_early.json：仅对应 incoming 的 metrics 48 项、threshold_decline 8 项交换。
- closed_loop_input_comparison.json：仅 incoming 的 metrics 16 项交换。

以上合计 144 个单字符。所有其余原始字节（包括数值的原始写法）完全不变。
总体、训练/留出、outgoing、流量、位置、需求档、逐车时间求和以及闭环指标不变。
无需重训、重采、读取六张 table 或重新仿真。不可把本修复再次用于新结果；脚本核对历史 SHA 并拒绝覆盖输出。

## 历史 REPORT.md 中另一个文字错误

“另一普通路邻域”795 条训练记录的 JSON incoming 编号原本正确，不能交换训练组。
历史 REPORT.md 把其名称写为 right/straight/left，应改读为 **left/straight/right**：

| incoming | n | gt0.1 标签均值 | gt3 标签均值 |
|---|---:|---:|---:|
| left（1） | 246 | 28.354 | 25.772 |
| straight（2） | 309 | 28.262 | 25.547 |
| right（3） | 240 | 29.771 | 26.496 |

数值和“同五特征下仍有条件差异”的观察不变；左右方向的解释须使用以上纠正名称。
历史独立验证的文件 SHA 只认证原文件；本目录的新证据单独认证修复结果。
"""
    (output_dir / "ERRATA.md").write_text(errata_text, encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--benchmark-workdir", type=Path, help="Original workspace with recorded entry features; pair with --repo")
    parser.add_argument("--repo", type=Path, help="Repository containing the frozen benchmark network/routes")
    args = parser.parse_args(argv)
    if bool(args.benchmark_workdir) != bool(args.repo):
        parser.error("--benchmark-workdir and --repo must be supplied together")
    result = repair_directory(args.source_dir, args.output_dir, args.benchmark_workdir, args.repo)
    print(json.dumps({"status": result["status"], "output": str(args.output_dir.resolve()),
                      "changed_group_tokens": result["total_changed_group_tokens"],
                      "independent_group_count_checks": result["independent_incoming_group_count_checks"],
                      "route_rows": result["benchmark_route_audit"]["rows"] if result["benchmark_route_audit"] else None}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
