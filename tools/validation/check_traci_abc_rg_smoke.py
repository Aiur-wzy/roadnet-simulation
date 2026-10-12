"""Small read-only-SUMO old/new collector parity plus independent R/G checks.

Inputs and existing runs are never overwritten. Generated configs, logs and
outputs are confined to a fresh --output-dir. No training or catch is run.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
from roadnet_paths import ROOT, source_path
import subprocess
import sys
import xml.etree.ElementTree as ET


def require(ok, message):
    if not ok:
        raise AssertionError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        result = {}
        for row in reader:
            key = row["vehicleID"], row["route_index"], row["roadID"]
            require(key not in result, "duplicate occurrence")
            require(None not in row and all(v is not None for v in row.values()), "malformed CSV")
            result[key] = row
    return fields, result


def independent_segments(plan, row):
    """Slow explicit phase expansion, independent of the production helper."""
    ds, states = plan["durations"], plan["states"]
    link = int(row["signal_link_indices"].split(";")[0])
    phase = int(row["signal_anchor_phase"])
    start = float(row["signal_anchor_next_switch_time"]) - ds[phase]
    reference = float(row["signal_reference_time"])
    expanded = []
    # e-d is 0/3 here. Many cycles make selection independent of where the
    # currently running merged movement segment begins or ends.
    for index in range(5 * len(ds)):
        p = (phase + index) % len(ds)
        end = start + ds[p]
        state = states[p][link].upper()
        if expanded and expanded[-1][0] == state:
            expanded[-1][2] = end
        else:
            expanded.append([state, start, end])
        start = end
    current = next(i for i, (_, begin, end) in enumerate(expanded) if begin <= reference < end)
    state, begin, end = expanded[current]
    skip = int(state == "Y")
    if skip:
        current += 1
        while expanded[current][0] == "Y":
            current += 1
        first = expanded[current]
        duration = first[2] - first[1]
    else:
        first = expanded[current]
        duration = end - reference
    following = current + 1
    while expanded[following][0] == "Y":
        following += 1
    second = expanded[following]
    return [first[0], duration, second[0], second[2] - second[1], state, skip]


def validate(old, new, output):
    fields, a = rows(old / "features.csv")
    new_fields, b = rows(new / "features.csv")
    audit_fields, aa = rows(old / "features.csv.audit.csv")
    _, ba = rows(new / "features.csv.audit.csv")
    signal_cols = ["entry_signal_type", "entry_signal_remaining_seconds", "next_signal_type", "next_signal_duration_seconds"]
    require(set(new_fields) - set(fields) == set(signal_cols), "not exactly four new formal fields")
    require(a.keys() == b.keys() == aa.keys() == ba.keys() and bool(a), "road coverage changed/empty")
    for key in a:
        require(all(a[key][f] == b[key][f] for f in fields), f"old formal feature/label changed: {key}")
        require(all(aa[key][f] == ba[key][f] for f in audit_fields), f"old audit changed: {key}")
    contract = json.loads((new / "features.csv.contract.json").read_text(encoding="utf-8"))
    require(contract["schema"] == "road-time-cap-context-abc-rg-v1", "wrong signal schema")
    require(len(contract["raw_features"]) == 9, "wrong feature count")
    require(contract["data_sha256"] == sha(new / "features.csv"), "bad formal SHA")
    require(contract["audit"]["sha256"] == sha(new / "features.csv.audit.csv"), "bad audit SHA")
    counts, examples, checked = Counter(), {}, 0
    for key, row in ba.items():
        status = row["signal_status"]
        counts[status] += 1
        require(all(row[f] == b[key][f] for f in signal_cols), "formal/audit signal differs")
        actual = [row[signal_cols[0]], float(row[signal_cols[1]]), row[signal_cols[2]], float(row[signal_cols[3]]),
                  row["signal_state_at_reference"], int(row["signal_yellow_skipped"])]
        if status != "controlled_static":
            require(actual == ["G", 0., "G", 0., "G", 0], "invalid no-exit sentinel")
            continue
        plan = contract["signal_contract"]["runtime_programs"][row["signal_tls_id"]]
        expected = independent_segments(plan, row)
        require(actual[::2] == expected[::2] and actual[-1] == expected[-1], "signal type/skip differs")
        require(all(math.isclose(actual[i], expected[i], abs_tol=1e-8, rel_tol=0) for i in (1, 3)),
                f"independent signal-duration mismatch: {key} {actual} {expected}")
        checked += 1
        counts["reference_" + actual[4]] += 1
        examples.setdefault(actual[4], {"key": key, "first_rg": actual[:2], "second_rg": actual[2:4],
                                       "anchor": float(row["signal_anchor_time"]), "reference": float(row["signal_reference_time"]),
                                       "tls": row["signal_tls_id"], "links": row["signal_link_indices"]})
    ta = {x.attrib["id"]: x.attrib for x in ET.parse(old / "tripinfo.xml").getroot().findall("tripinfo")}
    tb = {x.attrib["id"]: x.attrib for x in ET.parse(new / "tripinfo.xml").getroot().findall("tripinfo")}
    require(bool(ta) and ta == tb, "tripinfo vehicle attributes changed")
    for folder in (old, new):
        require((folder / "features.csv").read_bytes() == (folder / "legacy.csv").read_bytes(), "legacy mirror differs")
        stats = ET.parse(folder / "statistics.xml").getroot()
        require(int(stats.find("teleports").get("total")) == 0, "teleports in small smoke")
        require(int(stats.find("safety").get("collisions")) == 0, "collisions in small smoke")
    return {"passed": True, "paired_vehicles": len(ta), "paired_road_rows": len(a),
            "six_labels_exact_values": 6 * len(a), "old_five_features_and_audit_exact": True,
            "all_tripinfo_vehicle_attributes_exact": True, "signal_independent_checks": checked,
            "signal_counts": dict(counts), "signal_examples": examples,
            "runtime_clock_queries": contract["signal_contract"]["clock_queries"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--reference-repo", type=Path, required=True)
    parser.add_argument("--net-file", type=Path, required=True)
    parser.add_argument("--route-file", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sumo-binary", required=True)
    parser.add_argument("--step-length", type=float, default=1.0)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    inputs = {str(p.resolve()): sha(p) for p in (args.net_file, args.route_file)}
    result = {"passed": False, "step_length": args.step_length, "input_sha256": inputs, "runs": {}}
    try:
        for arm, source, collector in (("old", args.reference_repo, "TraCI_Python_Cap_Context_ABC.py"),
                                        ("new", args.repo, "TraCI_Python_Cap_Context_ABC_RG.py")):
            folder = output / arm
            folder.mkdir()
            cfg = ET.Element("configuration")
            inp = ET.SubElement(cfg, "input")
            ET.SubElement(inp, "net-file", value=str(args.net_file.resolve()))
            ET.SubElement(inp, "route-files", value=str(args.route_file.resolve()))
            ET.SubElement(ET.SubElement(cfg, "time"), "step-length", value=str(args.step_length))
            ET.SubElement(ET.SubElement(cfg, "random_number"), "seed", value="42")
            out = ET.SubElement(cfg, "output")
            ET.SubElement(out, "tripinfo-output", value=str(folder / "tripinfo.xml"))
            ET.SubElement(out, "statistic-output", value=str(folder / "statistics.xml"))
            ET.SubElement(out, "precision", value="12")
            cfgpath = folder / "run.sumocfg"
            ET.ElementTree(cfg).write(cfgpath, encoding="utf-8", xml_declaration=True)
            if (source / "roadnet_paths.py").is_file():
                collector_path = source_path(collector, root=source)
                collector_module = collector_path.relative_to(source).with_suffix("").as_posix().replace("/", ".")
                collector_command = [sys.executable, "-B", "-m", collector_module]
                env["PYTHONPATH"] = str(source.resolve()) + os.pathsep + env.get("PYTHONPATH", "")
            else:
                collector_path = source / collector
                collector_command = [sys.executable, "-B", str(collector_path.resolve())]
            command = [*collector_command, "--sumo-config", str(cfgpath),
                       "--net-file", str(args.net_file.resolve()), "--sumo-binary", args.sumo_binary,
                       "--outputs", "training,legacy", "--training-output", str(folder / "features.csv"),
                       "--legacy-edge-output", str(folder / "legacy.csv")]
            env = os.environ.copy()
            env.update(PYTHONUTF8="1", PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8")
            with (folder / "collector.log").open("wb") as log:
                run = subprocess.run(command, cwd=folder, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=600)
            result["runs"][arm] = {"command": command, "returncode": run.returncode,
                                     "collector_sha256": sha(collector_path)}
            require(run.returncode == 0, f"{arm} collector failed; see {folder / 'collector.log'}")
            print("COLLECTED " + arm, flush=True)
        result.update(validate(output / "old", output / "new", output))
        require(all(sha(p) == digest for p, digest in inputs.items()), "input file modified")
        result["inputs_unchanged"] = True
    except Exception as exc:
        result["passed"] = False
        result["error"] = type(exc).__name__ + ": " + str(exc)
    (output / "validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
