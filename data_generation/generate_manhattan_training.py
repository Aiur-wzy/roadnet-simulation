"""Generate the frozen Manhattan 36+6 structured demand plan, never run SUMO.

Each training family is sampled once. Paired profiles share vehicle ID, full
route and independently materialized SF; background departure is also shared.
The default CLI first checks *all* planned quotas, then emits selected families.
No path edits, hidden quota fallback, implicit vType or restricted LC injection.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import hashlib
import heapq
import json
import math
import os
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import quoteattr


VERSION = "manhattan-structured-demand-v1"
LENGTH_BOUNDARIES = (2000, 5000, 10000, 15000)
BLOCK = 300
HOURS = 6
WAVE = (.5, 1, 1.5, 1.5, 1, .5)
SF_CONTRACT = {"distribution": "truncated_normal", "mean": 1., "sd": .1,
               "minimum": .2, "maximum": 2., "digits": 12,
               "source": "independent synthetic materialization; not native SUMO replay"}
TELEPORT_SETTINGS = {"time-to-teleport": "-1", "time-to-teleport.highways": "-1",
    "time-to-teleport.disconnected": "-1", "time-to-teleport.ride": "-1",
    "time-to-teleport.bidi": "-1", "time-to-teleport.railsignal-deadlock": "-1",
    "time-to-teleport.remove": "false", "time-to-teleport.remove-constraint": "false",
    "collision.action": "warn", "intermodal-collision.action": "warn"}


def canonical(value):
    return json.dumps(value, sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf8")


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(part)
    return digest.hexdigest()


def seed_for(*parts):
    return int.from_bytes(hashlib.sha256(canonical(parts)).digest(), "big")


def write_json(path, data):
    with Path(path).open("x", encoding="utf8", newline="\n") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def jsonl(path):
    with Path(path).open(encoding="utf8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def indexed(path):
    result = {}
    for row in jsonl(path):
        if row["route_id"] in result:
            raise ValueError(f"duplicate route_id in {path}: {row['route_id']}")
        result[row["route_id"]] = row
    return result


def length_bin(meters):
    return sum(meters >= boundary for boundary in LENGTH_BOUNDARIES)


def load_inputs(plan_path, pool_dir, descriptors_path, summary_path, network):
    paths = {"plan": Path(plan_path), "routes": Path(pool_dir) / "routes.jsonl",
             "metadata": Path(pool_dir) / "metadata.jsonl", "pool_manifest": Path(pool_dir) / "manifest.json",
             "descriptors": Path(descriptors_path), "structure_summary": Path(summary_path), "network": Path(network)}
    hashes = {key: {"path": str(path.resolve()), "sha256": sha(path)} for key, path in paths.items()}
    plan = json.loads(paths["plan"].read_text(encoding="utf8"))
    manifest = json.loads(paths["pool_manifest"].read_text(encoding="utf8"))
    summary = json.loads(paths["structure_summary"].read_text(encoding="utf8"))
    if hashes["network"]["sha256"] != plan["domain"]["network_sha256"] or summary["network_sha256"] != hashes["network"]["sha256"]:
        raise ValueError("network SHA differs from the frozen original-network plan")
    for key in ("routes", "metadata"):
        if hashes[key]["sha256"] != manifest["outputs"][key + ".jsonl"]["sha256"]:
            raise ValueError(f"pool {key} SHA mismatch")
    if hashes["routes"]["sha256"] != plan["path_pool"]["routes_sha256"]:
        raise ValueError("plan and pool disagree")
    routes, metadata, descriptors = indexed(paths["routes"]), indexed(paths["metadata"]), indexed(paths["descriptors"])
    if set(routes) != set(metadata) or set(routes) != set(descriptors) or len(routes) != manifest["route_count"]:
        raise ValueError("route/metadata/descriptor identity sets do not match")
    examples = plan["merge_candidate_examples"]
    pool = []
    for rid in sorted(routes):
        edges = tuple(routes[rid]["edges"])
        if not edges or rid != "r_" + hashlib.sha256(" ".join(edges).encode("utf8")).hexdigest():
            raise ValueError("noncanonical or empty route")
        md, desc = metadata[rid], descriptors[rid]
        length = float(desc["length_m"])
        if not md["static_legal"] or abs(float(md["length_m"]) - length) > .001 or desc["edge_count"] != len(edges):
            raise ValueError("metadata/descriptor static contract mismatch")
        frequency = md["original_fullroute_vehicle_count"]
        if not isinstance(frequency, int) or frequency < 0 or not math.isfinite(length) or length <= 0:
            raise ValueError("invalid path frequency/length")
        pairs = set(zip(edges, edges[1:]))
        groups = {k for k, v in desc["selectors"].items() if v}
        for i, item in enumerate(examples):
            for side in ("a", "b"):
                if (item["from_" + side], item["to"]) in pairs:
                    groups.add(f"merge_{i}_{side}")
        pool.append({"route_id": rid, "edges": edges, "length_m": length, "bin": length_bin(length),
                     "frequency": frequency, "groups": groups, "sources": sorted({x["kind"] for x in md["sources"]}),
                     "quality_flags": md["quality_flags"]})
    if sum(p["frequency"] for p in pool) != 192484:
        raise ValueError("original demand frequency does not close to 192484")
    validate_plan(plan)
    for key, path in paths.items():
        if sha(path) != hashes[key]["sha256"]:
            raise ValueError(f"input changed while loading: {key}")
    return plan, pool, hashes


def validate_plan(plan):
    train, valid = plan["training_files"], plan["validation_files"]
    if len(train) != 36 or len(valid) != 6:
        raise ValueError("frozen plan must contain 36 train and 6 validation records")
    if {r["family_id"] for r in train} & {r["family_id"] for r in valid}:
        raise ValueError("train/validation family leakage")
    expected = {(s, r, p) for s in "BSCNMG" for r in (1500, 3600, 6000) for p in ("steady", "wave")}
    if {(r["structure"], r["planned_average_vehicles_per_hour"], r["profile"]) for r in train} != expected:
        raise ValueError("training factorial differs from frozen plan")
    if len({r["scenario_id"] for r in train + valid}) != 42:
        raise ValueError("duplicate scenario ID")
    for r in train + valid:
        rate = r["planned_average_vehicles_per_hour"]
        factors = (1,) * 6 if r["profile"] == "steady" else WAVE
        expected_bg = [rate * 3 // 5] * 6
        expected_target = [int(rate * 2 / 5 * f) for f in factors]
        if (r["depart_window_seconds"] != [0, 21600] or r["vehicle_count"] != rate * 6
                or r["background_hourly"] != expected_bg or r["target_hourly"] != expected_target
                or r["all_hourly"] != [a + b for a, b in zip(expected_bg, expected_target)]):
            raise ValueError(f"invalid count/time contract: {r['scenario_id']}")


def group_names(structure):
    return {"B": ["neutral"], "S": ["south"], "C": ["central"], "N": ["north"],
            "M": [f"merge_{i}_{side}" for i in range(3) for side in ("a", "b")],
            "G": ["single_lane", "short_edges"]}[structure]


def balanced_labels(weights, n):
    """Deterministic deficit-rounding; exact N with bounded stratum discrepancy."""
    if sum(weights) <= 0:
        raise ValueError("empty stratum distribution")
    counts, result, total = [0] * len(weights), [], sum(weights)
    for index in range(n):
        target = max(range(len(weights)), key=lambda j: (weights[j] * (index + 1) / total - counts[j], -j))
        counts[target] += 1
        result.append(target)
    return result


def transport(rows, columns, capacities):
    """Exact integer selector x length quotas with capacity-aware max flow."""
    nr, nc = len(rows), len(columns)
    source, sink, size = nr + nc, nr + nc + 1, nr + nc + 2
    capacity, residual, adjacent = {}, {}, [[] for _ in range(size)]
    def add(a, b, value):
        capacity[a, b] = residual[a, b] = int(value)
        residual[b, a] = 0
        adjacent[a].append(b)
        adjacent[b].append(a)
    for i, value in enumerate(rows):
        add(source, i, value)
        for j in range(nc):
            add(i, nr + j, capacities[i][j])
    for j, value in enumerate(columns):
        add(nr + j, sink, value)
    flow = 0
    while True:
        previous, queue = {source: None}, deque([source])
        while queue and sink not in previous:
            a = queue.popleft()
            for b in adjacent[a]:
                if b not in previous and residual[a, b] > 0:
                    previous[b] = a
                    queue.append(b)
        if sink not in previous:
            break
        amount, b = sum(rows), sink
        while previous[b] is not None:
            a = previous[b]
            amount = min(amount, residual[a, b])
            b = a
        b = sink
        while previous[b] is not None:
            a = previous[b]
            residual[a, b] -= amount
            residual[b, a] += amount
            b = a
        flow += amount
    if flow != sum(rows) or sum(rows) != sum(columns):
        raise ValueError(f"infeasible selector/length quotas: rows={rows}, bins={columns}, maxflow={flow}, capacities={capacities}")
    return [[capacity[i, nr + j] - residual[i, nr + j] for j in range(nc)] for i in range(nr)]


def weighted_sample(candidates, count, rng, pool, neutral):
    if len(candidates) < count:
        raise ValueError(f"without-replacement support {len(candidates)} < requested {count}")
    # Efraimidis-Spirakis exponential races. New-source provenance is not a quota.
    def score(index):
        weight = pool[index]["frequency"] if neutral else max(1, pool[index]["frequency"])
        if weight <= 0:
            raise ValueError("zero-weight path entered neutral sampler")
        return -math.log(max(rng.random(), 2.0 ** -53)) / weight
    return [index for _, index in heapq.nsmallest(count, ((score(i), i) for i in candidates))]


def target_block_schedule(rate, profile):
    factors = (1,) * 6 if profile == "steady" else WAVE
    result = []
    for hour, factor in enumerate(factors):
        count = rate * 2 / 5 * factor / 12
        if not count.is_integer():
            raise ValueError("target count is not an integer per 300s block")
        for within in range(12):
            result.extend([hour * 12 + within] * int(count))
    return result


def build_family(pool, record):
    """Return compact assignments and a preflight report; never write files."""
    rate, structure = record["planned_average_vehicles_per_hour"], record["structure"]
    background_per_block = rate * 3 / 5 / 12
    if not background_per_block.is_integer():
        raise ValueError("background quota must be integer per 300s block")
    family, rng = record["family_id"], random.Random(seed_for(VERSION, record["route_seed_proposal"], record["family_id"]))
    neutral = [i for i, p in enumerate(pool) if p["frequency"] > 0]
    bins = [sum(p["frequency"] for p in pool if p["bin"] == b) for b in range(5)]
    assignments = []
    for block in range(72):
        chosen = weighted_sample(neutral, int(background_per_block), rng, pool, True)
        for index in chosen:
            assignments.append({"role": "background", "group": "neutral", "path_index": index,
                                "steady_block": block, "wave_block": block})
    schedules = [target_block_schedule(rate, profile) for profile in ("steady", "wave")]
    total = len(schedules[0])
    names = group_names(structure)
    group_sequence = [j % len(names) for j in range(total)]
    target_bins = balanced_labels(bins, total) if structure in "SCNM" else None
    group_pool = [[i for i, path in enumerate(pool) if (name == "neutral" and path["frequency"] > 0) or name in path["groups"]] for name in names]
    used = [defaultdict(set), defaultdict(set)]
    begin = 0
    while begin < total:
        block_pair = schedules[0][begin], schedules[1][begin]
        end = begin + 1
        while end < total and (schedules[0][end], schedules[1][end]) == block_pair:
            end += 1
        unavailable = used[0][block_pair[0]] | used[1][block_pair[1]]
        available = [[i for i in indices if i not in unavailable] for indices in group_pool]
        row_counts = [group_sequence[begin:end].count(j) for j in range(len(names))]
        selections = []
        if target_bins is not None:
            col_counts = [target_bins[begin:end].count(b) for b in range(5)]
            cells = [[[i for i in group if pool[i]["bin"] == b] for b in range(5)] for group in available]
            quotas = transport(row_counts, col_counts, [[len(c) for c in row] for row in cells])
            # Select most constrained cells first; overlaps are explicitly
            # removed. If joint membership defeats quotas, fail, never refill.
            ordering = sorted(((len(cells[g][b]) / q, g, b, q) for g, row in enumerate(quotas)
                               for b, q in enumerate(row) if q))
            selected = set()
            for _, g, b, q in ordering:
                candidates = [i for i in cells[g][b] if i not in selected]
                chosen = weighted_sample(candidates, q, rng, pool, False)
                selections.extend((g, i) for i in chosen)
                selected.update(chosen)
        else:
            selected = set()
            for g in sorted(range(len(names)), key=lambda j: len(available[j]) / max(1, row_counts[j])):
                candidates = [i for i in available[g] if i not in selected]
                chosen = weighted_sample(candidates, row_counts[g], rng, pool, structure == "B")
                selections.extend((g, i) for i in chosen)
                selected.update(chosen)
        rng.shuffle(selections)
        for group, index in selections:
            assignments.append({"role": "target", "group": names[group], "path_index": index,
                                "steady_block": block_pair[0], "wave_block": block_pair[1]})
            used[0][block_pair[0]].add(index)
            used[1][block_pair[1]].add(index)
        begin = end
    if len(assignments) != rate * 6:
        raise AssertionError("family vehicle count does not close")
    for index, assignment in enumerate(assignments):
        assignment["vehicle_id"] = f"{family}_v{index:07d}"
    total_km = sum(pool[a["path_index"]]["length_m"] for a in assignments) / 1000
    reference_mean = sum(p["length_m"] * p["frequency"] for p in pool) / sum(p["frequency"] for p in pool)
    reference_km = reference_mean * len(assignments) / 1000
    delta = total_km / reference_km - 1
    report = {"family_id": family, "structure": structure, "rate": rate, "vehicle_count": len(assignments),
              "background_count": sum(a["role"] == "background" for a in assignments),
              "target_groups": dict(Counter(a["group"] for a in assignments if a["role"] == "target")),
              "path_km": total_km, "neutral_expected_path_km": reference_km, "relative_path_km_difference": delta,
              "length_target_applicable": structure in "SCNM", "length_target_within_5pct": abs(delta) <= .05,
              "actual_length_bin_counts": dict(Counter(pool[a["path_index"]]["bin"] for a in assignments)),
              "target_length_bin_counts": dict(Counter(pool[a["path_index"]]["bin"] for a in assignments if a["role"] == "target")),
              "source_vehicle_counts": dict(Counter("+".join(pool[a["path_index"]]["sources"]) for a in assignments)),
              "distinct_paths": len({a["path_index"] for a in assignments}),
              "quality_flag_vehicle_counts": dict(Counter(k for a in assignments for k, v in pool[a["path_index"]]["quality_flags"].items() if v)),
              "status": "QUOTAS_FEASIBLE" if structure not in "SCNM" or abs(delta) <= .05 else "LENGTH_TARGET_REQUIRES_REVIEW"}
    return assignments, report


def synthetic_sf(family, vehicle_id, seed):
    rng = random.Random(seed_for("sf-v1", family, vehicle_id, seed))
    for _ in range(10000):
        value = rng.gauss(1., .1)
        if .2 <= value <= 2.:
            return f"{value:.12f}"
    raise RuntimeError("SF rejection exhausted")


def materialize(assignments, record):
    rows = []
    profile = record["profile"]
    for a in assignments:
        domain = "background" if a["role"] == "background" else profile
        # Integer microsecond jitter cannot round into the next block/horizon.
        tick = a[profile + "_block"] * BLOCK * 1000000 + seed_for("depart-v1", record["time_seed_proposal"], a["vehicle_id"], domain) % (BLOCK * 1000000)
        rows.append(dict(a, depart_tick=tick, speed_factor=synthetic_sf(record["family_id"], a["vehicle_id"], record["route_seed_proposal"] + 30000)))
    return sorted(rows, key=lambda a: (a["depart_tick"], a["vehicle_id"]))


def audit_assignments(assignments, pool, record):
    rate = record["planned_average_vehicles_per_hour"]
    if len(assignments) != rate * 6 or len({a["vehicle_id"] for a in assignments}) != len(assignments):
        raise ValueError("assignment total/identity mismatch")
    targets = Counter(a["group"] for a in assignments if a["role"] == "target")
    names = group_names(record["structure"])
    if set(targets) != set(names) or max(targets.values()) - min(targets.values()) > 1:
        raise ValueError("target subgroup quota mismatch")
    for profile in ("steady", "wave"):
        block_ids = defaultdict(set)
        counts = Counter()
        for a in assignments:
            i = a["path_index"]
            if i < 0 or i >= len(pool):
                raise ValueError("unknown assigned path")
            if a["role"] == "background" or a["group"] == "neutral":
                if pool[i]["frequency"] <= 0:
                    raise ValueError("zero-probability path in neutral cohort")
            elif a["group"] not in pool[i]["groups"]:
                raise ValueError("assigned route does not match selector")
            key = a["role"], a[profile + "_block"]
            if i in block_ids[key]:
                raise ValueError(f"route reused within paired role block: {key}")
            block_ids[key].add(i)
            counts[key] += 1
        for block in range(72):
            factor = 1 if profile == "steady" else WAVE[block // 12]
            if counts["background", block] != rate * 3 / 5 / 12 or counts["target", block] != rate * 2 / 5 * factor / 12:
                raise ValueError("role/block quota mismatch")
    return {"status": "PASS", "identities_unique": True, "both_profile_blocks_checked": 144,
            "source_sequences_retained": True, "role_block_repeated_paths": 0,
            "target_subgroups": dict(targets)}


def seconds_text(tick):
    whole, fraction = divmod(tick, 1000000)
    return f"{whole}.{fraction:06d}"


def write_scenario(directory, pool, assignments, record, hashes, report):
    directory = Path(directory)
    assignment_audit = audit_assignments(assignments, pool, record)
    directory.mkdir(parents=True, exist_ok=False)
    rows = materialize(assignments, record)
    route_path = directory / "routes.rou.xml"
    with route_path.open("x", encoding="utf8", newline="\n") as xml, (directory / "vehicles.jsonl").open("x", encoding="utf8", newline="\n") as audit:
        xml.write('<?xml version="1.0" encoding="UTF-8"?>\n<routes>\n')
        # Deliberately no vType: the original route uses DEFAULT_VEHTYPE;
        # retain runtime dynamics/full LC instead of importing small-net caps.
        for a in rows:
            path = pool[a["path_index"]]
            xml.write(f'  <vehicle id={quoteattr(a["vehicle_id"])} depart={quoteattr(seconds_text(a["depart_tick"]))} speedFactor={quoteattr(a["speed_factor"])}><route edges={quoteattr(" ".join(path["edges"]))}/></vehicle>\n')
            audit.write(canonical({**a, "route_id": path["route_id"], "depart": seconds_text(a["depart_tick"]),
                                   "length_m": path["length_m"], "source": path["sources"]}).decode("utf8") + "\n")
        xml.write('</routes>\n')
    config = ET.Element("configuration")
    inputs = ET.SubElement(config, "input")
    net = os.path.relpath(hashes["network"]["path"], directory).replace("\\", "/")
    ET.SubElement(inputs, "net-file", value=net)
    ET.SubElement(inputs, "route-files", value="routes.rou.xml")
    time = ET.SubElement(config, "time")
    ET.SubElement(time, "begin", value="0")
    ET.SubElement(time, "step-length", value="1")
    processing = ET.SubElement(config, "processing")
    for key, value in TELEPORT_SETTINGS.items():
        ET.SubElement(processing, key, value=value)
    random_section = ET.SubElement(config, "random_number")
    ET.SubElement(random_section, "seed", value=str(record["sumo_seed_proposal"]))
    output = ET.SubElement(config, "output")
    ET.SubElement(output, "tripinfo-output", value="tripinfo.xml")
    ET.SubElement(output, "tripinfo-output.write-unfinished", value="true")
    ET.SubElement(output, "statistic-output", value="statistics.xml")
    ET.SubElement(output, "summary-output", value="summary.xml")
    ET.indent(config)
    with (directory / "run.sumocfg").open("xb") as stream:
        ET.ElementTree(config).write(stream, encoding="utf8", xml_declaration=True)
    hourly = Counter(a["depart_tick"] // 3600000000 for a in rows)
    if [hourly[i] for i in range(6)] != record["all_hourly"]:
        raise AssertionError("written departure hourly counts disagree with plan")
    # Recompute planned network demand from the actual complete-path multiset.
    # These are visits, not observed flow, simultaneous occupancy or congestion.
    path_counts = Counter(a["path_index"] for a in assignments)
    road_counts, pair_counts = Counter(), Counter()
    for index, count in path_counts.items():
        edges = pool[index]["edges"]
        for edge in edges:
            road_counts[edge] += count
        for pair in zip(edges, edges[1:]):
            pair_counts[pair] += count
    with (directory / "road_load.jsonl").open("x", encoding="utf8", newline="\n") as stream:
        for edge, count in sorted(road_counts.items()):
            stream.write(canonical({"edge": edge, "planned_visits": count}).decode("utf8") + "\n")
    with (directory / "roadpair_load.jsonl").open("x", encoding="utf8", newline="\n") as stream:
        for (source, target), count in sorted(pair_counts.items()):
            stream.write(canonical({"from": source, "to": target, "planned_visits": count}).decode("utf8") + "\n")
    write_json(directory / "manifest.json", {"schema": VERSION, "scenario": record, "inputs": hashes,
        "generator_sha256": sha(__file__), "status": "DEMAND_GENERATED_NOT_SIMULATED",
        "outputs": {name: {"sha256": sha(directory / name)} for name in ("routes.rou.xml", "vehicles.jsonl", "run.sumocfg", "road_load.jsonl", "roadpair_load.jsonl")},
        "sf_contract": SF_CONTRACT, "vtype_contract": "Original implicit DEFAULT_VEHTYPE; no maxSpeed/LC/CF overrides; runtime resolved values must be audited by collector",
        "teleport_settings": TELEPORT_SETTINGS, "planned_depart_horizon": 21600, "simulation_hard_end": None,
        "statistics": report, "assignment_audit": assignment_audit,
        "planned_network_visits": {"roads": len(road_counts), "roadpairs": len(pair_counts),
                                   "road_visits": sum(road_counts.values()), "roadpair_visits": sum(pair_counts.values())},
        "planned_truth": {"tripinfo": "tripinfo.xml", "status": "NOT_RUN"}})


def run(args):
    output = Path(args.output_dir)
    if output.exists():
        raise ValueError("output directory already exists; refusing overwrite")
    plan, pool, hashes = load_inputs(args.plan, args.pool_dir, args.descriptors, args.structure_summary, args.network)
    records = plan["training_files"] + plan["validation_files"]
    families = {}
    for record in records:
        families.setdefault(record["family_id"], []).append(record)
    selected = set(args.scenario or [r["scenario_id"] for r in records])
    if selected - {r["scenario_id"] for r in records}:
        raise ValueError("unknown requested scenario")
    output.mkdir(parents=True, exist_ok=False)
    reports, errors, assignments_by_family = [], [], {}
    for family, items in families.items():
        try:
            assignments, report = build_family(pool, items[0])
            report["assignment_audit"] = audit_assignments(assignments, pool, items[0])
            reports.append(report)
            if not args.preflight_only and any(r["scenario_id"] in selected for r in items):
                assignments_by_family[family] = assignments
            print(json.dumps({"family": family, "status": report["status"], "km_delta": report["relative_path_km_difference"]}), flush=True)
        except ValueError as error:
            errors.append({"family": family, "error": str(error)})
            print(json.dumps(errors[-1]), flush=True)
    for info in hashes.values():
        if sha(info["path"]) != info["sha256"]:
            raise ValueError("source changed during quota check")
    length_flags = [r["family_id"] for r in reports if r["status"] == "LENGTH_TARGET_REQUIRES_REVIEW"]
    result = {"schema": VERSION + "-preflight", "generator_sha256": sha(__file__), "python": sys.version,
              "inputs": hashes, "families": reports, "errors": errors,
              "length_review_required": length_flags, "status": "PASS" if not errors and not length_flags else "REQUIRES_REVIEW",
              "scenario_count": len(records), "xml_generated": False,
              "sampling": "60/40; weighted without replacement per role in both paired 300s windows; exact selector quotas; aggregate neutral length strata for S/C/N/M",
              "target_weight": "max(original_fullroute_frequency,1); B and background use frequency exactly",
              "length_reference": "p0 expected vehicle-km at same N, not a claimed realized paired neutral scenario"}
    write_json(output / "preflight.json", result)
    if args.preflight_only:
        return 0 if result["status"] == "PASS" else 2
    if errors or length_flags:
        raise ValueError("preflight requires review; no scenario XML was written")
    scene_index = []
    for record in records:
        if record["scenario_id"] in selected:
            report = next(r for r in reports if r["family_id"] == record["family_id"])
            scene_dir = output / record["scenario_id"]
            write_scenario(scene_dir, pool, assignments_by_family[record["family_id"]], record, hashes, report)
            scene_index.append({"scenario_id": record["scenario_id"], "split": record["split"], "family_id": record["family_id"],
                "route_file": f'{record["scenario_id"]}/routes.rou.xml', "sumo_config": f'{record["scenario_id"]}/run.sumocfg',
                "scenario_manifest": f'{record["scenario_id"]}/manifest.json', "vehicle_count": record["vehicle_count"],
                "route_sha256": sha(scene_dir / "routes.rou.xml"), "config_sha256": sha(scene_dir / "run.sumocfg"),
                "scenario_manifest_sha256": sha(scene_dir / "manifest.json"),
                "network_file": hashes["network"]["path"], "network_sha256": hashes["network"]["sha256"]})
    for info in hashes.values():
        if sha(info["path"]) != info["sha256"]:
            raise ValueError("source changed during generation; no ready manifest published")
    write_json(output / "manifest.json", {"schema": VERSION, "status": "DEMAND_GENERATED_NOT_SIMULATED", "ready": True,
        "inputs": hashes, "scenarios": scene_index, "generator_sha256": sha(__file__), "python": sys.version,
        "preflight_sha256": sha(output / "preflight.json"), "sf_contract": SF_CONTRACT})
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("plan", "pool-dir", "descriptors", "structure-summary", "network", "output-dir"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--preflight-only", action="store_true", help="Allocate/check all 42 plans in memory; write only compact quota report, no route/cfg")
    parser.add_argument("--scenario", action="append", help="Generate selected scenario ID; repeated option; omission selects all 42 after all-plan preflight")
    args = parser.parse_args()
    try:
        return run(args)
    except (ValueError, OSError, KeyError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
