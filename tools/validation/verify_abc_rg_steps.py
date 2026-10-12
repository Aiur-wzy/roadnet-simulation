#!/usr/bin/env python3
"""Small MOCK exports and independent exhaustive cross-language step checks.

Never trains or loads AutoGluon. Real-network figures are planning estimates
using an explicitly supplied old ABC base-row count, not actual trained tables.
Nearest-node expected values use brute-force distances, not the production
quantizer. The generated files must never replace a real trained model/table.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path
from roadnet_paths import ROOT, source_path
import struct
import subprocess
import sys

import numpy as np
import pandas as pd

import catching.model_catching_sumo_cap_context_abc_rg as catch
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, mock_predictor

STEPS = (1.0, 2.0, 2.5, 3.0, 5.0, 10.0, 120.0)
SYNTHETIC_NET = '''<net>
<edge id="a"><lane length="10" speed="10"/></edge>
<edge id="b"><lane length="20" speed="10"/></edge>
<tlLogic id="J" type="static" programID="0" offset="0">
<phase duration="2" state="G"/><phase duration="1" state="y"/>
<phase duration="3.5" state="G"/><phase duration="1" state="y"/>
<phase duration="5.25" state="r"/></tlLogic>
<tlLogic id="K" type="static" programID="0" offset="0">
<phase duration="6.3" state="G"/><phase duration="1" state="y"/>
<phase duration="9.1" state="r"/></tlLogic>
<connection from="a" to="b" fromLane="0" toLane="0" tl="J" linkIndex="0"/>
<connection from="b" to="a" fromLane="0" toLane="0" tl="K" linkIndex="0"/>
</net>'''
# Hand-derived from the two programs above: (entry, next, next duration, entry max).
SIGNAL_GROUPS = (('G', 'G', 3.5, 2.), ('G', 'R', 5.25, 3.5),
                 ('G', 'R', 9.1, 6.3), ('R', 'G', 2., 5.25),
                 ('R', 'G', 6.3, 9.1))


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def independent_nodes(maximum, step):
    values = [min(.001, maximum), maximum]
    for multiple in itertools.count(1):
        node = multiple * step
        if node >= maximum:
            break
        if node > values[0]:
            values.append(node)
    return sorted(set(values))


def nearest(nodes, raw):
    # Tie-to-higher is a secondary sort key across ALL nodes, independent of
    # production bisect/lower_bound. Tests normalize only the declared 8 ULPs.
    return min(nodes, key=lambda node: (abs(node - raw), -node))


def value_bits(value):
    return struct.unpack('<Q', struct.pack('<d', float(value)))[0]


def scalar_prediction(case):
    length, road, lane, cap, context, entry, remaining, nxt, duration = case
    return (10. + length / cap + (road // 2) * .125 + lane * .25
            + context * .5 + (entry == 'R') * .0625 + remaining * .03125
            + (nxt == 'R') * .015625 + duration * .0078125)


def predict(frame):
    assert list(frame) == catch.FEATURE_COLS
    for name, categories in catch.CATEGORICAL_FEATURES.items():
        assert isinstance(frame[name].dtype, pd.CategoricalDtype)
        assert frame[name].cat.categories.tolist() == categories
        assert not frame[name].cat.ordered
    return np.array([scalar_prediction(row) for row in frame.itertuples(index=False, name=None)],
                    dtype=np.float64)


def own_cases(header):
    grid = header['free_speed_cap_grid']
    for group in header['signal_catalog']:
        for node in group['nodes']:
            signal = (group['entry_type'], node, group['next_type'], group['next_duration_seconds'])
            for length, cap_idx, context, lane in itertools.product(
                    header['road_lengths'], range(grid['index_min'], grid['index_max'] + 1),
                    range(6), range(header['lane_flow_max'] + 1)):
                for road in range(lane, header['road_flow_max'] + 1):
                    yield (length, road, lane, cap_idx * grid['step'], context, *signal)


def assert_catalog(header, step):
    catalog = header['signal_catalog']
    expected = [('G', 'G', 0., 0., [0.])]
    expected.extend((a, b, duration, maximum, independent_nodes(maximum, step))
                    for a, b, duration, maximum in SIGNAL_GROUPS)
    actual = [(g['entry_type'], g['next_type'], g['next_duration_seconds'],
               g['entry_max_seconds'], g['nodes']) for g in catalog]
    assert actual == expected, (actual, expected)
    assert header['signal_remaining_step'] == step
    assert header['signal_point_count'] == sum(len(item[-1]) for item in expected)
    assert any(g['entry_type'] == g['next_type'] == 'G' and not g['sentinel'] for g in catalog)


def make_queries(path, header):
    counts = Counter()
    rows = []
    query_id = 0

    def add(case, wanted_node, accept, category):
        nonlocal query_id
        query_id += 1
        counts[category] += 1
        node_case = (*case[:6], wanted_node, *case[7:])
        expected = value_bits(scalar_prediction(node_case)) if accept else 0
        rows.append(' '.join(map(str, (f'q{query_id}_{category}', int(accept), *case,
                                       wanted_node, expected))))
        if accept:
            _, node = catch.quantize_signal(header['signal_catalog'], *case[5:])
            assert node[1] == wanted_node, (case, node, wanted_node)
        else:
            try:
                catch.quantize_signal(header['signal_catalog'], *case[5:])
            except ValueError:
                pass
            else:
                raise AssertionError(('Python accepted invalid query', case))

    # Every exported point in every other dimension: catches signal/block/flow
    # offset mistakes and confirms all model node outputs at binary64 precision.
    for case in own_cases(header):
        add(case, case[6], True, 'exact_node')
    for group in header['signal_catalog']:
        if group['sentinel']:
            continue
        entry, nxt, duration = group['entry_type'], group['next_type'], group['next_duration_seconds']
        nodes, maximum = group['nodes'], group['entry_max_seconds']
        def signal_case(raw, override_duration=None):
            return (10., 3, 2, 9., 5, entry, raw, nxt,
                    duration if override_duration is None else override_duration)
        # Smallest positive subnormal is parsed through decimal text by C++.
        for raw in (math.nextafter(0., 1.), 1e-12, nodes[0] / 4):
            add(signal_case(raw), nearest(nodes, raw), True, 'near_zero')
        for lower, upper in zip(nodes, nodes[1:]):
            midpoint = lower + (upper - lower) / 2
            for raw, label in ((math.nextafter(midpoint, lower), 'below_midpoint'),
                               (midpoint, 'midpoint'),
                               (math.nextafter(midpoint, upper), 'above_midpoint')):
                add(signal_case(raw), nearest(nodes, raw), True, label)
        for fraction in (.013, .173, .419, .833, .997):
            raw = maximum * fraction
            add(signal_case(raw), nearest(nodes, raw), True, 'interior')
        for ulps in (0, 1, 8):
            raw = maximum + ulps * math.ulp(maximum)
            add(signal_case(raw), maximum, True, 'endpoint_tolerance')
        for raw in (0., -.0001, maximum + 9 * math.ulp(maximum), maximum + .00001):
            add(signal_case(raw), 0., False, 'invalid_remaining')
        add(signal_case(maximum, duration + .000001), 0., False, 'unknown_duration')
    add((10., 3, 2, 9., 5, 'G', 0., 'G', 0.), 0., True, 'sentinel')
    add((10., 3, 2, 9., 5, 'G', .0001, 'G', 0.), 0., False, 'invalid_sentinel')
    add((10., 3, 2, 9., 5, 'R', 0., 'R', 0.), 0., False, 'invalid_sentinel')
    add((10., 3, 2, 9., 5, 'Y', 1., 'G', 2.), 0., False, 'invalid_type')
    path.write_text('\n'.join(rows) + '\n', encoding='utf-8')
    return dict(counts)


def compile_helper(output, compiler):
    binary = output / 'test_cap_context_abc_rg_steps.exe'
    command = [compiler, '-std=c++14', '-O2', '-Wall', '-Wextra', '-I', str(ROOT / 'simulator'),
               str(source_path('test_cap_context_abc_rg_steps.cpp')),
               str(source_path('cap_context_abc_rg_table.cpp')), str(source_path('signal_rg.cpp')),
               '-o', str(binary)]
    result = subprocess.run(command, text=True, capture_output=True, errors='replace')
    (output / 'compile.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    if result.returncode:
        raise RuntimeError(f'Helper compilation failed; see {output / "compile.log"}')
    return binary, command


def planning(net, base_rows):
    estimates = []
    for step in (1., 2., 3., 5., 10.):
        groups = catch.signal_catalog(net, step)
        for group in groups:
            wanted = [0.] if group['sentinel'] else independent_nodes(group['entry_max_seconds'], step)
            assert group['nodes'] == wanted
        points = sum(len(group['nodes']) for group in groups)
        rows = base_rows * points
        estimates.append({'step_seconds': step, 'signal_points': points,
                          'signal_groups_including_sentinel': len(groups),
                          'baseline_base_rows': base_rows, 'expanded_rows_estimate': rows,
                          'dense_values_bytes_estimate': rows * 8,
                          'dense_values_gib_estimate': rows * 8 / 2 ** 30,
                          'relative_to_1s': points / (estimates[0]['signal_points'] if estimates else points)})
    return {'role': 'planning only; historical ABC base grid; no server predictor loaded or full table generated',
            'network': str(net.resolve()), 'network_sha256': hashlib.sha256(net.read_bytes()).hexdigest(),
            'estimates': estimates}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--real-net', type=Path, required=True)
    parser.add_argument('--baseline-base-rows', type=int, default=8_360_172)
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary, compile_command = compile_helper(output, args.compiler)
    real_plan = planning(args.real_net, args.baseline_base_rows)
    write_json(output / 'real_network_planning.json', real_plan)
    results = []
    for step in STEPS:
        directory = output / ('step_' + format(step, 'g').replace('.', 'p'))
        model, net, contract = prepare_fixture(directory)
        net.write_text(SYNTHETIC_NET, encoding='utf-8')
        for name in ('entry_signal_remaining_seconds', 'next_signal_duration_seconds'):
            contract['training_features'][name]['max'] = 9.1
        write_json(model / 'training_contract.json', contract)
        options = catch.build_parser().parse_args([
            '--model-dir', str(model), '--net-file', str(net), '--output-dir', str(directory / 'export'),
            '--signal-remaining-step', str(step), '--cap-step', '1', '--cap-min', '8', '--cap-max', '9',
            '--max-road-flow', '3', '--max-lane-flow', '2', '--chunk-size', '71', '--dense-reference'])
        plan = catch.build_plan(options)
        assert_catalog(plan, step)
        manifest = catch.export_table(options, plan, mock_predictor(predict))
        header, actual = catch.read_table(directory / 'export' / 'table.rle')
        expected = np.array([scalar_prediction(case) for case in own_cases(header)], dtype='<f8')
        assert actual.tobytes() == expected.tobytes()
        assert expected.tobytes() == (directory / 'export' / 'reference.f64').read_bytes()
        query_path = directory / 'queries.txt'
        counts = make_queries(query_path, header)
        command = [str(binary), str(directory / 'export' / 'table.rle'), str(query_path), str(step),
                   str(header['signal_point_count']), str(header['expanded_rows'])]
        result = subprocess.run(command, text=True, capture_output=True, errors='replace')
        (directory / 'cpp.log').write_text(result.stdout + result.stderr, encoding='utf-8')
        if result.returncode:
            raise RuntimeError(f'C++ step {step} failed: {result.stdout}{result.stderr}')
        report = {'step_seconds': step, 'signal_points': header['signal_point_count'],
                  'exported_rows': manifest['rows'], 'runs': manifest['run_count'],
                  'payload_bytes': header['payload_bytes'], 'query_counts': counts,
                  'dense_rle_model_binary64_equal': True, 'cpp_result': result.stdout.strip(),
                  'command': command}
        write_json(directory / 'verification.json', report)
        results.append(report)
        print(result.stdout.strip(), flush=True)
    report = {'status': 'PASS', 'MOCK_ONLY': True, 'steps': results,
              'compile_command': compile_command, 'planning': real_plan,
              'accuracy_claim': 'None; validates quantization/format/query mechanics only. No training, server model, full export or MAE measured.'}
    write_json(output / 'verification.json', report)
    print(json.dumps({'status': report['status'], 'steps': len(results),
                      'node_value_checks': sum(r['exported_rows'] for r in results),
                      'output': str(output)}, indent=2))


if __name__ == '__main__':
    main()
