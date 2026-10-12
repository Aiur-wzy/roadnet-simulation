#!/usr/bin/env python3
"""Independent synthetic sparse-flow export and unchanged C++ reader checks.

No model training, real predictor, production table mutation, or accuracy claim.
All integer road-flow inputs are checked, including those never sent to predict.
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import os
from pathlib import Path
from roadnet_paths import ROOT, source_path
import struct
import subprocess
import sys

import numpy as np
import pandas as pd

import catching.model_catching_sumo_cap_context_abc_rg as catch
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, mock_predictor

CASES = [(1, 10, 1), (1, 10, 5), (2, 0, 1), (2, 10, 5), (2, 40, 5),
         (5, 0, 5), (5, 10, 1), (5, 10, 5), (5, 40, 1),
         (7, 0, 1), (7, 10, 5), (7, 40, 5)]
ROAD_MAX, LANE_MAX = 27, 16


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


def value_bits(value):
    return struct.unpack('<Q', struct.pack('<d', float(value)))[0]


def road_nodes(lane, step, dense_until, maximum=ROAD_MAX):
    # Brute force each legal integer, independent of production node generation.
    return [n for n in range(lane, maximum + 1)
            if n in (lane, maximum) or n <= dense_until or n % step == 0]


def nearest(values, raw):
    return min(values, key=lambda n: (abs(n - raw), -n))


def formula(row):
    length, road, lane, cap, context, entry, remaining, nxt, duration = row
    # Road-dependent, nonmonotone and key-dependent values expose wrong mappings.
    return (20.0 + length / cap + ((road * 7) % 11) * .125 + road * road * .03125
            + lane * .25 + context * .5 + road * context * .0625
            + (entry == 'R') * .0625 + remaining * .03125
            + (nxt == 'R') * .015625 + duration * .0078125)


def raw_decode(path):
    with path.open('rb') as stream:
        assert stream.readline() == b'CAMS_CAP_CONTEXT_ABC_RG_RLE_V1\n'
        header = json.loads(stream.readline())
        payload = stream.read()
    assert hashlib.sha256(payload).hexdigest() == header['payload_sha256']
    split = 4 * (header['group_count'] + 1)
    offsets = np.frombuffer(payload[:split], '<u4')
    records = np.frombuffer(payload[split:], [('low', '<u2'), ('high', '<u2'), ('value', '<f8')])
    values = []
    for group in range(header['group_count']):
        current = records[offsets[group]:offsets[group + 1]]
        lane = group % (header['lane_flow_max'] + 1)
        assert current['low'][0] == lane
        assert current['high'][-1] == header['road_flow_max']
        assert np.all(current['low'][1:] == current['high'][:-1] + 1)
        values.append(np.repeat(current['value'], current['high'].astype('i8') - current['low'] + 1))
    result = np.concatenate(values)
    assert len(result) == header['expanded_rows']
    return header, result, payload


def original_cases(header):
    grid = header['free_speed_cap_grid']
    for signal in header['signal_catalog']:
        for rem in signal['nodes']:
            for length, cap, context, lane in itertools.product(header['road_lengths'],
                    range(grid['index_min'], grid['index_max'] + 1), range(6),
                    range(header['lane_flow_max'] + 1)):
                for road in range(lane, header['road_flow_max'] + 1):
                    yield (length, road, lane, cap * grid['step'], context,
                           signal['entry_type'], rem, signal['next_type'], signal['next_duration_seconds'])


def write_query(stream, row, expected, accept=True):
    stream.write(' '.join(map(str, (int(accept), *row, value_bits(expected)))) + '\n')


def run_probe(binary, table, queries, header, output, environment):
    command = [str(binary), str(table), str(queries), str(header['expanded_rows']),
               str(header['signal_remaining_step'])]
    completed = subprocess.run(command, env=environment, text=True, capture_output=True)
    output.write_text(completed.stdout + completed.stderr, encoding='utf-8')
    if completed.returncode:
        raise RuntimeError(completed.stdout + completed.stderr)
    return completed.stdout.strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='g++')
    parser.add_argument('--production-root', type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    binary = output / ('test_roadflow_catch_reader.exe' if os.name == 'nt' else 'test_roadflow_catch_reader')
    command = [args.compiler, '-std=c++14', '-O2', '-Wall', '-Wextra', '-I', str(ROOT / 'simulator'),
               str(source_path('test_roadflow_catch_reader.cpp')), str(source_path('cap_context_abc_rg_table.cpp')),
               str(source_path('signal_rg.cpp')), '-o', str(binary)]
    compiled = subprocess.run(command, text=True, capture_output=True)
    (output / 'compile.log').write_text(compiled.stdout + compiled.stderr, encoding='utf-8')
    if compiled.returncode:
        raise RuntimeError('C++ compilation failed: ' + compiled.stderr)
    environment = os.environ.copy()
    compiler = Path(args.compiler)
    if compiler.is_absolute():
        environment['PATH'] = str(compiler.parent) + os.pathsep + environment.get('PATH', '')
    sources = {name: digest(source_path(name)) for name in ['cap_context_abc_rg_table.cpp', 'cap_context_abc_rg_table.h', 'signal_rg.cpp', 'signal_rg.h']}
    if args.production_root:
        for name, sha in sources.items():
            production_source = source_path(name, root=args.production_root)
            if not production_source.is_file():
                production_source = args.production_root / name
            assert digest(production_source) == sha, (name, 'reader changed')
    report = {'status': 'running', 'synthetic_only': True, 'unchanged_reader_sha256': sources,
              'compile_command': command, 'cases': [], 'method': 'Full original integer input coverage; independent brute-force nearest representatives; ties upward; original C++ lookup contains no road-flow quantizer'}
    payloads = {}
    for step, dense_until, signal_step in CASES:
        directory = output / f'road{step}_dense{dense_until}_signal{signal_step}'
        model, net, contract = prepare_fixture(directory)
        contract['training_features']['road_flow']['max'] = ROAD_MAX
        contract['training_features']['lane_flow']['max'] = LANE_MAX
        write_json(model / 'training_contract.json', contract)
        options = catch.build_parser().parse_args([
            '--model-dir', str(model), '--net-file', str(net), '--output-dir', str(directory / 'export'),
            '--signal-remaining-step', str(signal_step), '--road-flow-step', str(step),
            '--road-flow-dense-until', str(dense_until), '--max-road-flow', str(ROAD_MAX),
            '--max-lane-flow', str(LANE_MAX), '--cap-step', '1', '--cap-min', '8', '--cap-max', '9',
            '--chunk-size', '137', '--dense-reference'])
        plan = catch.build_plan(options)
        seen = []
        def prediction(frame):
            assert list(frame) == catch.FEATURE_COLS
            for name, categories in catch.CATEGORICAL_FEATURES.items():
                assert isinstance(frame[name].dtype, pd.CategoricalDtype)
                assert frame[name].cat.categories.tolist() == categories
            rows = list(frame.itertuples(index=False, name=None))
            for row in rows:
                assert int(row[1]) in road_nodes(int(row[2]), step, dense_until)
            seen.extend(rows)
            return np.array([formula(row) for row in rows])
        manifest = catch.export_table(options, plan, mock_predictor(prediction))
        table_path = directory / 'export/table.rle'
        header, decoded, payload = raw_decode(table_path)
        assert header['expanded_rows'] == plan['expanded_rows']
        static_signal_count = header['group_count'] // (LANE_MAX + 1)
        expected_prediction_rows = static_signal_count * sum(len(road_nodes(lane, step, dense_until)) for lane in range(LANE_MAX + 1))
        assert len(seen) == expected_prediction_rows
        assert len(set(seen)) == expected_prediction_rows, 'duplicate inference nodes'
        assert header['prediction_rows'] == expected_prediction_rows
        assert plan['prediction_rows'] == expected_prediction_rows
        expected = []
        queries = directory / 'queries.txt'
        quantized_count = low_exact_count = tied_count = endpoint_count = 0
        with queries.open('w', encoding='utf-8') as stream:
            for original in original_cases(header):
                lane, road = int(original[2]), int(original[1])
                nodes = road_nodes(lane, step, dense_until)
                representative = nearest(nodes, road)
                node_case = (original[0], representative, *original[2:])
                value = formula(node_case)
                expected.append(value)
                write_query(stream, original, value)
                quantized_count += representative != road
                low_exact_count += road <= dense_until
                if road <= dense_until:
                    assert representative == road
                dist = min(abs(n - road) for n in nodes)
                tied_count += sum(abs(n - road) == dist for n in nodes) > 1
                endpoint_count += road in (lane, ROAD_MAX)
            # Joint road and remaining-time lookup: non-node remaining values.
            boundary_queries = 0
            for signal in header['signal_catalog']:
                if signal['sentinel']:
                    continue
                for lower, upper in zip(signal['nodes'], signal['nodes'][1:]):
                    raw = (lower + upper) / 2
                    wanted = nearest(signal['nodes'], raw)
                    for lane, road in [(0, 11), (3, 12), (11, 12), (16, 27)]:
                        original = (10., road, lane, 8., 5, signal['entry_type'], raw,
                                    signal['next_type'], signal['next_duration_seconds'])
                        mapped = (10., nearest(road_nodes(lane, step, dense_until), road), lane,
                                  8., 5, signal['entry_type'], wanted, signal['next_type'], signal['next_duration_seconds'])
                        write_query(stream, original, formula(mapped))
                        boundary_queries += 1
            for road, lane in [(-1, 0), (28, 0), (3, 4), (17, 17)]:
                write_query(stream, (10., road, lane, 8., 0, 'G', 0., 'G', 0.), 0., False)
        expected = np.array(expected, dtype='<f8')
        assert decoded.tobytes() == expected.tobytes()
        assert (directory / 'export/reference.f64').read_bytes() == expected.tobytes()
        if step == 1:
            payloads[signal_step] = payload
        elif dense_until > ROAD_MAX and signal_step in payloads:
            assert payload == payloads[signal_step], 'all-dense fallback changed RLE payload'
        cpp = run_probe(binary, table_path, queries, header, directory / 'cpp.log', environment)
        record = {'road_step': step, 'dense_until': dense_until, 'signal_step': signal_step,
                  'expanded_rows': len(decoded), 'prediction_rows': expected_prediction_rows,
                  'quantized_input_rows': quantized_count, 'low_exact_rows': low_exact_count,
                  'tie_queries': tied_count, 'endpoint_queries': endpoint_count,
                  'joint_signal_midpoint_queries': boundary_queries, 'invalid_queries': 4,
                  'table_header_metadata': header.get('road_flow_quantization'), 'cpp': cpp,
                  'full_dense_rle_bruteforce_equal': True}
        report['cases'].append(record)
        print(f'PASS road={step} dense={dense_until} signal={signal_step}: predict={len(seen)} full={len(decoded)}', flush=True)
    # Golden pre-change V1 table with absent optional sparse-flow metadata.
    fixture = ROOT / 'tests/fixtures/cap_context_abc_rg_mock.rle'
    legacy_header, legacy_values, _ = raw_decode(fixture)
    assert 'road_flow_quantization' not in legacy_header
    legacy_queries = output / 'legacy_queries.txt'
    with legacy_queries.open('w', encoding='utf-8') as stream:
        for row, value in zip(original_cases(legacy_header), legacy_values):
            write_query(stream, row, value)
    legacy = run_probe(binary, fixture, legacy_queries, legacy_header, output / 'legacy_cpp.log', environment)
    report['legacy_fixture'] = {'path': str(fixture), 'sha256': digest(fixture), 'queries': len(legacy_values), 'cpp': legacy}
    report['status'] = 'PASS'
    report['query_count'] = sum(x['expanded_rows'] + x['joint_signal_midpoint_queries'] + x['invalid_queries'] for x in report['cases']) + len(legacy_values)
    report['accuracy_claim'] = 'None. Tests only export/schema/lookup mechanics; no trained model MAE or new closed-loop accuracy measured.'
    write_json(output / 'verification.json', report)
    print(json.dumps({'status': report['status'], 'cases': len(report['cases']), 'queries': report['query_count'], 'output': str(output)}, indent=2))


if __name__ == '__main__':
    main()
