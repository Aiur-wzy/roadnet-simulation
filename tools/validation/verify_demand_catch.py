#!/usr/bin/env python3
"""Independent tiny closed-loop demand-catch equivalence tests (mock model).

Test SUMO truth is generated only for these synthetic fixture routes. Reported
MAE is not a research result; the requirement is equality to full-table runs.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
from unittest import mock
import xml.etree.ElementTree as ET

import numpy as np

import catching.model_catching_sumo_cap_context_abc_rg as catch
from tests.python.test_model_catching_sumo_cap_context_abc_rg import prepare_fixture, mock_predictor

STEPS = [(1., 1), (5., 5), (2.5, 5)]
ROAD_MAX, LANE_MAX = 40, 30


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')


def formula(frame):
    """Strongly different from fallback, exposing additional closed-loop keys."""
    return (frame.road_length.to_numpy() / frame.free_speed_cap.to_numpy()
            + 17. + frame.road_flow.to_numpy() * .35
            + frame.lane_flow.to_numpy() * .125
            + frame.platoon_context.astype(int).to_numpy() * .375
            + frame.entry_signal_remaining_seconds.to_numpy() * .11
            + frame.next_signal_duration_seconds.to_numpy() * .07
            + (frame.entry_signal_type == 'R').to_numpy() * 2.)


def run_command(command, log, timeout=120):
    env = os.environ.copy()
    if os.name == 'nt':
        env['PATH'] = r'C:\Program Files\MinGW\mingw64\bin' + os.pathsep + env.get('PATH', '')
    result = subprocess.run([str(x) for x in command], env=env, text=True,
                            capture_output=True, errors='replace', timeout=timeout)
    Path(log).write_text(result.stdout + result.stderr, encoding='utf-8')
    return result


def prepare(output, source_fixture, sumo):
    folder = output / 'fixtures'
    model, net, contract = prepare_fixture(folder)
    net.write_bytes((source_fixture / 'net.xml').read_bytes())
    contract['source_label'] = 'travel_time_label_gt_0p1'
    ranges = contract['training_features']
    pairs, _ = catch.abc.network_pairs(net)
    lengths = sorted({x[0] for x in pairs})
    ranges['road_length'] = {'min': min(lengths), 'max': max(lengths)}
    ranges['free_speed_cap'] = {'min': 13.89, 'max': 13.89}
    ranges['road_flow'] = {'min': 0, 'max': ROAD_MAX}
    ranges['lane_flow'] = {'min': 0, 'max': LANE_MAX}
    catalog = catch.signal_catalog(net, 1.)
    ranges['entry_signal_remaining_seconds'] = {'min': 0, 'max': max(x['entry_max_seconds'] for x in catalog)}
    ranges['next_signal_duration_seconds'] = {'min': 0, 'max': max(x['next_duration_seconds'] for x in catalog)}
    write_json(model / 'training_contract.json', contract)
    source = ET.parse(source_fixture / 'fixture32.rou.xml').getroot()
    cases = []
    for name, spacing in [('crowded', .1), ('staggered', .7), ('separated', 2.0)]:
        scenario = folder / name
        scenario.mkdir()
        data = copy.deepcopy(source)
        vehicles = data.findall('vehicle')
        assert len(vehicles) == 32
        for i, vehicle in enumerate(vehicles):
            vehicle.set('depart', format(i * spacing, '.9g'))
        route = scenario / 'routes.rou.xml'
        truth = scenario / 'truth.tripinfo.xml'
        ET.ElementTree(data).write(route, encoding='utf-8', xml_declaration=True)
        command = [sumo, '--net-file', net, '--route-files', route, '--tripinfo-output', truth,
                   '--seed', '42', '--end', '3600', '--no-step-log', 'true', '--duration-log.disable', 'true']
        result = run_command(command, scenario / 'sumo.log')
        assert result.returncode == 0, result.stderr
        trips = ET.parse(truth).getroot().findall('tripinfo')
        assert len(trips) == 32
        assert {t.get('id') for t in trips} == {v.get('id') for v in vehicles}
        cases.append({'name': name, 'route': str(route), 'truth': str(truth), 'vehicles': 32,
                      'route_sha256': sha(route), 'truth_sha256': sha(truth)})
    report = {'synthetic_fixture_only': True, 'model': str(model), 'net': str(net),
              'net_sha256': sha(net), 'cases': cases, 'sumo': str(sumo)}
    write_json(output / 'fixtures.json', report)
    return report


def full_options(model, net, output, signal_step, road_step):
    return catch.build_parser().parse_args([
        '--model-dir', str(model), '--net-file', str(net), '--output-dir', str(output),
        '--expected-threshold', '0p1', '--signal-remaining-step', str(signal_step),
        '--road-flow-step', str(road_step), '--road-flow-dense-until', '10',
        '--max-road-flow', str(ROAD_MAX), '--max-lane-flow', str(LANE_MAX),
        '--cap-min', '13.89', '--cap-max', '13.89', '--chunk-size', '50000',
        '--max-rows', '5000000'])


def simulation_command(binary, net, case, table, output):
    return [str(binary), '--use-sumo', '--sumo-net', str(net),
            '--sumo-route', str(case['route']), '--sumo-tripinfo', str(case['truth']),
            '--travel-time-mode', 'table', '--travel-time-table', str(table),
            '--eval-output', str(output / 'vehicle_eval.csv'),
            '--prediction-tripinfo-output', str(output / 'prediction_tripinfo.xml'),
            '--cap-context-trace', str(output / 'cap_context_trace.csv')]


def driver_options(driver, fixture, binary, directory, signal_step, road_step, max_rounds=100):
    argv = ['--model-dir', fixture['model'], '--net-file', fixture['net'],
            '--simulator', str(binary), '--output-dir', str(directory),
            '--expected-threshold', '0p1', '--signal-remaining-step', str(signal_step),
            '--road-flow-step', str(road_step), '--road-flow-dense-until', '10',
            '--max-road-flow', str(ROAD_MAX), '--max-lane-flow', str(LANE_MAX),
            '--cap-min', '13.89', '--cap-max', '13.89', '--chunk-size', '50000',
            '--max-rounds', str(max_rounds)]
    for case in fixture['cases']:
        argv += ['--scenario', case['name'], case['route'], case['truth']]
    return driver.build_parser().parse_args(argv)


def require_no_complete(directory):
    summary = directory / 'summary.json'
    assert not summary.exists(), 'A failure must not emit a success summary'
    failure = json.loads((directory / 'failure.json').read_text())
    assert failure['status'] == 'failed' and failure['formal_results_accepted'] is False
    return failure


def read_cache(path):
    with Path(path).open('rb') as stream:
        magic, header = stream.readline(), json.loads(stream.readline())
        payload = stream.read()
    assert magic == b'CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1\n'
    assert len(payload) == header['entry_count'] * 16
    assert hashlib.sha256(payload).hexdigest() == header['payload_sha256']
    values = {}
    previous = None
    for group, road, value in struct.iter_unpack('<IId', payload):
        key = group, road
        assert previous is None or previous < key
        assert np.isfinite(value) and value >= 1
        values[key] = value
        previous = key
    return magic, header, payload, values


def independent_cache_check(header, values):
    lanes = header['lane_flow_max'] + 1
    grid = header['free_speed_cap_grid']
    caps = grid['index_max'] - grid['index_min'] + 1
    # Explicit mixed-radix enumeration independently checks the driver decode.
    keys_to_case = {}
    group = 0
    signals = []
    for signal in header['signal_catalog']:
        for remaining in signal['nodes']:
            signals.append((signal['entry_type'], remaining, signal['next_type'], signal['next_duration_seconds']))
    for signal in signals:
        for length in header['road_lengths']:
            for cap in range(grid['index_min'], grid['index_max'] + 1):
                for context in range(6):
                    for lane in range(lanes):
                        keys_to_case[group] = (length, lane, cap * grid['step'], context, signal)
                        group += 1
    assert group == header['group_count']
    decoded_rows = {}
    for (key, road), value in values.items():
        length, lane, cap, context, signal = keys_to_case[key]
        policy = header['road_flow_quantization']
        assert lane <= road <= header['road_flow_max']
        assert (road in (lane, header['road_flow_max']) or road <= policy['dense_until']
                or road % policy['step'] == 0)
        row = (length, road, lane, cap, context, *signal)
        expected = float(formula(catch.feature_frame([row]))[0])
        assert struct.pack('<d', value) == struct.pack('<d', max(expected, 1.))
        decoded_rows[(key, road)] = row
    return decoded_rows


def check_readable_nodes(paths, values, decoded_rows):
    seen = set()
    for path in paths:
        with path.open(newline='', encoding='utf-8-sig') as stream:
            reader = csv.DictReader(stream)
            assert {'group_index', *catch.FEATURE_COLS, catch.LABEL_COL}.issubset(reader.fieldnames or [])
            for record in reader:
                key = int(record['group_index']), int(record['road_flow'])
                assert key in values and key not in seen
                expected_row = decoded_rows[key]
                for column, expected in zip(catch.FEATURE_COLS, expected_row):
                    if column in ('entry_signal_type', 'next_signal_type'):
                        assert record[column] == expected
                    else:
                        assert struct.pack('<d', float(record[column])) == struct.pack('<d', float(expected)), (key, column)
                assert struct.pack('<d', float(record[catch.LABEL_COL])) == struct.pack('<d', values[key])
                seen.add(key)
    assert seen == set(values)
    return {'rows': len(seen), 'nine_inputs_and_prediction_binary64_equal_to_cache': True,
            'files': [{'path': str(path), 'sha256': sha(path)} for path in paths]}


def timeout_check(driver, fixture, binary, run, loader):
    directory = run / 'timeout_guard'
    args = driver_options(driver, fixture, binary, directory, 5., 5)
    args.simulation_timeout = 1.
    class TimeoutProcess:
        def __init__(self):
            self.terminated = False
            self.timeouts = []
        def wait(self, timeout=None):
            self.timeouts.append(timeout)
            if not self.terminated:
                assert timeout == 1., 'Configured simulation timeout not passed to child wait'
                raise subprocess.TimeoutExpired('mock-simulator', timeout)
            return -15
        def terminate(self):
            self.terminated = True
        def kill(self):
            self.terminated = True
    child = TimeoutProcess()
    with mock.patch.object(driver.subprocess, 'Popen', return_value=child):
        try:
            driver.run(args, predictor_loader=loader)
        except (RuntimeError, subprocess.TimeoutExpired):
            pass
        else:
            raise AssertionError('Timeout unexpectedly completed')
    assert child.terminated
    failure = require_no_complete(directory)
    return {'child_terminated': True, 'wait_timeouts': child.timeouts, 'failure': failure}


def fault_checks(driver, fixture, binary, plan, summary, output, full_table):
    directory = output / 'faults'
    directory.mkdir()
    case = fixture['cases'][0]
    magic, header, payload, _ = read_cache(summary['cache']['path'])
    tests = []

    def invoke(name, table, discovery):
        folder = directory / name
        folder.mkdir()
        command = simulation_command(binary, fixture['net'], case, table, folder)
        if discovery:
            command += ['--demand-requests-output', str(folder / 'missing.csv'),
                        '--demand-status-output', str(folder / 'status.json')]
        result = run_command(command, folder / 'simulator.log')
        assert result.returncode != 0, (name, 'bad input was accepted')
        tests.append({'name': name, 'exit_code': result.returncode,
                      'log': str(folder / 'simulator.log')})

    # Header-valid but absent-key demand cache is fatal in ordinary strict mode.
    empty = directory / 'empty.demand'
    driver.write_cache(empty, plan, header['scope_sha256'], {})
    invoke('strict_missing_no_fallback', empty, False)

    # Corruption may never become an excuse for discovery fallback.
    assert payload
    bad = directory / 'bad_hash.demand'
    changed = bytearray(payload)
    changed[-1] ^= 1
    bad.write_bytes(magic + json.dumps(header).encode() + b'\n' + changed)
    invoke('bad_hash_discovery_fails', bad, True)
    repeated = directory / 'duplicate_keys.demand'
    assert len(payload) >= 32
    repeated_payload = payload[:16] + payload[:16] + payload[32:]
    repeated_header = dict(header, payload_sha256=hashlib.sha256(repeated_payload).hexdigest())
    repeated.write_bytes(magic + json.dumps(repeated_header).encode() + b'\n' + repeated_payload)
    invoke('duplicate_keys_discovery_fails', repeated, True)

    # A fully valid tiny coverage domain cannot silently fall back for road=1.
    tiny_options = full_options(fixture['model'], fixture['net'], directory / 'never_exported',
                                header['signal_remaining_step'], header['road_flow_quantization']['step'])
    tiny_options.max_road_flow = tiny_options.max_lane_flow = 0
    tiny_plan = catch.build_plan(tiny_options)
    tiny_cache = directory / 'tiny_domain.demand'
    driver.write_cache(tiny_cache, tiny_plan, header['scope_sha256'], {})
    invoke('missing_domain_discovery_fails', tiny_cache, True)

    # Ordinary full RLE is strict; the discovery option cannot turn it into a
    # partial-cache fallback path. Only the separately marked demand format can.
    invoke('full_rle_rejects_discovery', full_table, True)
    return tests


def execute(fixture, binary, output):
    import catching.catch_abc_rg_demand as driver
    report = {'status': 'running', 'mock_model_only': True, 'real_sumo_small_fixture_truth': True,
              'binary_sha256': sha(binary), 'fixture': fixture, 'runs': [],
              'driver_sha256': sha(driver.__file__), 'exporter_sha256': sha(catch.__file__),
              'validation_script_sha256': sha(__file__),
              'accuracy_claim': 'None: equivalence to full table and completion guards only.'}
    model_loads = []
    def loader(path):
        model_loads.append(str(path))
        return mock_predictor(formula)
    def runner(command, log):
        return run_command(command, log).returncode
    for signal_step, road_step in STEPS:
        run = output / ('signal' + format(signal_step, 'g').replace('.', 'p') + f'_road{road_step}')
        run.mkdir()
        options = full_options(fixture['model'], fixture['net'], run / 'full_table', signal_step, road_step)
        plan = catch.build_plan(options)
        catch.export_table(options, plan, mock_predictor(formula))
        full_table = run / 'full_table/table.rle'
        references = {}
        for case in fixture['cases']:
            directory = run / 'full' / case['name']
            directory.mkdir(parents=True)
            command = simulation_command(binary, fixture['net'], case, full_table, directory)
            # Match the demand driver's frozen simulation settings exactly.
            command += driver.FIXED_ARGUMENTS[3:]
            result = run_command(command, directory / 'simulator.log')
            assert result.returncode == 0, result.stdout + result.stderr
            assert 'Finished vehicles: 32 / 32' in result.stdout
            references[case['name']] = directory
        demand_args = driver_options(driver, fixture, binary, run / 'demand', signal_step, road_step)
        load_count_before = len(model_loads)
        summary = driver.run(demand_args, predictor_loader=loader, runner=runner)
        assert summary['status'] == 'complete' and summary['final_strict_missing_queries'] == 0
        assert len(model_loads) - load_count_before == 1
        rounds = [json.loads(path.read_text()) for path in sorted((run / 'demand').glob('round_*/round.json'))]
        assert rounds[-1]['missing_unique_union'] == 0
        assert summary['discovery_rounds'] == len(rounds)
        provisional_scenarios_suppressed = 0
        for round_record in rounds:
            for scenario_record in round_record['scenarios']:
                if scenario_record['status']['missing_queries']:
                    provisional = (run / 'demand' / f"round_{round_record['round']:03d}"
                                   / scenario_record['name'])
                    assert not (provisional / 'predictions.xml').exists()
                    assert not (provisional / 'evaluation.csv').exists()
                    provisional_scenarios_suppressed += 1
        later_new = sum(r['missing_unique_union'] for r in rounds[1:])
        assert len(rounds) > 2 and later_new > 0, 'Fixture must demonstrate frozen first trace is insufficient'
        _, cache_header, _, values = read_cache(summary['cache']['path'])
        decoded_rows = independent_cache_check(cache_header, values)
        readable = check_readable_nodes(sorted((run / 'demand').glob('round_*/predicted_nodes.csv')), values, decoded_rows)
        comparisons = []
        for case in fixture['cases']:
            reference, final = references[case['name']], run / 'demand/final' / case['name']
            pairs = [('cap_context_trace.csv', 'trace.csv'), ('prediction_tripinfo.xml', 'predictions.xml'),
                     ('vehicle_eval.csv', 'evaluation.csv')]
            for old, new in pairs:
                assert (reference / old).read_bytes() == (final / new).read_bytes(), (case['name'], old, 'full/demand mismatch')
            status = json.loads((final / 'status.json').read_text())
            assert status['complete'] is True and status['missing_queries'] == 0
            assert status['finished_count'] == 32 and status['vehicle_count'] == 32
            comparisons.append({'case': case['name'], 'trace_tripinfo_evaluation_byte_identical': True,
                                'vehicles': 32, 'queries': status['query_count']})
        record = {'signal_step': signal_step, 'road_step': road_step, 'dense_until': 10,
                  'full_prediction_rows': plan['prediction_rows'], 'cache_entries': len(values),
                  'rounds': len(rounds), 'missing_per_round': [x['missing_unique_union'] for x in rounds],
                  'keys_first_discovered_after_round1': later_new, 'comparisons': comparisons,
                  'readable_nodes': readable,
                  'provisional_scenarios_with_no_prediction_or_evaluation_files': provisional_scenarios_suppressed,
                  'all_cache_values_match_independent_key_decode': True}
        report['runs'].append(record)
        print(json.dumps({'PASS': record}, ensure_ascii=False), flush=True)
        if signal_step == 5. and road_step == 5:
            failed_args = driver_options(driver, fixture, binary, run / 'maxround1', signal_step, road_step, 1)
            try:
                driver.run(failed_args, predictor_loader=loader, runner=runner)
            except RuntimeError as error:
                assert 'max-rounds' in str(error)
            else:
                raise AssertionError('Max-rounds 1 unexpectedly completed')
            report['max_rounds_failure'] = require_no_complete(run / 'maxround1')
            report['negative_tests'] = fault_checks(driver, fixture, binary, plan, summary, run, full_table)
            report['timeout_guard'] = timeout_check(driver, fixture, binary, run, loader)
        write_json(output / 'verification.partial.json', report)
    report['status'] = 'PASS'
    write_json(output / 'verification.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--source-fixture', type=Path, required=True)
    parser.add_argument('--sumo', type=Path, required=True)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--use-prepared', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    if args.use_prepared:
        fixture = json.loads((output / 'fixtures.json').read_text())
    else:
        output.mkdir(parents=True, exist_ok=False)
        fixture = prepare(output, args.source_fixture.resolve(), args.sumo.resolve())
    if args.prepare_only:
        print(json.dumps({'status': 'fixtures_ready', 'cases': len(fixture['cases']), 'output': str(output)}, indent=2))
        return
    if args.binary is None:
        parser.error('--binary required unless --prepare-only')
    report = execute(fixture, args.binary.resolve(), output)
    print(json.dumps({'status': report['status'], 'combinations': len(report['runs']), 'output': str(output)}, indent=2))


if __name__ == '__main__':
    main()
