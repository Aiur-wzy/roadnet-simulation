# Single-group CAMS evaluation scripts

## 1. Purpose

These scripts run CAMS/SUMO evaluation for one traffic group at a time. Each traffic group writes to its own directory, and every execution uses a separate run ID so repeated runs do not overwrite earlier results. The repository includes both one-wrapper-per-group entry points and a batch runner for all or selected groups.

## 2. Default inputs

```text
Network:
data/test_random_offsets_seed20260708.net.xml

Route:
data/<group>_no_change_random_offset_seed20260708.rou.xml

SUMO truth:
data/tripinfo_<group>_no_change_random_offset_seed20260708.xml

Travel-time mode:
speed-net

Lane discharge interval:
1
```

The current `./build/Simulation_Prediction --help` output supports `--use-sumo`, `--sumo-net`, `--sumo-route`, `--sumo-tripinfo`, `--eval-output`, `--travel-time-mode`, and `--lane-discharge-interval`. It does **not** list `--prediction-tripinfo-output`, so the scripts do not pass that option and do not create `prediction_tripinfo_<mode>.xml` unless the script support flag is updated after the executable adds the option.

## 3. Supported groups

The group list comes from `python3 generate_route_on_test_net_full.py --list-groups`.

| group | wrapper script | route input | tripinfo input |
| --- | --- | --- | --- |
| `free_all` | `scripts/single_group_eval/run_free_all.sh` | `data/free_all_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_free_all_no_change_random_offset_seed20260708.xml` |
| `light_all_seed1` | `scripts/single_group_eval/run_light_all_seed1.sh` | `data/light_all_seed1_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_light_all_seed1_no_change_random_offset_seed20260708.xml` |
| `light_all_seed2` | `scripts/single_group_eval/run_light_all_seed2.sh` | `data/light_all_seed2_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_light_all_seed2_no_change_random_offset_seed20260708.xml` |
| `medium_core` | `scripts/single_group_eval/run_medium_core.sh` | `data/medium_core_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_medium_core_no_change_random_offset_seed20260708.xml` |
| `heavy_all` | `scripts/single_group_eval/run_heavy_all.sh` | `data/heavy_all_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_heavy_all_no_change_random_offset_seed20260708.xml` |
| `oversat_all` | `scripts/single_group_eval/run_oversat_all.sh` | `data/oversat_all_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_oversat_all_no_change_random_offset_seed20260708.xml` |
| `west_bottleneck` | `scripts/single_group_eval/run_west_bottleneck.sh` | `data/west_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_west_bottleneck_no_change_random_offset_seed20260708.xml` |
| `north_bottleneck` | `scripts/single_group_eval/run_north_bottleneck.sh` | `data/north_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_north_bottleneck_no_change_random_offset_seed20260708.xml` |
| `east_bottleneck` | `scripts/single_group_eval/run_east_bottleneck.sh` | `data/east_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_east_bottleneck_no_change_random_offset_seed20260708.xml` |
| `south_bottleneck` | `scripts/single_group_eval/run_south_bottleneck.sh` | `data/south_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_south_bottleneck_no_change_random_offset_seed20260708.xml` |
| `southwest_bottleneck` | `scripts/single_group_eval/run_southwest_bottleneck.sh` | `data/southwest_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/tripinfo_southwest_bottleneck_no_change_random_offset_seed20260708.xml` |

## 4. Output directory structure

Outputs go under `eval_results/single_group/`. A mode/interval directory is derived from the selected travel-time mode and lane discharge interval, and each run ID gets a separate directory:

```text
eval_results/
└── single_group/
    └── oversat_all/
        └── speed-net_interval1/
            ├── 20260717_183000/
            │   ├── sumo_eval_speed-net.csv
            │   ├── run.log
            │   ├── command.txt
            │   └── metadata.txt
            └── 20260718_093000/
                └── ...
```

If a future executable supports `--prediction-tripinfo-output`, the run directory can also include `prediction_tripinfo_speed-net.xml`. By default, a second run receives a new timestamp run ID and does not overwrite the first run. If a user explicitly reuses a run ID, the script refuses to use an existing run directory unless `--force` is supplied.

## 5. Usage

Run one group:

```bash
bash scripts/single_group_eval/run_oversat_all.sh
```

Dry-run one group without creating eval outputs or executing CAMS:

```bash
bash scripts/single_group_eval/run_oversat_all.sh --dry-run
```

Run one group with a chosen run ID:

```bash
bash scripts/single_group_eval/run_oversat_all.sh \
  --run-id offset_seed20260708_test01
```

Run all groups:

```bash
bash scripts/run_all_single_group_cams_evals.sh
```

Dry-run all groups:

```bash
bash scripts/run_all_single_group_cams_evals.sh --dry-run
```

Run selected groups:

```bash
bash scripts/run_all_single_group_cams_evals.sh \
  --groups oversat_all,west_bottleneck,southwest_bottleneck
```

Continue after individual group failures:

```bash
bash scripts/run_all_single_group_cams_evals.sh \
  --continue-on-error
```

Use a shared experiment ID for all groups:

```bash
bash scripts/run_all_single_group_cams_evals.sh \
  --run-id random_offset_seed20260708_speed_net_interval1
```

## 6. Missing inputs

The scripts validate inputs before real execution. They do not generate route files, run SUMO, or create truth tripinfo automatically.

If a route file is missing, generate it explicitly, for example:

```bash
python3 generate_route_on_test_net_full.py \
  --group oversat_all \
  --output data/oversat_all_no_change_random_offset_seed20260708.rou.xml
```

If the matching SUMO truth tripinfo is missing, run SUMO with the matching configuration file, for example:

```bash
data/oversat_all_no_change_random_offset_seed20260708.sumocfg
```

## 7. PR safety

This change only adds scripts, documentation, and the ignore rule for `eval_results/single_group/`. CAMS was not run, eval CSV files were not generated, and prediction/SUMO tripinfo outputs were not generated. The PR should contain no experiment result files.
