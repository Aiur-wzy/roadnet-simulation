# Grouped speed-net vehicle-error evaluation

## Experiment setup

Both experiments used the current CAMS/SUMO-compatible simulator binary built from this repository, without installing SUMO or running external SUMO generation.

| Dataset | Route | Tripinfo | Net | Travel-time mode | Lane discharge interval | Debug/timeline outputs |
| --- | --- | --- | --- | --- | --- | --- |
| `1_rou` | `data/1.rou.xml` | `data/tripinfo_1.xml` | `data/test.net.xml` | `speed-net` | `1` second | disabled |
| `full_no_change` | `data/full_no_change.rou.xml` | `data/tripinfo_full_no_change.xml` | `data/test.net.xml` | `speed-net` | `1` second | disabled |

Signal offsets were confirmed at the expected current values: `J17=45`, `J21=36`, and `J22=72`. XML parsing passed for all route, tripinfo, and network files, and tlLogic phase state lengths matched the network connection link-index requirements.

## Input coverage

Coverage was complete for both datasets, so these results are treated as valid baselines.

| Dataset | Route vehicles | Tripinfo records | Missing truth count | Compared vehicles |
| --- | ---: | ---: | ---: | ---: |
| `1_rou` | 483 | 483 | 0 | 483 |
| `full_no_change` | 47,623 | 47,623 | 0 | 47,623 |

## Overall dataset-level comparison

| Dataset | Compared | Mean pred duration | Mean truth duration | Bias | MAE | RMSE | MAPE | Median abs error | P90 abs error | P95 abs error | abs error > 1000 | pred duration > 2000 | truth duration > 2000 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `1_rou` ALL | 483 | 191.11 | 290.778 | -99.6687 | 138.273 | 232.891 | 47.8078 | 66 | 459 | 614.8 | 0 | 0 | 0 |
| `full_no_change` ALL | 47,623 | 127.02 | 243.96 | -116.94 | 136.862 | 413.313 | 33.6337 | 8 | 336 | 676 | 1,323 | 0 | 444 |

The `1_rou` subset has a higher median absolute error than `full_no_change`, but it has no duration errors above 1000 seconds and no truth durations above 2000 seconds. The `full_no_change` aggregate has a very low median error but much higher RMSE, showing a severe tail that is hidden by the median.

## `full_no_change` vehicle groups found

The vehicle ID parser detected these groups: `east_bottleneck`, `free_all`, `heavy_all`, `light_all_seed1`, `light_all_seed2`, `medium_core`, `north_bottleneck`, `oversat_all`, `south_bottleneck`, `southwest_bottleneck`, and `west_bottleneck`.

| Group | Compared | Bias | MAE | RMSE | Severe >1000 | Worst vehicle | Worst signed error |
| --- | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| `east_bottleneck` | 2,672 | -7.73428 | 7.73428 | 11.884 | 0 | `east_bottleneck_veh_001369` | -105 |
| `free_all` | 2,050 | -6.59854 | 7.06488 | 14.8807 | 0 | `free_all_veh_001360` | -103 |
| `heavy_all` | 8,241 | -6.92756 | 10.5679 | 20.2251 | 0 | `heavy_all_veh_003814` | -104 |
| `light_all_seed1` | 4,018 | -10.2424 | 11.5565 | 26.2546 | 0 | `light_all_seed1_veh_001354` | -273 |
| `light_all_seed2` | 3,280 | -9.0003 | 12.0875 | 27.0119 | 0 | `light_all_seed2_veh_001192` | 246 |
| `medium_core` | 2,646 | -6.50756 | 7.23394 | 12.7232 | 0 | `medium_core_veh_000752` | -97 |
| `north_bottleneck` | 2,672 | -50.2305 | 50.4828 | 77.2989 | 0 | `north_bottleneck_veh_000047` | -242 |
| `oversat_all` | 13,694 | -318.371 | 382.241 | 759.2 | 1,323 | `oversat_all_veh_002607` | -8240 |
| `south_bottleneck` | 2,672 | -5.70284 | 6.07784 | 10.1226 | 0 | `south_bottleneck_veh_002655` | -101 |
| `southwest_bottleneck` | 2,672 | -170.175 | 171.349 | 207.728 | 0 | `southwest_bottleneck_veh_002067` | -507 |
| `west_bottleneck` | 3,006 | -141.697 | 148.677 | 183.268 | 0 | `west_bottleneck_veh_000591` | -687 |

### Group-level findings

- Highest MAE: `oversat_all` at 382.241 seconds.
- Highest RMSE: `oversat_all` at 759.2 seconds.
- Strongest underprediction bias: `oversat_all` at -318.371 seconds, meaning predicted durations are too short.
- Strongest overprediction bias: no group has positive average duration bias; all groups are underpredicted on average.
- Most severe errors above 1000 seconds: `oversat_all` contributes all 1,323 severe cases.
- Error domination: the aggregate severe tail is dominated by `oversat_all`, not by `free_all` or `heavy_all`. `west_bottleneck` and `southwest_bottleneck` have meaningful underprediction, but neither contributes errors above 1000 seconds.

### Per-group notes

#### `east_bottleneck`
2,672 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 7.73428/11.884 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `east_bottleneck_veh_001369` with predicted/truth durations of 56/161 seconds.

#### `free_all`
2,050 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 7.06488/14.8807 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `free_all_veh_001360` with predicted/truth durations of 52/155 seconds.

#### `heavy_all`
8,241 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 10.5679/20.2251 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `heavy_all_veh_003814` with predicted/truth durations of 110/214 seconds.

#### `light_all_seed1`
4,018 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 11.5565/26.2546 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `light_all_seed1_veh_001354` with predicted/truth durations of 94/367 seconds.

#### `light_all_seed2`
3,280 vehicles were compared. Bias is negative overall, so this group is mostly too-fast/underpredicted despite the worst individual vehicle being an overprediction. MAE/RMSE are 12.0875/27.0119 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `light_all_seed2_veh_001192` with predicted/truth durations of 359/113 seconds.

#### `medium_core`
2,646 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 7.23394/12.7232 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `medium_core_veh_000752` with predicted/truth durations of 110/207 seconds.

#### `north_bottleneck`
2,672 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 50.4828/77.2989 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `north_bottleneck_veh_000047` with predicted/truth durations of 121/363 seconds.

#### `oversat_all`
13,694 vehicles were compared. Bias is strongly negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 382.241/759.2 seconds, with 1,323 severe errors above 1000 seconds. Worst vehicle: `oversat_all_veh_002607` with predicted/truth durations of 44/8284 seconds. This group has the dominant severe tail and is the main source of aggregate error.

#### `south_bottleneck`
2,672 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 6.07784/10.1226 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `south_bottleneck_veh_002655` with predicted/truth durations of 46/147 seconds.

#### `southwest_bottleneck`
2,672 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 171.349/207.728 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `southwest_bottleneck_veh_002067` with predicted/truth durations of 112/619 seconds. This group has substantial systematic underprediction but not the extreme >1000-second tail.

#### `west_bottleneck`
3,006 vehicles were compared. Bias is negative, so this group is mostly too-fast/underpredicted. MAE/RMSE are 148.677/183.268 seconds, with 0 severe errors above 1000 seconds. Worst vehicle: `west_bottleneck_veh_000591` with predicted/truth durations of 107/794 seconds. This group is a secondary systematic-error source.

## Interpretation

The baseline does not perform similarly across vehicle groups. Most free/light/heavy/core/bottleneck groups have small-to-moderate errors, while `oversat_all` is a clear outlier with much larger MAE, RMSE, and all severe duration errors above 1000 seconds.

Errors are mostly underprediction by group: every detected `full_no_change` group has negative duration bias, meaning predicted travel times are too short on average. `light_all_seed2` has an individual worst overprediction, but its average bias remains negative.

The `full_no_change` aggregate hides important group-level variation. Its median absolute error is only 8 seconds, but the aggregate RMSE is 413.313 seconds because `oversat_all` produces a long severe tail. Reporting only aggregate medians or means would understate the group-specific failure mode.

The `1_rou` dataset behaves like a simpler/saner subset in the sense that coverage is complete and no vehicle exceeds 1000 seconds of absolute duration error. However, its median absolute error of 66 seconds and negative bias of -99.6687 seconds still show systematic underprediction.

## Recommendation

Future model calibration and evaluation should be group-aware. The `oversat_all`, bottleneck, free/light/heavy, and core groups should be reported separately in future papers or technical reports because aggregate `full_no_change` metrics conceal important differences. Targeted diagnostics should focus first on `oversat_all`, then on `southwest_bottleneck` and `west_bottleneck` for systematic underprediction without >1000-second tails.

## Files generated and committed

- `docs/grouped_eval_results/1_rou_speednet_eval_summary_by_group.csv`
- `docs/grouped_eval_results/1_rou_speednet_eval_summary_by_group.md`
- `docs/grouped_eval_results/full_no_change_speednet_eval_summary_by_group.csv`
- `docs/grouped_eval_results/full_no_change_speednet_eval_summary_by_group.md`
- `docs/grouped_speednet_eval_summary.md`
- `scripts/analyze_eval_by_vehicle_group.py`

## PR safety

Raw `sumo_eval.csv` files were not committed. `run.log` files were not committed. Diagnostic directories were not committed. Only small grouped summaries and the dependency-free analysis script were committed.
