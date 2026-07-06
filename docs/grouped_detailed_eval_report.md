# Detailed grouped SUMO/CAMS evaluation report

## Executive summary

- The enhanced pipeline now emits optional CAMS prediction tripinfo XML and validates it against `sumo_eval.csv`; both reruns reported zero missing, extra, or mismatched prediction records.
- Aggregate full_no_change metrics hide strong group differences: ALL MAE is 136.862s, while oversat_all reaches 382.241s and quiet groups remain near 6-12s MAE.
- Severe >1000s errors are concentrated in oversat_all: 1323 severe vehicles, 1776 underprediction cliffs, and 278 overprediction cliffs.
- Directional bottlenecks are mostly underprediction: southwest_bottleneck MAE 171.349s, west_bottleneck MAE 148.677s, north_bottleneck MAE 50.483s.
- Low-demand groups (free/light/medium/east/south/heavy) have no >1000s severe errors in this run and should not drive global tuning.
- Errors are left-heavy overall (negative bias -116.94s), indicating CAMS usually misses SUMO congestion duration rather than overcreating it, except oversat_all has mixed tails.
- Sorted-vs-unsorted summaries are committed separately so future regressions can distinguish distribution reproduction from per-vehicle delay assignment.

## Experiment setup

- Datasets: `data/1.rou.xml` with `data/tripinfo_1.xml`; `data/full_no_change.rou.xml` with `data/tripinfo_full_no_change.xml`.
- Network: `data/test.net.xml`.
- Simulator mode: `--travel-time-mode speed-net --lane-discharge-interval 1`.
- Prediction output: `--prediction-tripinfo-output <diag>/prediction_tripinfo.xml` (raw diag files not committed).
- Debug/timeline output: not enabled.

## Input coverage

- `1_rou`: 483 compared vehicles; prediction tripinfo consistency missing=0, extra=0, mismatchedDurations=0.
- `full_no_change`: 47,623 compared vehicles; prediction tripinfo consistency missing=0, extra=0, mismatchedDurations=0.

## Dataset-level comparison

| dataset | compared | bias | MAE | RMSE | severe >1000 | under cliffs | over cliffs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1_rou | 483 | -99.6687 | 138.273 | 232.891 | 0 | 5 | 0 |
| full_no_change | 47623 | -116.94 | 136.862 | 413.313 | 1323 | 1776 | 278 |

## Per-group detailed analysis for full_no_change

### oversat_all

- Size: 13694 vehicles.
- Unsorted paired metrics: bias=-318.371s, MAE=382.241s, RMSE=759.2s, p95 abs=1555s.
- Sorted distribution metrics: sortedMAE=318.371s, sortedRMSE=571.746s, diffMean=-318.371s.
- Left/right bias pattern: underprediction-dominated; under=11974 (0.874398), over=1632 (0.119176), severe under/over=1221/102.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=1776 (0.129692), over=278 (0.0203009).
- Worst vehicle: oversat_all_veh_002607 with pred=44s, truth=8284s, signed=-8240s.
- Route-generation basis: ALL routes, period=30 from 58000-68000: high-demand all-corridor stress.
- Possible cause: SUMO develops long queues under high/targeted demand that speed-net interval=1 does not propagate to the same vehicles.
- Recommendation: Prioritize targeted diagnostics.

### west_bottleneck

- Size: 3006 vehicles.
- Unsorted paired metrics: bias=-141.697s, MAE=148.677s, RMSE=183.268s, p95 abs=381s.
- Sorted distribution metrics: sortedMAE=141.697s, sortedRMSE=161s, diffMean=-141.697s.
- Left/right bias pattern: underprediction-dominated; under=2943 (0.979042), over=62 (0.0206254), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: west_bottleneck_veh_000591 with pred=107s, truth=794s, signed=-687s.
- Route-generation basis: WEST_IN routes starting from E9/E10, period=30.
- Possible cause: SUMO develops long queues under high/targeted demand that speed-net interval=1 does not propagate to the same vehicles.
- Recommendation: Prioritize targeted diagnostics.

### southwest_bottleneck

- Size: 2672 vehicles.
- Unsorted paired metrics: bias=-170.175s, MAE=171.349s, RMSE=207.728s, p95 abs=344s.
- Sorted distribution metrics: sortedMAE=170.459s, sortedRMSE=198.793s, diffMean=-170.175s.
- Left/right bias pattern: underprediction-dominated; under=2297 (0.859656), over=328 (0.122754), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: southwest_bottleneck_veh_002067 with pred=112s, truth=619s, signed=-507s.
- Route-generation basis: SOUTHWEST_IN routes starting from E18/E17, period=30.
- Possible cause: SUMO develops long queues under high/targeted demand that speed-net interval=1 does not propagate to the same vehicles.
- Recommendation: Prioritize targeted diagnostics.

### north_bottleneck

- Size: 2672 vehicles.
- Unsorted paired metrics: bias=-50.2305s, MAE=50.4828s, RMSE=77.2989s, p95 abs=152s.
- Sorted distribution metrics: sortedMAE=50.2305s, sortedRMSE=62.5593s, diffMean=-50.2305s.
- Left/right bias pattern: underprediction-dominated; under=2435 (0.911302), over=169 (0.0632485), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: north_bottleneck_veh_000047 with pred=121s, truth=363s, signed=-242s.
- Route-generation basis: NORTH_IN routes starting from E14/-E2, period=30.
- Possible cause: SUMO develops long queues under high/targeted demand that speed-net interval=1 does not propagate to the same vehicles.
- Recommendation: Prioritize targeted diagnostics.

### heavy_all

- Size: 8241 vehicles.
- Unsorted paired metrics: bias=-6.92756s, MAE=10.5679s, RMSE=20.2251s, p95 abs=19s.
- Sorted distribution metrics: sortedMAE=6.99066s, sortedRMSE=7.1529s, diffMean=-6.92756s.
- Left/right bias pattern: underprediction-dominated; under=7299 (0.885693), over=769 (0.0933139), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: heavy_all_veh_003814 with pred=110s, truth=214s, signed=-104s.
- Route-generation basis: ALL routes, period=50: heavy but less saturated all-corridor load.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

### free_all

- Size: 2050 vehicles.
- Unsorted paired metrics: bias=-6.59854s, MAE=7.06488s, RMSE=14.8807s, p95 abs=10s.
- Sorted distribution metrics: sortedMAE=6.59854s, sortedRMSE=6.6435s, diffMean=-6.59854s.
- Left/right bias pattern: underprediction-dominated; under=1966 (0.959024), over=54 (0.0263415), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: free_all_veh_001360 with pred=52s, truth=155s, signed=-103s.
- Route-generation basis: ALL routes with 50 uniform departures over 10000s.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

### light_all_seed1

- Size: 4018 vehicles.
- Unsorted paired metrics: bias=-10.2424s, MAE=11.5565s, RMSE=26.2546s, p95 abs=80s.
- Sorted distribution metrics: sortedMAE=10.2424s, sortedRMSE=12.5968s, diffMean=-10.2424s.
- Left/right bias pattern: underprediction-dominated; under=3706 (0.922349), over=247 (0.0614734), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: light_all_seed1_veh_001354 with pred=94s, truth=367s, signed=-273s.
- Route-generation basis: ALL routes with Poisson headways, seed=1, expected 100 departures.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

### light_all_seed2

- Size: 3280 vehicles.
- Unsorted paired metrics: bias=-9.0003s, MAE=12.0875s, RMSE=27.0119s, p95 abs=87.05s.
- Sorted distribution metrics: sortedMAE=9.11738s, sortedRMSE=9.37233s, diffMean=-9.0003s.
- Left/right bias pattern: underprediction-dominated; under=3031 (0.924085), over=204 (0.0621951), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: light_all_seed2_veh_001192 with pred=359s, truth=113s, signed=246s.
- Route-generation basis: ALL routes with Poisson headways, seed=2, expected 100 departures.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

### medium_core

- Size: 2646 vehicles.
- Unsorted paired metrics: bias=-6.50756s, MAE=7.23394s, RMSE=12.7232s, p95 abs=10s.
- Sorted distribution metrics: sortedMAE=6.50756s, sortedRMSE=6.66981s, diffMean=-6.50756s.
- Left/right bias pattern: underprediction-dominated; under=2500 (0.944822), over=110 (0.0415722), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: medium_core_veh_000752 with pred=110s, truth=207s, signed=-97s.
- Route-generation basis: CORE first 21 routes, period=80.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

### east_bottleneck

- Size: 2672 vehicles.
- Unsorted paired metrics: bias=-7.73428s, MAE=7.73428s, RMSE=11.884s, p95 abs=12s.
- Sorted distribution metrics: sortedMAE=7.73428s, sortedRMSE=8.54637s, diffMean=-7.73428s.
- Left/right bias pattern: underprediction-dominated; under=2672 (1), over=0 (0), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: east_bottleneck_veh_001369 with pred=56s, truth=161s, signed=-105s.
- Route-generation basis: EAST_IN routes starting from -E5/-E4, period=30.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

### south_bottleneck

- Size: 2672 vehicles.
- Unsorted paired metrics: bias=-5.70284s, MAE=6.07784s, RMSE=10.1226s, p95 abs=9s.
- Sorted distribution metrics: sortedMAE=5.73428s, sortedRMSE=6.42139s, diffMean=-5.70284s.
- Left/right bias pattern: underprediction-dominated; under=2363 (0.884356), over=253 (0.0946856), severe under/over=0/0.
- Truth-duration-bin pattern: see `full_no_change_truth_duration_bins_by_group.csv`; high-bin cliff diagnosis is summarized in the cliff CSV.
- Cliff counts: under=0 (0), over=0 (0).
- Worst vehicle: south_bottleneck_veh_002655 with pred=46s, truth=147s, signed=-101s.
- Route-generation basis: SOUTH_IN routes starting from -E3/-E19, period=30.
- Possible cause: Demand is light or less conflicting, so CAMS and SUMO stay close and cliff behavior is absent.
- Recommendation: Keep as regression guard; do not tune globally from this group alone.

## 1_rou analysis

- Group `all_1_rou`: 483 vehicles; bias=-99.6687s, MAE=138.273s, RMSE=232.891s; under cliffs=5, over cliffs=0; worst vehicle=veh_188.
- This legacy fixed-route-style dataset is smaller than full_no_change and is underprediction-biased, but has no >1000s absolute-error tail in the committed summary.

## Sorted vs unsorted interpretation

- Unsorted metrics compare each CAMS vehicle against the corresponding SUMO vehicle and expose wrong per-vehicle delay assignment.
- Sorted metrics compare independently sorted predicted/truth durations; when sorted metrics are much better than unsorted metrics, CAMS may reproduce the distribution but assign delay to the wrong vehicles.

## Left/right bias interpretation

- Underprediction-dominated groups: oversat_all, west_bottleneck, southwest_bottleneck, north_bottleneck, and most light/free groups by count.
- Mixed-tail group: oversat_all has both underprediction and overprediction cliffs, so aggregate negative bias hides false-congestion cases.
- No group outside oversat_all has >1000s severe tails in this run.

## Cliff / 断崖式误差 diagnosis

- Underprediction cliffs mean SUMO has long waiting/congestion while CAMS predicts short travel times. They appear primarily in oversat_all and smaller forms in `1_rou`.
- Overprediction cliffs mean CAMS creates long waiting/downstream blocking absent in SUMO. They appear in oversat_all only in the full_no_change summary.
- Top route patterns and depart bins are in the route-pattern and cliff CSVs; they should be used to choose a small set of movement-level diagnostics rather than enabling global timelines.

## Route-generation-based possible causes

- `generate_route_on_test_net_full.py` defines 41 expanded valid routes, broad route families (ALL/CORE/WEST_IN/NORTH_IN/EAST_IN/SOUTH_IN/SOUTHWEST_IN), and staged scenarios with separated time windows.
- `oversat_all` uses all routes every 30s, making it the clearest network-wide saturation stress and the likely source of queue-spillback mismatch.
- Directional bottlenecks restrict starts to corridor families; west/southwest/north groups likely expose directional queue propagation and downstream-blocking gaps.
- `free_all`, light seeds, medium_core, east_bottleneck, south_bottleneck, and heavy_all are useful controls because they have small errors under the same simulator configuration.

## Recommendations

- Keep group-aware reporting in future evaluations.
- Prioritize oversat_all diagnostics and separate its underprediction and overprediction tails.
- Separately diagnose west and southwest bottleneck underprediction.
- Avoid using aggregate ALL metrics alone.
- Do not globally tune lane-discharge interval solely from ALL metrics.
- Consider targeted movement/route-level diagnostics only for specific groups and top route patterns.

## Files committed and PR safety

- Raw `sumo_eval.csv` files are not committed.
- Raw `prediction_tripinfo.xml` files are not committed.
- `run.log` files are not committed.
- Diagnostic directories are not committed.
- Only source changes, compact CSV/Markdown summaries, and this report are intended for commit.
