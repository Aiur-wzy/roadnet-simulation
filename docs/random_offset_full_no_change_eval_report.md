# Random-offset full no-change CAMS evaluation report

## 1. Executive summary

- The randomized-offset-network baseline is mixed: MAE improves slightly versus the old original-net baseline (132.560s vs 136.862s) and median/P90/P95 absolute errors improve (9/317/503s vs 8/336/676s), but RMSE worsens sharply (872.778s vs 413.313s) because a smaller set of CAMS overprediction failures now has extreme durationError up to +39,585s.
- SUMO truth became faster on average in the randomized-offset scenario (222.061s vs 243.96s), while CAMS became slower on average (168.898s vs 127.02s). This is an old original-net baseline vs new randomized-offset-network baseline comparison, not a same-truth calibration comparison.
- Missed congestion / `pred << truth` remains the dominant direction: 43,163 vehicles underpredict, 755 vehicles exceed 1000s underprediction, and 1,108 vehicles meet undercliff criteria.
- False blocking / `truth << pred` increases in count relative to the old original-net baseline direction count (4,322 overpredictions vs 3,828), but severe overprediction is concentrated: 57 vehicles exceed +1000s and 85 meet overcliff criteria.
- `oversat_all` dominates severe error: it contributes 1,103 of 1,108 undercliffs, all 85 overcliffs, all 57 severe overpredictions, and 755 severe underpredictions.
- `southwest_bottleneck` and `west_bottleneck` are the next most important non-oversat groups: southwest is pure underprediction with MAE 171.582s; west is mixed/overprediction-leaning with positive bias +73.918s and MAE 159.952s.
- Sorted and unsorted metrics tell different stories only for the hardest groups. Most quiet/control groups have both low sorted and low unsorted error. `oversat_all` has both very poor sorted and unsorted metrics, indicating that the distribution itself is wrong, not merely assigned to the wrong vehicles. `west_bottleneck` has sorted MAE lower than unsorted MAE, indicating partial distribution reproduction but wrong vehicle-level delay assignment.
- Top undercliff routes shifted away from the old worst non-modified route `-E3 -> -E15 -> -E16 -> -E17`; the new top undercliff routes include `-E2 -> -E13 -> E12 -> -E11 -> -E17` and `-E5 -> -E13 -> E12 -> -E11 -> -E18`.
- Top false-blocking routes shifted: `E10 -> -E12 -> E13 -> E5` no longer appears as the top overcliff family; the new worst false-blocking cases are `E9 -> -E11 -> E16 -> E15 -> E4` and `E9 -> -E11 -> -E18`, with extreme predicted durations near 39,700s.
- Offset exposure is moderately associated with severe errors, but not causal by itself. Severe `oversat_all` exposed vehicles have higher MAE and cliff rates than non-exposed vehicles, yet quiet groups also commonly traverse modified/randomized signals without cliffs.
- Recommended next traces: overprediction false-blocking vehicles `oversat_all_veh_000454`, `oversat_all_veh_000372`, `oversat_all_veh_000497`; missed-congestion vehicle `oversat_all_veh_007523`; route families `E9 -> -E11 -> E16 -> E15 -> E4`, `E9 -> -E11 -> -E18`, `-E2 -> -E13 -> E12 -> -E11 -> -E17`, and `-E5 -> -E13 -> E12 -> -E11 -> -E18` in depart bins around 58,200-58,500s and 60,000-66,000s.

## 2. Scope and inputs

- Network: `data/test_random_offsets_seed20260708.net.xml`.
- Route file: `data/full_no_change.rou.xml`.
- Truth tripinfo used: `data/tripinfo_full_no_change_random_net.xml`. The user-requested path `data/tripinfo_no_change_random_net.xml` was not present in this checkout; the matching regenerated random-net tripinfo present in `data/` was used instead.
- CAMS command used `--travel-time-mode speed-net` and `--lane-discharge-interval 1`.
- Prediction tripinfo was written locally to `diag_random_offset_full_no_change/prediction_tripinfo.xml` and was not committed.
- No SUMO run, tripinfo regeneration, network XML edit, route XML edit, or tripinfo XML edit was performed.

## 3. Input validation

- XML parsing succeeded for the random-offset net, route file, and random-net tripinfo.
- Route/tripinfo coverage was complete: 47,623 route vehicles, 47,623 tripinfo records, 0 missing tripinfo records, and 0 tripinfo records outside routes.
- Prediction tripinfo consistency check from the grouped analyzer reported `missing=0 extra=0 mismatchedDurations=0`.
- Randomized offsets matched the expected values: J17=4, J20=68, J21=4, J22=54, J24=7, J25=47; each program has 8 phases and a 90s cycle.
- tlLogic state length validation passed.

## 4. Dataset-level metrics

### Current randomized-offset run

| Metric | Value |
|---|---:|
| simulatedVehicles | 47,623 |
| truthVehicles | 47,623 |
| comparedVehicles | 47,623 |
| missingTruth / invalidSkipped / etaMissingSkipped / truthNotSimulated | 0 / 0 / 0 / 0 |
| meanPredDuration | 168.898s |
| meanTruthDuration | 222.061s |
| Bias (`pred - truth`) | -53.163s |
| MAE / RMSE / MAPE | 132.560s / 872.778s / 39.157% |
| Median / P75 / P90 / P95 / P99 abs error | 9s / 96s / 317s / 503s / 1391.78s |
| Arrival MAE / RMSE / Bias | 2148.74s / 6374.90s / +257.839s |
| meanPredSpeed / meanTruthSpeed | 4.819 / 3.640 |
| speedBias / speedMAE | +1.180 / 1.349 |
| absDurationError > 60 / 120 / 300 / 600 / 1000 | 16,995 / 10,692 / 5,032 / 1,825 / 812 |
| relativeDurationError > 0.5 / 1.0 / 3.0 / 5.0 | 12,084 / 945 / 115 / 69 |
| predDuration > 1000 / > 2000 | 59 / 57 |
| truthDuration > 1000 / > 2000 | 936 / 119 |
| durationError > 0 / < 0 / == 0 | 4,322 / 43,163 / 138 |
| sorted duration MAE / RMSE | 94.762s / 781.138s |

### Old original-net baseline vs new randomized-offset-network baseline

Caveat: the network offsets and SUMO truth changed, so this is a scenario comparison, not same-truth calibration.

| Metric | Old original-net baseline | New randomized-offset baseline | Direction |
|---|---:|---:|---|
| comparedVehicles | 47,623 | 47,623 | same |
| meanPredDuration | 127.020s | 168.898s | CAMS slower |
| meanTruthDuration | 243.960s | 222.061s | SUMO truth faster |
| Bias | -116.940s | -53.163s | less under-biased |
| MAE | 136.862s | 132.560s | slightly better |
| RMSE | 413.313s | 872.778s | much worse tail |
| MAPE | 33.634% | 39.157% | worse relative error |
| Median abs error | 8s | 9s | slightly worse center |
| P90 / P95 abs error | 336s / 676s | 317s / 503s | better upper quantiles |
| absDurationError > 1000 | 1,323 | 812 | fewer severe absolute errors |
| predDuration > 2000 | 0 | 57 | new extreme CAMS false-blocking tail |
| truthDuration > 2000 | 444 | 119 | fewer extreme SUMO truth durations |
| durationError > 0 | 3,828 | 4,322 | more overprediction count |
| durationError < 0 | 43,186 | 43,163 | essentially unchanged underprediction count |

## 5. Group-level detailed analysis

| Group | N | Pred mean | Truth mean | Bias | MAE | RMSE | MAPE | Med/P90/P95/P99 abs | Under / Over | Severe under / over | Under / over cliffs | Sorted MAE/RMSE | Worst vehicle | Interpretation and recommendation |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|---|---|---|---|
| ALL | 47,623 | 168.898 | 222.061 | -53.163 | 132.560 | 872.778 | 39.157% | 9 / 317 / 503 / 1391.78 | 43,163 / 4,322 | 755 / 57 | 1,108 / 85 | 94.762 / 781.138 | oversat_all_veh_000454 (+39,585s) | Underprediction-dominated overall, with a small but catastrophic overprediction tail. Trace both oversat false-blocking and oversat missed-congestion routes. |
| east_bottleneck | 2,672 | 104.068 | 110.351 | -6.283 | 18.134 | 32.510 | 16.331% | 7 / 82 / 85 / 87 | 2,467 / 205 | 0 / 0 | 0 / 0 | 11.896 / 13.346 | east_bottleneck_veh_000456 (-95s) | Quiet underprediction-dominated control; sorted and unsorted both good. No urgent trace. |
| free_all | 2,050 | 105.290 | 116.410 | -11.120 | 11.202 | 24.815 | 9.320% | 5 / 10 / 93 / 96 | 2,048 / 1 | 0 / 0 | 0 / 0 | 11.120 / 11.785 | free_all_veh_000862 (-184s) | Stable control; almost pure mild underprediction. |
| heavy_all | 8,241 | 109.638 | 125.235 | -15.597 | 17.436 | 32.549 | 13.241% | 7 / 81 / 93 / 96 | 8,076 / 140 | 0 / 0 | 0 / 0 | 15.597 / 16.054 | heavy_all_veh_002214 (-101s) | Stable underprediction-dominated high-volume control; no cliffs. |
| light_all_seed1 | 4,018 | 113.946 | 124.185 | -10.239 | 20.480 | 53.473 | 15.330% | 6 / 83 / 94 / 189.66 | 3,875 / 125 | 0 / 0 | 0 / 0 | 15.778 / 30.834 | light_all_seed1_veh_001235 (+625s) | Mostly underprediction with a mild overprediction tail. Monitor but not a primary target. |
| light_all_seed2 | 3,280 | 110.327 | 121.887 | -11.559 | 16.583 | 34.931 | 12.743% | 6 / 77.1 / 93 / 162 | 3,172 / 92 | 0 / 0 | 0 / 0 | 13.060 / 14.864 | light_all_seed2_veh_000785 (+265s) | Quiet mixed-light group; no severe tail. |
| medium_core | 2,646 | 101.387 | 112.365 | -10.979 | 10.979 | 23.543 | 9.293% | 6 / 10 / 91 / 95 | 2,646 / 0 | 0 / 0 | 0 / 0 | 10.979 / 12.905 | medium_core_veh_001381 (-99s) | Pure mild underprediction; stable. |
| north_bottleneck | 2,672 | 97.606 | 147.530 | -49.924 | 49.924 | 114.439 | 22.969% | 8 / 150.9 / 191 / 616.16 | 2,642 / 0 | 0 / 0 | 5 / 0 | 49.924 / 84.280 | north_bottleneck_veh_000783 (-906s) | Underprediction-dominated with a small undercliff tail; trace only after oversat/southwest. |
| oversat_all | 13,694 | 265.642 | 399.421 | -133.779 | 350.920 | 1620.350 | 84.419% | 134 / 716 / 1144 / 2005 | 11,413 / 2,248 | 755 / 57 | 1,103 / 85 | 278.446 / 1463.380 | oversat_all_veh_000454 (+39,585s) | Primary failure group. Both duration distribution and vehicle pairing are wrong. Trace false blocking and missed congestion separately. |
| south_bottleneck | 2,672 | 96.666 | 116.842 | -20.176 | 20.176 | 34.642 | 17.099% | 8 / 74 / 93 / 95 | 2,672 / 0 | 0 / 0 | 0 / 0 | 20.176 / 21.453 | south_bottleneck_veh_002072 (-99s) | Pure mild underprediction; quiet. |
| southwest_bottleneck | 2,672 | 135.879 | 307.461 | -171.582 | 171.582 | 205.152 | 47.997% | 185 / 315 / 341 / 411.29 | 2,672 / 0 | 0 / 0 | 0 / 0 | 171.582 / 191.862 | southwest_bottleneck_veh_001556 (-435s) | Persistent missed-congestion group: no cliffs, but broad systematic underprediction. Trace route/depart bins after oversat cliffs. |
| west_bottleneck | 3,006 | 345.361 | 271.443 | +73.918 | 159.952 | 217.293 | 57.747% | 116 / 395 / 473 / 556 | 1,480 / 1,511 | 0 / 0 | 0 / 0 | 110.497 / 178.246 | west_bottleneck_veh_001275 (+638s) | Mixed-tail/overprediction-leaning. Sorted improves versus unsorted, so delay distribution is partly reproduced but assigned to wrong vehicles. |

## 6. Sorted vs unsorted analysis

- Unsorted paired metrics measure per-vehicle matching quality; sorted distribution metrics measure whether CAMS reproduces the duration distribution independent of identity.
- Quiet groups (`free_all`, `heavy_all`, `medium_core`, `east_bottleneck`, `south_bottleneck`, light groups) have low sorted and unsorted error, so they are stable controls.
- `oversat_all` has unsorted MAE/RMSE 350.920/1620.350s and sorted MAE/RMSE 278.446/1463.380s. Both are bad, so the duration distribution itself is wrong.
- `west_bottleneck` has unsorted MAE 159.952s but sorted MAE 110.497s. CAMS partly reproduces the distribution but assigns delay to the wrong vehicles.
- `southwest_bottleneck` has identical bias/MAE because every vehicle is underpredicted; sorted remains high at 171.582s, indicating systematic missing delay.

## 7. Error direction analysis

### A. `pred << truth`: CAMS too fast / missed SUMO congestion / underprediction

- Dataset-wide underprediction dominates: 43,163 underpredictions and 755 severe underpredictions >1000s.
- `oversat_all` is the severe source: 11,413 underpredictions, 755 severe underpredictions, and 1,103 undercliffs.
- `southwest_bottleneck` is pure systematic underprediction with MAE 171.582s but no formal cliffs.
- `north_bottleneck` is pure/near-pure underprediction and has 5 undercliffs.
- Control groups are underprediction-dominated but low magnitude.

### B. `truth << pred`: CAMS too slow / possible false blocking / false waiting / downstream-full artifact / overprediction

- Dataset-wide overprediction count increased to 4,322, with 57 severe overpredictions and 85 overcliffs.
- All overcliffs are in `oversat_all`; the worst vehicle is `oversat_all_veh_000454` with pred=39,783s, truth=198s, durationError=+39,585s.
- `west_bottleneck` is the only non-oversat group with overprediction-leaning bias (+73.918s), but it has no >1000s severe overprediction and no cliffs.
- Top false-blocking routes now center on `E9 -> -E11 -> E16 -> E15 -> E4` and `E9 -> -E11 -> -E18`, not the previous `E10 -> -E12 -> E13 -> E5` example.

## 8. Cliff / severe-tail analysis

Definitions used: underprediction cliff if `truthDuration >= 900 and predDuration <= 300` or `durationError <= -1000`; overprediction cliff if `predDuration >= 900 and truthDuration <= 300` or `durationError >= 1000`.

- Dataset cliffs: 1,108 undercliffs and 85 overcliffs.
- `oversat_all`: 1,103 undercliffs and 85 overcliffs.
- `north_bottleneck`: 5 undercliffs and 0 overcliffs.
- All other groups: 0 cliffs.
- Top undercliff route families include `-E2 -> -E13 -> E12 -> -E11 -> -E17`, `-E5 -> -E13 -> E12 -> -E11 -> -E18`, `E9 -> -E12 -> E20 -> E15 -> -E0 -> E5`, `-E5 -> -E13 -> -E14`, and `-E3 -> -E0 -> -E13 -> -E14`.
- Top overcliff route families include `E9 -> -E11 -> E16 -> E15 -> E4`, `E9 -> -E11 -> -E18`, `E10 -> -E12 -> E13 -> E5`, `E18 -> E11 -> -E12 -> -E14`, and `E18 -> E11 -> -E12 -> E13 -> E5`.
- Top undercliff depart bins include `[60000,60300)`, `[65700,66000)`, `[62100,62400)`, `[61200,61500)`, and `[65400,65700)`.
- Top overcliff depart bins include `[58200,58500)`, `[58500,58800)`, `[62700,63000)`, `[63000,63300)`, and `[62400,62700)`.

## 9. Route/depart-time findings

- Severe errors are highly concentrated in `oversat_all`, which spans depart 58,000-67,990s and aligns with the high-flow oversaturation window.
- The new worst overprediction cases appear very early in the oversat window around 58,200-58,500s and produce prediction durations near 39,700s while SUMO truth stays near 136-198s.
- Missed-congestion undercliffs are concentrated later in the oversat window, especially around 60,000-66,000s.
- `southwest_bottleneck` has no cliffs but a broad systematic underprediction across its directional window; this looks like persistent missing queue/waiting rather than isolated false blocking.
- `west_bottleneck` has mixed under/over signs and lower sorted than unsorted error, suggesting route/depart assignment issues rather than only aggregate demand or global discharge error.

## 10. Offset/signal exposure analysis

- Randomized offsets: J17=4, J20=68, J21=4, J22=54, J24=7, J25=47.
- 81.57% of all vehicles traverse at least one previously modified focus signal J17/J21/J22; rates are 46.94% for J17, 52.08% for J21, and 49.51% for J22.
- `oversat_all` exposed vehicles have higher MAE than non-exposed vehicles (395.384s vs 167.505s), higher undercliff rate (9.245% vs 3.144%), and overcliffs only among exposed vehicles (0.771% vs 0%).
- However, quiet groups such as `free_all`, `heavy_all`, and both light groups also have about 80.49% modified-signal exposure with zero cliffs. Exposure alone is therefore insufficient as a causal explanation.
- Top false-blocking recommended traces are exposed to J17 plus all-signal combinations J17/J20 or J17/J20/J24/J25.
- Top undercliff routes are mixed: some traverse J17/J21/J22, while others do not. This weakens any claim that randomized offsets alone explain undercliffs.
- Association classification: **moderate** for overprediction false-blocking in `oversat_all` because overcliffs are exposed and concentrated in J17 routes; **weak-to-moderate** for underprediction because severe missed-congestion routes include both exposed and non-exposed families.

## 11. Comparison with original-net diagnosis

- Original-net `oversat_all` had 1,776 underprediction cliffs and 278 overprediction cliffs. The new randomized-offset run has 1,103 undercliffs and 85 overcliffs in `oversat_all`, so cliff counts decreased in both directions.
- The old worst underprediction route `-E3 -> -E15 -> -E16 -> -E17` with 246 undercliffs and no modified-signal exposure no longer appears as the dominant undercliff family. New top undercliff routes are led by `-E2 -> -E13 -> E12 -> -E11 -> -E17` and `-E5 -> -E13 -> E12 -> -E11 -> -E18`.
- The previously severe underprediction routes exposed to J17/J21/J22 remain relevant: `-E5 -> -E13 -> E12 -> -E11 -> -E18` and `-E2 -> -E13 -> E12 -> -E11 -> -E17` are still among the new top undercliff families.
- The old overprediction routes `-E19 -> E15 -> E4` and `E10 -> -E12 -> E13 -> E5` are no longer the only leading false-blocking pattern. `E10 -> -E12 -> E13 -> E5` still appears in the overcliff list, but the new worst route families are `E9 -> -E11 -> E16 -> E15 -> E4` and `E9 -> -E11 -> -E18`.
- The previous false-blocking example `oversat_all_veh_002590` on `E10 -> -E12 -> E13 -> E5` improved as a dominant pattern in this scenario, but the randomized offsets introduced or exposed a different J17-centered false-blocking route family with much larger individual prediction durations.
- SUMO truth congestion became less extreme at the dataset level: truthDuration >2000 decreased from 444 to 119 and mean truth duration decreased from 243.960s to 222.061s.
- CAMS false-blocking count increased slightly (durationError >0: 3,828 to 4,322), but formal overcliffs decreased in `oversat_all` (278 to 85). Random offsets may have mitigated the previous broad false-blocking pattern while leaving a narrower catastrophic artifact that requires trace.
- Directional bottleneck behavior remains important: `southwest_bottleneck` is still a systematic missed-congestion group; `west_bottleneck` remains mixed/overprediction-leaning; `north_bottleneck` has only a small undercliff tail.

## 12. Recommendations

- Keep this random-offset network for further testing because it reduces many severe truth-duration and oversat cliff counts while revealing a narrower, easier-to-trace false-blocking artifact.
- Do not treat this as a final calibration improvement: RMSE worsened because the remaining false-blocking tail is extremely large.
- Target trace `oversat_all_veh_000454`, `oversat_all_veh_000372`, and `oversat_all_veh_000497` first for false blocking / downstream-full artifacts.
- Target trace `oversat_all_veh_007523` and route families `-E2 -> -E13 -> E12 -> -E11 -> -E17` and `-E5 -> -E13 -> E12 -> -E11 -> -E18` for missed SUMO congestion.
- Trace `southwest_bottleneck` separately as a systematic underprediction problem, not as a cliff-only problem.
- Avoid global lane-discharge interval tuning for this case; quiet/control groups are already stable and global tuning may degrade them.
- If future SUMO/tripinfo runs show deadlock-like behavior or a large rebound in truthDuration >2000, regenerate another offset candidate; this run does not show dataset-level deadlock-like truth behavior.

## 13. Files generated

Committed compact outputs:

- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_detailed_group_summary.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_sorted_distribution_by_group.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_left_right_bias_by_group.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_truth_duration_bins_by_group.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_cliff_summary_by_group.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_route_pattern_summary.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_detailed_group_report.md`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_depart_time_error_bins.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_top_*csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_signal_offset_exposure_by_*.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_initial_phase_exposure_top_*.csv`
- `docs/random_offset_eval_results/full_no_change_random_offset_seed20260708_recommended_trace_vehicles.csv`
- `docs/random_offset_full_no_change_eval_report.md`

## 14. PR safety

- No raw `sumo_eval.csv` committed.
- No raw `prediction_tripinfo.xml` committed.
- No run logs committed.
- No `diag_*` directory committed.
- No build outputs committed.
- No network XML, route XML, or tripinfo XML changed.
