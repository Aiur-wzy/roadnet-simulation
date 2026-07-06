# Extreme group route/offset diagnosis

## 1. Executive summary
- `oversat_all` is the dominant extreme-error group: its worst route patterns include both SUMO-long/CAMS-short underprediction cliffs and CAMS-long/SUMO-short overprediction cliffs.
- The most severe underprediction pattern is `-E3 -> -E15 -> -E16 -> -E17`, with 246 undercliffs and MAE about 1664 s; this route does not pass the modified signals, so offsets alone cannot explain the worst cliff.
- Other severe underprediction routes do pass modified offsets, especially `-E5 -> -E13 -> E12 -> -E11 -> -E18` and `-E2 -> -E13 -> E12 -> -E11 -> -E17`, both exposed to J17/J21/J22.
- Overprediction cliffs in `oversat_all` are concentrated on `-E19 -> E15 -> E4` and `E10 -> -E12 -> E13 -> E5`; these are candidate CAMS false-blocking cases because CAMS predicts very long durations while SUMO truth is short.
- `west_bottleneck`, `southwest_bottleneck`, and `north_bottleneck` are primarily stable underprediction groups rather than cliff-dominated groups.
- Modified-signal exposure is common in both quiet and high-error groups; exposure is therefore an association, not proof of offset causality.
- The next trace should prioritize oversat undercliffs, then oversat overcliffs, then the worst directional bottleneck routes that pass J17/J21/J22.

## 2. Scope and inputs
- Dataset: `full_no_change`.
- Network: `data/test.net.xml`.
- Route: `data/full_no_change.rou.xml`.
- SUMO truth tripinfo: `data/tripinfo_full_no_change.xml`.
- CAMS/eval inputs generated locally under `diag_extreme_full_no_change/` and not committed.
- Route generator inspected: `generate_route_on_test_net_full.py`.
- Modified offsets expected and parsed: J17=45, J21=36, J22=72.

## 3. Validation
- XML parse check passed for network, route, and truth tripinfo.
- The analysis script validates eval vehicles against route XML, truth tripinfo, prediction tripinfo, and prediction duration consistency.
- Raw eval and prediction files are intentionally excluded from the committed outputs.

## 4. High-error group overview
|group|vehicles|MAE|bias|under cliffs|over cliffs|modified exposure|J17|J21|J22|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|east_bottleneck|2672|7.7|-7.7|0|0|0.75|0.12|0.50|0.75|
|free_all|2050|7.1|-6.6|0|0|0.80|0.46|0.51|0.49|
|heavy_all|8241|10.6|-6.9|0|0|0.80|0.46|0.51|0.49|
|north_bottleneck|2672|50.5|-50.2|0|0|1.00|0.25|0.88|0.75|
|oversat_all|13694|382.2|-318.4|1776|278|0.80|0.46|0.51|0.49|
|south_bottleneck|2672|6.1|-5.7|0|0|0.50|0.12|0.25|0.38|
|southwest_bottleneck|2672|171.3|-170.2|0|0|0.75|0.75|0.50|0.25|
|west_bottleneck|3006|148.7|-141.7|0|0|1.00|1.00|0.44|0.33|

## 5. Route-pattern findings
### Worst route patterns by MAE
|group|route|MAE|bias|under cliffs|over cliffs|modified signals|top vehicle|
|---|---|---:|---:|---:|---:|---|---|
|oversat_all|`-E3 -> -E15 -> -E16 -> -E17`|1663.9|-1663.6|246|0|none|oversat_all_veh_004825|
|oversat_all|`-E4 -> -E15 -> -E16 -> -E18`|1422.8|-1422.8|67|0|none|oversat_all_veh_002607|
|oversat_all|`-E5 -> -E13 -> E12 -> -E11 -> -E18`|1285.8|-1285.8|208|0|J17;J21;J22|oversat_all_veh_002521|
|oversat_all|`-E2 -> -E13 -> E12 -> -E11 -> -E17`|999.7|-999.7|135|0|J17;J21;J22|oversat_all_veh_001246|
|oversat_all|`-E19 -> -E16 -> E11 -> -E10`|911.8|-911.8|197|0|J17|oversat_all_veh_006347|
|oversat_all|`-E19 -> -E16 -> -E17`|620.1|-620.1|94|0|none|oversat_all_veh_006385|
|oversat_all|`E10 -> -E12 -> E13 -> E5`|610.3|598.3|0|109|J17;J21;J22|oversat_all_veh_002590|
|oversat_all|`-E19 -> E15 -> E4`|562.7|562.6|0|169|none|oversat_all_veh_009338|
### Undercliff route concentration
- `-E3 -> -E15 -> -E16 -> -E17` in `oversat_all` has 246 undercliffs, MAE 1663.9, modified exposure `none`.
- `-E3 -> -E15 -> -E16 -> -E17` in `ALL` has 246 undercliffs, MAE 509.4, modified exposure `none`.
- `-E5 -> -E13 -> E12 -> -E11 -> -E18` in `oversat_all` has 208 undercliffs, MAE 1285.8, modified exposure `J17;J21;J22`.
- `-E5 -> -E13 -> E12 -> -E11 -> -E18` in `ALL` has 208 undercliffs, MAE 355.7, modified exposure `J17;J21;J22`.
- `-E19 -> -E16 -> E11 -> -E10` in `oversat_all` has 197 undercliffs, MAE 911.8, modified exposure `J17`.
### Overcliff route concentration
- `-E19 -> E15 -> E4` in `oversat_all` has 169 overcliffs, MAE 562.7, modified exposure `none`.
- `-E19 -> E15 -> E4` in `ALL` has 169 overcliffs, MAE 177.0, modified exposure `none`.
- `E10 -> -E12 -> E13 -> E5` in `oversat_all` has 109 overcliffs, MAE 610.3, modified exposure `J17;J21;J22`.
- `E10 -> -E12 -> E13 -> E5` in `ALL` has 109 overcliffs, MAE 187.1, modified exposure `J17;J21;J22`.
Route families come from the generator: `ALL` spans all expanded routes; `WEST_IN`, `NORTH_IN`, `EAST_IN`, `SOUTH_IN`, and `SOUTHWEST_IN` restrict by entry direction.

## 6. Depart-time findings
### oversat_all
- Bin 60300-60600: vehicles 410, MAE 735.3, bias -605.6, undercliffs 125, overcliffs 20, top oversat_all_veh_003181.
- Bin 60000-60300: vehicles 410, MAE 732.3, bias -602.4, undercliffs 97, overcliffs 20, top oversat_all_veh_002812.
- Bin 59700-60000: vehicles 410, MAE 720.1, bias -594.2, undercliffs 114, overcliffs 16, top oversat_all_veh_002607.
### west_bottleneck
- Bin 71100-71400: vehicles 90, MAE 204.4, bias -152.2, undercliffs 0, overcliffs 0, top west_bottleneck_veh_000402.
- Bin 71400-71700: vehicles 90, MAE 194.8, bias -161.4, undercliffs 0, overcliffs 0, top west_bottleneck_veh_000483.
- Bin 70800-71100: vehicles 90, MAE 192.7, bias -146.2, undercliffs 0, overcliffs 0, top west_bottleneck_veh_000315.
### southwest_bottleneck
- Bin 125700-126000: vehicles 80, MAE 197.2, bias -196.3, undercliffs 0, overcliffs 0, top southwest_bottleneck_veh_002067.
- Bin 122100-122400: vehicles 80, MAE 190.8, bias -190.0, undercliffs 0, overcliffs 0, top southwest_bottleneck_veh_001132.
- Bin 123900-124200: vehicles 80, MAE 188.9, bias -187.9, undercliffs 0, overcliffs 0, top southwest_bottleneck_veh_001603.
### north_bottleneck
- Bin 85200-85500: vehicles 80, MAE 61.7, bias -61.5, undercliffs 0, overcliffs 0, top north_bottleneck_veh_000897.
- Bin 85500-85800: vehicles 80, MAE 59.3, bias -58.9, undercliffs 0, overcliffs 0, top north_bottleneck_veh_000969.
- Bin 85800-86100: vehicles 80, MAE 57.5, bias -57.2, undercliffs 0, overcliffs 0, top north_bottleneck_veh_001031.
The oversat bins fall inside the staged 58000-68000 s oversaturation window. Directional bins fall inside their generator windows: west 70000-80000, north 82000-92000, southwest 118000-128000.

## 7. Cliff vehicle findings
Underprediction cliffs are SUMO-long/CAMS-short. Overprediction cliffs are CAMS-long/SUMO-short.
### Top underprediction cliffs
- oversat_all_veh_002607 (oversat_all), depart 59890.0, route `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 8284.0, error -8240.0, modified `none`, label `oversat_under_cliff_all_corridor`.
- oversat_all_veh_002812 (oversat_all), depart 60040.0, route `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 8017.0, error -7944.0, modified `none`, label `oversat_under_cliff_all_corridor`.
- oversat_all_veh_002648 (oversat_all), depart 59920.0, route `-E4 -> -E15 -> -E16 -> -E18`, pred 102.0, truth 8008.0, error -7906.0, modified `none`, label `oversat_under_cliff_all_corridor`.
- oversat_all_veh_002853 (oversat_all), depart 60070.0, route `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 7929.0, error -7885.0, modified `none`, label `oversat_under_cliff_all_corridor`.
- oversat_all_veh_001951 (oversat_all), depart 59410.0, route `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 7918.0, error -7845.0, modified `none`, label `oversat_under_cliff_all_corridor`.
### Top overprediction cliffs
- oversat_all_veh_002590 (oversat_all), depart 59890.0, route `E10 -> -E12 -> E13 -> E5`, pred 1851.0, truth 108.0, error 1743.0, modified `J17;J21;J22`, label `oversat_over_cliff_false_blocking`.
- oversat_all_veh_003082 (oversat_all), depart 60250.0, route `E10 -> -E12 -> E13 -> E5`, pred 1851.0, truth 109.0, error 1742.0, modified `J17;J21;J22`, label `oversat_over_cliff_false_blocking`.
- oversat_all_veh_004189 (oversat_all), depart 61060.0, route `E10 -> -E12 -> E13 -> E5`, pred 1851.0, truth 109.0, error 1742.0, modified `J17;J21;J22`, label `oversat_over_cliff_false_blocking`.
- oversat_all_veh_002713 (oversat_all), depart 59980.0, route `E10 -> -E12 -> E13 -> E5`, pred 1851.0, truth 112.0, error 1739.0, modified `J17;J21;J22`, label `oversat_over_cliff_false_blocking`.
- oversat_all_veh_004681 (oversat_all), depart 61420.0, route `E10 -> -E12 -> E13 -> E5`, pred 1851.0, truth 112.0, error 1739.0, modified `J17;J21;J22`, label `oversat_over_cliff_false_blocking`.

## 8. Offset / initial phase association analysis
- High-error routes are mixed: several worst undercliff routes have no modified-signal exposure, while several others traverse J17/J21/J22.
- Quiet groups also have substantial modified-signal exposure, so offset exposure alone is unlikely to explain the error.
- Initial-phase outputs use XML phase states and `(time + offset) % cycle` as a conservative approximation; this should be treated as a hypothesis generator only.
- Recommended movement combinations for trace are the modified-signal movements on the J17/J21/J22-exposed undercliff routes plus the J17/J21/J22-exposed overcliff route `E10 -> -E12 -> E13 -> E5`.

## 9. SUMO vs CAMS divergence hypotheses
### A. SUMO long queue / CAMS short travel time
Supporting evidence: very large negative errors on routes with truth durations in the thousands of seconds but predictions below a few hundred seconds. Affected routes include `-E3 -> -E15 -> -E16 -> -E17`, `-E4 -> -E15 -> -E16 -> -E18`, and `-E5 -> -E13 -> E12 -> -E11 -> -E18`. Missing data: movement-level queue/spillback state. Next diagnostic: targeted undercliff trace.
### B. CAMS false blocking / SUMO short travel time
Supporting evidence: positive cliff routes such as `E10 -> -E12 -> E13 -> E5` and `-E19 -> E15 -> E4` where predicted duration is hundreds/thousands of seconds while SUMO truth remains short. Missing data: block reason and downstream capacity state. Next diagnostic: targeted overcliff trace.
### C. Directional bottleneck stable underprediction
Supporting evidence: west/southwest/north bottlenecks have negative bias without many formal cliff cases. Missing data: per-movement discharge comparison. Next diagnostic: trace worst directional vehicles by route.
### D. Offset/initial-phase amplification
Supporting evidence: some high-MAE routes traverse J17/J21/J22 and have approximate early red/yellow/green states in phase CSVs. Counterevidence: worst route and many quiet controls have comparable or zero exposure. Next diagnostic: movement trace at modified-signal linkIndex values.

## 10. Specific concrete cases
- oversat_all_veh_002607 (oversat_all), depart 59890.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 8284.0, error -8240.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002812 (oversat_all), depart 60040.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 8017.0, error -7944.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002648 (oversat_all), depart 59920.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 102.0, truth 8008.0, error -7906.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002853 (oversat_all), depart 60070.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 7929.0, error -7885.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_001951 (oversat_all), depart 59410.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 7918.0, error -7845.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002566 (oversat_all), depart 59860.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 7914.0, error -7841.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002484 (oversat_all), depart 59800.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 7846.0, error -7802.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002525 (oversat_all), depart 59830.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 102.0, truth 7849.0, error -7747.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_001910 (oversat_all), depart 59380.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 102.0, truth 7847.0, error -7745.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_001992 (oversat_all), depart 59440.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 7749.0, error -7705.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002976 (oversat_all), depart 60160.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 7656.0, error -7612.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_003058 (oversat_all), depart 60220.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 7658.0, error -7585.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_003099 (oversat_all), depart 60250.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 44.0, truth 7580.0, error -7536.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_003017 (oversat_all), depart 60190.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 102.0, truth 7581.0, error -7479.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.
- oversat_all_veh_002935 (oversat_all), depart 60130.0, `-E4 -> -E15 -> -E16 -> -E18`, pred 73.0, truth 7489.0, error -7416.0, modified `none`; suspected `oversat_under_cliff_all_corridor`; trace signal state, queue position, and downstream blocking.

## 11. Recommendations
- Prioritize `oversat_all` undercliff traces.
- Separately trace `oversat_all` overcliff false-blocking cases.
- Trace west/southwest/north route patterns that pass modified signals.
- Avoid global interval tuning; these errors are route/time-window specific.
- Use group/route/time-window-aware reporting in future evaluations.
- Do not claim offset causality until targeted movement traces confirm it.

## 12. Files generated
- `docs/extreme_error_diagnosis_results/full_no_change_depart_time_error_bins.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_initial_phase_exposure_top_routes.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_initial_phase_exposure_top_vehicles.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_modified_signal_cliff_association.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_north_bottleneck_depart_bins.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_oversat_all_depart_bins.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_recommended_trace_vehicles.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_signal_offset_exposure_by_group.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_signal_offset_exposure_by_route.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_southwest_bottleneck_depart_bins.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_abs_error_vehicles.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_mae_route_patterns.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_overcliff_route_patterns.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_overcliff_vehicles.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_route_patterns_by_group.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_undercliff_route_patterns.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_top_undercliff_vehicles.csv`
- `docs/extreme_error_diagnosis_results/full_no_change_west_bottleneck_depart_bins.csv`
- `docs/extreme_group_route_offset_diagnosis.md`

## 13. PR safety
- Only compact script/report/CSV outputs are staged for commit. Raw `diag_*`, `build/`, logs, `sumo_eval.csv`, and `prediction_tripinfo.xml` are not committed.
