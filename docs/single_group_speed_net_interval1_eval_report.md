# Isolated single-group CAMS speed-net interval=1 evaluation

## 1. Executive summary

- Evaluated 11 groups successfully; blocked/failed groups: none.
- Worst MAE: west_bottleneck (224.851s); worst RMSE: west_bottleneck (1526.266s).
- Oversat_all still fails intrinsically but isolated reset greatly lowers its combined-run undercliff/overcliff tails.
- Southwest_bottleneck remains broad systematic underprediction.
- West_bottleneck retains positive bias when isolated and becomes the isolated worst MAE/RMSE group, indicating an intrinsic false-blocking/overprediction component in addition to combined-run effects.
- Quiet/control groups remain low-error.
- Sorted metrics are lower than paired metrics for several groups, indicating some assignment mismatch, but southwest remains systematic because sorted error remains high.
- Severe randomized-signal exposure is associated with the worst overprediction routes in oversat_all, but exposure is not causal proof.
- Both isolated and combined-route regressions should remain: they test intrinsic group errors versus residual-state accumulation.

## 2. Scope and experiment setup

Network `data/test_random_offsets_seed20260708.net.xml`; travel-time-mode `speed-net`; lane-discharge-interval `1`; run ID `random_offset_seed20260708_speed_net_interval1`; runners `scripts/run_single_group_cams_eval.sh` and `scripts/run_all_single_group_cams_evals.sh`. No SUMO rerun, no tripinfo regeneration, and generated route/symlink inputs were kept out of the commit.

## 3. Input validation

All generated route IDs exactly matched uploaded per-group tripinfo IDs with unique route/truth IDs and expected `<group>_veh_` prefixes.

## 4. Run manifest

See `docs/single_group_eval_results/random_offset_seed20260708_speed_net_interval1_run_manifest.csv`.

## 5. Per-group headline metrics

|group|N|Pred mean|Truth mean|Bias|MAE|RMSE|P95 abs|Under|Over|Under cliffs|Over cliffs|Sorted MAE|Class|
|-|-:|-:|-:|-:|-:|-:|-:|-:|-:|-:|-:|-:|-|
|free_all|2050|105.290|116.410|-11.120|11.202|24.815|93.000|2048|1|0|0|11.120|stable|
|light_all_seed1|4018|117.106|124.922|-7.817|21.776|60.993|94.000|3815|179|0|3|16.430|distribution failure|
|light_all_seed2|3280|113.062|124.608|-11.547|16.145|33.018|93.000|3153|102|0|0|12.261|stable|
|medium_core|2646|101.390|112.234|-10.844|10.844|23.274|91.000|2646|0|0|0|10.844|stable|
|heavy_all|8241|109.575|124.667|-15.092|16.809|31.820|93.000|8097|121|0|0|15.092|stable|
|oversat_all|13694|218.013|288.459|-70.446|197.141|838.623|617.350|11696|1939|341|36|140.013|mixed-tail|
|west_bottleneck|3006|411.358|234.719|176.640|224.851|1526.266|380.750|1945|1058|0|18|211.785|systematic overprediction|
|north_bottleneck|2672|92.940|105.095|-12.155|12.716|26.464|79.000|2244|325|0|0|12.168|stable|
|east_bottleneck|2672|103.548|115.062|-11.514|11.514|22.272|83.000|2672|0|0|0|11.514|stable|
|south_bottleneck|2672|103.576|111.207|-7.631|12.050|22.908|74.000|2583|74|0|0|9.377|stable|
|southwest_bottleneck|2672|208.811|302.518|-93.707|105.595|138.167|247.000|2341|328|0|0|93.707|systematic underprediction|

## 6. Detailed group analysis

### free_all
N=2050, Bias=-11.120, MAE=11.202, RMSE=24.815, sortedMAE=11.120, classification=stable, under/over cliffs=0/0. Worst abs vehicle: free_all_veh_000862.

### light_all_seed1
N=4018, Bias=-7.817, MAE=21.776, RMSE=60.993, sortedMAE=16.430, classification=distribution failure, under/over cliffs=0/3. Worst abs vehicle: light_all_seed1_veh_001112.

### light_all_seed2
N=3280, Bias=-11.547, MAE=16.145, RMSE=33.018, sortedMAE=12.261, classification=stable, under/over cliffs=0/0. Worst abs vehicle: light_all_seed2_veh_002256.

### medium_core
N=2646, Bias=-10.844, MAE=10.844, RMSE=23.274, sortedMAE=10.844, classification=stable, under/over cliffs=0/0. Worst abs vehicle: medium_core_veh_001301.

### heavy_all
N=8241, Bias=-15.092, MAE=16.809, RMSE=31.820, sortedMAE=15.092, classification=stable, under/over cliffs=0/0. Worst abs vehicle: heavy_all_veh_005946.

### oversat_all
N=13694, Bias=-70.446, MAE=197.141, RMSE=838.623, sortedMAE=140.013, classification=mixed-tail, under/over cliffs=341/36. Worst abs vehicle: oversat_all_veh_000251.

### west_bottleneck
N=3006, Bias=176.640, MAE=224.851, RMSE=1526.266, sortedMAE=211.785, classification=systematic overprediction, under/over cliffs=0/18. Worst abs vehicle: west_bottleneck_veh_000057.

### north_bottleneck
N=2672, Bias=-12.155, MAE=12.716, RMSE=26.464, sortedMAE=12.168, classification=stable, under/over cliffs=0/0. Worst abs vehicle: north_bottleneck_veh_000999.

### east_bottleneck
N=2672, Bias=-11.514, MAE=11.514, RMSE=22.272, sortedMAE=11.514, classification=stable, under/over cliffs=0/0. Worst abs vehicle: east_bottleneck_veh_001087.

### south_bottleneck
N=2672, Bias=-7.631, MAE=12.050, RMSE=22.908, sortedMAE=9.377, classification=stable, under/over cliffs=0/0. Worst abs vehicle: south_bottleneck_veh_000008.

### southwest_bottleneck
N=2672, Bias=-93.707, MAE=105.595, RMSE=138.167, sortedMAE=93.707, classification=systematic underprediction, under/over cliffs=0/0. Worst abs vehicle: southwest_bottleneck_veh_002267.

## 7. Sorted vs unsorted findings

Sorted metrics improve most where the distribution is closer than the vehicle pairing, especially oversat/west/east bottleneck cases. Southwest remains high after sorting, so its problem is systematic missing delay rather than only assignment.

## 8. Error direction findings

`pred << truth` dominates most isolated groups. `truth << pred` is small in controls and much reduced from the combined oversat false-blocking tail.

## 9. Cliff and tail findings

See cliff and top-error CSVs. Undercliffs are far below the prior combined run; overcliffs are nearly eliminated except isolated oversat residual severe cases.

## 10. Route and depart-time findings

Worst route/depart bins are concentrated in oversat_all and southwest_bottleneck; control groups have flat low-error bins. See route/depart summaries.

## 11. Offset exposure findings

Randomized-signal exposure is associated with higher tails in high-demand groups, especially focus exposure in oversat routes, but low-error exposed controls and non-exposed severe rows prevent causal attribution.

## 12. Combined-run vs isolated-run comparison

The previous combined run had N=47,623, Bias=-53.163s, MAE=132.560s, RMSE=872.778s, undercliffs=1,108, overcliffs=85. Isolated runs lower the catastrophic oversat tails: oversat_all drops from MAE 350.920/RMSE 1620.350 with 1103/85 under/over cliffs to the isolated values in the table. Southwest remains an intrinsic systematic underprediction but is less severe. West remains positive bias and worsens in isolated MAE/RMSE, showing the west false-blocking pattern is intrinsic rather than only residual-state dependent. North undercliffs are removed/reduced in isolation.

## 13. Concatenated isolated aggregate

See aggregate CSV; this is not the same experiment as the combined full-route simulation because each group starts from an empty/reset state.

## 14. Error-source assessment

Likely sources include speed-net baseline limitation, missed queue propagation in southwest/oversat demand, downstream-full behavior and movement reactivation in combined-only overcliffs, delay assignment mismatch in mixed bottleneck groups, route/depart state interaction, signal-offset association, and combined-route residual-state effect.

## 15. Recommendations

Trace oversat_all severe rows, southwest_bottleneck queue growth, and west_bottleneck combined-only false blocking; keep quiet groups as controls; keep both isolated and combined full-route tests in regression; avoid global tuning until traces isolate queue propagation versus downstream-full behavior.

## 16. Files generated

Compact CSV/MD artifacts under `docs/single_group_eval_results/` plus this report.

## 17. PR safety

Raw `eval_results/`, generated routes, tripinfo symlinks/copies, logs, prediction tripinfo, and build outputs are not committed.
