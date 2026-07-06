# Full no-change kinematic evaluation summary

## 1. Experiment setup

- Route: `data/full_no_change.rou.xml`.
- Network: `data/test.net.xml`.
- SUMO truth tripinfo: `data/tripinfo_full_no_change.xml`.
- Primary travel-time mode: `--travel-time-mode kinematic` with default `--kinematic-congestion-alpha 1.0`.
- Optional sensitivity run: `--kinematic-congestion-alpha 0.5`, run because the default kinematic result had severe overprediction and the run was cheap.
- No production simulator logic, `data/test.net.xml`, training/catching/table lookup semantics, or evaluation semantics were modified.
- No verbose travel-time, split-extreme, or timeline debug options were enabled.

## 2. Build and validation status

- Initial `git status --short` was clean, and the starting commit was `9a69e68 Merge pull request #52 from Aiur-wzy/lji5ij-codex/sumo/cams`.
- Required files existed: `data/test.net.xml`, `data/full_no_change.rou.xml`, and `data/tripinfo_full_no_change.xml`.
- Signal offsets matched the expected current state: `J17=45`, `J21=36`, `J22=72`.
- Existing lane-capacity logic was inspected but not modified. `Graph::hasLaneStorageCapacity(...)` exists, `laneCapacity` is documented as the authoritative macro storage rule, and the formula is `floor((max(0.0, road.length) + vehicleGap) / vehicleSpace) + 1`.
- `cmake -S . -B build` completed successfully with a Boost-not-found warning; the build uses the `std::thread` fallback.
- `cmake --build build -j2` completed successfully.
- `data/test.net.xml` parsed successfully.
- tlLogic phase state lengths matched the controlled connection link-index counts.
- The `sumo` executable was unavailable in this environment, so the optional direct SUMO smoke command was not run.

## 3. Generated files

The default kinematic run generated only the existing simulator's standard evaluation outputs in `diag_full_kinematic/`:

- `diag_full_kinematic/eval_distribution_metrics.csv`
- `diag_full_kinematic/eval_extreme_duration_errors.csv`
- `diag_full_kinematic/eval_extreme_route_summary.csv`
- `diag_full_kinematic/eval_grouped_metrics.csv`
- `diag_full_kinematic/eval_summary.txt`
- `diag_full_kinematic/run.log`
- `diag_full_kinematic/sumo_eval.csv`

The alpha-0.5 sensitivity run generated the same standard output set under `diag_full_kinematic_alpha05/`.

## 4. Overall kinematic evaluation summary

### Default kinematic alpha = 1.0

| Metric | Value |
|---|---:|
| simulatedVehicles | 47,623 |
| truthVehicles | 47,623 |
| comparedVehicles | 47,623 |
| missingTruth | 0 |
| invalidSkipped | 0 |
| etaMissingSkipped | 0 |
| truthNotSimulated | 0 |
| meanPredDuration | 629.277 |
| meanTruthDuration | 214.285 |
| Bias | +414.992 |
| MAE | 529.316 |
| RMSE | 3,293.81 |
| MAPE | 383.948% |
| Median absolute error | 83 |
| P90 absolute error | 1,097 |
| P95 absolute error | 1,646 |
| MAE arrival | 11,907.3 |
| RMSE arrival | 25,624.3 |
| Bias arrival | +11,664.2 |
| meanPredSpeed | 2.9102 |
| meanTruthSpeed | 4.17272 |
| speedBias | -1.26251 |
| speedMAE | 2.0463 |

### Threshold counts, alpha = 1.0

| Threshold | Count |
|---|---:|
| absDurationError > 300 | 12,648 |
| absDurationError > 600 | 8,046 |
| absDurationError > 1000 | 5,665 |
| relativeDurationError > 1.0 | 16,770 |
| relativeDurationError > 3.0 | 8,638 |
| relativeDurationError > 5.0 | 5,488 |
| predDuration > 1000 | 6,487 |
| predDuration > 2000 | 1,222 |
| truthDuration > 1000 | 1,134 |
| truthDuration > 2000 | 324 |

### Error direction counts, alpha = 1.0

| Direction | Count |
|---|---:|
| durationError > 0 | 30,259 |
| durationError < 0 | 16,291 |
| durationError == 0 | 1,073 |

## 5. Vehicle-level extreme analysis, alpha = 1.0

Top 20 vehicles by absolute duration error were also the top too-slow predictions:

| vehicleID | predDuration | truthDuration | durationError | absDurationError | relativeDurationError |
|---|---:|---:|---:|---:|---:|
| heavy_all_veh_000816 | 77473 | 103 | 77370 | 77370 | 751.165 |
| heavy_all_veh_000857 | 77513 | 145 | 77368 | 77368 | 533.572 |
| heavy_all_veh_000734 | 77393 | 112 | 77281 | 77281 | 690.009 |
| heavy_all_veh_000775 | 77433 | 152 | 77281 | 77281 | 508.428 |
| heavy_all_veh_000813 | 77335 | 73 | 77262 | 77262 | 1058.38 |
| heavy_all_veh_000854 | 77375 | 113 | 77262 | 77262 | 683.735 |
| heavy_all_veh_000772 | 77295 | 36 | 77259 | 77259 | 2146.08 |
| heavy_all_veh_000815 | 77375 | 122 | 77253 | 77253 | 633.221 |
| heavy_all_veh_000856 | 77415 | 162 | 77253 | 77253 | 476.87 |
| heavy_all_veh_000693 | 77353 | 157 | 77196 | 77196 | 491.694 |
| heavy_all_veh_000533 | 77282 | 90 | 77192 | 77192 | 857.689 |
| heavy_all_veh_000492 | 77329 | 138 | 77191 | 77191 | 559.355 |
| heavy_all_veh_000611 | 77273 | 83 | 77190 | 77190 | 930 |
| heavy_all_veh_000652 | 77313 | 124 | 77189 | 77189 | 622.492 |
| heavy_all_veh_000488 | 77330 | 144 | 77186 | 77186 | 536.014 |
| heavy_all_veh_000529 | 77281 | 96 | 77185 | 77185 | 804.01 |
| heavy_all_veh_000690 | 77215 | 44 | 77171 | 77171 | 1753.89 |
| heavy_all_veh_000731 | 77255 | 84 | 77171 | 77171 | 918.702 |
| heavy_all_veh_000774 | 77335 | 167 | 77168 | 77168 | 462.084 |
| heavy_all_veh_000733 | 77295 | 132 | 77163 | 77163 | 584.568 |

Top 20 too-fast predictions:

| vehicleID | predDuration | truthDuration | durationError | absDurationError | relativeDurationError |
|---|---:|---:|---:|---:|---:|
| oversat_all_veh_001787 | 1006 | 9382 | -8376 | 8376 | 0.892773 |
| oversat_all_veh_001869 | 1036 | 9409 | -8373 | 8373 | 0.889893 |
| oversat_all_veh_001828 | 1065 | 9436 | -8371 | 8371 | 0.887134 |
| oversat_all_veh_001746 | 1035 | 9316 | -8281 | 8281 | 0.888901 |
| oversat_all_veh_001705 | 976 | 9255 | -8279 | 8279 | 0.894543 |
| oversat_all_veh_001664 | 1005 | 9108 | -8103 | 8103 | 0.889657 |
| oversat_all_veh_001541 | 916 | 8486 | -7570 | 7570 | 0.892058 |
| oversat_all_veh_001500 | 945 | 8512 | -7567 | 7567 | 0.88898 |
| oversat_all_veh_001623 | 946 | 8509 | -7563 | 7563 | 0.888824 |
| oversat_all_veh_001582 | 975 | 8536 | -7561 | 7561 | 0.885778 |
| oversat_all_veh_001910 | 1095 | 8106 | -7011 | 7011 | 0.864915 |
| oversat_all_veh_001459 | 886 | 7464 | -6578 | 6578 | 0.881297 |
| oversat_all_veh_001951 | 1066 | 7294 | -6228 | 6228 | 0.853852 |
| oversat_all_veh_000980 | 214 | 6148 | -5934 | 5934 | 0.965192 |
| oversat_all_veh_000979 | 205 | 5830 | -5625 | 5625 | 0.964837 |
| oversat_all_veh_000857 | 292 | 5808 | -5516 | 5516 | 0.949725 |
| oversat_all_veh_001021 | 212 | 5717 | -5505 | 5505 | 0.962918 |
| oversat_all_veh_002033 | 1096 | 6588 | -5492 | 5492 | 0.833637 |
| oversat_all_veh_001992 | 1125 | 6584 | -5459 | 5459 | 0.829131 |
| oversat_all_veh_000897 | 209 | 5560 | -5351 | 5351 | 0.96241 |

The default kinematic run introduced obvious vehicle-level extreme errors: 1,222 vehicles had predicted duration greater than 2,000 seconds, and the worst predicted durations were about 77,000 seconds for vehicles whose truth durations were under 200 seconds.

## 6. Comparison with speed-net interval experiments

| Metric | speed-net interval=1 | speed-net interval=2 | speed-net interval=3 | kinematic current, alpha=1.0 | kinematic sensitivity, alpha=0.5 |
|---|---:|---:|---:|---:|---:|
| compared vehicles | 47,623 | 47,623 | 47,623 | 47,623 | 47,623 |
| meanPredDuration | 127.02 | 272.821 | 461.693 | 629.277 | 356.515 |
| meanTruthDuration | 214.285 | 214.285 | 214.285 | 214.285 | 214.285 |
| Bias | -87.2656 | +58.5357 | +247.407 | +414.992 | +142.229 |
| MAE | 130.484 | 202.665 | 339.476 | 529.316 | 285.439 |
| RMSE | 379.224 | 1007.89 | 1265.93 | 3293.81 | 1591.24 |
| MAPE | 52.422% | 126.306% | 290.328% | 383.948% | 170.869% |
| absDurationError > 1000 | n/a | 1,235 | n/a | 5,665 | 2,550 |
| relativeDurationError > 3.0 | n/a | n/a | n/a | 8,638 | 3,602 |
| predDuration > 2000 | n/a | 371 | n/a | 1,222 | 701 |
| truthDuration > 2000 | n/a | n/a | n/a | 324 | 324 |
| max movement wait | 978s | n/a | 36,526s | not generated in lightweight run | not generated in lightweight run |
| movementWaitingTime > 1000 | 0 | n/a | 1,252 | not generated in lightweight run | not generated in lightweight run |
| movementWaitingTime > 2000 | n/a | n/a | 618 | not generated in lightweight run | not generated in lightweight run |

Kinematic alpha=1.0 does not reduce speed-net interval=1's optimistic bias in a useful way; it overcorrects to a much larger pessimistic bias. It also does not avoid interval=2/3-style overprediction: it is worse than interval=3 on mean predicted duration, bias, MAE, RMSE, MAPE, and vehicle-level outlier counts where comparable. The alpha=0.5 sensitivity run is materially better than alpha=1.0, but it remains worse than speed-net interval=1 and interval=2 on MAE, RMSE, MAPE, and predDuration > 2000.

## 7. Limitations of lightweight eval

- Movement-level waiting was not analyzed because movement timeline outputs were not generated by the default lightweight evaluation run.
- The comparison table uses the provided historical speed-net reference numbers where direct files were not available.
- The alpha sensitivity was intentionally limited to a single alpha=0.5 run; no broad calibration sweep was performed.

## 8. Recommendation

Recommendation: **D. Kinematic is worse; do not use it for now.**

Default kinematic mode substantially overpredicts duration, produces worse MAE/RMSE/MAPE than all three provided speed-net interval references, and creates clear vehicle-level extreme predictions. Alpha=0.5 reduces the damage, but still does not beat the current speed-net interval=1 baseline and still leaves many large outliers. If kinematic mode remains of interest, it should be treated as requiring calibration and diagnostics before any baseline switch.

## 9. Commands run

```bash
git status --short
git log -1 --oneline
python3 - <<'PY'
import xml.etree.ElementTree as ET
root = ET.parse("data/test.net.xml").getroot()
for tlid in ["J17", "J21", "J22"]:
    node = root.find(f"tlLogic[@id='{tlid}']")
    print(tlid, node.get("offset") if node is not None else "MISSING")
PY
rg -n "hasLaneStorageCapacity|laneCapacity|vehicleGap|vehicleSpace" . -g '!build' -g '!diag_*'
cmake -S . -B build
cmake --build build -j2
python3 - <<'PY'
import xml.etree.ElementTree as ET
ET.parse("data/test.net.xml")
print("XML OK")
PY
python3 - <<'PY'
import xml.etree.ElementTree as ET
root = ET.parse("data/test.net.xml").getroot()
links = {}
for c in root.findall("connection"):
    if c.get("tl") and c.get("linkIndex") is not None:
        links.setdefault(c.get("tl"), set()).add(int(c.get("linkIndex")))
ok = True
for tl in root.findall("tlLogic"):
    tlid = tl.get("id")
    expected = max(links.get(tlid, {-1})) + 1 if links.get(tlid) else 0
    for p in tl.findall("phase"):
        state = p.get("state", "")
        if len(state) != expected:
            print("MISMATCH", tlid, len(state), expected, state)
            ok = False
print("tlLogic state lengths OK" if ok else "tlLogic state length mismatches found")
PY
sumo -n data/test.net.xml --no-step-log true --duration-log.disable true --end 1
rm -rf diag_full_kinematic
mkdir -p diag_full_kinematic
./build/Simulation_Prediction --use-sumo --sumo-net data/test.net.xml --sumo-route data/full_no_change.rou.xml --sumo-tripinfo data/tripinfo_full_no_change.xml --eval-output diag_full_kinematic/sumo_eval.csv --travel-time-mode kinematic 2>&1 | tee diag_full_kinematic/run.log
find diag_full_kinematic -maxdepth 1 -type f -print | sort
python3 - <<'PY'
# standard-library csv parsing for threshold counts and top vehicles
PY
rm -rf diag_full_kinematic_alpha05
mkdir -p diag_full_kinematic_alpha05
./build/Simulation_Prediction --use-sumo --sumo-net data/test.net.xml --sumo-route data/full_no_change.rou.xml --sumo-tripinfo data/tripinfo_full_no_change.xml --eval-output diag_full_kinematic_alpha05/sumo_eval.csv --travel-time-mode kinematic --kinematic-congestion-alpha 0.5 2>&1 | tee diag_full_kinematic_alpha05/run.log
git status --short
git diff --stat
git diff --cached --stat
git diff --check
```

## 10. PR safety / repository hygiene

- No generated eval CSV/log files are intended to be committed.
- `.gitignore` changed minimally to ignore `diag_*/` and `*.log`.
- Raw generated outputs remain ignored/untracked and are excluded from the commit.
- Final git status and final diff stats were checked before commit.
