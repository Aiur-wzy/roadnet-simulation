# Interval 2 Lightweight Evaluation Summary

## 1. Experiment setup

- Simulator: current SUMO/CAMS build from this branch.
- Network: `data/test.net.xml`.
- Routes: `data/full_no_change.rou.xml`.
- Truth tripinfo: `data/tripinfo_full_no_change.xml`.
- Travel-time mode: `speed-net`.
- Lane discharge interval: `2` seconds.
- Evaluation output path: `diag_full_interval2/sumo_eval.csv`.
- Extreme split and movement timeline options were not enabled.

## 2. Build and validation status

- `git status --short` was clean before the run.
- Starting commit: `379664c Merge pull request #51 from Aiur-wzy/fvb55s-codex/sumo/cams`.
- Required input files existed.
- Current signal offsets were confirmed: J17 = 45, J21 = 36, J22 = 72.
- Existing lane-capacity logic was inspected and not modified. `Graph::hasLaneStorageCapacity(...)` is present, lane capacity is documented as the authoritative macro storage rule, and the configured capacity formula is `floor((max(0.0, road.length) + vehicleGap) / vehicleSpace) + 1`.
- `cmake -S . -B build` completed successfully.
- `cmake --build build -j2` completed successfully.
- `data/test.net.xml` parsed successfully.
- tlLogic phase state lengths matched the expected connection link-index counts.
- SUMO executable was unavailable in the environment, so the optional `sumo -n ... --end 1` validation was not run.

## 3. Generated files

The simulator generated the following files in `diag_full_interval2/`:

- `eval_distribution_metrics.csv`
- `eval_extreme_duration_errors.csv`
- `eval_extreme_route_summary.csv`
- `eval_grouped_metrics.csv`
- `eval_summary.txt`
- `run.log`
- `sumo_eval.csv`

No generated evaluation CSV/log files are committed.

## 4. Overall evaluation summary

### Coverage

| Metric | Value |
|---|---:|
| simulatedVehicles | 47,623 |
| truthVehicles | 47,623 |
| comparedVehicles | 47,623 |
| missingTruth | 0 |
| invalidSkipped | 0 |
| etaMissingSkipped | 0 |
| truthNotSimulated | 0 |

### Duration metrics

| Metric | Value |
|---|---:|
| meanPredDuration | 272.821 |
| meanTruthDuration | 214.285 |
| Bias | +58.5357 |
| MAE | 202.665 |
| RMSE | 1007.89 |
| MAPE | 126.306% |
| Median absolute error | 56 |
| P90 absolute error | 355 |
| P95 absolute error | 646 |

### Arrival metrics

| Metric | Value |
|---|---:|
| MAE arrival | 2316.05 |
| RMSE arrival | 5142.48 |
| Bias arrival | +628.724 |

### Speed metrics

| Metric | Value |
|---|---:|
| meanPredSpeed | 3.80248 |
| meanTruthSpeed | 4.17272 |
| speedBias | -0.370234 |
| speedMAE | 1.89797 |

## 5. Vehicle-level extreme analysis

### Threshold counts

| Threshold | Count |
|---|---:|
| absDurationError > 300 | 5,937 |
| absDurationError > 600 | 2,633 |
| absDurationError > 1000 | 1,235 |
| relativeDurationError > 1.0 | 9,364 |
| relativeDurationError > 3.0 | 2,023 |
| relativeDurationError > 5.0 | 1,204 |
| predDuration > 1000 | 1,168 |
| predDuration > 2000 | 371 |
| truthDuration > 1000 | 1,134 |
| truthDuration > 2000 | 324 |

### Error direction counts

| Direction | Count |
|---|---:|
| durationError > 0 | 24,224 |
| durationError < 0 | 22,963 |
| durationError == 0 | 436 |

### Top 20 vehicles by absolute duration error

| vehicleID | predDuration | truthDuration | durationError | absDurationError | relativeDurationError |
|---|---:|---:|---:|---:|---:|
| oversat_all_veh_009338 | 28323 | 56 | 28267 | 28267 | 504.768 |
| oversat_all_veh_009379 | 28323 | 109 | 28214 | 28214 | 258.844 |
| oversat_all_veh_009461 | 28237 | 54 | 28183 | 28183 | 521.907 |
| oversat_all_veh_009420 | 28237 | 82 | 28155 | 28155 | 343.354 |
| oversat_all_veh_009543 | 28151 | 82 | 28069 | 28069 | 342.305 |
| oversat_all_veh_009502 | 28151 | 111 | 28040 | 28040 | 252.613 |
| oversat_all_veh_009625 | 28132 | 110 | 28022 | 28022 | 254.745 |
| oversat_all_veh_009584 | 28065 | 54 | 28011 | 28011 | 518.722 |
| oversat_all_veh_009707 | 28045 | 55 | 27990 | 27990 | 508.909 |
| oversat_all_veh_009666 | 28045 | 83 | 27962 | 27962 | 336.892 |
| oversat_all_veh_009789 | 27959 | 83 | 27876 | 27876 | 335.855 |
| oversat_all_veh_009748 | 27959 | 111 | 27848 | 27848 | 250.883 |
| oversat_all_veh_009830 | 27873 | 55 | 27818 | 27818 | 505.782 |
| oversat_all_veh_009871 | 27873 | 120 | 27753 | 27753 | 231.275 |
| oversat_all_veh_009953 | 27787 | 64 | 27723 | 27723 | 433.172 |
| oversat_all_veh_009912 | 27787 | 92 | 27695 | 27695 | 301.033 |
| oversat_all_veh_010035 | 27701 | 91 | 27610 | 27610 | 303.407 |
| oversat_all_veh_009994 | 27701 | 119 | 27582 | 27582 | 231.782 |
| oversat_all_veh_010076 | 27615 | 63 | 27552 | 27552 | 437.333 |
| oversat_all_veh_000372 | 22713 | 126 | 22587 | 22587 | 179.262 |

### Top 20 too-slow predictions

The top 20 too-slow predictions are identical to the top 20 absolute duration errors above; all have positive duration error.

### Top 20 too-fast predictions

| vehicleID | predDuration | truthDuration | durationError | absDurationError | relativeDurationError |
|---|---:|---:|---:|---:|---:|
| oversat_all_veh_001869 | 46 | 9409 | -9363 | 9363 | 0.995111 |
| oversat_all_veh_001828 | 74 | 9436 | -9362 | 9362 | 0.992158 |
| oversat_all_veh_001787 | 102 | 9382 | -9280 | 9280 | 0.989128 |
| oversat_all_veh_001746 | 46 | 9316 | -9270 | 9270 | 0.995062 |
| oversat_all_veh_001705 | 74 | 9255 | -9181 | 9181 | 0.992004 |
| oversat_all_veh_001664 | 102 | 9108 | -9006 | 9006 | 0.988801 |
| oversat_all_veh_001500 | 46 | 8512 | -8466 | 8466 | 0.994596 |
| oversat_all_veh_001623 | 46 | 8509 | -8463 | 8463 | 0.994594 |
| oversat_all_veh_001582 | 74 | 8536 | -8462 | 8462 | 0.991331 |
| oversat_all_veh_001541 | 102 | 8486 | -8384 | 8384 | 0.98798 |
| oversat_all_veh_001910 | 102 | 8106 | -8004 | 8004 | 0.987417 |
| oversat_all_veh_001459 | 74 | 7464 | -7390 | 7390 | 0.990086 |
| oversat_all_veh_001951 | 74 | 7294 | -7220 | 7220 | 0.989855 |
| oversat_all_veh_001992 | 46 | 6584 | -6538 | 6538 | 0.993013 |
| oversat_all_veh_002033 | 102 | 6588 | -6486 | 6486 | 0.984517 |
| oversat_all_veh_002074 | 74 | 6132 | -6058 | 6058 | 0.987932 |
| oversat_all_veh_001418 | 102 | 6136 | -6034 | 6034 | 0.983377 |
| oversat_all_veh_001377 | 46 | 5991 | -5945 | 5945 | 0.992322 |
| oversat_all_veh_001336 | 74 | 6016 | -5942 | 5942 | 0.987699 |
| oversat_all_veh_001295 | 102 | 5783 | -5681 | 5681 | 0.982362 |

Movement-level waiting could not be analyzed from the default outputs because extreme timeline outputs were intentionally not generated.

## 6. Interval 1 vs interval 2 vs interval 3 comparison

| Metric | interval=1 | interval=2 | interval=3 |
|---|---:|---:|---:|
| compared vehicles | 47,623 | 47,623 | 47,623 |
| meanPredDuration | 127.02 | 272.821 | 461.693 |
| meanTruthDuration | 214.285 | 214.285 | 214.285 |
| Bias | -87.2656 | +58.5357 | +247.407 |
| MAE | 130.484 | 202.665 | 339.476 |
| RMSE | 379.224 | 1007.89 | 1265.93 |
| MAPE | 52.422% | 126.306% | 290.328% |
| absDurationError > 300 | not available from reference | 5,937 | not available from reference |
| absDurationError > 600 | not available from reference | 2,633 | not available from reference |
| absDurationError > 1000 | not available from reference | 1,235 | not available from reference |
| relativeDurationError > 3.0 | not available from reference | 2,023 | not available from reference |
| predDuration > 2000 | not available from reference | 371 | not available from reference |
| truthDuration > 2000 | not available from reference | 324 | not available from reference |
| max movement wait | 978s | not generated in this lightweight run | 36,526s |
| movementWaitingTime > 500 | 226 | not generated in this lightweight run | 3,785 |
| movementWaitingTime > 1000 | 0 | not generated in this lightweight run | 1,252 |
| movementWaitingTime > 2000 | not available from reference | not generated in this lightweight run | 618 |
| bottleneck movements | movement 18 | not generated in this lightweight run | movements 69, 64, 30, 34, 41, 18, 7, 70 |

Interval 2 reduces interval 1's optimistic duration bias from -87.2656 seconds to a smaller but pessimistic +58.5357 seconds. However, it worsens MAE from 130.484 to 202.665, RMSE from 379.224 to 1007.89, and MAPE from 52.422% to 126.306%. It avoids interval 3's much more severe average pessimistic overcorrection, but the interval 2 vehicle-level output still contains obvious extreme predictions, including 371 vehicles with predicted duration above 2000 seconds and individual overpredictions above 28,000 seconds.

## 7. Limitations of this lightweight eval

- This run used only evaluation outputs produced by the existing simulator code.
- No new evaluation output types, debug CSVs, or movement-timeline instrumentation were added.
- Extreme split and movement timeline options were intentionally not enabled.
- Because no movement timeline was generated, interval 2 movement-level lockups cannot be ruled out or characterized from this run.
- Interval 1 and interval 3 threshold counts were not recomputed because direct output directories were not used; the comparison uses the provided reference metrics where available.

## 8. Recommendation

Recommendation: **D. Keep interval configurable and report sensitivity across 1/2/3**.

Interval 2 is directionally useful as a sensitivity point because it reduces interval 1's optimistic bias and avoids interval 3's severe average overcorrection. It should not replace interval 1 as the default based on this lightweight evaluation, because its MAE, RMSE, and MAPE are worse than interval 1 and it introduces clear vehicle-level extreme overpredictions. It also should not be rejected entirely, because it remains materially less overcorrected than interval 3 and may help diagnose calibration around discharge intervals.

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
if command -v sumo >/dev/null; then sumo -n data/test.net.xml --no-step-log true --duration-log.disable true --end 1; else echo 'SUMO unavailable'; fi
rm -rf diag_full_interval2
mkdir -p diag_full_interval2
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net data/test.net.xml \
  --sumo-route data/full_no_change.rou.xml \
  --sumo-tripinfo data/tripinfo_full_no_change.xml \
  --eval-output diag_full_interval2/sumo_eval.csv \
  --travel-time-mode speed-net \
  --lane-discharge-interval 2 \
  2>&1 | tee diag_full_interval2/run.log
find diag_full_interval2 -maxdepth 1 -type f -print | sort
python3 - <<'PY'
# one-off standard-library CSV summary command; not committed
PY
git status --short
git diff --stat
git diff --cached --stat
git diff --check
git add docs/interval2_lightweight_eval_summary.md
git commit -m "Add interval 2 lightweight evaluation summary"
```

## 10. PR safety / repository hygiene

- No generated eval CSV/log files were committed.
- `.gitignore` was not changed.
- The only committed content is this concise markdown report.
- Final `git diff --stat` after commit was empty.
- Final `git diff --cached --stat` after commit was empty.
- Final generated output and build directories were removed from the working tree after the report was committed, leaving no generated artifacts staged or unstaged.
