# RoadNet / CAMS Simulation Pipeline

## 1. Project Overview

RoadNet is a cycle-aware macroscopic traffic simulation system built around SUMO network inputs. The C++ simulator parses SUMO roads, lanes, junctions, connections, traffic-light programs, and movement `linkIndex` metadata, then runs a CAMS-style signal-aware simulation and optional prediction-vs-truth evaluation.

The repository supports two workflows:

1. **SUMO/CAMS workflow**: the current recommended workflow for experiments. It starts from a SUMO `.net.xml` file and generated `.rou.xml` demand, uses SUMO and TraCI to create ground truth and a slim model-training CSV, trains an AutoGluon road travel-time model, catches the model into a lookup table, and evaluates the C++ CAMS simulation against SUMO `tripinfo.xml`.
2. **Legacy BJ workflow**: the older Manhattan/Beijing-style text-file workflow, kept as a compatibility and regression workflow.

High-level SUMO/CAMS pipeline:

```text
test.net.xml
    -> generate_route_on_test_net_full.py
    -> all_congestion_routes.rou.xml
    -> SUMO tripinfo.xml
    -> TraCI_Python_Adjusted.py
    -> TraCI_output_adjusted.csv  [slim training table]
    -> model_training_sumo_v1.py
    -> models_v1/
    -> model_catching_sumo_v1.py
    -> model_catching_sumo_v1.txt
    -> C++ Simulation_Prediction with --travel-time-mode table
    -> sumo_eval.csv / eval_summary.txt / grouped metrics / distribution metrics
```

Optional TraCI debug outputs:

```text
cams_model_training_data.csv
cams_vehicle_movement_events.csv
cams_movement_cycle_summary.csv
```

## 2. Repository Structure

| Path | Role |
| --- | --- |
| `main.cpp` | CLI parsing, SUMO/BJ workflow selection, resolved configuration printing, smoke-test orchestration, travel-time table load reporting, full simulation orchestration, and evaluation entry. |
| `head.h` | Core data structures and the `Graph` class, including roads, nodes, movements, signals, vehicles, SUMO raw structures, travel-time table formats, SUMO v1 lookup keys, graph containers, validation declarations, and path plumbing. |
| `data_preparation.cpp` | SUMO net/route parsing, legacy graph readers, graph construction, movement/lane-group/signal-program construction, and route conversion helpers. |
| `data_cleaning.cpp` | SUMO network, connection, signal-program, route, and input validation helpers. |
| `simulation.cpp` | Cycle-aware simulation, movement dispatch, travel-time prediction, legacy and SUMO v1 table parsing/lookup, SUMO `tripinfo.xml` evaluation, grouped metrics, distribution metrics, and legacy evaluation. |
| `generate_route_on_test_net_full.py` | Route demand generation from predefined valid SUMO route sequences, including the staged `all-congestion` profile. |
| `TraCI_Python_Adjusted.py` | TraCI-based data collector for the slim training CSV, the fuller CAMS debug/training-compatible CSV, vehicle movement records, and cycle summaries. |
| `model_training_sumo_v1.py` | Active AutoGluon training script for the slim SUMO/CAMS v1 feature schema. |
| `model_catching_sumo_v1.py` | Active lookup-table builder that enumerates SUMO/CAMS v1 features and catches AutoGluon predictions into a space-separated table. |
| `config_defaults.h` | Default relative paths used when command-line arguments are omitted. Prefer explicit CLI paths for reproducible experiments. |
| `CMakeLists.txt` | C++ build configuration for the `Simulation_Prediction` executable. |
| `README.md` | This end-to-end experiment pipeline guide. |

## 3. Environment Setup

### 3.1 C++ build dependencies

Required tools/libraries:

- CMake.
- A C++14-capable compiler such as `g++`.
- POSIX threads (`pthread`).
- Boost headers/libraries if required by the local toolchain configuration.

Build with CMake:

```bash
cmake -S . -B build
cmake --build build -j2
```

The CMake binary is usually:

```bash
./build/Simulation_Prediction
```

Direct `g++` fallback:

```bash
g++ -std=c++14 -O0 -g \
  main.cpp data_preparation.cpp simulation.cpp data_cleaning.cpp \
  -o roadnet_sim -lpthread
```

Sanity check:

```bash
./build/Simulation_Prediction --help
```

### 3.2 SUMO and TraCI setup

SUMO must be installed and `SUMO_HOME` must point to the SUMO installation so Python can import TraCI.

Linux example:

```bash
export SUMO_HOME=/usr/share/sumo
export PATH=$SUMO_HOME/bin:$PATH
```

Windows PowerShell example:

```powershell
$env:SUMO_HOME="C:\Program Files (x86)\Eclipse\Sumo"
$env:Path="$env:SUMO_HOME\bin;$env:Path"
```

Check SUMO and TraCI:

```bash
sumo --version
python -c "import os, sys; sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools')); import traci; print('traci ok')"
```

If Python reports that `traci` cannot be imported, fix `SUMO_HOME` before running the TraCI collector.

## 4. Step 1: Generate SUMO Route Demand

Create SUMO `.rou.xml` demand from predefined valid route sequences in `generate_route_on_test_net_full.py`.

Recommended large all-congestion command:

```bash
python generate_route_on_test_net_full.py \
  --profile all-congestion \
  --output all_congestion_routes.rou.xml \
  --manifest-output all_congestion_manifest.csv
```

The `all-congestion` profile is intended to include staged free-flow, light, medium, heavy, oversaturated, directional bottleneck, and single-route pressure scenarios.

Expected outputs:

```text
all_congestion_routes.rou.xml
all_congestion_manifest.csv
```

Sanity check:

```bash
python generate_route_on_test_net_full.py --help
```

## 5. Step 2: Create or Update SUMO Config

Point a SUMO `.sumocfg` file to the network and generated demand.

Example `map.sumo.cfg`:

```xml
<configuration>
    <input>
        <net-file value="test.net.xml"/>
        <route-files value="all_congestion_routes.rou.xml"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="16000"/>
    </time>
</configuration>
```

Choose an `end` time long enough for all vehicles to depart and finish.

Sanity check:

```bash
sumo -c map.sumo.cfg --check-route-files true
```

If the installed SUMO version does not support `--check-route-files`, run a short simulation and inspect route-loading errors:

```bash
sumo -c map.sumo.cfg --end 300 --no-step-log true
```

## 6. Step 3: Run SUMO to Generate TripInfo Ground Truth

Generate SUMO microscopic travel-time ground truth for final evaluation.

```bash
sumo -c map.sumo.cfg \
  --tripinfo-output tripinfo.xml \
  --duration-log.statistics true \
  --no-step-log true
```

`tripinfo.xml` contains per-vehicle SUMO truth. The C++ evaluator reads each `<tripinfo>` record's `duration` as ground-truth travel time and uses `arrival` as a secondary arrival-time check. Records are aligned by SUMO vehicle id, not by XML line order.

Expected output:

```text
tripinfo.xml
```

## 7. Step 4: Run TraCI to Generate the Slim Training CSV

Replay the SUMO simulation through TraCI and collect the preferred large-scale model-training input.

`--outputs legacy` still writes `TraCI_output_adjusted.csv` for CLI compatibility, but the content is **not** the old legacy wide edge table. It is now the slim SUMO/CAMS model-training table.

Recommended command:

```bash
python TraCI_Python_Adjusted.py \
  --sumo-config map.sumo.cfg \
  --net-file test.net.xml \
  --sumo-binary sumo \
  --outputs legacy \
  --legacy-edge-output TraCI_output_adjusted.csv \
  --waiting-region-length 80 \
  --stop-speed-threshold 0.1 \
  --queue-speed-threshold 2.0 \
  --low-speed-threshold 2.0 \
  --downstream-block-occupancy-threshold 85
```

Expected output:

```text
TraCI_output_adjusted.csv
```

Expected columns:

```text
has_waiting,road_length,turn_type,road_flow,lane_flow,travel_time_label
```

Feature and label semantics:

- `has_waiting`: whether the vehicle had waiting before entering the current road / previous movement waiting state.
- `road_length`: selected/current lane length sampled by TraCI/CAMS.
- `turn_type`: fixed SUMO/C++ numeric encoding.
- `road_flow`: vehicle count on the current edge.
- `lane_flow`: selected-lane vehicle count. Do not derive this as `road_flow / lane_num`.
- `travel_time_label`: `driving_time + low_speed_time`.

The label excludes `red_light_waiting_time`, `green_queue_wait_time`, and `downstream_block_wait_time` because signal waiting, queue waiting, and downstream blocking are handled by the macro CAMS dispatch logic, not by the road travel-time model.

Optional debug command:

```bash
python TraCI_Python_Adjusted.py \
  --sumo-config map.sumo.cfg \
  --net-file test.net.xml \
  --sumo-binary sumo \
  --outputs legacy,training,events,cycles
```

Expected debug outputs:

```text
cams_model_training_data.csv
cams_vehicle_movement_events.csv
cams_movement_cycle_summary.csv
```

Use `TraCI_output_adjusted.csv` as the recommended large-scale training input. Use `cams_model_training_data.csv` when you need the fuller debug/training-compatible table; it contains many more fields and is wider and slower to load.

Sanity checks:

```bash
head TraCI_output_adjusted.csv
python - <<'PY'
import pandas as pd
df = pd.read_csv("TraCI_output_adjusted.csv")
print(df.head())
print(df.describe())
print(df["turn_type"].value_counts().sort_index())
print(df["has_waiting"].value_counts())
PY
```

## 8. Step 5: Train the SUMO v1 Road-Level Travel-Time Model

`model_training_sumo_v1.py` is the active AutoGluon training script for the slim SUMO/CAMS schema.

Recommended command:

```bash
python model_training_sumo_v1.py \
  --csv TraCI_output_adjusted.csv \
  --save_path models_v1 \
  --time_limit 36000
```

The script also accepts `--net`, but the default slim base-feature model does not require it.

Expected input:

```text
TraCI_output_adjusted.csv
```

Expected model input features:

```text
has_waiting
road_length
turn_type
road_flow
lane_flow
```

Expected AutoGluon label:

```text
travel_time_no_waiting
```

The training script maps:

```text
travel_time_no_waiting = travel_time_label
```

Do not add `Delay_Time`, red-light waiting, green-queue waiting, or downstream-block waiting to this label.

Expected output:

```text
models_v1/
```

## 9. Step 6: Generate the SUMO v1 Travel-Time Lookup Table

`model_catching_sumo_v1.py` is the active lookup table builder. It loads the AutoGluon model from `models_v1/`, enumerates feature combinations, predicts `travel_time_no_waiting`, and writes a space-separated table for C++ table mode.

Recommended command:

```bash
python model_catching_sumo_v1.py \
  --csv TraCI_output_adjusted.csv \
  --model_path models_v1 \
  --output_txt model_catching_sumo_v1.txt \
  --max_road_flow 200 \
  --max_lane_flow 80 \
  --lane_flow_step 1
```

Notes:

- `--max_road_flow` controls `road_flow` enumeration.
- `--max_lane_flow` controls selected-lane `lane_flow` enumeration.
- `lane_flow` is enumerated independently but constrained to `lane_flow <= road_flow`.
- The script is Python 3.6 compatible.
- The output table header is:

```text
has_waiting road_length turn_type road_flow lane_flow travel_time_no_waiting
```

Expected output:

```text
model_catching_sumo_v1.txt
```

Sanity checks:

```bash
head model_catching_sumo_v1.txt
wc -l model_catching_sumo_v1.txt
```

## 10. Step 7: Run C++ CAMS Simulation in Table Mode

Start with a smoke test:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --smoke-test
```

Run SUMO v1 table-mode simulation and evaluation:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --sumo-tripinfo tripinfo.xml \
  --eval-output sumo_eval.csv \
  --travel-time-mode table \
  --travel-time-table model_catching_sumo_v1.txt \
  --tt-fallback speed-net
```

Optional table debugging:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --sumo-tripinfo tripinfo.xml \
  --eval-output sumo_eval.csv \
  --travel-time-mode table \
  --travel-time-table model_catching_sumo_v1.txt \
  --tt-fallback speed-net \
  --verbose-travel-time
```

Expected logs include:

```text
[TravelTime] Loaded SUMO v1 table entries: ...
[TravelTime] table hits: ...
[TravelTime] table misses: ...
```

If a legacy RoadKey table is provided instead, the loader falls back to the legacy parser and logs:

```text
[TravelTime] Loaded legacy table dictionary entries: ...
```

Supported SUMO route XML formats include global route definitions with vehicle route references and inline routes nested inside vehicles. Internal SUMO edges whose ids begin with `:` are skipped during route conversion. Vehicles whose remaining route references an unknown edge are skipped with a `[SUMO Route Warning]` instead of being mapped to road id `0`.

Expected evaluation outputs:

```text
sumo_eval.csv
eval_summary.txt
eval_grouped_metrics.csv
eval_distribution_metrics.csv
```

## 11. Travel-Time Modes

The C++ simulator supports these travel-time modes:

- `speed-net`: computes `length / speed_limit` with safety clamping and a minimum return value.
- `min-time`: uses precomputed `minTravelTime` when available, otherwise uses `speed-net`.
- `table`: loads either a legacy RoadKey table or a SUMO_V1 table depending on the first-line header. SUMO_V1 is the recommended mode for the trained AutoGluon lookup table produced by `model_catching_sumo_v1.py`.
- `kinematic`: uses a formula-based road travel-time approximation.
- `model`: reserved; currently not a full external model-service path unless implemented in the C++ runtime. Use `table` for the caught AutoGluon model.

## 12. SUMO_V1 Lookup Table Schema

SUMO_V1 table header:

```text
has_waiting road_length turn_type road_flow lane_flow travel_time_no_waiting
```

Feature definitions:

- `has_waiting`: `0/1` previous waiting state.
- `road_length`: selected/current lane length.
- `turn_type`: fixed SUMO/C++ encoding:
  - `0` = unknown/end/missing
  - `1` = left or partially left, `l/L`
  - `2` = straight, `s`
  - `3` = right or partially right, `r/R`
  - `4` = turn/u-turn, `t`
- `road_flow`: current edge vehicle count.
- `lane_flow`: selected-lane vehicle count. Do not derive this as `road_flow / lane_num`.
- `travel_time_no_waiting`: model prediction for road travel time, trained from `travel_time_label`.

C++ table lookup quantizes floating-point fields before hashing:

```text
road_length_q = round(road_length * 10000)
lane_flow_q = round(lane_flow * 1000000)
```

The current C++ SUMO_V1 lookup uses `features.lane_flow` directly, matching the slim TraCI output and catching script feature semantics.

## 13. Recommended End-to-End Quickstart

```bash
cmake -S . -B build
cmake --build build -j2

python generate_route_on_test_net_full.py \
  --profile all-congestion \
  --output all_congestion_routes.rou.xml \
  --manifest-output all_congestion_manifest.csv

sumo -c map.sumo.cfg \
  --tripinfo-output tripinfo.xml \
  --duration-log.statistics true \
  --no-step-log true

python TraCI_Python_Adjusted.py \
  --sumo-config map.sumo.cfg \
  --net-file test.net.xml \
  --sumo-binary sumo \
  --outputs legacy \
  --legacy-edge-output TraCI_output_adjusted.csv

python model_training_sumo_v1.py \
  --csv TraCI_output_adjusted.csv \
  --save_path models_v1 \
  --time_limit 36000

python model_catching_sumo_v1.py \
  --csv TraCI_output_adjusted.csv \
  --model_path models_v1 \
  --output_txt model_catching_sumo_v1.txt \
  --max_road_flow 200 \
  --max_lane_flow 80

./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --sumo-tripinfo tripinfo.xml \
  --eval-output sumo_eval.csv \
  --travel-time-mode table \
  --travel-time-table model_catching_sumo_v1.txt \
  --tt-fallback speed-net
```

## 14. Output Files

| File | Produced by | Purpose |
| --- | --- | --- |
| `all_congestion_routes.rou.xml` | route generator | SUMO route demand. |
| `all_congestion_manifest.csv` | route generator | Scenario metadata. |
| `tripinfo.xml` | SUMO | Microscopic ground truth. |
| `TraCI_output_adjusted.csv` | TraCI | Preferred slim training table with `has_waiting,road_length,turn_type,road_flow,lane_flow,travel_time_label`. |
| `cams_model_training_data.csv` | TraCI | Fuller CAMS debug/training-compatible table; wider and slower to load than the slim CSV. |
| `cams_vehicle_movement_events.csv` | TraCI | Movement-level signal event debug data. |
| `cams_movement_cycle_summary.csv` | TraCI | Signal-cycle discharge summary. |
| `models_v1/` | `model_training_sumo_v1.py` | AutoGluon model artifact. |
| `model_catching_sumo_v1.txt` | `model_catching_sumo_v1.py` | SUMO_V1 lookup table for C++ table mode. |
| `sumo_eval.csv` | C++ simulator | Per-vehicle prediction-vs-truth evaluation. |
| `eval_summary.txt` | C++ simulator | Overall evaluation summary. |
| `eval_grouped_metrics.csv` | C++ simulator | Grouped error analysis. |
| `eval_distribution_metrics.csv` | C++ simulator | Distribution-level error analysis. |

## 15. Troubleshooting and Sanity Checks

### SUMO / TraCI import failures

If SUMO cannot import `traci`, check `SUMO_HOME` and make sure `$SUMO_HOME/tools` is available to Python.

```bash
python -c "import os, sys; sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools')); import traci; print('traci ok')"
```

### Model catching cannot load AutoGluon model

Check that `--model_path` points to the directory produced by `model_training_sumo_v1.py`.

```bash
python model_catching_sumo_v1.py --model_path models_v1 --help
```

### High table misses

Run with `--verbose-travel-time` and check:

- `road_length` precision and the C++ `round(road_length * 10000)` quantization.
- `turn_type` encoding consistency between TraCI, catching, and C++.
- `lane_flow` semantics: selected-lane vehicle count, not `road_flow / lane_num`.
- `--max_road_flow` and `--max_lane_flow` coverage in `model_catching_sumo_v1.py`.
- Whether C++ lookup uses `features.lane_flow` rather than deriving `road_flow / lane_num`.

### Evaluation has missing truth

Check SUMO `tripinfo.xml` vehicle ids and the route generation manifest. The C++ evaluator matches by SUMO vehicle id.

### CSV loading is slow

Use the slim `TraCI_output_adjusted.csv` for large-scale model training. Reserve `cams_model_training_data.csv` for debugging and compatibility because it is wider and slower to load.

### General route/config checks

```bash
python generate_route_on_test_net_full.py --help
sumo -c map.sumo.cfg --check-route-files true
head TraCI_output_adjusted.csv
head model_catching_sumo_v1.txt
```

## 16. Legacy BJ Workflow (Compatibility / Regression)

The legacy BJ workflow is separate from the recommended SUMO/CAMS pipeline. Keep it for compatibility with Manhattan/Beijing-style data and regression checks.

Use a base directory when the standard legacy files are colocated:

```bash
./roadnet_sim --base data/Manhattan_Data --read-num 1000
```

Or pass every file explicitly:

```bash
./roadnet_sim \
  --bj data/Manhattan_network_BJ.txt \
  --bj-min-time data/Manhattan_network_min_Travel_Time.txt \
  --road-info data/beijingMoreRoadInfo \
  --query data/query.txt \
  --route data/route.txt \
  --time data/time.txt \
  --read-num 1000
```

Required legacy files:

- `Manhattan_network_BJ.txt`
- `Manhattan_network_min_Travel_Time.txt`
- `beijingMoreRoadInfo`
- `query.txt`
- `route.txt`
- `time.txt`

Legacy data notes:

- BJ network file format: `node1 node2 edgeID length`.
- Min travel-time file format: `node1 node2 minTime`.
- Road-info files contain additional road attributes such as direction, speed limit, lane count, width, and kind.
- Query, route, and time files are aligned by index. The `i`th query corresponds to the `i`th route and `i`th time record.

Sanity check: start with a small `--read-num`, such as `1000`, before running the full legacy dataset.

## 17. Command-Line Reference

### C++ simulator options

| Argument | Meaning | Default / notes | Workflow |
| --- | --- | --- | --- |
| `--help` | Print usage and exit. | Off | Both |
| `--use-sumo` | Enable the SUMO `.net.xml` workflow. Without it, legacy BJ is the default. | Off | SUMO |
| `--smoke-test` | In SUMO mode, prepare, validate, print sample signal states, and exit. | Off | SUMO |
| `--base <dir>` | Derive standard input paths from `<dir>`. | `./data/Manhattan_Data` from `config_defaults.h` | Both |
| `--sumo-net <path>` | SUMO `.net.xml` input path. | `./test.net.xml` | SUMO |
| `--sumo-route <path>` | SUMO `.rou.xml` route file for full SUMO simulation. | Empty | SUMO |
| `--sumo-tripinfo <path>` | SUMO `tripinfo.xml` ground-truth output used for evaluation. | Empty | SUMO |
| `--eval-output <path>` | CSV path for per-vehicle SUMO prediction-vs-truth rows. Sibling summary/grouped/distribution files are also written. | Empty | SUMO |
| `--travel-time-mode <speed-net|min-time|table|model|kinematic>` | Select the single-road travel-time predictor. | `min-time` | Both |
| `--travel-time-table <path>` | Dictionary path for table-mode lookups. Supports legacy RoadKey tables and SUMO_V1 tables. | Derived default table path | Both |
| `--tt-fallback <speed-net|min-time>` | Fallback predictor for table misses or unimplemented model mode. | `speed-net` | Both |
| `--model-host <host>` | Host reserved for an external model service path if implemented. | `127.0.0.1` | Both |
| `--model-port <port>` | Port reserved for an external model service path if implemented. | `9000` | Both |
| `--verbose-travel-time` | Print per-miss or diagnostic travel-time messages. | Off | Both |
| `--kinematic-congestion-alpha <value>` | Congestion multiplier for `kinematic` mode; negative values clamp to `0`. | `1.0` | Both |
| `--lane-discharge-interval <k>` | Global seconds per movement lane-discharge slot; values `<=0` clamp to `1`. | `1` | Both |
| `--read-num <n>` | Number of query/route/time records to read. In SUMO route mode, this limits valid vehicles when greater than zero. | `192484` | Both |
| `--no-eval` | Skip final MSE/evaluation, including SUMO tripinfo evaluation. | Evaluation enabled | Both |
| `--bj <path>` | Legacy BJ graph file. | `<base>/Manhattan_network_BJ.txt` | Legacy |
| `--bj-min-time <path>` | Legacy min-travel-time file. | `<base>/Manhattan_network_min_Travel_Time.txt` | Legacy |
| `--road-info <path>` | Legacy road-info file. | `<base>/beijingMoreRoadInfo` | Legacy |
| `--query <path>` | Query file. | `<base>/query.txt` | Legacy route-data workflow |
| `--route <path>` | Route file. | `<base>/route.txt` | Legacy route-data workflow |
| `--time <path>` | Observed/ground-truth time file. | `<base>/time.txt` | Legacy evaluation |
| `--cut` | Cut route/query/time data before simulation. | `false` | Legacy |
| `--avg-length <n>` | Average length used by cut helpers. | `30` | Legacy |

Environment variables are supported as a lower-priority fallback only:

```bash
USE_SUMO_NET=1 SUMO_NET_PATH=test.net.xml ./roadnet_sim
```

### Route-generator options

| Argument | Meaning |
| --- | --- |
| `--output <path>` | Output `.rou.xml` file. |
| `--profile all-congestion` | Generate the staged all-congestion route file. |
| `--manifest-output <path>` | Scenario manifest CSV path for `--profile all-congestion`. |
| `--route-option <name>` | Route group or individual route id, such as `ALL`, `CORE`, `WEST_IN`, or `route_01_E9_to_E5_original`. |
| `--mode <uniform|poisson|period>` | Departure-time generation mode. |
| `--num-vehicles <n>` | Number of departure times; total vehicles equals departure times × selected routes. |
| `--period <seconds>` | Period between departure times when `--mode period`. |
| `--seed <n>` | Random seed for stochastic departure generation. |
| `--start-time <seconds>` | First departure-time bound. |
| `--end-time <seconds>` | Last departure-time bound. |
| `--depart-lane <value>` | SUMO `departLane` attribute. |
| `--depart-speed <value>` | SUMO `departSpeed` attribute. |
| `--depart-pos <value>` | SUMO `departPos` attribute. |

### TraCI collector options

| Argument | Meaning |
| --- | --- |
| `--sumo-config <path>` | SUMO `.sumocfg` used for TraCI replay. |
| `--net-file <path>` | SUMO `.net.xml` file used for network metadata. |
| `--sumo-binary <sumo|sumo-gui|path>` | SUMO binary launched through TraCI. |
| `--outputs <legacy,training,events,cycles,all,none>` | Select CSV outputs. Default `legacy` writes the slim `TraCI_output_adjusted.csv`. |
| `--legacy-edge-output <path>` | Output path for the slim model-training table. Default `TraCI_output_adjusted.csv`. |
| `--training-output <path>` | Fuller debug/training-compatible CSV enabled by `--outputs training` or `all`. |
| `--vehicle-event-output <path>` | Vehicle-movement event debug CSV enabled by `--outputs events` or `all`. |
| `--cycle-summary-output <path>` | Movement-level green-window summary CSV enabled by `--outputs cycles` or `all`. |
| `--waiting-region-length <meters>` | Distance before a signalized stop line treated as the waiting region. |
| `--stop-speed-threshold <m/s>` | Speed at or below this value is treated as stopped. |
| `--queue-speed-threshold <m/s>` | Speed at or below this value inside the waiting region is treated as queued. |
| `--low-speed-threshold <m/s>` | Speed above stop threshold and at or below this value is counted as low-speed time. |
| `--downstream-block-occupancy-threshold <percent>` | Downstream lane occupancy percentage above which storage is treated as blocked. |

### Model training options

| Argument | Meaning |
| --- | --- |
| `--csv <path>` | Slim TraCI training CSV. Default `TraCI_output_adjusted.csv`. |
| `--save_path <dir>` | AutoGluon model output directory. Default `models_v1`. |
| `--time_limit <seconds>` | AutoGluon training time limit. |
| `--test_size <ratio>` | Test split ratio. |
| `--random_state <n>` | Random seed. |
| `--net <path>` | Optional SUMO net path; unused by the default slim base-feature model. |

### Model catching options

| Argument | Meaning |
| --- | --- |
| `--csv <path>` | Training CSV used to discover observed feature values. Default `TraCI_output_adjusted.csv`. |
| `--model_path <dir>` | AutoGluon model directory. Default `models_v1`. |
| `--output_txt <path>` | Space-separated SUMO_V1 table output. Default `model_catching_sumo_v1.txt`. |
| `--max_road_flow <n>` | Maximum `road_flow` to enumerate. Use `-1` to infer from CSV max. |
| `--road_flow_step <n>` | `road_flow` enumeration step. |
| `--max_lane_flow <n>` | Maximum selected-lane `lane_flow` to enumerate. Default is `max_road_flow`. |
| `--lane_flow_step <n>` | Independent `lane_flow` enumeration step. |
| `--chunk_size <n>` | Prediction batch size. |

## 18. README Maintenance Notes

This README uses relative paths such as `test.net.xml`, `map.sumo.cfg`, `all_congestion_routes.rou.xml`, `TraCI_output_adjusted.csv`, and `tripinfo.xml` for reproducibility. Server-specific absolute paths such as `/data5/...` should be treated only as local deployment examples and should not be required for the standard workflow.

Before committing README changes, verify that Markdown code fences are balanced and run any configured Markdown linter if one is added to the repository.
