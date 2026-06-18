# RoadNet / CAMS Simulation Pipeline

## 1. Project Overview

RoadNet is a cycle-aware macroscopic traffic simulation system built around SUMO network inputs. The C++ simulator parses SUMO roads, lanes, junctions, connections, traffic-light programs, and movement `linkIndex` metadata, then runs a CAMS-style signal-aware simulation and optional prediction-vs-truth evaluation.

The repository supports two workflows:

1. **SUMO-based workflow**: the current recommended workflow for experiments. It starts from a SUMO `.net.xml` file and generated `.rou.xml` demand, uses SUMO and TraCI to create ground truth and training data, and evaluates the C++ CAMS simulation against SUMO `tripinfo.xml`.
2. **Legacy BJ workflow**: the older Manhattan/Beijing-style text-file workflow, kept for compatibility and regression checks.

High-level experiment pipeline:

1. Generate SUMO route demand as a `.rou.xml` file.
2. Run SUMO to generate microscopic `tripinfo.xml` ground truth.
3. Run TraCI to collect road-level model training data and CAMS movement/cycle ground truth.
4. Train a road-level travel-time model. **[TODO]**
5. Convert trained model predictions into a lookup table for fast C++ travel-time lookup. **[TODO]**
6. Run the C++ CAMS simulation with `speed-net`, `min-time`, `table`, `kinematic`, or future `model` travel-time mode.
7. Evaluate predicted travel time against SUMO `tripinfo.xml`.

```text
test.net.xml
    |
    +--> generate_route_on_test_net_full.py
    |       -> all_congestion_routes.rou.xml
    |
    +--> SUMO
    |       -> tripinfo.xml
    |
    +--> TraCI_Python_Adjusted.py
    |       -> cams_model_training_data.csv
    |       -> cams_vehicle_movement_events.csv
    |       -> cams_movement_cycle_summary.csv
    |       -> TraCI_output_adjusted.csv
    |
    +--> model_training / table generation  [TODO]
    |       -> travel_time_table.txt
    |
    +--> C++ Simulation_Prediction
            -> sumo_eval.csv
            -> eval_summary.txt
            -> eval_grouped_metrics.csv
            -> eval_distribution_metrics.csv
```

## 2. Repository Structure

| Path | Role |
| --- | --- |
| `main.cpp` | CLI parsing, SUMO/BJ workflow selection, resolved configuration printing, smoke-test orchestration, full simulation orchestration, and evaluation entry. |
| `head.h` | Core data structures and the `Graph` class, including roads, nodes, movements, signals, vehicles, SUMO raw structures, graph containers, validation declarations, and path plumbing. |
| `data_preparation.cpp` | SUMO net/route parsing, legacy graph readers, graph construction, movement/lane-group/signal-program construction, and route conversion helpers. |
| `data_cleaning.cpp` | SUMO network, connection, signal-program, route, and input validation helpers. |
| `simulation.cpp` | Cycle-aware simulation, movement dispatch, travel-time prediction, SUMO `tripinfo.xml` evaluation, grouped metrics, distribution metrics, and legacy evaluation. |
| `generate_route_on_test_net_full.py` | Route demand generation from predefined valid SUMO route sequences, including the staged `all-congestion` profile. |
| `TraCI_Python_Adjusted.py` | TraCI-based data collector for road-level model training rows, vehicle movement records, cycle summaries, and legacy-compatible edge output. |
| `model_training/` | Planned location for model training and lookup-table generation scripts. **[TODO: directory/scripts are not yet present in this repository.]** |
| `config_defaults.h` | Default relative paths used when command-line arguments are omitted. Prefer explicit CLI paths for reproducible experiments. |
| `CMakeLists.txt` | C++ build configuration for the `Simulation_Prediction` executable. |
| `README.md` | This end-to-end experiment pipeline guide. |

## 3. Environment Setup

### 3.1 C++ build dependencies

**Purpose:** build the C++ CAMS simulator.

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

If the executable starts but fails to load system libraries, set `LD_LIBRARY_PATH` for the libraries installed on that machine. For example, on some Linux servers:

```bash
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH ./build/Simulation_Prediction --help
```

**Expected input:** C++ source files and `CMakeLists.txt`.

**Expected output:** `build/Simulation_Prediction` or `roadnet_sim`.

**Sanity check:**

```bash
./build/Simulation_Prediction --help
```

### 3.2 SUMO and TraCI setup

**Purpose:** enable SUMO simulation, `tripinfo.xml` generation, and TraCI replay.

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

**Expected input:** installed SUMO and configured `SUMO_HOME`.

**Expected output:** SUMO version text and `traci ok`.

**Sanity check:** fix `SUMO_HOME` first if Python reports that `traci` cannot be imported.

## 4. Step 1: Generate SUMO Route Demand

**Purpose:** create SUMO `.rou.xml` demand from predefined valid route sequences in `generate_route_on_test_net_full.py`.

The generator supports route groups such as:

- `ALL`
- `CORE`
- `WEST_IN`
- `NORTH_IN`
- `EAST_IN`
- `SOUTH_IN`
- `SOUTHWEST_IN`
- `ORIGINAL`
- individual route ids such as `route_01_E9_to_E5_original`

Supported departure modes:

- `uniform`
- `poisson`
- `period`

Basic command:

```bash
python generate_route_on_test_net_full.py \
  --output expanded_routes.rou.xml \
  --route-option ALL \
  --mode uniform \
  --num-vehicles 10 \
  --start-time 0 \
  --end-time 1200
```

Total vehicles = number of departure times × number of selected routes. For `poisson`, `--num-vehicles` is the expected departure count.

Recommended large all-congestion file command:

```bash
python generate_route_on_test_net_full.py \
  --profile all-congestion \
  --output all_congestion_routes.rou.xml \
  --manifest-output all_congestion_manifest.csv
```

The `all-congestion` profile is implemented and is intended to include staged free-flow, light, medium, heavy, oversaturated, directional bottleneck, and single-route pressure scenarios.

**Expected input:** built-in route definitions in `generate_route_on_test_net_full.py`.

**Expected output:**

```text
all_congestion_routes.rou.xml
all_congestion_manifest.csv
```

**Sanity check:**

```bash
python generate_route_on_test_net_full.py --help
```

## 5. Step 2: Create or Update SUMO Config

**Purpose:** point a SUMO `.sumocfg` file to the network and generated demand.

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

Choose an `end` time long enough for all vehicles to depart and finish. If vehicles are still active at the end time, extend the simulation horizon before generating final evaluation data.

**Expected input:** `test.net.xml` and `all_congestion_routes.rou.xml`.

**Expected output:** `map.sumo.cfg`.

**Sanity check:**

```bash
sumo -c map.sumo.cfg --check-route-files true
```

If the installed SUMO version does not support `--check-route-files`, run a short SUMO simulation and inspect route-loading errors.

## 6. Step 3: Run SUMO to Generate TripInfo Ground Truth

**Purpose:** generate SUMO microscopic travel-time ground truth for final evaluation.

Command:

```bash
sumo \
  -c map.sumo.cfg \
  --tripinfo-output tripinfo.xml \
  --duration-log.statistics true \
  --no-step-log true
```

Optional GUI command:

```bash
sumo-gui \
  -c map.sumo.cfg \
  --tripinfo-output tripinfo.xml
```

`tripinfo.xml` contains per-vehicle SUMO truth. The C++ evaluator reads each `<tripinfo>` record's `duration` as ground-truth travel time and uses `arrival` as a secondary arrival-time check. Records are aligned by SUMO vehicle id, not by XML line order.

**Expected input:** `map.sumo.cfg`, `test.net.xml`, and `all_congestion_routes.rou.xml`.

**Expected output:**

```text
tripinfo.xml
```

**Sanity check:** confirm that `tripinfo.xml` contains `<tripinfo ... id="..." duration="..." .../>` records for the vehicles you expect to evaluate.

## 7. Step 4: Run TraCI to Generate Model Training Data

**Purpose:** replay the SUMO simulation through TraCI and collect road-level training data plus CAMS movement/cycle debug files.

Command:

```bash
python TraCI_Python_Adjusted.py \
  --sumo-config map.sumo.cfg \
  --net-file test.net.xml \
  --sumo-binary sumo \
  --training-output cams_model_training_data.csv \
  --vehicle-event-output cams_vehicle_movement_events.csv \
  --cycle-summary-output cams_movement_cycle_summary.csv \
  --legacy-edge-output TraCI_output_adjusted.csv \
  --waiting-region-length 80 \
  --stop-speed-threshold 0.1 \
  --queue-speed-threshold 2.0 \
  --low-speed-threshold 2.0 \
  --downstream-block-occupancy-threshold 85
```

Training label:

```text
travel_time_label = driving_time + low_speed_time
```

The label is intended to represent pure road travel time before crossing the stop line. It excludes red-light waiting, green-queue waiting, and downstream-block waiting. Debug fields such as `red_light_waiting_time`, `green_queue_wait_time`, and `downstream_block_wait_time` are retained for analysis but are not included in `travel_time_label`.

**Expected input:** `map.sumo.cfg`, `test.net.xml`, and a working SUMO/TraCI environment.

**Expected output:**

```text
cams_model_training_data.csv
cams_vehicle_movement_events.csv
cams_movement_cycle_summary.csv
TraCI_output_adjusted.csv
```

**Sanity check:**

```bash
head cams_model_training_data.csv
```

Verify that `travel_time_label > 0`, flow columns cover a useful range, and debug waiting-time fields are present but not part of the label formula.

## 8. Step 5: Train Road-Level Travel-Time Model [TODO]

**Purpose:** train a road-level model that predicts `travel_time_label` from static road features and dynamic traffic-state features.

This section is intentionally a placeholder. The model-training workflow will be completed later.

**Expected input:**

```text
cams_model_training_data.csv
```

**Expected output:**

```text
trained_model/
model_metrics.json
```

Placeholder command:

```bash
python model_training/train_travel_time_model.py \
  --input cams_model_training_data.csv \
  --output-dir trained_model \
  --target travel_time_label
```

Expected feature columns:

```text
road_length
turn_type
road_flow
lane_flow
lane_num
speed_limit
vehicle_length
vehicle_min_gap
lane_capacity
lane_occupied_length
has_waiting
waiting_duration
```

TODO:

- finalize feature preprocessing
- finalize train/validation/test split strategy
- choose model family
- save trained model artifact
- write evaluation report

**Sanity check [TODO]:** after implementation, confirm that `model_metrics.json` reports validation/test metrics and that the saved artifact can reload without retraining.

## 9. Step 6: Generate Travel-Time Lookup Table [TODO]

**Purpose:** precompute model predictions into a fast lookup table for the C++ simulator. This avoids querying an ML model online for every road event during simulation.

This section is intentionally a placeholder. The table-generation workflow will be completed later.

**Expected input:**

```text
trained_model/
feature_grid_config.yaml
```

**Expected output:**

```text
travel_time_table.txt
```

Placeholder command:

```bash
python model_training/build_travel_time_table.py \
  --model-dir trained_model \
  --output travel_time_table.txt \
  --grid-config feature_grid_config.yaml
```

Current C++ table mode uses a `RoadKey` dictionary derived from static and dynamic cycle-aware road state. Static fields include lane count, rounded speed limit, and rounded length; dynamic fields include current running vehicle count and waiting-buffer occupancy. Delay and low-speed features are currently set to `0`, so table matching is approximate.

TODO: migrate the table schema from the current `RoadKey` format to the future `BasicRoadModelFeatures` schema used by the road-level training dataset.

**Sanity check [TODO]:** after implementation, run a small C++ simulation with `--travel-time-mode table --verbose-travel-time` and verify that table hit rates are acceptable and fallback usage is understood.

## 10. Step 7: Run C++ CAMS Simulation

**Purpose:** run the cycle-aware C++ simulation on the SUMO network and generated route demand.

Start with a smoke test:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --smoke-test
```

The smoke test parses and validates the SUMO network, then prints preparation details such as parsed edges, skipped internal edges, junctions, traffic lights, connections, `tlLogic` programs, phases, movements, lane groups, waiting buffers, validation summary, and sample signal states.

No-evaluation simulation run:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --travel-time-mode speed-net \
  --read-num 100000 \
  --no-eval
```

Table-mode simulation run:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --travel-time-mode table \
  --travel-time-table travel_time_table.txt \
  --tt-fallback speed-net \
  --read-num 100000 \
  --no-eval
```

Available travel-time modes:

```text
speed-net
min-time
table
model
kinematic
```

Mode meanings:

- `speed-net`: uses `road.length / road.speedLimit` with safety clamping and a minimum return value.
- `min-time`: uses precomputed minimum travel time when available, otherwise falls back to `speed-net`.
- `table`: looks up a precomputed table and falls back according to `--tt-fallback` when the key is missing.
- `model`: reserved for a future external model service; the current implementation falls back according to `--tt-fallback`.
- `kinematic`: uses the current kinematic/congestion approximation when available.

Supported SUMO route XML formats include global route definitions with vehicle route references and inline routes nested inside vehicles. Internal SUMO edges whose ids begin with `:` are skipped during route conversion. Vehicles whose remaining route references an unknown edge are skipped with a `[SUMO Route Warning]` instead of being mapped to road id `0`.

**Expected input:** `test.net.xml`, `all_congestion_routes.rou.xml`, and optionally `travel_time_table.txt`.

**Expected output:** simulator logs and, when `--no-eval` is omitted with truth data, evaluation outputs.

**Sanity check:** run the smoke test before full simulation and inspect route warnings before trusting large-batch results.

## 11. Step 8: Run Final Evaluation Against SUMO TripInfo

**Purpose:** compare C++ predicted vehicle travel times with SUMO microscopic ground truth.

Command:

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --sumo-tripinfo tripinfo.xml \
  --travel-time-mode table \
  --travel-time-table travel_time_table.txt \
  --tt-fallback speed-net \
  --read-num 100000 \
  --eval-output sumo_eval.csv
```

For baseline evaluation before the table workflow is complete, replace `--travel-time-mode table --travel-time-table travel_time_table.txt --tt-fallback speed-net` with `--travel-time-mode speed-net` or `--travel-time-mode min-time`.

**Expected input:** `test.net.xml`, `all_congestion_routes.rou.xml`, and `tripinfo.xml`.

**Expected output:**

```text
sumo_eval.csv
eval_summary.txt
eval_grouped_metrics.csv
eval_distribution_metrics.csv
```

Output meanings:

- `sumo_eval.csv`: per-vehicle prediction-vs-truth rows.
- `eval_summary.txt`: overall MAE, MSE, RMSE, MAPE, bias, coverage, and distribution-summary values.
- `eval_grouped_metrics.csv`: grouped error analysis by duration, waiting time, time loss, route length, and movement count.
- `eval_distribution_metrics.csv`: distribution-level comparison of predicted and truth durations.

**Sanity check:** confirm that the evaluator reports matched records. Vehicle ids are matched by id, not by XML line order.

## 12. Recommended End-to-End Command Sequence

```bash
# 1. Build C++ simulator
cmake -S . -B build
cmake --build build -j2

# 2. Generate large route demand
python generate_route_on_test_net_full.py \
  --profile all-congestion \
  --output all_congestion_routes.rou.xml \
  --manifest-output all_congestion_manifest.csv

# 3. Generate SUMO tripinfo ground truth
sumo \
  -c map.sumo.cfg \
  --tripinfo-output tripinfo.xml \
  --duration-log.statistics true \
  --no-step-log true

# 4. Generate TraCI training data
python TraCI_Python_Adjusted.py \
  --sumo-config map.sumo.cfg \
  --net-file test.net.xml \
  --training-output cams_model_training_data.csv

# 5. Train model [TODO]
python model_training/train_travel_time_model.py \
  --input cams_model_training_data.csv \
  --output-dir trained_model \
  --target travel_time_label

# 6. Build lookup table [TODO]
python model_training/build_travel_time_table.py \
  --model-dir trained_model \
  --output travel_time_table.txt

# 7. Run final simulation and evaluation
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --sumo-route all_congestion_routes.rou.xml \
  --sumo-tripinfo tripinfo.xml \
  --travel-time-mode table \
  --travel-time-table travel_time_table.txt \
  --tt-fallback speed-net \
  --read-num 100000 \
  --eval-output sumo_eval.csv
```

## 13. Output Files

| File | Produced by | Purpose |
| --- | --- | --- |
| `all_congestion_routes.rou.xml` | route generator | SUMO route demand. |
| `all_congestion_manifest.csv` | route generator | Scenario metadata. |
| `tripinfo.xml` | SUMO | Microscopic ground truth. |
| `cams_model_training_data.csv` | TraCI | Road-level model training dataset. |
| `cams_vehicle_movement_events.csv` | TraCI | Movement-level signal event debug data. |
| `cams_movement_cycle_summary.csv` | TraCI | Signal-cycle discharge summary. |
| `TraCI_output_adjusted.csv` | TraCI | Compact legacy-compatible summary. |
| `trained_model/` | model training [TODO] | Trained model artifact. |
| `travel_time_table.txt` | table generation [TODO] | Fast lookup table for C++ simulation. |
| `sumo_eval.csv` | C++ simulator | Per-vehicle prediction-vs-truth evaluation. |
| `eval_summary.txt` | C++ simulator | Overall evaluation summary. |
| `eval_grouped_metrics.csv` | C++ simulator | Grouped error analysis. |
| `eval_distribution_metrics.csv` | C++ simulator | Distribution-level error analysis. |

## 14. Debugging and Sanity Checks

### Check route generation

```bash
python generate_route_on_test_net_full.py --help
```

Confirm that the needed route groups, `--profile all-congestion`, and departure options are listed.

### Check SUMO config

```bash
sumo -c map.sumo.cfg --check-route-files true
```

If `--check-route-files` is unsupported in the installed SUMO version, run a short SUMO simulation and inspect route errors:

```bash
sumo -c map.sumo.cfg --end 300 --no-step-log true
```

### Check TraCI data

After running TraCI, inspect:

```bash
head cams_model_training_data.csv
```

Check that:

- `travel_time_label > 0`.
- `road_flow` and `lane_flow` cover low, medium, and high values.
- `turn_type` includes left, straight, right, and other turns when available.
- `red_light_waiting_time`, `green_queue_wait_time`, and `downstream_block_wait_time` are debug fields and are not included in `travel_time_label`.

### Check C++ simulator

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net test.net.xml \
  --smoke-test
```

Common issues to inspect:

- Missing required files: readers fail with messages such as `[Fatal] ... cannot open required file '<path>'`.
- Invalid SUMO connection edge references.
- `linkIndex` outside the active phase `state` string.
- Route road pair missing a movement.
- Query/route/time size mismatch in legacy mode.
- Table-mode misses; use `--verbose-travel-time` while debugging lookup tables.

Useful internal validation helpers include `validate_sumo_network()`, `validate_sumo_connections()`, `validate_sumo_signal_programs()`, and `validate_sumo_routes()`.

## 15. Legacy BJ Workflow

**Purpose:** run the older text-file workflow for compatibility with Manhattan/Beijing-style data.

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

**Expected input:** legacy text files.

**Expected output:** simulator logs and legacy evaluation output unless `--no-eval` is set.

**Sanity check:** start with a small `--read-num` such as `1000` before running the full legacy dataset.

## 16. Command-Line Reference

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
| `--travel-time-table <path>` | Dictionary path for table-mode lookups. | Derived default table path | Both |
| `--tt-fallback <speed-net|min-time>` | Fallback predictor for table misses or unimplemented model mode. | `speed-net` | Both |
| `--model-host <host>` | Host reserved for a future external model service. | `127.0.0.1` | Both |
| `--model-port <port>` | Port reserved for a future external model service. | `9000` | Both |
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
| `--training-output <path>` | Output CSV for road-level CAMS model training rows. |
| `--vehicle-event-output <path>` | Output CSV for vehicle-movement ground-truth events. |
| `--cycle-summary-output <path>` | Output CSV for movement-level green-window summaries. |
| `--legacy-edge-output <path>` | Compatibility CSV containing compact vehicle-edge summaries. |
| `--waiting-region-length <meters>` | Distance before a signalized stop line treated as the waiting region. |
| `--stop-speed-threshold <m/s>` | Speed at or below this value is treated as stopped. |
| `--queue-speed-threshold <m/s>` | Speed at or below this value inside the waiting region is treated as queued. |
| `--low-speed-threshold <m/s>` | Speed above stop threshold and at or below this value is counted as low-speed time. |
| `--downstream-block-occupancy-threshold <percent>` | Downstream lane occupancy percentage above which storage is treated as blocked. |

## 17. README Quality Requirements

This README uses relative paths such as `test.net.xml`, `map.sumo.cfg`, `all_congestion_routes.rou.xml`, and `tripinfo.xml` for reproducibility. Server-specific absolute paths such as `/data5/...` should be treated only as local deployment examples and should not be required for the standard workflow.

Each pipeline stage above includes:

1. purpose
2. command
3. expected input
4. expected output
5. sanity check

Unfinished model-training and table-generation stages are explicitly marked as **[TODO]**. Before committing README changes, verify that Markdown code fences are balanced and run any configured Markdown linter if one is added to the repository.
