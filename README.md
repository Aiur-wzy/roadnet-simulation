# RoadNet Simulation

## 1. Project Overview

This project is a macroscopic road traffic simulation system. It supports two input workflows:

- **Legacy BJ text-network workflow** for the original Manhattan/Beijing-style text files.
- **SUMO `.net.xml` workflow** that parses real SUMO edges, lanes, junctions, connections, `tlLogic`, phases, and movement-level signal metadata.

The simulation code includes cycle-aware / signal-driven traffic logic. In SUMO mode, movement signal state is derived from each connection's `linkIndex` and the active SUMO `tlLogic` phase, so `phase.state[linkIndex]` determines whether that movement is red, yellow, green, or always open at a given time.

## 2. Code Structure

- `head.h`  
  Core data structures and the `Graph` class. This includes road, node, movement, signal, vehicle, SUMO raw structures, graph containers, validation declarations, and default-path plumbing.

- `config_defaults.h`  
  Small default-path file. Edit `roadnet_defaults::DEFAULT_BASE_DIR` here only if you do not want to pass paths on the command line.

- `main.cpp`  
  Program entry point, command-line parsing, environment-variable fallback, workflow selection, resolved-configuration printing, SUMO smoke-test orchestration, simulation orchestration, and optional evaluation.

- `data_preparation.cpp`  
  Legacy graph readers, query/route/time readers, SUMO `.net.xml` parser, graph construction, movement construction, lane-group construction, signal-program construction, and route conversion helpers.

- `data_cleaning.cpp`  
  Validation functions for old graph consistency and SUMO network, connection, signal-program, and route checks.

- `simulation.cpp`  
  Legacy simulation utilities, cycle-aware signal-driven simulation, ETA/evaluation helpers, and traffic prediction utilities.

- `update.cpp` and related update/delete files (`deletion_operation.cpp`, `eta_update.cpp`)  
  Dynamic update and legacy maintenance logic used by the older workflow.

- `CMakeLists.txt`  
  Build configuration for the executable and thread/Boost linking.

## 3. Build Instructions

### CMake

```bash
cmake -S . -B build
cmake --build build -j2
```

The CMake target is currently named `Simulation_Prediction`. You can run it as `./build/Simulation_Prediction` after building.

### Direct `g++`

```bash
g++ -std=c++14 -O0 -g \
  main.cpp data_preparation.cpp simulation.cpp data_cleaning.cpp \
  update.cpp deletion_operation.cpp eta_update.cpp \
  -o roadnet_sim -lpthread
```

The exact source list may need to match the current repository if files are added or removed.

## 4. Quick Start: SUMO Smoke Test

```bash
./roadnet_sim --use-sumo --sumo-net test.net.xml --smoke-test
```

Expected output includes a resolved configuration summary and SUMO preparation details such as:

- parsed SUMO edges
- normal roads
- skipped internal edges
- junctions
- traffic lights
- connections
- `tlLogic` programs
- phases
- movements
- lane groups
- waiting buffers
- validation summary
- sample signal states at `t=0,10,42,45,90`

The old environment-variable style also still works:

```bash
USE_SUMO_NET=1 SUMO_NET_PATH=test.net.xml ./roadnet_sim
```


## 5. Quick Start: SUMO Full Simulation Without Evaluation

```bash
./build/Simulation_Prediction \
  --use-sumo \
  --sumo-net /data5/zhiyuan/roadnet_simulation/test.net.xml \
  --sumo-route /data5/zhiyuan/roadnet_simulation/high.rou.xml \
  --travel-time-mode speed-net \
  --read-num 1000 \
  --no-eval
```

This full SUMO workflow uses real SUMO network, connection, movement, and signal-plan data:

- `--sumo-net` provides the road network, lanes, junctions, connections, movement `linkIndex` values, and signal plans from a SUMO `.net.xml` file.
- `--sumo-route` provides vehicles, departure times, and SUMO edge routes from a `.rou.xml` file.
- `--travel-time-mode speed-net` predicts each road's travel time as `road.length / road.speedLimit` with safety clamping and a minimum return value of one second.
- `--no-eval` skips MSE/MAE/RMSE/MAPE-style evaluation because `high.rou.xml` contains route demand rather than ground-truth travel times.

Supported SUMO route XML formats:

1. Global route definitions with vehicle route references:

   ```xml
   <route id="fixed_route_e9" edges="E9 -E12 E20 E15 -E0 E5"/>
   <vehicle id="veh_0" route="fixed_route_e9" depart="0.00"/>
   ```

2. Inline routes nested inside vehicles:

   ```xml
   <vehicle id="veh_0" depart="0.00">
     <route edges="E9 -E12 E20 E15 -E0 E5"/>
   </vehicle>
   ```

Internal SUMO edges whose IDs begin with `:` are skipped during route conversion. Vehicles whose remaining route references an unknown edge are skipped with a `[SUMO Route Warning]` message rather than being mapped to road ID `0`.

## 6. Quick Start: Legacy BJ Workflow

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

## 7. Command-Line Arguments

| Argument | Meaning | Default | Workflow |
| --- | --- | --- | --- |
| `--help` | Print usage and exit. | Off | Both |
| `--use-sumo` | Enable the SUMO `.net.xml` workflow. Without it, legacy BJ is the default. | Off | SUMO |
| `--smoke-test` | In SUMO mode, run preparation, validation, sample signal-state output, then exit. | Off | SUMO |
| `--base <dir>` | Derive standard input paths from `<dir>`. | `./data/Manhattan_Data` from `config_defaults.h` | Both |
| `--sumo-net <path>` | SUMO `.net.xml` input path. | `./test.net.xml` | SUMO |
| `--sumo-route <path>` | SUMO `.rou.xml` route file used for full SUMO simulation. | Empty | SUMO |
| `--bj <path>` | Legacy BJ graph file. | `<base>/Manhattan_network_BJ.txt` | Legacy |
| `--bj-min-time <path>` | Legacy min-travel-time file. | `<base>/Manhattan_network_min_Travel_Time.txt` | Legacy |
| `--road-info <path>` | Legacy road-info file. | `<base>/beijingMoreRoadInfo` | Legacy |
| `--query <path>` | Query file. | `<base>/query.txt` | Both route-data workflows |
| `--route <path>` | Route file. | `<base>/route.txt` | Both route-data workflows |
| `--time <path>` | Observed/ground-truth time file. | `<base>/time.txt` | Legacy evaluation |
| `--time-no-wait <path>` | Optional no-wait time file used by related readers. | `<base>/time_no_wait.txt` | Legacy/helper |
| `--read-num <n>` | Number of query/route/time records to read. In SUMO route mode, this limits valid vehicles when greater than zero. Values larger than file length read the full file. | `192484` | Both |
| `--cut` | Cut route/query/time data before simulation. | `false` | Legacy |
| `--avg-length <n>` | Average length used by the cut helpers. | `30` | Legacy |
| `--no-eval` | Skip the final MSE/evaluation step. | Evaluation enabled | Both |
| `--travel-time-mode <speed-net|min-time|table|model>` | Select the single-road travel-time predictor used by cycle-aware simulation. | `min-time` | Both |
| `--travel-time-table <path>` | Dictionary path for table-mode lookups. | `<base>/model_catching_with_travel_time_1.txt` | Both |
| `--tt-fallback <speed-net|min-time>` | Fallback predictor for table misses or unimplemented model mode. | `speed-net` | Both |
| `--model-host <host>` | Host reserved for a future external model service. | `127.0.0.1` | Both |
| `--model-port <port>` | Port reserved for a future external model service. | `9000` | Both |
| `--verbose-travel-time` | Print per-miss travel-time diagnostics. | Off | Both |

Environment variables are supported as fallback only and have lower priority than command-line arguments:

```bash
USE_SUMO_NET=1 SUMO_NET_PATH=test.net.xml ./roadnet_sim
```


## 8. Travel-Time Prediction Modes

`Graph::predictRoadTravelTime(roadID, vehicleID)` is configurable with `--travel-time-mode`:

- `--travel-time-mode speed-net`: uses `road.length / road.speedLimit` for each road segment.
- `--travel-time-mode min-time`: uses precomputed `minTravelTime` when available, otherwise falls back to speed-net. This is the default and preserves the previous behavior.
- `--travel-time-mode table`: builds a `RoadKey` from the current cycle-aware road state and looks up the dictionary from `--travel-time-table`. If no key is found, it falls back according to `--tt-fallback speed-net|min-time`.
- `--travel-time-mode model`: reserved for an external model service. The current implementation calls the `queryExternalTravelTimeModel(...)` stub, prints a warning once when no model is implemented, and falls back according to `--tt-fallback`.

Example speed-net run:

```bash
./roadnet_sim --travel-time-mode speed-net
```

Example min-time run:

```bash
./roadnet_sim --travel-time-mode min-time
```

Example table run:

```bash
./roadnet_sim \
  --travel-time-mode table \
  --travel-time-table data/model_catching_with_travel_time_1.txt \
  --tt-fallback speed-net
```

Example model-stub run:

```bash
./roadnet_sim \
  --travel-time-mode model \
  --model-host 127.0.0.1 \
  --model-port 9000 \
  --tt-fallback min-time
```

Table-mode `RoadKey` values are constructed from available static and dynamic cycle-aware fields. Static road fields use lane count, rounded speed limit, and rounded length; dynamic fields use current running vehicle count plus waiting-buffer occupancy. Delay and low-speed features are currently set to `0`, so table-mode matching is approximate until those features are represented directly in the cycle-aware simulator.

## 9. Default Paths: Where to Change If Not Using Args

The recommended way to run experiments is to pass command-line arguments, because commands are reproducible and explicit.

If you do not want to use command-line arguments, edit the default path section in:

```cpp
// config_defaults.h
static const std::string DEFAULT_BASE_DIR = "./data/Manhattan_Data";
static const std::string DEFAULT_SUMO_NET_PATH = "./test.net.xml";
```

Changing `DEFAULT_BASE_DIR`, or calling `Graph::set_base_path(baseDir)`, updates the standard derived paths:

- `BJ`
- `BJ_minTravleTime`
- `beijingMoreRoadInfo`
- `queryPath`
- `route_path`
- `time_path`
- `time_path_no_wait`
- `sumoNetPath`

Example:

```cpp
static const std::string DEFAULT_BASE_DIR = "./data/Manhattan_Data";
```

Explicit path arguments override base-derived paths. For example, the following uses `other/query.txt` for queries while all other standard files come from `data/Manhattan_Data`:

```bash
./roadnet_sim --base data/Manhattan_Data --query other/query.txt
```

## 10. Data Format Notes

### Legacy BJ data

- BJ network file: `node1 node2 edgeID length`
- Min travel time file: `node1 node2 minTime`
- Road info file: additional road attributes such as direction, speed limit, lane count, width, and kind.
- Query, route, and time files are aligned by index. The `i`th query corresponds to the `i`th route and `i`th time record.

### SUMO data

- `edge` and `lane` elements define roads and lane geometry/speed/length.
- `junction` elements define nodes and traffic lights.
- `connection` elements define from-road to-road movements and lane mappings.
- `tlLogic` and `phase` elements define traffic-light timing.
- `phase.state[linkIndex]` determines the signal state for the movement associated with that SUMO connection.

## 11. Validation and Debugging

SUMO validation helpers include:

- `validate_sumo_network()`
- `validate_sumo_connections()`
- `validate_sumo_signal_programs()`
- `validate_sumo_routes()`

Common errors to check:

- Missing file path: required readers now fail with `[Fatal] ... cannot open required file '<path>'`.
- Invalid SUMO connection edge references.
- `linkIndex` outside the active phase `state` string.
- Route road pair missing a movement.
- Query/route/time size mismatch.

## 12. Suggested Development Workflow

1. First run the SUMO smoke test.
2. Check the parsed network summary.
3. Check movement construction and sample signal-state output.
4. Connect route/query/time data.
5. Run the full cycle-aware simulation.
6. Evaluate MSE/MAE/RMSE/MAPE or skip evaluation with `--no-eval` while debugging.

## Example Startup Output

SUMO mode:

```text
[Config] Mode: SUMO
[Config] Smoke test: true
[Config] Base: ./data/Manhattan_Data
[Config] SUMO net: test.net.xml
[Config] Query: ./data/Manhattan_Data/query.txt
[Config] Route: ./data/Manhattan_Data/route.txt
[Config] Time: ./data/Manhattan_Data/time.txt
[Config] Time no wait: ./data/Manhattan_Data/time_no_wait.txt
[Config] Read num: 192484
[Config] Cut: false
[Config] Avg length: 30
[Config] Evaluation: true
```

Legacy mode remains the default when `--use-sumo` is not provided:

```text
[Config] Mode: Legacy BJ
[Config] Smoke test: false
[Config] Base: ./data/Manhattan_Data
[Config] BJ graph: ./data/Manhattan_Data/Manhattan_network_BJ.txt
[Config] BJ min travel time: ./data/Manhattan_Data/Manhattan_network_min_Travel_Time.txt
[Config] Road info: ./data/Manhattan_Data/beijingMoreRoadInfo
[Config] Query: ./data/Manhattan_Data/query.txt
[Config] Route: ./data/Manhattan_Data/route.txt
[Config] Time: ./data/Manhattan_Data/time.txt
```
