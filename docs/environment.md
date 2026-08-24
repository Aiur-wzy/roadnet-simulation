# Environment and Dependency Specification

This document pins down the runtime, build, and package requirements for the
RoadNet / CAMS pipeline. It was derived from the actual imports and build
configuration in this repository (see `requirements.txt` for the pip-installable
subset).

## 1. Python

| Item | Requirement | Notes |
| --- | --- | --- |
| Recommended version | **3.10 – 3.12** | Practical floor set by AutoGluon 1.x. Local dev verified on 3.10.11. |
| Syntax floor | 3.7 | The code itself only needs `dataclasses` and `from __future__ import annotations`; no walrus/match/3.10-only syntax. AutoGluon is what forces 3.9+. |

### Pip packages

| Package | Min version | Used by | Verified version (local dev) |
| --- | --- | --- | --- |
| pandas | >= 1.5 | TraCI collectors, training, catching, all analysis scripts | 2.3.3 |
| numpy | >= 1.23 | Training, catching, analysis | 2.2.6 |
| autogluon.tabular | >= 1.1 | `model_training_sumo_v1/v2.py`, `model_catching_sumo_v1.py`, `vehicle_feature_pipeline/*` | (server-side) |
| scikit-learn | >= 1.0 | Training/eval splits and metrics, feature analysis | (server-side) |
| matplotlib | >= 3.7 | `model_evaluation_v1*.py` only — optional | 3.10.8 |
| scipy | >= 1.9 | `train_data/build_balanced_training_routes.py` only (`scipy.optimize.milp`) — optional | 1.15.3 |

Everything else imported by the Python code is standard library
(`argparse, csv, math, os, re, statistics, json, glob, bisect, collections,
dataclasses, pathlib, random, shutil, sys, tempfile, types, typing, unittest,
warnings, xml.*, importlib.util`).

### TraCI (not a pip package)

`traci` and `traci.constants` are imported by `TraCI_Python_Adjusted.py`,
`TraCI_Python_Threshold_Sweep.py`, and `vehicle_feature_pipeline/traci_vehicle_features.py`.
They ship with SUMO, not PyPI:

```bash
export SUMO_HOME=/usr/share/sumo
export PATH=$SUMO_HOME/bin:$PATH
python -c "import os, sys; sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools')); import traci; print('traci ok')"
```

## 2. SUMO

| Item | Requirement |
| --- | --- |
| Version | **>= 1.20** — the bundled networks (`test.net.xml`, `train_data/variants/new4x/inputs/test_new.net.xml`) are net-file format `version="1.20"`; older SUMO releases cannot read them. |
| Components | `sumo` binary on `PATH`; `tools/` importable by Python for TraCI. |

## 3. C++ simulator

| Item | Requirement | Notes |
| --- | --- | --- |
| CMake | >= 3.20 | From `cmake_minimum_required(VERSION 3.20)`. |
| Language standard | C++14 (required) | `CMAKE_CXX_STANDARD 14`, `CXX_STANDARD_REQUIRED ON`. |
| Compiler (Linux) | g++ >= 5 | Any compiler with full C++14 support. |
| Compiler (macOS) | clang >= 3.4 (Xcode 8+) | |
| Compiler (Windows) | MSVC >= 19.14 (VS 2017 15.7) | |
| Third-party C++ libraries | **None** | Sources include only the C++ standard library and POSIX `sys/types.h`. No Boost, no threads: the README's `-lpthread`/Boost notes are legacy; the current `CMakeLists.txt` links nothing extra. |

Build and smoke test:

```bash
cmake -S . -B build
cmake --build build -j2
./build/Simulation_Prediction --use-sumo --sumo-net test.net.xml --smoke-test
```

## 4. Shell tooling

`run_all_training_traci.sh` and the wrappers under `train_data/variants/*/commands/`
require `bash` plus standard GNU userland tools (`awk`, coreutils). No `jq`,
`sed` beyond coreutils, or other external tools are used.

## 5. GPU (optional)

Training scripts accept `--num_gpus`. CPU-only AutoGluon works fine; GPU
acceleration additionally needs a CUDA-enabled PyTorch build (and, for the
`best_quality` preset, GPU builds of LightGBM/XGBoost/CatBoost help but are not
required). This only affects training speed, not correctness.

## 6. Verification checklist

```bash
python --version                                  # 3.10-3.12
pip install -r requirements.txt
python -c "import pandas, numpy, sklearn; print('core ok')"
python -c "from autogluon.tabular import TabularPredictor; print('autogluon ok')"
sumo --version                                    # >= 1.20
python -c "import os, sys; sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools')); import traci; print('traci ok')"
cmake --version                                   # >= 3.20
g++ --version                                     # >= 5
cmake -S . -B build && cmake --build build -j2
./build/Simulation_Prediction --use-sumo --sumo-net test.net.xml --smoke-test
python -m unittest test_missed_green.py
```
