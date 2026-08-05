# Repository Guidelines

## Project Structure & Module Organization

The C++14 simulator lives at the root. `main.cpp` handles CLI orchestration, `head.h` defines shared types, and `data_preparation.cpp`, `data_cleaning.cpp`, and `simulation.cpp` implement network loading, validation, simulation, and evaluation. Root Python programs generate routes, collect TraCI features, train AutoGluon models, and export lookup tables. Scenario networks and routes belong in `data/` or `train_data/`; analysis and batch tools belong in `scripts/`. Keep reports in `docs/` and evaluation artifacts in `eval/`. Treat `build/`, `cmake-build-debug/`, `cmake_install.cmake`, and the root `Makefile` as generated files.

## Build, Test, and Development Commands

- `cmake -S . -B build` configures a fresh out-of-source build.
- `cmake --build build -j2` builds `Simulation_Prediction` (use the `.exe` suffix on Windows).
- `./build/Simulation_Prediction --help` checks startup and lists supported options.
- `./build/Simulation_Prediction --use-sumo --sumo-net test.net.xml --smoke-test` validates the bundled SUMO network without a full experiment.
- `python -m unittest test_missed_green.py` runs the current automated unit tests.
- `python generate_route_on_test_net_full.py --help` or `python TraCI_Python_Adjusted.py --help` inspects pipeline options before launching data-heavy jobs.

## Coding Style & Naming Conventions

Use four-space indentation and preserve local style. C++ must remain C++14-compatible; place opening braces on the declaration line and use `PascalCase` for types and `snake_case` for new helpers. Python follows PEP 8: `snake_case` functions and variables, `PascalCase` classes, and `UPPER_SNAKE_CASE` constants. No formatter or linter is configured, so avoid unrelated reformatting and keep CLI flags descriptive and hyphenated.

## Testing Guidelines

Name Python tests `test_*.py` and use deterministic, small fixtures. Extend `test_missed_green.py` for TraCI movement logic. C++ has no CTest target; for simulator changes, build cleanly and run the SUMO smoke test plus the smallest relevant scenario. Report the exact network, route, travel-time mode, and output metrics used. No formal coverage threshold is defined.

## Commit & Pull Request Guidelines

This checkout contains no Git history, so no repository-specific message pattern can be inferred. Use short imperative subjects such as `Fix missed-green cycle accounting`, and keep each commit focused. Pull requests should describe the changed workflow, list validation commands and results, identify affected datasets/configuration, and link related issues. Include metric deltas or screenshots when evaluation behavior changes; avoid committing generated models, build outputs, or large experiment results unless they are intentional review artifacts.

## Configuration & Reproducibility

Set `SUMO_HOME` and add its `bin` directory to `PATH` before TraCI work. Prefer explicit relative CLI paths and deterministic seeds; never commit machine-specific absolute paths.
