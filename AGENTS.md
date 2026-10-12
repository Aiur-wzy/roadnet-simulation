# Repository Guidelines

## Project Handbook

When present in a local research checkout, `PROJECT_MANUAL.md` at the repository root is the single ongoing record for project status, milestones, issues, TODOs and research decisions. Before relevant work, read its current-status and TODO sections plus the related issue. After substantive authorized work, update those entries and the change log with scope, evidence and limitations; keep detailed reports and raw results as supporting artifacts. The handbook and research reports are intentionally not public; a clean public checkout does not require them. Do not create a parallel active progress/TODO handbook. A paused item requires an explicit user instruction to resume; recording a candidate does not authorize implementation or experiments.


## Project Structure & Module Organization

The C++14 simulator lives in `simulator/`; CMake remains at the root. Python core code is separated by responsibility: `collection/`, `training/`, `catching/`, `features/`, `data_generation/`, and `evaluation/`. Current model entry points are `training.train_road_time` and `catching.catch_road_time`, with their independent native-v2 contract. Do not add old CSV/model compatibility or legacy imports to these new modules. Older ABC_RG entry points and their `_shared/` dependencies remain at their original paths for historical reproduction, not as dependencies of the new model pipeline. Independent older entry points also live under `legacy/`. The frozen collector dependency tree must not be changed by a model-only migration. `scripts/{collection,training,evaluation}/` contains shell orchestration only; environment/validation/maintenance Python utilities belong under `tools/`. Python and C++ tests live in `tests/python/` and `tests/cpp/`, with small resources in `tests/fixtures/`. Use `roadnet_paths.ROOT` / `source_path` for repository resources and source digests. See `docs/source_layout.json` for the old-to-new mapping and `docs/usage.md` for current commands and scope.

Keep datasets in `data/` / `train_data/`, exported tables in `catch/` (not `catching/`), reports in `docs/`, and experiment evidence in `eval/` or `outputs/`. Do not rewrite frozen manifests or historical source digests during maintenance. Old scientific baselines are not automatically obsolete. Confirm dependencies before recoverable archival; preserve per-file hashes and original paths under `garbage/`. Never move running/paused job dependencies. Treat existing build directories, `cmake_install.cmake`, and the root `Makefile` as generated files, but do not overwrite binaries used by a paused task.

## Build, Test, and Development Commands

- `cmake -S . -B build_current` configures a fresh out-of-source build; choose another fresh name if this directory is already in use.
- `cmake --build build_current -j2` builds `Simulation_Prediction` (use the `.exe` suffix on Windows).
- `./build_current/Simulation_Prediction --help` checks startup and lists supported options.
- `./build_current/Simulation_Prediction --use-sumo --sumo-net data/net/test_new.net.xml --smoke-test` validates the bundled SUMO network without a full experiment.
- `ctest --test-dir build_current --output-on-failure` runs the C++ tests in a freshly configured build.
- `python -m tools.validation.run_python_tests` runs Python unit/mock/wrapper tests; `--include-sumo` runs each registered live SUMO integration class in a separate interpreter to avoid test TraCI stubs contaminating it. Disclose environment-dependent skips. `--dry-run --include-sumo` only prints the plan.
- `python -m training.train_road_time --help` and `python -m catching.catch_road_time --help` inspect current model entry points without fitting/exporting. Older ABC_RG module names are historical, not native-v2 adapters.
- `python -m data_generation.generate_route_on_test_net_full --help` or `python -m collection.TraCI_Python_Cap_Context_ABC_RG --help` inspects data pipeline options. Run module commands from repository root.

## Coding Style & Naming Conventions

Use four-space indentation and preserve local style. C++ must remain C++14-compatible; place opening braces on the declaration line and use `PascalCase` for types and `snake_case` for new helpers. Python follows PEP 8: `snake_case` functions and variables, `PascalCase` classes, and `UPPER_SNAKE_CASE` constants. No formatter or linter is configured, so avoid unrelated reformatting and keep CLI flags descriptive and hyphenated.

## Testing Guidelines

Name Python tests `test_*.py` and use deterministic, small fixtures. Extend `tests/python/test_missed_green.py` for TraCI movement logic. The current CMake defines 24 CTest cases; use a fresh independent build rather than trusting old cached test inventories. Set `ROADNET_TEST_BUILD_DIR` to run Python/native C++ table parity tests. For simulator changes, build cleanly and run the relevant tests plus the smallest authorized scenario. Report the exact network, route, travel-time mode, and output metrics used. No formal coverage threshold is defined. Do not invoke legacy evaluation scripts with `--help`: their historical main blocks do not implement a CLI and may load models and write output.

## Commit & Pull Request Guidelines

Preserve existing user changes in this Git checkout. Use short imperative subjects such as `Fix missed-green cycle accounting`, and keep each commit focused. Pull requests should describe the changed workflow, list validation commands and results, identify affected datasets/configuration, and link related issues. Include metric deltas or screenshots when evaluation behavior changes; avoid committing generated models, build outputs, or large experiment results unless they are intentional review artifacts.

## Configuration & Reproducibility

Set `SUMO_HOME` and add its `bin` directory to `PATH` before TraCI work. Prefer explicit relative CLI paths and deterministic seeds; never commit machine-specific absolute paths.
The public repository contains source, tests, usage documentation and the small
network fixture listed in `data/README.md`. Research notebooks, papers, routes,
truth outputs, trained models and generated catch tables stay local. Do not force
add ignored artifacts. Untracking a file must preserve its local bytes and must
not rewrite Git history. Catch exports remain paired locally with their feature
schema, source model and training threshold. See `docs/usage.md` for commands
and `.gitignore` for the public documentation allowlist.
