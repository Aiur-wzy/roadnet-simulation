# Single-group route generation summary

## Purpose

The route generator now supports generating one congestion group, selected congestion groups, or every congestion group as separate route files. This keeps large generated `.rou.xml` files out of review while preserving the exact scenario definitions already used by the combined all-congestion route generation path.

This PR commits lightweight `.sumocfg` files only. They document the intended per-group experiment setup and point to future local route and tripinfo paths.

## Modified script

Script: `generate_route_on_test_net_full.py`

New CLI options:

- `--list-groups`: print all available congestion groups and metadata, then exit.
- `--group <group_name>`: generate one selected group into a single `.rou.xml` file.
- `--groups <comma-separated group names>`: generate multiple selected groups as separate `.rou.xml` files.
- `--all-groups`: generate every known group as separate `.rou.xml` files.
- `--output <path>`: output route path for `--group`.
- `--output-dir <path>`: output directory for `--groups` or `--all-groups` route files.
- `--prefix <prefix>`: filename suffix used for per-group generated files.
- `--force`: allow overwriting generated route and sumocfg files.
- `--write-sumocfg-only`: write matching `.sumocfg` files without generating route XML.
- `--sumocfg`: write matching `.sumocfg` files alongside generated route XML.
- `--net-file <path>`: network file referenced by generated `.sumocfg` files.
- `--tripinfo-output-dir <path>`: directory used for future tripinfo output paths inside `.sumocfg` files.
- `--sumocfg-output-dir <path>`: directory where generated `.sumocfg` files are written.

The original combined generation behavior is preserved through:

```bash
python3 generate_route_on_test_net_full.py \
  --profile all-congestion \
  --output data/full_no_change.rou.xml \
  --manifest-output data/all_congestion_manifest.csv
```

## Available groups

| group | description | route family | density / period | time window | seed if relevant | future route file | sumocfg file | future tripinfo output |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `free_all` | Free-flow baseline across all expanded routes. | `ALL` | uniform, num_departures=50 | 0-10000 |  | `data/free_all_no_change_random_offset_seed20260708.rou.xml` | `data/free_all_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_free_all_no_change_random_offset_seed20260708.xml` |
| `light_all_seed1` | Light poisson demand across all expanded routes using seed 1. | `ALL` | poisson, num_departures=100 | 11000-21000 | 1 | `data/light_all_seed1_no_change_random_offset_seed20260708.rou.xml` | `data/light_all_seed1_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_light_all_seed1_no_change_random_offset_seed20260708.xml` |
| `light_all_seed2` | Light poisson demand across all expanded routes using seed 2. | `ALL` | poisson, num_departures=100 | 22000-32000 | 2 | `data/light_all_seed2_no_change_random_offset_seed20260708.rou.xml` | `data/light_all_seed2_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_light_all_seed2_no_change_random_offset_seed20260708.xml` |
| `medium_core` | Medium periodic demand on the core route subset. | `CORE` | period=80s | 34000-44000 |  | `data/medium_core_no_change_random_offset_seed20260708.rou.xml` | `data/medium_core_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_medium_core_no_change_random_offset_seed20260708.xml` |
| `heavy_all` | Heavy periodic demand across all expanded routes. | `ALL` | period=50s | 46000-56000 |  | `data/heavy_all_no_change_random_offset_seed20260708.rou.xml` | `data/heavy_all_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_heavy_all_no_change_random_offset_seed20260708.xml` |
| `oversat_all` | Oversaturated periodic demand across all expanded routes. | `ALL` | period=30s | 58000-68000 |  | `data/oversat_all_no_change_random_offset_seed20260708.rou.xml` | `data/oversat_all_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_oversat_all_no_change_random_offset_seed20260708.xml` |
| `west_bottleneck` | Bottleneck demand focused on west inbound routes. | `WEST_IN` | period=30s | 70000-80000 |  | `data/west_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/west_bottleneck_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_west_bottleneck_no_change_random_offset_seed20260708.xml` |
| `north_bottleneck` | Bottleneck demand focused on north inbound routes. | `NORTH_IN` | period=30s | 82000-92000 |  | `data/north_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/north_bottleneck_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_north_bottleneck_no_change_random_offset_seed20260708.xml` |
| `east_bottleneck` | Bottleneck demand focused on east inbound routes. | `EAST_IN` | period=30s | 94000-104000 |  | `data/east_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/east_bottleneck_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_east_bottleneck_no_change_random_offset_seed20260708.xml` |
| `south_bottleneck` | Bottleneck demand focused on south inbound routes. | `SOUTH_IN` | period=30s | 106000-116000 |  | `data/south_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/south_bottleneck_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_south_bottleneck_no_change_random_offset_seed20260708.xml` |
| `southwest_bottleneck` | Bottleneck demand focused on southwest inbound routes. | `SOUTHWEST_IN` | period=30s | 118000-128000 |  | `data/southwest_bottleneck_no_change_random_offset_seed20260708.rou.xml` | `data/southwest_bottleneck_no_change_random_offset_seed20260708.sumocfg` | `data/tripinfo_southwest_bottleneck_no_change_random_offset_seed20260708.xml` |

## Committed sumocfg files

- `data/free_all_no_change_random_offset_seed20260708.sumocfg`
- `data/light_all_seed1_no_change_random_offset_seed20260708.sumocfg`
- `data/light_all_seed2_no_change_random_offset_seed20260708.sumocfg`
- `data/medium_core_no_change_random_offset_seed20260708.sumocfg`
- `data/heavy_all_no_change_random_offset_seed20260708.sumocfg`
- `data/oversat_all_no_change_random_offset_seed20260708.sumocfg`
- `data/west_bottleneck_no_change_random_offset_seed20260708.sumocfg`
- `data/north_bottleneck_no_change_random_offset_seed20260708.sumocfg`
- `data/east_bottleneck_no_change_random_offset_seed20260708.sumocfg`
- `data/south_bottleneck_no_change_random_offset_seed20260708.sumocfg`
- `data/southwest_bottleneck_no_change_random_offset_seed20260708.sumocfg`

## Commands for future route generation

Generate all group route files locally:

```bash
python3 generate_route_on_test_net_full.py \
  --all-groups \
  --output-dir data \
  --prefix no_change_random_offset_seed20260708
```

Generate one selected group locally:

```bash
python3 generate_route_on_test_net_full.py \
  --group oversat_all \
  --output data/oversat_all_no_change_random_offset_seed20260708.rou.xml
```

Generate selected groups locally:

```bash
python3 generate_route_on_test_net_full.py \
  --groups west_bottleneck,southwest_bottleneck,north_bottleneck \
  --output-dir data \
  --prefix no_change_random_offset_seed20260708
```

List groups:

```bash
python3 generate_route_on_test_net_full.py --list-groups
```

Regenerate committed sumocfg files only:

```bash
python3 generate_route_on_test_net_full.py \
  --write-sumocfg-only \
  --all-groups \
  --sumocfg-output-dir data \
  --prefix no_change_random_offset_seed20260708 \
  --net-file data/test_random_offsets_seed20260708.net.xml \
  --tripinfo-output-dir data
```

Generated `.rou.xml` files from the route-generation commands above can be used locally for experiments, but they should not be committed if they are too large for review.

## Latest random-offset network

The latest random-offset network is `data/test_random_offsets_seed20260708.net.xml`. All committed `.sumocfg` files reference it as `test_random_offsets_seed20260708.net.xml` because the `.sumocfg` files live in `data/`.

## Tripinfo note

No tripinfo output was generated for this PR. After generating a per-group route file locally, run SUMO with the matching `.sumocfg` to produce the matching `tripinfo_<group>_no_change_random_offset_seed20260708.xml` output. Existing full-route tripinfo should not be reused for per-group route files.

## PR size and safety note

Generated route XML files are intentionally not committed to avoid exceeding PR diff limits. Users can generate them locally with the documented commands.

## Validation

Validation performed without running SUMO, generating tripinfo, or running CAMS evaluation:

- `python3 generate_route_on_test_net_full.py --list-groups` works.
- `python3 -m py_compile generate_route_on_test_net_full.py` passes.
- Sumocfg XML parsing passes for all committed per-group sumocfg files.
- Sumocfg references are correct for the latest random-offset network, future per-group route files, and future per-group tripinfo outputs.
- No generated per-group `.rou.xml` files are committed.
