#!/usr/bin/env bash
# Server collector for the 100 materialized new4x_sf episodes. No SF resampling.
# Usage: bash scripts/collection/run_new4x_sf_traci.sh <TAG> [run_all_training_traci.sh options]
# The six candidate labels always mean speed > cutoff. The legacy stop/low
# settings below do not change those definitions.
set -Eeuo pipefail

die() { echo "[ERROR] $*" >&2; exit 1; }
usage() {
    printf '%s\n' \
        'Usage: bash scripts/collection/run_new4x_sf_traci.sh <TAG> [runner options]' \
        'Example: bash scripts/collection/run_new4x_sf_traci.sh sf_threshold_sweep --jobs 8 --dry-run' \
        'Then remove --dry-run to collect. Outputs: new4x_sf/traci_runs/<TAG>/.' \
        'Defaults: legacy stop=1, low=3, queue=2 m/s; six labels=0.1,0.5,1,1.5,2,3.' \
        'Forwarded: --jobs, --start-index, --resume/--no-resume, thresholds, --dry-run, etc.' \
        'Reserved: --run-tag and --output-root (use the mandatory TAG).' \
        'Optional environment: DATASET_DIR (materialized dataset), SF_REAL_SUMO_BINARY, SUMO_HOME.'
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DRIVER="$REPO_ROOT/scripts/collection/run_all_training_traci.sh"
[[ "$#" -ge 1 ]] || { usage >&2; exit 1; }
if [[ "$1" == "--help" || "$1" == "-h" ]]; then usage; exit 0; fi
TAG="$1"; shift
[[ "$TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "invalid TAG: use letters, digits, dot, underscore or hyphen"
for argument in "$@"; do
    case "$argument" in
        --output-root|--output-root=*|--run-tag|--run-tag=*)
            die "output path is fixed by TAG; do not forward $argument" ;;
    esac
done

DATASET_DIR="$(realpath -m "${DATASET_DIR:-$REPO_ROOT/train_data/variants/new4x_sf}")"
NET_FILE="$DATASET_DIR/inputs/dfbf38f23154_test_new.net.xml"
TRACI_SCRIPT="$REPO_ROOT/collection/_shared/TraCI_Python_Threshold_Sweep.py"
TRACI_MODULE="collection._shared.TraCI_Python_Threshold_Sweep"
JOB_FILE="$DATASET_DIR/commands/traci_jobs.tsv"
MATERIALIZATION_MARKER="$DATASET_DIR/manifests/materialization_run.json"
[[ -f "$DRIVER" ]] || die "runner not found: $DRIVER"
[[ -f "$TRACI_SCRIPT" ]] || die "threshold collector not found: $TRACI_SCRIPT"
[[ -f "$REPO_ROOT/collection/_shared/TraCI_Python_Adjusted.py" ]] || die "base collector not found"
[[ -f "$NET_FILE" ]] || die "materialized new4x network not found: $NET_FILE"
[[ -f "$JOB_FILE" ]] || die "completed 100-episode SF manifest not found: $JOB_FILE"
[[ -f "$MATERIALIZATION_MARKER" ]] || die "not an SF-materialized dataset: $DATASET_DIR"
grep -Eq '"schema"[[:space:]]*:[[:space:]]*"explicit-speed-factor-v1"' "$MATERIALIZATION_MARKER" \
    || die "unsupported materialization schema: $MATERIALIZATION_MARKER"

# Fail before the scheduler creates output directories if the manifest points
# outside the explicit-SF tree, or if only part of the dataset was transferred.
while IFS=$'\t' read -r episode_id split sumocfg remaining; do
    [[ -n "$episode_id" ]] || continue
    [[ "$episode_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "unsafe episode ID: $episode_id"
    [[ "$sumocfg" == "sumocfg/$episode_id.sumocfg" ]] || die "unexpected config path for $episode_id: $sumocfg"
    config_path="$DATASET_DIR/$sumocfg"
    [[ -f "$config_path" ]] || die "missing explicit config: $config_path"
    [[ -f "$DATASET_DIR/routes/$episode_id.rou.xml" ]] || die "missing explicit route for $episode_id"
    grep -Fq 'value="../inputs/dfbf38f23154_test_new.net.xml"' "$config_path" \
        || die "config uses an unexpected network: $config_path"
    grep -Fq "value=\"../routes/$episode_id.rou.xml\"" "$config_path" \
        || die "config does not use its materialized route: $config_path"
done < <(tail -n +2 "$JOB_FILE")

# The existing scheduler overrides tripinfo, but not statistic-output stored
# in SUMO configs. The extra static wrapper keeps both inside this run.
SF_REAL_SUMO_BINARY="${SF_REAL_SUMO_BINARY:-${SUMO_BINARY:-sumo}}"
SUMO_BINARY="$REPO_ROOT/scripts/collection/sumo_sf_collection.sh"
[[ -x "$SUMO_BINARY" ]] || die "SUMO wrapper is not executable; run chmod +x scripts/collection/sumo_sf_collection.sh"
[[ "$SF_REAL_SUMO_BINARY" != "$SUMO_BINARY" ]] || die "SF_REAL_SUMO_BINARY must be the actual SUMO executable"
export SF_REAL_SUMO_BINARY SUMO_BINARY DATASET_DIR NET_FILE TRACI_SCRIPT TRACI_MODULE
export EXPECTED_EPISODES=100
export TRACI_OUTPUTS=legacy,training
export LABEL_SPEED_THRESHOLDS=0.1,0.5,1.0,1.5,2.0,3.0
export STOP_SPEED_THRESHOLD="${STOP_SPEED_THRESHOLD:-1.0}"
export LOW_SPEED_THRESHOLD="${LOW_SPEED_THRESHOLD:-3.0}"
export QUEUE_SPEED_THRESHOLD="${QUEUE_SPEED_THRESHOLD:-2.0}"
# Prevent unrelated shell defaults from redirecting output or conflicting with
# --output-root. Remaining runner options are passed through unchanged.
export RUN_TAG=""
export OUTPUT_ROOT="$DATASET_DIR/traci_runs/$TAG"

printf '[new4x_sf] dataset=%s\n[new4x_sf] output=%s\n' "$DATASET_DIR" "$OUTPUT_ROOT"
printf '[new4x_sf] actual SUMO=%s\n' "$SF_REAL_SUMO_BINARY"
printf '[new4x_sf] six cutoffs=%s m/s; legacy stop/low do not change these labels\n' "$LABEL_SPEED_THRESHOLDS"
exec bash "$DRIVER" --output-root "$OUTPUT_ROOT" "$@"
