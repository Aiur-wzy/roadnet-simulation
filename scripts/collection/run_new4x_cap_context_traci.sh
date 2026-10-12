#!/usr/bin/env bash
# New cap/context collection over the same 100 explicit-SF episodes.
# Existing SF wrapper/data are untouched. V1 deliberately does NOT resume.
set -Eeuo pipefail

die() { echo "[ERROR] $*" >&2; exit 1; }
usage() {
    printf '%s\n' \
        'Usage: bash scripts/collection/run_new4x_cap_context_traci.sh <NEW_TAG> [runner options]' \
        'Example: bash scripts/collection/run_new4x_cap_context_traci.sh cap_context_v1 --jobs 8 --dry-run' \
        'Then remove --dry-run to collect into new4x_sf/traci_runs/<NEW_TAG>/.' \
        'Existing files/directories and --resume are rejected, including old SF runs.' \
        'On interruption choose a NEW tag; optional --start-index selects a suffix only.' \
        'Default crossing=3 s; legacy stop/low/queue=1/3/2 m/s; six fixed speed cutoffs.' \
        'Formal features are cap/context raw5; every CSV has .contract.json and .audit.csv.' \
        'Forwarded: --jobs, --start-index, --no-resume, legacy thresholds, --dry-run.' \
        'Reserved: --output-root, --run-tag, --resume and --context-crossing-time.' \
        'Optional environment: DATASET_DIR, SF_REAL_SUMO_BINARY, SUMO_HOME.'
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DRIVER="$REPO_ROOT/scripts/collection/run_all_training_traci.sh"
[[ "$#" -ge 1 ]] || { usage >&2; exit 1; }
if [[ "$1" == "--help" || "$1" == "-h" ]]; then usage; exit 0; fi
TAG="$1"; shift
[[ "$TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "invalid TAG: use letters, digits, dot, underscore or hyphen"
REQUESTED_DRY_RUN="${DRY_RUN:-0}"
for argument in "$@"; do
    case "$argument" in
        --output-root|--output-root=*|--run-tag|--run-tag=*)
            die "output path is fixed by NEW_TAG; do not forward $argument" ;;
        --resume|--resume=*)
            die "cap/context v1 cannot resume; use a NEW_TAG to avoid accepting incomplete or old-schema outputs" ;;
        --context-crossing-time|--context-crossing-time=*)
            die "this batch wrapper freezes context crossing at 3 s; custom collection requires a separately verified command" ;;
        --dry-run) REQUESTED_DRY_RUN=1 ;;
        --help|-h) usage; exit 0 ;;
    esac
done
[[ "$REQUESTED_DRY_RUN" == 0 || "$REQUESTED_DRY_RUN" == 1 ]] || die "DRY_RUN must be 0 or 1"

DATASET_DIR="$(realpath -m "${DATASET_DIR:-$REPO_ROOT/train_data/variants/new4x_sf}")"
NET_FILE="$DATASET_DIR/inputs/dfbf38f23154_test_new.net.xml"
TRACI_SCRIPT="$REPO_ROOT/collection/_shared/TraCI_Python_Cap_Context.py"
TRACI_MODULE="collection._shared.TraCI_Python_Cap_Context"
JOB_FILE="$DATASET_DIR/commands/traci_jobs.tsv"
MATERIALIZATION_MARKER="$DATASET_DIR/manifests/materialization_run.json"
[[ -f "$DRIVER" ]] || die "runner not found: $DRIVER"
[[ -f "$TRACI_SCRIPT" ]] || die "cap/context collector not found: $TRACI_SCRIPT"
[[ -f "$REPO_ROOT/features/cap_context_features.py" ]] || die "cap/context feature contract module not found"
[[ -f "$REPO_ROOT/features/turn_context.py" ]] || die "T04 shared turn metadata module not found; sync it with the collector"
[[ -f "$REPO_ROOT/collection/_shared/TraCI_Python_Adjusted.py" ]] || die "base collector not found"
[[ -f "$REPO_ROOT/collection/_shared/TraCI_Python_Threshold_Sweep.py" ]] || die "threshold collector not found"
[[ -f "$NET_FILE" ]] || die "materialized new4x network not found: $NET_FILE"
[[ -f "$JOB_FILE" ]] || die "completed 100-episode SF manifest not found: $JOB_FILE"
[[ -f "$MATERIALIZATION_MARKER" ]] || die "not an SF-materialized dataset: $DATASET_DIR"
grep -Eq '"schema"[[:space:]]*:[[:space:]]*"explicit-speed-factor-v1"' "$MATERIALIZATION_MARKER" \
    || die "unsupported materialization schema: $MATERIALIZATION_MARKER"

while IFS=$'\t' read -r episode_id split sumocfg remaining; do
    [[ -n "$episode_id" ]] || continue
    [[ "$episode_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "unsafe episode ID: $episode_id"
    [[ "$split" == train || "$split" == validation ]] || die "unknown split for $episode_id: $split"
    [[ "$sumocfg" == "sumocfg/$episode_id.sumocfg" ]] || die "unexpected config path for $episode_id: $sumocfg"
    config_path="$DATASET_DIR/$sumocfg"
    [[ -f "$config_path" ]] || die "missing explicit config: $config_path"
    [[ -f "$DATASET_DIR/routes/$episode_id.rou.xml" ]] || die "missing explicit route for $episode_id"
    grep -Fq 'value="../inputs/dfbf38f23154_test_new.net.xml"' "$config_path" \
        || die "config uses an unexpected network: $config_path"
    grep -Fq "value=\"../routes/$episode_id.rou.xml\"" "$config_path" \
        || die "config does not use its materialized route: $config_path"
done < <(tail -n +2 "$JOB_FILE")

SF_REAL_SUMO_BINARY="${SF_REAL_SUMO_BINARY:-${SUMO_BINARY:-sumo}}"
SUMO_BINARY="$REPO_ROOT/scripts/collection/sumo_sf_collection.sh"
[[ -x "$SUMO_BINARY" ]] || die "SUMO wrapper is not executable; run chmod +x scripts/collection/sumo_sf_collection.sh"
[[ "$SF_REAL_SUMO_BINARY" != "$SUMO_BINARY" ]] || die "SF_REAL_SUMO_BINARY must name the actual SUMO executable"
export SF_REAL_SUMO_BINARY SUMO_BINARY DATASET_DIR NET_FILE TRACI_SCRIPT TRACI_MODULE
export EXPECTED_EPISODES=100
export TRACI_OUTPUTS=legacy,training
export LABEL_SPEED_THRESHOLDS=0.1,0.5,1.0,1.5,2.0,3.0
export STOP_SPEED_THRESHOLD="${STOP_SPEED_THRESHOLD:-1.0}"
export LOW_SPEED_THRESHOLD="${LOW_SPEED_THRESHOLD:-3.0}"
export QUEUE_SPEED_THRESHOLD="${QUEUE_SPEED_THRESHOLD:-2.0}"
export RUN_TAG=""
export RESUME=0
export OUTPUT_ROOT="$(realpath -m "$DATASET_DIR/traci_runs/$TAG")"
[[ "$OUTPUT_ROOT" == "$DATASET_DIR/traci_runs/"* ]] || die "output resolved outside the dataset traci_runs tree"
[[ ! -e "$OUTPUT_ROOT" && ! -L "$OUTPUT_ROOT" ]] \
    || die "NEW_TAG output already exists; no overwrite/resume is permitted: $OUTPUT_ROOT"

printf '[cap_context_v1] dataset=%s\n[cap_context_v1] output=%s\n' "$DATASET_DIR" "$OUTPUT_ROOT"
printf '[cap_context_v1] collector=%s\n[cap_context_v1] actual SUMO=%s\n' "$TRACI_SCRIPT" "$SF_REAL_SUMO_BINARY"
printf '[cap_context_v1] crossing=3 s; six cutoffs=%s; resume=OFF\n' "$LABEL_SPEED_THRESHOLDS"
# Atomically reserve a fresh directory for real collection, so simultaneous
# invocations cannot both pass the old-output check and overwrite each other.
# No directory is created for a dry-run. A failed preflight may leave this
# empty reserved directory; pick another tag rather than reusing it silently.
if [[ "$REQUESTED_DRY_RUN" == 0 ]]; then
    mkdir -p "$DATASET_DIR/traci_runs"
    mkdir "$OUTPUT_ROOT" || die "could not exclusively reserve NEW_TAG: $OUTPUT_ROOT"
fi
exec bash "$DRIVER" --output-root "$OUTPUT_ROOT" "$@" --no-resume
