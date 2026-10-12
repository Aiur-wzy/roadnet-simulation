#!/usr/bin/env bash
# ABC collector over the unchanged 100 explicit-SF episodes. Never resumes.
# Old context0 -> A/B/C=0/4/5; old context1/2/3 retain their original meaning.
set -Eeuo pipefail

die() { echo "[ERROR] $*" >&2; exit 1; }
usage() {
    printf '%s\n' \
        'Usage: bash scripts/collection/run_new4x_cap_context_abc_traci.sh <NEW_TAG> [runner options]' \
        'Example: bash scripts/collection/run_new4x_cap_context_abc_traci.sh cap_context_abc_v1 --jobs 8 --dry-run' \
        'Then remove --dry-run to collect into new4x_sf/traci_runs/<NEW_TAG>/.' \
        'Fresh directories only; --resume and existing output paths are rejected.' \
        'On interruption choose another NEW_TAG; --start-index selects a suffix only.' \
        '100 unique episodes retain the original train80/validation20 manifest split.' \
        'ABC: old0 -> A=0/B=4/C=5; old1/2/3 unchanged. No numeric ordering implied.' \
        'Fixed collector defaults: overdue threshold=0 s, epsilon=1e-9 s, crossing=3 s.' \
        'C requires max(0,decision_time-free_finish_time) > threshold+epsilon.' \
        'Legacy stop/low/queue=1/3/2 m/s; all six fixed label cutoffs in ONE replay.' \
        'Each feature CSV is accompanied by .contract.json and .audit.csv.' \
        'Forwarded: --jobs/-j, --start/--start-index, --poll-interval, --no-resume, --dry-run.' \
        'Reserved: output paths, resume, context and legacy speed-threshold overrides.' \
        'Sync ABC collector + abc features + shared Cap_Context + cap features +' \
        'Adjusted + Threshold_Sweep + turn_context, plus this script and both driver wrappers.' \
        'Optional environment: DATASET_DIR, SF_REAL_SUMO_BINARY, SUMO_HOME, DRY_RUN.'
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
            die "ABC collection cannot resume; choose a NEW_TAG instead of accepting incomplete or old-schema output" ;;
        --context-crossing-time|--context-crossing-time=*|--context0-overdue-threshold|--context0-overdue-threshold=*)
            die "ABC batch freezes collector defaults: crossing=3 s and context0 overdue threshold=0 s" ;;
        --stop-speed-threshold|--stop-speed-threshold=*|--low-speed-threshold|--low-speed-threshold=*|--queue-speed-threshold|--queue-speed-threshold=*)
            die "ABC batch preserves legacy stop/low/queue=1/3/2 m/s; speed-threshold overrides are not accepted" ;;
        --dry-run) REQUESTED_DRY_RUN=1 ;;
        --help|-h) usage; exit 0 ;;
    esac
done
[[ "$REQUESTED_DRY_RUN" == 0 || "$REQUESTED_DRY_RUN" == 1 ]] || die "DRY_RUN must be 0 or 1"

DATASET_DIR="$(realpath -m "${DATASET_DIR:-$REPO_ROOT/train_data/variants/new4x_sf}")"
NET_FILE="$DATASET_DIR/inputs/dfbf38f23154_test_new.net.xml"
TRACI_SCRIPT="$REPO_ROOT/collection/_shared/TraCI_Python_Cap_Context_ABC.py"
TRACI_MODULE="collection._shared.TraCI_Python_Cap_Context_ABC"
JOB_FILE="$DATASET_DIR/commands/traci_jobs.tsv"
MATERIALIZATION_MARKER="$DATASET_DIR/manifests/materialization_run.json"
[[ -f "$DRIVER" ]] || die "runner not found: $DRIVER"
for dependency in \
    collection/_shared/TraCI_Python_Cap_Context_ABC.py features/cap_context_abc_features.py \
    collection/_shared/TraCI_Python_Cap_Context.py features/cap_context_features.py \
    collection/_shared/TraCI_Python_Adjusted.py collection/_shared/TraCI_Python_Threshold_Sweep.py features/turn_context.py; do
    [[ -f "$REPO_ROOT/$dependency" ]] || die "collector dependency missing: $dependency; sync the complete ABC collector bundle"
done
[[ -f "$NET_FILE" ]] || die "materialized new4x network not found: $NET_FILE"
[[ -f "$JOB_FILE" ]] || die "100-episode SF manifest not found: $JOB_FILE"
[[ -f "$MATERIALIZATION_MARKER" ]] || die "not an SF-materialized dataset: $DATASET_DIR"
grep -Eq '"schema"[[:space:]]*:[[:space:]]*"explicit-speed-factor-v1"' "$MATERIALIZATION_MARKER" \
    || die "unsupported materialization schema: $MATERIALIZATION_MARKER"

declare -A SEEN_EPISODES=()
episode_count=0
train_count=0
validation_count=0
while IFS=$'\t' read -r episode_id split sumocfg legacy full events cycles log_file; do
    [[ -n "$episode_id" ]] || continue
    log_file="${log_file%$'\r'}"
    [[ "$episode_id" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || die "unsafe episode ID: $episode_id"
    [[ -z "${SEEN_EPISODES[$episode_id]+x}" ]] || die "duplicate episode ID: $episode_id"
    SEEN_EPISODES["$episode_id"]=1
    case "$split" in
        train) train_count=$((train_count + 1)) ;;
        validation) validation_count=$((validation_count + 1)) ;;
        *) die "unknown split for $episode_id: $split" ;;
    esac
    episode_count=$((episode_count + 1))
    [[ "$sumocfg" == "sumocfg/$episode_id.sumocfg" ]] || die "unexpected config path for $episode_id: $sumocfg"
    [[ "${legacy##*/}" == "TraCI_output_adjusted_$episode_id.csv" ]] || die "unexpected legacy basename for $episode_id"
    [[ "${full##*/}" == "cams_model_training_data_$episode_id.csv" ]] || die "unexpected feature basename for $episode_id"
    [[ "${events##*/}" == "cams_vehicle_events_$episode_id.csv" ]] || die "unexpected events basename for $episode_id"
    [[ "${cycles##*/}" == "cams_cycle_summary_$episode_id.csv" ]] || die "unexpected cycles basename for $episode_id"
    [[ "${log_file##*/}" == "traci_$episode_id.log" ]] || die "unexpected log basename for $episode_id"
    config_path="$DATASET_DIR/$sumocfg"
    [[ -f "$config_path" ]] || die "missing explicit config: $config_path"
    [[ -f "$DATASET_DIR/routes/$episode_id.rou.xml" ]] || die "missing explicit route for $episode_id"
    grep -Fq 'value="../inputs/dfbf38f23154_test_new.net.xml"' "$config_path" \
        || die "config uses an unexpected network: $config_path"
    grep -Fq "value=\"../routes/$episode_id.rou.xml\"" "$config_path" \
        || die "config does not use its materialized route: $config_path"
done < <(tail -n +2 "$JOB_FILE")
[[ "$episode_count" == 100 && "$train_count" == 80 && "$validation_count" == 20 ]] \
    || die "expected 100 unique episodes with train80/validation20; found $episode_count / $train_count / $validation_count"

SF_REAL_SUMO_BINARY="${SF_REAL_SUMO_BINARY:-${SUMO_BINARY:-sumo}}"
SUMO_BINARY="$REPO_ROOT/scripts/collection/sumo_sf_collection.sh"
[[ -x "$SUMO_BINARY" ]] || die "SUMO wrapper is not executable; run chmod +x scripts/collection/sumo_sf_collection.sh"
[[ "$SF_REAL_SUMO_BINARY" != "$SUMO_BINARY" ]] || die "SF_REAL_SUMO_BINARY must name the actual SUMO executable"
export SF_REAL_SUMO_BINARY SUMO_BINARY DATASET_DIR NET_FILE TRACI_SCRIPT TRACI_MODULE
export EXPECTED_EPISODES=100
export TRACI_OUTPUTS=legacy,training
export LABEL_SPEED_THRESHOLDS=0.1,0.5,1.0,1.5,2.0,3.0
export STOP_SPEED_THRESHOLD=1.0
export LOW_SPEED_THRESHOLD=3.0
export QUEUE_SPEED_THRESHOLD=2.0
export RUN_TAG=""
export RESUME=0
export OUTPUT_ROOT="$(realpath -m "$DATASET_DIR/traci_runs/$TAG")"
[[ "$OUTPUT_ROOT" == "$DATASET_DIR/traci_runs/"* ]] || die "output resolved outside the dataset traci_runs tree"
[[ ! -e "$OUTPUT_ROOT" && ! -L "$OUTPUT_ROOT" ]] \
    || die "NEW_TAG output already exists; no overwrite/resume is permitted: $OUTPUT_ROOT"

printf '[cap_context_abc_v1] dataset=%s\n[cap_context_abc_v1] output=%s\n' "$DATASET_DIR" "$OUTPUT_ROOT"
printf '[cap_context_abc_v1] collector=%s\n[cap_context_abc_v1] actual SUMO=%s\n' "$TRACI_SCRIPT" "$SF_REAL_SUMO_BINARY"
printf '[cap_context_abc_v1] episodes=%s train=%s validation=%s\n' "$episode_count" "$train_count" "$validation_count"
printf '[cap_context_abc_v1] A/B/C=0/4/5; old1/2/3 retained; overdue=0 s; epsilon=1e-9 s; crossing=3 s\n'
printf '[cap_context_abc_v1] six label cutoffs=%s; resume=OFF\n' "$LABEL_SPEED_THRESHOLDS"
# The unchanged driver has no arbitrary collector-options pass-through. The ABC
# entry point defines overdue=0 and crossing=3; this wrapper rejects overrides.
# Atomically reserve a fresh directory for real collection only. On preflight
# failure/interruption pick another tag; never reuse a partially created run.
if [[ "$REQUESTED_DRY_RUN" == 0 ]]; then
    mkdir -p "$DATASET_DIR/traci_runs"
    mkdir "$OUTPUT_ROOT" || die "could not exclusively reserve NEW_TAG: $OUTPUT_ROOT"
fi
exec bash "$DRIVER" --output-root "$OUTPUT_ROOT" "$@" --no-resume
