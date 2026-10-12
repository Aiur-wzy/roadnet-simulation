#!/usr/bin/env bash
set -Eeuo pipefail

# Run TraCI episodes concurrently with a fixed-size worker pool.
# Whenever one episode finishes, the next queued episode is started automatically.
#
# Expected input layout (all paths may be overridden with environment variables):
#   current-directory/
#   ├── data/test_new.net.xml
#   ├── TraCI_Python_Adjusted.py
#   ├── run_all_training_traci.sh
#   └── train_data/
#       ├── commands/traci_jobs.tsv
#       ├── sumocfg/*.sumocfg
#       └── variants/<RUN_TAG>/        # created by this script
#
# A run tag or an explicit output root is required so a new strategy never
# writes into the existing train_data/outputs tree.
#
# Basic usage (parallelism defaults to the number of available CPU cores):
#   bash scripts/collection/run_all_training_traci.sh \
#     --run-tag low3p0 \
#     --low-speed-threshold 3.0
#
# Run at most 8 episodes at the same time:
#   bash scripts/collection/run_all_training_traci.sh --run-tag low3p0 --jobs 8
#   RUN_TAG=low3p0 MAX_PARALLEL=8 bash scripts/collection/run_all_training_traci.sh
#
# Resume from an explicit 1-based episode number (inclusive):
#   bash scripts/collection/run_all_training_traci.sh \
#     --run-tag low3p0 \
#     --start-index 37 \
#     --jobs 8
#
# Completed episodes are skipped by default when their status and all requested
# outputs are complete. Use RESUME=0 to rerun them.

die() {
    echo "[ERROR] $*" >&2
    exit 1
}

detect_cpu_count() {
    local count=""
    if command -v nproc >/dev/null 2>&1; then
        count="$(nproc)"
    elif command -v getconf >/dev/null 2>&1; then
        count="$(getconf _NPROCESSORS_ONLN 2>/dev/null || true)"
    fi
    if [[ ! "$count" =~ ^[1-9][0-9]*$ ]]; then
        count=1
    fi
    printf '%s\n' "$count"
}

usage() {
    cat <<'EOF'
Usage:
  bash scripts/collection/run_all_training_traci.sh [START_INDEX]
  bash scripts/collection/run_all_training_traci.sh [options]

Options:
  -j, --jobs N             Maximum concurrent TraCI episodes.
                           Default: number of available CPU cores.
  --start, --start-index N Start from 1-based manifest row N (inclusive).
  --run-tag TAG            Store this strategy under
                           train_data/variants/TAG/.
  --output-root DIR        Store this strategy under an explicit directory.
                           Use either --run-tag or --output-root, not both.
  --stop-speed-threshold MPS
                           Forward the stopped/low-speed boundary to TraCI.
                           Default: 0.1.
  --low-speed-threshold MPS
                           Forward the low-speed/normal-speed boundary to TraCI.
                           Default: 2.0.
  --queue-speed-threshold MPS
                           Forward the independent queue-speed boundary to TraCI.
                           Default: 2.0.
  --poll-interval SECONDS  Seconds between worker-pool checks. Default: 1.
  --resume                 Skip completed episodes (default).
  --no-resume              Rerun completed episodes in the selected range.
  --dry-run                Print queued commands without running them.
  -h, --help               Show this help.

Environment equivalents:
  MAX_PARALLEL, START_INDEX, POLL_INTERVAL, RESUME, DRY_RUN,
  DATASET_DIR, TRACI_SCRIPT, TRACI_MODULE, SUMO_BINARY, TRACI_OUTPUTS, NET_FILE,
  EXPECTED_EPISODES, RUN_TAG, OUTPUT_ROOT, STOP_SPEED_THRESHOLD,
  LOW_SPEED_THRESHOLD, QUEUE_SPEED_THRESHOLD

Examples:
  bash scripts/collection/run_all_training_traci.sh \
    --run-tag low3p0 \
    --low-speed-threshold 3.0 \
    --jobs 8

  bash scripts/collection/run_all_training_traci.sh \
    --run-tag low3p0 \
    --low-speed-threshold 3.0 \
    --start-index 37 \
    --jobs 12

  RUN_TAG=low3p0 LOW_SPEED_THRESHOLD=3.0 MAX_PARALLEL=8 \
    bash scripts/collection/run_all_training_traci.sh

  DRY_RUN=1 bash scripts/collection/run_all_training_traci.sh \
    --run-tag low3p0 \
    --low-speed-threshold 3.0
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
DATASET_DIR="${DATASET_DIR:-$REPO_ROOT/train_data}"
TRACI_MODULE="${TRACI_MODULE:-}"
if [[ -z "${TRACI_SCRIPT:-}" ]]; then
    TRACI_MODULE="${TRACI_MODULE:-collection._shared.TraCI_Python_Adjusted}"
    TRACI_SCRIPT="$REPO_ROOT/${TRACI_MODULE//.//}.py"
fi
SUMO_BINARY="${SUMO_BINARY:-sumo}"
TRACI_OUTPUTS="${TRACI_OUTPUTS:-legacy}"
NET_FILE="${NET_FILE:-$REPO_ROOT/data/test_new.net.xml}"
EXPECTED_EPISODES="${EXPECTED_EPISODES:-80}"
RESUME="${RESUME:-1}"
DRY_RUN="${DRY_RUN:-0}"
START_INDEX="${START_INDEX:-1}"
MAX_PARALLEL="${MAX_PARALLEL:-$(detect_cpu_count)}"
POLL_INTERVAL="${POLL_INTERVAL:-1}"
RUN_TAG="${RUN_TAG:-}"
OUTPUT_ROOT="${OUTPUT_ROOT:-}"
STOP_SPEED_THRESHOLD="${STOP_SPEED_THRESHOLD:-0.1}"
LOW_SPEED_THRESHOLD="${LOW_SPEED_THRESHOLD:-2.0}"
QUEUE_SPEED_THRESHOLD="${QUEUE_SPEED_THRESHOLD:-2.0}"

positional_start_seen=0
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -j|--jobs)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            MAX_PARALLEL="$2"
            shift 2
            ;;
        --start|--start-index)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            START_INDEX="$2"
            positional_start_seen=1
            shift 2
            ;;
        --run-tag)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            RUN_TAG="$2"
            shift 2
            ;;
        --output-root)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            OUTPUT_ROOT="$2"
            shift 2
            ;;
        --stop-speed-threshold)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            STOP_SPEED_THRESHOLD="$2"
            shift 2
            ;;
        --low-speed-threshold)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            LOW_SPEED_THRESHOLD="$2"
            shift 2
            ;;
        --queue-speed-threshold)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            QUEUE_SPEED_THRESHOLD="$2"
            shift 2
            ;;
        --poll-interval)
            [[ "$#" -ge 2 ]] || die "$1 requires a value"
            POLL_INTERVAL="$2"
            shift 2
            ;;
        --resume)
            RESUME=1
            shift
            ;;
        --no-resume)
            RESUME=0
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            usage >&2
            die "unknown option: $1"
            ;;
        *)
            [[ "$positional_start_seen" -eq 0 ]] || die "START_INDEX was provided more than once"
            START_INDEX="$1"
            positional_start_seen=1
            shift
            ;;
    esac
done
[[ "$#" -eq 0 ]] || die "unexpected positional arguments: $*"

[[ "$RESUME" == "0" || "$RESUME" == "1" ]] || die "RESUME must be 0 or 1"
[[ "$DRY_RUN" == "0" || "$DRY_RUN" == "1" ]] || die "DRY_RUN must be 0 or 1"
[[ "$EXPECTED_EPISODES" =~ ^[0-9]+$ ]] || die "EXPECTED_EPISODES must be a non-negative integer"
[[ "$START_INDEX" =~ ^[1-9][0-9]*$ ]] || die "START_INDEX must be a positive integer"
[[ "$MAX_PARALLEL" =~ ^[1-9][0-9]*$ ]] || die "MAX_PARALLEL/--jobs must be a positive integer"
[[ "$POLL_INTERVAL" =~ ^([0-9]+([.][0-9]+)?|[.][0-9]+)$ ]] \
    || die "POLL_INTERVAL must be a positive number"
awk -v value="$POLL_INTERVAL" 'BEGIN { exit !(value > 0) }' \
    || die "POLL_INTERVAL must be greater than zero"
[[ -n "$STOP_SPEED_THRESHOLD" ]] || die "STOP_SPEED_THRESHOLD must not be empty"
[[ -n "$LOW_SPEED_THRESHOLD" ]] || die "LOW_SPEED_THRESHOLD must not be empty"
[[ -n "$QUEUE_SPEED_THRESHOLD" ]] || die "QUEUE_SPEED_THRESHOLD must not be empty"
[[ -n "$RUN_TAG" || -n "$OUTPUT_ROOT" ]] \
    || die "set --run-tag TAG or --output-root DIR to keep this run separate"
[[ -z "$RUN_TAG" || -z "$OUTPUT_ROOT" ]] \
    || die "use either --run-tag or --output-root, not both"
if [[ -n "$RUN_TAG" ]]; then
    [[ "$RUN_TAG" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] \
        || die "RUN_TAG may contain only letters, numbers, dot, underscore, and hyphen"
fi

DATASET_DIR="$(realpath -m "$DATASET_DIR")"
TRACI_SCRIPT="$(realpath -m "$TRACI_SCRIPT")"
TRACI_COMMAND=(python3)
if [[ -n "$TRACI_MODULE" ]]; then
    TRACI_COMMAND+=(-m "$TRACI_MODULE")
else
    TRACI_COMMAND+=("$TRACI_SCRIPT")
fi
NET_FILE="$(realpath -m "$NET_FILE")"
if [[ -n "$RUN_TAG" ]]; then
    OUTPUT_ROOT="$DATASET_DIR/variants/$RUN_TAG"
else
    OUTPUT_ROOT="$(realpath -m "$OUTPUT_ROOT")"
fi
JOB_FILE="$DATASET_DIR/commands/traci_jobs.tsv"
BASELINE_OUTPUT_ROOT="$(realpath -m "$DATASET_DIR/outputs")"
[[ "$OUTPUT_ROOT" != "$BASELINE_OUTPUT_ROOT" ]] \
    || die "OUTPUT_ROOT must not be the existing baseline directory: $BASELINE_OUTPUT_ROOT"
[[ "$OUTPUT_ROOT" != "$DATASET_DIR" ]] \
    || die "OUTPUT_ROOT must not be the dataset directory itself: $DATASET_DIR"

TRACI_OUTPUT_DIR="$OUTPUT_ROOT/traci"
TRAINING_OUTPUT_DIR="$OUTPUT_ROOT/training"
EVENTS_OUTPUT_DIR="$OUTPUT_ROOT/events"
CYCLES_OUTPUT_DIR="$OUTPUT_ROOT/cycles"
ANALYSIS_OUTPUT_DIR="$OUTPUT_ROOT/analysis"
TRIPINFO_OUTPUT_DIR="$OUTPUT_ROOT/tripinfo"
LOG_OUTPUT_DIR="$OUTPUT_ROOT/logs"
STATUS_DIR="$OUTPUT_ROOT/status"
RUNTIME_DIR="$OUTPUT_ROOT/runtime"
RUN_CONFIG_FILE="$OUTPUT_ROOT/run_config.env"
SUMO_WRAPPER="$RUNTIME_DIR/sumo_with_tripinfo_override.sh"

[[ -d "$DATASET_DIR" ]] || die "dataset directory not found: $DATASET_DIR"
[[ -f "$JOB_FILE" ]] || die "TraCI job manifest not found: $JOB_FILE"
[[ -f "$TRACI_SCRIPT" ]] || die "TraCI collector not found: $TRACI_SCRIPT"
[[ -f "$NET_FILE" ]] || die "network file not found: $NET_FILE (set NET_FILE to override)"

ACTUAL_EPISODES="$(awk -F '\t' 'NR > 1 && $1 != "" { count++ } END { print count + 0 }' "$JOB_FILE")"
[[ "$ACTUAL_EPISODES" -ge 1 ]] || die "job manifest contains no episodes: $JOB_FILE"
if [[ "$EXPECTED_EPISODES" -ne 0 && "$ACTUAL_EPISODES" -ne "$EXPECTED_EPISODES" ]]; then
    die "expected $EXPECTED_EPISODES episodes, but manifest contains $ACTUAL_EPISODES"
fi
[[ "$START_INDEX" -le "$ACTUAL_EPISODES" ]] \
    || die "START_INDEX=$START_INDEX is outside the available range 1-$ACTUAL_EPISODES"
SELECTED_EPISODES=$((ACTUAL_EPISODES - START_INDEX + 1))

REAL_SUMO_BINARY="$SUMO_BINARY"
if [[ "$DRY_RUN" == "0" ]]; then
    [[ -n "${SUMO_HOME:-}" ]] || die "SUMO_HOME is not set"
    [[ -d "$SUMO_HOME/tools" ]] || die "SUMO tools directory not found: $SUMO_HOME/tools"
    REAL_SUMO_BINARY="$(command -v "$SUMO_BINARY" 2>/dev/null)" \
        || die "SUMO binary not found: $SUMO_BINARY"
    REAL_SUMO_BINARY="$(realpath -m "$REAL_SUMO_BINARY")"
    "${TRACI_COMMAND[@]}" --help >/dev/null
fi

NORMALIZED_OUTPUTS="$(printf '%s' "$TRACI_OUTPUTS" | tr '[:upper:]' '[:lower:]' | tr -d ' ')"

render_run_config() {
    printf 'schema_version=1\n'
    printf 'dataset_dir=%q\n' "$DATASET_DIR"
    printf 'net_file=%q\n' "$NET_FILE"
    printf 'traci_script=%q\n' "$TRACI_SCRIPT"
    printf 'traci_module=%q\n' "$TRACI_MODULE"
    printf 'sumo_binary=%q\n' "$SUMO_BINARY"
    printf 'traci_outputs=%q\n' "$NORMALIZED_OUTPUTS"
    printf 'expected_episodes=%q\n' "$EXPECTED_EPISODES"
    printf 'stop_speed_threshold=%q\n' "$STOP_SPEED_THRESHOLD"
    printf 'low_speed_threshold=%q\n' "$LOW_SPEED_THRESHOLD"
    printf 'queue_speed_threshold=%q\n' "$QUEUE_SPEED_THRESHOLD"
}

check_output_root() {
    if [[ -f "$RUN_CONFIG_FILE" ]]; then
        if ! cmp -s <(render_run_config) "$RUN_CONFIG_FILE"; then
            echo "[ERROR] output root belongs to a different run configuration:" >&2
            echo "        $OUTPUT_ROOT" >&2
            echo "        Choose a new --run-tag/--output-root for different thresholds or outputs." >&2
            return 1
        fi
        return 0
    fi

    if [[ -d "$OUTPUT_ROOT" ]] \
        && [[ -n "$(find "$OUTPUT_ROOT" -mindepth 1 -maxdepth 1 -print -quit 2>/dev/null)" ]]; then
        die "OUTPUT_ROOT is non-empty but has no run_config.env: $OUTPUT_ROOT"
    fi
}

write_sumo_wrapper() {
    local temp_wrapper="${SUMO_WRAPPER}.tmp.${BASHPID}"
    printf '%s\n' \
        '#!/usr/bin/env bash' \
        'set -Eeuo pipefail' \
        ': "${TRACI_REAL_SUMO_BINARY:?TRACI_REAL_SUMO_BINARY is required}"' \
        ': "${TRACI_TRIPINFO_OUTPUT:?TRACI_TRIPINFO_OUTPUT is required}"' \
        'exec "$TRACI_REAL_SUMO_BINARY" "$@" --tripinfo-output "$TRACI_TRIPINFO_OUTPUT"' \
        > "$temp_wrapper"
    chmod 700 "$temp_wrapper"
    mv -f "$temp_wrapper" "$SUMO_WRAPPER"
}

initialize_output_root() {
    local temp_config="${RUN_CONFIG_FILE}.tmp.${BASHPID}"
    mkdir -p \
        "$TRACI_OUTPUT_DIR" \
        "$TRAINING_OUTPUT_DIR" \
        "$EVENTS_OUTPUT_DIR" \
        "$CYCLES_OUTPUT_DIR" \
        "$ANALYSIS_OUTPUT_DIR" \
        "$TRIPINFO_OUTPUT_DIR" \
        "$LOG_OUTPUT_DIR" \
        "$STATUS_DIR" \
        "$RUNTIME_DIR"
    if [[ ! -f "$RUN_CONFIG_FILE" ]]; then
        render_run_config > "$temp_config"
        mv -f "$temp_config" "$RUN_CONFIG_FILE"
    fi
    write_sumo_wrapper
}

check_output_root
if [[ "$DRY_RUN" == "0" ]]; then
    initialize_output_root
fi

output_enabled() {
    local output_name="$1"
    [[ ",$NORMALIZED_OUTPUTS," == *",all,"* || ",$NORMALIZED_OUTPUTS," == *",$output_name,"* ]]
}

analysis_outputs_complete() {
    local episode_id="$1"
    [[ -s "$ANALYSIS_OUTPUT_DIR/vehicle_trips_$episode_id.csv" ]] || return 1
    [[ -s "$ANALYSIS_OUTPUT_DIR/vehicle_segments_$episode_id.csv" ]] || return 1
    [[ -s "$ANALYSIS_OUTPUT_DIR/movement_tail_summary_$episode_id.csv" ]] || return 1
    [[ -s "$ANALYSIS_OUTPUT_DIR/extreme_vehicle_cases_$episode_id.csv" ]] || return 1
    [[ -s "$ANALYSIS_OUTPUT_DIR/extreme_segment_cases_$episode_id.csv" ]] || return 1
    [[ -s "$ANALYSIS_OUTPUT_DIR/scenario_summary_$episode_id.csv" ]] || return 1
    return 0
}

outputs_complete() {
    local episode_id="$1"
    local tripinfo_abs="$2"
    local legacy_abs="$3"
    local full_abs="$4"
    local events_abs="$5"
    local cycles_abs="$6"

    [[ -s "$tripinfo_abs" ]] || return 1
    if output_enabled legacy; then
        [[ -s "$legacy_abs" ]] || return 1
    fi
    if output_enabled training; then
        [[ -s "$full_abs" ]] || return 1
    fi
    if output_enabled events; then
        [[ -s "$events_abs" ]] || return 1
    fi
    if output_enabled cycles; then
        [[ -s "$cycles_abs" ]] || return 1
    fi
    if output_enabled analysis; then
        analysis_outputs_complete "$episode_id" || return 1
    fi
    return 0
}

write_status_file() {
    local status_file="$1"
    shift
    local temp_file="${status_file}.tmp.${BASHPID}"
    printf '%s\n' "$@" > "$temp_file"
    mv -f "$temp_file" "$status_file"
}

declare -a JOB_MANIFEST_INDEX=()
declare -a JOB_EPISODE_ID=()
declare -a JOB_SPLIT=()
declare -a JOB_SUMOCFG=()
declare -a JOB_LEGACY=()
declare -a JOB_FULL=()
declare -a JOB_EVENTS=()
declare -a JOB_CYCLES=()
declare -a JOB_LOG=()
declare -a JOB_TRIPINFO=()
declare -a JOB_STATUS=()
declare -A SEEN_EPISODE_IDS=()

selected_count=0
skipped=0
dry_run_count=0
manifest_index=0

echo "Dataset:       $DATASET_DIR"
echo "Episodes:      $ACTUAL_EPISODES"
echo "Start index:   $START_INDEX (inclusive)"
echo "Selected:      $SELECTED_EPISODES"
echo "Parallel jobs: $MAX_PARALLEL"
echo "Poll interval: ${POLL_INTERVAL}s"
echo "Network:       $NET_FILE"
echo "TraCI script:  $TRACI_SCRIPT"
echo "SUMO binary:   $SUMO_BINARY"
echo "Outputs:       $TRACI_OUTPUTS"
echo "Output root:   $OUTPUT_ROOT"
echo "Stop speed:    $STOP_SPEED_THRESHOLD m/s"
echo "Low speed:     $LOW_SPEED_THRESHOLD m/s"
echo "Queue speed:   $QUEUE_SPEED_THRESHOLD m/s"
echo "Resume:        $RESUME"
echo "Dry run:       $DRY_RUN"

# Read and validate the complete selected queue before starting any background
# workers. This prevents a malformed later row from leaving earlier jobs orphaned.
while IFS=$'\t' read -r episode_id split sumocfg legacy full events cycles log_file; do
    [[ -n "$episode_id" ]] || continue
    log_file="${log_file%$'\r'}"
    manifest_index=$((manifest_index + 1))

    if [[ "$manifest_index" -lt "$START_INDEX" ]]; then
        continue
    fi
    selected_count=$((selected_count + 1))

    [[ -z "${SEEN_EPISODE_IDS[$episode_id]+x}" ]] \
        || die "duplicate episode_id in selected manifest rows: $episode_id"
    SEEN_EPISODE_IDS["$episode_id"]=1

    sumocfg_abs="$DATASET_DIR/$sumocfg"
    legacy_abs="$TRACI_OUTPUT_DIR/$(basename "$legacy")"
    full_abs="$TRAINING_OUTPUT_DIR/$(basename "$full")"
    events_abs="$EVENTS_OUTPUT_DIR/$(basename "$events")"
    cycles_abs="$CYCLES_OUTPUT_DIR/$(basename "$cycles")"
    log_abs="$LOG_OUTPUT_DIR/$(basename "$log_file")"
    tripinfo_abs="$TRIPINFO_OUTPUT_DIR/tripinfo_$episode_id.xml"
    status_file="$STATUS_DIR/$episode_id.status"

    [[ -f "$sumocfg_abs" ]] || die "missing sumocfg for $episode_id: $sumocfg_abs"

    if [[ "$RESUME" == "1" && -f "$status_file" ]] \
        && head -n 1 "$status_file" | grep -qx 'SUCCESS'; then
        if outputs_complete "$episode_id" "$tripinfo_abs" "$legacy_abs" "$full_abs" "$events_abs" "$cycles_abs"; then
            echo "[SKIP $manifest_index/$ACTUAL_EPISODES] $episode_id already completed"
            skipped=$((skipped + 1))
            continue
        fi
        echo "[RETRY $manifest_index/$ACTUAL_EPISODES] $episode_id has an incomplete output set"
    fi

    if [[ "$DRY_RUN" == "1" ]]; then
        command=(
            env
            "TRACI_REAL_SUMO_BINARY=$REAL_SUMO_BINARY"
            "TRACI_TRIPINFO_OUTPUT=$tripinfo_abs"
            "${TRACI_COMMAND[@]}"
            --sumo-config "$sumocfg_abs"
            --net-file "$NET_FILE"
            --sumo-binary "$SUMO_WRAPPER"
            --outputs "$TRACI_OUTPUTS"
            --legacy-edge-output "$legacy_abs"
            --training-output "$full_abs"
            --vehicle-event-output "$events_abs"
            --cycle-summary-output "$cycles_abs"
            --stop-speed-threshold "$STOP_SPEED_THRESHOLD"
            --low-speed-threshold "$LOW_SPEED_THRESHOLD"
            --queue-speed-threshold "$QUEUE_SPEED_THRESHOLD"
        )
        echo "[DRY-RUN $manifest_index/$ACTUAL_EPISODES] $episode_id split=$split"
        printf '  '
        printf '%q ' "${command[@]}"
        printf '\n'
        dry_run_count=$((dry_run_count + 1))
        continue
    fi

    JOB_MANIFEST_INDEX+=("$manifest_index")
    JOB_EPISODE_ID+=("$episode_id")
    JOB_SPLIT+=("$split")
    JOB_SUMOCFG+=("$sumocfg_abs")
    JOB_LEGACY+=("$legacy_abs")
    JOB_FULL+=("$full_abs")
    JOB_EVENTS+=("$events_abs")
    JOB_CYCLES+=("$cycles_abs")
    JOB_LOG+=("$log_abs")
    JOB_TRIPINFO+=("$tripinfo_abs")
    JOB_STATUS+=("$status_file")
done < <(tail -n +2 "$JOB_FILE")

[[ "$selected_count" -eq "$SELECTED_EPISODES" ]] \
    || die "manifest row count changed while it was being read"

if [[ "$DRY_RUN" == "1" ]]; then
    echo "Summary: start_index=$START_INDEX selected=$selected_count dry_run=$dry_run_count skipped=$skipped"
    exit 0
fi

mkdir -p "$STATUS_DIR"

worker_abort() {
    local signal_name="$1"
    local exit_code="$2"
    if [[ -n "${collector_pid:-}" ]]; then
        kill -TERM "$collector_pid" 2>/dev/null || true
        wait "$collector_pid" 2>/dev/null || true
    fi
    write_status_file "$status_file" \
        "INTERRUPTED" \
        "finished_at=$(date --iso-8601=seconds)" \
        "signal=$signal_name"
    exit "$exit_code"
}

run_job() {
    local slot="$1"
    local manifest_row="${JOB_MANIFEST_INDEX[$slot]}"
    local episode_id="${JOB_EPISODE_ID[$slot]}"
    local split="${JOB_SPLIT[$slot]}"
    local sumocfg_abs="${JOB_SUMOCFG[$slot]}"
    local legacy_abs="${JOB_LEGACY[$slot]}"
    local full_abs="${JOB_FULL[$slot]}"
    local events_abs="${JOB_EVENTS[$slot]}"
    local cycles_abs="${JOB_CYCLES[$slot]}"
    local log_abs="${JOB_LOG[$slot]}"
    local tripinfo_abs="${JOB_TRIPINFO[$slot]}"
    local status_file="${JOB_STATUS[$slot]}"
    local collector_pid=""
    local collector_status=0
    local started_at="$(date --iso-8601=seconds)"
    local finished_at=""
    local -a missing_outputs=()
    local -a command=(
        env
        "TRACI_REAL_SUMO_BINARY=$REAL_SUMO_BINARY"
        "TRACI_TRIPINFO_OUTPUT=$tripinfo_abs"
        "${TRACI_COMMAND[@]}"
        --sumo-config "$sumocfg_abs"
        --net-file "$NET_FILE"
        --sumo-binary "$SUMO_WRAPPER"
        --outputs "$TRACI_OUTPUTS"
        --legacy-edge-output "$legacy_abs"
        --training-output "$full_abs"
        --vehicle-event-output "$events_abs"
        --cycle-summary-output "$cycles_abs"
        --stop-speed-threshold "$STOP_SPEED_THRESHOLD"
        --low-speed-threshold "$LOW_SPEED_THRESHOLD"
        --queue-speed-threshold "$QUEUE_SPEED_THRESHOLD"
    )

    trap 'worker_abort TERM 143' TERM
    trap 'worker_abort INT 130' INT

    mkdir -p \
        "$(dirname "$legacy_abs")" \
        "$(dirname "$full_abs")" \
        "$(dirname "$events_abs")" \
        "$(dirname "$cycles_abs")" \
        "$(dirname "$log_abs")" \
        "$(dirname "$tripinfo_abs")" \
        "$(dirname "$status_file")" \
        "$ANALYSIS_OUTPUT_DIR"

    write_status_file "$status_file" \
        "RUNNING" \
        "started_at=$started_at" \
        "worker_pid=$BASHPID" \
        "manifest_index=$manifest_row"

    {
        printf 'episode_id=%s\n' "$episode_id"
        printf 'split=%s\n' "$split"
        printf 'manifest_index=%s\n' "$manifest_row"
        printf 'started_at=%s\n' "$started_at"
        printf 'command='
        printf '%q ' "${command[@]}"
        printf '\n\n'
    } > "$log_abs"

    "${command[@]}" >> "$log_abs" 2>&1 &
    collector_pid=$!
    set +e
    wait "$collector_pid"
    collector_status=$?
    set -e
    collector_pid=""
    trap - TERM INT

    if [[ ! -s "$tripinfo_abs" ]]; then
        missing_outputs+=("$tripinfo_abs")
    fi
    if output_enabled legacy && [[ ! -s "$legacy_abs" ]]; then
        missing_outputs+=("$legacy_abs")
    fi
    if output_enabled training && [[ ! -s "$full_abs" ]]; then
        missing_outputs+=("$full_abs")
    fi
    if output_enabled events && [[ ! -s "$events_abs" ]]; then
        missing_outputs+=("$events_abs")
    fi
    if output_enabled cycles && [[ ! -s "$cycles_abs" ]]; then
        missing_outputs+=("$cycles_abs")
    fi
    if output_enabled analysis; then
        local analysis_abs
        for analysis_abs in \
            "$ANALYSIS_OUTPUT_DIR/vehicle_trips_$episode_id.csv" \
            "$ANALYSIS_OUTPUT_DIR/vehicle_segments_$episode_id.csv" \
            "$ANALYSIS_OUTPUT_DIR/movement_tail_summary_$episode_id.csv" \
            "$ANALYSIS_OUTPUT_DIR/extreme_vehicle_cases_$episode_id.csv" \
            "$ANALYSIS_OUTPUT_DIR/extreme_segment_cases_$episode_id.csv" \
            "$ANALYSIS_OUTPUT_DIR/scenario_summary_$episode_id.csv"; do
            if [[ ! -s "$analysis_abs" ]]; then
                missing_outputs+=("$analysis_abs")
            fi
        done
    fi

    finished_at="$(date --iso-8601=seconds)"
    {
        printf '\nfinished_at=%s\n' "$finished_at"
        printf 'collector_exit=%s\n' "$collector_status"
        printf 'missing_outputs=%s\n' "${missing_outputs[*]:-none}"
    } >> "$log_abs"

    if [[ "$collector_status" -eq 0 && "${#missing_outputs[@]}" -eq 0 ]]; then
        write_status_file "$status_file" \
            "SUCCESS" \
            "started_at=$started_at" \
            "finished_at=$finished_at" \
            "collector_exit=$collector_status"
        return 0
    fi

    write_status_file "$status_file" \
        "FAILED" \
        "started_at=$started_at" \
        "finished_at=$finished_at" \
        "collector_exit=$collector_status" \
        "missing_outputs=${missing_outputs[*]:-none}"
    return 1
}

TOTAL_QUEUED="${#JOB_EPISODE_ID[@]}"
next_slot=0
running=0
succeeded=0
failed=0
launched=0
progress_on_line=0
last_non_tty_progress=0
declare -A PID_TO_SLOT=()

end_progress_line() {
    if [[ "$progress_on_line" -eq 1 ]]; then
        printf '\n'
        progress_on_line=0
    fi
}

log_event() {
    end_progress_line
    printf '%s\n' "$*"
}

print_progress() {
    local force="${1:-0}"
    local queued=$((TOTAL_QUEUED - next_slot))
    local now
    now="$(date +%s)"
    if [[ -t 1 ]]; then
        printf '\r\033[K[STATUS %s] running=%d/%d queued=%d succeeded=%d failed=%d skipped=%d' \
            "$(date '+%H:%M:%S')" "$running" "$MAX_PARALLEL" "$queued" \
            "$succeeded" "$failed" "$skipped"
        progress_on_line=1
    elif [[ "$force" -eq 1 || $((now - last_non_tty_progress)) -ge 30 ]]; then
        printf '[STATUS %s] running=%d/%d queued=%d succeeded=%d failed=%d skipped=%d\n' \
            "$(date '+%H:%M:%S')" "$running" "$MAX_PARALLEL" "$queued" \
            "$succeeded" "$failed" "$skipped"
        last_non_tty_progress="$now"
    fi
}

launch_next_job() {
    local slot="$next_slot"
    local pid
    run_job "$slot" &
    pid=$!
    PID_TO_SLOT["$pid"]="$slot"
    next_slot=$((next_slot + 1))
    running=$((running + 1))
    launched=$((launched + 1))
    log_event "[START ${JOB_MANIFEST_INDEX[$slot]}/$ACTUAL_EPISODES] ${JOB_EPISODE_ID[$slot]} split=${JOB_SPLIT[$slot]} pid=$pid running=$running/$MAX_PARALLEL"
}

reap_finished_jobs() {
    local pid slot worker_status
    REAPED_ANY=0
    for pid in "${!PID_TO_SLOT[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            continue
        fi

        slot="${PID_TO_SLOT[$pid]}"
        set +e
        wait "$pid"
        worker_status=$?
        set -e
        unset 'PID_TO_SLOT[$pid]'
        running=$((running - 1))
        REAPED_ANY=1

        if [[ "$worker_status" -eq 0 ]]; then
            succeeded=$((succeeded + 1))
            log_event "[OK ${JOB_MANIFEST_INDEX[$slot]}/$ACTUAL_EPISODES] ${JOB_EPISODE_ID[$slot]} running=$running/$MAX_PARALLEL log=${JOB_LOG[$slot]}"
        else
            failed=$((failed + 1))
            log_event "[FAIL ${JOB_MANIFEST_INDEX[$slot]}/$ACTUAL_EPISODES] ${JOB_EPISODE_ID[$slot]} exit=$worker_status running=$running/$MAX_PARALLEL log=${JOB_LOG[$slot]}"
        fi
    done
}

abort_scheduler() {
    local signal_name="$1"
    local exit_code="$2"
    local pid
    trap - INT TERM
    end_progress_line
    echo "[INTERRUPT] received $signal_name; stopping $running running job(s)..." >&2
    for pid in "${!PID_TO_SLOT[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    for pid in "${!PID_TO_SLOT[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    echo "[INTERRUPT] workers stopped; rerun with RESUME=1 to continue." >&2
    exit "$exit_code"
}

trap 'abort_scheduler INT 130' INT
trap 'abort_scheduler TERM 143' TERM

if [[ "$TOTAL_QUEUED" -eq 0 ]]; then
    echo "Summary: start_index=$START_INDEX selected=$selected_count queued=0 succeeded=0 skipped=$skipped failed=0"
    exit 0
fi

echo "Queued for execution: $TOTAL_QUEUED episode(s)"

while [[ "$next_slot" -lt "$TOTAL_QUEUED" || "$running" -gt 0 ]]; do
    while [[ "$running" -lt "$MAX_PARALLEL" && "$next_slot" -lt "$TOTAL_QUEUED" ]]; do
        launch_next_job
    done

    print_progress 1
    if [[ "$running" -eq 0 ]]; then
        break
    fi

    while :; do
        sleep "$POLL_INTERVAL"
        reap_finished_jobs
        print_progress 0
        if [[ "$REAPED_ANY" -eq 1 ]]; then
            break
        fi
    done
done

end_progress_line
trap - INT TERM

echo "Summary: start_index=$START_INDEX selected=$selected_count launched=$launched succeeded=$succeeded skipped=$skipped failed=$failed"

if [[ "$failed" -ne 0 ]]; then
    exit 1
fi
