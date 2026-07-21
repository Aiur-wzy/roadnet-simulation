#!/usr/bin/env bash
set -Eeuo pipefail

# Run TraCI episodes concurrently with a fixed-size worker pool.
# Whenever one episode finishes, the next queued episode is started automatically.
#
# Expected layout (all paths may be overridden with environment variables):
#   current-directory/
#   ├── data/test_new.net.xml
#   ├── TraCI_Python_Adjusted.py
#   ├── run_all_training_traci.sh
#   └── train_data/
#       ├── commands/traci_jobs.tsv
#       ├── sumocfg/*.sumocfg
#       └── outputs/
#
# Basic usage (parallelism defaults to the number of available CPU cores):
#   bash ./run_all_training_traci.sh
#
# Run at most 8 episodes at the same time:
#   bash ./run_all_training_traci.sh --jobs 8
#   MAX_PARALLEL=8 bash ./run_all_training_traci.sh
#
# Resume from an explicit 1-based episode number (inclusive):
#   bash ./run_all_training_traci.sh --start-index 37 --jobs 8
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
  bash ./run_all_training_traci.sh [START_INDEX]
  bash ./run_all_training_traci.sh [options]

Options:
  -j, --jobs N             Maximum concurrent TraCI episodes.
                           Default: number of available CPU cores.
  --start, --start-index N Start from 1-based manifest row N (inclusive).
  --poll-interval SECONDS  Seconds between worker-pool checks. Default: 1.
  --resume                 Skip completed episodes (default).
  --no-resume              Rerun completed episodes in the selected range.
  --dry-run                Print queued commands without running them.
  -h, --help               Show this help.

Environment equivalents:
  MAX_PARALLEL, START_INDEX, POLL_INTERVAL, RESUME, DRY_RUN,
  DATASET_DIR, TRACI_SCRIPT, SUMO_BINARY, TRACI_OUTPUTS, NET_FILE,
  EXPECTED_EPISODES

Examples:
  bash ./run_all_training_traci.sh --jobs 8
  bash ./run_all_training_traci.sh --start-index 37 --jobs 12
  MAX_PARALLEL=8 RESUME=1 bash ./run_all_training_traci.sh
  DRY_RUN=1 bash ./run_all_training_traci.sh --start-index 37
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASET_DIR="${DATASET_DIR:-$SCRIPT_DIR/train_data}"
TRACI_SCRIPT="${TRACI_SCRIPT:-$SCRIPT_DIR/TraCI_Python_Adjusted.py}"
SUMO_BINARY="${SUMO_BINARY:-sumo}"
TRACI_OUTPUTS="${TRACI_OUTPUTS:-legacy}"
NET_FILE="${NET_FILE:-$SCRIPT_DIR/data/test_new.net.xml}"
EXPECTED_EPISODES="${EXPECTED_EPISODES:-80}"
RESUME="${RESUME:-1}"
DRY_RUN="${DRY_RUN:-0}"
START_INDEX="${START_INDEX:-1}"
MAX_PARALLEL="${MAX_PARALLEL:-$(detect_cpu_count)}"
POLL_INTERVAL="${POLL_INTERVAL:-1}"

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

DATASET_DIR="$(realpath -m "$DATASET_DIR")"
TRACI_SCRIPT="$(realpath -m "$TRACI_SCRIPT")"
NET_FILE="$(realpath -m "$NET_FILE")"
JOB_FILE="$DATASET_DIR/commands/traci_jobs.tsv"
STATUS_DIR="$DATASET_DIR/outputs/status"

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

if [[ "$DRY_RUN" == "0" ]]; then
    [[ -n "${SUMO_HOME:-}" ]] || die "SUMO_HOME is not set"
    [[ -d "$SUMO_HOME/tools" ]] || die "SUMO tools directory not found: $SUMO_HOME/tools"
    command -v "$SUMO_BINARY" >/dev/null 2>&1 || die "SUMO binary not found: $SUMO_BINARY"
    python3 "$TRACI_SCRIPT" --help >/dev/null
fi

NORMALIZED_OUTPUTS="$(printf '%s' "$TRACI_OUTPUTS" | tr '[:upper:]' '[:lower:]' | tr -d ' ')"

output_enabled() {
    local output_name="$1"
    [[ ",$NORMALIZED_OUTPUTS," == *",all,"* || ",$NORMALIZED_OUTPUTS," == *",$output_name,"* ]]
}

outputs_complete() {
    local tripinfo_abs="$1"
    local legacy_abs="$2"
    local full_abs="$3"
    local events_abs="$4"
    local cycles_abs="$5"

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
    legacy_abs="$DATASET_DIR/$legacy"
    full_abs="$DATASET_DIR/$full"
    events_abs="$DATASET_DIR/$events"
    cycles_abs="$DATASET_DIR/$cycles"
    log_abs="$DATASET_DIR/$log_file"
    tripinfo_abs="$DATASET_DIR/outputs/tripinfo/tripinfo_$episode_id.xml"
    status_file="$STATUS_DIR/$episode_id.status"

    [[ -f "$sumocfg_abs" ]] || die "missing sumocfg for $episode_id: $sumocfg_abs"

    if [[ "$RESUME" == "1" && -f "$status_file" ]] \
        && head -n 1 "$status_file" | grep -qx 'SUCCESS'; then
        if outputs_complete "$tripinfo_abs" "$legacy_abs" "$full_abs" "$events_abs" "$cycles_abs"; then
            echo "[SKIP $manifest_index/$ACTUAL_EPISODES] $episode_id already completed"
            skipped=$((skipped + 1))
            continue
        fi
        echo "[RETRY $manifest_index/$ACTUAL_EPISODES] $episode_id has an incomplete output set"
    fi

    if [[ "$DRY_RUN" == "1" ]]; then
        command=(
            python3 "$TRACI_SCRIPT"
            --sumo-config "$sumocfg_abs"
            --net-file "$NET_FILE"
            --sumo-binary "$SUMO_BINARY"
            --outputs "$TRACI_OUTPUTS"
            --legacy-edge-output "$legacy_abs"
            --training-output "$full_abs"
            --vehicle-event-output "$events_abs"
            --cycle-summary-output "$cycles_abs"
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
        python3 "$TRACI_SCRIPT"
        --sumo-config "$sumocfg_abs"
        --net-file "$NET_FILE"
        --sumo-binary "$SUMO_BINARY"
        --outputs "$TRACI_OUTPUTS"
        --legacy-edge-output "$legacy_abs"
        --training-output "$full_abs"
        --vehicle-event-output "$events_abs"
        --cycle-summary-output "$cycles_abs"
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
        "$(dirname "$status_file")"

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
