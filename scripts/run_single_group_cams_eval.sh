#!/usr/bin/env bash
set -euo pipefail
set -o pipefail

DEFAULT_NET_FILE="data/test_random_offsets_seed20260708.net.xml"
DEFAULT_DATA_DIR="data"
DEFAULT_OUTPUT_ROOT="eval_results/single_group"
DEFAULT_TRAVEL_TIME_MODE="speed-net"
DEFAULT_LANE_DISCHARGE_INTERVAL="1"
ROUTE_SUFFIX="no_change_random_offset_seed20260708"
SIM_BIN="./build/Simulation_Prediction"
PREDICTION_TRIPINFO_SUPPORTED="false"

usage() {
  cat <<'USAGE'
Usage: scripts/run_single_group_cams_eval.sh --group <group_name> [options]

Run one CAMS/SUMO single-group evaluation with isolated outputs.

Required:
  --group <group_name>              Traffic group name.

Options:
  --run-id <run_id>                 Run directory id (default: YYYYMMDD_HHMMSS).
  --output-root <path>              Output root (default: eval_results/single_group).
  --net-file <path>                 SUMO net file (default: data/test_random_offsets_seed20260708.net.xml).
  --data-dir <path>                 Input data directory (default: data).
  --travel-time-mode <mode>         Travel-time mode (default: speed-net).
  --lane-discharge-interval <int>   Lane discharge interval (default: 1).
  --dry-run                         Print inputs and command without running CAMS or creating outputs.
  --force                           Allow overwriting an existing run directory for the same run id.
  --help                            Show this help.
USAGE
}

shell_join() {
  printf '%q ' "$@"
  printf '\n'
}

safe_name() {
  local value="$1"
  value="${value//\//_}"
  value="${value// /_}"
  printf '%s' "$value" | tr -c 'A-Za-z0-9._-' '_'
}

error() {
  printf 'ERROR: %b\n' "$1" >&2
}

GROUP=""
RUN_ID=""
OUTPUT_ROOT="$DEFAULT_OUTPUT_ROOT"
NET_FILE="$DEFAULT_NET_FILE"
DATA_DIR="$DEFAULT_DATA_DIR"
TRAVEL_TIME_MODE="$DEFAULT_TRAVEL_TIME_MODE"
LANE_DISCHARGE_INTERVAL="$DEFAULT_LANE_DISCHARGE_INTERVAL"
DRY_RUN="false"
FORCE="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --group) GROUP="${2:-}"; shift 2 ;;
    --run-id) RUN_ID="${2:-}"; shift 2 ;;
    --output-root) OUTPUT_ROOT="${2:-}"; shift 2 ;;
    --net-file) NET_FILE="${2:-}"; shift 2 ;;
    --data-dir) DATA_DIR="${2:-}"; shift 2 ;;
    --travel-time-mode) TRAVEL_TIME_MODE="${2:-}"; shift 2 ;;
    --lane-discharge-interval) LANE_DISCHARGE_INTERVAL="${2:-}"; shift 2 ;;
    --dry-run) DRY_RUN="true"; shift ;;
    --force) FORCE="true"; shift ;;
    --help|-h) usage; exit 0 ;;
    *) error "unknown argument: $1"; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$GROUP" ]]; then
  error "--group is required"
  usage >&2
  exit 2
fi
if [[ -z "$RUN_ID" ]]; then
  RUN_ID="$(date +%Y%m%d_%H%M%S)"
fi

ROUTE_FILE="$DATA_DIR/${GROUP}_${ROUTE_SUFFIX}.rou.xml"
TRIPINFO_FILE="$DATA_DIR/tripinfo_${GROUP}_${ROUTE_SUFFIX}.xml"
SUMOCFG_FILE="$DATA_DIR/${GROUP}_${ROUTE_SUFFIX}.sumocfg"
MODE_SAFE="$(safe_name "$TRAVEL_TIME_MODE")"
INTERVAL_SAFE="$(safe_name "$LANE_DISCHARGE_INTERVAL")"
RUN_CLASS="${MODE_SAFE}_interval${INTERVAL_SAFE}"
RUN_DIR="$OUTPUT_ROOT/$GROUP/$RUN_CLASS/$RUN_ID"
EVAL_OUTPUT="$RUN_DIR/sumo_eval_${MODE_SAFE}.csv"
PREDICTION_TRIPINFO_OUTPUT="$RUN_DIR/prediction_tripinfo_${MODE_SAFE}.xml"

CMD=("$SIM_BIN" --use-sumo --sumo-net "$NET_FILE" --sumo-route "$ROUTE_FILE" --sumo-tripinfo "$TRIPINFO_FILE" --eval-output "$EVAL_OUTPUT")
if [[ "$PREDICTION_TRIPINFO_SUPPORTED" == "true" ]]; then
  CMD+=(--prediction-tripinfo-output "$PREDICTION_TRIPINFO_OUTPUT")
fi
CMD+=(--travel-time-mode "$TRAVEL_TIME_MODE" --lane-discharge-interval "$LANE_DISCHARGE_INTERVAL")

if [[ "$DRY_RUN" == "true" ]]; then
  printf 'DRY RUN: CAMS will not be executed and no output directory will be created.\n'
  printf 'group=%s\nrun_id=%s\nrun_dir=%s\n' "$GROUP" "$RUN_ID" "$RUN_DIR"
  printf 'network=%s [%s]\n' "$NET_FILE" "$([[ -f "$NET_FILE" ]] && echo present || echo missing)"
  printf 'route=%s [%s]\n' "$ROUTE_FILE" "$([[ -f "$ROUTE_FILE" ]] && echo present || echo missing)"
  printf 'tripinfo=%s [%s]\n' "$TRIPINFO_FILE" "$([[ -f "$TRIPINFO_FILE" ]] && echo present || echo missing)"
  printf 'prediction_tripinfo_supported=%s\n' "$PREDICTION_TRIPINFO_SUPPORTED"
  printf 'command:\n'
  shell_join "${CMD[@]}"
  exit 0
fi

if [[ ! -x "$SIM_BIN" ]]; then
  error "missing executable: $SIM_BIN"
  exit 1
fi
if [[ ! -f "$NET_FILE" ]]; then
  error "missing network file:\n$NET_FILE"
  exit 1
fi
if [[ ! -f "$ROUTE_FILE" ]]; then
  printf 'ERROR: missing route file:\n%s\n\nGenerate it first with:\npython3 generate_route_on_test_net_full.py \\\n  --group %s \\\n  --output %s\n' "$ROUTE_FILE" "$GROUP" "$ROUTE_FILE" >&2
  exit 1
fi
if [[ ! -f "$TRIPINFO_FILE" ]]; then
  printf 'ERROR: missing matching SUMO tripinfo:\n%s\n\nRun SUMO with:\n%s\n' "$TRIPINFO_FILE" "$SUMOCFG_FILE" >&2
  exit 1
fi
if [[ -e "$RUN_DIR" && "$FORCE" != "true" ]]; then
  error "run directory already exists; choose a new --run-id or pass --force:\n$RUN_DIR"
  exit 1
fi
mkdir -p "$RUN_DIR"
if [[ "$FORCE" != "true" ]]; then
  for output in "$EVAL_OUTPUT" "$RUN_DIR/run.log"; do
    if [[ -e "$output" ]]; then error "refusing to overwrite existing file: $output"; exit 1; fi
  done
  if [[ "$PREDICTION_TRIPINFO_SUPPORTED" == "true" && -e "$PREDICTION_TRIPINFO_OUTPUT" ]]; then
    error "refusing to overwrite existing file: $PREDICTION_TRIPINFO_OUTPUT"; exit 1
  fi
fi

shell_join "${CMD[@]}" > "$RUN_DIR/command.txt"
GIT_COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [[ -z "$(git status --short 2>/dev/null || true)" ]]; then GIT_STATUS_CLEAN="true"; else GIT_STATUS_CLEAN="false"; fi
{
  printf 'group=%s\n' "$GROUP"
  printf 'run_id=%s\n' "$RUN_ID"
  printf 'timestamp=%s\n' "$(date -Iseconds)"
  printf 'git_commit=%s\n' "$GIT_COMMIT"
  printf 'git_status_clean=%s\n' "$GIT_STATUS_CLEAN"
  printf 'network=%s\nroute=%s\ntripinfo=%s\n' "$NET_FILE" "$ROUTE_FILE" "$TRIPINFO_FILE"
  printf 'travel_time_mode=%s\nlane_discharge_interval=%s\n' "$TRAVEL_TIME_MODE" "$LANE_DISCHARGE_INTERVAL"
  printf 'eval_output=%s\n' "$EVAL_OUTPUT"
  if [[ "$PREDICTION_TRIPINFO_SUPPORTED" == "true" ]]; then printf 'prediction_tripinfo_output=%s\n' "$PREDICTION_TRIPINFO_OUTPUT"; else printf 'prediction_tripinfo_output=unsupported\n'; fi
  printf 'host=%s\nworking_directory=%s\n' "$(hostname)" "$(pwd)"
} > "$RUN_DIR/metadata.txt"

set +e
"${CMD[@]}" 2>&1 | tee "$RUN_DIR/run.log"
exit_code=${PIPESTATUS[0]}
set -e
{
  printf 'exit_code=%s\n' "$exit_code"
  printf 'finished_at=%s\n' "$(date -Iseconds)"
} >> "$RUN_DIR/metadata.txt"
exit "$exit_code"
