#!/usr/bin/env bash
set -euo pipefail

DEFAULT_OUTPUT_ROOT="eval_results/single_group"
DEFAULT_TRAVEL_TIME_MODE="speed-net"
DEFAULT_LANE_DISCHARGE_INTERVAL="1"
ALL_GROUPS=(free_all light_all_seed1 light_all_seed2 medium_core heavy_all oversat_all west_bottleneck north_bottleneck east_bottleneck south_bottleneck southwest_bottleneck)

usage() {
  cat <<'USAGE'
Usage: scripts/run_all_single_group_cams_evals.sh [options]

Run CAMS single-group evaluations for all or selected traffic groups.

Options:
  --run-id <shared_run_id>          Shared run id (default: one YYYYMMDD_HHMMSS for all groups).
  --output-root <path>              Output root (default: eval_results/single_group).
  --groups <comma-separated groups> Groups to run (default: all actual groups).
  --dry-run                         Print commands without running CAMS.
  --continue-on-error               Continue remaining groups after a failure.
  --travel-time-mode <mode>         Travel-time mode (default: speed-net).
  --lane-discharge-interval <int>   Lane discharge interval (default: 1).
  --help                            Show this help.
USAGE
}

RUN_ID=""
OUTPUT_ROOT="$DEFAULT_OUTPUT_ROOT"
GROUPS_CSV=""
DRY_RUN="false"
CONTINUE_ON_ERROR="false"
TRAVEL_TIME_MODE="$DEFAULT_TRAVEL_TIME_MODE"
LANE_DISCHARGE_INTERVAL="$DEFAULT_LANE_DISCHARGE_INTERVAL"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --run-id) RUN_ID="${2:-}"; shift 2 ;;
    --output-root) OUTPUT_ROOT="${2:-}"; shift 2 ;;
    --groups) GROUPS_CSV="${2:-}"; shift 2 ;;
    --dry-run) DRY_RUN="true"; shift ;;
    --continue-on-error) CONTINUE_ON_ERROR="true"; shift ;;
    --travel-time-mode) TRAVEL_TIME_MODE="${2:-}"; shift 2 ;;
    --lane-discharge-interval) LANE_DISCHARGE_INTERVAL="${2:-}"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) printf 'ERROR: unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "$RUN_ID" ]]; then
  RUN_ID="$(date +%Y%m%d_%H%M%S)"
fi
if [[ -n "$GROUPS_CSV" ]]; then
  IFS=',' read -r -a GROUPS <<< "$GROUPS_CSV"
else
  GROUPS=("${ALL_GROUPS[@]}")
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SINGLE_SCRIPT="$SCRIPT_DIR/run_single_group_cams_eval.sh"
successes=()
failures=()

for group in "${GROUPS[@]}"; do
  printf '\n=== Running group: %s (run_id=%s) ===\n' "$group" "$RUN_ID"
  cmd=("$SINGLE_SCRIPT" --group "$group" --run-id "$RUN_ID" --output-root "$OUTPUT_ROOT" --travel-time-mode "$TRAVEL_TIME_MODE" --lane-discharge-interval "$LANE_DISCHARGE_INTERVAL")
  if [[ "$DRY_RUN" == "true" ]]; then cmd+=(--dry-run); fi
  if "${cmd[@]}"; then
    successes+=("$group")
  else
    failures+=("$group")
    if [[ "$CONTINUE_ON_ERROR" != "true" ]]; then
      printf 'ERROR: group failed: %s\n' "$group" >&2
      exit 1
    fi
  fi
done

printf '\nSuccessful groups: %s\n' "${successes[*]:-none}"
printf 'Failed groups: %s\n' "${failures[*]:-none}"
if [[ ${#failures[@]} -gt 0 ]]; then exit 1; fi
