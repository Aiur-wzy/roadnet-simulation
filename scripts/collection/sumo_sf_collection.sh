#!/usr/bin/env bash
# Invoked by run_all_training_traci.sh's per-run SUMO wrapper. Keep statistics
# next to the run-specific tripinfo rather than overwriting materialization QA.
set -Eeuo pipefail

die() { echo "[ERROR] $*" >&2; exit 1; }
SF_REAL_SUMO_BINARY="${SF_REAL_SUMO_BINARY:-sumo}"
actual_sumo="$(command -v "$SF_REAL_SUMO_BINARY")" || die "SUMO executable not found: $SF_REAL_SUMO_BINARY"
[[ "$(realpath -m "$actual_sumo")" != "$(realpath -m "${BASH_SOURCE[0]}")" ]] \
    || die "refusing recursive SF SUMO wrapper invocation"

if [[ -z "${TRACI_TRIPINFO_OUTPUT:-}" ]]; then
    # Only standalone information queries may run without an output context.
    if [[ "$#" -eq 1 ]]; then
        case "$1" in --help|-h|--version|-V) exec "$actual_sumo" "$@" ;; esac
    fi
    die "TRACI_TRIPINFO_OUTPUT is required for simulation; run via run_new4x_sf_traci.sh"
fi

[[ "$TRACI_TRIPINFO_OUTPUT" == *.xml ]] || die "run tripinfo path must end with .xml"
statistics_output="${TRACI_TRIPINFO_OUTPUT%.xml}.statistics.xml"
declare -a forwarded=()
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --statistic-output)
            [[ "$#" -ge 2 ]] || die "--statistic-output requires a value"
            shift 2 ;;
        --statistic-output=*) shift ;;
        *) forwarded+=("$1"); shift ;;
    esac
done
# SUMO command-line options override config-file values. Strip any prior CLI
# statistic-output so there is exactly one controlled value, not duplicates.
exec "$actual_sumo" "${forwarded[@]}" --statistic-output "$statistics_output"
