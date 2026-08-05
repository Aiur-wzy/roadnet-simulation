#!/usr/bin/env bash
set -Eeuo pipefail

DATASET_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_NET_REL=../data/test_new.net.xml
DEFAULT_TRACI_REL=../TraCI_Python_Adjusted.py
TRACI_SCRIPT="${TRACI_SCRIPT:-$DATASET_DIR/$DEFAULT_TRACI_REL}"
SUMO_BINARY="${SUMO_BINARY:-sumo}"
TRACI_OUTPUTS="${TRACI_OUTPUTS:-legacy}"
NET_FILE="${NET_FILE:-$DATASET_DIR/$DEFAULT_NET_REL}"
JOB_FILE="$DATASET_DIR/commands/traci_jobs.tsv"

[[ -f "$NET_FILE" ]] || { echo "Network file not found: $NET_FILE" >&2; exit 1; }
[[ -f "$TRACI_SCRIPT" ]] || { echo "TraCI script not found: $TRACI_SCRIPT" >&2; exit 1; }
[[ -f "$JOB_FILE" ]] || { echo "TraCI job manifest not found: $JOB_FILE" >&2; exit 1; }

tail -n +2 "$JOB_FILE" | while IFS=$'\t' read -r episode_id split sumocfg legacy full events cycles log_file; do
    log_file="${log_file%$'\r'}"
    sumocfg_path="$DATASET_DIR/$sumocfg"
    [[ -f "$sumocfg_path" ]] || { echo "SUMO config not found: $sumocfg_path" >&2; exit 1; }
    mkdir -p \
        "$(dirname "$DATASET_DIR/$legacy")" \
        "$(dirname "$DATASET_DIR/$full")" \
        "$(dirname "$DATASET_DIR/$events")" \
        "$(dirname "$DATASET_DIR/$cycles")" \
        "$(dirname "$DATASET_DIR/$log_file")"
    echo "[RUN] $episode_id demand-data split=$split"
    python3 "$TRACI_SCRIPT" \
        --sumo-config "$sumocfg_path" \
        --net-file "$NET_FILE" \
        --sumo-binary "$SUMO_BINARY" \
        --outputs "$TRACI_OUTPUTS" \
        --legacy-edge-output "$DATASET_DIR/$legacy" \
        --training-output "$DATASET_DIR/$full" \
        --vehicle-event-output "$DATASET_DIR/$events" \
        --cycle-summary-output "$DATASET_DIR/$cycles" \
        2>&1 | tee "$DATASET_DIR/$log_file"
done
