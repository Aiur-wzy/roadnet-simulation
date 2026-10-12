#!/usr/bin/env bash
# Serial training: strict road_flow < 5 receives raw weight 3, others weight 1.
# The trainer normalizes train weights to mean 1; validation remains unweighted.
set -Eeuo pipefail

usage() {
    printf '%s\n' \
        'Usage: bash scripts/training/train_sf_lowflow_six.sh [--dry-run | --help]' \
        'Order: gt_0p1, gt_3, gt_1, gt_0p5, gt_1p5, gt_2 (serial; stop on failure).' \
        'Defaults: 80 train / 20 validation files, 5400 s per fit, seed 42,' \
        '          CPU (0 GPUs), 10 bag folds, 2 stack levels, road_flow < 5 weight 3.' \
        'Environment overrides: DATASET, RUN_DIR, MODEL_ROOT, TRAIN_GPUS, PYTHON_BIN.' \
        '  DATASET: train_data/variants/new4x_sf under the repository.' \
        '  RUN_DIR: $DATASET/traci_runs/sf_threshold_sweep.' \
        '  MODEL_ROOT: models/sf_lowflow_lt5_x3_20260910 under the repository.' \
        '  TRAIN_GPUS: nonnegative integer (default 0); PYTHON_BIN: executable (default python).' \
        'Relative overrides are resolved from the repository root.' \
        'A real run requires a new MODEL_ROOT; existing output is never overwritten.' \
        '--dry-run invokes all six trainer validations, but creates no model output.'
}

die() { printf '[ERROR] %s\n' "$*" >&2; exit 2; }

DRY_RUN=0
for argument in "$@"; do
    case "$argument" in
        --dry-run) DRY_RUN=1 ;;
        --help) usage; exit 0 ;;
        *) die "unsupported argument: $argument (use --help)" ;;
    esac
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
cd -- "$REPO_ROOT"

TRAINER="${TRAINER:-$REPO_ROOT/legacy/training/model_training_sumo_sf_lowflow.py}"
TRAIN_MODULE="${TRAIN_MODULE:-legacy.training.model_training_sumo_sf_lowflow}"
DATASET="${DATASET:-$REPO_ROOT/train_data/variants/new4x_sf}"
RUN_DIR="${RUN_DIR:-$DATASET/traci_runs/sf_threshold_sweep}"
MODEL_ROOT="${MODEL_ROOT:-$REPO_ROOT/models/sf_lowflow_lt5_x3_20260910}"
TRAIN_GPUS="${TRAIN_GPUS:-0}"
PYTHON_BIN="${PYTHON_BIN:-python}"

[[ -f "$TRAINER" ]] || die "trainer not found: $TRAINER"
[[ "$TRAIN_GPUS" =~ ^[0-9]+$ ]] || die "TRAIN_GPUS must be a nonnegative integer"
command -v "$PYTHON_BIN" >/dev/null 2>&1 || die "Python executable not found: $PYTHON_BIN"
if [[ "$DRY_RUN" -eq 0 && ( -e "$MODEL_ROOT" || -L "$MODEL_ROOT" ) ]]; then
    die "MODEL_ROOT already exists; choose a new directory: $MODEL_ROOT"
fi

TRAIN_COMMAND=("$PYTHON_BIN")
if [[ "$TRAINER" == "$REPO_ROOT/legacy/training/model_training_sumo_sf_lowflow.py" ]]; then
    TRAIN_COMMAND+=(-m "$TRAIN_MODULE")
else
    TRAIN_COMMAND+=("$TRAINER")
fi

COMMON_ARGS=(
    --dataset-dir "$DATASET"
    --run-dir "$RUN_DIR"
    --expected-train-files 80
    --expected-validation-files 20
    --sample-weighting low-flow
    --low-flow-cutoff 5
    --low-flow-weight 3
    --time-limit 5400
    --random-state 42
    --num-gpus "$TRAIN_GPUS"
    --bag-folds 10
    --stack-levels 2
)
if [[ "$DRY_RUN" -eq 1 ]]; then
    COMMON_ARGS+=(--dry-run)
fi

for cutoff in 0p1 3 1 0p5 1p5 2; do
    printf '[RUN] gt_%s (road_flow < 5: raw weight 3; otherwise 1)\n' "$cutoff"
    if "${TRAIN_COMMAND[@]}" "${COMMON_ARGS[@]}" \
        --label-column "travel_time_label_gt_$cutoff" \
        --save-path "$MODEL_ROOT/gt_$cutoff"; then
        printf '[DONE] gt_%s\n' "$cutoff"
    else
        status=$?
        printf '[ERROR] gt_%s failed (exit %s); remaining thresholds were not started.\n' \
            "$cutoff" "$status" >&2
        exit "$status"
    fi
done

if [[ "$DRY_RUN" -eq 1 ]]; then
    printf '%s\n' '[COMPLETE] All six dry-run validations passed; no models were trained.'
else
    printf '[COMPLETE] All six fits completed under %s\n' "$MODEL_ROOT"
fi
