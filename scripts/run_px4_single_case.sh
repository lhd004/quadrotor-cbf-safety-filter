#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -lt 3 ]]; then
    echo "Usage: $0 CASE_NAME SCENARIO CONTROLLER [px4_cbf_offboard.py args...]" >&2
    exit 2
fi

PROJECT_DIR="${PROJECT_DIR:-$HOME/Thesis_2}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
OUT_DIR="$PROJECT_DIR/output/px4_offboard_experiments"
LOG_DIR="$OUT_DIR/logs"

case_name="$1"
scenario="$2"
controller="$3"
shift 3

mkdir -p "$OUT_DIR" "$LOG_DIR"

pid_pattern='PX4-Autopilot|px4_sitl|build/px4_sitl_default/bin/px4|gz sim'
started_pids_file="$LOG_DIR/${case_name}_single_pids_started.txt"

snapshot_pids() {
    pgrep -af "$pid_pattern" | awk '{print $1}' | sort -n || true
}

stop_sitl() {
    if [[ -f "$started_pids_file" ]]; then
        tac "$started_pids_file" | xargs -r kill 2>/dev/null || true
        sleep 3
        tac "$started_pids_file" | xargs -r kill -9 2>/dev/null || true
    fi
}

trap stop_sitl EXIT

before_file="$LOG_DIR/${case_name}_single_pids_before.txt"
after_file="$LOG_DIR/${case_name}_single_pids_after.txt"

snapshot_pids > "$before_file"
(
    cd "$PX4_DIR"
    HEADLESS=1 make px4_sitl gz_x500 > "$LOG_DIR/${case_name}_single_sitl.log" 2>&1
) &

sleep 12
snapshot_pids > "$after_file"
comm -13 "$before_file" "$after_file" > "$started_pids_file" || true

(
    cd "$PROJECT_DIR"
    timeout 55 python3 scripts/px4_cbf_offboard.py \
        --duration 18 \
        --scenario "$scenario" \
        --controller "$controller" \
        --output "$OUT_DIR/${case_name}.csv" \
        "$@"
) | tee "$LOG_DIR/${case_name}_single_controller.log"

