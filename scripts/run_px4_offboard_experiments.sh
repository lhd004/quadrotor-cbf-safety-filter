#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$HOME/Thesis_2}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
OUT_DIR="$PROJECT_DIR/output/px4_offboard_experiments"
LOG_DIR="$OUT_DIR/logs"

mkdir -p "$OUT_DIR" "$LOG_DIR"

pid_pattern='PX4-Autopilot|px4_sitl|build/px4_sitl_default/bin/px4|gz sim'
started_pids_file=""

snapshot_pids() {
    pgrep -af "$pid_pattern" | awk '{print $1}' | sort -n || true
}

start_sitl() {
    local case_name="$1"
    local before_file="$LOG_DIR/${case_name}_pids_before.txt"
    local after_file="$LOG_DIR/${case_name}_pids_after.txt"
    started_pids_file="$LOG_DIR/${case_name}_pids_started.txt"

    snapshot_pids > "$before_file"
    (
        cd "$PX4_DIR"
        HEADLESS=1 make px4_sitl gz_x500 > "$LOG_DIR/${case_name}_sitl.log" 2>&1
    ) &

    sleep 12
    snapshot_pids > "$after_file"
    comm -13 "$before_file" "$after_file" > "$started_pids_file" || true
}

stop_sitl() {
    if [[ -n "${started_pids_file:-}" && -f "$started_pids_file" ]]; then
        tac "$started_pids_file" | xargs -r kill 2>/dev/null || true
        sleep 3
        tac "$started_pids_file" | xargs -r kill -9 2>/dev/null || true
    fi
}

run_case() {
    local case_name="$1"
    local scenario="$2"
    local controller="$3"
    shift
    shift
    shift
    echo "=== Running ${case_name} ==="
    start_sitl "$case_name"
    set +e
    (
        cd "$PROJECT_DIR"
        timeout 55 python3 scripts/px4_cbf_offboard.py \
            --duration 18 \
            --scenario "$scenario" \
            --controller "$controller" \
            --output "$OUT_DIR/${case_name}.csv" \
            "$@"
    ) | tee "$LOG_DIR/${case_name}_controller.log"
    local result=${PIPESTATUS[0]}
    set -e
    stop_sitl
    if [[ "$result" -ne 0 ]]; then
        echo "Case ${case_name} failed with exit code ${result}" >&2
        return "$result"
    fi
}

trap stop_sitl EXIT

run_case "single_obstacle_baseline_no_cbf" "single_obstacle" "baseline_no_cbf" \
    --disable-cbf --goal-y 2.0 --speed 0.35 --safe-radius 1.15 --alpha 0.7 --margin 0.05
run_case "single_obstacle_cbf_velocity_filter" "single_obstacle" "cbf_velocity_filter" \
    --goal-y 2.0 --speed 0.35 --safe-radius 1.15 --alpha 0.7 --margin 0.05

run_case "large_obstacle_baseline_no_cbf" "large_obstacle" "baseline_no_cbf" \
    --disable-cbf --goal-y 2.0 --speed 0.35 --safe-radius 1.35 --alpha 0.7 --margin 0.05
run_case "large_obstacle_cbf_velocity_filter" "large_obstacle" "cbf_velocity_filter" \
    --goal-y 2.0 --speed 0.35 --max-speed 0.55 --safe-radius 1.35 --alpha 0.9 --margin 0.08

run_case "multi_obstacle_baseline_no_cbf" "multi_obstacle" "baseline_no_cbf" \
    --disable-cbf --goal-y 1.4 --speed 0.35 --obstacles "2.6,-0.45,0.80;3.7,0.55,0.80" --alpha 0.7 --margin 0.05
run_case "multi_obstacle_cbf_velocity_filter" "multi_obstacle" "cbf_velocity_filter" \
    --goal-y 1.4 --speed 0.35 --obstacles "2.6,-0.45,0.80;3.7,0.55,0.80" --alpha 0.7 --margin 0.05

run_case "narrow_passage_baseline_no_cbf" "narrow_passage" "baseline_no_cbf" \
    --disable-cbf --goal-y 0.0 --speed 0.32 --obstacles "3.0,-0.95,0.72;3.0,0.95,0.72" --alpha 0.65 --margin 0.04
run_case "narrow_passage_cbf_velocity_filter" "narrow_passage" "cbf_velocity_filter" \
    --goal-y 0.0 --speed 0.32 --obstacles "3.0,-0.95,0.72;3.0,0.95,0.72" --alpha 0.65 --margin 0.04

run_case "wind_bias_baseline_no_cbf" "wind_bias" "baseline_no_cbf" \
    --disable-cbf --goal-y 2.0 --speed 0.35 --safe-radius 1.15 --alpha 0.7 --margin 0.05 --wind-vx 0.10 --wind-vy -0.18
run_case "wind_bias_cbf_velocity_filter" "wind_bias" "cbf_velocity_filter" \
    --goal-y 2.0 --speed 0.35 --max-speed 0.85 --safe-radius 1.15 --alpha 0.35 --margin 0.14 --wind-vx 0.10 --wind-vy -0.18

python3 "$PROJECT_DIR/scripts/plot_px4_offboard_experiments.py" \
    "$OUT_DIR"/*_baseline_no_cbf.csv \
    "$OUT_DIR"/*_cbf_velocity_filter.csv \
    --output-dir "$OUT_DIR"

echo "Results written to $OUT_DIR"
