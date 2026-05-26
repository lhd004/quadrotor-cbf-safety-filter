#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${PROJECT_DIR:-$HOME/Thesis_2}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
OUT_DIR="$PROJECT_DIR/output/px4_physical_wind_experiments"
LOG_DIR="$OUT_DIR/logs"
REPEATS="${REPEATS:-3}"
PX4_GZ_WORLD_NAME="${PX4_GZ_WORLD_NAME:-windy}"

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
        PX4_GZ_WORLD="$PX4_GZ_WORLD_NAME" HEADLESS=1 make px4_sitl gz_x500 \
            > "$LOG_DIR/${case_name}_sitl.log" 2>&1
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
    local controller="$2"
    shift 2
    echo "=== Running ${case_name} in PX4_GZ_WORLD=${PX4_GZ_WORLD_NAME} ==="
    start_sitl "$case_name"
    set +e
    (
        cd "$PROJECT_DIR"
        timeout 60 python3 scripts/px4_cbf_offboard.py \
            --duration 20 \
            --scenario "physical_wind" \
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

for repeat in $(seq 1 "$REPEATS"); do
    prefix=$(printf "r%02d" "$repeat")
    echo "### Physical-wind repeat ${repeat}/${REPEATS} ###"

    run_case "${prefix}_physical_wind_baseline_no_cbf" "baseline_no_cbf" \
        --disable-cbf --goal-y 2.0 --speed 0.35 \
        --obstacle-x 2.7 --obstacle-y 1.55 --safe-radius 0.85 \
        --alpha 0.7 --margin 0.05

    run_case "${prefix}_physical_wind_cbf_velocity_filter" "cbf_velocity_filter" \
        --goal-y 2.0 --speed 0.35 --max-speed 0.85 \
        --obstacle-x 2.7 --obstacle-y 1.55 --safe-radius 0.85 \
        --alpha 0.45 --margin 0.16
done

python3 "$PROJECT_DIR/scripts/summarize_px4_repeated_experiments.py" \
    "$OUT_DIR"/r*_*.csv \
    --output-dir "$OUT_DIR"

python3 "$PROJECT_DIR/scripts/plot_px4_offboard_experiments.py" \
    "$OUT_DIR"/*_baseline_no_cbf.csv \
    "$OUT_DIR"/*_cbf_velocity_filter.csv \
    --output-dir "$OUT_DIR"

cat > "$OUT_DIR/experiment_note.md" <<EOF
# PX4/Gazebo Physical-Wind Experiment

World: \`PX4_GZ_WORLD=${PX4_GZ_WORLD_NAME}\`

The PX4 Gazebo \`windy.sdf\` world defines a world wind vector through the Gazebo
\`<wind><linear_velocity>5 2 0</linear_velocity></wind>\` element. The offboard
controller does not add the command-level \`--wind-vx/--wind-vy\` bias in this
experiment; any wind effect comes from the Gazebo world and PX4/Gazebo vehicle
simulation.

The obstacle is intentionally placed near the wind-deflected baseline path
(\`x=2.7\`, \`y=1.55\`, radius \`0.85\` m) so that the case tests safety
filtering under a physical Gazebo wind world rather than only free-flight
tracking in wind.

The experiment is still a velocity-level offboard safety-filter validation, not
a formal proof of the complete PX4 flight stack.
EOF

echo "Physical-wind PX4/Gazebo results written to $OUT_DIR"
