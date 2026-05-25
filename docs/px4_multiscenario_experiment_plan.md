# PX4/Gazebo Multi-Scenario Validation Plan

This plan upgrades the PX4/Gazebo evidence from a single smoke test to a small multi-scenario validation set. The claim remains bounded: these experiments support flight-stack implementability of the safety-filtering principle, not formal forward invariance of PX4 internal controllers.

## Implemented Scripts

- `scripts/px4_cbf_offboard.py`  
  Supports single or multiple circular obstacles using `--obstacles "x,y,r;x,y,r"`. It logs `scenario`, `controller`, and obstacle definitions into each CSV.

- `scripts/run_px4_offboard_experiments.sh`  
  Runs four scenarios with two controllers each:
  - `single_obstacle`
  - `large_obstacle`
  - `multi_obstacle`
  - `narrow_passage`

- `scripts/plot_px4_offboard_experiments.py`  
  Generates:
  - `output/px4_offboard_experiments/summary.csv`
  - `output/px4_offboard_experiments/summary_table.tex`
  - `output/px4_offboard_experiments/multi_scenario_safety.png`
  - per-scenario comparison figures

## Run Command

Run on the Ubuntu machine where PX4 SITL and Gazebo are installed:

```bash
cd /path/to/quadrotor-cbf-safety-filter
PROJECT_DIR=$PWD PX4_DIR=/path/to/PX4-Autopilot bash scripts/run_px4_offboard_experiments.sh
```

The runner starts one PX4 SITL instance per case and only stops the process IDs it started. This avoids broad process cleanup on a shared server.

## Expected Evidence

For each scenario, the desired pattern is:

- `baseline_no_cbf`: may enter the unsafe set, shown by negative `min_h`.
- `cbf_velocity_filter`: should keep `min_h > 0` or improve the minimum barrier value.

If a scenario fails because PX4 does not enter `OFFBOARD`, keep the controller log and SITL log. Do not silently delete failed runs, because failure modes are useful for deciding whether the experiment is too aggressive or the setpoint sequence needs adjustment.

## Manuscript Use

If the multi-scenario run succeeds, replace the current PX4 table in the manuscript with:

```latex
\input{../output/px4_offboard_experiments/summary_table.tex}
```

Then add `multi_scenario_safety.png` as the main PX4 evidence figure, while keeping the claim limited to flight-stack implementation evidence.
