# Quadrotor CBF Safety Filter

This repository contains the code, processed data, and figure source files for a quadrotor safety-control study on disturbance-observer-assisted control barrier function (CBF) filtering.

The repository supports three validation layers:

1. A second-order translational CBF-QP safety-filter demo.
2. A full 6-DOF quadrotor simulation with disturbance observer, robust tracking, thrust and attitude realization, and ablation studies.
3. CBF-gain and actuation-limit robustness checks for the full 6-DOF controller.
4. PX4 SITL + Gazebo offboard validation of a velocity-level CBF safety-filtering principle, including repeated start-up runs and a fixed physical-wind Gazebo stress test.
5. Supplementary PX4 FMU HITL diagnostic logs for real flight-controller communication and buffered velocity-level safety filtering.

The PX4/Gazebo experiment is included as practical flight-stack evidence. It should not be interpreted as a formal proof of forward invariance for PX4 internal controllers.

## Repository Layout

```text
scripts/
  cbf_quadrotor_safety_demo.py          Second-order CBF-QP demo
  cbf_quadrotor_6dof_demo.py            Full 6-DOF quadrotor simulation
  run_6dof_batch_experiments.py         Batch ablation and diagnostics
  run_6dof_robustness_experiments.py    CBF-gain and actuation-limit robustness checks
  px4_cbf_offboard.py                   PX4 MAVLink offboard CBF filter
  run_px4_offboard_experiments.sh       PX4/Gazebo batch runner
  run_px4_physical_wind_experiments.sh  PX4/Gazebo fixed-wind stress-test runner
  run_px4_repeated_experiments.sh       PX4/Gazebo repeated start-up runner
  run_px4_single_case.sh                PX4/Gazebo single-case rerun helper
  plot_px4_offboard_experiments.py      PX4 log summarization and figures
  summarize_px4_repeated_experiments.py PX4 repeated-run summary tables
  plot_control_pipeline.py              Control-pipeline figure generation

data/
  6dof_batch/                           Processed 6-DOF simulation summaries
  px4_offboard_experiments/             Processed PX4/Gazebo offboard logs
  px4_offboard_repeated/                Repeated PX4/Gazebo start-up logs
  px4_physical_wind_experiments/        Fixed physical-wind PX4/Gazebo logs
  hitl_redesign_buffered_r013_n3/        PX4 FMU HITL diagnostic campaign logs
  hitl_clean_attempts/                   Low-altitude clean-HITL attempt summaries

figures/                                Manuscript figures in PNG/PDF/SVG
docs/                                   Reproduction notes and claim-evidence map
```

## Python Environment

Use Python 3.10 or later.

```bash
python -m pip install -r requirements.txt
```

For the 6-DOF batch:

```bash
python scripts/run_6dof_batch_experiments.py
```

Expected outputs are written to `output/6dof_batch/` when the script is run from the repository root.

For the PX4/Gazebo plotting step:

```bash
python scripts/plot_px4_offboard_experiments.py \
  data/px4_offboard_experiments/*_baseline_no_cbf.csv \
  data/px4_offboard_experiments/*_cbf_velocity_filter.csv \
  --output-dir output/px4_offboard_experiments
```

## PX4/Gazebo Requirement

The PX4 offboard experiments require a separate Ubuntu environment with PX4-Autopilot, Gazebo, and `pymavlink`. See `docs/PX4_GAZEBO_REPRODUCTION.md`.

The batch runner restarts PX4 SITL for each scenario and only stops process IDs started by that scenario. For parameter checks or failed-case reruns, use `scripts/run_px4_single_case.sh` with the same PX4/Gazebo environment.

## Key Processed Data

- `data/6dof_batch/summary.csv`: 6-DOF ablation metrics.
- `data/6dof_batch/observer_summary.csv`: disturbance-observer error metrics.
- `data/6dof_batch/safety_filter_diagnostics.csv`: CBF-QP success rate, active-filter rate, correction magnitude, and slack diagnostics.
- `data/6dof_robustness/robustness_summary.csv`: CBF-gain sensitivity and actuation-limit stress metrics.
- `data/px4_offboard_experiments/summary.csv`: PX4/Gazebo multi-scenario summary.
- `data/px4_offboard_repeated/repeated_runs_summary.csv`: PX4/Gazebo repeated start-up summary over three runs.
- `data/px4_physical_wind_experiments/repeated_runs_summary.csv`: PX4/Gazebo fixed physical-wind summary over three start-up runs.
- `data/hitl_redesign_buffered_r013_n3/hitl_campaign_summary.csv`: PX4 FMU HITL diagnostic summary with a repeated baseline-negative/CBF-positive barrier-sign contrast.
- `data/hitl_clean_attempts/*.csv`: follow-up low-altitude HITL attempt summaries showing why these runs are treated as diagnostic rather than clean repeated validation.

## Main Reproducibility Boundary

The 6-DOF simulations reproduce the model-level controller and diagnostics. The PX4/Gazebo files reproduce a velocity-level offboard safety-filter validation, including command-level wind-bias and fixed physical-wind Gazebo cases. The fixed-wind case is a targeted stress test in the PX4 `windy` world, not a turbulent-wind campaign. The HITL files document real PX4 FMU communication and diagnostic barrier-sign behavior, but they are not clean repeated hardware validation and do not constitute real flight or motor/propeller actuator tests.
