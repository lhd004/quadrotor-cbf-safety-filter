# Source Data Index

This file maps manuscript tables and figures to the processed data and scripts used to generate them.

## Manuscript Tables

| Manuscript item | Source file(s) | Notes |
|---|---|---|
| Table `tab:related_work_positioning` | `paper/main_elsarticle.tex` | Literature-positioning table based on cited papers. |
| Table `tab:batch_6dof` | `output/6dof_batch/summary_table.tex`, `output/6dof_batch/summary.csv` | Multi-scenario 6-DOF controller comparison, including APF and finite-horizon predictive safety-filter baselines. |
| Table `tab:robustness_checks` | `output/6dof_robustness/robustness_table.tex`, `output/6dof_robustness/robustness_summary.csv` | CBF-gain and actuation-limit robustness checks. |
| Table `tab:px4_reproducibility` | `paper/main_elsarticle.tex`, `scripts/run_px4_offboard_experiments.sh`, `scripts/run_px4_physical_wind_experiments.sh` | PX4/Gazebo reproducibility settings. |
| Table `tab:px4_multiscenario` | `output/px4_offboard_experiments/summary_table.tex`, `output/px4_offboard_experiments/summary.csv` | Single-run PX4/Gazebo scripted scenarios. |
| Table `tab:px4_repeatability` | `output/px4_offboard_repeated/repeatability_table.tex`, `output/px4_offboard_repeated/repeated_runs_summary.csv` | Three repeated PX4/Gazebo start-up runs for scripted scenarios. |
| Table `tab:px4_physical_wind` | `output/px4_physical_wind_experiments/repeatability_table.tex`, `output/px4_physical_wind_experiments/repeated_runs_summary.csv` | Three repeated PX4/Gazebo runs in the fixed physical-wind `windy` world. |

## Manuscript Figures

| Manuscript item | Source file(s) | Notes |
|---|---|---|
| Figure `fig:control_pipeline` | `output/paper_figures/control_pipeline.png` | Control architecture diagram. |
| Figure `fig:observer_diagnostic` | `output/6dof_batch/figures/disturbance_observer_diagnostic.png`, `output/6dof_batch/observer_summary.csv` | Disturbance-observer diagnostic. |
| Figure `fig:main_trajectory` | `output/6dof_batch/figures/main_trajectory_3d.png` | Nominal 6-DOF trajectory comparison. |
| Figure `fig:safety_by_scenario` | `output/6dof_batch/figures/safety_by_scenario.png` | 6-DOF safety metric by scenario. |
| Figure `fig:tracking_by_scenario` | `output/6dof_batch/figures/tracking_by_scenario.png` | 6-DOF mean tracking error by scenario. |
| Figure `fig:px4_trajectories` | `output/px4_offboard_experiments/multi_scenario_trajectories.png` | PX4/Gazebo scripted-scenario ground-plane trajectories. |
| Figure `fig:px4_multiscenario` | `output/px4_offboard_experiments/multi_scenario_safety.png` | PX4/Gazebo scripted-scenario barrier values. |
| Figure `fig:px4_physical_wind` | `output/px4_physical_wind_experiments/physical_wind_comparison.png` | PX4/Gazebo fixed physical-wind trajectory and safety comparison. |

## Main Scripts

| Purpose | Script |
|---|---|
| 6-DOF batch simulation | `scripts/run_6dof_batch_experiments.py` |
| 6-DOF robustness checks | `scripts/run_6dof_robustness_checks.py` |
| PX4/Gazebo scripted scenarios | `scripts/run_px4_offboard_experiments.sh` |
| PX4/Gazebo fixed physical-wind scenario | `scripts/run_px4_physical_wind_experiments.sh` |
