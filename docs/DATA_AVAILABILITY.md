# Data and Code Availability

## Ready-to-paste statement before DOI assignment

The code and processed data supporting the simulation and PX4/Gazebo validation results are deposited in a public repository at `https://github.com/lhd004/quadrotor-cbf-safety-filter` and archived at `[Zenodo DOI]`. The repository includes the 6-DOF simulation scripts, batch experiment runner, PX4 offboard validation scripts, processed CSV logs, generated summary tables, and figure source files. The PX4/Gazebo data correspond to deterministic SITL runs using a velocity-level offboard safety filter and do not include physical wind-field or hardware experiments.

## Dataset-to-file map

| Result supported | Files |
|---|---|
| 6-DOF ablation table | `data/6dof_batch/summary.csv`, `data/6dof_batch/summary_table.tex` |
| Disturbance-observer diagnostic | `data/6dof_batch/observer_summary.csv`, `figures/disturbance_observer_diagnostic.*` |
| CBF-QP feasibility and active-filter diagnostics | `data/6dof_batch/safety_filter_diagnostics.csv` |
| PX4/Gazebo multi-scenario validation | `data/px4_offboard_experiments/summary.csv`, per-run CSV files, `figures/multi_scenario_*.{png,pdf,svg}` |

## Repository actions before submission

1. Replace author placeholders in `CITATION.cff`.
2. Public GitHub repository: completed at `https://github.com/lhd004/quadrotor-cbf-safety-filter`.
3. Versioned GitHub release: `v1.0.0`.
4. Link the GitHub repository to Zenodo and create a DOI for `v1.0.0`.
5. Replace `[Zenodo DOI]` in this file and in the manuscript.
6. Confirm whether the repository should include the manuscript source after journal submission rules are checked.
