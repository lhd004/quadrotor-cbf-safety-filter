# Source Data Index

This file maps manuscript tables and figures to the processed data and scripts used to generate them.

## Manuscript Tables

| Manuscript item | Source file(s) | Notes |
|---|---|---|
| Table `tab:related_work_positioning` | `paper/main_elsarticle.tex` | Literature-positioning table based on cited papers. |
| Table `tab:batch_6dof` | `output/6dof_batch/summary_table.tex`, `output/6dof_batch/summary.csv` | Multi-scenario 6-DOF controller comparison, including APF and finite-horizon predictive safety-filter baselines. |
| Table `tab:robustness_checks` | `output/6dof_robustness/robustness_table.tex`, `output/6dof_robustness/robustness_summary.csv` | CBF-gain and actuation-limit robustness checks. |
| Table `tab:monte_carlo` | `output/6dof_monte_carlo/monte_carlo_table.tex`, `output/6dof_monte_carlo/monte_carlo_summary.csv`, `output/6dof_monte_carlo/monte_carlo_detail.csv` | Thirty randomized 6-DOF stress trials over obstacle placement, wind-equivalent bias, initial-state perturbation, disturbance scale, and actuation limits. |
| Table `tab:px4_reproducibility` | `paper/main_elsarticle.tex`, `scripts/run_px4_offboard_experiments.sh`, `scripts/run_px4_physical_wind_experiments.sh` | PX4/Gazebo reproducibility settings. |
| Table `tab:px4_multiscenario` | `output/px4_offboard_experiments/summary_table.tex`, `output/px4_offboard_experiments/summary.csv` | Single-run PX4/Gazebo scripted scenarios. |
| Table `tab:px4_repeatability` | `output/px4_offboard_repeated/repeatability_table.tex`, `output/px4_offboard_repeated/repeated_runs_summary.csv` | Three repeated PX4/Gazebo start-up runs for scripted scenarios. |
| Table `tab:px4_physical_wind` | `output/px4_physical_wind_experiments/repeatability_table.tex`, `output/px4_physical_wind_experiments/repeated_runs_summary.csv` | Three repeated PX4/Gazebo runs in the fixed physical-wind `windy` world. |
| Preliminary PX4 FMU HITL smoke checks | `output/hitl/hitl_summary_2026_05_26.csv`, `output/hitl/hitl_baseline_no_cbf.csv`, `output/hitl/hitl_smoke_cbf_conservative.csv`, `output/hitl/hitl_cbf_velocity_filter.csv` | Real PX4 FMU v6X HITL smoke logs used only as hardware-link evidence, not as repeated statistical validation. |
| PX4 FMU HITL campaign diagnostics | `output/hitl_campaign/hitl_campaign_summary.csv`, `output/hitl_campaign/hitl_repaired_pair_summary.csv`, `output/hitl_campaign/events/*_events.csv`, `docs/hitl_campaign_report_2026_05_27.md` | Diagnostic campaign with per-run MAVLink events. Current runs exposed intermittent arming-health rejection, local-frame initialization transients, and post-arm attitude warnings, so they are not used as manuscript performance evidence. |
| Strict PX4 FMU HITL campaign attempt | `output/hitl_strict_n3/hitl_campaign_summary.csv`, `output/hitl_strict_n3/events/*_events.csv`, `docs/hitl_strict_campaign_report_2026_05_27.md` | Attempted three repeated baseline/CBF HITL pairs under strict acceptance criteria. The campaign did not meet manuscript-ready criteria because of startup telemetry dropouts, PX4 health warnings, one failsafe, and lack of a repeatable baseline violation. |
| Strict PX4 FMU HITL retry candidate | `output/hitl_strict_retry_n1/hitl_clean_pair_candidate_summary.csv`, `output/hitl_strict_retry_n1/events/*_events.csv`, `docs/hitl_strict_campaign_report_2026_05_27.md` | One post-reboot candidate pair produced the desired baseline-negative/CBF-positive numerical separation, but still failed strict manuscript-ready criteria because of magnetometer/attitude warnings and a failsafe event. |
| Strict PX4 FMU HITL reboot-before-case retry | `output/hitl_strict_retry_n1_reboot_each/hitl_campaign_summary.csv`, `output/hitl_strict_retry_n1_reboot_each/events/*_events.csv`, `docs/hitl_strict_campaign_report_2026_05_27.md` | Rebooting the FMU before each case improved run completion, but the CBF case still had a negative barrier and post-arm health/failsafe messages, so it is not manuscript-ready. |
| Redesigned buffered PX4 FMU HITL diagnostic campaign | `output/hitl_redesign_buffered_r013_n3/hitl_campaign_summary.csv`, `output/hitl_redesign_buffered_r013_n3/events/*_events.csv`, `docs/hitl_redesign_report_2026_05_27.md` | Three repeated pairs show the desired physical-radius barrier-sign contrast after adding a buffered velocity-level HITL safety filter: all baseline runs have negative `min_h`, and all CBF-filtered runs have positive `min_h`. Several logs still contain PX4 attitude/failsafe health warnings, so this remains supplementary diagnostic evidence rather than clean repeated hardware validation. |
| Low-altitude clean-HITL redesign attempts | `output/hitl_clean_attempt_alt04_*`, especially `output/hitl_clean_attempt_alt04_forwardobs_n1/hitl_campaign_summary.csv`, `output/hitl_clean_attempt_alt04_r085_d85_strongfilter_n1/hitl_campaign_summary.csv`, and `docs/hitl_redesign_report_2026_05_27.md` | Follow-up attempts with `ALTITUDE=0.4`, post-arm hold, and origin reset showed that clean short HITL pairs are possible, but no case achieved both a baseline violation and a positive CBF barrier margin under clean event logs. These outputs are negative/diagnostic evidence and should not be promoted to the manuscript performance tables. |

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
| Figure `fig:monte_carlo` | `output/6dof_monte_carlo/figures/monte_carlo_safety_distribution.png` | Randomized 6-DOF Monte Carlo safety-margin distribution and positive-barrier success rates. |

## Main Scripts

| Purpose | Script |
|---|---|
| 6-DOF batch simulation | `scripts/run_6dof_batch_experiments.py` |
| 6-DOF robustness checks | `scripts/run_6dof_robustness_checks.py` |
| PX4/Gazebo scripted scenarios | `scripts/run_px4_offboard_experiments.sh` |
| PX4/Gazebo fixed physical-wind scenario | `scripts/run_px4_physical_wind_experiments.sh` |
| PX4 FMU HITL offboard smoke test | `scripts/px4_cbf_offboard.py`, `docs/hitl_experiment_report_2026_05_26.md` |
| PX4 FMU HITL diagnostic campaign | `scripts/run_px4_hitl_campaign.sh`, `scripts/summarize_px4_hitl_campaign.py`, `docs/hitl_campaign_report_2026_05_27.md`, `docs/hitl_redesign_report_2026_05_27.md` |
| Randomized 6-DOF Monte Carlo stress test | `scripts/run_6dof_monte_carlo_experiments.py` |
