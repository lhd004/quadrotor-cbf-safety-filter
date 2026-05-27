# PX4 HITL Scenario Redesign Report

Date: 2026-05-27

## Purpose

The original HITL scenarios were not suitable as manuscript-grade evidence because the baseline did not reliably violate the safety boundary and the CBF case did not reliably keep a positive barrier margin. The redesign therefore used the observed HITL trajectories to create a repeatable obstacle-crossing task and modified the safety filter to handle the velocity-level PX4/HITL dynamics more robustly.

## Controller changes

Two optional arguments were added to `scripts/px4_cbf_offboard.py` and exposed through `scripts/run_px4_hitl_campaign.sh`.

- `--tangent-gain`: adds a tangential bypass velocity only when the CBF constraint is active.
- `--filter-radius-buffer`: uses an enlarged radius for the CBF filtering calculation while logging the physical barrier value with the original obstacle radius.

Both defaults are zero, so existing non-HITL and earlier HITL runs are unchanged unless these options are explicitly enabled.

## Best redesigned scenario

The strongest candidate so far is:

```bash
GOAL_X=0.0
GOAL_Y=1.0
OBSTACLE_X=0.0
OBSTACLE_Y=0.30
SAFE_RADIUS=0.13
CBF_ALPHA=5.0
CBF_MARGIN=0.25
TANGENT_GAIN=0.12
FILTER_RADIUS_BUFFER=0.12
SPEED=0.15
MAX_SPEED=0.45
RUN_DURATION=20
SETTLE_WINDOW=15
```

Remote output:

```text
~/Thesis_2/output/hitl_redesign_buffered_r013_n3/
```

## Numerical result

| Case | min h values | Mean min h | Interpretation |
| --- | ---: | ---: | --- |
| Baseline, n=3 | -0.01245, -0.01326, -0.01213 | -0.01262 | Repeatable physical-radius violation |
| CBF, n=3 | 0.07326, 0.07068, 0.06444 | 0.06946 | Repeatable positive physical-radius margin |

This is the first repeated HITL campaign that produces the desired safety-barrier contrast: all baseline runs violate the physical safety set and all CBF-filtered runs preserve a positive physical barrier margin.

## Remaining limitation

The campaign is still not fully manuscript-clean under the strict health criterion. PX4 reported `Preflight Fail: Attitude failure (roll)` and `Failsafe activated` in several event logs. These warnings occurred despite complete trajectory CSVs and accepted OFFBOARD/arming commands. Therefore the result should not yet be described as a clean repeated hardware-in-the-loop validation.

Recommended wording if used now:

> A repeated PX4 FMU HITL diagnostic campaign showed the expected barrier-sign contrast under a buffered velocity-level safety filter, but post-run PX4 health warnings remained. We therefore treat this result as supplementary diagnostic evidence rather than as primary flight-stack validation.

## Decision

Do not promote this HITL result to the main manuscript table yet. It is useful as a supplementary diagnostic result and as the current best scenario design. For AST submission, the main evidence should remain PX4/Gazebo/SITL plus simulation baselines unless the HITL health warnings are eliminated in a follow-up run.

## Follow-up clean-HITL attempt

After the buffered `r=0.13 m` campaign, a lower-altitude clean-HITL route was tested to reduce the early `Attitude failure (roll)` and `Failsafe activated` messages. The controller script was extended with two optional run-control arguments:

- `--post-arm-hold`: holds the takeoff/hover setpoint after arming before starting the logged horizontal task.
- `--reset-origin-after-hold`: resets the relative horizontal task origin after the post-arm hold, while preserving the original altitude target.

The cleanest low-altitude smoke pair used:

```bash
ALTITUDE=0.4
POST_ARM_HOLD=6
RESET_ORIGIN_AFTER_HOLD=1
RUN_DURATION=8.5
SPEED=0.12
MAX_SPEED=0.30
```

Key follow-up outputs:

| Output directory | Health state | Baseline min h | CBF min h | Interpretation |
| --- | --- | ---: | ---: | --- |
| `output/hitl_clean_attempt_alt04_hold6_n1/` | clean | `0.007614` | `0.012814` | Clean low-altitude HITL pair, but baseline did not violate the barrier. |
| `output/hitl_clean_attempt_alt04_resetorigin_r011_n1/` | clean | `-0.002742` | `-0.011547` | Clean and baseline-negative, but CBF also violated the barrier. |
| `output/hitl_clean_attempt_alt04_centered_r008_d85_n1/` | clean | `0.001536` | `0.016847` | Clean and CBF-positive, but baseline remained positive. |
| `output/hitl_clean_attempt_alt04_centered_r009_d85_n1/` | clean | `-0.006171` | `-0.003583` | Clean and baseline-negative, but CBF also violated the barrier. |
| `output/hitl_clean_attempt_alt04_longpath_n1/` | attitude warning | `0.109771` | `0.008095` | Longer path did not create a baseline violation and reintroduced PX4 attitude warnings. |
| `output/hitl_clean_attempt_alt04_r085_d85_strongfilter_n1/` | mixed | `-0.001074` | `-0.004421` | Baseline violated, but the CBF case also violated; stronger filtering generated large corrections and did not remain tracking-feasible. |
| `output/hitl_clean_attempt_alt04_forwardobs_n1/` | clean | `0.037021` | `0.045586` | Forward obstacle placement produced a clean pair, but the task was too conservative and baseline did not violate. |

Conclusion: the low-altitude route removes the major PX4 health/failsafe issue when the task is short and conservative, but it has not yet produced a clean barrier-sign contrast. Near-origin obstacles can make the baseline negative, but the CBF-filtered case also enters the physical safety radius because the velocity-level command is applied through PX4 closed-loop dynamics with finite tracking response. Moving the obstacle farther downstream improves cleanliness but makes the baseline too conservative. Therefore, a repeated clean `baseline min h < 0` and `CBF min h > 0` HITL campaign has not been achieved.

The best currently publishable HITL statement remains the buffered repeated diagnostic result from `output/hitl_redesign_buffered_r013_n3/`, with the explicit caveat that PX4 health/failsafe messages remain in several logs.

Recommended next clean-HITL step: stop tuning near-origin micro-obstacles. A manuscript-grade clean HITL route likely needs either (i) a longer, more trackable horizontal segment with a stable simulator-side obstacle farther from takeoff, or (ii) a position/trajectory-level avoidance implementation that gives PX4 enough preview distance instead of relying on a late velocity-level correction.
