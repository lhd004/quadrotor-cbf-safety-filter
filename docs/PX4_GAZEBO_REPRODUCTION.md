# PX4/Gazebo Reproduction Notes

The PX4/Gazebo validation uses PX4 SITL with the `gz_x500` vehicle and a companion Python process that sends MAVLink local-NED velocity and altitude setpoints.

## Environment

Recommended:

- Ubuntu 22.04 or 24.04
- PX4-Autopilot
- Gazebo supported by the PX4 setup script
- Python 3.10 or later
- `pymavlink`, `numpy`, and `matplotlib`

Install Python requirements:

```bash
python3 -m pip install -r requirements.txt
```

Follow the official PX4 Ubuntu setup instructions for PX4-Autopilot and Gazebo.

## Single PX4 SITL Run

Terminal 1:

```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
```

Terminal 2:

```bash
python3 scripts/px4_cbf_offboard.py \
  --duration 18 \
  --scenario single_obstacle \
  --controller cbf_velocity_filter \
  --obstacles "3.0,0.0,1.15" \
  --output data/px4_offboard_experiments/single_obstacle_cbf_velocity_filter.csv
```

## Batch PX4/Gazebo Runs

Set `PX4_DIR` to the PX4-Autopilot checkout and run:

```bash
PX4_DIR=~/PX4-Autopilot PROJECT_DIR=$PWD bash scripts/run_px4_offboard_experiments.sh
```

The runner starts one SITL process per case and only stops process IDs that it created.

## Claim Boundary

These experiments validate whether a velocity-level CBF safety-filtering principle can be executed through a PX4 offboard interface. They do not prove forward invariance for the full PX4 closed-loop flight stack. The wind-bias case is a command-level velocity-bias stress test, not a physical Gazebo wind-field simulation.

