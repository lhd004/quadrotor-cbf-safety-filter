# PX4/Gazebo Physical-Wind Experiment

World: `PX4_GZ_WORLD=windy`

The PX4 Gazebo `windy.sdf` world defines a world wind vector through the Gazebo
`<wind><linear_velocity>5 2 0</linear_velocity></wind>` element. The offboard
controller does not add the command-level `--wind-vx/--wind-vy` bias in this
experiment; any wind effect comes from the Gazebo world and PX4/Gazebo vehicle
simulation.

The obstacle is intentionally placed near the wind-deflected baseline path
(`x=2.7`, `y=1.55`, radius `0.85` m) so that the case tests safety
filtering under a physical Gazebo wind world rather than only free-flight
tracking in wind.

The experiment is still a velocity-level offboard safety-filter validation, not
a formal proof of the complete PX4 flight stack.
