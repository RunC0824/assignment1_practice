# Lab 2: Odometry vs. Ground Truth

## Problem and method
Evaluate wheel-integrated differential-drive odometry against Gazebo global pose, following the six Practical Guide steps (PDF pages 10–11).

1. Gazebo Harmonic / ROS 2 Jazzy installed runtime; the bundled differential-drive example is copied into world.sdf. Only the blue robot is retained. Physics step: 0.001 s. Odometry and PosePublisher rate: 50 Hz.
2. Native Gazebo streams: /model/vehicle_blue/odometry (gz.msgs.Odometry), /model/vehicle_blue/pose (gz.msgs.Pose_V; global model pose), /model/vehicle_blue/cmd_vel (gz.msgs.Twist).
3. Repeatable odometry-feedback path: 2 m forward, stop 1 s, left 90 degrees, stop 1 s, 1 m forward, then stop. Limits: 0.3 m/s and 0.5 rad/s. Position tolerance 0.003 m, angle tolerance 0.005 rad. Ground truth never enters the controller or odometry estimate.
4. Both streams are recorded at identical simulator timestamps. Raw odometry and global ground truth remain in the CSV. The axle midpoint is 0.5542825 m forward of the model origin (mean wheel-link X offsets). The estimate is converted to model origin using estimated heading and this fixed geometry only. A fixed initial rigid transform aligns the odometry frame to the world frame for evaluation; no continuous correction is applied.
5. e(t) = sqrt((x_odom_world-x_gt)^2 + (y_odom_world-y_gt)^2).
6. The comparison figure includes separate paths and an overlay; the error figure shows error accumulation.

## Measured results
- Paired samples: 1160
- Duration: 23.180 s
- Final Euclidean position error: 0.066625834 m
- Final heading error: 0.000072657 rad
- Position RMSE: 0.047499906 m
- Maximum position error: 0.066626248 m

## Analysis and limitations
Comparing different body reference points initially produced a 0.786239 m endpoint discrepancy. The turn rotates the 0.5542825 m lever arm; this is a coordinate-reference effect rather than accumulated localisation drift. The reported results correct this lever arm with model geometry and estimated heading.
The estimate integrates wheel motion with fixed wheel radius (0.3 m) and wheel separation (1.25 m). Ground truth comes independently from the simulated rigid-body pose. Wheel-ground slip, caster contact, contact solver discretisation, and small differences between assumed and effective wheel geometry can therefore produce disagreement. Turning is particularly sensitive to effective wheel separation and differential wheel slip. The world includes wheel friction parameters and slip1=0.035; the actual effect depends on the physics engine and cannot be inferred from the parameter alone. No artificial noise or calibration bias was injected in this practical. This run does not isolate individual causes, so the listed mechanisms are plausible explanations rather than measured causal contributions. Exact timestamp pairing removes reception-delay mismatch; initial frame alignment removes a fixed coordinate offset, not drift. Acceleration limits and feedback mean actual execution time differs from a simple distance/speed calculation.

## Reproduce
From this folder, source /opt/ros/jazzy/setup.bash. Start `gz sim -r world.sdf`, then run `python3 scripts/run_experiment.py`, followed by `MPLCONFIGDIR=/tmp/lab2-mpl python3 scripts/analyse.py`. Restart Gazebo with the original world before repeating the same global trajectory. Do not run the controller simultaneously twice.

Source: Intelligent Robotics lab 2.pdf, Practical Guide - Odometry vs. Ground Truth. This completes the practical exercise; the separate Assignment 1 requirements are not the same task.
