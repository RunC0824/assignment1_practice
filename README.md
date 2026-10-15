
An indoor Gazebo scene, a differential-drive robot, timed open-loop motion,
and separate timestamped wheel-joint and physical ground-truth recordings.
Localisation estimation, measurement noise modelling and the assessed report
will be implemented in later stages. Stage 1 recordings are interface validation,
not the final localisation experiment.

Target environment: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.15.0.

## Build and run

```bash
cd ~/IRA1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
./scripts/start_sim.sh
```

In a second terminal, start the recorder and square command sequence:

```bash
cd ~/IRA1
./scripts/run_demo.sh
```

The robot is commanded to travel four 2 m straight sections at 0.25 m/s,
with nominal 90 degree turns at 0.5 rad/s and stops between sections.
These are open-loop commands, not guarantees of exact physical travel.
The driver uses simulation time and continues publishing zero velocity after
completion. Press Ctrl+C to stop it and the recorder. Close Gazebo to stop the
simulation. Start a fresh simulation before repeating a run so the initial
condition is the same. Do not run multiple drivers simultaneously.

## Data and coordinate conventions
`/wheel_states`: actual wheel-joint angles in radians, obtained from Gazebo's JointStatePublisher. These are the only motion measurements intended for the later wheel-odometry estimator.
`/ground_truth`: physical model pose published by Gazebo's OdometryPublisher,separate from wheel-integrated odometry. Evaluation only.
`/cmd_vel`: prescribed body velocity command. No ground-truth feedback.
`/clock`: simulation time. Raw CSV streams retain their own timestamps and must be aligned by time during later evaluation.
Wheel radius: 0.15 m. Wheel separation: 0.625 m.
Robot model origin: midpoint of the drive axle in x/y; initial x=y=yaw=0.
Positive yaw turns counterclockwise; x is initially forward and y left.
DiffDrive's internal wheel odometry and TF use `/internal/*_unused` topics;they are deliberately excluded from the ROS bridge.
