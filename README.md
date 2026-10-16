
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


## Stage 2: wheel odometry and error tracking

Start a fresh simulation with the robot at its configured origin. Do not use
Gazebo reset during a run, or run two copies of the simulation simultaneously.
Build again after source changes:

```bash
cd ~/IRA1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
./scripts/start_sim.sh
```

In a second terminal:

```bash
cd ~/IRA1
./scripts/run_stage2.sh
```

This starts the raw recorder, wheel-only estimator, independent evaluator,
rectangle motion and a live trajectory/error window. Stop the experiment using
Ctrl+C in its terminal after the driver reports completion. Gazebo runs in the
first terminal and stays open. Close it or press Ctrl+C there before a new run.
The live view may be reopened after capture:

```bash
python3 scripts/live_stage2.py "$(cat data/latest_stage2.txt)"
```

The ROS domain is now 42 and the Gazebo partition is `ira1_stage2` to separate
this work from older demonstration processes. All ROS terminals must source
`scripts/environment.sh` when running manual commands. Closing Gazebo now also
stops its bridge. Exiting the estimator or evaluator stops the experiment launch.

### Method and uncertainty assumptions

`ira1_sim/odometry.py` implements the Lecture 2 short-step midpoint update:

- delta_s = (delta_s_right + delta_s_left) / 2
- delta_theta = (delta_s_right - delta_s_left) / wheel_separation
- x += delta_s * cos(theta + delta_theta/2)
- y += delta_s * sin(theta + delta_theta/2)
- theta = wrap(theta + delta_theta)

The known initial pose is (0 m, 0 m, 0 rad), set in configuration. It is never
initialised, corrected or aligned using simulator pose. The estimator subscribes
only to `/wheel_states` and publishes `/estimated_pose`. The velocity commands,
Gazebo's internal odometry and ground truth are not estimator inputs.

`config/odometry.yaml` defines illustrative encoder uncertainty parameters:

- Quantise cumulative joint angles to 2048 counts/revolution before differencing.
- Convert angular increments using the nominal 0.15 m radius.
- For each wheel: measured_delta = (1 + bias) * quantised_delta + epsilon.
- Independent Gaussian epsilon has standard deviation
  k * sqrt(abs(quantised_delta)), with k = 0.01 m/sqrt(m).
- Left scale bias = 0; right scale bias = +0.01 (a 1% calibration mismatch).
- Random seed = 42. Noise is applied only to nonzero measured wheel increments,
  so a stationary wheel does not accumulate random travel.
- Pose updates aggregate wheel angles at approximately 50 Hz using message
  timestamps, rather than applying fixed noise at every 1 kHz physics step.

These are stated experimental assumptions, not parameters supplied by the
lecturer or measured from hardware. The geometry, wheel-odometry update and
qualitative error sources follow Lecture 2. No Kalman filter, covariance
propagation, Markov localisation, particle filter, SLAM or ground-truth correction
is used. ROS covariance arrays are not a computed uncertainty model.

`ira1_sim/evaluation.py` aligns each estimate with bracketing ground-truth
samples: linear interpolation for x/y and shortest-angle interpolation for yaw.
It waits for a future truth sample rather than extrapolating. Gaps over 0.1 s
are rejected and counted. This interpolation is evaluation bookkeeping only.

Position error is sqrt((x_est-x_true)^2 + (y_est-y_true)^2). Heading error is
wrap(theta_est-theta_true) in [-pi, pi). The evaluator also records component
errors, running RMSE, maximum errors and last-sample errors. It never sends
any correction to the estimator.

### Stage 2 files

Each new run is saved in `data/stage2_DATE_TIME_ID/`:

- `wheel_states.csv` and `ground_truth.csv`: independent raw streams.
- `estimated_pose.csv`: estimated pose and raw/perturbed wheel increments.
- `estimator_config.json`: exact parameters, seed and model assumptions.
- `errors.csv`: timestamp-aligned poses and position/heading errors.
- `summary.json`: running/final metrics, drop counts and evaluation time range.

`data/latest_stage2.txt` contains the latest run directory. Preview figures are
functional checks; the final experiment comparison and one-page assessed report
are Stage 3 work. Statistics currently include all aligned samples, including
stops, so they must not be described as moving-only statistics.

### Verification

```bash
PYTHONPATH=src/ira1_sim python3 -m unittest discover -s tests -v
```

The tests cover straight motion, in-place turning, the lecture's 2-degree error
example, quantisation, aggregation, reproducible noise, stationary behaviour,
time reset rejection, delayed ground truth and angle wrapping across pi.

After capture, finalise the saved statistics and check the run:

```bash
python3 scripts/summarize_stage2.py "$(cat data/latest_stage2.txt)"
python3 scripts/check_stage2.py
```

The finaliser recomputes metrics from the saved aligned error rows, including
after interrupted capture. It does not invent missing end samples. The checker
also verifies that every evaluated estimate exists in `estimated_pose.csv` and
replays the saved random seed against the recorded wheel perturbations.

To keep the live windows open while freezing a checked data interval:

```bash
python3 scripts/snapshot_stage2.py
```

This copies complete rows from all four CSV streams, cuts them at their common
saved end time, recomputes summary metrics and checks the result. The frozen
folder contains `snapshot_info.json` and `check.txt`. `data/latest_stage2.txt`
then points to this frozen evidence; existing live windows keep displaying the
original capture. Stop the live experiment with Ctrl+C in its terminal when
finished observing. Snapshot data and live capture are kept separate.


### Saved Stage 2 validation

`validation/stage2/` contains a checked functional run: parameter metadata,
validation output, metric summary and a trajectory/error preview. The run has
5,138 matched estimates and errors, with final position error 0.230118 m and
final signed heading error 9.974321 degrees. Metrics include stationary samples.
The saved preview was generated from this same frozen run. Raw CSV recordings
remain local; rerun the commands above to generate new measurements.
The controlled noise comparison and final assessed report belong to Stage 3.
