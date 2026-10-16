# IRA1 — Wheel Odometry and Error Tracking



**Environment:** 
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.15.0.

A differential-drive robot follows a short square route. Wheel odometry estimates
its position and heading; independent simulator ground truth evaluates the error.

## Run

Build and open Gazebo:

```bash
cd ~/IRA1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
./scripts/start_sim.sh
```

In another terminal, start the robot, recording and live plots:

```bash
cd ~/IRA1
./scripts/run_stage2.sh
```

Start each run with a fresh simulation at the configured origin. Press Ctrl+C in
the experiment terminal to stop recording; close Gazebo before starting another run.

## Method

Lecture 2 midpoint wheel odometry, starting from known pose `(0, 0, 0)`.
Input: `/wheel_states`; output: `/estimated_pose`.
Ground truth is used only for timestamp-aligned evaluation.
Wheel radius: 0.15 m; separation: 0.625 m; encoder: 2048 counts/revolution.
Measurement model: `measured_delta = (1 + bias) * delta + epsilon`, with Gaussian standard deviation `0.01 * sqrt(abs(delta))` metres.
Left/right scale bias: 0% / 1%; random seed: 42. These are experiment assumptions.

Parameters: `src/ira1_sim/config/odometry.yaml`.

## Results

Each run saves raw wheel data, ground truth, estimated poses, errors and parameters
under `data/stage2_*/`. The latest directory is listed in `data/latest_stage2.txt`.

`results/` contains the final one-page English report, labelled trajectory plot,
error comparison and numerical results. The controlled comparison replays the
same wheel data with k = 0 and k = 0.01; all other parameters remain fixed.
Final position errors are **0.2218 m** and **0.1808 m**, respectively (seed 42).
This single realisation does not establish a statistical trend.
`validation/stage2/` retains the earlier functional check.

Save a checked snapshot while keeping the live windows open:

```bash
python3 scripts/snapshot_stage2.py
```

## Checks

```bash
PYTHONPATH=src/ira1_sim python3 -m unittest discover -s tests -v
python3 scripts/check_stage2.py
```

## Reproduce final results

```bash
python3 scripts/compare_noise.py
```

The compressed recordings in `experiments/input/` make this comparison repeatable
without launching Gazebo. To rebuild the report, run `scripts/build_report.py`
with a Python environment containing ReportLab and the DejaVu fonts.
The Canvas ZIP includes the report, plots, runnable source, recordings and `.git`.
