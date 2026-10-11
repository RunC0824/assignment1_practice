import csv,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parents[1]
r=list(csv.DictReader((P/'data/trajectory.csv').open()));m=json.loads((P/'results/metrics.json').read_text())
def c(k):return [float(a[k]) for a in r]
fig,axs=plt.subplots(1,3,figsize=(15,4.8))
for ax,k,title in [(axs[0],'aligned_odom','Wheel odometry'),(axs[1],'gt','Global ground truth')]:
 ax.plot(c(k+'_x'),c(k+'_y'),label=title);ax.scatter(c(k+'_x')[0],c(k+'_y')[0],label='Start');ax.scatter(c(k+'_x')[-1],c(k+'_y')[-1],marker='x',label='End')
 ax.set(title=title,xlabel='World X (m)',ylabel='World Y (m)');ax.axis('equal');ax.grid();ax.legend()
axs[2].plot(c('gt_x'),c('gt_y'),label='Ground truth');axs[2].plot(c('aligned_odom_x'),c('aligned_odom_y'),'--',label='Aligned odometry');axs[2].set(title='Trajectory comparison',xlabel='World X (m)',ylabel='World Y (m)');axs[2].axis('equal');axs[2].grid();axs[2].legend()
fig.tight_layout();fig.savefig(P/'results/trajectory_comparison.png',dpi=200);plt.close(fig)
fig,ax=plt.subplots(figsize=(8,4));ax.plot(c('time_s'),c('error_m'),label='Position error')
for p in m['phases']:ax.axvline(p['start_sim_s']-float(r[0]['sim_time_s']),color='gray',alpha=.5)
ax.set(xlabel='Simulation time since start (s)',ylabel='Euclidean position error (m)',title='Odometry error over time');ax.grid();ax.legend();fig.tight_layout();fig.savefig(P/'results/position_error.png',dpi=200)
text=f'''# Lab 2: Odometry vs. Ground Truth

## Problem and method
Evaluate wheel-integrated differential-drive odometry against Gazebo global pose, following the six Practical Guide steps (PDF pages 10–11).

1. Gazebo Harmonic / ROS 2 Jazzy installed runtime; the bundled differential-drive example is copied into world.sdf. Only the blue robot is retained. Physics step: 0.001 s. Odometry and PosePublisher rate: 50 Hz.
2. Native Gazebo streams: /model/vehicle_blue/odometry (gz.msgs.Odometry), /model/vehicle_blue/pose (gz.msgs.Pose_V; global model pose), /model/vehicle_blue/cmd_vel (gz.msgs.Twist).
3. Repeatable odometry-feedback path: 2 m forward, stop 1 s, left 90 degrees, stop 1 s, 1 m forward, then stop. Limits: 0.3 m/s and 0.5 rad/s. Position tolerance 0.003 m, angle tolerance 0.005 rad. Ground truth never enters the controller or odometry estimate.
4. Both streams are recorded at identical simulator timestamps. Raw odometry and global ground truth remain in the CSV. The axle midpoint is 0.5542825 m forward of the model origin (mean wheel-link X offsets). The estimate is converted to model origin using estimated heading and this fixed geometry only. A fixed initial rigid transform aligns the odometry frame to the world frame for evaluation; no continuous correction is applied.
5. e(t) = sqrt((x_odom_world-x_gt)^2 + (y_odom_world-y_gt)^2).
6. The comparison figure includes separate paths and an overlay; the error figure shows error accumulation.

## Measured results
- Paired samples: {m['samples']}
- Duration: {m['duration_s']:.3f} s
- Final Euclidean position error: {m['final_position_error_m']:.9f} m
- Final heading error: {m['final_yaw_error_rad']:.9f} rad
- Position RMSE: {m['rmse_m']:.9f} m
- Maximum position error: {m['max_error_m']:.9f} m

## Analysis and limitations
Comparing different body reference points initially produced a 0.786239 m endpoint discrepancy. The turn rotates the 0.5542825 m lever arm; this is a coordinate-reference effect rather than accumulated localisation drift. The reported results correct this lever arm with model geometry and estimated heading.
The estimate integrates wheel motion with fixed wheel radius (0.3 m) and wheel separation (1.25 m). Ground truth comes independently from the simulated rigid-body pose. Wheel-ground slip, caster contact, contact solver discretisation, and small differences between assumed and effective wheel geometry can therefore produce disagreement. Turning is particularly sensitive to effective wheel separation and differential wheel slip. The world includes wheel friction parameters and slip1=0.035; the actual effect depends on the physics engine and cannot be inferred from the parameter alone. No artificial noise or calibration bias was injected in this practical. This run does not isolate individual causes, so the listed mechanisms are plausible explanations rather than measured causal contributions. Exact timestamp pairing removes reception-delay mismatch; initial frame alignment removes a fixed coordinate offset, not drift. Acceleration limits and feedback mean actual execution time differs from a simple distance/speed calculation.

## Reproduce
From this folder, source /opt/ros/jazzy/setup.bash. Start `gz sim -r world.sdf`, then run `python3 scripts/run_experiment.py`, followed by `MPLCONFIGDIR=/tmp/lab2-mpl python3 scripts/analyse.py`. Restart Gazebo with the original world before repeating the same global trajectory. Do not run the controller simultaneously twice.

Source: Intelligent Robotics lab 2.pdf, Practical Guide - Odometry vs. Ground Truth. This completes the practical exercise; the separate Assignment 1 requirements are not the same task.
'''
(P/'README.md').write_text(text)
