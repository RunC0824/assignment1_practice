#!/usr/bin/env python3
"""Read-only live trajectory/error display from the independent evaluator CSV."""
import argparse
import csv
import math
from pathlib import Path
import matplotlib

parser = argparse.ArgumentParser()
parser.add_argument('run_dir', type=Path)
parser.add_argument('--snapshot', type=Path, help='Save a static preview instead of opening a window')
args = parser.parse_args()
matplotlib.use('Agg' if args.snapshot else 'TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.rcParams.update({'font.size': 10})
fig = plt.figure(figsize=(11, 8), constrained_layout=True)
fig.canvas.manager.set_window_title('IRA1 - Stage 2: wheel odometry and errors')
grid = fig.add_gridspec(2, 2, height_ratios=[1.4, 1])
trajectory = fig.add_subplot(grid[0, :])
position = fig.add_subplot(grid[1, 0])
heading = fig.add_subplot(grid[1, 1])
truth_line, = trajectory.plot([], [], color='#1e293b', label='Simulator ground truth', linewidth=2)
estimate_line, = trajectory.plot([], [], color='#d35d27', label='Wheel odometry estimate', linewidth=1.8)
trajectory.set(xlabel='x (m)', ylabel='y (m)', title='Trajectory comparison')
trajectory.set_aspect('equal', adjustable='datalim')
trajectory.legend(loc='upper left')
position_line, = position.plot([], [], color='#2563eb')
position.set(xlabel='Elapsed evaluation time (s)', ylabel='Position error (m)', title='Euclidean position error')
heading_line, = heading.plot([], [], color='#8b5cf6')
heading.set(xlabel='Elapsed evaluation time (s)', ylabel='Heading error (deg)', title='Signed heading error')
for axis in (trajectory, position, heading):
    axis.grid(True, alpha=.25)


def update(_=None):
    rows = []
    path = args.run_dir/'errors.csv'
    if path.exists():
        with path.open() as stream:
            for raw in csv.DictReader(stream):
                try:
                    rows.append({key: float(value) for key, value in raw.items()})
                except (ValueError, TypeError):
                    continue  # A writer may be partway through its last CSV row.
    if not rows:
        fig.suptitle('IRA1 Stage 2 | waiting for aligned measurements')
        return
    times = [row['time_s']-rows[0]['time_s'] for row in rows]
    truth_line.set_data([r['truth_x_m'] for r in rows], [r['truth_y_m'] for r in rows])
    estimate_line.set_data([r['estimated_x_m'] for r in rows], [r['estimated_y_m'] for r in rows])
    position_line.set_data(times, [r['position_error_m'] for r in rows])
    heading_line.set_data(times, [math.degrees(r['heading_error_rad']) for r in rows])
    for axis in (trajectory, position, heading):
        axis.relim()
        axis.autoscale_view()
    last = rows[-1]
    fig.suptitle(f'IRA1 Stage 2 | t={times[-1]:.1f} s | '
                f'position error={last["position_error_m"]:.3f} m | '
                f'heading error={math.degrees(last["heading_error_rad"]):.2f} deg')


update()
if args.snapshot:
    args.snapshot.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.snapshot, dpi=150)
else:
    animation = FuncAnimation(fig, update, interval=1000, cache_frame_data=False)
    plt.show()
