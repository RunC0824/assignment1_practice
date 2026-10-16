#!/usr/bin/env python3
"""Validate raw Stage 1 data; does not implement the localisation estimator."""
import csv
import math
from pathlib import Path
import sys

folder = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parents[1]/'data/stage1_validation'
streams = {}
for name in ['wheel_states','ground_truth']:
    with (folder/(name+'.csv')).open() as f:
        rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]
    assert len(rows)>100, f'{name}: insufficient samples'
    assert all(math.isfinite(v) for row in rows for v in row.values()), f'{name}: non-finite values'
    assert all(b['time_s']>=a['time_s'] for a,b in zip(rows,rows[1:])), f'{name}: time went backwards'
    streams[name] = rows
    print(f'{name}: {len(rows)} samples, t={rows[0]["time_s"]:.3f} to {rows[-1]["time_s"]:.3f} s')
w=streams['wheel_states']; g=streams['ground_truth']
assert w[-1]['left_angle_rad']-w[0]['left_angle_rad']>20, 'Left wheel did not complete expected motion'
assert w[-1]['right_angle_rad']-w[0]['right_angle_rad']>30, 'Right wheel did not complete expected motion'
for key in ['x_m','y_m']:
    extent=max(row[key] for row in g)-min(row[key] for row in g)
    assert extent>1.0, f'Insufficient physical motion in {key}'
    print(f'{key} extent: {extent:.3f} m')
yaw=[2*math.atan2(row['qz'],row['qw']) for row in g]
assert max(yaw)-min(yaw)>3., 'Insufficient physical turning'
print('PASS: timestamps, wheel motion, physical translation and turning verified.')
print('This is simulation interface validation, not localisation accuracy analysis.')
