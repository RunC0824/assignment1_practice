#!/usr/bin/env python3
"""Check a completed Stage 2 run from saved evidence, without modifying it."""
import csv
import json
import math
from pathlib import Path
import random
import sys

root = Path(__file__).resolve().parents[1]
folder = Path(sys.argv[1]) if len(sys.argv)>1 else Path((root/'data/latest_stage2.txt').read_text().strip())


def read(name):
    with (folder/name).open() as stream:
        rows = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]
    assert rows, f'{name}: no samples'
    assert all(math.isfinite(value) for row in rows for value in row.values()), f'{name}: non-finite data'
    assert all(b['time_s']>=a['time_s'] for a,b in zip(rows, rows[1:])), f'{name}: time moved backwards'
    return rows


estimates, errors = read('estimated_pose.csv'), read('errors.csv')
truth, wheels = read('ground_truth.csv'), read('wheel_states.csv')
assert len(estimates)>2000 and len(errors)>2000, 'Need a complete square-motion run'
assert errors[-1]['time_s']-errors[0]['time_s']>54, 'Run ended before the nominal motion sequence could complete'
saved_estimates = {row['time_s']: row for row in estimates}
for row in errors:
    assert row['time_s'] in saved_estimates, 'An error row has no saved estimate at its timestamp'
    saved = saved_estimates[row['time_s']]
    for axis in ('x_m', 'y_m', 'theta_rad'):
        assert math.isclose(row['estimated_'+axis], saved[axis], abs_tol=1e-12)
config = json.loads((folder/'estimator_config.json').read_text())
assert config['ground_truth_used_for_estimation'] is False
parameters = config['parameters']
rng = random.Random(parameters['seed'])
perturbation = 0.0
for row in estimates[1:]:
    for side in ('left','right'):
        raw = row[f'raw_{side}_m']
        sigma = parameters['noise_std_m_per_sqrt_m']*math.sqrt(abs(raw))
        epsilon = rng.gauss(0.0, sigma) if sigma else 0.0
        expected = (1+parameters[f'{side}_scale_bias'])*raw+epsilon
        actual = row[f'measured_{side}_m']
        assert math.isclose(expected, actual, rel_tol=1e-12, abs_tol=1e-12), 'Noise/seed replay differs'
        perturbation += abs(actual-raw)
if parameters['noise_std_m_per_sqrt_m'] or parameters['left_scale_bias'] or parameters['right_scale_bias']:
    assert perturbation>0, 'Configured uncertainty was not applied'
for row in errors:
    dx = row['estimated_x_m']-row['truth_x_m']
    dy = row['estimated_y_m']-row['truth_y_m']
    heading = (row['estimated_theta_rad']-row['truth_theta_rad']+math.pi)%(2*math.pi)-math.pi
    assert math.isclose(row['position_error_m'], math.hypot(dx,dy), abs_tol=1e-12)
    assert math.isclose(row['heading_error_rad'], heading, abs_tol=1e-12)
    assert -math.pi<=row['heading_error_rad']<math.pi
for key in ('x_m','y_m'):
    assert max(row[key] for row in truth)-min(row[key] for row in truth)>1, 'Missing physical movement'
summary = json.loads((folder/'summary.json').read_text())
assert summary['samples']==len(errors), 'Final summary and CSV sample counts differ'
rmse = math.sqrt(sum(row['position_error_m']**2 for row in errors)/len(errors))
assert math.isclose(summary['position_rmse_m'], rmse, abs_tol=1e-12)
print('PASS: complete motion, finite monotonic data, seeded uncertainty replay and saved error calculations.')
print(f'Raw samples: wheels={len(wheels)}, truth={len(truth)}')
print(f'Pose estimates={len(estimates)}, aligned errors={len(errors)}')
print(f'Position RMSE={rmse:.6f} m; final position error={errors[-1]["position_error_m"]:.6f} m')
print(f'Final heading error={math.degrees(errors[-1]["heading_error_rad"]):.6f} deg')
print(f'Unalignable samples={summary["dropped_unalignable_samples"]}; pending at last live checkpoint={summary.get("live_checkpoint_pending_samples", summary.get("pending_samples"))}')
print('Scope: preliminary Stage 2 functional run, including stationary samples.')
