#!/usr/bin/env python3
"""Replay the same encoder recording with one changed noise coefficient.

Ground truth is opened only after both wheel-only estimates have been generated.
All source recordings needed to reproduce the saved figures are included as gzip.
"""
import argparse
import bisect
import csv
from dataclasses import asdict, fields
import gzip
import hashlib
import io
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/ira1_sim'))
from ira1_sim.odometry import WheelOdometry, Sample
from ira1_sim.evaluation import Pose, interpolate_truth, pose_error

DEFAULTS = dict(wheel_radius_m=0.15, wheel_separation_m=0.625,
    encoder_counts_per_rev=2048, update_period_s=0.02,
    noise_std_m_per_sqrt_m=0.0, left_scale_bias=0.0, right_scale_bias=0.01,
    seed=42, initial_x_m=0.0, initial_y_m=0.0, initial_theta_rad=0.0)


def load_rows(path):
    opener = gzip.open if path.suffix=='.gz' else open
    with opener(path, 'rt', newline='') as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames
        rows = [{k:float(row[k]) for k in names} for row in reader
                if all(row.get(k) not in (None, '') for k in names)]
    if not rows or not all(math.isfinite(v) for row in rows for v in row.values()):
        raise ValueError(f'Empty or invalid recording: {path}')
    if any(b['time_s']<a['time_s'] for a,b in zip(rows,rows[1:])):
        raise ValueError(f'Recording time went backwards: {path}')
    return names, rows


def estimate(wheels, config):
    model = WheelOdometry(**config)
    samples = []
    for row in wheels:
        sample = model.update(row['time_s'], row['left_angle_rad'], row['right_angle_rad'])
        if sample is not None:
            samples.append(sample)
    return samples


def write_rows(path, names, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=names, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def capture(source, target):
    if any(target.glob('*.csv.gz')):
        raise ValueError('Input already exists; remove it explicitly before replacing the experiment')
    # Wheel motion defines the evaluation window; no true pose initialises the estimate.
    wheel_names, wheels = load_rows(source/'wheel_states.csv')
    baseline = estimate(wheels, DEFAULTS)
    moving = [sample for sample in baseline if abs(sample.raw_left_m)+abs(sample.raw_right_m)>1e-9]
    wheel_distance = sum((abs(s.raw_left_m)+abs(s.raw_right_m))/2 for s in baseline)
    if wheel_distance<9.0 or not moving:
        raise ValueError('Wait for the four-side motion sequence to finish before capturing')
    motion_end = moving[-1].time_s
    cutoff = motion_end + 1.0
    truth_names, truth = load_rows(source/'ground_truth.csv')
    if min(wheels[-1]['time_s'],truth[-1]['time_s']) < cutoff+0.03:
        raise ValueError('Wait for at least one second of settling after the final wheel motion')
    # Retain one future truth sample for bracketing the endpoint. Keep initial rest
    # for the known-start odometry integration, but exclude it from RMSE later.
    for name,names,rows in [('wheel_states',wheel_names,wheels),('ground_truth',truth_names,truth)]:
        rows = [r for r in rows if r['time_s']<=cutoff+0.05]
        payload = io.StringIO(newline='')
        writer = csv.DictWriter(payload, fieldnames=names)
        writer.writeheader(); writer.writerows(rows)
        (target/(name+'.csv.gz')).write_bytes(gzip.compress(payload.getvalue().encode(), mtime=0))
    metadata = dict(source_run=source.name,
        initial_pose='Known configured origin (0 m, 0 m, 0 rad)',
        motion_start_s=moving[0].time_s-moving[0].dt_s,
        motion_end_s=motion_end, evaluation_end_target_s=cutoff,
        window_rule='First encoder motion to one second after the last encoder motion; no extended idle tail',
        total_absolute_wheel_distance_m=wheel_distance)
    metadata['sha256'] = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in target.glob('*.csv.gz')}
    (target/'source_info.json').write_text(json.dumps(metadata, indent=2)+'\n')


def align(samples, truth):
    times = [p.time_s for p in truth]
    rows = []
    for sample in samples:
        i = bisect.bisect_right(times, sample.time_s)-1
        if i<0 or i+1>=len(truth):
            continue
        first, second = truth[i], truth[i+1]
        if second.time_s-first.time_s>0.1:
            continue
        target = interpolate_truth(first, second, sample.time_s)
        rows.append(pose_error(Pose(sample.time_s,sample.x_m,sample.y_m,sample.theta_rad), target))
    return rows


def metrics(rows):
    n=len(rows)
    return dict(samples=n, first_time_s=rows[0]['time_s'], last_time_s=rows[-1]['time_s'],
        duration_s=rows[-1]['time_s']-rows[0]['time_s'],
        final_position_error_m=rows[-1]['position_error_m'],
        final_heading_error_deg=math.degrees(rows[-1]['heading_error_rad']),
        position_rmse_m=math.sqrt(sum(r['position_error_m']**2 for r in rows)/n),
        max_position_error_m=max(r['position_error_m'] for r in rows),
        heading_rmse_deg=math.degrees(math.sqrt(sum(r['heading_error_rad']**2 for r in rows)/n)))


def figures(output, errors, start):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    colors={'baseline':'#1976a3','noisy':'#d25d29'}
    labels={'baseline':'Wheel odometry: k = 0','noisy':'Wheel odometry: k = 0.01'}
    reference=errors['baseline']
    fig,ax=plt.subplots(figsize=(7.3,6.6),constrained_layout=True)
    ax.plot([r['truth_x_m'] for r in reference],[r['truth_y_m'] for r in reference],color='#202c39',linewidth=2.4,label='Simulator ground truth')
    for name,rows in errors.items():
        ax.plot([r['estimated_x_m'] for r in rows],[r['estimated_y_m'] for r in rows],color=colors[name],linewidth=1.8,linestyle='--' if name=='baseline' else '-',label=labels[name])
        ax.plot(rows[-1]['estimated_x_m'],rows[-1]['estimated_y_m'],'o',color=colors[name],markersize=5)
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Differential-drive localisation: controlled noise comparison')
    ax.set_aspect('equal',adjustable='box'); ax.grid(True,alpha=.22)
    ax.legend(loc='upper center',bbox_to_anchor=(0.5,-0.13),frameon=False,fontsize=10)
    fig.savefig(output/'trajectory_comparison.png',dpi=220)
    plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(7.3,6.2),sharex=True,constrained_layout=True)
    for name,rows in errors.items():
        t=[r['time_s']-start for r in rows]
        axes[0].plot(t,[r['position_error_m'] for r in rows],color=colors[name],label=labels[name])
        axes[1].plot(t,[math.degrees(r['heading_error_rad']) for r in rows],color=colors[name],label=labels[name])
    axes[0].set(ylabel='Position error (m)',title='Errors evaluated at matching times in the world frame')
    axes[1].set(xlabel='Time since first wheel motion (s)',ylabel='Signed heading error (deg)')
    axes[0].legend(fontsize=10)
    for ax in axes: ax.grid(True,alpha=.22)
    fig.savefig(output/'error_comparison.png',dpi=220)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',type=Path,help='Freeze a completed live run as the formal input')
    parser.add_argument('--input-dir',type=Path,default=ROOT/'experiments/input')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'results')
    args=parser.parse_args()
    args.input_dir.mkdir(parents=True,exist_ok=True); args.output_dir.mkdir(parents=True,exist_ok=True)
    if args.capture: capture(args.capture,args.input_dir)
    info=json.loads((args.input_dir/'source_info.json').read_text())
    for name,digest in info['sha256'].items():
        if hashlib.sha256((args.input_dir/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Formal input checksum changed: '+name)
    _,wheels=load_rows(args.input_dir/'wheel_states.csv.gz')
    configs={'baseline':dict(DEFAULTS),'noisy':dict(DEFAULTS,noise_std_m_per_sqrt_m=0.01)}
    estimates={name:estimate(wheels,cfg) for name,cfg in configs.items()}
    changed=[key for key in configs['baseline'] if configs['baseline'][key]!=configs['noisy'][key]]
    assert changed==['noise_std_m_per_sqrt_m']
    # The two estimates are already complete before simulator pose is used.
    _,truth_rows=load_rows(args.input_dir/'ground_truth.csv.gz')
    truth=[]
    for row in truth_rows:
        qx,qy,qz,qw=(row[k] for k in ['qx','qy','qz','qw'])
        yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        truth.append(Pose(row['time_s'],row['x_m'],row['y_m'],yaw))
    errors={}
    for name,samples in estimates.items():
        rows=align(samples,truth)
        rows=[r for r in rows if info['motion_start_s']-1e-9<=r['time_s']<=info['evaluation_end_target_s']+1e-9]
        if len(rows)<2000: raise ValueError('Insufficient aligned motion samples')
        errors[name]=rows
        write_rows(args.output_dir/(name+'_estimates.csv'),[f.name for f in fields(Sample)],[asdict(s) for s in samples])
        write_rows(args.output_dir/(name+'_errors.csv'),list(rows[0]),rows)
    assert [r['time_s'] for r in errors['baseline']]==[r['time_s'] for r in errors['noisy']]
    output=dict(method='Lecture 2 midpoint wheel odometry with error tracking',
        changed_parameter=changed[0],configs=configs,
        evaluation_window=info,metrics={name:metrics(rows) for name,rows in errors.items()},
        limitations='One recorded route and one random seed; no statistical claim across independent trials')
    (args.output_dir/'metrics.json').write_text(json.dumps(output,indent=2)+'\n')
    figures(args.output_dir,errors,info['motion_start_s'])
    print(json.dumps(output['metrics'],indent=2))


if __name__=='__main__': main()
