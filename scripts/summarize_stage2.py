#!/usr/bin/env python3
"""Rebuild final statistics from captured aligned errors, also after interruption."""
import argparse
import csv
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('run_dir', type=Path)
args = parser.parse_args()
folder = args.run_dir
with (folder/'errors.csv').open() as stream:
    rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(stream)]
if not rows:
    raise SystemExit('No aligned errors to summarise')
path = folder/'summary.json'
summary = json.loads(path.read_text()) if path.exists() else {}
# Do not infer any new errors from unmatched end-of-capture samples.
summary.update(samples=len(rows), first_time_s=rows[0]['time_s'], last_time_s=rows[-1]['time_s'],
    position_rmse_m=math.sqrt(sum(r['position_error_m']**2 for r in rows)/len(rows)),
    heading_rmse_rad=math.sqrt(sum(r['heading_error_rad']**2 for r in rows)/len(rows)),
    max_position_error_m=max(r['position_error_m'] for r in rows),
    max_abs_heading_error_rad=max(abs(r['heading_error_rad']) for r in rows),
    last_position_error_m=rows[-1]['position_error_m'],
    last_heading_error_rad=rows[-1]['heading_error_rad'],
    summary_source='Final statistics recomputed from saved errors.csv; unmatched end samples are not extrapolated')
# Pending/drop counts describe the last live checkpoint, not inferred shutdown state.
summary['live_checkpoint_pending_samples'] = summary.pop('pending_samples', summary.get('live_checkpoint_pending_samples'))
temporary = folder/'summary.json.tmp'
temporary.write_text(json.dumps(summary, indent=2)+'\n')
temporary.replace(path)
print(json.dumps(summary, indent=2))
