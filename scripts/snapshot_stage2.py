#!/usr/bin/env python3
"""Freeze a consistent saved-data interval while the live display remains open."""
import csv
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
source = Path(sys.argv[1]) if len(sys.argv)>1 else Path((root/'data/latest_stage2.txt').read_text().strip())
if (source/'snapshot_info.json').exists():
    raise SystemExit('This directory is already a frozen snapshot')
streams = {}
for filename in ['wheel_states.csv', 'ground_truth.csv', 'estimated_pose.csv', 'errors.csv']:
    with (source/filename).open() as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        # Ignore only a partially written last line; every complete row is retained.
        rows = [row for row in reader if all(row.get(key) not in (None, '') for key in fields)]
    if not rows:
        raise SystemExit(f'No complete rows in {filename}')
    streams[filename] = fields, rows
cutoff = min(float(rows[-1]['time_s']) for fields, rows in streams.values())
target = Path(tempfile.mkdtemp(prefix='stage2_snapshot_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_', dir=root/'data'))
for filename, (fields, rows) in streams.items():
    with (target/filename).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(row for row in rows if float(row['time_s'])<=cutoff)
for filename in ['estimator_config.json', 'summary.json']:
    shutil.copy2(source/filename, target/filename)
(target/'snapshot_info.json').write_text(json.dumps(dict(source_run=str(source), cutoff_time_s=cutoff,
    method='Copy complete CSV rows and trim all four streams to their common saved end time; do not interpolate missing end samples'), indent=2)+'\n')
subprocess.run([sys.executable, str(root/'scripts/summarize_stage2.py'), str(target)], check=True)
result = subprocess.run([sys.executable, str(root/'scripts/check_stage2.py'), str(target)], capture_output=True, text=True)
(target/'check.txt').write_text(result.stdout+result.stderr)
if result.returncode:
    raise SystemExit(result.stdout+result.stderr)
(root/'data/latest_stage2.txt').write_text(str(target)+'\n')
print(result.stdout)
print('Frozen Stage 2 evidence:', target)
