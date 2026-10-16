#!/usr/bin/env python3
"""Create the Canvas ZIP from committed files and local Git history."""
import argparse
from pathlib import Path
import subprocess
import zipfile

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,default=ROOT/'submission/IRA1_Keran_Chen.zip')
args=parser.parse_args()
status=subprocess.check_output(['git','-C',str(ROOT),'status','--porcelain','--untracked-files=no'],text=True)
if status.strip():
    raise SystemExit('Commit tracked changes before packaging so the ZIP and history agree')
tracked=subprocess.check_output(['git','-C',str(ROOT),'ls-files','-z']).decode().split('\0')
files=[ROOT/name for name in tracked if name]
metadata=ROOT/'.git'
if not metadata.is_dir():
    raise SystemExit('A full local .git directory is required for this coursework submission')
files += [p for p in metadata.rglob('*') if p.is_file() and not p.name.endswith('.lock')]
required=[ROOT/'results/Assignment1_Report_Keran_Chen.pdf',ROOT/'results/trajectory_comparison.png',metadata/'HEAD']
assert all(p in files for p in required), 'Required submission component missing'
args.output.parent.mkdir(parents=True,exist_ok=True)
with zipfile.ZipFile(args.output,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
    for path in sorted(set(files)):
        archive.write(path,Path('IRA1')/path.relative_to(ROOT))
with zipfile.ZipFile(args.output) as archive:
    assert archive.testzip() is None
    assert all(not name.startswith('/') and '..' not in Path(name).parts for name in archive.namelist())
print(f'Created {args.output} ({args.output.stat().st_size:,} bytes, {len(set(files))} files)')
