#!/usr/bin/env python3
"""Read-only live display of the first-stage raw data capture."""
from pathlib import Path
import csv
import time
root = Path(__file__).resolve().parents[1]
folder = root/'data/stage1_validation'
try:
    while True:
        print('\033[2J\033[H', end='')
        print('IRA1 | Keran Chen | Stage 1: Simulation foundation\n')
        print('Project: '+str(root))
        print('Platform: ROS 2 Jazzy + Gazebo Sim 8\n')
        if (folder/'check.txt').exists():
            print('STAGE 1 VALIDATED | capture stopped | Gazebo remains open\n')
        for filename in ['wheel_states.csv', 'ground_truth.csv']:
            path = folder/filename
            if not path.exists():
                print(filename+': waiting')
                continue
            with path.open() as stream:
                rows = list(csv.reader(stream))
            print(f'{filename}: {max(0,len(rows)-1)} samples')
            if len(rows)>1:
                print('  Latest: '+', '.join(f'{k}={v}' for k,v in zip(rows[0], rows[-1])))
        print('\nThe Gazebo window shows the physical robot.\n')
        print('Current scope: scene, motion commands, raw wheel data and ground truth.')
        print('Localisation and noise modelling are reserved for Stage 2.')
        print('GitHub upload awaits repository details.\n')
        print('Close this terminal to stop this display only.')
        time.sleep(2)
except KeyboardInterrupt:
    pass
