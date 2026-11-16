#!/usr/bin/env python3
"""Explicit, bounded right-arm policy execution through the guarded ROS bridge."""
import argparse
from datetime import datetime
import math
from pathlib import Path
from openarm_smolvla_shadow import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True,
                        help='allow right-arm/gripper motion; keep the workspace clear')
    parser.add_argument('--seconds', type=float, default=10, help='maximum trial duration (default 10)')
    parser.add_argument('--model', default=str(Path.home()/'robot_ai/models/smolvla_cube_20k'))
    parser.add_argument('--task', default='Put the cube in the basket')
    parser.add_argument('--domain-id', type=int, default=84)
    parser.add_argument('--port', type=int, default=8081)
    parser.add_argument('--output', default=f'/tmp/openarm_policy_{datetime.now():%Y%m%d_%H%M%S_%f}.jsonl')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 360:
        parser.error('--seconds must be greater than 0 and at most 360')
    try:
        run(args)
    except KeyboardInterrupt:
        print('Policy stopped by operator. VR remains disarmed; motor torque is not released.')
    except Exception as exc:
        parser.exit(1, f'Policy stopped: {exc}. No automatic restart.\n')


if __name__ == '__main__':
    main()
