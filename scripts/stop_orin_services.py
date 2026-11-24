#!/usr/bin/env python3
"""Stop only the recorded Orin project supervisors, never robot/global ROS jobs."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--compute', action='store_true')
    parser.add_argument('--web', action='store_true')
    parser.add_argument('--standalone', action='store_true')
    args = parser.parse_args()
    records = []
    if args.standalone:
        records.append(('logs/orin_standalone/current_run.json', 'orin_standalone.py'))
    if args.compute:
        records.append(('logs/go2_orin_research/current_run.json', 'orin_research_wireless.py'))
    if args.web:
        records.append(('logs/orin_web/current_run.json', 'orin_research_web.py'))
    if not records:
        parser.error('Choose --compute, --web or --standalone')
    for name, script in records:
        path = ROOT / name
        if not path.is_file():
            continue
        pid = json.loads(path.read_text())['pid']
        process = Path('/proc', str(pid))
        try:
            command = (process / 'cmdline').read_bytes()
            working = (process / 'cwd').resolve()
        except OSError:
            continue
        if script.encode() not in command or working != ROOT:
            raise RuntimeError('Recorded PID no longer belongs to this project: ' + str(pid))
        os.kill(pid, signal.SIGTERM)
        deadline = time.monotonic() + 35
        while process.exists() and time.monotonic() < deadline:
            time.sleep(.2)
        if process.exists():
            raise RuntimeError('Supervisor is still stopping: ' + str(pid))
        print('Stopped ' + script + ' PID ' + str(pid))


if __name__ == '__main__':
    main()
