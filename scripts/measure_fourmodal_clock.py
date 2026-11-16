#!/usr/bin/env python3
"""Read remote wall time over SSH; report offset and round-trip uncertainty."""
import argparse
import json
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='unitree@192.168.123.18')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    samples = []
    for _ in range(5):
        start = time.time()
        completed = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', args.host,
            "python3 -c 'import time; print(time.time())'"], capture_output=True, text=True, timeout=12, check=True)
        end = time.time()
        remote = float(completed.stdout.strip())
        samples.append(dict(round_trip_s=end - start, remote_stamp=remote,
                            local_midpoint_stamp=(start + end) / 2,
                            workstation_minus_robot_s=(start + end) / 2 - remote))
    best = min(samples, key=lambda sample: sample['round_trip_s'])
    report = dict(method='ssh_wall_clock_midpoint_min_rtt', measured_at=time.time(), host=args.host,
                  workstation_minus_robot_s=best['workstation_minus_robot_s'],
                  round_trip_uncertainty_s=best['round_trip_s'] / 2, samples=samples,
                  hardware_synchronization_validated=False,
                  note='Clock estimate only. Does not bound sensor exposure/scan/ADC latency or establish hardware synchronization.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
