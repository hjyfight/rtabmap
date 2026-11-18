#!/usr/bin/env python3
"""Prepare display-only clock estimates for sensors timestamped on the wired robot."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rtabmap_features'))
from rtabmap_features.calibration import load_calibration


def prepare_profile(calibration_path, clock_path, output_path, now=None):
    clock_path, output_path = Path(clock_path), Path(output_path)
    clock = json.loads(clock_path.read_text())
    now = time.time() if now is None else now
    for key in ('measured_at', 'workstation_minus_robot_s', 'round_trip_uncertainty_s'):
        value = clock.get(key)
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('Invalid clock report field: ' + key)
    if not 0 <= now - clock['measured_at'] < 86400:
        raise ValueError('Clock report must be less than one day old; measure again')
    if clock['round_trip_uncertainty_s'] < 0:
        raise ValueError('Negative clock uncertainty')
    if (clock.get('method') != 'ssh_wall_clock_midpoint_min_rtt' or
            clock.get('hardware_synchronization_validated') is not False):
        raise ValueError('Expected an explicit unverified SSH wall-clock estimate')
    profile = load_calibration(calibration_path)
    if any(sensor['status'] == 'measured' for sensor in profile['sensors'].values()):
        raise ValueError('Preserve measured calibration; use start_object_multimodal_fusion.sh directly')
    base_hash = profile.pop('profile_sha256')
    profile['version'] += '_robot_clock_estimate'
    profile['measurement_scope'] = 'estimated_clock_offsets'
    profile['runtime_clock_estimate'] = {
        'method': clock['method'], 'host': clock.get('host'),
        'measured_at': clock['measured_at'],
        'clock_report_sha256': hashlib.sha256(clock_path.read_bytes()).hexdigest(),
        'base_calibration_sha256': base_hash,
        'hardware_synchronization_validated': False,
        'scope': 'All input acquisition timestamps use the robot wall clock; registered depth follows RGB. '
                 'Does not establish sensor synchronization or mounting calibration.',
    }
    for sensor in profile['sensors'].values():
        sensor['clock_offset_s'] = clock['workstation_minus_robot_s']
        sensor['clock_uncertainty_s'] = clock['round_trip_uncertainty_s']
        sensor['clock_offset_method'] = clock['method']
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    load_calibration(output_path)
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, default=ROOT / 'config/object_multimodal_calibration.json')
    parser.add_argument('--clock-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare_profile(args.calibration, args.clock_report, args.output))


if __name__ == '__main__':
    main()
