#!/usr/bin/env python3
"""Capture the local WHEELTEC USB microphone and classify sound on Orin."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import yaml

from orin_network_perception import environment, load_profile, supervise

ROOT = Path(__file__).resolve().parents[1]


def check_environment(device, env):
    python = ROOT / '.venv-yamnet/bin/python3'
    match = re.fullmatch(r'(?:plug)?hw:CARD=([A-Za-z0-9_]+),DEV=([0-9]+)', device)
    if not match:
        raise ValueError('Use a stable ALSA name, e.g. hw:CARD=XFMDPV0018,DEV=0')
    card, number = match.groups()
    capture = Path('/proc/asound') / card / ('pcm' + number + 'c/info')
    checks = {
        'usb_capture_present': capture.is_file(),
        'yamnet_python_present': python.is_file(),
        'yamnet_model_present': (ROOT / 'models/yamnet/yamnet.tflite').is_file(),
        'ros_messages_present': (ROOT / 'install/features/local_setup.bash').is_file(),
    }
    report = dict(ready=False, alsa_device=device, ros_domain_id=int(env['ROS_DOMAIN_ID']),
                  checks=checks, errors=[])
    if not all(checks.values()):
        report['errors'].append('Connect WHEELTEC and run bash scripts/install_yamnet.sh first')
        return report
    for name, command in [
        ('ros_imports', [str(python), '-c',
                        'import rclpy; from rtabmap_features_msgs.msg import AudioFrame; from std_msgs.msg import String']),
        ('model_check', [str(python), str(ROOT / 'scripts/predict_yamnet_audio.py'), '--check'])
    ]:
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
        checks[name] = result.returncode == 0
        if result.returncode:
            report['errors'].append(name + ': ' + result.stderr[-3000:])
        elif name == 'model_check':
            report[name] = json.loads(result.stdout)
    report['ready'] = all(checks.values())
    return report


def check_publishers(env):
    # Query DDS directly; the ROS CLI daemon may belong to a different domain.
    code = '''import json, time, rclpy
rclpy.init()
node = rclpy.create_node('orin_audio_preflight')
try:
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    print(json.dumps({topic: [info.node_name for info in node.get_publishers_info_by_topic(topic)]
        for topic in ['/multimodal/audio', '/perception/audio/events']}))
finally:
    node.destroy_node()
    rclpy.shutdown()
'''
    result = subprocess.run([str(ROOT / '.venv-yamnet/bin/python3'), '-c', code],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=15, check=True)
    publishers = json.loads(result.stdout)
    if any(publishers.values()):
        raise ValueError('Audio publisher already exists in this ROS domain; reuse or stop it first: ' +
                         json.dumps(publishers))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/orin_network_perception.yaml')
    parser.add_argument('--alsa-device', default='hw:CARD=XFMDPV0018,DEV=0')
    parser.add_argument('--domain', type=int, help='Override the profile ROS domain (default: 74)')
    parser.add_argument('--check', action='store_true', help='Check USB/ROS/model without opening the microphone')
    parser.add_argument('--host', action='store_true', help='Use the workstation ROS environment instead of the Orin network profile')
    parser.add_argument('--localization', action='store_true', help='Also listen for M260C speech-wakeup direction events')
    args = parser.parse_args()
    if args.domain is not None and not 0 <= args.domain <= 232:
        parser.error('Require ROS domain 0..232')
    if args.host:
        env = dict(os.environ)
        env['ROS_DOMAIN_ID'] = str(args.domain if args.domain is not None else 0)
    else:
        profile = load_profile(args.profile)
        if args.domain is not None:
            profile['network']['ros_domain_id'] = args.domain
        if profile['network']['audio_transport']:
            parser.error('Local USB capture requires network.audio_transport: false to avoid a second audio source')
        env = environment(profile)
    report = check_environment(args.alsa_device, env)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if not report['ready'] or args.check:
        return 0 if report['ready'] else 1
    base = ROOT / 'logs/audio/yamnet' / ('host' if args.host else 'orin')
    base.mkdir(parents=True, exist_ok=True)
    device_key = re.sub(r'[^A-Za-z0-9_.-]', '_', args.alsa_device)
    with (base / (device_key + '.lock')).open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('This WHEELTEC device already has an Orin audio run') from None
        check_publishers(env)
        run = base / (time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
        run.mkdir()
        (run / 'check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        (run / 'supervisor.json').write_text(json.dumps(dict(pid=os.getpid(), domain=int(env['ROS_DOMAIN_ID']),
            alsa_device=args.alsa_device, profile=str(args.profile.resolve())), indent=2) + '\n')
        # Local audio uses the Orin clock, so the sensor transport clock offset does not apply.
        (run / 'yamnet.yaml').write_text(yaml.safe_dump({'yamnet_audio_events': {'ros__parameters': {
            'audio_clock_offset_s': 0.0, 'journal_path': str(run / 'events.jsonl')}}}))
        python = str(ROOT / '.venv-yamnet/bin/python3')
        frame_id = 'host_microphone_array' if args.host else 'orin_microphone_array'
        commands = [
            ('capture', [python, '-u', str(ROOT / 'scripts/publish_fourmodal_audio.py'),
                         '--alsa-device', args.alsa_device, '--frame-id', frame_id]),
            ('yamnet', ['bash', str(ROOT / 'scripts/start_yamnet_audio.sh'),
                        '--ros-args', '--params-file', str(run / 'yamnet.yaml')]),
        ]
        if args.localization:
            commands.append(('direction', [python, '-u', str(ROOT/'scripts/m260c_localization.py'),
                                          '--frame-id', frame_id, '--journal', str(run/'direction.jsonl')]))
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop)
        try:
            supervise(commands, run, env)
        except KeyboardInterrupt:
            print('Stopped Orin microphone and YAMNet. Logs: ' + str(run), flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
