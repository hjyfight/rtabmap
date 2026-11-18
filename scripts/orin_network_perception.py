#!/usr/bin/env python3
"""Configurable Orin receive/perception entry; never operates robot-side drivers."""
import argparse
import fcntl
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_profile(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get('schema') != 1:
        raise ValueError('Invalid Orin profile schema')
    for section in ('network', 'perception', 'mapping'):
        if not isinstance(data.get(section), dict):
            raise ValueError('Missing profile section: ' + section)
    network, perception, mapping = (data[k] for k in ('network', 'perception', 'mapping'))
    if not isinstance(network.get('interface'), str) or not re.fullmatch(r'[A-Za-z0-9_.:-]+', network['interface']):
        raise ValueError('Invalid network interface')
    for key in ('sensor_host', 'display_host'):
        ipaddress.IPv4Address(network[key])
    for key, lower, upper in (('sensor_port', 1, 65535), ('ros_domain_id', 0, 232)):
        value = network[key]
        if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
            raise ValueError('Invalid ' + key)
    if not isinstance(network.get('audio_transport'), bool):
        raise ValueError('audio_transport must be boolean')
    for key in ('sensor_clock_offset_s', 'audio_clock_offset_s', 'clock_uncertainty_s'):
        value = network[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Invalid clock configuration')
    if network['clock_uncertainty_s'] < 0 or not isinstance(network.get('clock_method'), str):
        raise ValueError('Invalid clock provenance')
    if perception.get('device') != 'cuda:0':
        raise ValueError('This Orin profile requires cuda:0')
    for key in ('rgb_max_hz', 'max_input_age_s', 'features_period_s', 'fusion_period_s'):
        value = perception[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Invalid perception rate/age setting: ' + key)
    if not isinstance(perception.get('require_all_modalities'), bool):
        raise ValueError('require_all_modalities must be boolean')
    if isinstance(perception.get('max_objects'), bool) or not isinstance(perception.get('max_objects'), int) or not 1 <= perception['max_objects'] <= 16:
        raise ValueError('max_objects must be 1..16')
    if mapping.get('semantic_backend') not in ('emsanet', 'yoloe_pf'):
        raise ValueError('Unknown semantic backend')
    extrinsics = mapping.get('camera_to_lidar')
    if extrinsics is not None and (not isinstance(extrinsics, list) or len(extrinsics) != 6 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in extrinsics)):
        raise ValueError('camera_to_lidar requires six finite values')
    if not isinstance(mapping.get('calibration_version'), str):
        raise ValueError('calibration_version must be a string')
    return data


def runtime_parameters(profile):
    n, p, m = (profile[k] for k in ('network', 'perception', 'mapping'))
    offset = float(n['sensor_clock_offset_s'])
    return {
        'fourmodal_features': {'ros__parameters': {
            'device': p['device'], 'min_anchor_period_s': float(p['features_period_s']),
            'max_input_age_s': float(p['max_input_age_s']),
            'allow_partial': not p['require_all_modalities'],
            'clock_offsets_s': [offset, offset, offset, float(n['audio_clock_offset_s'])],
            'calibration_version': m['calibration_version']}},
        'yolo_segmentation': {'ros__parameters': {
            'device': p['device'], 'max_inference_hz': float(p['rgb_max_hz']),
            'max_input_age_s': float(p['max_input_age_s']), 'rgb_clock_offset_s': offset,
            'clock_offset_uncertainty_s': float(n['clock_uncertainty_s']),
            'clock_offset_method': n['clock_method'], 'fusion_mode': 'shadow'}},
        'nir_segmentation': {'ros__parameters': {'device': p['device']}},
        'segmentation_fusion': {'ros__parameters': {
            'device': p['device'], 'min_period_s': float(p['fusion_period_s']),
            'max_objects': p['max_objects']}},
        'object_multimodal_fusion': {'ros__parameters': {
            'device': p['device'], 'min_period_s': float(p['fusion_period_s']),
            'max_input_age_s': float(p['max_input_age_s'])}}}


def environment(profile):
    n = profile['network']
    if not Path('/sys/class/net', n['interface']).exists():
        raise ValueError('Network interface does not exist: ' + n['interface'])
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(n['ros_domain_id']), ROS_LOCALHOST_ONLY='0',
               RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', PYTHONNOUSERSITE='1',
               OMP_NUM_THREADS='2', OPENBLAS_NUM_THREADS='2', RTABMAP_PROJECT_ROOT=str(ROOT),
               YOLO_CONFIG_DIR=str(ROOT / 'models/.ultralytics-orin'),
               MPLCONFIGDIR=str(ROOT / 'models/.matplotlib-orin'))
    env['CYCLONEDDS_URI'] = (
        '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="' + n['interface'] +
        '"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>'
        '<Peers><Peer Address="' + n['display_host'] + '"/><Peer Address="127.0.0.1"/>'
        '</Peers></Discovery></Domain></CycloneDDS>')
    return env


def deployment_check(root=ROOT):
    required = ['.venv-segmentation-gpu/bin/python3', '.venv-fourmodal/bin/python3',
                '.venv-yoloe/bin/python3', '.tools/unibind/.source-revision',
                'install/features/local_setup.bash', 'install/segmentation/local_setup.bash',
                'install/semantic/local_setup.bash', 'logs/orin_perception/environment.json',
                'models/yoloe-11s-seg-pf.pt', 'models/unibind/pretrained_weights.manifest.json']
    required += ['scripts/run_object_multimodal_fusion.py','config/object_multimodal_fusion.yaml',
                 'config/object_multimodal_task.json','config/object_multimodal_calibration.json']
    checks = {name: (root / name).is_file() for name in required}
    checkpoint = root / 'models/unibind/pretrained_weights.pt'
    manifest = root / 'models/unibind/pretrained_weights.manifest.json'
    checks['unibind_checkpoint_complete'] = (checkpoint.is_file() and manifest.is_file() and
        checkpoint.stat().st_size == json.loads(manifest.read_text())['bytes'])
    return dict(deployment_ready=all(checks.values()), checks=checks)


def commands_for(action, profile, run, root=ROOT):
    n = profile['network']
    python = str(root / '.venv-segmentation-gpu/bin/python3')
    if action == 'receive':
        command = ['/usr/bin/python3', '-u', str(root / 'scripts/hotspot_sensor_transport.py'),
                   'receive', '--host', n['sensor_host'], '--port', str(n['sensor_port']),
                   '--socket-timeout', '60']
        if n['audio_transport']:
            command.append('--audio')
        return [('receive', command)]
    if action == 'lidar':
        return [('lidar', ['ros2', 'launch', str(root / 'launch/orin_lidar_display_test.launch.py')])]
    if action == 'lidar-imu':
        return [('lidar_imu', ['ros2', 'launch', str(root / 'launch/livox_realsense_3dslam.launch.py'),
            'use_camera:=false', 'use_imu:=true', 'filter_imu:=true', 'deskewing:=true',
            'assemble:=false', 'qos:=2', 'qos_imu:=2', 'rtabmap_viz:=false',
            'loop_status_window:=false', 'database_path:=' + str(run / 'map.db')])]
    if action == 'semantic':
        if profile['mapping']['camera_to_lidar'] is None:
            raise ValueError('Fill mapping.camera_to_lidar with measured mounting extrinsics before semantic mapping')
        script = 'start_orin_slam.sh' if profile['mapping']['semantic_backend'] == 'emsanet' else 'start_semantic_yoloe.sh'
        args = [] if script == 'start_orin_slam.sh' else ['--gpu']
        return [('semantic', ['bash', str(root / 'scripts' / script)] + args +
                ['rtabmap_viz:=false', 'loop_status_window:=false'])]
    runtime = str(run / 'runtime.yaml')
    def node(script, configs):
        cmd = [python, str(root / 'scripts' / script), '--ros-args']
        for config in configs:
            cmd += ['--params-file', str(root / 'config' / config)]
        return cmd + ['--params-file', runtime]
    if action=='target-fusion':
        return [('targets',node('run_object_multimodal_fusion.py',['object_multimodal_fusion.yaml']))]
    if action=='target-perception':
        return [('rgb',node('run_yolo_segmentation.py',['yolo_segmentation.yaml','yoloe_segmentation.yaml'])),
                ('targets',node('run_object_multimodal_fusion.py',['object_multimodal_fusion.yaml']))]
    return [
        ('rgb', node('run_yolo_segmentation.py', ['yolo_segmentation.yaml', 'yoloe_segmentation.yaml'])),
        ('nir', node('run_nir_segmentation.py', ['segmentation_fusion.yaml'])),
        ('fusion', node('run_segmentation_fusion.py', ['segmentation_fusion.yaml'])),
        ('features', node('run_fourmodal_features.py', ['fourmodal_features.yaml']))]


def supervise(commands, run, env):
    children, outputs = [], []
    try:
        for name, command in commands:
            output = (run / (name + '.log')).open('w')
            outputs.append(output)
            child = subprocess.Popen(command, cwd=ROOT, env=env, stdout=output,
                                     stderr=subprocess.STDOUT, start_new_session=True)
            children.append((name, child))
        (run / 'workers.json').write_text(json.dumps([
            dict(name=name, pid=child.pid) for name, child in children], indent=2) + '\n')
        print('Started Orin workers. Logs: ' + str(run), flush=True)
        while all(child.poll() is None for _, child in children):
            time.sleep(.5)
        raise RuntimeError('Worker exited: ' + ', '.join(
            f'{name}={child.returncode}' for name, child in children if child.poll() is not None))
    finally:
        for _, child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for _, child in children:
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        for output in outputs:
            output.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', nargs='?', default='check',
                        choices=['check', 'receive', 'perception', 'target-fusion', 'target-perception', 'lidar', 'lidar-imu', 'semantic'])
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/orin_network_perception.yaml')
    args = parser.parse_args()
    profile = load_profile(args.profile)
    check = deployment_check()
    if args.action == 'check':
        check.update(profile=str(args.profile.resolve()), network=profile['network'],
                     camera_extrinsics_configured=profile['mapping']['camera_to_lidar'] is not None,
                     robot_connection_opened=False)
        print(json.dumps(check, indent=2))
        return 0 if check['deployment_ready'] else 1
    if args.action in ('perception', 'target-fusion', 'target-perception', 'semantic') and not check['deployment_ready']:
        raise ValueError('Run check and complete the missing environment/model/build files first')
    env = environment(profile)
    run = ROOT / 'logs/orin_perception' / f'{args.action}_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}'
    run.mkdir(parents=True)
    lock = (ROOT / 'logs/orin_perception' / f'{args.action}_domain_{env["ROS_DOMAIN_ID"]}.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    (run / 'runtime.yaml').write_text(yaml.safe_dump(runtime_parameters(profile)))
    commands = commands_for(args.action, profile, run)
    if args.action == 'semantic':
        for key, value in zip(('CAM_X', 'CAM_Y', 'CAM_Z', 'CAM_ROLL', 'CAM_PITCH', 'CAM_YAW'),
                              profile['mapping']['camera_to_lidar']):
            env[key] = str(value)
    if args.action in ('perception','target-perception'):
        registered = subprocess.run(['/usr/bin/python3', str(ROOT / 'scripts/check_registered_depth.py')],
                                    env=env, cwd=ROOT, capture_output=True, text=True, check=True)
        if registered.stdout.strip() == 'start':
            commands.insert(0, ('depth_registration', ['ros2', 'launch',
                str(ROOT / 'launch/realsense_depth_registration.launch.py')]))
        elif registered.stdout.strip() != 'reuse':
            raise ValueError('Cannot determine registered depth publisher state')
    def stop_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        supervise(commands, run, env)
    except KeyboardInterrupt:
        print('Stopped this Orin run. Logs: ' + str(run))
    finally:
        lock.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
