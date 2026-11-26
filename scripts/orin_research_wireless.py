#!/usr/bin/env python3
"""Orin SLAM/research with either Go2 transport or directly connected sensors."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from orin_network_perception import environment, supervise
from fusion_research_display import display_commands
from orin_direct_sensors import (sensor_commands, check_direct_hardware, sensor_clock_offset,
                                camera_lidar_extrinsics_current, use_current_extrinsics,
                                semantic_mapping_enabled)

ROOT = Path(__file__).resolve().parents[1]


def research_commands(root, profile, run):
    """Build the complete process graph without opening devices or launching jobs."""
    n = profile['network']
    source = n.get('sensor_source', 'go2')
    sensor_qos = '1' if source == 'orin' else '2'
    audio = profile.get('audio', {'source': 'go2'})
    semantic = profile.get('semantic', {'enabled': False})
    extrinsics_current = camera_lidar_extrinsics_current(profile)
    current_parameters = use_current_extrinsics(profile)
    semantic_enabled = semantic_mapping_enabled(profile)
    mapping = profile.get('mapping', {})
    database = root / mapping['database_path'] if mapping.get('database_path') else run / 'map.db'
    runtime, public = run / 'runtime.json', run / 'public'
    if source == 'orin':
        commands = sensor_commands(root, profile['local_sensors'])
    elif source == 'go2':
        commands = [('receive', ['/usr/bin/python3', '-u', str(root / 'scripts/hotspot_sensor_transport.py'),
            'receive', '--host', n['go2_host'], '--port', str(n['sensor_port']), '--audio', '--socket-timeout', '60'])]
    else:
        raise ValueError('network.sensor_source must be go2 or orin')
    commands.extend([
        ('depth_registration', ['ros2', 'launch', str(root / 'launch/realsense_depth_registration.launch.py')]),
        ('slam', ['ros2', 'launch', str(root / 'launch/livox_realsense_3dslam.launch.py'),
            'use_camera:=' + str(semantic_enabled).lower(),
            'semantic_enabled:=' + str(semantic_enabled).lower(),
            'register_depth:=false', 'publish_camera_tf:=false', 'loop_diagnostics_enabled:=false',
            'use_imu:=true', 'filter_imu:=true', 'deskewing:=true', 'assemble:=false',
            'qos:=' + sensor_qos, 'qos_imu:=' + sensor_qos,
            'rtabmap_viz:=false', 'loop_status_window:=false',
            'database_path:=' + str(database)] +
            (['delete_db_on_start:=false'] if mapping.get('database_path') else []) +
            (['odom_initial_pose:=' + ' '.join(str(float(v)) for v in mapping['initial_pose'])]
             if mapping.get('initial_pose') is not None else [])),
        ('research', [str(root / '.venv-indoor-research/bin/python3'), '-u',
            str(root / 'scripts/run_fusion_research.py'), '--config', str(runtime)]),
        ('results', ['/usr/bin/python3', '-u', str(root / 'scripts/research_result_transport.py'),
            'serve', '--directory', str(public), '--host', n['orin_host'], '--port', str(n['results_port'])]),
    ])
    if extrinsics_current or current_parameters:
        commands.append(('camera_tf', ['/usr/bin/python3', '-u', str(root / 'scripts/research_camera_tf.py'),
            '--config', str(runtime), '--report', str(run / 'camera_tf.json')]))
    if semantic_enabled:
        # The existing SLAM launch owns rgbd_sync and its validation guard.
        commands.append(('semantic', ['ros2', 'launch',
            str(root / 'src/rtabmap_semantic/launch/semantic_mapping.launch.py'),
            'database_path:=' + str(database),
            'model_path:=' + str(root / semantic['weights']),
            'semantic_python:=' + str(root / '.venv-segmentation-gpu/bin/python3'),
            'semantic_device:=cuda:0', 'semantic_backend:=' + semantic['backend'],
            'semantic_config:=' + str(root / semantic['config'])]))
    else:
        commands.append(('rgbd', ['ros2', 'run', 'rtabmap_sync', 'rgbd_sync', '--ros-args',
            '-p', 'qos:=2', '-p', 'approx_sync:=true', '-p', 'approx_sync_max_interval:=0.05',
            '-r', 'rgb/image:=/camera/color/image_raw', '-r', 'rgb/camera_info:=/camera/color/camera_info',
            '-r', 'depth/image:=/camera/depth_registered/image_rect', '-r', 'rgbd_image:=/rgbd_image']))
    commands.extend(display_commands(root))
    if audio['source'] == 'orin':
        commands.insert(0, ('audio', ['pasuspender', '--', '/usr/bin/python3', '-u',
            str(root / 'scripts/publish_fourmodal_audio.py'), '--alsa-device', audio['alsa_device'],
            '--topic', audio.get('topic', '/multimodal/audio/orin'), '--frame-id', 'orin_microphone_array',
            '--window-seconds', '.5', '--publish-period', '.1']))
    if n.get('enable_display_transport', True):
        commands.append(('display_send', ['/usr/bin/python3', '-u', str(root / 'scripts/research_display_transport.py'),
            'send', '--host', n['orin_host'], '--port', str(n['display_port']), '--socket-timeout', '10']))
    return commands


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/go2_orin_research_network.json')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--require-hardware', action='store_true', help='Make --check fail when local hardware is not connected')
    args = parser.parse_args()
    profile = json.loads(args.profile.read_text())
    n, clock = profile['network'], profile['clock']
    source = n.get('sensor_source', 'go2')
    if source not in ('go2', 'orin'):
        raise ValueError('network.sensor_source must be go2 or orin')
    audio = profile.get('audio', {'source': 'go2'})
    semantic = profile.get('semantic', {'enabled': False})
    extrinsics_current = camera_lidar_extrinsics_current(profile)
    semantic_enabled = semantic_mapping_enabled(profile)
    current_parameters = use_current_extrinsics(profile)
    if semantic_enabled and semantic['backend'] != 'yoloe_pf':
        raise ValueError('This wireless semantic profile requires the existing YOLOE backend')
    if audio['source'] not in ('go2', 'orin'):
        raise ValueError('audio.source must be go2 or orin')
    if source == 'orin' and audio['source'] != 'orin':
        raise ValueError('Local Orin mode requires local Orin audio')
    required = ['.venv-indoor-research/bin/python3', 'install/features/local_setup.bash',
                'models/odin_scannet_swin_semantic.pth.download', 'models/ptv3_scannet_best.pth',
                'models/avsegformer_res50_avss.pth', 'models/yolov8s-seg.pt']
    checks = {name: (ROOT / name).is_file() for name in required}
    if current_parameters:
        checks['current_calibration_present'] = (ROOT / profile['local_sensors']['current_calibration_path']).is_file()
    mapping = profile.get('mapping', {})
    if mapping.get('database_path'):
        checks['resume_database_present'] = (ROOT / mapping['database_path']).is_file()
    if mapping.get('initial_pose') is not None:
        pose=mapping['initial_pose']
        if len(pose)!=6 or not all(math.isfinite(float(v)) for v in pose):
            raise ValueError('initial_pose requires six finite XYZ/RPY values')
    hardware, direct_details = {}, {}
    if source == 'orin':
        direct_checks, hardware, direct_details = check_direct_hardware(ROOT, profile['local_sensors'])
        checks.update(direct_checks)
    if semantic_enabled:
        for name in ('.venv-segmentation-gpu/bin/python3', semantic['weights'], semantic['config'],
                     'install/semantic/local_setup.bash',
                     'install/semantic/rtabmap_semantic/lib/rtabmap_semantic/keyframe_bridge'):
            checks[name] = (ROOT / name).is_file()
    if audio['source'] == 'orin':
        hardware['orin_usb_audio_capture'] = Path('/proc/asound/XFMDPV0018/pcm0c/info').is_file()
    preview = research_commands(ROOT, profile, ROOT / 'logs/go2_orin_research/preflight')
    if args.check:
        ready = all(checks.values()) and all(hardware.values())
        print(json.dumps(dict(ready=ready, software_ready=all(checks.values()), hardware_ready=all(hardware.values()),
            checks=checks, hardware=hardware, direct_details=direct_details, network=n, clock=clock,
            sensor_source=source, camera_lidar_extrinsics_current=extrinsics_current,
            use_current_extrinsics=current_parameters,
            semantic_enabled=semantic_enabled,
            command_names=[name for name, _ in preview]), indent=2))
        return 0 if all(checks.values()) and (not args.require_hardware or ready) else 1
    if not all(checks.values()):
        raise RuntimeError('Missing Orin research environment/models: ' + str(checks))
    if not all(hardware.values()):
        raise RuntimeError('Connect local sensors and configure the MID360 Ethernet interface first: ' + str(hardware))
    env = environment({'network': dict(interface='lo', display_host='127.0.0.1',
                                       ros_domain_id=n['ros_domain_id'])})
    env['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/>'
        '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
        '<MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
        '</Peers></Discovery></Domain></CycloneDDS>')
    env.update(CUDA_HOME='/usr/local/cuda', CUMM_CUDA_ARCH_LIST='8.7',
               PATH='/usr/local/cuda/bin:' + env['PATH'])
    base = ROOT / 'logs/go2_orin_research'
    base.mkdir(parents=True, exist_ok=True)
    with (base / 'domain74.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run = base / (time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
        run.mkdir()
        public = run / 'public'
        public.mkdir()
        cfg = json.loads((ROOT / 'config/fusion_research.json').read_text())
        sensor_offset = sensor_clock_offset(profile)
        cfg.update(public_directory=str(public), clock_mode='configured_network_profile',
                   rgb_clock_offset_s=sensor_offset, nir_clock_offset_s=sensor_offset,
                   lidar_clock_offset_s=sensor_offset,
                   audio_clock_offset_s=sensor_offset if audio['source'] == 'go2' else 0.,
                   clock_alignment=dict(clock, sensor_capture_host=source, audio_capture_host=audio['source'],
                       audio_clock_offset_s=sensor_offset if audio['source'] == 'go2' else 0.),
                   publish_ros_snapshots=False, camera_lidar_mount_changed=not extrinsics_current)
        cfg['use_current_extrinsics']=current_parameters
        if current_parameters:
            cfg['current_calibration_path']=profile['local_sensors']['current_calibration_path']
        if audio['source'] == 'orin':
            # Use a separate topic so a later Go2 reconnect cannot mix two
            # acquisition clocks or microphones into the same PCM timeline.
            cfg['audio_topic'] = audio.get('topic', '/multimodal/audio/orin')
        runtime = run / 'runtime.json'
        runtime.write_text(json.dumps(cfg, indent=2))
        (base / 'current_run.json').write_text(json.dumps(dict(pid=os.getpid(), run=str(run),
            profile=str(args.profile.resolve()),
            database_path=str(ROOT / mapping['database_path']) if mapping.get('database_path') else str(run/'map.db'),
            results_url='http://' + n['orin_host'] + ':' + str(n['results_port'])), indent=2))
        library = ROOT / 'build/dynamic_cloud/libdynamic_cloud.so'
        if not library.exists() or (ROOT / 'scripts/dynamic_cloud_filter.cpp').stat().st_mtime > library.stat().st_mtime:
            subprocess.run(['bash', str(ROOT / 'scripts/build_dynamic_cloud.sh')], env=env, check=True)
        commands = research_commands(ROOT, profile, run)
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            supervise(commands, run, env)
        except KeyboardInterrupt:
            print('Stopped this Orin research run: ' + str(run), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
