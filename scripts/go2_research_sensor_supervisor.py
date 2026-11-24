#!/usr/bin/env python3
"""Own only the Go2 camera/Livox/PCM and their wireless transport processes."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--root', type=Path, default=Path.home() / 'go2_orin_network')
    args = parser.parse_args()
    root = args.root.resolve()
    logs = root / 'logs'
    logs.mkdir(parents=True, exist_ok=True)
    with (root / 'sensors.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for item in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                exe = item.read_bytes().split(b'\0')[0]
                if exe.endswith((b'/livox_ros_driver2_node', b'/realsense2_camera_node')):
                    raise RuntimeError('A sensor driver is already running: ' + item.parent.name)
            except OSError:
                pass
        run = logs / time.strftime('%Y%m%d_%H%M%S')
        run.mkdir()
        # This overlay is passed last by rs_launch. Keep the private run settings
        # separate from the user's camera config, which enabled both IR streams.
        camera_config = run / 'camera.yaml'
        camera_config.write_text('''/camera/camera:
  ros__parameters:
    color_qos: SENSOR_DATA
    color_info_qos: DEFAULT
    depth_qos: DEFAULT
    depth_info_qos: DEFAULT
    enable_color: true
    enable_depth: true
    enable_infra1: false
    enable_infra2: true
    align_depth.enable: false
    enable_sync: false
''')
        commands = [
            ('lidar', ['/usr/bin/python3', '/opt/ros/foxy/bin/ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py']),
            ('camera', ['/usr/bin/python3', '/opt/ros/foxy/bin/ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                "config_file:='" + str(camera_config) + "'",
                'rgb_camera.profile:=424x240x6', 'depth_module.profile:=640x480x6',
                'align_depth.enable:=false', 'enable_color:=true', 'enable_depth:=true',
                'enable_infra1:=false', 'enable_infra2:=true', 'enable_sync:=false']),
            ('audio', ['pasuspender', '--', '/usr/bin/python3', '-u', str(root / 'scripts/publish_fourmodal_audio.py'),
                '--alsa-device', 'hw:CARD=XFMDPV0018,DEV=0', '--frame-id', 'go2_microphone_array',
                '--window-seconds', '.5', '--publish-period', '.1']),
            ('transport', ['/usr/bin/python3', '-u', str(root / 'scripts/hotspot_sensor_transport.py'),
                'send', '--host', args.host, '--port', '7447', '--audio', '--jpeg-quality', '0', '--socket-timeout', '60']),
        ]
        children, outputs, restarts = {}, {}, {}
        def close_group(child):
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        def spawn(name, command):
            child = subprocess.Popen(command, stdout=outputs[name], stderr=subprocess.STDOUT,
                                     start_new_session=True)
            children[name] = child
            return child
        def save_status():
            info = dict(pid=os.getpid(), host=args.host, ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                        run=str(run), children={name: child.pid for name, child in children.items()},
                        restarts=restarts)
            temporary = root / 'running.json.tmp'
            temporary.write_text(json.dumps(info, indent=2))
            temporary.replace(root / 'running.json')
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            for name, command in commands:
                output = (run / (name + '.log')).open('w')
                outputs[name] = output
                restarts[name] = 0
                spawn(name, command)
            save_status()
            print('Go2 wireless sensors started: ' + str(run), flush=True)
            while True:
                for name, command in commands:
                    child = children[name]
                    if child.poll() is not None:
                        print(name + ' exited (' + str(child.returncode) + '); restarting in 3s', flush=True)
                        close_group(child)
                        time.sleep(3)
                        restarts[name] += 1
                        spawn(name, command)
                        save_status()
                time.sleep(.5)
        except KeyboardInterrupt:
            pass
        finally:
            for child in children.values():
                close_group(child)
            for output in outputs.values():
                output.close()


if __name__ == '__main__':
    main()
