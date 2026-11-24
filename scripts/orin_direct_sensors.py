"""Local MID360/RealSense inputs; no Go2 SDK, transport or state topics."""
import ipaddress
import json
from pathlib import Path
import subprocess


def sensor_clock_offset(profile):
    if profile['network'].get('sensor_source', 'go2') == 'orin':
        return float(profile['clock'].get('sensor_clock_offset_s', 0.))
    return float(profile['clock']['orin_minus_go2_s'])


def sensor_commands(root, config):
    lidar_config = root / config['lidar_config']
    return [
        ('lidar', ['ros2', 'run', 'livox_ros_driver2', 'livox_ros_driver2_node', '--ros-args',
            '-r', '__node:=livox_lidar_publisher', '-p', 'xfer_format:=0', '-p', 'multi_topic:=0',
            '-p', 'data_src:=0', '-p', 'publish_freq:=10.0', '-p', 'output_data_type:=0',
            '-p', 'frame_id:=livox_frame', '-p', 'user_config_path:=' + str(lidar_config),
            '-p', 'cmdline_input_bd_code:=livox0000000001']),
        ('camera', ['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
            'camera_name:=camera', 'camera_namespace:=',
            'rgb_camera.color_profile:=424,240,6', 'depth_module.depth_profile:=640,480,6',
            'depth_module.infra_profile:=640,480,6', 'enable_color:=true', 'enable_depth:=true',
            'enable_infra1:=false', 'enable_infra2:=true', 'enable_gyro:=false', 'enable_accel:=false',
            'align_depth.enable:=false', 'enable_sync:=false']),
    ]


def check_direct_hardware(root, config):
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    software, hardware, details = {}, {}, {}
    for package in ('livox_ros_driver2', 'realsense2_camera'):
        try:
            details[package] = get_package_prefix(package)
            software[package] = True
        except PackageNotFoundError:
            software[package] = False
    path = root / config['lidar_config']
    software['lidar_config_present'] = path.is_file()
    if not path.is_file():
        return software, {'lidar_config_valid': False}, details
    data = json.loads(path.read_text())
    host = str(ipaddress.IPv4Address(config['lidar_host_ip']))
    lidar = str(ipaddress.IPv4Address(config['lidar_ip']))
    destinations = data['MID360']['host_net_info']
    software['lidar_config_addresses_match'] = (all(destinations[k] == host for k in
        ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'))
        and any(row['ip'] == lidar for row in data['lidar_configs']))
    interface = config.get('lidar_interface', '')
    hardware['wired_interface_present'] = bool(interface.startswith(('eth', 'en')) and Path('/sys/class/net', interface).is_dir()
        and not Path('/sys/class/net', interface, 'wireless').exists())
    addresses = []
    if hardware['wired_interface_present']:
        rows = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show', 'dev', interface], text=True))
        addresses = [v['local'] for row in rows for v in row.get('addr_info', [])]
    hardware['lidar_host_ip_assigned'] = host in addresses
    hardware['realsense_usb_present'] = any(
        (node / 'idVendor').is_file() and (node / 'idProduct').is_file()
        and (node / 'idVendor').read_text().strip() == '8086'
        and (node / 'idProduct').read_text().strip() == '0b3a'
        for node in Path('/sys/bus/usb/devices').iterdir())
    details.update(lidar_interface=interface, lidar_host_ip=host, lidar_ip=lidar,
                   interface_ipv4_addresses=addresses, acquisition_timing_verified=False)
    return software, hardware, details
