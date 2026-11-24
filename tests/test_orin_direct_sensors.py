import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from orin_direct_sensors import sensor_commands, sensor_clock_offset
from orin_research_wireless import research_commands


def test_direct_profile_has_no_go2_endpoint_or_clock_dependency():
    profile = json.loads((ROOT / 'config/orin_direct_research.json').read_text())
    assert profile['network']['sensor_source'] == 'orin'
    assert 'go2_host' not in profile['network']
    assert sensor_clock_offset(profile) == 0.
    assert profile['audio']['source'] == 'orin'


def test_direct_livox_is_pointcloud2_and_camera_uses_humble_parameter_names():
    profile = json.loads((ROOT / 'config/orin_direct_research.json').read_text())
    commands = dict(sensor_commands(ROOT, profile['local_sensors']))
    assert 'xfer_format:=0' in commands['lidar']
    assert 'user_config_path:=' + str(ROOT / 'config/orin_mid360_direct.json') in commands['lidar']
    assert 'rgb_camera.color_profile:=424,240,6' in commands['camera']
    assert 'depth_module.depth_profile:=640,480,6' in commands['camera']
    assert 'camera_namespace:=' in commands['camera']
    assert 'enable_gyro:=false' in commands['camera']
    assert 'enable_accel:=false' in commands['camera']
    assert all('unitree' not in part and '7447' not in part for command in commands.values() for part in command)


def test_existing_wireless_source_keeps_its_measured_clock_offset():
    profile = json.loads((ROOT / 'config/go2_orin_research_network.json').read_text())
    assert sensor_clock_offset(profile) == profile['clock']['orin_minus_go2_s']


def test_complete_direct_startup_needs_no_robot_or_workstation_endpoint(tmp_path):
    profile = json.loads((ROOT / 'config/orin_direct_research.json').read_text())
    commands = dict(research_commands(ROOT, profile, tmp_path))
    assert {'lidar', 'camera', 'audio', 'slam', 'semantic', 'research', 'results'} <= commands.keys()
    assert 'receive' not in commands
    assert 'display_send' not in commands
    # The existing semantic SLAM launch owns its single RGB-D synchronizer.
    assert 'rgbd' not in commands
    assert 'database_path:=' + str(tmp_path / 'map.db') in commands['slam']
    assert 'database_path:=' + str(tmp_path / 'map.db') in commands['semantic']
    assert 'semantic_enabled:=true' in commands['slam']
    assert all('hotspot_sensor_transport' not in arg and '192.168.1.' not in arg
               and '192.168.123.18' not in arg for command in commands.values() for arg in command)


def test_complete_wireless_startup_keeps_remote_receiver_and_single_semantic_sync(tmp_path):
    profile = json.loads((ROOT / 'config/go2_orin_research_network.json').read_text())
    commands = dict(research_commands(ROOT, profile, tmp_path))
    assert profile['network']['go2_host'] in commands['receive']
    assert str(profile['network']['sensor_port']) in commands['receive']
    assert {'slam', 'semantic', 'research', 'display_send'} <= commands.keys()
    assert 'lidar' not in commands and 'camera' not in commands and 'rgbd' not in commands
