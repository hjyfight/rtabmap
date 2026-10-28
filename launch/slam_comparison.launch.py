"""Run one comparison arm at a time with live sensors or rosbag playback."""
import datetime
import importlib.util
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'launch/livox_realsense_3dslam.launch.py'
MODES = {
    'lidar': {'use_camera': 'false'},
    'rgbd': {'use_camera': 'true'},
}


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise ValueError('mode 必须为 lidar 或 rgbd；当前主 launch 不支持深度点云直接融合。')
    run_dir = Path(LaunchConfiguration('run_dir').perform(context)).expanduser().resolve()
    # The underlying mapping launch uses -d: never pass it an existing database.
    run_dir.mkdir(parents=True, exist_ok=False)
    options = {name: LaunchConfiguration(name).perform(context) for name in FORWARDED}
    options.update(MODES[mode])
    options.update(database_path=str(run_dir / 'map.db'), localization='false', export_cloud='false')
    (run_dir / 'manifest.json').write_text(json.dumps(
        {'mode': mode, 'launch_arguments': options}, ensure_ascii=False, indent=2))
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(BASE)),
                                 launch_arguments=options.items()),
    ]


# Reuse the original argument declarations, so calibration/topic/ICP options remain available.
_spec = importlib.util.spec_from_file_location('fusion_launch', BASE)
_base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_base)
EXCLUDED = {'database_path', 'localization', 'export_cloud', 'use_camera'}
DECLARATIONS = [a for a in _base.generate_launch_description().entities
                if isinstance(a, DeclareLaunchArgument) and a.name not in EXCLUDED]
FORWARDED = [a.name for a in DECLARATIONS]


def generate_launch_description():
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return LaunchDescription(DECLARATIONS + [
        DeclareLaunchArgument('mode', default_value='lidar',
                              description='lidar=雷达+IMU，无回环；rgbd=雷达+IMU，RGB-D辅助回环。'),
        DeclareLaunchArgument('run_dir', default_value=str(ROOT / 'logs/comparison' / stamp),
                              description='全新的结果目录；已有目录会拒绝启动，避免覆盖结果。'),
        OpaqueFunction(function=setup),
    ])
