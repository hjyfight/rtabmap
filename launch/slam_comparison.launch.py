"""Run one comparison arm at a time with live sensors or rosbag playback."""
import datetime
import importlib.util
import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'launch/livox_realsense_3dslam.launch.py'
MODES = {
    'lidar': {'use_camera': 'false', 'use_color_loop': 'false', 'fuse_depth': 'false'},
    'depth': {'use_camera': 'true', 'use_color_loop': 'false', 'fuse_depth': 'true'},
    'rgbd': {'use_camera': 'true', 'use_color_loop': 'true', 'fuse_depth': 'false'},
}


def setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise ValueError('mode must be lidar, depth or rgbd')
    run_dir = Path(LaunchConfiguration('run_dir').perform(context)).expanduser().resolve()
    # The underlying mapping launch uses -d: never pass it an existing database.
    run_dir.mkdir(parents=True, exist_ok=False)
    options = {name: LaunchConfiguration(name).perform(context) for name in FORWARDED}
    options.update(MODES[mode])
    options.update(database_path=str(run_dir / 'map.db'), localization='false',
                   export_cloud='false', publish_map_cloud='true', diagnostics='false',
                   geometry_loop='true')
    (run_dir / 'manifest.json').write_text(json.dumps(
        {'mode': mode, 'launch_arguments': options}, ensure_ascii=False, indent=2))
    return [
        ExecuteProcess(cmd=[FindExecutable(name='python3'),
                            str(ROOT / 'scripts/record_slam_comparison.py'),
                            '--output', str(run_dir), '--ros-args', '-p',
                            'use_sim_time:=' + options['use_sim_time']], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(BASE)),
                                 launch_arguments=options.items()),
    ]


# Reuse the original argument declarations, so calibration/topic/ICP options remain available.
_spec = importlib.util.spec_from_file_location('fusion_launch', BASE)
_base = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_base)
EXCLUDED = {'database_path', 'localization', 'export_cloud', 'publish_map_cloud',
            'diagnostics', 'use_camera', 'use_color_loop', 'fuse_depth', 'geometry_loop'}
DECLARATIONS = [a for a in _base.generate_launch_description().entities
                if isinstance(a, DeclareLaunchArgument) and a.name not in EXCLUDED]
FORWARDED = [a.name for a in DECLARATIONS]


def generate_launch_description():
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return LaunchDescription(DECLARATIONS + [
        DeclareLaunchArgument('mode', default_value='lidar',
                              description='lidar=纯雷达；depth=原代码深度融合；rgbd=视觉+深度回环'),
        DeclareLaunchArgument('run_dir', default_value=str(ROOT / 'logs/comparison' / stamp),
                              description='全新的结果目录；已有目录会拒绝启动，避免覆盖结果。'),
        OpaqueFunction(function=setup),
    ])
