"""Existing Livox/RealSense SLAM with object labels in its native 3D window."""
from pathlib import Path
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value=str(project / 'maps' / f'semantic_{stamp}.db')),
        DeclareLaunchArgument('model_path', default_value=str(project / 'models' / 'yolov8n-seg.pt')),
        DeclareLaunchArgument('semantic_python', default_value=str(project / '.venv-semantic' / 'bin' / 'python3')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(project / 'launch' / 'livox_realsense_3dslam.launch.py')),
            launch_arguments={'use_camera': 'true', 'semantic_enabled': 'true',
                              'database_path': LaunchConfiguration('database_path')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(project / 'src' /
            'rtabmap_semantic' / 'launch' / 'semantic_mapping.launch.py')),
            # Required child arguments must be explicit, even when already declared above.
            launch_arguments={
                'database_path': LaunchConfiguration('database_path'),
                'model_path': LaunchConfiguration('model_path'),
                'semantic_python': LaunchConfiguration('semantic_python'),
            }.items()),
    ])
