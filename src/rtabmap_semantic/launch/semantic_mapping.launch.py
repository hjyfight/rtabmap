from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('database_path'),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('semantic_python', default_value='python3'),
        DeclareLaunchArgument('semantic_database_path', default_value=''),
        DeclareLaunchArgument('semantic_device', default_value='cpu'),
        DeclareLaunchArgument('semantic_config', default_value=PathJoinSubstitution([
            FindPackageShare('rtabmap_semantic'), 'config', 'semantic.yaml'])),
        DeclareLaunchArgument('semantic_replay_existing', default_value='true'),
        Node(package='rtabmap_semantic', executable='semantic_mapper',
             prefix=[LaunchConfiguration('semantic_python')], output='screen',
             parameters=[LaunchConfiguration('semantic_config'), {
                 'database_path': LaunchConfiguration('database_path'),
                 'semantic_database_path': LaunchConfiguration('semantic_database_path'),
                 'model_path': LaunchConfiguration('model_path'),
                 'device': LaunchConfiguration('semantic_device')}]),
        Node(package='rtabmap_semantic', executable='keyframe_bridge', output='screen',
             parameters=[{'replay_existing': LaunchConfiguration('semantic_replay_existing')}]),
    ])
