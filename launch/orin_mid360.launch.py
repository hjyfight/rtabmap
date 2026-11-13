"""MID360 PointCloud2 + its built-in IMU; requires an actual network config."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', description='MID360 JSON with measured device and Orin Ethernet IPs'),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen', parameters=[{
                 'xfer_format': 0, 'multi_topic': 0, 'data_src': 0,
                 'publish_freq': 10.0, 'output_data_type': 0,
                 'frame_id': 'livox_frame',
                 'user_config_path': LaunchConfiguration('config_path'),
                 'cmdline_input_bd_code': 'livox0000000001',
             }]),
    ])
