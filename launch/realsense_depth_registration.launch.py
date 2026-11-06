"""Register native rectified depth using camera calibration and optical-frame TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    defaults = {
        'raw_depth_topic': '/camera/depth/image_rect_raw',
        'raw_depth_info_topic': '/camera/depth/camera_info',
        'camera_info_topic': '/camera/color/camera_info',
        'depth_topic': '/camera/depth_registered/image_rect',
        'registered_depth_info_topic': '/camera/depth_registered/camera_info',
        'queue_size': '30',
        'use_sim_time': 'false',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        Node(
            package='depth_image_proc', executable='register_node',
            name='depth_registration', output='screen',
            parameters=[{'queue_size': LaunchConfiguration('queue_size'),
                         'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'use_rgb_timestamp': False, 'fill_upsampling_holes': False}],
            remappings=[
                ('depth/image_rect', LaunchConfiguration('raw_depth_topic')),
                ('depth/camera_info', LaunchConfiguration('raw_depth_info_topic')),
                ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
                ('depth_registered/image_rect', LaunchConfiguration('depth_topic')),
                ('depth_registered/camera_info', LaunchConfiguration('registered_depth_info_topic')),
            ],
        ),
    ])
