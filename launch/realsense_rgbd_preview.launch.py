"""Provide registered RGB-D input for MITI while mapping is stopped."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_image_proc', executable='register_node', name='depth_registration',
            output='screen',
            parameters=[{'queue_size': 30, 'use_sim_time': False,
                         'use_rgb_timestamp': False, 'fill_upsampling_holes': False}],
            remappings=[('depth/image_rect', '/camera/depth/image_rect_raw'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth_registered/image_rect', '/camera/depth_registered/image_rect'),
                        ('depth_registered/camera_info', '/camera/depth_registered/camera_info')]),
        Node(
            package='rtabmap_sync', executable='rgbd_sync', name='depth_preview_rgbd_sync', output='screen',
            parameters=[{'approx_sync': True, 'approx_sync_max_interval': 0.05,
                         'qos': 2, 'qos_camera_info': 2, 'sync_queue_size': 30,
                         'topic_queue_size': 10, 'use_sim_time': False}],
            remappings=[('rgb/image', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image', '/camera/depth_registered/image_rect'),
                        ('rgbd_image', '/depth_preview/rgbd_image')]),
    ])
