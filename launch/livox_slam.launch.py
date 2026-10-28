"""PointCloud2 -> ICP odometry -> RTAB-Map, using the lidar frame as base."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    frame = LaunchConfiguration('frame_id')
    cloud = LaunchConfiguration('lidar_topic')
    common = {
        'frame_id': ParameterValue(frame, value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'Icp/PointToPlane': 'true',
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '0.5',
        'Icp/Iterations': '30',
        'Icp/CorrespondenceRatio': '0.1',
    }
    return LaunchDescription([
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('database_path', default_value=str(root / 'maps/livox.db')),
        Node(
            package='rtabmap_odom', executable='icp_odometry',
            name='icp_odometry', namespace='rtabmap', output='screen',
            parameters=[common, {
                'odom_frame_id': 'odom', 'publish_tf': True, 'qos': 2,
            }],
            remappings=[('scan_cloud', cloud)],
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap',
            name='rtabmap', namespace='rtabmap', output='screen',
            parameters=[common, {
                'database_path': ParameterValue(LaunchConfiguration('database_path'), value_type=str),
                'subscribe_depth': False, 'subscribe_rgb': False,
                'subscribe_scan_cloud': True, 'approx_sync': True,
                'qos_scan': 2, 'qos_odom': 2,
                'Reg/Strategy': '1', 'Grid/Sensor': '0', 'Grid/RangeMax': '20',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/NeighborLinkRefining': 'true',
                'Mem/NotLinkedNodesKept': 'false',
            }],
            remappings=[('scan_cloud', cloud)],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rtabmap_rviz',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', str(root / 'config/livox.rviz')],
            parameters=[{'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}],
        ),
    ])
