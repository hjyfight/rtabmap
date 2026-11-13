"""Real Go2 lidar -> Orin ICP/RTAB-Map -> standard MITI display topics."""
import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    common = {
        'frame_id': 'livox_frame', 'use_sim_time': False,
        'Icp/PointToPlane': 'true', 'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '0.5', 'Icp/Iterations': '30',
        'Icp/CorrespondenceRatio': '0.1',
    }
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value=str(
            root / 'maps' / ('orin_display_test_' + stamp + '.db'))),
        Node(
            package='rtabmap_odom', executable='icp_odometry',
            name='icp_odometry', output='screen',
            parameters=[common, {
                'odom_frame_id': 'icp_odom', 'publish_tf': True, 'qos': 2,
                # ICP-only F2M needs a pose guess after its initialization frames.
                'Odom/GuessMotion': 'true',
                'log_to_rosout_level': 2,
            }],
            remappings=[('scan_cloud', '/livox/lidar'), ('odom', '/icp_odom')],
        ),
        Node(
            package='rtabmap_slam', executable='rtabmap',
            name='rtabmap', output='screen',
            parameters=[common, {
                'database_path': ParameterValue(
                    LaunchConfiguration('database_path'), value_type=str),
                'subscribe_depth': False, 'subscribe_rgb': False,
                'subscribe_scan_cloud': True, 'approx_sync': True,
                'qos_scan': 2, 'qos_odom': 2,
                'Reg/Strategy': '1', 'Grid/Sensor': '0', 'Grid/RangeMax': '20',
                'RGBD/ProximityBySpace': 'true',
                'RGBD/NeighborLinkRefining': 'true',
                'Mem/NotLinkedNodesKept': 'false',
            }],
            remappings=[('scan_cloud', '/livox/lidar'), ('odom', '/icp_odom')],
        ),
    ])
