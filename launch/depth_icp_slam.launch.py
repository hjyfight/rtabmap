"""Livox odometry with depth-only geometric loop closures.

The RealSense color stream is deliberately unused. The depth image is
projected to a point cloud, merged with the Livox cloud, and registered by
RTAB-Map with ICP. RTAB-Map's proximity-by-space path then adds geometric
loop links and optimizes the pose graph.
"""

from pathlib import Path

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in (
        '1', 'true', 'yes', 'on')


def launch_setup(context: LaunchContext, *args, **kwargs):
    root = Path(__file__).resolve().parents[1]
    (root / 'maps').mkdir(parents=True, exist_ok=True)
    (root / 'logs').mkdir(parents=True, exist_ok=True)

    frame_id = LaunchConfiguration('frame_id').perform(context)
    lidar_topic = LaunchConfiguration('lidar_topic').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    depth_frame = LaunchConfiguration('depth_frame').perform(context)
    qos = int(LaunchConfiguration('qos').perform(context))
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    localization = _bool(context, 'localization')
    database_path = LaunchConfiguration('database_path').perform(context)
    if not database_path:
        database_path = str(root / 'maps/go2_depth_icp.db')

    extrinsics = [float(LaunchConfiguration(f'depth_{name}').perform(context))
                  for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')]
    extrinsics_warning = []
    if all(abs(value) < 1e-6 for value in extrinsics):
        extrinsics_warning.append(LogInfo(msg=(
            '[DEPTH_ICP] WARNING: depth_* 外参全部为 0。仅用于链路验证；'
            f'正式建图请填写 {frame_id} -> {depth_frame} 的实测外参。')))

    if depth_frame == 'camera_color_optical_frame':
        extrinsics_warning.append(LogInfo(msg=(
            '[DEPTH_ICP] 提示: RealSense驱动通常已发布 camera_link -> camera_color_optical_frame TF。\n'
            '[DEPTH_ICP] 若测量的是雷达到相机机身外参，建议设置 depth_frame:=camera_link 避免TF多父级冲突。'
        )))

    voxel_size_str = LaunchConfiguration('voxel_size').perform(context)
    voxel_size_val = float(voxel_size_str)
    min_loop_overlap_str = LaunchConfiguration('min_loop_closure_overlap').perform(context)
    expected_rate = float(LaunchConfiguration('expected_update_rate').perform(context))
    assembling_time = float(LaunchConfiguration('assembling_time').perform(context))
    topic_q_size = int(LaunchConfiguration('topic_queue_size').perform(context))
    sync_q_size = int(LaunchConfiguration('sync_queue_size').perform(context))

    common = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': qos,
        'approx_sync': True,
        'wait_for_transform': 0.2,
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '20',
        'Icp/VoxelSize': voxel_size_str,
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': '0.5',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.5',
    }

    nodes = [
        # This TF must be replaced with the measured Livox-to-camera extrinsic.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='depth_camera_extrinsic', output='screen',
            arguments=[
                '--x', LaunchConfiguration('depth_x'),
                '--y', LaunchConfiguration('depth_y'),
                '--z', LaunchConfiguration('depth_z'),
                '--roll', LaunchConfiguration('depth_roll'),
                '--pitch', LaunchConfiguration('depth_pitch'),
                '--yaw', LaunchConfiguration('depth_yaw'),
                '--frame-id', frame_id,
                '--child-frame-id', depth_frame,
            ]),
        # Depth image -> XYZ cloud. Decimation keeps the network/ICP load low.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz',
            name='depth_to_cloud', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'qos': qos,
                'qos_camera_info': qos,
                'approx_sync': True,
                'decimation': 2,
                'min_depth': 0.3,
                'max_depth': 8.0,
                'voxel_size': voxel_size_val,
                'filter_nans': True,
            }],
            remappings=[
                ('depth/image', depth_topic),
                ('depth/camera_info', camera_info_topic),
                ('cloud', 'depth_cloud'),
            ]),
        # Livox remains the odometry source; depth is used only for geometric
        # constraints and map coverage.
        Node(
            package='rtabmap_odom', executable='icp_odometry',
            name='icp_odometry', output='screen',
            parameters=[common, {
                'expected_update_rate': expected_rate,
                'always_process_most_recent_frame': False,
                'always_check_imu_tf': False,
                'odom_frame_id': 'icp_odom',
                'publish_tf': True,
                'Odom/ScanKeyFrameThr': '0.6',
                'OdomF2M/ScanSubtractRadius': voxel_size_str,
                'OdomF2M/ScanMaxSize': '15000',
                'OdomF2M/BundleAdjustment': 'false',
                'Icp/CorrespondenceRatio': '0.05',
                'Icp/OutlierRatio': '0.7',
            }],
            remappings=[('scan_cloud', lidar_topic), ('imu', 'imu_not_used')]),
        # Merge the two geometric sensors in the Livox frame. The fixed frame
        # compensates for the small timestamp difference between sensors.
        Node(
            package='rtabmap_util', executable='point_cloud_aggregator',
            name='depth_livox_aggregator', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'qos': qos,
                'count': 2,
                'frame_id': frame_id,
                'fixed_frame_id': 'icp_odom',
                'approx_sync': True,
                'approx_sync_max_interval': 0.25,
                'wait_for_transform': 2.0,
                'xyz_output': True,
            }],
            remappings=[
                ('cloud1', lidar_topic),
                ('cloud2', 'depth_cloud'),
                ('combined_cloud', 'combined_cloud'),
            ]),
        Node(
            package='rtabmap_util', executable='point_cloud_assembler',
            name='combined_cloud_assembler', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'qos': qos,
                'qos_odom': qos,
                'assembling_time': assembling_time,
                'fixed_frame_id': 'icp_odom',
                'range_min': 0.3,
                'wait_for_transform': 2.0,
                'approx_sync': True,
            }],
            remappings=[('cloud', 'combined_cloud'), ('odom', 'icp_odom')]),
    ]

    rtabmap_params = {
        **common,
        'database_path': database_path if database_path else '',
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': True,
        'odom_sensor_sync': False,
        'odom_frame_id': '',
        'Rtabmap/DetectionRate': '1',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/LocalRadius': '5',
        'RGBD/LinearUpdate': '0.2',
        'RGBD/AngularUpdate': '0.2',
        'RGBD/NeighborLinkRefining': 'false',
        'RGBD/OptimizeMaxError': '1.0',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'false',
        'Optimizer/Strategy': '2',
        'Optimizer/Robust': 'true',
        'Optimizer/Iterations': '20',
        'Icp/CorrespondenceRatio': ParameterValue(min_loop_overlap_str, value_type=str),
        'Grid/3D': 'true',
        'Grid/Sensor': '0',
        'Grid/RangeMin': '0.3',
        'Grid/RangeMax': '20',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
    }
    rtabmap_args = [] if localization else ['-d']
    if database_path:
        rtabmap_args.append(database_path)

    nodes.extend([
        Node(
            condition=UnlessCondition(LaunchConfiguration('localization')),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params, {
                'topic_queue_size': topic_q_size,
                'sync_queue_size': sync_q_size,
            }],
            remappings=[('scan_cloud', 'assembled_cloud'),
                        ('odom', 'icp_odom'), ('imu', 'imu_not_used')],
            arguments=rtabmap_args),
        Node(
            condition=IfCondition(LaunchConfiguration('localization')),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[rtabmap_params, {
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                'topic_queue_size': topic_q_size,
                'sync_queue_size': sync_q_size,
            }],
            remappings=[('scan_cloud', 'assembled_cloud'),
                        ('odom', 'icp_odom'), ('imu', 'imu_not_used')]),
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[rtabmap_params, {
                'odometry_node_name': 'icp_odometry',
                'subscribe_rgb': False, 'subscribe_depth': False,
                'subscribe_rgbd': False,
            }],
            remappings=[('scan_cloud', 'assembled_cloud'),
                        ('odom', 'icp_odom'), ('imu', 'imu_not_used')]),
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2', executable='rviz2', name='rtabmap_rviz', output='screen',
            arguments=['-d', str(root / 'config/livox.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]),
        ExecuteProcess(
            cmd=[FindExecutable(name='python3'),
                 str(root / 'scripts/depth_loop_diagnostics.py'),
                 '--log-file', str(root / 'logs/depth_icp_loop_events.jsonl')],
            output='screen'),
    ])
    return extrinsics_warning + nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument('database_path', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='livox_frame'),
        DeclareLaunchArgument('depth_frame', default_value='camera_link'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/aligned_depth_to_color/camera_info'),
        DeclareLaunchArgument('qos', default_value='1'),
        DeclareLaunchArgument('topic_queue_size', default_value='30'),
        DeclareLaunchArgument('sync_queue_size', default_value='30'),
        DeclareLaunchArgument('expected_update_rate', default_value='15.0'),
        DeclareLaunchArgument('voxel_size', default_value='0.08'),
        DeclareLaunchArgument('assembling_time', default_value='0.3'),
        DeclareLaunchArgument('min_loop_closure_overlap', default_value='0.2'),
        DeclareLaunchArgument('rtabmap_viz', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    for name in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        args.append(DeclareLaunchArgument(f'depth_{name}', default_value='0.0'))
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
