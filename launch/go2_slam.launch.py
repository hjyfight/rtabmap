"""Go2: Livox ICP odometry + RealSense RGB-D mapping, with configurable extrinsics.

Start the sensor drivers first (see README), then:

  ros2 launch go2_slam.launch.py \
      lidar_x:=... lidar_y:=... lidar_z:=... lidar_roll:=... lidar_pitch:=... lidar_yaw:=... \
      camera_x:=... camera_y:=... camera_z:=... camera_roll:=... camera_pitch:=... camera_yaw:=...

The defaults publish identity base_link->sensor transforms, which are only
correct if you really mount both sensors at base_link. Set the measured values
or disable `publish_static_tf` and provide the transforms from your URDF.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _value(context, name):
    return LaunchConfiguration(name).perform(context)


def _flag(context, name):
    return _value(context, name).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    root = '/home/jyfight/Project/rtabmap_slam'
    frame = _value(context, 'frame_id')
    lidar_frame = _value(context, 'lidar_frame')
    lidar_topic = _value(context, 'lidar_topic')
    rgb_topic = _value(context, 'rgb_topic')
    depth_topic = _value(context, 'depth_topic')
    camera_info_topic = _value(context, 'camera_info_topic')
    use_icp = _flag(context, 'use_icp_odom')
    publish_static_tf = _flag(context, 'publish_static_tf')
    use_sim_time = _flag(context, 'use_sim_time')

    nodes = []
    if publish_static_tf:
        for prefix, child in (('lidar', lidar_frame), ('camera', 'camera_link')):
            nodes.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name=f'go2_{prefix}_tf', output='screen',
                arguments=[
                    '--x', _value(context, f'{prefix}_x'),
                    '--y', _value(context, f'{prefix}_y'),
                    '--z', _value(context, f'{prefix}_z'),
                    '--roll', _value(context, f'{prefix}_roll'),
                    '--pitch', _value(context, f'{prefix}_pitch'),
                    '--yaw', _value(context, f'{prefix}_yaw'),
                    '--frame-id', frame,
                    '--child-frame-id', child,
                ]))

    common = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'Icp/PointToPlane': 'true',
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '0.5',
        'Icp/Iterations': '30',
        'Icp/CorrespondenceRatio': '0.1',
    }

    if use_icp:
        nodes.append(Node(
            package='rtabmap_odom', executable='icp_odometry',
            name='icp_odometry', namespace='rtabmap', output='screen',
            parameters=[common, {
                # The ICP node publishes odom->frame_id. Use base_link so the
                # tree stays odom->base_link->livox_frame (one parent per frame).
                'frame_id': ParameterValue(LaunchConfiguration('frame_id'), value_type=str),
                'odom_frame_id': 'odom', 'publish_tf': True, 'qos': 2,
            }],
            remappings=[('scan_cloud', lidar_topic)],
        ))
        odom_topic = '/rtabmap/odom'
    else:
        odom_topic = _value(context, 'odom_topic')

    nodes.append(Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap', namespace='rtabmap', output='screen',
        parameters=[common, {
            'frame_id': frame,
            'odom_frame_id': 'odom',
            'database_path': ParameterValue(LaunchConfiguration('database_path'), value_type=str),
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'qos': 2, 'qos_image': 2, 'qos_camera_info': 2,
            'qos_scan': 2, 'qos_odom': 2,
            'wait_for_transform': 0.2,
            'Reg/Strategy': ParameterValue(LaunchConfiguration('reg_strategy'), value_type=str),
            'Grid/Sensor': ParameterValue(LaunchConfiguration('grid_sensor'), value_type=str),
            'Grid/3D': 'true', 'Grid/RangeMax': '20',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/NeighborLinkRefining': 'true',
            'Mem/NotLinkedNodesKept': 'false',
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('scan_cloud', lidar_topic),
            ('odom', odom_topic),
        ],
    ))

    nodes.append(Node(
        package='rviz2', executable='rviz2', name='rtabmap_rviz',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', f'{root}/config/livox.rviz'],
        parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
    ))
    return nodes


def generate_launch_description():
    args = [
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('use_icp_odom', default_value='true'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),
        DeclareLaunchArgument('reg_strategy', default_value='1'),
        DeclareLaunchArgument('grid_sensor', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('database_path',
                              default_value='/home/jyfight/Project/rtabmap_slam/maps/go2.db'),
    ]
    for prefix in ('lidar', 'camera'):
        for field in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
            args.append(DeclareLaunchArgument(f'{prefix}_{field}', default_value='0.0'))
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
