# Description:
#   RGB + RGB-D + LiDAR 三者融合 3D SLAM
#   专为 Livox LiDAR (MID360/Avia/HAP) + RealSense D435i 组合设计
#
#   Livox 是非重复扫描LiDAR，点云随时间累积密度增加，因此
#   搭配 point_cloud_assembler 可获得更加稠密的点云。
#
#   数据融合策略：
#   - Livox LiDAR  → ICP odometry（精确几何里程计）+ 稠密点云建图
#   - RealSense RGB → 视觉特征提取，用于回环检测
#   - RealSense Depth → 为视觉特征提供3D深度，辅助回环验证
#   - RealSense IMU  → LiDAR运动畸变校正（deskewing）
#
# Requirements:
#   - Livox ROS2 driver:   livox_ros_driver2
#   - RealSense ROS2 driver: ros-$ROS_DISTRO-realsense2-camera
#   - rtabmap_ros
#
# Usage:
#   【方式一：标准模式（每帧处理）】
#   1. 启动 Livox
#     $ ros2 launch livox_ros_driver2 msg_MID360_launch.py
#   2. 启动 RealSense
#     $ ros2 launch realsense2_camera rs_launch.py \
#         align_depth.enable:=true enable_gyro:=true enable_accel:=true \
#         unite_imu_method:=2 rgb_camera.profile:=640x360x30
#   3. 启动融合 3D SLAM
#     $ ros2 launch rtabmap_examples livox_realsense_3dslam.launch.py
#
#   【方式二：装配模式（累积点云，推荐）】
#     $ ros2 launch rtabmap_examples livox_realsense_3dslam.launch.py assemble:=true
#
#   【方式三：纯定位模式】
#     $ ros2 launch rtabmap_examples livox_realsense_3dslam.launch.py \
#         localization:=true database_path:=/path/to/rtabmap.db
#
#   【方式四：无相机纯LiDAR 3D SLAM】
#     $ ros2 launch rtabmap_examples livox_realsense_3dslam.launch.py use_camera:=false
#
#   【方式五：点云着色导出（RGB-D 给 LiDAR 点云着色）】
#     正常建图结束后（数据库已保存），单独运行着色导出：
#     $ ros2 launch rtabmap_examples livox_realsense_3dslam.launch.py \
#         export_cloud:=true database_path:=~/.ros/rtabmap.db
#     在数据库目录生成 <name>_cloud.ply：全局 LiDAR 点云（--scan）投影到
#     相机帧取色（--cam_projection），每个点带 RGB 颜色。
#     加 export_mesh:=true 可同时导出带纹理的 mesh（OBJ + 纹理图）。
#     也可以在 rtabmap_viz 中用 File > Export 3D clouds... 交互式导出。

import os
import math

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):

    frame_id = LaunchConfiguration('frame_id')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Livox LiDAR 参数 ----
    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + '/deskewed'

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    # ---- RealSense / 相机参数 ----
    use_camera = LaunchConfiguration('use_camera').perform(context)
    use_camera = use_camera == 'true' or use_camera == 'True'

    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    # ---- IMU 参数 ----
    imu_topic = LaunchConfiguration('imu_topic')
    imu_topic_value = imu_topic.perform(context)
    raw_imu_topic = LaunchConfiguration('raw_imu_topic')
    raw_imu_topic_value = raw_imu_topic.perform(context)
    imu_used = imu_topic_value != ''

    # ---- GUI 配置文件 ----
    use_intensity_coloring = LaunchConfiguration('use_intensity_coloring').perform(context)
    use_intensity_coloring = use_intensity_coloring == 'true' or use_intensity_coloring == 'True'
    gui_cfg = LaunchConfiguration('gui_cfg').perform(context)
    if use_intensity_coloring and not gui_cfg:
        gui_cfg = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'livox_realsense_gui.ini')

    # ---- 装配模式 ----
    assemble = LaunchConfiguration('assemble').perform(context)
    assemble = assemble == 'true' or assemble == 'True'

    # ---- Deskewing ----
    deskewing = LaunchConfiguration('deskewing').perform(context)
    deskewing = deskewing == 'true' or deskewing == 'True'

    # ---- 是否启用 IMU ----
    use_imu = LaunchConfiguration('use_imu').perform(context)
    use_imu = use_imu == 'true' or use_imu == 'True'

    # 用户禁用IMU 或 无相机且未显式配置IMU → 禁用
    if not use_imu:
        imu_used = False
        deskewing = False
    elif not use_camera and imu_topic_value == '/imu/data':
        imu_used = False
        deskewing = False

    deskewing_slerp = LaunchConfiguration('deskewing_slerp').perform(context)
    deskewing_slerp = deskewing_slerp == 'true' or deskewing_slerp == 'True'

    # ---- 相机-LiDAR 外参（静态 TF）----
    cam_to_lidar_x = float(LaunchConfiguration('cam_to_lidar_x').perform(context))
    cam_to_lidar_y = float(LaunchConfiguration('cam_to_lidar_y').perform(context))
    cam_to_lidar_z = float(LaunchConfiguration('cam_to_lidar_z').perform(context))
    cam_to_lidar_roll = float(LaunchConfiguration('cam_to_lidar_roll').perform(context))
    cam_to_lidar_pitch = float(LaunchConfiguration('cam_to_lidar_pitch').perform(context))
    cam_to_lidar_yaw = float(LaunchConfiguration('cam_to_lidar_yaw').perform(context))

    # 诊断：检查相机-LiDAR 外参是否已标定
    cam_extrinsics_all_zero = (
        abs(cam_to_lidar_x) < 0.001 and abs(cam_to_lidar_y) < 0.001 and
        abs(cam_to_lidar_z) < 0.001 and abs(cam_to_lidar_roll) < 0.001 and
        abs(cam_to_lidar_pitch) < 0.001 and abs(cam_to_lidar_yaw) < 0.001
    )
    if cam_extrinsics_all_zero:
        log_warn = [
            LogInfo(msg='\n'
            '=============================================\n'
            '[COLOR_DIAG] 相机-LiDAR 外参全为零！\n'
            '[COLOR_DIAG] 这会导致 RGB 着色位置严重错位。\n'
            '[COLOR_DIAG] 请测量并传入真实外参，例如：\n'
            '[COLOR_DIAG]   cam_to_lidar_x:=0.15 cam_to_lidar_y:=0.0 cam_to_lidar_z:=-0.05\n'
            '[COLOR_DIAG]   cam_to_lidar_roll:=0.0  cam_to_lidar_pitch:=0.0  cam_to_lidar_yaw:=0.0\n'
            '=============================================\n'),
        ]
    else:
        log_warn = []

    # ---- 外部里程计 ----
    external_odom_frame_id = LaunchConfiguration('external_odom_frame_id').perform(context)

    # ---- 固定参考系（用于 deskewing）----
    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id:
        if external_odom_frame_id:
            fixed_frame_id = external_odom_frame_id
        elif imu_used:
            fixed_frame_from_imu = True
            fixed_frame_id = frame_id.perform(context) + '_stabilized'

    # 强制禁用 deskewing（简化链路）
    deskewing = False
    if not fixed_frame_id or not deskewing:
        lidar_topic_deskewed = lidar_topic

    # ---- 定位/建图模式 ----
    localization_cond = LaunchConfiguration('localization')
    localization_value = localization_cond.perform(context)
    localization_value = localization_value == 'true' or localization_value == 'True'

    database_path = LaunchConfiguration('database_path').perform(context)
    if database_path:
        database_path = os.path.expanduser(database_path)

    # ---- 着色导出模式（RGB-D 给 LiDAR 点云着色）----
    # 独立模式：映射结束后单独运行，用相机图像给数据库中的全局 LiDAR
    # 点云着色并导出 PLY。此模式下不启动任何 SLAM 节点。
    export_cloud = LaunchConfiguration('export_cloud').perform(context)
    export_cloud = export_cloud == 'true' or export_cloud == 'True'
    if export_cloud:
        database_path_value = database_path
        if not database_path_value:
            database_path_value = os.path.expanduser('~/.ros/rtabmap.db')

        export_dir_value = LaunchConfiguration('export_dir').perform(context)
        export_mesh_value = LaunchConfiguration('export_mesh').perform(context)
        export_mesh_value = export_mesh_value == 'true' or export_mesh_value == 'True'
        export_texture_size_value = LaunchConfiguration('export_texture_size').perform(context)
        export_opt_value = LaunchConfiguration('export_opt').perform(context)

        # --scan: 用 LiDAR 扫描装配全局点云（而非 RGB-D 深度）
        # --cam_projection: 把点投影到相机帧取 RGB 颜色
        # --texture_d2c: 每个点优先用离它最近的相机取色
        export_cmd = [
            FindExecutable(name='rtabmap-export'),
            '--scan',
            '--cloud',
            '--cam_projection',
            '--texture_d2c',
            '--opt', export_opt_value,
        ]
        if export_mesh_value:
            # 同时导出带纹理的 mesh（OBJ + 纹理图）
            export_cmd += ['--mesh', '--texture', '--texture_size', export_texture_size_value]
        if export_dir_value:
            export_cmd += ['--output_dir', export_dir_value]
        export_cmd.append(database_path_value)

        output_base = os.path.splitext(os.path.basename(database_path_value))[0]
        msg = ('点云着色导出（RGB-D → LiDAR）: %s -> %s_cloud.ply（相机颜色）'
               % (database_path_value, output_base))
        if not use_camera:
            msg += '。警告：use_camera=false，数据库中无图像，点云不会被着色。'

        diag_msg = (
            '\n=============================================\n'
            '[COLOR_DIAG] 离线着色导出诊断信息\n'
            '[COLOR_DIAG] 数据库路径   : %s\n'
            '[COLOR_DIAG] 导出命令     : %s\n'
            '[COLOR_DIAG] LiDAR话题    : %s\n'
            '[COLOR_DIAG] RGB话题      : %s\n'
            '[COLOR_DIAG] Depth话题    : %s\n'
            '[COLOR_DIAG] CameraInfo   : %s\n'
            '[COLOR_DIAG] 外参 X/Y/Z   : %.3f / %.3f / %.3f (m)\n'
            '[COLOR_DIAG] 外参 R/P/Y   : %.3f / %.3f / %.3f (rad)\n'
            '[COLOR_DIAG] use_camera   : %s\n'
            '[COLOR_DIAG] assemble模式 : %s\n'
            '[COLOR_DIAG] 着色方式     : --scan (扫描装配) + --cam_projection (投影着色) + --texture_d2c (最近相机)\n'
            '[COLOR_DIAG] 位姿优化     : --opt %s\n'
            '=============================================\n'
        ) % (
            database_path_value,
            ' '.join(export_cmd),
            lidar_topic.perform(context),
            rgb_topic.perform(context),
            depth_topic.perform(context),
            camera_info_topic.perform(context),
            cam_to_lidar_x, cam_to_lidar_y, cam_to_lidar_z,
            cam_to_lidar_roll, cam_to_lidar_pitch, cam_to_lidar_yaw,
            str(use_camera),
            LaunchConfiguration('assemble').perform(context),
            export_opt_value,
        )

        return log_warn + [
            LogInfo(msg=diag_msg),
            LogInfo(msg=msg),
            ExecuteProcess(cmd=export_cmd, output='screen'),
        ]

    # ============================================================
    #  参数配置
    # ============================================================
    max_correspondence_distance = voxel_size_value * 10.0

    # 共享参数（ICP相关，odometry和slam共用）
    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': LaunchConfiguration('qos'),
        'approx_sync': True,  # 近似同步容忍时间戳不完美对齐
        'wait_for_transform': 0.2,
        # ---- 6DoF 3D 核心参数 ----
        'Reg/Force3DoF': 'false',
        # ---- ICP 配准参数 ----
        'Icp/PointToPlane': 'true',
        'Icp/Iterations': '20',
        'Icp/VoxelSize': str(voxel_size_value),
        'Icp/Epsilon': '0.001',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/MaxTranslation': '3',
        'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.5',
    }

    # ICP 里程计专有参数
    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'always_process_most_recent_frame': False,
        'always_check_imu_tf': False,  # IMU TF不存在时跳过检查, 否则ICP输出全零
        'deskewing': not fixed_frame_id and deskewing,
        'odom_frame_id': 'icp_odom',
        'guess_frame_id': fixed_frame_id if not fixed_frame_id and deskewing else '',
        'deskewing_slerp': deskewing_slerp,
        # ---- Livox 里程计参数 ----
        'Odom/ScanKeyFrameThr': '0.15',          # 降低关键帧阈值，更多约束抑制角漂移
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'true',       # 局部BA抑制roll/pitch累积
        'Icp/CorrespondenceRatio': '0.01',
        'Icp/PointToPlane': 'true',               # 面特征约束提升角精度
    }
    # wait_imu_to_init 已移除 —— IMU 数据不传给 rtabmap/icp_odometry（未标定会引入地图倾斜）

    # RTAB-Map SLAM 参数
    rtabmap_parameters = {
    	'database_path': database_path if database_path else os.path.expanduser('~/.ros/rtabmap.db'),
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': use_camera,
        'subscribe_odom_info': not external_odom_frame_id,
        'subscribe_scan_cloud': True,
        'odom_frame_id': (external_odom_frame_id if external_odom_frame_id else ''),
        'odom_sensor_sync': use_camera,  # 需要 rgbd_image 同步，相机链路有问题时阻塞 rtabmap 启动
        # ---- 回环检测频率（装配模式下降低但仍启用）----
        'Rtabmap/DetectionRate': ('1' if assemble else '1'),
        # ---- 3D 回环检测 ----
        'RGBD/ProximityBySpace': 'true',    # false=用视觉词袋做回环（外观匹配），true=用空间距离
        'RGBD/ProximityMaxGraphDepth': '0',     # 0=无限搜索，不限制图深度
        'RGBD/ProximityPathMaxNeighbors': '10', # 回环搜索邻近节点数
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'false',
        'Grid/3D': 'true',
        'Grid/FromDepth': 'false',  # 从LiDAR点云建几何，RGB图像仅着色
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.3',
        'Grid/FullUpdate': 'false',
        'Grid/RayTracing': 'false',
        # ---- 记忆体 ----
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        # ---- 配准策略 ----
        'Reg/Strategy': '2' if use_camera else '1',  # 配准策略：2=回环检测用视觉+ICP（最鲁棒）
        'Reg/Force3DoF': 'false',
        # ---- 重力约束（IMU 找平，0=禁用。不使用 IMU 的情况下 Force3DoF 防止 roll/pitch 漂移）----
        'Optimizer/GravitySigma': '0',
        # ---- 回环检测 ----
        'Icp/CorrespondenceRatio': str(LaunchConfiguration('min_loop_closure_overlap').perform(context)),
        # ---- 视觉回环 ----
        'Kp/MaxFeatures': '400',
        'Vis/MaxFeatures': '600',
        'Vis/InlierDistance': '0.1',
        'Vis/MinInliers': '10',
        'Mem/RehearsalSimilarity': '0.15',   # 视觉相似度阈值（越低越容易检出回环，默认0.2）
        # ---- 使用深度信息校正视觉特征 ----
        'Kp/WordsUseDepthCorrection': 'true',
    }

    # ============================================================
    #  Remappings 和 Arguments
    # ============================================================
    # 共享 remappings（rtabmap/icp_odometry/rtabmap_viz 共用）
    # icp_odometry 需要 IMU 初始化（但不用于建图方向）。
    # rtabmap 的 IMU 在后面单独覆盖为 imu_not_used，避免 GravitySigma 污染。
    remappings = [('odom', 'icp_odom')]
    if imu_used:
        remappings.append(('imu', imu_topic.perform(context)))
    else:
        remappings.append(('imu', 'imu_not_used'))

    if use_camera:
        # rgbd_image 是由 rgbd_sync 输出的已同步 RGB+D+CameraInfo 组合消息
        remappings.append(('rgbd_image', 'rgbd_image'))

    rtabmap_args = []
    if not localization_value:
        rtabmap_args.append('-d')
    if database_path:
        rtabmap_args.append(database_path)

    # 可视化的点云话题
    scan_viz_topic = lidar_topic_deskewed
    if assemble:
        scan_viz_topic = 'assembled_cloud'

    # ============================================================
    #  节点构建
    # ============================================================
    nodes = []

    # ----- RGB-D Sync（同步RGB+Depth用于视觉回环检测）-----
    if use_camera:
        nodes.append(
            Node(
                package='rtabmap_sync', executable='rgbd_sync', output='screen',
                parameters=[{
                    'approx_sync': True,  # 近似同步容忍时间戳不完美对齐
                    'approx_sync_max_interval': 10.0,  # RGB/深度时间戳差可达数秒(传感器时钟源不同)
                    'use_sim_time': use_sim_time,
                    'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                    'topic_queue_size': LaunchConfiguration('topic_queue_size'),
                }],
                remappings=[
                    ('rgb/image', rgb_topic.perform(context)),
                    ('rgb/camera_info', camera_info_topic.perform(context)),
                    ('depth/image', depth_topic.perform(context)),
                ],
            )
        )

    # ----- Madgwick IMU 滤波器（计算 IMU 方向四元数，用于 deskewing 和重力约束）-----
    # Livox MID360 自带 IMU 不输出方向四元数，需要通过 Madgwick 滤波器估计。
    # 滤波后输出 /imu/data → rtabmap 将其用于 deskewing 和 Optimizer/GravitySigma 约束。
    if imu_used:
        nodes.append(
            Node(
                package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
                parameters=[{
                    'use_mag': False,
                    'world_frame': 'enu',
                    'publish_tf': False,
                }],
                remappings=[('imu/data_raw', raw_imu_topic_value)],
            )
        )

    # ----- IMU 重力方向稳定参考系（用于 deskewing）-----
    # deskewing 已强制禁用 → imu_to_tf 不需要。如果将来启用 deskewing 再打开。
    if fixed_frame_from_imu and deskewing:
        nodes.append(
            Node(
                package='rtabmap_util', executable='imu_to_tf', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'base_frame_id': frame_id,
                    'wait_for_transform_duration': 0.001,
                }],
                remappings=[('imu/data', imu_topic.perform(context))],
            )
        )

    # ----- LiDAR Deskewing（运动畸变校正）-----
    if fixed_frame_id and deskewing:
        nodes.append(
            Node(
                package='rtabmap_util', executable='lidar_deskewing', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'wait_for_transform': 0.2,
                    'slerp': deskewing_slerp,
                }],
                remappings=[('input_cloud', lidar_topic_value)],
            )
        )

    # ----- Livox 点云装配（累积非重复扫描点云）-----
    if assemble:
        nodes.append(
            Node(
                package='rtabmap_util', executable='point_cloud_assembler', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'assembling_time': LaunchConfiguration('assembling_time'),
                    'fixed_frame_id': (external_odom_frame_id if external_odom_frame_id else 'icp_odom'),
                    'range_min': 0.3,
                    'topic_queue_size': LaunchConfiguration('topic_queue_size'),
                    'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                    'wait_for_transform': 10.0,
                    'approx_sync': True,  # 近似同步容忍时间戳不完美对齐
                }],
                remappings=[
                    ('cloud', lidar_topic_deskewed),
                    ('odom', 'icp_odom'),
                ],
            )
        )

    # ----- ICP Odometry（LiDAR里程计）-----
    if not external_odom_frame_id:
        nodes.append(
            Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[shared_parameters, icp_odometry_parameters],
                remappings=remappings + [('scan_cloud', lidar_topic_deskewed),
                                         ('imu', 'imu_not_used')],   # IMU未标定干扰ICP输出
                arguments=['--ros-args', '--log-level', 'info'],
            )
        )

    # ----- RTAB-Map SLAM（融合节点）-----
    # rtabmap 通过 subscribe_rgbd=true 接收 rgbd_sync 输出的 rgbd_image，
    # 通过 subscribe_scan_cloud=true 接收 LiDAR 点云（或装配后的云），
    # 内部使用 rgbdScan3d 同步器将两者精确对齐。

    # 建图模式
    nodes.append(
        Node(
            condition=UnlessCondition(localization_cond),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'rgbd_cameras': 1,
                         'topic_queue_size': LaunchConfiguration('topic_queue_size'),
                         'sync_queue_size': LaunchConfiguration('sync_queue_size')}],
            remappings=remappings + [('scan_cloud',
                                      'assembled_cloud' if assemble else lidar_topic_deskewed),
                                     ('imu', 'imu_not_used')],   # 阻断IMU: 未标定不污染建图方向
            arguments=rtabmap_args,
        )
    )

    # 定位模式
    nodes.append(
        Node(
            condition=IfCondition(localization_cond),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'Mem/IncrementalMemory': 'False',
                         'Mem/InitWMWithAllNodes': 'True',
                         'rgbd_cameras': 1,
                         'topic_queue_size': LaunchConfiguration('topic_queue_size'),
                         'sync_queue_size': LaunchConfiguration('sync_queue_size')}],
            remappings=remappings + [('scan_cloud',
                                  'assembled_cloud' if assemble else lidar_topic_deskewed),
                                 ('imu', 'imu_not_used')],
        )
    )

    # ----- 可视化 ----
    # rtabmap_viz 不订阅 RGB-D 数据，避免未标定外参导致地图显示乱色。
    # RGB-D 数据仍然由 rtabmap SLAM 节点订阅，用于视觉回环检测。
    # gui_cfg 配置文件将 3D 云的默认着色模式设为强度（intensity）而非 RGB。
    rtabmap_viz_args = []
    if gui_cfg:
        rtabmap_viz_args.append(gui_cfg)
    nodes.append(
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'odometry_node_name': 'icp_odometry',
                         'subscribe_rgbd': False,
                         'subscribe_rgb': False,
                         'subscribe_depth': False}],
            remappings=remappings + [('scan_cloud', scan_viz_topic),
                                     ('imu', 'imu_not_used')],
            arguments=rtabmap_viz_args,
        )
    )

    # ----- 回环检测诊断节点（监控相机链路 & 回环事件）-----
    if use_camera:
        nodes.append(
            ExecuteProcess(
                cmd=[FindExecutable(name='python3'),
                     '/home/jyfight/Project/rtabmap_slam/scripts/loop_closure_diagnostics.py'],
                output='screen',
            )
        )

    # ----- 静态 TF 变换（LiDAR → 相机）-----
    # RealSense 自动发布 camera_link → camera_color_optical_frame
    # 我们只需要补上 livox_frame → camera_link 这段
    if use_camera:
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', str(cam_to_lidar_x),
                    '--y', str(cam_to_lidar_y),
                    '--z', str(cam_to_lidar_z),
                    '--roll', str(cam_to_lidar_roll),
                    '--pitch', str(cam_to_lidar_pitch),
                    '--yaw', str(cam_to_lidar_yaw),
                    '--frame-id', frame_id.perform(context),
                    '--child-frame-id', 'camera_link',
                ],
                output='screen',
            )
        )

    camera_yaw_deg = math.degrees(cam_to_lidar_yaw)
    camera_pitch_deg = math.degrees(cam_to_lidar_pitch)
    camera_roll_deg = math.degrees(cam_to_lidar_roll)

    diag_info = LogInfo(msg=(
        '\n=============================================\n'
        '[COLOR_DIAG] 建图参数摘要\n'
        '[COLOR_DIAG] 外参 X/Y/Z       : %.3f / %.3f / %.3f (m)\n'
        '[COLOR_DIAG] 外参 R/P/Y       : %.3f / %.3f / %.3f (rad) = %.1f / %.1f / %.1f (deg)\n'
        '[COLOR_DIAG] LiDAR话题        : %s\n'
        '[COLOR_DIAG] RGB话题          : %s\n'
        '[COLOR_DIAG] Depth话题        : %s\n'
        '[COLOR_DIAG] CameraInfo       : %s\n'
        '[COLOR_DIAG] assemble模式     : %s\n'
        '[COLOR_DIAG] deskewing        : %s (强制禁用)\n'
        '[COLOR_DIAG] odom_sensor_sync : True (时间戳对齐)\n'
        '[COLOR_DIAG] voxel_size       : %s\n'
        '[COLOR_DIAG] 固定参考系       : %s\n'
        '[COLOR_DIAG] -------------------------------------------------\n'
        '[COLOR_DIAG] IMU 数据链路:\n'
        '[COLOR_DIAG]   原始数据 : %s\n'
        '[COLOR_DIAG]   Madgwick : %s\n'
        '[COLOR_DIAG]   输出话题 : %s (→ rtabmap)\n'
        '[COLOR_DIAG]   使用IMU   : %s\n'
        '[COLOR_DIAG]   GravitySigma : 0 (禁用)\n'
        '[COLOR_DIAG]   IMU→rtabmap : 已阻断\n'
        '[COLOR_DIAG]   imu_to_tf   : 不启动 (deskewing=off, 不需要)\n'
        '[COLOR_DIAG] -------------------------------------------------\n'
        '[COLOR_DIAG] TF链: %s -> camera_link -> camera_color_optical_frame\n'
        '[COLOR_DIAG]   相机光轴在livox_frame中:\n'
        '[COLOR_DIAG]     optical Z(前) = livox X(前) 旋转后方向\n'
        '[COLOR_DIAG]   验证命令: ros2 run tf2_ros tf2_echo %s camera_color_optical_frame\n'
        '[COLOR_DIAG] -------------------------------------------------\n'
        '=============================================\n'
    ) % (
        cam_to_lidar_x, cam_to_lidar_y, cam_to_lidar_z,
        cam_to_lidar_roll, cam_to_lidar_pitch, cam_to_lidar_yaw,
        camera_roll_deg, camera_pitch_deg, camera_yaw_deg,
        lidar_topic.perform(context),
        rgb_topic.perform(context),
        depth_topic.perform(context),
        camera_info_topic.perform(context),
        LaunchConfiguration('assemble').perform(context),
        str(deskewing),
        str(voxel_size_value),
        fixed_frame_id if fixed_frame_id else '无（deskewing禁用时不使用）',
        raw_imu_topic.perform(context),
        '已启用（imu_filter_madgwick）' if imu_used else '已禁用',
        imu_topic.perform(context) if imu_used else '无（IMU已禁用）',
        str(imu_used),
        frame_id.perform(context),
        frame_id.perform(context),
    ))

    return log_warn + [diag_info] + nodes


def generate_launch_description():
    return LaunchDescription([

        # ============================================================
        #  通用参数
        # ============================================================
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock.'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='定位模式（需要已有数据库）。'),

        DeclareLaunchArgument(
            'database_path', default_value='',
            description='RTAB-Map 数据库路径 (如 ~/.ros/rtabmap.db)。'),

        DeclareLaunchArgument(
            'frame_id', default_value='livox_frame',
            description='LiDAR/机器人 base frame。'),

        DeclareLaunchArgument(
            'qos', default_value='1',
            description='QoS: 0=系统默认, 1=Reliable, 2=Best Effort.'),

        DeclareLaunchArgument(
            'topic_queue_size', default_value='30',
            description='话题队列大小。'),

        DeclareLaunchArgument(
            'sync_queue_size', default_value='30',
            description='同步队列大小。'),

        DeclareLaunchArgument(
            'use_intensity_coloring', default_value='true',
            description='使用 LiDAR 强度着色（红色梯度）替代 RGB 着色。设为 false 恢复默认 RGB 着色。'),

        DeclareLaunchArgument(
            'gui_cfg', default_value='',
            description='rtabmap_viz GUI 配置文件路径（自定义 ini 文件，留空则根据 use_intensity_coloring 自动选择）。'),

        # ============================================================
        #  Livox LiDAR 参数
        # ============================================================
        DeclareLaunchArgument(
            'lidar_topic', default_value='/livox/lidar',
            description='Livox LiDAR PointCloud2 话题。'),

        DeclareLaunchArgument(
            'expected_update_rate', default_value='15.0',
            description='LiDAR预期帧率，必须高于实际帧率（10Hz LiDAR设15Hz）。过低会导致每帧被丢弃。'),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='点云降采样体素大小(m)。室内: 0.05-0.1, 室外: 0.2-0.5。'),

        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description='回环检测最小点云重叠率。'),

        # ============================================================
        #  装配模式（Livox 推荐开启）
        # ============================================================
        DeclareLaunchArgument(
            'assemble', default_value='true',
            description='启用点云装配模式。Livox非重复扫描建议开启，累积更多点云提升建图质量。'),

        DeclareLaunchArgument(
            'assembling_time', default_value='0.3',
            description='点云装配时间(s)。值越大点云越稠密，但建图速率降低。Livox推荐0.5-1.0。'),

        DeclareLaunchArgument(
            'depth_assembling_time', default_value='2.0',
            description='深度点云装配时间(s)。值越大累积的深度点越稠密。'),

        # ============================================================
        #  Deskewing 参数
        # ============================================================
        DeclareLaunchArgument(
            'deskewing', default_value='true',
            description='启用 LiDAR 运动畸变校正（需要 IMU 或外部里程计）。'),

        DeclareLaunchArgument(
            'deskewing_slerp', default_value='true',
            description='使用 slerp 插值进行 deskewing（更快但精度略低）。'),

        DeclareLaunchArgument(
            'fixed_frame_id', default_value='',
            description='Deskewing 固定参考系。留空则从 IMU 自动生成。'),

        DeclareLaunchArgument(
            'external_odom_frame_id', default_value='',
            description='外部里程计 TF frame（若提供则禁用 icp_odometry）。'),

        # ============================================================
        #  RealSense / 相机参数
        # ============================================================
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='使用 RealSense RGB-D 进行视觉回环检测和着色。'),

        DeclareLaunchArgument(
            'rgb_topic', default_value='/camera/color/image_raw',
            description='RGB 图像话题。'),

        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/aligned_depth_to_color/image_raw',
            description='深度图像话题（与RGB对齐）。'),

        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/color/camera_info',
            description='相机内参话题。'),

        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='是否启用 IMU（false 时禁用 deskewing 和 IMU 约束）。'),

        DeclareLaunchArgument(
            'raw_imu_topic', default_value='/livox/imu',
            description='原始 IMU 话题（Madgwick 滤波后输出到 imu_topic）。'),

        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data',
            description='IMU 话题（用于 deskewing 和里程计约束。默认使用 imu_filter_madgwick 滤波后的输出）。'),

        # ============================================================
        #  相机-LiDAR 外参（静态 TF 变换）
        # ============================================================
        DeclareLaunchArgument(
            'cam_to_lidar_x', default_value='0.0',
            description='相机到 LiDAR 的 X 偏移 (m)。'),
        DeclareLaunchArgument(
            'cam_to_lidar_y', default_value='0.0',
            description='相机到 LiDAR 的 Y 偏移 (m)。'),
        DeclareLaunchArgument(
            'cam_to_lidar_z', default_value='0.0',
            description='相机到 LiDAR 的 Z 偏移 (m)。'),
        DeclareLaunchArgument(
            'cam_to_lidar_roll', default_value='0.0',
            description='相机到 LiDAR 的 Roll 旋转 (rad)。'),
        DeclareLaunchArgument(
            'cam_to_lidar_pitch', default_value='0.0',
            description='相机到 LiDAR 的 Pitch 旋转 (rad)。'),
        DeclareLaunchArgument(
            'cam_to_lidar_yaw', default_value='0.0',
            description='相机到 LiDAR 的 Yaw 旋转 (rad)。'),

        # ============================================================
        #  点云着色导出（RGB-D → LiDAR 点云）
        # ============================================================
        DeclareLaunchArgument(
            'export_cloud', default_value='false',
            description='着色导出模式：映射结束后单独运行，用 RGB-D 给数据库中的全局 LiDAR 点云着色并导出 PLY。'),

        DeclareLaunchArgument(
            'export_dir', default_value='',
            description='着色导出输出目录（默认与数据库同目录）。'),

        DeclareLaunchArgument(
            'export_mesh', default_value='false',
            description='着色导出时同时生成带纹理的 mesh（OBJ + 纹理图）。'),

        DeclareLaunchArgument(
            'export_texture_size', default_value='8192',
            description='纹理分辨率：1024/2048/4096/8192/16384。'),

        DeclareLaunchArgument(
            'export_opt', default_value='2',
            description='导出时位姿优化方式：0=重新全局优化, 2=使用数据库中已有的优化位姿（更快）。'),

        OpaqueFunction(function=launch_setup),
    ])
