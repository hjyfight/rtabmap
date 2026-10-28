# 默认使用 Livox 雷达及内置 IMU：use_camera=false，use_imu=true。
# 去畸变、IMU 旋转预测与后端重力约束默认开启；纯雷达不进行回环检测。
# 默认逐帧建图；确认运动跟踪稳定后可传 assemble:=true 累积点云。
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
#   - Livox IMU → Madgwick 方向估计、LiDAR去畸变、ICP旋转预测与重力约束
#
# Requirements:
#   - Livox ROS2 driver:   livox_ros_driver2
#   - RealSense ROS2 driver: ros-$ROS_DISTRO-realsense2-camera
#   - rtabmap_ros
#
# Usage:
#   【方式一：标准模式（每帧处理）】
#   1. 启动 Livox
#     $ ros2 launch livox_ros_driver2 rviz_MID360_launch.py
#     必须使用 xfer_format=0 的 PointCloud2，不能使用 CustomMsg。
#   2. 启动 RealSense
#     $ ros2 launch realsense2_camera rs_launch.py \
#         align_depth.enable:=true enable_gyro:=true enable_accel:=true \
#         unite_imu_method:=2 rgb_camera.profile:=640x360x30
#   3. 启动融合 3D SLAM
#     $ ros2 launch /home/jyfight/Project/rtabmap_slam/launch/livox_realsense_3dslam.launch.py use_camera:=true
#
#   【方式二：装配模式（运动跟踪稳定后使用）】
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
    use_camera = use_camera.lower() in ('true', '1', 'yes', 'on')

    # 视觉同步、深度筛选和短期记忆可独立调整，不依赖外参或装配开关。
    rgbd_sync_max_interval = float(LaunchConfiguration('rgbd_sync_max_interval').perform(context))
    visual_max_depth = float(LaunchConfiguration('visual_max_depth').perform(context))
    fusion_stm_size = int(LaunchConfiguration('fusion_stm_size').perform(context))
    proximity_by_space = use_camera and (
        LaunchConfiguration('proximity_by_space').perform(context).lower() in ('true', '1', 'yes', 'on'))
    loop_overlap = float(LaunchConfiguration('min_loop_closure_overlap').perform(context))
    for name, value in (('rgbd_sync_max_interval', rgbd_sync_max_interval),
                        ('visual_max_depth', visual_max_depth)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} 必须是大于零的有限数。')
    if fusion_stm_size < 1:
        raise ValueError('fusion_stm_size 必须是正整数。')
    if not math.isfinite(loop_overlap) or not 0 <= loop_overlap <= 1:
        raise ValueError('min_loop_closure_overlap 必须是 0 到 1 之间的有限数。')

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
    assemble = assemble.lower() in ('true', '1', 'yes', 'on')

    # ---- Deskewing ----
    deskewing = LaunchConfiguration('deskewing').perform(context)
    deskewing = deskewing.lower() in ('true', '1', 'yes', 'on')

    # ---- 是否启用 IMU ----
    use_imu = LaunchConfiguration('use_imu').perform(context)
    use_imu = use_imu.lower() in ('true', '1', 'yes', 'on')
    # Livox 内置 IMU 与相机开关无关。外部里程计也可独立提供去畸变参考系。
    imu_used = use_imu and bool(imu_topic_value)
    filter_imu = LaunchConfiguration('filter_imu').perform(context).lower() in ('true', '1', 'yes', 'on')
    normalize_livox_timestamps = LaunchConfiguration('normalize_livox_timestamps').perform(context).lower() in ('true', '1', 'yes', 'on')
    gravity_sigma = float(LaunchConfiguration('gravity_sigma').perform(context))
    if not math.isfinite(gravity_sigma) or gravity_sigma < 0:
        raise ValueError('gravity_sigma 必须是非负有限数。')
    if use_imu and not imu_topic_value:
        raise ValueError('use_imu=true 时必须指定 imu_topic。')
    if imu_used and filter_imu and (not raw_imu_topic_value or raw_imu_topic_value == imu_topic_value):
        raise ValueError('滤波器的 raw_imu_topic 必须非空且不同于 imu_topic；已有方向四元数时传 filter_imu:=false。')

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
    if use_camera and cam_extrinsics_all_zero:
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
    external_odom_topic = LaunchConfiguration('external_odom_topic').perform(context)

    # ---- 固定参考系（用于 deskewing）----
    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id:
        if external_odom_frame_id:
            fixed_frame_id = external_odom_frame_id
        elif imu_used and deskewing:
            fixed_frame_from_imu = True
            fixed_frame_id = frame_id.perform(context) + '_stabilized'

    if deskewing and not fixed_frame_id:
        deskewing = False
        log_warn.append(LogInfo(msg='[SLAM] 没有 IMU 或外部固定参考系，当前配置关闭去畸变。'))
    if deskewing and fixed_frame_from_imu:
        log_warn.append(LogInfo(msg=(
            '[SLAM] 当前 IMU 参考系可补偿旋转；扫描内平移补偿需要提供高频外部里程计 TF。')))
    if deskewing and (fixed_frame_id == frame_id.perform(context) or
                      (not external_odom_frame_id and fixed_frame_id == 'icp_odom')):
        raise ValueError('去畸变参考系必须独立于雷达本身及待计算的 icp_odom，避免 TF 循环依赖。')
    lidar_topic_normalized = lidar_topic_value + '/time_normalized'
    lidar_deskewing_input = (lidar_topic_normalized
                            if deskewing and normalize_livox_timestamps else lidar_topic_value)
    if not deskewing:
        lidar_topic_deskewed = lidar_topic_value

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
            ' '.join(part.perform(context) if hasattr(part, 'perform') else str(part) for part in export_cmd),
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
        'always_check_imu_tf': False,  # 静态外参只在初始化时查询，仍要求有效 TF
        'wait_imu_to_init': imu_used,
        'qos_imu': LaunchConfiguration('qos_imu'),
        'deskewing': False,  # 统一在前级去畸变，ICP/装配/后端使用同一份补偿点云
        'odom_frame_id': 'icp_odom',
        'guess_frame_id': fixed_frame_id if deskewing else '',
        'deskewing_slerp': deskewing_slerp,
        # ---- Livox 里程计参数 ----
        'Odom/ScanKeyFrameThr': '0.4',           # 匹配率降至 40% 时更新局部几何关键帧
        'OdomF2M/ScanSubtractRadius': str(voxel_size_value),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': '0',          # 整数枚举；纯 ICP 不使用视觉 BA
        'Icp/CorrespondenceRatio': '0.01',
        'Icp/PointToPlane': 'true',               # 面特征约束提升角精度
    }
    # IMU 只提供方向及旋转初值；此配置不是紧耦合 IMU 预积分里程计。

    # RTAB-Map SLAM 参数
    rtabmap_parameters = {
        'database_path': database_path if database_path else '',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_rgbd': use_camera,
        'subscribe_odom_info': not external_odom_frame_id,
        'subscribe_scan_cloud': True,
        'odom_frame_id': (external_odom_frame_id if external_odom_frame_id else ''),
        'odom_sensor_sync': use_camera,  # 需要 rgbd_image 同步，相机链路有问题时阻塞 rtabmap 启动
        # ---- 回环检测频率（装配模式下降低但仍启用）----
        'Rtabmap/DetectionRate': ('1' if assemble else '1'),
        # ---- 回环检测：仅融合模式使用视觉候选和 ICP 验证 ----
        'Rtabmap/PublishStats': 'true',
        'Rtabmap/LoopThr': '0.11' if use_camera else '1.0',
        'RGBD/AggressiveLoopThr': '0.05' if use_camera else '1.0',
        'RGBD/ProximityBySpace': str(proximity_by_space).lower(),
        'RGBD/ProximityByTime': 'false',
        'RGBD/ProximityMaxGraphDepth': '0',     # 0=无限搜索，不限制图深度
        'RGBD/ProximityPathMaxNeighbors': '10', # 空间 ICP 分支合并邻近扫描，不依赖视觉视场
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'false',
        'Grid/3D': 'true',
        'Grid/Sensor': '0',  # 从LiDAR点云建几何，RGB图像仅着色
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.3',
        'Grid/RayTracing': 'false',
        # ---- 记忆体 ----
        'Mem/LaserScanVoxelSize': str(voxel_size_value),  # 每份入库扫描降采样，非全局去重
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': str(fusion_stm_size) if use_camera else '30',  # 融合历史节点更早进入可检索工作记忆
        # ---- 配准策略 ----
        'Reg/Strategy': '2' if use_camera else '1',  # 全局视觉+ICP；空间邻近另有独立扫描 ICP 分支
        'Reg/Force3DoF': 'false',  # 与ICP里程计保持6DoF，保留高度、横滚与俯仰
        # ---- 重力约束（0=禁用；不能用压平三维姿态代替重力约束）----
        'Optimizer/Strategy': '1',  # 本机已安装 g2o，支持重力约束
        'Optimizer/GravitySigma': str(gravity_sigma) if imu_used else '0',
        'qos_imu': LaunchConfiguration('qos_imu'),
        # ---- 回环检测 ----
        'Icp/CorrespondenceRatio': str(loop_overlap),
        # ---- 视觉回环 ----
        'Kp/MaxFeatures': '400' if use_camera else '-1',
        'Vis/MaxFeatures': '600' if use_camera else '0',
        'Kp/MaxDepth': str(visual_max_depth),
        'Vis/MaxDepth': str(visual_max_depth),
        'Vis/InlierDistance': '0.1',
        'Vis/MinInliers': '10',
        'Mem/RehearsalSimilarity': '0.6' if use_camera else '0.0',  # 相似节点合并阈值；全局回环阈值由 Rtabmap/LoopThr 控制
    }

    # ============================================================
    #  Remappings 和 Arguments
    # ============================================================
    # 共享 remappings（rtabmap/icp_odometry/rtabmap_viz 共用）
    # 每个节点只保留一条 IMU remap，启用时让 ICP 与后端收到同一份方向数据。
    odom_topic = external_odom_topic if external_odom_frame_id else 'icp_odom'
    remappings = [('odom', odom_topic)]
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
                    'approx_sync_max_interval': rgbd_sync_max_interval,  # 限制 RGB/Depth/CameraInfo 时间差
                    'use_sim_time': use_sim_time,
                    'qos': LaunchConfiguration('qos'),
                    'qos_camera_info': LaunchConfiguration('qos'),
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
    if imu_used and filter_imu:
        nodes.append(
            Node(
                package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'use_mag': False,
                    'world_frame': 'enu',
                    'publish_tf': False,
                }],
                remappings=[('imu/data_raw', raw_imu_topic_value),
                            ('imu/data', imu_topic_value)],
            )
        )

    # ----- IMU 重力方向稳定参考系（用于 deskewing）-----
    # ICP 发布 icp_odom -> stabilized，IMU 发布 stabilized -> base，避免重复 TF 父节点。
    if fixed_frame_from_imu and deskewing:
        nodes.append(
            Node(
                package='rtabmap_util', executable='imu_to_tf', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'fixed_frame_id': fixed_frame_id,
                    'base_frame_id': frame_id,
                    'qos': LaunchConfiguration('qos_imu'),
                    'wait_for_transform_duration': 0.001,
                }],
                remappings=[('imu/data', imu_topic.perform(context))],
            )
        )

    # ----- Livox 逐点时间单位兼容 -----
    # 本机 RTAB-Map 按数值量级猜测 FLOAT64 时间单位，无法正确识别设备启动时钟的纳秒值。
    # 先依据 header 归一为秒；不改 header、坐标或真实采样时间。
    if deskewing and normalize_livox_timestamps:
        nodes.append(
            ExecuteProcess(
                cmd=[FindExecutable(name='python3'),
                     os.path.join(os.path.dirname(__file__), '..', 'scripts', 'livox_pointcloud_time.py'),
                     '--ros-args', '-p', ['use_sim_time:=', use_sim_time],
                     '-p', ['qos:=', LaunchConfiguration('qos')],
                     '-p', ['time_unit:=', LaunchConfiguration('livox_timestamp_unit')],
                     '-r', 'input_cloud:=' + lidar_topic_value,
                     '-r', 'output_cloud:=' + lidar_topic_normalized],
                output='screen',
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
                    'qos': LaunchConfiguration('qos'),
                    'queue_size': 5,
                }],
                remappings=[('input_cloud', lidar_deskewing_input),
                            (lidar_deskewing_input + '/deskewed', lidar_topic_deskewed)],
            )
        )

    # ----- Livox 点云装配（累积非重复扫描点云）-----
    if assemble:
        assembler_odom_topic = external_odom_topic if external_odom_frame_id else 'icp_odom'
        nodes.append(
            Node(
                package='rtabmap_util', executable='point_cloud_assembler', output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'assembling_time': LaunchConfiguration('assembling_time'),
                    'voxel_size': voxel_size_value,  # 累积后降采样，默认 5 cm
                    'qos': LaunchConfiguration('qos'),
                    'qos_odom': LaunchConfiguration('qos'),
                    'fixed_frame_id': (external_odom_frame_id if external_odom_frame_id else 'icp_odom'),
                    'range_min': 0.3,
                    'topic_queue_size': LaunchConfiguration('topic_queue_size'),
                    'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                    'wait_for_transform': 10.0,
                    'approx_sync': True,  # 近似同步容忍时间戳不完美对齐
                }],
                remappings=[
                    ('cloud', lidar_topic_deskewed),
                    ('odom', assembler_odom_topic),
                ],
            )
        )

    # ----- ICP Odometry（LiDAR里程计）-----
    if not external_odom_frame_id:
        nodes.append(
            Node(
                package='rtabmap_odom', executable='icp_odometry', output='screen',
                parameters=[shared_parameters, icp_odometry_parameters],
                remappings=remappings + [('scan_cloud', lidar_topic_deskewed)],
                additional_env={'OMP_NUM_THREADS': '1', 'OPENBLAS_NUM_THREADS': '1'},
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
                                      'assembled_cloud' if assemble else lidar_topic_deskewed)],
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
                                  'assembled_cloud' if assemble else lidar_topic_deskewed)],
        )
    )

    # ----- 可视化 ----
    # 直接订阅开关只影响实时 RGB-D；mapData 仍携带后端保存的图像/深度。
    # 是否绘制 RGB-D 地图云由 GUI 配置控制，必须用 -d 正确加载。
    # RGB-D 数据仍然由 rtabmap SLAM 节点订阅，用于视觉回环检测。
    # 默认 GUI 仅显示雷达地图云，显示降采样与节点筛选不修改后端数据。
    rtabmap_viz_args = []
    if gui_cfg:
        rtabmap_viz_args.extend(['-d', gui_cfg])
    nodes.append(
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'odometry_node_name': 'icp_odometry',
                         'subscribe_rgbd': False,
                         'subscribe_rgb': False,
                         'subscribe_depth': False}],
            remappings=remappings + [('scan_cloud', scan_viz_topic)],
            arguments=rtabmap_viz_args,
        )
    )

    # ----- 仅融合模式启动回环诊断；纯雷达仅建图 -----
    if use_camera:
        nodes.append(
            ExecuteProcess(
                cmd=[FindExecutable(name='python3'),
                     os.path.join(os.path.dirname(__file__), '..', 'scripts',
                                  'loop_closure_diagnostics.py'),
                     '--lidar-topic', lidar_topic_value,
                     '--rgb-topic', rgb_topic.perform(context),
                     '--depth-topic', depth_topic.perform(context),
                     '--camera-info-topic', camera_info_topic.perform(context),
                     '--assemble', str(assemble).lower()],
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

    diag_info = LogInfo(msg=(
        '\n=============================================\n'
        '[SLAM] 建图参数摘要\n'
        f'[SLAM] 相机: {use_camera}；装配: {assemble}；体素: {voxel_size_value} m\n'
        f'[SLAM] 原始点云: {lidar_topic_value}\n'
        f'[SLAM] 去畸变: {deskewing}；固定参考系: {fixed_frame_id or "无"}\n'
        f'[SLAM] 点时间归一化: {deskewing and normalize_livox_timestamps}\n'
        f'[SLAM] ICP 输入: {lidar_topic_deskewed}\n'
        f'[SLAM] 后端输入: {"assembled_cloud" if assemble else lidar_topic_deskewed}\n'
        f'[SLAM] 使用 IMU: {imu_used}；Madgwick: {imu_used and filter_imu}\n'
        f'[SLAM] 原始 IMU: {raw_imu_topic_value}；方向 IMU: {imu_topic_value if imu_used else "禁用"}\n'
        f'[SLAM] IMU→ICP/RTAB-Map: {"启用" if imu_used else "禁用"}\n'
        f'[SLAM] imu_to_tf: {fixed_frame_from_imu and deskewing}\n'
        f'[SLAM] GravitySigma: {rtabmap_parameters["Optimizer/GravitySigma"]}；优化器: g2o\n'
        f'[SLAM] 回环检测: {"视觉候选 + ICP 验证（已启用诊断日志）" if use_camera else "关闭（纯雷达建图）"}\n'
        f'[SLAM] 空间回环: {rtabmap_parameters["RGBD/ProximityBySpace"]}\n'
        f'[SLAM] 回环 ICP 最小对应率: {loop_overlap}；STM: {rtabmap_parameters["Mem/STMSize"]}\n'
        f'[SLAM] RGB-D 同步上限: {rgbd_sync_max_interval} s；视觉最大深度: {visual_max_depth} m\n'
        f'[SLAM] odom_sensor_sync: {rtabmap_parameters["odom_sensor_sync"]}\n'
        '=============================================\n'
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
            'qos', default_value='2',
            description='QoS: 0=系统默认, 1=Reliable, 2=Best Effort.'),

        DeclareLaunchArgument(
            'qos_imu', default_value='2',
            description='IMU QoS: 0=系统默认, 1=Reliable, 2=Best Effort.'),

        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='启动 RTAB-Map 可视化；无显示或测试时设 false。'),

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
            description='回环 ICP 最小对应率；不是体素相交比例。默认 0.2，按拒绝日志调节。'),

        DeclareLaunchArgument(
            'proximity_by_space', default_value='true',
            description='融合模式启用空间邻近视觉/扫描回环；纯雷达模式仍关闭回环。'),

        DeclareLaunchArgument(
            'fusion_stm_size', default_value='10',
            description='融合模式短期记忆节点数；小场景可实测 5，STM 中节点不参加历史回环。'),

        DeclareLaunchArgument(
            'visual_max_depth', default_value='4.0',
            description='视觉关键点最大深度(m)，同时用于词袋和视觉配准。'),

        DeclareLaunchArgument(
            'rgbd_sync_max_interval', default_value='0.05',
            description='RGB、对齐深度和内参近似同步允许的最大时间差(s)。'),

        # ============================================================
        #  装配模式（先验证逐帧运动跟踪，再按需开启）
        # ============================================================
        DeclareLaunchArgument(
            'assemble', default_value='false',
            description='按里程计装配已去畸变的点云；确认运动跟踪稳定后开启。'),

        DeclareLaunchArgument(
            'assembling_time', default_value='0.3',
            description='点云装配时间(s)，默认0.3；增大会累积更多里程计误差。'),

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
            'deskewing_slerp', default_value='false',
            description='true 只插值扫描两端姿态以提高速度；默认逐点查询高频 IMU TF。'),

        DeclareLaunchArgument(
            'normalize_livox_timestamps', default_value='true',
            description='去畸变前将 Livox FLOAT64 绝对点时间归一成秒；其他雷达可关闭。'),

        DeclareLaunchArgument(
            'livox_timestamp_unit', default_value='auto',
            description='点时间单位：auto/seconds/milliseconds/microseconds/nanoseconds；auto 根据 header 判断。'),

        DeclareLaunchArgument(
            'fixed_frame_id', default_value='',
            description='Deskewing 固定参考系。留空则从 IMU 自动生成。'),

        DeclareLaunchArgument(
            'external_odom_frame_id', default_value='',
            description='外部里程计 TF frame（若提供则禁用 icp_odometry）。'),

        DeclareLaunchArgument(
            'external_odom_topic', default_value='/odom',
            description='外部里程计 Odometry 话题；设置 external_odom_frame_id 后供 RTAB-Map 及可选点云装配器使用。'),

        # ============================================================
        #  RealSense / 相机参数
        # ============================================================
        DeclareLaunchArgument(
            'use_camera', default_value='false',
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
            description='使用 Livox IMU 方向辅助 ICP 和重力约束；独立于相机开关。'),

        DeclareLaunchArgument(
            'filter_imu', default_value='true',
            description='启动 Madgwick；imu_topic 已由外部滤波器提供有效四元数时设 false。'),

        DeclareLaunchArgument(
            'gravity_sigma', default_value='0.3',
            description='IMU 重力约束标准差(rad)，0 禁用；use_imu=false 时自动禁用。'),

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
