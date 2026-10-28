"""Regression checks for acquisition-time preservation and the launched data graph.

Run after sourcing ROS: python3 -m unittest discover -s tests -p 'test_livox_motion_config.py'
"""

from array import array
import importlib.util
import os
from pathlib import Path
import struct
import unittest
from unittest.mock import patch

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('ROS_LOG_DIR', '/tmp/livox_motion_test_logs')


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


time_adapter = load_module('livox_time', ROOT / 'scripts/livox_pointcloud_time.py')
slam_launch = load_module('livox_launch', ROOT / 'launch/livox_realsense_3dslam.launch.py')


def make_cloud(header=6932864.055079328, scale=1e9, bigendian=False, padded=False):
    cloud = PointCloud2()
    cloud.header.frame_id = 'livox_frame'
    cloud.header.stamp.sec = int(header)
    cloud.header.stamp.nanosec = round((header - int(header)) * 1e9)
    cloud.width, cloud.height = 3, 2 if padded else 1
    cloud.point_step = 26
    cloud.row_step = cloud.width * cloud.point_step + (7 if padded else 0)
    cloud.is_bigendian = bigendian
    cloud.fields = [
        PointField(name='x', offset=0, datatype=7, count=1),
        PointField(name='y', offset=4, datatype=7, count=1),
        PointField(name='z', offset=8, datatype=7, count=1),
        PointField(name='intensity', offset=12, datatype=7, count=1),
        PointField(name='tag', offset=16, datatype=2, count=1),
        PointField(name='line', offset=17, datatype=2, count=1),
        PointField(name='timestamp', offset=18, datatype=8, count=1),
    ]
    data = bytearray([0xAB] * (cloud.height * cloud.row_step))
    for row in range(cloud.height):
        for col in range(cloud.width):
            index = row * cloud.width + col
            struct.pack_into(('>' if bigendian else '<') + 'ffffBBd', data,
                             row * cloud.row_step + col * cloud.point_step,
                             5.0, float(index), -1.0, 30.0, 16, 2,
                             (header + index * 0.01) * scale)
    cloud.data = array('B', data)
    return cloud


class TimestampTests(unittest.TestCase):
    def test_uptime_and_epoch_all_units(self):
        for header in (0.0, 6932864.055079328, 1793012532.329):
            for unit, factor in time_adapter.TIME_SCALES.items():
                with self.subTest(header=header, unit=unit):
                    cloud = make_cloud(header=header, scale=1.0 / factor)
                    output, found_unit = time_adapter.normalize_timestamps(cloud)
                    self.assertEqual(found_unit, unit)
                    values = np.ndarray((1, 3), dtype='<f8', buffer=output.data,
                                        offset=18, strides=(cloud.row_step, cloud.point_step))
                    np.testing.assert_allclose(values[0], [header + i * 0.01 for i in range(3)],
                                               rtol=0, atol=1e-6)
                    self.assertEqual(output.header, cloud.header)

    def test_preserves_geometry_padding_and_input(self):
        for bigendian in (False, True):
            cloud = make_cloud(bigendian=bigendian, padded=True)
            original = bytes(cloud.data)
            output, _ = time_adapter.normalize_timestamps(cloud)
            self.assertEqual(bytes(cloud.data), original)
            self.assertEqual(output.fields, cloud.fields)
            self.assertEqual(output.point_step, cloud.point_step)
            for row in range(2):
                for col in range(3):
                    offset = row * cloud.row_step + col * cloud.point_step
                    self.assertEqual(bytes(output.data[offset:offset+18]), original[offset:offset+18])
                padding = row * cloud.row_step + 3 * cloud.point_step
                self.assertEqual(bytes(output.data[padding:padding+7]), original[padding:padding+7])
            values = np.ndarray((2, 3), dtype='>f8' if bigendian else '<f8', buffer=output.data,
                                offset=18, strides=(cloud.row_step, cloud.point_step))
            self.assertAlmostEqual(values[1, 2] - values[0, 0], 0.05, places=6)

    def test_already_seconds_not_scaled_twice(self):
        cloud = make_cloud(scale=1.0)
        output, unit = time_adapter.normalize_timestamps(cloud)
        self.assertIs(output, cloud)
        self.assertEqual(unit, 'seconds')

    def test_rejects_invalid_or_mismatched_times(self):
        cloud = make_cloud()
        cloud.header.stamp.sec += 100
        with self.assertRaises(ValueError):
            time_adapter.normalize_timestamps(cloud)
        cloud = make_cloud()
        cloud.data = cloud.data[:-1]
        with self.assertRaises(ValueError):
            time_adapter.normalize_timestamps(cloud)
        cloud = make_cloud()
        cloud.fields[-1].name = 'no_time'
        with self.assertRaises(ValueError):
            time_adapter.normalize_timestamps(cloud)
        cloud = make_cloud()
        struct.pack_into('<d', cloud.data, 18, float('nan'))
        with self.assertRaises(ValueError):
            time_adapter.normalize_timestamps(cloud)


def resolve_launch(**overrides):
    context = LaunchContext()
    context.launch_configurations.update({key: str(value) for key, value in overrides.items()})
    for action in slam_launch.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = {}
    for action in slam_launch.launch_setup(context):
        if not isinstance(action, Node) or (action.condition and not action.condition.evaluate(context)):
            continue
        parameters = {}
        for entry in evaluate_parameters(context, action._Node__parameters or []):
            parameters.update(entry)
        remaps = [(perform_substitutions(context, source), perform_substitutions(context, target))
                  for source, target in (action._Node__remappings or [])]
        nodes[str(action.node_executable)] = (parameters, remaps)
    return nodes


class LaunchGraphTests(unittest.TestCase):
    def test_visualizer_passes_config_with_required_d_flag(self):
        for overrides, expected_path in (
                ({}, str(ROOT / 'launch' / '..' / 'config' / 'livox_realsense_gui.ini')),
                ({'gui_cfg': '/tmp/custom lidar display.ini'}, '/tmp/custom lidar display.ini')):
            with self.subTest(overrides=overrides), patch.object(
                    slam_launch, 'Node', wraps=Node) as node_factory:
                # resolve_launch uses Node for isinstance; the factory is patched only in the launch module.
                resolve_launch(use_camera='true', **overrides)
                viz = [call.kwargs for call in node_factory.call_args_list
                       if call.kwargs.get('executable') == 'rtabmap_viz']
                self.assertEqual(len(viz), 1)
                self.assertEqual(viz[0]['arguments'], ['-d', expected_path])

    def test_only_fusion_starts_diagnostics_with_configured_topics(self):
        for use_camera in ('false', 'true'):
            with self.subTest(use_camera=use_camera), patch.object(
                    slam_launch, 'ExecuteProcess', wraps=slam_launch.ExecuteProcess) as process:
                resolve_launch(use_camera=use_camera, assemble='false',
                               lidar_topic='/robot/lidar', rgb_topic='/robot/rgb',
                               depth_topic='/robot/depth', camera_info_topic='/robot/camera_info')
                commands = [call.kwargs['cmd'] for call in process.call_args_list
                            if any(isinstance(part, str) and part.endswith('loop_closure_diagnostics.py')
                                   for part in call.kwargs['cmd'])]
                self.assertEqual(len(commands), 1 if use_camera == 'true' else 0)
                if commands:
                    command = commands[0]
                    for flag, value in (('--lidar-topic', '/robot/lidar'),
                                        ('--rgb-topic', '/robot/rgb'),
                                        ('--depth-topic', '/robot/depth'),
                                        ('--camera-info-topic', '/robot/camera_info'),
                                        ('--assemble', 'false')):
                        self.assertEqual(command[command.index(flag) + 1], value)

    def test_default_lidar_has_imu_deskew_gravity_without_loop(self):
        nodes = resolve_launch(rtabmap_viz='false')
        self.assertNotIn('point_cloud_assembler', nodes)
        self.assertNotIn('rgbd_sync', nodes)
        self.assertNotIn('rtabmap_viz', nodes)
        self.assertIn('imu_filter_madgwick_node', nodes)
        self.assertIn('imu_to_tf', nodes)
        self.assertIn('lidar_deskewing', nodes)
        odom, remaps = nodes['icp_odometry']
        self.assertTrue(odom['wait_imu_to_init'])
        self.assertEqual(odom['guess_frame_id'], 'livox_frame_stabilized')
        self.assertFalse(odom['deskewing'])  # no double deskew
        self.assertEqual([target for source, target in remaps if source == 'imu'], ['/imu/data'])
        self.assertEqual(dict(remaps)['scan_cloud'], '/livox/lidar/deskewed')
        slam, remaps = nodes['rtabmap']
        self.assertEqual(slam['Optimizer/GravitySigma'], '0.3')
        self.assertEqual(slam['RGBD/ProximityBySpace'], 'false')
        self.assertEqual(slam['RGBD/ProximityByTime'], 'false')
        self.assertEqual(slam['Rtabmap/LoopThr'], '1.0')
        self.assertEqual(slam['RGBD/AggressiveLoopThr'], '1.0')
        self.assertEqual(slam['Mem/STMSize'], '30')
        self.assertEqual(slam['Mem/RehearsalSimilarity'], '0.0')
        self.assertEqual(dict(remaps)['imu'], '/imu/data')
        self.assertEqual(dict(remaps)['scan_cloud'], '/livox/lidar/deskewed')
        self.assertEqual(slam['Reg/Force3DoF'], 'false')

    def test_assembler_only_merges_deskewed_clouds(self):
        nodes = resolve_launch(assemble='true')
        self.assertEqual(dict(nodes['point_cloud_assembler'][1])['cloud'], '/livox/lidar/deskewed')
        self.assertEqual(dict(nodes['point_cloud_assembler'][1])['odom'], 'icp_odom')
        self.assertEqual(dict(nodes['icp_odometry'][1])['scan_cloud'], '/livox/lidar/deskewed')
        self.assertEqual(dict(nodes['rtabmap'][1])['scan_cloud'], 'assembled_cloud')

    def test_fusion_adds_scan_proximity_without_replacing_visual_pipeline(self):
        nodes = resolve_launch(use_camera='true')
        slam = nodes['rtabmap'][0]
        self.assertEqual(slam['RGBD/ProximityBySpace'], 'true')
        self.assertEqual(slam['RGBD/ProximityPathMaxNeighbors'], '10')
        self.assertEqual(slam['Reg/Strategy'], '2')
        self.assertEqual(slam['Mem/RehearsalSimilarity'], '0.6')
        self.assertEqual(slam['Kp/MaxDepth'], '4.0')
        self.assertEqual(slam['Vis/MaxDepth'], '4.0')
        self.assertNotIn('Kp/WordsUseDepthCorrection', slam)
        self.assertEqual(nodes['rgbd_sync'][0]['approx_sync_max_interval'], 0.05)
        disabled = resolve_launch(use_camera='true', proximity_by_space='false')
        self.assertEqual(disabled['rtabmap'][0]['RGBD/ProximityBySpace'], 'false')

    def test_fusion_tuning_keeps_odometry_on_single_scans(self):
        nodes = resolve_launch(use_camera='true', assemble='true', voxel_size='0.1',
                               min_loop_closure_overlap='0.08', fusion_stm_size='5',
                               visual_max_depth='3.0', rgbd_sync_max_interval='0.02')
        slam, remaps = nodes['rtabmap']
        self.assertEqual(slam['Mem/STMSize'], '5')
        self.assertEqual(slam['Icp/CorrespondenceRatio'], '0.08')
        self.assertEqual(slam['Mem/LaserScanVoxelSize'], '0.1')
        self.assertEqual(slam['Kp/MaxDepth'], '3.0')
        self.assertEqual(slam['Vis/MaxDepth'], '3.0')
        self.assertEqual(nodes['rgbd_sync'][0]['approx_sync_max_interval'], 0.02)
        self.assertEqual(nodes['point_cloud_assembler'][0]['voxel_size'], 0.1)
        self.assertEqual(dict(remaps)['scan_cloud'], 'assembled_cloud')
        self.assertEqual(dict(nodes['icp_odometry'][1])['scan_cloud'], '/livox/lidar/deskewed')
        self.assertEqual(nodes['icp_odometry'][0]['Icp/CorrespondenceRatio'], '0.01')
        unassembled = resolve_launch(use_camera='true', assemble='false', voxel_size='0.1')
        self.assertEqual(unassembled['rtabmap'][0]['Mem/LaserScanVoxelSize'], '0.1')

    def test_rejects_invalid_fusion_tuning(self):
        for overrides in ({'rgbd_sync_max_interval': '0'}, {'rgbd_sync_max_interval': 'nan'},
                          {'visual_max_depth': '-1'}, {'visual_max_depth': 'inf'},
                          {'fusion_stm_size': '0'}, {'fusion_stm_size': '1.5'},
                          {'min_loop_closure_overlap': '-0.1'},
                          {'min_loop_closure_overlap': '1.1'},
                          {'min_loop_closure_overlap': 'nan'}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                resolve_launch(**overrides)

    def test_custom_filtered_imu_and_sim_clock(self):
        nodes = resolve_launch(imu_topic='/robot/imu', filter_imu='false', use_sim_time='true')
        self.assertNotIn('imu_filter_madgwick_node', nodes)
        for name in ('imu_to_tf', 'icp_odometry', 'rtabmap', 'lidar_deskewing'):
            self.assertTrue(nodes[name][0]['use_sim_time'])
        self.assertEqual(dict(nodes['icp_odometry'][1])['imu'], '/robot/imu')
        self.assertEqual(dict(nodes['rtabmap'][1])['imu'], '/robot/imu')

    def test_no_imu_disables_gravity_and_unavailable_deskew(self):
        nodes = resolve_launch(use_imu='false')
        self.assertNotIn('imu_filter_madgwick_node', nodes)
        self.assertNotIn('imu_to_tf', nodes)
        self.assertNotIn('lidar_deskewing', nodes)
        self.assertFalse(nodes['icp_odometry'][0]['wait_imu_to_init'])
        self.assertEqual(nodes['rtabmap'][0]['Optimizer/GravitySigma'], '0')
        self.assertEqual(dict(nodes['icp_odometry'][1])['scan_cloud'], '/livox/lidar')

    def test_external_odom_can_deskew_without_imu(self):
        nodes = resolve_launch(use_imu='false', external_odom_frame_id='robot_odom')
        self.assertNotIn('icp_odometry', nodes)
        self.assertIn('lidar_deskewing', nodes)
        self.assertEqual(nodes['lidar_deskewing'][0]['fixed_frame_id'], 'robot_odom')
        self.assertEqual(nodes['rtabmap'][0]['odom_frame_id'], 'robot_odom')
        self.assertEqual(dict(nodes['rtabmap'][1])['odom'], '/odom')

        assembled = resolve_launch(
            use_imu='false', external_odom_frame_id='robot_odom',
            external_odom_topic='/robot/odom', assemble='true')
        self.assertNotIn('icp_odometry', assembled)
        self.assertEqual(dict(assembled['point_cloud_assembler'][1])['odom'], '/robot/odom')
        self.assertEqual(dict(assembled['rtabmap'][1])['odom'], '/robot/odom')

    def test_no_deskew_keeps_imu_without_duplicate_base_tf(self):
        nodes = resolve_launch(deskewing='false')
        self.assertNotIn('imu_to_tf', nodes)
        self.assertNotIn('lidar_deskewing', nodes)
        self.assertTrue(nodes['icp_odometry'][0]['wait_imu_to_init'])
        self.assertEqual(nodes['icp_odometry'][0]['guess_frame_id'], '')

    def test_camera_and_localization_keep_imu(self):
        nodes = resolve_launch(use_camera='true', localization='true')
        self.assertIn('rgbd_sync', nodes)
        self.assertTrue(nodes['rtabmap'][0]['subscribe_rgbd'])
        self.assertEqual(nodes['rtabmap'][0]['Rtabmap/LoopThr'], '0.11')
        self.assertEqual(nodes['rtabmap'][0]['RGBD/AggressiveLoopThr'], '0.05')
        self.assertEqual(nodes['rtabmap'][0]['Rtabmap/PublishStats'], 'true')
        self.assertEqual(nodes['rtabmap'][0]['Mem/STMSize'], '10')
        self.assertEqual(nodes['rtabmap'][0]['Mem/RehearsalSimilarity'], '0.6')
        self.assertEqual(nodes['rtabmap'][0]['Mem/IncrementalMemory'], 'False')
        self.assertEqual(dict(nodes['rtabmap'][1])['imu'], '/imu/data')

    def test_rejects_circular_deskew_and_filter_topics(self):
        for overrides in ({'fixed_frame_id': 'icp_odom'}, {'fixed_frame_id': 'livox_frame'},
                          {'raw_imu_topic': '/imu/data'}, {'gravity_sigma': '-1'},
                          {'cam_to_lidar_yaw': 'nan'}, {'cam_to_lidar_x': 'inf'}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                resolve_launch(**overrides)


if __name__ == '__main__':
    unittest.main()
