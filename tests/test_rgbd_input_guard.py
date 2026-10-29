"""Regression coverage for the observed live depth corruption and orphan IMU nodes."""
import copy
import importlib.util
from pathlib import Path
import unittest

from rtabmap_msgs.msg import RGBDImage


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = load_script('guard_rgbd_input')
preflight = load_script('verify_fusion_input')


def valid_pair():
    pair = RGBDImage()
    for image, encoding, size in ((pair.rgb, 'rgb8', 3), (pair.depth, '16UC1', 2)):
        image.width, image.height = 424, 240
        image.encoding, image.step = encoding, 424 * size
        image.data = bytes(image.step * image.height)
        image.header.frame_id = 'camera_color_optical_frame'
        image.header.stamp.sec = 7185689
    pair.header = copy.deepcopy(pair.rgb.header)
    pair.rgb_camera_info.width, pair.rgb_camera_info.height = 424, 240
    pair.rgb_camera_info.header = copy.deepcopy(pair.rgb.header)
    pair.rgb_camera_info.k = [300., 0., 212., 0., 300., 120., 0., 0., 1.]
    return pair


class RGBDGuardTests(unittest.TestCase):
    def test_valid_pair_is_preserved(self):
        pair = valid_pair()
        original = copy.deepcopy(pair)
        self.assertIsNone(guard.validate_rgbd(pair))
        self.assertEqual(pair, original)

    def test_intermittent_640x480_depth_is_rejected(self):
        good = valid_pair()
        bad = copy.deepcopy(good)
        bad.depth.width, bad.depth.height = 640, 480
        bad.depth.step = 1280
        bad.depth.data = bytes(1280 * 480)
        self.assertIsNone(guard.validate_rgbd(good))
        self.assertIn('aligned depth=640x480', guard.validate_rgbd(bad))
        self.assertIsNone(guard.validate_rgbd(good))

    def test_bad_calibration_and_frame_are_rejected(self):
        for field in ('size', 'intrinsics', 'frame'):
            with self.subTest(field=field):
                pair = valid_pair()
                if field == 'size':
                    pair.rgb_camera_info.width = 640
                elif field == 'intrinsics':
                    pair.rgb_camera_info.k[0] = float('nan')
                else:
                    pair.depth.header.frame_id = 'camera_depth_optical_frame'
                self.assertIsNotNone(guard.validate_rgbd(pair))

    def test_invalid_storage_and_timing_are_rejected(self):
        for field in ('buffer', 'encoding', 'stamp'):
            with self.subTest(field=field):
                pair = valid_pair()
                if field == 'buffer':
                    pair.depth.data = pair.depth.data[:-2]
                elif field == 'encoding':
                    pair.depth.encoding = 'rgb8'
                else:
                    pair.depth.header.stamp.nanosec = 50_000_001
                self.assertIsNotNone(guard.validate_rgbd(pair))


class PreflightTests(unittest.TestCase):
    def sample(self):
        metadata = dict(width=424, height=240, frame_id='camera_color_optical_frame', stamp=7185689.)
        return {'nodes': ['/camera/camera', '/livox_lidar_publisher'], 'topics': {
            name: dict(count=4, last_samples=[metadata], sizes={'424x240': 4},
                       publishers=[{'node': '/camera/camera'}])
            for name in ('rgb', 'aligned_depth', 'color_info')}}

    def test_drivers_alone_can_start(self):
        self.assertEqual(preflight.verify(self.sample()), (424, 240))

    def test_guarded_launch_requires_three_valid_pairs_amid_bad_depth(self):
        sample = self.sample()
        for name, item in sample['topics'].items():
            item['last_samples'] = [dict(item['last_samples'][0], stamp=7185689. + i,
                                        encoding='16UC1' if name == 'aligned_depth' else 'rgb8')
                                    for i in range(4)]
        depth = sample['topics']['aligned_depth']
        depth['sizes'] = {'424x240': 3, '640x480': 1}
        depth['last_samples'][-1].update(width=640, height=480)
        with self.assertRaises(ValueError):
            preflight.verify(sample)
        self.assertEqual(preflight.verify(sample, guarded=True), (424, 240))
        depth['last_samples'][-2].update(width=640, height=480)
        with self.assertRaisesRegex(ValueError, '至少 3'):
            preflight.verify(sample, guarded=True)

    def test_orphan_preprocessors_block_start_without_backend(self):
        for name in ('/imu_filter_madgwick', '/livox_pointcloud_time', '/imu_to_tf',
                     '/lidar_deskewing', '/loop_closure_diagnostics', '/rgbd_input_guard'):
            with self.subTest(node=name):
                sample = self.sample()
                sample['nodes'].append(name)
                with self.assertRaisesRegex(ValueError, name):
                    preflight.verify(sample)


if __name__ == '__main__':
    unittest.main()
