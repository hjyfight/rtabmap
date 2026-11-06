"""Prevent a healthy SLAM input from masking a broken MITI display connection."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from check_miti_live import display_issues


class DisplayCheckTests(unittest.TestCase):
    def setUp(self):
        image = dict(width=424, height=240, frame='camera_color_optical_frame')
        self.topics = {
            '/camera/color/image_raw': deepcopy(image),
            '/miti/aligned_depth/image_raw': deepcopy(image),
            '/miti/aligned_depth/camera_info': deepcopy(image),
            '/cloud_map': dict(points=7000),
            '/miti/grid_map': dict(width=230, height=146, observed_cells=1400),
            '/tf': dict(frames=['map -> icp_odom', 'icp_odom -> livox_frame']),
            '/tf_static': dict(frames=['livox_frame -> camera_link',
                                      'camera_link -> camera_depth_frame',
                                      'camera_depth_frame -> camera_depth_optical_frame']),
        }

    def test_complete_display_with_split_dynamic_and_static_tf(self):
        self.assertEqual(display_issues(self.topics), [])

    def test_rgbd_and_maps_do_not_mask_missing_camera_mount(self):
        self.topics['/tf_static']['frames'].remove('livox_frame -> camera_link')
        self.assertIn('TF path', ' '.join(display_issues(self.topics)))

    def test_empty_pointcloud_and_unobserved_grid_are_failures(self):
        self.topics['/cloud_map']['points'] = 0
        self.topics['/miti/grid_map']['observed_cells'] = 0
        self.assertEqual(len(display_issues(self.topics)), 2)

    def test_depth_and_calibration_must_match_rgb(self):
        self.topics['/miti/aligned_depth/image_raw']['width'] = 640
        self.topics['/miti/aligned_depth/camera_info']['frame'] = 'camera_depth_optical_frame'
        self.assertEqual(len(display_issues(self.topics)), 2)

    def test_cyclic_tf_is_rejected_without_hanging(self):
        self.topics['/tf_static']['frames'].append('camera_depth_optical_frame -> camera_link')
        self.assertIn('TF path', ' '.join(display_issues(self.topics)))


if __name__ == '__main__':
    unittest.main()
