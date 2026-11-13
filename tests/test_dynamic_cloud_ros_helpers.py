from pathlib import Path
import sys
import unittest

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_cloud_node import ScanQueue, xyz_message
from map_geometry_node import cloud_xyz


class DynamicCloudRosHelpers(unittest.TestCase):
    def scan(self, stamp):
        return PointCloud2(header=Header(frame_id='lidar', stamp=Time(sec=stamp)))

    def test_latest_timestamp_with_a_matching_pose_wins_and_older_backlog_is_discarded(self):
        queue = ScanQueue()
        for stamp in (1, 3, 2):
            queue.add(self.scan(stamp), 10.)
        def pose(message):
            if message.header.stamp.sec == 3:
                raise ValueError('pose not available yet')
            return np.eye(4)
        chosen = queue.take(10.1, pose)
        self.assertEqual(chosen[0].header.stamp.sec, 2)
        self.assertEqual(len(queue.items), 1)
        self.assertEqual(queue.take(10.2, lambda _: np.eye(4))[0].header.stamp.sec, 3)
        self.assertIsNone(queue.take(10.3, pose))

    def test_missing_pose_expiry_and_duplicate_stamps_never_reprocess_the_same_scan(self):
        queue = ScanQueue()
        message = self.scan(4)
        queue.add(message, 10.)
        queue.add(message, 10.1)
        self.assertEqual(len(queue.items), 1)
        self.assertIsNone(queue.take(12., lambda _: np.eye(4)))
        for stamp in range(100):
            queue.add(self.scan(stamp), 12.)
        self.assertEqual(len(queue.items), 30)
        queue.take(12.1, lambda _: np.eye(4))
        queue.add(self.scan(99), 12.2)
        self.assertIsNone(queue.take(12.3, lambda _: np.eye(4)))

    def test_pointcloud_preserves_map_coordinates_and_empty_live_snapshot_replaces_old_data(self):
        xyz = np.array([[1, 2, 3], [4, 5, 6]], dtype=float)
        message = xyz_message(xyz, 'map', Time(sec=5))
        np.testing.assert_array_equal(cloud_xyz(message), xyz)
        self.assertEqual(message.header.frame_id, 'map')
        self.assertEqual(message.header.stamp.sec, 5)
        empty = xyz_message(np.empty((0, 3)), 'map', Time(sec=6))
        self.assertEqual(empty.width, 0)
        self.assertEqual(len(empty.data), 0)


if __name__ == '__main__':
    unittest.main()
