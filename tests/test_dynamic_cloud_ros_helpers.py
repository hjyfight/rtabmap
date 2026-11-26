from pathlib import Path
from collections import deque
from types import MethodType, SimpleNamespace
import sys
import time
import unittest
from unittest.mock import MagicMock

import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_cloud_node import DynamicCloudNode, ScanQueue, xyz_message
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

    def display_node(self, map_received=True):
        points = np.array([[1., 2., 3.], [4., 5., 6.]], dtype=np.float32)
        filter = MagicMock()
        filter.points = points
        filter.set_map.side_effect = lambda value: setattr(filter, 'points', value)
        filter.filtered_map.side_effect = lambda: filter.points
        filter.scan.return_value = dict(accepted=True)
        node = SimpleNamespace(filter=filter, session=('map.db', 'old_semantic_epoch'),
            map_received=map_received, map_dirty=False, await_map=False,
            queue=ScanQueue(), outputs=deque(maxlen=30), status={},
            frame='map', scan_topic='/livox/lidar/deskewed', confirmations=3,
            max_range=12., last_live=-float('inf'), live_present=False,
            live_timeout=1.5, last_publish=-float('inf'), map_rate=2.,
            last_input_stamp=5., live_pub=MagicMock(), map_pub=MagicMock(), status_pub=MagicMock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=5))),
            reload_map_snapshot=MagicMock(), transform=lambda _: np.eye(4))
        node.clear_live = MethodType(DynamicCloudNode.clear_live, node)
        return node

    def test_semantic_epoch_change_preserves_the_same_geometric_map_and_resumes_live_scans(self):
        node = self.display_node()
        original = node.filter.points.copy()
        DynamicCloudNode.on_session(node, SimpleNamespace(map_key='map.db', map_uuid='new_semantic_epoch'))
        np.testing.assert_array_equal(node.filter.points, original)
        self.assertTrue(node.map_received)
        node.filter.reset.assert_called_once()
        node.filter.set_map.assert_not_called()
        message = xyz_message(np.array([[1., 0., 0.]]), 'lidar', Time(sec=5))
        node.queue.add(message, time.monotonic())
        DynamicCloudNode.update(node)
        np.testing.assert_array_equal(cloud_xyz(node.live_pub.publish.call_args.args[0]), [[1., 0., 0.]])
        np.testing.assert_array_equal(cloud_xyz(node.map_pub.publish.call_args.args[0]), original)
        self.assertEqual(node.status['state'], 'ready')

    def test_different_map_discards_geometry_and_requests_its_retained_snapshot(self):
        node = self.display_node()
        DynamicCloudNode.on_session(node, SimpleNamespace(map_key='different.db', map_uuid='new_semantic_epoch'))
        self.assertFalse(node.map_received)
        self.assertEqual(len(node.filter.points), 0)
        node.reload_map_snapshot.assert_called_once()
        self.assertEqual(node.live_pub.publish.call_args.args[0].width, 0)

    def test_valid_timestamped_live_scan_is_displayed_while_full_map_is_unavailable(self):
        node = self.display_node(map_received=False)
        matrix = np.eye(4)
        matrix[:3, 3] = [3., 4., 5.]
        node.transform = lambda _: matrix
        node.queue.add(xyz_message(np.array([[1., 0., 0.]]), 'lidar', Time(sec=5)), time.monotonic())
        DynamicCloudNode.update(node)
        np.testing.assert_array_equal(cloud_xyz(node.live_pub.publish.call_args.args[0]), [[4., 4., 5.]])
        self.assertEqual(len(node.queue.items), 0)
        node.filter.scan.assert_not_called()
        self.assertEqual(node.status['state'], 'waiting')

    def test_missing_timestamped_pose_does_not_display_an_untransformed_scan(self):
        node = self.display_node(map_received=False)
        def missing_pose(_):
            raise ValueError('timestamped pose not available')
        node.transform = missing_pose
        node.queue.add(xyz_message(np.array([[1., 0., 0.]]), 'lidar', Time(sec=5)), time.monotonic())
        DynamicCloudNode.update(node)
        node.live_pub.publish.assert_not_called()
        node.filter.scan.assert_not_called()


if __name__ == '__main__':
    unittest.main()
