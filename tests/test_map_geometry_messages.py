import sys
import unittest
from pathlib import Path

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_geometry_node import cloud_xyz, wall_markers


class MessageTests(unittest.TestCase):
    def test_organized_big_endian_cloud_with_padding(self):
        raw = np.zeros((2, 64), dtype=np.uint8)
        view = np.ndarray((2, 2, 3), dtype='>f8', buffer=raw, strides=(64, 24, 8))
        view[:] = np.arange(12).reshape(2, 2, 3)
        message = PointCloud2(height=2, width=2, point_step=24, row_step=64, is_bigendian=True,
                             data=raw.tobytes(), fields=[PointField(name=key, offset=i*8,
                             datatype=PointField.FLOAT64, count=1) for i, key in enumerate(('x', 'y', 'z'))])
        np.testing.assert_array_equal(cloud_xyz(message), np.arange(12).reshape(4, 3))
        message.row_step = 4
        with self.assertRaises(ValueError):
            cloud_xyz(message)

    def test_empty_snapshot_removes_old_surfaces_and_labels_without_global_clear(self):
        result = wall_markers([], Header(frame_id='map'), 24)
        self.assertEqual(len(result.markers), 26)
        self.assertTrue(all(marker.action == Marker.DELETE for marker in result.markers))
        self.assertTrue(all(marker.ns.startswith('rtabmap/walls/') for marker in result.markers))


if __name__ == '__main__':
    unittest.main()
