import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_topology import NavigationConfig
from navigation_topology_node import NavigationTopologyNode, navigation_markers, process_grid


class NavigationRosTests(unittest.TestCase):
    def grid(self):
        message = OccupancyGrid(header=Header(frame_id='map'))
        message.info.width = message.info.height = 50
        message.info.resolution = .1
        message.info.origin.orientation.w = 1.
        values = np.full((50, 50), 100, dtype=np.int8)
        values[3:47, 3:47] = 0
        message.data = values.ravel().tolist()
        return message

    def test_ros_grid_and_markers_keep_geometry_and_clear_deleted_nodes(self):
        message = self.grid()
        graph, labels = process_grid(message, NavigationConfig())
        markers = navigation_markers(graph, message.header).markers
        self.assertEqual(markers[0].action, Marker.DELETEALL)
        self.assertTrue(any(m.ns == 'navigation/nodes' for m in markers))
        self.assertTrue(all(m.header.frame_id == 'map' for m in markers))
        empty = navigation_markers(dict(nodes=[], edges=[]), message.header).markers
        self.assertEqual(len(empty), 1)
        self.assertEqual(empty[0].action, Marker.DELETEALL)

    def test_invalid_quaternion_tilt_and_payload_are_rejected(self):
        for failure in ('zero', 'tilt', 'size'):
            message = self.grid()
            if failure == 'zero':
                message.info.origin.orientation.w = 0.
            elif failure == 'tilt':
                message.info.origin.orientation.x = .1
            else:
                message.data.pop()
            with self.assertRaises(ValueError):
                process_grid(message, NavigationConfig())

    def test_invalid_grid_clears_retained_routes_and_the_export(self):
        with tempfile.TemporaryDirectory() as directory:
            node = NavigationTopologyNode.__new__(NavigationTopologyNode)
            node.config = NavigationConfig()
            node.epoch, node.frame, node.topic = 1, 'map', '/miti/grid_map'
            node.markers, node.graph_pub, node.status_pub = Mock(), Mock(), Mock()
            node.export = Path(directory)/'navigation.json'
            node.export.write_text('{"nodes":[{"id":"old"}]}')
            node.pose = None
            bad = self.grid()
            bad.data.pop()
            node.on_grid(bad)
            self.assertIsNone(node.graph)
            self.assertEqual(node.epoch, 2)
            self.assertFalse(json.loads(node.export.read_text())['nodes'])
            self.assertFalse(json.loads(node.graph_pub.publish.call_args.args[0].data)['edges'])


if __name__ == '__main__':
    unittest.main()
