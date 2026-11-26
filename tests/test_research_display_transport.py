import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import hotspot_sensor_transport as transport
from research_display_transport import DISPLAY_STREAMS, DisplayTransport


class DisplayTransportTest(unittest.TestCase):
    def node(self):
        node = DisplayTransport.__new__(DisplayTransport)
        node.snapshot_cache = {}
        node.dynamic_transforms = {}
        node.last_queued = {}
        return node

    def test_late_static_tf_reader_receives_all_retained_camera_messages(self):
        context = Context()
        rclpy.init(context=context, domain_id=76)
        publisher_node = Node('static_camera_history_publisher', context=context)
        reader_node = Node('static_camera_history_reader', context=context)
        executor = SingleThreadedExecutor(context=context)
        try:
            publisher = publisher_node.create_publisher(TFMessage, '/tf_static',
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            children = ['camera_depth_frame', 'camera_depth_optical_frame',
                        'camera_color_optical_frame']
            for child in children:
                publisher.publish(TFMessage(transforms=[TransformStamped(child_frame_id=child)]))
            node = self.node()
            node.args = SimpleNamespace(mode='send')
            received = set()
            reader_node.create_subscription(TFMessage, '/tf_static',
                lambda message: received.update(t.child_frame_id for t in message.transforms),
                node.stream_qos('/tf_static', TFMessage))
            deadline = time.monotonic() + 4
            while received != set(children) and time.monotonic() < deadline:
                rclpy.spin_once(reader_node, executor=executor, timeout_sec=.05)
            self.assertEqual(received, set(children))
        finally:
            executor.shutdown()
            reader_node.destroy_node()
            publisher_node.destroy_node()
            rclpy.shutdown(context=context)

    def test_separate_tf_publishers_are_aggregated_with_original_stamps(self):
        node = self.node()
        one = TransformStamped(child_frame_id='icp_odom')
        one.header.stamp.sec = 100
        two = TransformStamped(child_frame_id='livox_frame')
        two.header.stamp.sec = 90
        with patch.object(transport.Transport, 'queue') as queued:
            node.queue(0, TFMessage(transforms=[one]))
            node.last_queued.clear()
            node.queue(0, TFMessage(transforms=[two]))
        message = queued.call_args.args[1]
        self.assertEqual(message.transforms, [one, two])

    def test_reconnect_replays_unchanged_latched_map(self):
        node = self.node()
        cloud = PointCloud2(width=1)
        cloud.header.stamp.sec = 123
        with patch.object(transport.Transport, 'queue') as queued:
            node.queue(8, cloud)
            queued.reset_mock()
            node.replay_cached()
        self.assertEqual(queued.call_args.args, (8, cloud))
        self.assertEqual(node.snapshot_cache[8].header.stamp.sec, 123)

    def test_camera_rate_limit_does_not_delay_tf(self):
        node = self.node()
        with patch.object(transport.Transport, 'queue') as queued, patch('research_display_transport.time.monotonic', side_effect=[1., 1.1, 1.1]):
            node.queue(2, object())
            node.queue(2, object())
            node.queue(0, TFMessage())
        self.assertEqual([call.args[0] for call in queued.call_args_list], [2, 0])

    def test_display_cloud_protocol_preserves_geometry_and_header(self):
        cloud = PointCloud2(height=1, width=2, point_step=12, row_step=24,
            fields=[PointField(name='x', offset=0, datatype=7, count=1),
                    PointField(name='y', offset=4, datatype=7, count=1),
                    PointField(name='z', offset=8, datatype=7, count=1)], data=bytes(range(24)))
        cloud.header.frame_id = 'map'
        cloud.header.stamp.sec = 123
        with patch.object(transport, 'STREAMS', DISPLAY_STREAMS):
            packet, _ = transport.encode(8, cloud)
            index, restored = transport.decode(packet[:transport.HEADER.size], packet[transport.HEADER.size:])
        self.assertEqual(index, 8)
        self.assertEqual(restored, cloud)


if __name__ == '__main__':
    unittest.main()
