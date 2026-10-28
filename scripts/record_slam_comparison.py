#!/usr/bin/env python3
"""Save real map/graph data and loop evidence from a single comparison run."""
import argparse
import json
import os
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info, MapGraph
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def stat_value(stats, key):
    """RTAB-Map stats use Group/Name/Unit, including an empty trailing unit."""
    return next((float(v) for k, v in stats.items()
                 if k == key or k.startswith(key + '/')), None)


class Recorder(Node):
    def __init__(self, output):
        super().__init__('slam_comparison_recorder')
        self.output = output
        self.graph = None
        self.cloud = None
        self.seen = set()
        # Best effort / volatile subscriptions also match reliable / latched publishers.
        self.create_subscription(MapGraph, '/mapGraph', self.on_graph, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cloud_map', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Info, '/info', self.on_info, qos_profile_sensor_data)
        self.create_timer(5.0, self.save)
        self.get_logger().info('Recording /mapGraph, /cloud_map and /info to ' + str(output))

    def on_graph(self, msg):
        self.graph = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'frame_id': msg.header.frame_id,
            'poses': {str(i): [p.position.x, p.position.y, p.position.z]
                      for i, p in zip(msg.poses_id, msg.poses)},
            'links': [{'from': link.from_id, 'to': link.to_id, 'type': link.type}
                      for link in msg.links],
        }

    def on_cloud(self, msg):
        self.cloud = msg  # Keep latest only; convert at the periodic save, not each frame.

    def on_info(self, msg):
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        for kind, target in [('global', msg.loop_closure_id),
                             ('proximity', msg.proximity_detection_id)]:
            key = (int(msg.ref_id), int(target), kind)
            if target <= 0 or key in self.seen:
                continue
            self.seen.add(key)
            event = dict(ref_id=key[0], target_id=key[1], type=kind,
                         stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                         optimization_error=stat_value(stats, 'Loop/Optimization_error'),
                         optimization_iterations=stat_value(stats, 'Loop/Optimization_iterations'),
                         visual_inliers=stat_value(stats, 'Loop/Visual_inliers'), stats=stats)
            with (self.output / 'events.jsonl').open('a') as stream:
                stream.write(json.dumps(event) + '\n')
            self.get_logger().info(f'Accepted {kind} loop: {key[0]} <-> {key[1]}')

    def save(self):
        if self.graph is not None:
            tmp = self.output / 'graph.tmp'
            tmp.write_text(json.dumps(self.graph))
            os.replace(tmp, self.output / 'graph.json')
        if self.cloud is not None:
            xyz = np.array(list(point_cloud2.read_points(
                self.cloud, field_names=('x', 'y', 'z'), skip_nans=True)),
                dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
            xyz = np.column_stack([xyz['x'], xyz['y'], xyz['z']])
            tmp = self.output / 'cloud.tmp.npz'
            np.savez_compressed(tmp, xyz=xyz)
            os.replace(tmp, self.output / 'cloud.npz')
            self.cloud = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args, ros_args = parser.parse_known_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=ros_args)
    node = Recorder(args.output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
