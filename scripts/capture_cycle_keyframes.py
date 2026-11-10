#!/usr/bin/env python3
"""Capture real RGB-D map updates for isolated timing replay. Read-only ROS."""
import argparse
from copy import deepcopy
import datetime
import json
from pathlib import Path
import time


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from rtabmap_msgs.msg import MapData, Node
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames', type=int, default=12)
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not 1 <= args.frames <= 1000 or not 1 <= args.seconds <= 3600:
        p.error('invalid frame count or timeout')
    if args.output.exists():
        p.error('output already exists')
    rclpy.init()
    probe = rclpy.create_node('capture_cycle_keyframes')
    frames, poses = {}, {}
    snapshot = MapData()
    started = time.monotonic()
    def receive(msg):
        snapshot.header = msg.header
        poses.update(zip(msg.graph.poses_id, msg.graph.poses))
        for n in msg.nodes:
            if n.id <= 0 or n.id in frames or len(frames) >= args.frames:
                continue
            if not (len(n.data.left.data) or len(n.data.left_compressed)) or not (
                    len(n.data.right.data) or len(n.data.right_compressed)):
                continue
            copy = Node(id=n.id, stamp=n.stamp, map_id=n.map_id)
            for field in ('header','left','right','left_compressed','right_compressed',
                          'left_camera_info','right_camera_info','local_transform'):
                setattr(copy.data, field, deepcopy(getattr(n.data, field)))
            frames[n.id] = copy
            print(f'已采集 {len(frames)}/{args.frames} 个实际关键帧', flush=True)
    subscription = probe.create_subscription(MapData, '/mapData', receive, qos_profile_sensor_data)
    try:
        while rclpy.ok() and time.monotonic()-started < args.seconds and (
                len(frames) < args.frames or any(i not in poses for i in frames)):
            rclpy.spin_once(probe, timeout_sec=.1)
        if len(frames) < args.frames or any(i not in poses for i in frames):
            raise RuntimeError('Insufficient new RGB-D keyframes/poses; no complete capture saved')
        snapshot.nodes = list(frames.values())
        snapshot.graph.header = snapshot.header
        snapshot.graph.poses_id = list(frames)
        snapshot.graph.poses = [poses[i] for i in frames]
        args.output.mkdir(parents=True)
        (args.output/'map_data.cdr').write_bytes(serialize_message(snapshot))
        manifest = dict(mode='live_keyframe_capture', source_topic='/mapData',
                        captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                        elapsed_s=time.monotonic()-started, frame_count=len(frames),
                        nodes=[dict(node_id=n.id, node_stamp=n.stamp, session_id=n.map_id) for n in snapshot.nodes],
                        note='Original RGB-D payloads and optimized poses; LiDAR payload omitted for semantic replay.')
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        print(args.output, flush=True)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
