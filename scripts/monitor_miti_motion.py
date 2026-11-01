#!/usr/bin/env python3
"""Read-only movement monitoring. No robot commands or SLAM parameter changes."""
import argparse
from collections import Counter
import datetime
import json
import math
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rtabmap_msgs.msg import Info, MapGraph, OdomInfo
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from loop_closure_diagnostics import detection_snapshot, read_stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=180)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 3600:
        parser.error('--seconds must be 5–3600')
    rclpy.init()
    node = rclpy.create_node('miti_motion_monitor')
    started = time.monotonic()
    stamp = datetime.datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    folder = Path(__file__).resolve().parents[1] / 'logs/miti'
    folder.mkdir(parents=True, exist_ok=True)
    output = folder / f'motion_{stamp}.jsonl'
    counts, prior = Counter(), Counter()
    last_seen, max_gaps, sensor_stamps, backwards = {}, {}, {}, Counter()
    state = {'odom_lost': 0, 'loops': [], 'travel_m': 0.0, 'max_step_m': 0.0,
             'max_speed_m_s': 0.0, 'graph_nodes': None, 'semantic': {}, 'input': {}}
    first_pose = last_pose = None
    last_pose_stamp = None
    graph_poses = {}
    graph_shift = 0.0
    warnings = Counter()
    subscriptions = []
    loop_seen = set()
    odom_stats_topics = set()

    def callback(key, msg):
        nonlocal first_pose, last_pose, last_pose_stamp, graph_poses, graph_shift
        now = time.monotonic()
        counts[key] += 1
        if key in last_seen:
            max_gaps[key] = max(max_gaps.get(key, 0.0), now-last_seen[key])
        last_seen[key] = now
        header = getattr(msg, 'header', None)
        sensor_stamp = None
        if header is not None:
            sensor_stamp = header.stamp.sec + header.stamp.nanosec / 1e9
            if key in sensor_stamps and sensor_stamp < sensor_stamps[key]:
                backwards[key] += 1
            sensor_stamps[key] = sensor_stamp
        if key == 'odom':
            pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            if not all(math.isfinite(v) for v in pose):
                warnings['non-finite odometry'] += 1
                return
            if first_pose is None:
                first_pose = pose
            if last_pose is not None:
                step = math.dist(pose, last_pose)
                state['travel_m'] += step
                state['max_step_m'] = max(state['max_step_m'], step)
                dt = sensor_stamp-last_pose_stamp
                if dt > 0:
                    state['max_speed_m_s'] = max(state['max_speed_m_s'], step/dt)
            last_pose, last_pose_stamp = pose, sensor_stamp
            state['distance_from_start_m'] = math.dist(pose, first_pose)
        elif key == 'odom_stats':
            state['odom_lost'] += int(msg.lost)
            state['icp_ratio'] = float(msg.icp_inliers_ratio)
            state['icp_estimation_ms'] = 1000*float(msg.time_estimation)
            state['icp_interval_s'] = float(msg.interval)
        elif key == 'graph':
            state['graph_nodes'] = len(msg.poses_id)
            poses = {i: (p.position.x, p.position.y, p.position.z) for i, p in zip(msg.poses_id, msg.poses)}
            graph_shift = max([graph_shift] + [math.dist(p, graph_poses[i]) for i, p in poses.items() if i in graph_poses])
            graph_poses = poses
        elif key == 'slam':
            state['loop_state'] = detection_snapshot(msg, read_stats(msg))['state']
            state['ref_id'] = int(msg.ref_id)
            event = (int(msg.ref_id), int(msg.loop_closure_id), int(msg.proximity_detection_id))
            if (event[1] > 0 or event[2] > 0) and event not in loop_seen:
                loop_seen.add(event)
                state['loops'].append({'t': round(now-started, 2), 'node': event[0],
                                       'global_id': event[1], 'proximity_id': event[2]})
        elif key in ('semantic', 'input'):
            try:
                state[key] = json.loads(msg.data)
            except json.JSONDecodeError:
                warnings[f'invalid {key} JSON'] += 1
        elif key == 'cloud':
            state['map_points'] = msg.width*msg.height
        elif key == 'logs' and msg.level >= 30:
            if any(n in msg.name for n in ('rtabmap', 'icp_odometry', 'semantic', 'rgbd', 'deskew')):
                warnings[f'{msg.name}: {msg.msg[:350]}'] += 1

    for topic, kind, key in [('/livox/lidar', PointCloud2, 'lidar'), ('/livox/imu', Imu, 'imu'),
                             ('/camera/color/image_raw', Image, 'rgb'),
                             ('/camera/aligned_depth_to_color/image_raw', Image, 'depth'),
                             ('/icp_odom', Odometry, 'odom'), ('/mapGraph', MapGraph, 'graph'),
                             ('/info', Info, 'slam'), ('/cloud_map', PointCloud2, 'cloud'),
                             ('/semantic/status', String, 'semantic'),
                             ('/semantic/input_status', String, 'input'), ('/rosout', Log, 'logs')]:
        subscriptions.append(node.create_subscription(kind, topic, lambda m, k=key: callback(k, m), qos_profile_sensor_data))
    previous_report = started
    windows = []
    print(json.dumps({'event': 'monitor_started', 'seconds': args.seconds, 'report': str(output)}, ensure_ascii=False), flush=True)
    try:
        with output.open('w') as stream:
            while rclpy.ok() and time.monotonic()-started < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now-previous_report < 5:
                    continue
                if not odom_stats_topics:
                    choices = [topic for topic, kinds in node.get_topic_names_and_types()
                               if 'rtabmap_msgs/msg/OdomInfo' in kinds]
                    if choices:
                        # Full and lite topics describe the same update. Receive only one,
                        # preferring lite so monitoring does not pull the local scan map.
                        topic = sorted(choices, key=lambda name: (not name.endswith('_lite'), name))[0]
                        subscriptions.append(node.create_subscription(OdomInfo, topic, lambda m: callback('odom_stats', m), qos_profile_sensor_data))
                        odom_stats_topics.add(topic)
                window = {'t': round(now-started, 1),
                          'hz': {key: round((value-prior[key])/(now-previous_report), 2)
                                 for key, value in counts.items() if key != 'logs'},
                          'age_s': {key: round(now-value, 2) for key, value in last_seen.items() if key != 'logs'},
                          'state': json.loads(json.dumps(state)), 'max_graph_shift_m': round(graph_shift, 4),
                          'warnings': dict(warnings.most_common(5))}
                windows.append(window)
                line = json.dumps(window, ensure_ascii=False)
                stream.write(line+'\n'); stream.flush(); print(line, flush=True)
                prior, previous_report = counts.copy(), now
    except KeyboardInterrupt:
        pass
    finally:
        summary = {'elapsed_s': round(time.monotonic()-started, 2), 'counts': dict(counts),
                   'max_receive_gap_s': {k: round(v, 3) for k, v in max_gaps.items()},
                   'backwards_stamps': dict(backwards), 'state': state,
                   'max_graph_shift_m': graph_shift, 'warnings': dict(warnings),
                   'odom_stats_topics': sorted(odom_stats_topics), 'windows': len(windows),
                   'note': 'Distances are odometry estimates, not ground truth; wall-clock minus sensor stamps is not used as latency.'}
        summary_path = output.with_suffix('.summary.json')
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({'event': 'monitor_finished', 'summary': str(summary_path), **summary}, ensure_ascii=False), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
