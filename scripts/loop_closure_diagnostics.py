#!/usr/bin/env python3
"""监控融合建图链路，逐帧记录回环检测状态、候选结果和成功事件。

随 livox_realsense_3dslam.launch.py 的 use_camera=true 分支自动启动。
独立运行：source /home/jyfight/Project/rtabmap_slam/setup.bash 后执行本脚本。
日志默认保存到项目 logs/loop_closure_events.jsonl。
"""

import argparse
from collections import deque
import datetime
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from rtabmap_msgs.msg import Info

REPORT_PERIOD = 5.0

TOPICS = [
    ('/livox/lidar', PointCloud2, 'lidar'),
    ('/assembled_cloud', PointCloud2, 'assembled_cloud'),
    ('/camera/aligned_depth_to_color/image_raw', Image, 'depth'),
    ('/camera/color/image_raw', Image, 'rgb'),
    ('/camera/color/camera_info', CameraInfo, 'camera_info'),
]


def read_stats(msg):
    """统计键为分类/名称/单位，兼容 /words、/ms 和空单位。"""
    return {'/'.join(key.split('/')[:2]): float(value) if math.isfinite(value) else None
            for key, value in zip(msg.stats_keys, msg.stats_values)}


def detection_snapshot(msg, stats):
    """/info 是本帧处理结果；候选和拒绝标志不能当作已成功的闭环。"""
    candidate_id = int(stats.get('Loop/Highest_hypothesis_id') or 0)
    global_id = int(msg.loop_closure_id)
    proximity_id = int(msg.proximity_detection_id)
    rejected = bool(stats.get('Loop/RejectedHypothesis'))
    small_movement = bool(stats.get('Memory/Small_movement'))
    fast_movement = bool(stats.get('Memory/Fast_movement'))
    space_paths = int(stats.get('Proximity/Space_paths') or 0)
    scan_paths = int(stats.get('Proximity/Space_scan_paths_checked') or 0)
    visual_paths = int(stats.get('Proximity/Space_visual_paths_checked') or 0)
    suppressed_id = int(stats.get('Loop/Suppressed_hypothesis_id') or 0)
    timings = [stats.get('Timing/' + name) for name in
               ('Likelihood_computation', 'Posterior_computation', 'Hypotheses_creation')]
    search_performed = (True if any(value is not None and value > 0 for value in timings)
                        else False if all(value is not None for value in timings) else None)
    search_time_ms = sum(timings) if all(value is not None for value in timings) else None

    if global_id > 0 or proximity_id > 0:
        state, label = 'CONFIRMED', '已接受回环约束'
    elif stats.get('Memory/Working_memory_size') == 0 and not (space_paths or scan_paths or visual_paths):
        state, label = 'COLLECTING_HISTORY', '积累历史关键帧，工作记忆尚无可比较节点'
    elif stats.get('Keypoint/Current_frame') == 0 and not (space_paths or scan_paths or visual_paths):
        state, label = 'NO_VISUAL_FEATURES', '当前帧没有可用视觉词'
    elif suppressed_id > 0:
        state, label = 'CANDIDATE_SUPPRESSED', '全局候选已由空间回环抑制'
    elif rejected:
        state, label = 'CANDIDATE_REJECTED', '候选被后端拒绝'
    elif fast_movement and search_performed is not True:
        state, label = 'FAST_MOVEMENT', '运动过快，本帧跳过回环搜索'
    elif small_movement and search_performed is not True:
        # 小位移并非总会跳过搜索，优先使用后端实际搜索计时判断。
        state, label = 'SMALL_MOVEMENT', '运动较小，候选可能沿用上一帧'
    elif candidate_id > 0:
        state, label = 'CANDIDATE_UNCONFIRMED', '发现候选，本帧未形成有效回环'
    elif space_paths > 0 or scan_paths > 0 or visual_paths > 0:
        state, label = 'PROXIMITY_UNCONFIRMED', '已搜索空间路径，本帧未形成有效回环'
    elif search_performed is False:
        state, label = 'SEARCH_SKIPPED', '本帧未执行全局候选搜索'
    elif stats.get('Loop/Highest_hypothesis_id') is None:
        state, label = 'STATS_UNAVAILABLE', 'SLAM 数据已到达，候选统计不可用'
    else:
        state, label = 'SEARCHING', ('已执行全局搜索，本帧无候选' if search_performed
                                     else '收到处理结果，本帧无候选（搜索计时不可用）')

    # 空间回环成功时，全局视觉候选仍可能被拒绝；分别记录两者的结果。
    if candidate_id > 0 and candidate_id == global_id:
        candidate_state, candidate_label = 'CONFIRMED', '已接受全局回环约束'
    elif candidate_id > 0 and candidate_id == suppressed_id:
        candidate_state, candidate_label = 'CANDIDATE_SUPPRESSED', '候选已由空间回环抑制'
    elif rejected:
        candidate_state, candidate_label = 'CANDIDATE_REJECTED', '候选被后端拒绝'
    elif fast_movement and search_performed is not True:
        candidate_state, candidate_label = 'FAST_MOVEMENT', '运动过快，本帧跳过回环搜索'
    elif small_movement and search_performed is not True:
        candidate_state, candidate_label = 'SMALL_MOVEMENT', '运动较小，候选可能沿用上一帧'
    elif candidate_id > 0:
        candidate_state, candidate_label = 'CANDIDATE_UNCONFIRMED', '本帧未接受该候选'
    else:
        candidate_state, candidate_label = 'NO_CANDIDATE', '本帧无全局候选'

    return {
        'state': state,
        'state_label': label,
        'candidate_id': candidate_id,
        'candidate_score': stats.get('Loop/Highest_hypothesis_value'),
        'candidate_state': candidate_state,
        'candidate_label': candidate_label,
        'rejected': rejected,
        'suppressed_candidate_id': suppressed_id,
        'small_movement': small_movement,
        'fast_movement': fast_movement,
        'search_performed': search_performed,
        'search_time_ms': search_time_ms,
        'global_loop_id': global_id,
        'proximity_detection_id': proximity_id,
        'space_paths': space_paths,
        'scan_paths_checked': scan_paths,
        'visual_paths_checked': visual_paths,
        'working_memory_nodes': stats.get('Memory/Working_memory_size'),
        'short_time_memory_nodes': stats.get('Memory/Short_time_memory_size'),
        'visual_words': stats.get('Keypoint/Current_frame'),
        'rehearsal_similarity': stats.get('Memory/Rehearsal_sim'),
        'rehearsal_merged_id': stats.get('Memory/Rehearsal_merged'),
        'visual_matches': stats.get('Loop/Visual_matches'),
        'visual_inliers': stats.get('Loop/Visual_inliers'),
        'visual_inliers_ratio': stats.get('Loop/Visual_inliers_ratio'),
        # 本机 RTAB-Map 不一定发布回环 ICP 内点，不能用里程计内点代替。
        'icp_inliers': stats.get('Loop/ICP_inliers'),
        'icp_inliers_ratio': stats.get('Loop/ICP_inliers_ratio'),
        'optimization_error': stats.get('Loop/Optimization_error'),
        'optimization_iterations': stats.get('Loop/Optimization_iterations'),
    }


def metric(value, format_spec='.0f'):
    return format(value, format_spec) if value is not None else 'N/A'


def remember(key, recent):
    if key in recent:
        return False
    recent.append(key)
    return True


class TopicMonitor:
    """BestEffort 可接收两种可靠性发布端；按时间戳避免重复计数。"""

    def __init__(self, node, topic, msg_type, count_fn):
        self.count_fn = count_fn
        self._last_stamp = None
        self.subscription = node.create_subscription(
            msg_type, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        header = getattr(msg, 'header', None)
        if header is not None:
            key = (header.stamp.sec, header.stamp.nanosec)
            if key == self._last_stamp:
                return
            self._last_stamp = key
        self.count_fn()


class LoopClosureDiagnostics(Node):
    def __init__(self, log_file=None, topics=None, assemble=False):
        super().__init__('loop_closure_diagnostics')
        default_dir = os.path.normpath(
            os.path.join(os.path.dirname(__file__), '..', 'logs'))
        self.log_file = os.path.abspath(os.path.expanduser(
            log_file or os.path.join(default_dir, 'loop_closure_events.jsonl')))
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)

        self.topics = TOPICS if topics is None else topics
        self.assemble = assemble
        self.counts = {name: 0 for _, _, name in self.topics}
        self.info_count = 0
        self.last_info_time = None
        self.last_report_time = time.monotonic()
        self.loop_count = 0
        self.detection_count = 0
        self.current_nodes = 0
        self.last_detection = None
        self.recent_info = deque(maxlen=128)
        self.recent_loops = deque(maxlen=128)

        self.monitors = [TopicMonitor(self, topic, msg_type,
                                     lambda n=name: self._count(n))
                         for topic, msg_type, name in self.topics]
        # 同时兼容有、无 rtabmap 命名空间；重复转发的同一帧只计一次。
        self.info_subscriptions = [
            self.create_subscription(Info, topic, self._info_cb, qos_profile_sensor_data)
            for topic in ('/info', '/rtabmap/info')]
        self.create_timer(REPORT_PERIOD, self._report)
        self.get_logger().info(
            '[回环检测状态] WAITING_FOR_SLAM | 诊断已启动，等待 /info 或 /rtabmap/info\n'
            f'  日志: {self.log_file}\n'
            f'  监控话题: {", ".join(topic for topic, _, _ in self.topics)}')

    def _count(self, name):
        self.counts[name] += 1

    def _write_event(self, event):
        try:
            with open(self.log_file, 'a', encoding='utf-8') as stream:
                stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        except (OSError, ValueError) as error:
            self.get_logger().error(f'写入回环日志失败: {error}')

    def _info_cb(self, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec,
               int(msg.ref_id), int(msg.loop_closure_id), int(msg.proximity_detection_id))
        if not remember(key, self.recent_info):
            return

        self.last_info_time = time.monotonic()
        stats = read_stats(msg)
        self.info_count += 1
        self.detection_count += 1
        # 局部图节点数并非数据库中所有历史节点的总数。
        self.current_nodes = int(stats.get('Memory/Local_graph_size') or len(msg.wm_state))
        snapshot = detection_snapshot(msg, stats)
        event = {
            'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds'),
            'time': time.time(),
            'type': 'DETECTION_STATUS',
            'ref_id': int(msg.ref_id),
            'nodes': self.current_nodes,
            'detection_count': self.detection_count,
            **snapshot,
        }
        self.last_detection = event
        self._write_event(event)
        self.get_logger().info(
            f'[回环检测 #{self.detection_count}] 关键帧 #{msg.ref_id} | '
            f'{snapshot["state"]}: {snapshot["state_label"]} | '
            f'工作记忆:{metric(snapshot["working_memory_nodes"])} / '
            f'短期记忆:{metric(snapshot["short_time_memory_nodes"])} | '
            f'视觉词:{metric(snapshot["visual_words"])} | '
            f'全局搜索耗时:{metric(snapshot["search_time_ms"], ".3f")}ms | '
            f'图节点:{self.current_nodes} | 累计成功:{self.loop_count}')

        if snapshot['candidate_id'] > 0:
            self._write_event({**event, 'type': 'LOOP_CANDIDATE',
                               'state': snapshot['candidate_state'],
                               'state_label': snapshot['candidate_label']})
            self.get_logger().info(
                f'[回环候选] #{msg.ref_id} -> #{snapshot["candidate_id"]} | '
                f'分数:{metric(snapshot["candidate_score"], ".4f")} | '
                f'结果:{snapshot["candidate_label"]} | '
                f'视觉匹配:{metric(snapshot["visual_matches"])} / '
                f'内点:{metric(snapshot["visual_inliers"])}')
        if snapshot['space_paths'] or snapshot['scan_paths_checked'] or snapshot['visual_paths_checked']:
            self.get_logger().info(
                f'[空间回环搜索] 候选路径:{snapshot["space_paths"]} | '
                f'ICP检查:{snapshot["scan_paths_checked"]} | '
                f'视觉检查:{snapshot["visual_paths_checked"]} | '
                f'结果:{snapshot["state_label"]}')

        for event_type, target_id in (('GLOBAL_LOOP', int(msg.loop_closure_id)),
                                      ('PROXIMITY_ICP', int(msg.proximity_detection_id))):
            loop_key = (event_type, int(msg.ref_id), target_id)
            if target_id > 0 and remember(loop_key, self.recent_loops):
                self._record_loop(event, event_type, target_id)

    def _record_loop(self, detection, event_type, target_id):
        self.loop_count += 1
        event = {**detection, 'type': event_type, 'target_id': target_id,
                 'loop_count': self.loop_count}
        self._write_event(event)
        label = '全局回环' if event_type == 'GLOBAL_LOOP' else '空间几何 ICP 回环'
        ratio = event['icp_inliers_ratio']
        ratio_text = f'{ratio * 100:.1f}%' if ratio is not None else 'N/A'
        self.get_logger().info(
            f'[回环成功 #{self.loop_count}] {label} ({event_type}) | '
            f'#{event["ref_id"]} <-> #{target_id}\n'
            f'  视觉匹配:{metric(event["visual_matches"])} / '
            f'内点:{metric(event["visual_inliers"])} | '
            f'ICP内点:{metric(event["icp_inliers"])} / 重合率:{ratio_text}\n'
            f'  位姿图优化误差:{metric(event["optimization_error"], ".4f")} | '
            f'迭代次数:{metric(event["optimization_iterations"])} | '
            f'当前图节点:{self.current_nodes}')

    def _report(self):
        now = time.monotonic()
        elapsed = max(now - self.last_report_time, 1e-6)
        self.last_report_time = now
        hz = {name: count / elapsed for name, count in self.counts.items()}
        self.counts = dict.fromkeys(self.counts, 0)
        hz['slam_info'] = self.info_count / elapsed
        self.info_count = 0
        info_ok = self.last_info_time is not None and now - self.last_info_time < REPORT_PERIOD * 1.5
        if self.last_info_time is None:
            state, label = 'WAITING_FOR_SLAM', '等待 SLAM 数据，尚未收到检测结果'
        elif not info_ok:
            state, label = 'SLAM_TIMEOUT', 'SLAM 数据已中断，检查传感器与同步链路'
        else:
            state = self.last_detection['state']
            label = self.last_detection['state_label']
        assembled = f'{hz["assembled_cloud"]:.1f}Hz' if self.assemble else '未启用'
        self.get_logger().info(
            f'[建图状态] 雷达:{hz["lidar"]:.1f}Hz | RGB:{hz["rgb"]:.1f}Hz | '
            f'深度:{hz["depth"]:.1f}Hz | 相机内参:{hz["camera_info"]:.1f}Hz | '
            f'装配点云:{assembled} | SLAM:{hz["slam_info"]:.1f}Hz\n'
            f'[回环检测状态] {state}: {label} | '
            f'累计处理:{self.detection_count} | 累计成功:{self.loop_count}')
        self._write_event({
            'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds'),
            'time': time.time(), 'type': 'DETECTION_HEARTBEAT',
            'state': state, 'state_label': label, 'slam_ok': info_ok,
            'rates_hz': hz, 'nodes': self.current_nodes,
            'detection_count': self.detection_count, 'loop_count': self.loop_count,
        })


def main():
    parser = argparse.ArgumentParser(description='RTAB-Map Loop Closure Diagnostics')
    parser.add_argument('--log-file', default='', help='JSON Lines 日志输出路径')
    parser.add_argument('--lidar-topic', default=TOPICS[0][0])
    parser.add_argument('--assembled-topic', default=TOPICS[1][0])
    parser.add_argument('--depth-topic', default=TOPICS[2][0])
    parser.add_argument('--rgb-topic', default=TOPICS[3][0])
    parser.add_argument('--camera-info-topic', default=TOPICS[4][0])
    parser.add_argument('--assemble', choices=('true', 'false'), default='false')
    args, ros_args = parser.parse_known_args()
    topics = list(zip((args.lidar_topic, args.assembled_topic, args.depth_topic,
                       args.rgb_topic, args.camera_info_topic),
                      (entry[1] for entry in TOPICS), (entry[2] for entry in TOPICS)))
    rclpy.init(args=ros_args)
    node = LoopClosureDiagnostics(log_file=args.log_file, topics=topics,
                                  assemble=args.assemble == 'true')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
