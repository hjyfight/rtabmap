#!/usr/bin/env python3
"""Log depth/ICP loop closures and pose-graph optimization statistics."""

import argparse
import datetime
import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import Image, PointCloud2


class DepthLoopDiagnostics(Node):
    def __init__(self, log_file):
        super().__init__('depth_loop_diagnostics')
        self.log_file = log_file
        self.counts = {'depth': 0, 'lidar': 0, 'depth_cloud': 0, 'combined_cloud': 0}
        self.last_info = None
        self.loops = 0
        self.last_loop = (0, 0)
        self.started = time.monotonic()
        topics = [
            ('/camera/aligned_depth_to_color/image_raw', Image, 'depth'),
            ('/livox/lidar', PointCloud2, 'lidar'),
            ('/depth_cloud', PointCloud2, 'depth_cloud'),
            ('/combined_cloud', PointCloud2, 'combined_cloud'),
        ]
        for topic, msg_type, name in topics:
            self.create_subscription(
                msg_type, topic, lambda _msg, n=name: self._count(n),
                qos_profile_sensor_data)
        self.create_subscription(Info, '/info', self._info_cb, 10)
        self.create_subscription(Info, '/rtabmap/info', self._info_cb, 10)
        self.create_timer(5.0, self._report)
        self.get_logger().info('Depth-only ICP loop diagnostics started.')

    def _count(self, name):
        self.counts[name] += 1

    def _stats(self, msg):
        return dict(zip(msg.stats_keys, msg.stats_values))

    def _write(self, event):
        if not self.log_file:
            return
        with open(self.log_file, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(event, ensure_ascii=False) + '\n')

    def _info_cb(self, msg):
        stats = self._stats(msg)
        self.last_info = time.monotonic()
        pair = (int(msg.ref_id), int(msg.loop_closure_id))
        proximity = int(msg.proximity_detection_id)
        is_new_loop = msg.loop_closure_id > 0 and pair != self.last_loop
        is_new_proximity = proximity > 0 and proximity != self.last_loop[0]

        if is_new_loop or is_new_proximity:
            self.loops += 1
            self.last_loop = pair if is_new_loop else (proximity, 0)
            event_type = 'ICP_GLOBAL_LOOP' if is_new_loop else 'ICP_PROXIMITY_CLOSURE'
            target_id = int(msg.loop_closure_id) if is_new_loop else proximity
            nodes_count = int(stats.get('Map/Nodes', len(msg.wm_state)))
            inliers = float(stats.get('Loop/ICP_inliers', 0.0))
            inliers_ratio = float(stats.get('Loop/ICP_inliers_ratio', 0.0))
            opt_error = float(stats.get('Loop/Optimization_error', 0.0))
            opt_iters = int(float(stats.get('Loop/Optimization_iterations', 0.0)))
            now_str = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]

            event = {
                'timestamp': now_str,
                'time': time.time(),
                'event': event_type,
                'ref_id': int(msg.ref_id),
                'loop_target_id': target_id,
                'nodes': nodes_count,
                'icp_inliers': inliers,
                'icp_inliers_ratio': inliers_ratio,
                'optimization_error': opt_error,
                'optimization_iterations': opt_iters,
                'proximity_icp_multi': float(stats.get('Proximity/Space_detections_added_icp_multi', 0.0)),
                'proximity_icp_global': float(stats.get('Proximity/Space_detections_added_icp_global', 0.0)),
            }
            self._write(event)

            type_name = "全局ICP回环" if is_new_loop else "空间邻近ICP闭环"
            log_msg = (
                f"\n==================== [回环检测与建图优化触发] ====================\n"
                f"  时间: {now_str}\n"
                f"  类型: {type_name} ({event_type})\n"
                f"  回环匹配: 当前关键帧 #{msg.ref_id}  <--->  历史回环帧 #{target_id}\n"
                f"  ICP内点匹配数: {inliers:.0f} (重合匹配率: {inliers_ratio * 100:.1f}%)\n"
                f"  位姿图优化误差(Optimization Error): {opt_error:.4f}\n"
                f"  位姿图迭代次数: {opt_iters}\n"
                f"  当前地图节点总数: {nodes_count} | 累计回环次数: {self.loops}\n"
                f"=================================================================="
            )
            self.get_logger().warn(log_msg)

    def _report(self):
        now = time.monotonic()
        hz = {name: count / 5.0 for name, count in self.counts.items()}
        for name in self.counts:
            self.counts[name] = 0
        info_ok = self.last_info is not None and now - self.last_info < 5.0
        report = {
            'time': time.time(), 'event': 'STATUS', 'info': info_ok,
            'loops': self.loops, 'hz': hz,
        }
        self._write(report)
        status_str = "正常运行" if info_ok else "未接收到SLAM数据(检查数据链)"
        self.get_logger().info(
            f"[建图状态] 深度图像:{hz['depth']:.1f}Hz | 雷达点云:{hz['lidar']:.1f}Hz | "
            f"深度点云:{hz['depth_cloud']:.1f}Hz | 融合点云:{hz['combined_cloud']:.1f}Hz | "
            f"SLAM后端:{status_str} | 回环次数:{self.loops}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-file', default='')
    args = parser.parse_args()
    rclpy.init()
    node = DepthLoopDiagnostics(args.log_file)
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
