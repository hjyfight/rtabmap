#!/usr/bin/env python3
"""Continuously reject invalid aligned RGB-D pairs before they reach RTAB-Map."""
import json
import math
import time


def input_qos_profile(value):
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1, 2):
        raise ValueError('input_qos must be 0 (default), 1 (reliable) or 2 (best effort)')
    reliability = {0: ReliabilityPolicy.SYSTEM_DEFAULT, 1: ReliabilityPolicy.RELIABLE,
                   2: ReliabilityPolicy.BEST_EFFORT}[value]
    return QoSProfile(depth=5, reliability=reliability)


class InputWatchdog:
    """Renew a silent reader only while its source publisher is still present."""
    def __init__(self, reconnect, timeout=5., clock=time.monotonic):
        if isinstance(timeout, bool) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('input timeout must be positive and finite')
        self.reconnect, self.timeout, self.clock = reconnect, timeout, clock
        self.last_received = self.last_reconnected = clock()
        self.reconnections = 0

    def received(self):
        self.last_received = self.clock()

    def check(self, publisher_count):
        now = self.clock()
        age = max(0., now-self.last_received)
        if publisher_count > 0 and age > self.timeout and now-self.last_reconnected >= self.timeout:
            self.reconnect()
            self.last_reconnected = now
            self.reconnections += 1
        return dict(input_age_seconds=age, input_reconnections=self.reconnections)


def validate_rgbd(message, max_interval=0.05):
    """Return a rejection reason, or None; never resize or retimestamp input."""
    rgb, depth, info = message.rgb, message.depth, message.rgb_camera_info
    shape = (rgb.width, rgb.height)
    if min(shape) <= 0:
        return 'RGB image is empty'
    if (depth.width, depth.height) != shape:
        return f'RGB={rgb.width}x{rgb.height}, aligned depth={depth.width}x{depth.height}'
    if (info.width, info.height) != shape:
        return f'RGB={rgb.width}x{rgb.height}, camera_info={info.width}x{info.height}'
    for image, encodings in ((rgb, {'rgb8': 3, 'bgr8': 3, 'mono8': 1}),
                              (depth, {'16UC1': 2, '32FC1': 4})):
        if image.encoding not in encodings:
            return f'Unsupported image encoding: {image.encoding}'
        if image.step < image.width * encodings[image.encoding] or len(image.data) != image.step * image.height:
            return 'Image buffer size/row stride does not match dimensions'
    if not all(math.isfinite(x) for x in info.k) or info.k[0] <= 0 or info.k[4] <= 0:
        return 'Invalid camera intrinsics'
    headers = (rgb.header, depth.header, info.header)
    if not rgb.header.frame_id or any(h.frame_id != rgb.header.frame_id for h in headers):
        return 'RGB, aligned depth and camera_info frames differ'
    stamps = [h.stamp.sec * 1_000_000_000 + h.stamp.nanosec for h in headers]
    if max(stamps) - min(stamps) > round(max_interval * 1_000_000_000):
        return 'RGB, aligned depth and camera_info timestamps exceed sync tolerance'
    return None


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rtabmap_msgs.msg import RGBDImage
    from std_msgs.msg import String

    class RGBDInputGuard(Node):
        def __init__(self):
            super().__init__('rgbd_input_guard')
            self.max_interval = self.declare_parameter('max_interval', 0.05).value
            if not math.isfinite(self.max_interval) or self.max_interval <= 0:
                raise ValueError('max_interval must be positive and finite')
            self.accepted = self.rejected = 0
            self.last_error = ''
            self.publisher = self.create_publisher(
                RGBDImage, 'output', QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
            self.status = self.create_publisher(String, '/semantic/input_status', 1)
            self.input_qos = input_qos_profile(self.declare_parameter('input_qos', 2).value)
            self.input_subscription = self.create_subscription(RGBDImage, 'input', self.on_pair, self.input_qos)
            self.input_watchdog = InputWatchdog(self.reload_input, self.declare_parameter('input_timeout', 5.).value)
            self.create_timer(1.0, self.publish_status)

        def reload_input(self):
            self.destroy_subscription(self.input_subscription)
            self.input_subscription = self.create_subscription(RGBDImage, 'input', self.on_pair, self.input_qos)
            self.get_logger().warning('RGB-D input stopped despite an available publisher; renewed the input subscription.')

        def on_pair(self, message):
            self.input_watchdog.received()
            error = validate_rgbd(message, self.max_interval)
            if error:
                self.rejected += 1
                self.last_error = error
                self.get_logger().error(
                    f'Dropped invalid RGB-D pair ({self.rejected} total): {error}. '
                    'Check camera alignment; this frame was not sent to SLAM.', throttle_duration_sec=5.0)
                return
            self.accepted += 1
            self.publisher.publish(message)

        def publish_status(self):
            delivery = self.input_watchdog.check(self.count_publishers(self.input_subscription.topic_name))
            self.status.publish(String(data=json.dumps(dict(
                accepted=self.accepted, rejected=self.rejected, last_error=self.last_error, **delivery))))

    rclpy.init(args=args)
    node = None
    try:
        node = RGBDInputGuard()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
