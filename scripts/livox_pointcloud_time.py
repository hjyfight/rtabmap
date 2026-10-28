#!/usr/bin/env python3
"""Normalize Livox absolute FLOAT64 point times without changing acquisition time.

RTAB-Map versions that infer units from Unix-epoch magnitudes misread Livox
device-uptime nanoseconds. Match units against the cloud header instead.
Coordinates, the header, other fields, row padding and point order are preserved.
"""

from array import array
import copy

import numpy as np


TIME_SCALES = {
    'seconds': 1.0,
    'milliseconds': 1e-3,
    'microseconds': 1e-6,
    'nanoseconds': 1e-9,
}


def normalize_timestamps(cloud, time_unit='auto', max_scan_duration=1.0):
    """Return (cloud with second-based point times, detected input unit).

    Reject timestamps inconsistent with the header rather than retimestamping
    the scan with wall-clock time or feeding invalid times into deskewing.
    """
    if time_unit != 'auto' and time_unit not in TIME_SCALES:
        raise ValueError(f'Unsupported time_unit: {time_unit}')
    if cloud.width == 0 or cloud.height == 0:
        return cloud, 'empty'

    fields = [field for field in cloud.fields if field.name == 'timestamp']
    if len(fields) != 1 or fields[0].datatype != 8 or fields[0].count != 1:
        raise ValueError('Expected one FLOAT64 timestamp field (Livox PointCloud2).')
    field = fields[0]
    if (field.offset + 8 > cloud.point_step or
            cloud.row_step < cloud.width * cloud.point_step or
            len(cloud.data) < cloud.height * cloud.row_step):
        raise ValueError('Invalid PointCloud2 timestamp offset, row stride or data length.')

    dtype = np.dtype('>f8' if cloud.is_bigendian else '<f8')
    times = np.ndarray(
        (cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
        offset=field.offset, strides=(cloud.row_step, cloud.point_step))
    if not np.all(np.isfinite(times)):
        raise ValueError('Non-finite point timestamps.')

    header_seconds = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
    first = float(np.min(times))
    last = float(np.max(times))
    candidates = TIME_SCALES.items() if time_unit == 'auto' else [(time_unit, TIME_SCALES[time_unit])]
    matches = []
    for unit, scale in candidates:
        start, end = first * scale, last * scale
        if (start >= 0 and end - start <= max_scan_duration and
                max(abs(start - header_seconds), abs(end - header_seconds)) <= max_scan_duration):
            matches.append((abs(start - header_seconds), unit, scale))
    if not matches:
        raise ValueError('Point timestamps do not match the header within a one-second scan.')
    # Stable sort prefers seconds for an already normalized scan (including stamp=0).
    _, unit, scale = min(matches, key=lambda match: match[0])
    if scale == 1.0:
        return cloud, unit

    data = bytearray(cloud.data)
    output_times = np.ndarray(
        times.shape, dtype=dtype, buffer=data, offset=field.offset,
        strides=(cloud.row_step, cloud.point_step))
    output_times *= scale
    output = copy.copy(cloud)
    output.data = array('B', data)
    return output, unit


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    class LivoxPointCloudTime(Node):
        def __init__(self):
            super().__init__('livox_pointcloud_time')
            self.unit = self.declare_parameter('time_unit', 'auto').value
            if self.unit != 'auto' and self.unit not in TIME_SCALES:
                raise ValueError(f'Unsupported time_unit: {self.unit}')
            qos_value = self.declare_parameter('qos', 2).value
            reliability = {
                0: ReliabilityPolicy.SYSTEM_DEFAULT,
                1: ReliabilityPolicy.RELIABLE,
                2: ReliabilityPolicy.BEST_EFFORT,
            }
            if qos_value not in reliability:
                raise ValueError('qos must be 0, 1 or 2.')
            self.publisher = self.create_publisher(
                PointCloud2, 'output_cloud', QoSProfile(depth=5, reliability=reliability[qos_value]))
            # A best-effort reader accepts both reliable and best-effort driver publishers.
            self.subscription = self.create_subscription(
                PointCloud2, 'input_cloud', self.on_cloud, qos_profile_sensor_data)
            self.reported_unit = None

        def on_cloud(self, message):
            try:
                output, unit = normalize_timestamps(message, self.unit)
            except ValueError as error:
                self.get_logger().error(f'Dropping invalid timestamp scan: {error}', throttle_duration_sec=5.0)
                return
            if unit != 'empty' and unit != self.reported_unit:
                self.get_logger().info(f'Point timestamps: {unit} -> seconds; acquisition header preserved.')
                self.reported_unit = unit
            self.publisher.publish(output)

    rclpy.init(args=args)
    node = None
    try:
        node = LivoxPointCloudTime()
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
