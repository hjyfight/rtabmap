"""Loopback-only TCP receive/ROS validation; never connects to robot hardware."""
from copy import deepcopy
import json
import os
from pathlib import Path
import socket
import sys
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rtabmap_features_msgs.msg import AudioFrame

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import hotspot_sensor_transport as transport


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Run this fixture in an isolated localhost-only ROS domain')
    server = socket.socket()
    server.bind(('127.0.0.1', 0))
    server.listen(1)
    server.settimeout(5)
    rclpy.init()
    args = SimpleNamespace(mode='receive', host='127.0.0.1', port=server.getsockname()[1],
                           socket_timeout=10., jpeg_quality=0, audio=True)
    receiver = transport.Transport(args)
    probe = rclpy.create_node('orin_loopback_transport_probe')
    received, subscriptions, connection = {}, [], None
    try:
        for index, (topic, kind) in enumerate(transport.STREAMS):
            received[index] = []
            subscriptions.append(probe.create_subscription(
                kind, topic, received[index].append, qos_profile_sensor_data))
        connection, _ = server.accept()
        deadline = time.monotonic() + .7
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.05)
        messages = [
            PointCloud2(height=1, width=10, point_step=16, row_step=160,
                fields=[PointField(name='x', offset=0, datatype=7, count=1),
                        PointField(name='y', offset=4, datatype=7, count=1),
                        PointField(name='z', offset=8, datatype=7, count=1),
                        PointField(name='time', offset=12, datatype=7, count=1)],
                data=np.arange(40, dtype='<f4').tobytes()),
            Imu(),
            Image(height=16, width=24, step=72, encoding='rgb8', data=np.arange(1152, dtype=np.uint8).tobytes()),
            Image(height=16, width=24, step=48, encoding='16UC1', data=np.arange(384, dtype='<u2').tobytes()),
            Image(height=16, width=24, step=24, encoding='mono8', data=np.arange(384, dtype=np.uint8).tobytes()),
            CameraInfo(width=24, height=16), CameraInfo(width=24, height=16), CameraInfo(width=24, height=16),
            TFMessage(transforms=[TransformStamped(child_frame_id='fixture_dynamic')]),
            TFMessage(transforms=[TransformStamped(child_frame_id='fixture_static')]),
            AudioFrame(sample_rate=16000, channels=1, samples=np.linspace(-.1, .1, 32000, dtype=np.float32).tolist(),
                       source_id='fixture-audio', stamp_quality='synthetic_test_clock')]
        for message in messages:
            if hasattr(message, 'header'):
                message.header.frame_id = 'fixture_sensor'
                message.header.stamp.sec = 123
                message.header.stamp.nanosec = 456
            else:
                message.transforms[0].header.frame_id = 'fixture_parent'
                message.transforms[0].transform.rotation.w = 1.
        snapshots = deepcopy(messages)
        packets = b''.join(transport.encode(i, message)[0] for i, message in enumerate(messages))
        # Split arbitrary packet boundaries to exercise exact TCP framing.
        connection.sendall(packets[:7])
        connection.sendall(packets[7:91])
        connection.sendall(packets[91:])
        deadline = time.monotonic() + 8
        while not all(received.values()) and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=.05)
        assert all(received.values()), {i: len(v) for i, v in received.items()}
        for i, original in enumerate(snapshots):
            assert original == received[i][0], f'Payload or metadata changed in stream {i}'
        assert transport.STATIC_STREAM_INDEX == 9
        assert transport.STREAMS[10][0] == '/multimodal/audio'
        result = dict(passed=True, loopback_only=True, robot_contacted=False,
                      streams=len(messages), legacy_stream_ids_preserved=True,
                      rgb_depth_nir_bytes_identical=True, lidar_timing_preserved=True,
                      audio_samples_and_stamp_provenance_preserved=True)
        output = ROOT / 'logs/orin_perception/network_transport.json'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result))
    finally:
        receiver.stop.set()
        if receiver.active_connection:
            try:
                receiver.active_connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        receiver.thread.join(timeout=6)
        if connection:
            connection.close()
        server.close()
        receiver.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
