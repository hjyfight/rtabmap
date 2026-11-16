#!/usr/bin/env python3
"""Publish live ALSA PCM windows. Host-estimated stamps are explicitly marked."""
import argparse
from collections import deque
import subprocess
import signal
import tempfile
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtabmap_features_msgs.msg import AudioFrame


def main():
    def shutdown_signal(signum, frame):
        raise KeyboardInterrupt
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--alsa-device', default='hw:CARD=XFMDPV0018,DEV=0')
    p.add_argument('--sample-rate', type=int, default=16000)
    p.add_argument('--channels', type=int, default=1)
    p.add_argument('--topic', default='/multimodal/audio')
    p.add_argument('--frame-id', default='microphone_array')
    p.add_argument('--window-seconds', type=float, default=2.0)
    p.add_argument('--publish-period', type=float, default=0.5)
    args, ros_args = p.parse_known_args()
    if not 8000 <= args.sample_rate <= 192000 or not 1 <= args.channels <= 64:
        p.error('Invalid audio format')
    if not 0.1 <= args.publish_period <= args.window_seconds <= 10:
        p.error('Require 0.1 <= publish-period <= window-seconds <= 10')
    rclpy.init(args=ros_args)
    signal.signal(signal.SIGTERM, shutdown_signal)
    node = rclpy.create_node('fourmodal_audio_capture')
    publisher = node.create_publisher(AudioFrame, args.topic, qos_profile_sensor_data)
    chunk_frames = int(args.sample_rate * args.publish_period)
    window_frames = int(args.sample_rate * args.window_seconds)
    chunks = deque(maxlen=int(np.ceil(window_frames / chunk_frames)) + 1)
    process = None
    try:
        with tempfile.TemporaryFile() as errors:
            process = subprocess.Popen([
                'arecord', '-q', '-D', args.alsa_device, '-t', 'raw', '-f', 'S16_LE',
                '-r', str(args.sample_rate), '-c', str(args.channels), '--buffer-time=200000'],
                stdout=subprocess.PIPE, stderr=errors)
            sequence = 0
            node.get_logger().info('Capturing live audio; stamps are host_read_end_estimate, not ADC hardware time')
            while rclpy.ok():
                payload = process.stdout.read(chunk_frames * args.channels * 2)
                received_ns = node.get_clock().now().nanoseconds
                if not rclpy.ok():
                    break
                if len(payload) != chunk_frames * args.channels * 2:
                    errors.seek(0)
                    raise RuntimeError('ALSA capture ended: ' + errors.read().decode(errors='replace'))
                chunks.append(np.frombuffer(payload, '<i2').reshape(-1, args.channels).astype(np.float32) / 32768)
                window = np.concatenate(chunks, axis=0)[-window_frames:]
                if len(window) < window_frames:
                    continue
                sequence += 1
                message = AudioFrame()
                first_ns = received_ns - int(round(len(window) / args.sample_rate * 1e9))
                message.header.stamp.sec, message.header.stamp.nanosec = divmod(first_ns, 1000000000)
                message.header.frame_id = args.frame_id
                message.sample_rate, message.channels = args.sample_rate, args.channels
                message.samples = window.reshape(-1).tolist()
                message.source_id = f'alsa:{args.alsa_device}:{first_ns}:{sequence}'
                message.stamp_quality = 'host_read_end_estimate'
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        if process:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
