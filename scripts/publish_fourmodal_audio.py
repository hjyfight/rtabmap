#!/usr/bin/env python3
"""Publish live ALSA PCM windows. Host-estimated stamps are explicitly marked."""
import argparse
from collections import deque
import os
import select
import subprocess
import signal
import tempfile
import time
import numpy as np


def read_pcm_chunk(stream, size, timeout, running):
    """Read one complete chunk, handling pipe fragments and a stalled recorder."""
    payload = bytearray()
    deadline = time.monotonic() + timeout
    while len(payload) < size and running():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError('ALSA capture timed out')
        ready, _, _ = select.select([stream], [], [], min(remaining, .2))
        if not ready:
            continue
        part = os.read(stream.fileno(), size - len(payload))
        if not part:
            raise RuntimeError('ALSA capture ended')
        payload.extend(part)
    return bytes(payload) if len(payload) == size else None


def stop_recorder(process):
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    if process.stdout:
        process.stdout.close()


def capture_payloads(command, chunk_bytes, *, running, on_retry, timeout=3.0,
                     retry_seconds=2.0, wait=time.sleep, process_factory=subprocess.Popen):
    """Retry this same ALSA device; None marks a new continuous recording session."""
    while running():
        process = None
        reason = ''
        with tempfile.TemporaryFile() as errors:
            try:
                process = process_factory(command, stdout=subprocess.PIPE, stderr=errors, bufsize=0)
                # Never splice pre-disconnect samples into a reconnected window.
                yield None
                while running():
                    payload = read_pcm_chunk(process.stdout, chunk_bytes, timeout, running)
                    if payload is not None:
                        yield payload
            except (OSError, RuntimeError) as error:
                reason = str(error)
            finally:
                stop_recorder(process)
            if reason:
                errors.seek(0)
                detail = errors.read(4096).decode(errors='replace').strip()
                reason += ': ' + detail if detail else ''
        if running():
            on_retry(reason)
            wait(retry_seconds)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rtabmap_features_msgs.msg import AudioFrame

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
    p.add_argument('--retry-seconds', type=float, default=2.0)
    args, ros_args = p.parse_known_args()
    if not 8000 <= args.sample_rate <= 192000 or not 1 <= args.channels <= 64:
        p.error('Invalid audio format')
    if not 0.1 <= args.publish_period <= args.window_seconds <= 10:
        p.error('Require 0.1 <= publish-period <= window-seconds <= 10')
    if not 0.25 <= args.retry_seconds <= 30:
        p.error('Require 0.25 <= retry-seconds <= 30')
    rclpy.init(args=ros_args)
    signal.signal(signal.SIGTERM, shutdown_signal)
    node = rclpy.create_node('fourmodal_audio_capture')
    publisher = node.create_publisher(AudioFrame, args.topic, qos_profile_sensor_data)
    chunk_frames = int(args.sample_rate * args.publish_period)
    window_frames = int(args.sample_rate * args.window_seconds)
    chunks = deque(maxlen=int(np.ceil(window_frames / chunk_frames)) + 1)
    command = ['arecord', '-q', '-D', args.alsa_device, '-t', 'raw', '-f', 'S16_LE',
               '-r', str(args.sample_rate), '-c', str(args.channels), '--buffer-time=200000']
    capture = capture_payloads(command, chunk_frames * args.channels * 2,
        running=rclpy.ok, timeout=max(3.0, args.publish_period * 4), retry_seconds=args.retry_seconds,
        on_retry=lambda reason: node.get_logger().warning(
            f'Audio interrupted; retrying {args.alsa_device} in {args.retry_seconds:g}s: {reason}'))
    sequence = 0
    try:
        node.get_logger().info('Capturing live audio; stamps are host_read_end_estimate, not ADC hardware time')
        for payload in capture:
            if payload is None:
                chunks.clear()
                continue
            received_ns = node.get_clock().now().nanoseconds
            if not rclpy.ok():
                break
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
        capture.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
