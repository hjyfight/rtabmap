#!/usr/bin/env python3
"""Single-copy Livox/RealSense transport for a routed phone hotspot.

Run send in an onboard, loopback-only ROS domain; run receive in the mapping
domain. Byte shuffling improves lossless point/depth compression without changing
coordinates, timestamps, depth pixels, calibration, or ROS topic names. Optional
RGB-only JPEG preserves image dimensions and timestamps with lossy color pixels.
"""
import argparse
from array import array
from collections import Counter, OrderedDict, deque
import json
import signal
import socket
import struct
import threading
import time
import zlib

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

STREAMS = [
    ('/livox/lidar', PointCloud2), ('/livox/imu', Imu),
    ('/camera/color/image_raw', Image),
    ('/camera/depth/image_rect_raw', Image),
    ('/camera/infra2/image_rect_raw', Image),
    ('/camera/color/camera_info', CameraInfo),
    ('/camera/depth/camera_info', CameraInfo),
    ('/camera/infra2/camera_info', CameraInfo),
    ('/tf', TFMessage), ('/tf_static', TFMessage),
]
HEADER = struct.Struct('!BBII')  # stream index, shuffle width, raw size, compressed size
MAX_FRAME = 16 * 1024 * 1024


def sender_stream_indices(lidar_only=False, lidar_imu_only=False):
    if lidar_only and lidar_imu_only:
        raise ValueError('choose only one lidar transport mode')
    wanted = None
    if lidar_imu_only:
        wanted = {'/livox/lidar', '/livox/imu', '/tf_static'}
    elif lidar_only:
        wanted = {'/livox/lidar', '/tf_static'}
    return tuple(index for index, (topic, _) in enumerate(STREAMS)
                 if wanted is None or topic in wanted)


def shuffle(data, width, reverse=False):
    """Reversible transpose of byte columns, also preserving row padding bytes."""
    if width <= 1 or len(data) % width:
        raise ValueError('invalid byte shuffle width')
    matrix = np.frombuffer(data, dtype=np.uint8)
    shape = (width, -1) if reverse else (-1, width)
    return matrix.reshape(shape).T.copy().tobytes()


def image_difference(data, message, reverse=False):
    """Reversible pixel predictor; 16-bit depth retains every original depth unit."""
    depth = message.encoding in ('16UC1', 'mono16')
    dtype = np.dtype(('>u2' if message.is_bigendian else '<u2') if depth else 'u1')
    channels = message.step // message.width // dtype.itemsize
    pixels = np.frombuffer(data, dtype=dtype).reshape(-1, channels)
    if reverse:
        return np.cumsum(pixels, axis=0, dtype=dtype).astype(dtype).tobytes()
    residual = pixels.copy()
    residual[1:] = pixels[1:] - pixels[:-1]
    return residual.tobytes()


def encode(index, message, jpeg_quality=0):
    original = None
    width = 0
    if isinstance(message, PointCloud2):
        width = message.point_step
    elif isinstance(message, Image) and message.width and message.step % message.width == 0:
        width = message.step // message.width
    predicted = isinstance(message, Image) and 0 < width < 64
    if not 0 < width < 64 or not len(message.data) or len(message.data) % width:
        width = 0
        predicted = False
    if width and (width > 1 or predicted):
        original = message.data
        data = bytes(original)
        if predicted:
            data = image_difference(data, message)
        if width > 1:
            data = shuffle(data, width)
        message.data = array('B', data)
    else:
        width = 0
    jpeg = (jpeg_quality > 0 and index == 2 and isinstance(message, Image)
            and message.encoding in ('rgb8', 'bgr8') and message.step == message.width * 3)
    if jpeg:
        import cv2
        if original is not None:
            message.data = original
        original = message.data
        pixels = np.frombuffer(bytes(original), np.uint8).reshape(message.height, message.width, 3)
        if message.encoding == 'rgb8':
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        success, packed = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not success:
            raise ValueError('JPEG encoding failed')
        message.data = array('B', packed.tobytes())
        width, predicted = 0, False
    try:
        raw = serialize_message(message)
    finally:
        if original is not None:
            message.data = original
    compressed = zlib.compress(raw, 1)
    mode = width | (128 if predicted else 0) | (64 if jpeg else 0)
    return HEADER.pack(index, mode, len(raw), len(compressed)) + compressed, len(raw)


def decode(header, payload):
    index, width, raw_size, size = HEADER.unpack(header)
    if index >= len(STREAMS) or not 0 < raw_size <= MAX_FRAME or not 0 < size <= MAX_FRAME:
        raise ValueError('invalid transport frame')
    if len(payload) != size:
        raise ValueError('truncated transport frame')
    decoder = zlib.decompressobj()
    raw = decoder.decompress(payload, raw_size + 1)
    if len(raw) != raw_size or not decoder.eof or decoder.unused_data:
        raise ValueError('invalid compressed payload')
    message = deserialize_message(raw, STREAMS[index][1])
    jpeg = bool(width & 64)
    predicted = bool(width & 128)
    width &= 63
    if jpeg:
        import cv2
        if index != 2 or predicted or width or message.encoding not in ('rgb8', 'bgr8'):
            raise ValueError('invalid JPEG image frame')
        pixels = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
        if pixels is None or pixels.shape != (message.height, message.width, 3):
            raise ValueError('invalid JPEG dimensions')
        if message.encoding == 'rgb8':
            pixels = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
        message.data = array('B', pixels.tobytes())
    if width:
        if not isinstance(message, (PointCloud2, Image)):
            raise ValueError('shuffle is only valid for point clouds and images')
        data = bytes(message.data)
        if width > 1:
            data = shuffle(data, width, reverse=True)
        if predicted:
            if not isinstance(message, Image):
                raise ValueError('pixel prediction is only valid for images')
            data = image_difference(data, message, reverse=True)
        message.data = array('B', data)
    return index, message


def read_exact(connection, size, stop):
    parts = bytearray()
    while len(parts) < size and not stop.is_set():
        data = connection.recv(size - len(parts))
        if not data:
            raise ConnectionError('peer disconnected')
        parts.extend(data)
    if len(parts) != size:
        raise ConnectionError('stopping')
    return bytes(parts)


class Transport(Node):
    def __init__(self, args):
        super().__init__('hotspot_sensor_' + args.mode)
        self.args = args
        self.stop = threading.Event()
        self.condition = threading.Condition()
        self.latest = OrderedDict()
        self.imu = deque(maxlen=512)
        self.static = {}
        self.received_static = None
        self.connected = False
        self.active_connection = None
        self.stats = Counter()
        self.started = time.monotonic()
        self.last_link_warning = 0.0
        self.outputs = []
        self.subscriptions_owned = []
        send_indices = sender_stream_indices(getattr(args, 'lidar_only', False),
                                             getattr(args, 'lidar_imu_only', False))
        for index, (topic, kind) in enumerate(STREAMS):
            if args.mode == 'send' and index not in send_indices:
                continue
            qos = QoSProfile(depth=100 if kind is Imu else 5,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            if kind in (PointCloud2, CameraInfo, TFMessage) or topic in (
                    '/camera/depth/image_rect_raw', '/camera/infra2/image_rect_raw'):
                qos.reliability = ReliabilityPolicy.RELIABLE
            if topic == '/tf_static':
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if args.mode == 'send':
                self.subscriptions_owned.append(self.create_subscription(
                    kind, topic, lambda msg, i=index: self.queue(i, msg), qos))
            else:
                self.outputs.append(self.create_publisher(kind, topic, qos))
        self.create_timer(10, self.report)
        if args.mode == 'receive':
            # Some display bridges choose volatile QoS when static TF has multiple
            # publishers. Replay the same complete snapshot for late subscribers.
            self.create_timer(2, self.replay_static)
            self.subscriptions_owned.append(self.create_subscription(
                TFMessage, '/tf_static', self.cache_static,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.thread = threading.Thread(target=self.network, daemon=True)
        self.thread.start()

    def queue(self, index, message):
        if STREAMS[index][0] == '/tf_static':
            with self.condition:
                for transform in message.transforms:
                    self.static[transform.child_frame_id] = transform
                message = TFMessage(transforms=list(self.static.values()))
        if not self.connected and index != len(STREAMS) - 1:
            return
        packet, raw_size = encode(index, message, self.args.jpeg_quality)
        with self.condition:
            self.stats['raw_bytes'] += raw_size
            self.stats['encoded_bytes'] += len(packet)
            self.stats['input_' + STREAMS[index][0]] += 1
            if STREAMS[index][1] is Imu:
                if len(self.imu) == self.imu.maxlen:
                    self.stats['imu_queue_drops'] += 1
                self.imu.append((index, packet))
            else:
                if index in self.latest and index != len(STREAMS) - 1:
                    self.stats['latest_queue_drops'] += 1
                self.latest[index] = packet
            self.condition.notify()

    def send_session(self, connection):
        with self.condition:
            self.latest.clear()
            self.imu.clear()
            self.connected = True
            if self.static:
                msg = TFMessage(transforms=list(self.static.values()))
                self.latest[len(STREAMS) - 1] = encode(len(STREAMS) - 1, msg)[0]
        while not self.stop.is_set():
            with self.condition:
                if not self.latest and not self.imu:
                    self.condition.wait(0.5)
                    continue
                # Drain IMU batches, then one large stream; neither can starve.
                batch = list(self.imu)
                self.imu.clear()
                if self.latest:
                    batch.append(self.latest.popitem(last=False))
            for index, packet in batch:
                connection.sendall(packet)
                self.stats['wire_bytes'] += len(packet)
                self.stats['sent_' + STREAMS[index][0]] += 1

    def receive_session(self, connection):
        self.connected = True
        while not self.stop.is_set():
            header = read_exact(connection, HEADER.size, self.stop)
            index, width, raw_size, size = HEADER.unpack(header)
            if index >= len(STREAMS) or not 0 < size <= MAX_FRAME or not 0 < raw_size <= MAX_FRAME:
                raise ValueError('invalid transport header')
            payload = read_exact(connection, size, self.stop)
            index, message = decode(header, payload)
            if STREAMS[index][0] == '/tf_static':
                self.cache_static(message)
            self.outputs[index].publish(message)
            self.stats['wire_bytes'] += HEADER.size + size
            self.stats['received_' + STREAMS[index][0]] += 1

    def replay_static(self):
        if self.received_static is not None:
            self.outputs[len(STREAMS) - 1].publish(self.received_static)

    def cache_static(self, message):
        # Include the mapping launch's camera mounting transform as well as the
        # onboard camera tree; volatile bridges need both after a page reload.
        with self.condition:
            for transform in message.transforms:
                self.static[transform.child_frame_id] = transform
            self.received_static = TFMessage(transforms=list(self.static.values()))

    def network(self):
        listener = None
        try:
            if self.args.mode == 'send':
                listener = socket.socket()
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.args.host, self.args.port))
                listener.listen(1)
                listener.settimeout(1)
            while not self.stop.is_set():
                connection = None
                try:
                    if listener:
                        try:
                            connection, address = listener.accept()
                        except socket.timeout:
                            continue
                    else:
                        connection = socket.create_connection((self.args.host, self.args.port), timeout=5)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection.settimeout(getattr(self.args, 'socket_timeout', 5))
                    self.active_connection = connection
                    self.get_logger().info('Compressed sensor link connected')
                    if listener:
                        self.send_session(connection)
                    else:
                        self.receive_session(connection)
                except (OSError, ValueError, ConnectionError) as error:
                    now = time.monotonic()
                    if not self.stop.is_set() and rclpy.ok() and now - self.last_link_warning >= 10:
                        self.get_logger().warning('Compressed sensor link: ' + str(error))
                        self.last_link_warning = now
                finally:
                    self.connected = False
                    self.active_connection = None
                    if connection:
                        connection.close()
                self.stop.wait(1)
        except OSError as error:
            self.get_logger().error(str(error))
            self.stop.set()
        finally:
            if listener:
                listener.close()

    def report(self):
        elapsed = time.monotonic() - self.started
        result = dict(self.stats)
        result.update(connected=self.connected, seconds=round(elapsed, 1),
                      average_mbps=round(self.stats['wire_bytes'] * 8 / elapsed / 1e6, 2))
        self.get_logger().info(json.dumps(result, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['send', 'receive'])
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('--jpeg-quality', type=int, default=0,
                        help='RGB-only JPEG quality 1..100; 0 keeps RGB lossless')
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--lidar-only', action='store_true',
                           help='Send only lidar and static TF, retaining the existing wire stream IDs')
    selection.add_argument('--lidar-imu-only', action='store_true',
                           help='Send lidar, its IMU and static TF, without camera streams')
    parser.add_argument('--socket-timeout', type=float, default=5,
                        help='Established TCP read/write timeout in seconds')
    args = parser.parse_args()
    if not 0 <= args.jpeg_quality <= 100:
        parser.error('--jpeg-quality must be between 0 and 100')
    if not 0 < args.socket_timeout < float('inf'):
        parser.error('--socket-timeout must be positive and finite')
    # Humble's default C signal handler invalidates the context before the network
    # thread has stopped. Foxy has no signal_handler_options argument.
    import inspect
    if 'signal_handler_options' in inspect.signature(rclpy.init).parameters:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init()
    def stop_signal(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    node = Transport(args)
    try:
        while rclpy.ok() and not node.stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        if node.active_connection:
            try:
                node.active_connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        with node.condition:
            node.condition.notify_all()
        node.thread.join(timeout=6)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
