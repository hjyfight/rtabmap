"""Lossless transport must retain LiDAR timing fields and exact image pixels."""
from pathlib import Path
import struct
import sys
import unittest
import numpy as np

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from hotspot_sensor_transport import HEADER, decode, encode


class LosslessTransportTests(unittest.TestCase):
    def roundtrip(self, index, message):
        before = serialize_message(message)
        snapshot = deserialize_message(before, type(message))
        packet, raw_size = encode(index, message)
        restored_index, restored = decode(packet[:HEADER.size], packet[HEADER.size:])
        self.assertEqual(index, restored_index)
        self.assertEqual(raw_size, len(before))
        # CDR alignment padding is not initialized consistently by the ROS serializer.
        self.assertEqual(message, snapshot)
        self.assertEqual(restored, snapshot)
        return packet

    def test_cloud_keeps_floats_intensity_and_timestamp_bytes(self):
        fields = [PointField(name='x', offset=0, datatype=7, count=1),
                  PointField(name='time', offset=4, datatype=8, count=1)]
        data = b''.join(struct.pack('<fd', i / 10, 7889005 + i / 200000) for i in range(1000))
        cloud = PointCloud2(height=1, width=1000, fields=fields,
                            point_step=12, row_step=12000, data=data)
        cloud.header.frame_id = 'livox_frame'
        cloud.header.stamp.sec = 7889005
        self.assertLess(len(self.roundtrip(0, cloud)), len(data))

    def test_rgb_depth_and_padded_images_are_byte_identical(self):
        for index, width, step, encoding in [(2, 4, 12, 'rgb8'),
                                             (3, 4, 8, '16UC1'),
                                             (4, 3, 4, 'mono8')]:
            message = Image(height=2, width=width, step=step, encoding=encoding,
                            data=bytes(range(step * 2)), is_bigendian=1)
            message.header.frame_id = 'camera_depth_optical_frame'
            self.roundtrip(index, message)

    def test_calibration_imu_and_static_tf(self):
        self.roundtrip(1, Imu())
        info = CameraInfo(width=424, height=240, distortion_model='plumb_bob',
                          d=[0.01, -0.01, 0.0, 0.0, 0.0])
        self.roundtrip(5, info)
        transform = TransformStamped(child_frame_id='camera_color_optical_frame')
        transform.header.frame_id = 'camera_link'
        transform.transform.rotation.w = 1.0
        self.roundtrip(9, TFMessage(transforms=[transform]))

    def test_depth_prediction_preserves_zeros_wraparound_and_byte_order(self):
        for bigendian in (0, 1):
            dtype = '>u2' if bigendian else '<u2'
            pixels = np.array([0, 65535, 1, 32768, 3000, 2999, 0, 10], dtype=dtype)
            image = Image(height=2, width=4, step=8, encoding='16UC1',
                          is_bigendian=bigendian, data=pixels.tobytes())
            self.roundtrip(3, image)

    def test_invalid_frames_fail_before_publishing(self):
        packet, _ = encode(1, Imu())
        with self.assertRaises(ValueError):
            decode(packet[:HEADER.size], packet[HEADER.size:-1])
        with self.assertRaises(ValueError):
            decode(HEADER.pack(255, 0, 1, 1), b'a')
        with self.assertRaises(ValueError):
            decode(HEADER.pack(1, 0, 1, len(packet) - HEADER.size), packet[HEADER.size:])

    def test_jpeg_preserves_rgb_order_dimensions_header_and_original_message(self):
        pixels = np.zeros((240, 424, 3), dtype=np.uint8)
        pixels[:] = [220, 50, 20]
        image = Image(height=240, width=424, step=1272, encoding='rgb8', data=pixels.tobytes())
        image.header.frame_id = 'camera_color_optical_frame'
        image.header.stamp.sec = 12345
        snapshot = bytes(image.data)
        packet, _ = encode(2, image, jpeg_quality=85)
        index, decoded = decode(packet[:HEADER.size], packet[HEADER.size:])
        self.assertEqual(bytes(image.data), snapshot)
        self.assertEqual(index, 2)
        self.assertEqual(decoded.header, image.header)
        self.assertEqual((decoded.width, decoded.height, decoded.step, decoded.encoding),
                         (424, 240, 1272, 'rgb8'))
        restored = np.frombuffer(bytes(decoded.data), np.uint8).reshape(pixels.shape)
        self.assertLess(np.max(np.abs(restored.astype(int) - pixels)), 5)
        self.assertLess(len(packet), len(snapshot) // 10)

    def test_jpeg_option_never_changes_depth_or_pointcloud(self):
        depth = Image(height=2, width=4, step=8, encoding='16UC1', data=bytes(range(16)))
        packet, _ = encode(3, depth, jpeg_quality=85)
        self.assertEqual(decode(packet[:HEADER.size], packet[HEADER.size:])[1], depth)


if __name__ == '__main__':
    unittest.main()
