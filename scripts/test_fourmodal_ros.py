#!/usr/bin/env python3
"""Isolated ROS transport validation, optionally using the real frozen CPU model."""
import argparse
import base64
import json
from pathlib import Path
import sys
import subprocess
import tempfile
import time
import urllib.request
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from cv_bridge import CvBridge
from rtabmap_features_msgs.msg import AudioFrame, FeatureSet

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_features'))
from rtabmap_features.node import FourModalNode
from rtabmap_features.bundle import load_bundle
from rtabmap_features.fusion import MODALITIES


class TransportFixture:
    provenance = dict(backend='transport_fixture_not_pretrained', checkpoint_sha256='fixture',
                      parameters_trainable=0)

    def encode(self, samples, anchor):
        result = {}
        for i, name in enumerate(MODALITIES):
            if name in samples:
                vector = np.zeros(1024, np.float32)
                vector[i] = 1
                result[name] = vector
        return result, {}, {}


def set_stamp(header, seconds):
    header.stamp.sec, header.stamp.nanosec = divmod(int(round(seconds * 1e9)), 1000000000)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--transport-only', action='store_true')
    p.add_argument('--device', choices=['cpu', 'cuda:0'], default='cpu')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--bridge-port', type=int)
    p.add_argument('--http-port', type=int)
    args = p.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'min_anchor_period_s:=0.0', '-p', 'max_input_age_s:=10.0',
                    '-p', 'device:=' + args.device])
    node, sender, executor, bridge_client = None, None, None, None
    bridge_directory = tempfile.TemporaryDirectory(prefix='fourmodal_bridge_')
    try:
        if args.bridge_port:
            bridge_file = Path(bridge_directory.name) / 'frames.json'
            bridge_client = subprocess.Popen([
                str(PROJECT / '.tools/miti/root/usr/lib/miti/node/bin/node'),
                str(PROJECT / 'tests/capture_fourmodal_bridge.js'), str(args.bridge_port), str(bridge_file)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if args.http_port:
            http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            base = f'http://127.0.0.1:{args.http_port}'
            with http.open(base + '/rtabmap-fourmodal-demo.json', timeout=5) as response:
                assert json.load(response)['mode'] == 'saved'
            with http.open(base + '/rtabmap-fourmodal-audio.png', timeout=5) as response:
                assert response.read().startswith(b'\x89PNG')
            with http.open(base + '/rtabmap-panel-assets.json', timeout=5) as response:
                assets = json.load(response)['assets']
            for asset in assets:
                with http.open(base + asset, timeout=5) as response:
                    assert 'rtabmapFourmodalFactory' in response.read().decode()
        node = FourModalNode(encoder=TransportFixture() if args.transport_only else None)
        sender = rclpy.create_node('fourmodal_test_sender')
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.add_node(sender)
        outputs = []
        subscription = sender.create_subscription(FeatureSet, '/multimodal/features', outputs.append, 2)
        visualizations, spectra, statuses = [], [], []
        visualization_subscription = sender.create_subscription(String, '/miti/fourmodal_features',
                                                                 visualizations.append, 2)
        spectrum_subscription = sender.create_subscription(Image, '/miti/audio_spectrum/image_raw', spectra.append, 2)
        status_subscription = sender.create_subscription(String, '/miti/fourmodal_status', statuses.append, 2)
        publishers = {
            name: sender.create_publisher(kind, topic, qos_profile_sensor_data)
            for name, kind, topic in [
                ('rgb', Image, '/camera/color/image_raw'),
                ('infrared', Image, '/camera/infra2/image_rect_raw'),
                ('lidar', PointCloud2, '/livox/lidar'),
                ('audio', AudioFrame, '/multimodal/audio')]}
        samples, manifest = load_bundle(PROJECT / 'logs/fourmodal/smoke_inputs/manifest.json',
                                         allow_unaligned=True)
        bridge = CvBridge()

        def publish(stamp, good_audio):
            rgb = bridge.cv2_to_imgmsg(samples['rgb'].payload, 'rgb8')
            ir = bridge.cv2_to_imgmsg(samples['infrared'].payload, 'mono8')
            rgb.header.frame_id, ir.header.frame_id = 'fixture_rgb', 'fixture_infrared'
            set_stamp(rgb.header, stamp)
            set_stamp(ir.header, stamp + 0.01)
            header = Header(frame_id='fixture_lidar')
            set_stamp(header, stamp - 0.01)
            cloud = point_cloud2.create_cloud_xyz32(header, samples['lidar'].payload)
            audio = AudioFrame()
            audio.header.frame_id = 'fixture_array'
            set_stamp(audio.header, stamp - 1 if good_audio else stamp - 20)
            audio.sample_rate = 16000
            audio.channels = 1
            t = np.arange(32000) / 16000
            audio.samples = (0.1 * np.sin(2 * np.pi * 440 * t)).astype(np.float32).tolist()
            audio.source_id = f'fixture-audio-{stamp}-{good_audio}'
            audio.stamp_quality = 'synthetic_test_clock'
            for name, message in [('rgb', rgb), ('infrared', ir), ('lidar', cloud), ('audio', audio)]:
                publishers[name].publish(message)

        def spin_for(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=0.05)

        spin_for(0.5)
        stale_stamp = sender.get_clock().now().nanoseconds / 1e9
        for _ in range(5):
            publish(stale_stamp, False)
            spin_for(0.1)
        assert not outputs, 'Stale audio incorrectly produced a complete fusion result'
        stamp = sender.get_clock().now().nanoseconds / 1e9
        for _ in range(3):
            publish(stamp, True)
            spin_for(0.1)
        deadline = time.monotonic() + 180
        while not outputs and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not outputs:
            raise RuntimeError(f'No feature output: {node.state}')
        result = outputs[0]
        assert result.complete and list(result.valid_mask) == [True] * 4
        assert list(result.modality_order) == list(MODALITIES)
        independent = np.asarray(result.independent_features).reshape(4, 1024)
        np.testing.assert_allclose(np.linalg.norm(independent, axis=1), 1, atol=1e-5)
        np.testing.assert_array_equal(result.independent_features, result.joint_features)
        assert len(result.encoder_features) == 4096 and len(result.fused_features) == 1024
        assert abs(result.source_stamps[1] - result.source_stamps[0] - 0.01) < 1e-5
        report = json.loads(result.metadata_json)
        assert report['samples']['infrared']['frame_id'] == 'fixture_infrared'
        assert report['samples']['audio']['attributes']['stamp_quality'] == 'synthetic_test_clock'
        # Republishing the identical RGB acquisition must not run another inference.
        before = node.sequence
        publish(stamp, True)
        spin_for(0.5)
        assert node.sequence == before, 'Duplicate RGB acquisition was inferred again'
        assert visualizations and spectra and statuses, 'MITI feature/spectrum/status messages missing'
        visual = json.loads(visualizations[0].data)
        assert visual['mode'] == 'live' and visual['complete']
        np.testing.assert_allclose(visual['fused_features'], result.fused_features)
        assert len(visual['modalities']) == 4 and len(visual['cosine_similarities']) == 4
        spectrum = spectra[0]
        assert spectrum.encoding == 'rgb8' and (spectrum.height, spectrum.width) == (128, 204)
        assert len(spectrum.data) == 128 * 204 * 3
        # Spectrum header records the audio crop's source time, not publication time.
        assert abs(spectrum.header.stamp.sec + spectrum.header.stamp.nanosec / 1e9 - (stamp - 1)) < 1e-5
        assert json.loads(statuses[-1].data)['schema'] == 1
        if bridge_client:
            _, error = bridge_client.communicate(timeout=10)
            assert bridge_client.returncode == 0, error
            frames = json.loads(bridge_file.read_text())
            decoded = {topic: deserialize_message(base64.b64decode(data),
                Image if topic.endswith('image_raw') else String) for topic, data in frames.items()}
            bridged = json.loads(decoded['/miti/fourmodal_features'].data)
            np.testing.assert_allclose(bridged['fused_features'], result.fused_features)
            assert len(decoded['/miti/audio_spectrum/image_raw'].data) == 128 * 204 * 3
            assert json.loads(decoded['/miti/fourmodal_status'].data)['schema'] == 1
            # Exercise the actual panel parser/rendering with the bridged payload.
            visual_file = Path(bridge_directory.name) / 'snapshot.json'
            visual_file.write_text(json.dumps(bridged))
            subprocess.run([str(PROJECT / '.tools/miti/root/usr/lib/miti/node/bin/node'),
                str(PROJECT / 'tests/render_fourmodal_bridge.js'), str(visual_file)], check=True)
        summary = dict(passed=True, mode='isolated_ros_fixture_transport',
                       device=args.device,
                       real_pretrained_model=not args.transport_only,
                       stale_audio_rejected=True, duplicate_rgb_rejected=True,
                       feature_dim=1024, joint_dim=4096, result=report,
                       miti_visualization=True, miti_spectrum_shape=[128, 204, 3],
                       foxglove_cdr_verified=bool(bridge_client), http_assets_verified=bool(args.http_port),
                       note='Fixture sensor messages and synthetic times; not a synchronized hardware accuracy test.')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({k: v for k, v in summary.items() if k != 'result'}, ensure_ascii=False, indent=2))
    finally:
        if bridge_client and bridge_client.poll() is None:
            bridge_client.terminate(); bridge_client.wait(timeout=4)
        bridge_directory.cleanup()
        if node:
            node.close()
        if executor:
            executor.shutdown()
        if sender:
            sender.destroy_node()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
