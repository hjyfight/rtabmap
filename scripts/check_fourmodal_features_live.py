#!/usr/bin/env python3
"""Read-only validation of live physical-sensor features and MITI CDR delivery."""
import argparse
import base64
import json
from pathlib import Path
import subprocess
import tempfile
import time
import urllib.request
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rtabmap_features_msgs.msg import FeatureSet

PROJECT = Path(__file__).resolve().parents[1]
APP = PROJECT / '.tools/miti/root/usr/lib/miti'


def verify_flow(visual, feature=None):
    assert visual['flow_version'] == 1
    raw = np.asarray([row['encoder_features'] for row in visual['modalities']], dtype=np.float32)
    normalized = np.asarray([row['features'] for row in visual['modalities']], dtype=np.float32)
    weights = np.asarray([row['weight'] for row in visual['modalities']], dtype=np.float32)
    np.testing.assert_allclose(normalized, raw / np.linalg.norm(raw.astype(np.float64), axis=1)[:, None], atol=1e-7)
    np.testing.assert_array_equal(visual['joint_features'], normalized.reshape(-1))
    mean = (normalized * weights[:, None]).sum(axis=0)
    np.testing.assert_array_equal(visual['weighted_mean_features'], mean)
    np.testing.assert_allclose(visual['weighted_mean_norm'], np.linalg.norm(mean.astype(np.float64)), atol=1e-7)
    np.testing.assert_allclose(visual['fused_features'], mean / np.linalg.norm(mean.astype(np.float64)), atol=1e-7)
    if feature is not None:
        np.testing.assert_array_equal(raw.reshape(-1), feature.encoder_features)
        np.testing.assert_array_equal(normalized.reshape(-1), feature.independent_features)
        np.testing.assert_array_equal(visual['joint_features'], feature.joint_features)
        np.testing.assert_array_equal(visual['fused_features'], feature.fused_features)
        for row in visual['modalities']:
            assert row['topic'] and row['preprocessing']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=40)
    parser.add_argument('--bridge-port', type=int, default=8765)
    parser.add_argument('--web-url', default='http://127.0.0.1:3002')
    parser.add_argument('--depth-preview', action='store_true', help='Also require live auxiliary depth through MITI')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('fourmodal_physical_feature_check')
    features, visuals, spectra = [], [], []
    subscriptions = [
        node.create_subscription(FeatureSet, '/multimodal/features', features.append, 2),
        node.create_subscription(String, '/miti/fourmodal_features', visuals.append, 2),
        node.create_subscription(Image, '/miti/audio_spectrum/image_raw', spectra.append, 2)]
    client = None
    directory = tempfile.TemporaryDirectory(prefix='fourmodal_live_bridge_')
    started = time.monotonic()
    try:
        frame_file = Path(directory.name) / 'frames.json'
        client = subprocess.Popen([str(APP / 'node/bin/node'), str(PROJECT / 'tests/capture_fourmodal_bridge.js'),
            str(args.bridge_port), str(frame_file)] + (['/miti/aligned_depth/image_raw'] if args.depth_preview else []),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(features) >= 3 and len(visuals) >= 3 and spectra and client.poll() == 0:
                break
        if not features:
            raise RuntimeError('No full physical-sensor feature output; inspect feature node status/log')
        for feature in features:
            assert feature.complete and all(feature.valid_mask) and feature.fused_valid
            assert feature.model_sha256 == '9c4793c7bd5c56d3fc948769f953f45a2e1cee0af1a068137e9407c1ba86d9b0'
            independent = np.asarray(feature.independent_features).reshape(4, 1024)
            np.testing.assert_allclose(np.linalg.norm(independent, axis=1), 1, atol=1e-5)
            np.testing.assert_allclose(np.linalg.norm(feature.fused_features), 1, atol=1e-5)
            report = json.loads(feature.metadata_json)
            assert report['context']['alignment'] == 'estimated_clock_offsets'
            assert report['samples']['audio']['attributes']['stamp_quality'] == 'host_read_end_estimate'
            assert report['provenance']['parameters_trainable'] == 0
            for i, name in enumerate(feature.modality_order):
                assert abs(feature.source_stamps[i] - report['samples'][name]['stamp']) < 1e-6
            assert abs(feature.header.stamp.sec + feature.header.stamp.nanosec / 1e9 - feature.source_stamps[0]) < 1e-6
        assert visuals and spectra
        visual_by_sequence = {v['sequence']: v for v in (json.loads(message.data) for message in visuals)}
        matched = 0
        for feature in features:
            if feature.sequence in visual_by_sequence:
                verify_flow(visual_by_sequence[feature.sequence], feature)
                matched += 1
        assert matched >= 3, f'Only {matched} matching FeatureSet/visualization pairs'
        assert len(spectra[-1].data) == 128 * 204 * 3 and spectra[-1].encoding == 'rgb8'
        if client.poll() is None:
            _, error = client.communicate(timeout=5)
        else:
            _, error = client.communicate()
        assert client.returncode == 0, error
        frames = json.loads(frame_file.read_text())
        decoded = {topic: deserialize_message(base64.b64decode(data), Image if topic.endswith('image_raw') else String)
                   for topic, data in frames.items()}
        visual = json.loads(decoded['/miti/fourmodal_features'].data)
        assert visual['mode'] == 'live' and visual['complete'] and visual['alignment'] == 'estimated_clock_offsets'
        verify_flow(visual)
        depth_info = None
        if args.depth_preview:
            depth = decoded['/miti/aligned_depth/image_raw']
            assert depth.width > 0 and depth.height > 0 and len(depth.data) == depth.step * depth.height
            assert depth.encoding.lower() in ('16uc1', '32fc1', 'mono16')
            depth_info = dict(width=depth.width, height=depth.height, encoding=depth.encoding,
                              bytes=len(depth.data), frame_id=depth.header.frame_id)
        payload = Path(directory.name) / 'snapshot.json'
        payload.write_text(json.dumps(visual))
        subprocess.run([str(APP / 'node/bin/node'), str(PROJECT / 'tests/render_fourmodal_bridge.js'), str(payload)], check=True)
        http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with http.open(args.web_url + '/api/bridge', timeout=5) as response:
            bridge = json.load(response)
        # A frontend reusing an existing bridge has no bridge control endpoint.
        # Its connectivity is established by the actual CDR frames above.
        if bridge.get('available'):
            assert bridge['state'] == 'running' and bridge['domainId'] == 0
        else:
            assert bridge.get('available') is False
        with http.open(args.web_url + '/rtabmap-panel-assets.json', timeout=5) as response:
            assets = json.load(response)['assets']
        for asset in assets:
            with http.open(args.web_url + asset, timeout=5) as response:
                assert 'rtabmapFourmodalFactory' in response.read().decode()
        with http.open(args.web_url + '/rtabmap-fusion-layout.json', timeout=5) as response:
            layout = json.load(response)
        assert not any(widget.get('props', {}).get('topic') == '/arc_camera/image_raw' for widget in layout['widgets'])
        results = [json.loads(feature.metadata_json) for feature in features]
        report = dict(passed=True, physical_sensor_data=True, complete_feature_sets=len(features),
            elapsed_s=time.monotonic() - started, cpu_inference_s=[result['total_inference_s'] for result in results],
            feature_dim=1024, joint_dim=4096, all_features_normalized=True,
            exact_feature_flow_pairs=matched, raw_intermediate_and_output_verified=True,
            auxiliary_depth_bridge=depth_info, fusion_layout_without_arc=True,
            source_timestamps_preserved=True, matching_policy='estimated_clock_offsets',
            clock_offsets_s=results[-1]['context']['clock_offsets_s'],
            time_gaps_s_after_offset=results[-1]['time_gaps_s'],
            foxglove_cdr_verified=True, native_panel_parser_and_react_verified=True,
            bridge_control_available=bool(bridge.get('available')),
            miti_spectrum_shape=[128, 204, 3], http_assets_verified=True,
            browser_visual_rendering_verified=False, hardware_synchronization_validated=False,
            microphone_colocation_validated=False, semantic_accuracy_or_fusion_gain_verified=False,
            model_sha256=features[-1].model_sha256, last_result=results[-1])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'last_result'}, indent=2))
    finally:
        if client and client.poll() is None:
            client.terminate(); client.wait(timeout=4)
        directory.cleanup(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
