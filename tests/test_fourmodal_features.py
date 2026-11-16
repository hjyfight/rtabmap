"""Sensor identity, stale-data rejection, feature contracts and CPU point operators."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_features'))
from rtabmap_features.fusion import MODALITIES, Sample, match_samples, fuse_features
from rtabmap_features.preprocessing import image_tensor, audio_tensor, prepare_points
from rtabmap_features.point_ops import fps, TorchKNN
from rtabmap_features.bundle import read_points, load_bundle, save_features


def vector(index):
    x = np.zeros(1024, np.float32)
    x[index] = 2
    return x


class FusionTests(unittest.TestCase):
    def test_fixed_slots_and_unit_norm_weighted_output(self):
        result = fuse_features({name: vector(i) for i, name in enumerate(MODALITIES)})
        np.testing.assert_array_equal(result['concat'].reshape(4, 1024), result['independent'])
        np.testing.assert_array_equal(result['encoder_features'][0], vector(0))
        np.testing.assert_allclose(result['fused'][:4], [0.5] * 4)
        np.testing.assert_array_equal(result['valid_mask'], [True] * 4)
        self.assertTrue(result['complete'])

    def test_partial_keeps_missing_slots_and_renormalizes_weights(self):
        result = fuse_features({'rgb': vector(0), 'lidar': vector(1)},
                               {'rgb': 1, 'lidar': 3}, allow_partial=True)
        np.testing.assert_array_equal(result['valid_mask'], [True, False, True, False])
        np.testing.assert_allclose(result['weights'], [0.25, 0, 0.75, 0])
        self.assertFalse(result['complete'])
        self.assertTrue(np.isnan(result['cosine_similarities'][1]).all())
        self.assertEqual(np.count_nonzero(result['independent'][1]), 0)

    def test_missing_modalities_are_rejected_by_default(self):
        with self.assertRaisesRegex(ValueError, 'Missing required'):
            fuse_features({'rgb': vector(0)})

    def test_invalid_features_rejected(self):
        for bad in (np.zeros(1024), np.full(1024, np.nan), np.ones(1023)):
            with self.subTest(shape=bad.shape), self.assertRaises(ValueError):
                fuse_features({'rgb': bad}, allow_partial=True)

    def test_bad_weights_rejected(self):
        for weights in ({'rgb': -1}, {'rgb': float('nan')}, {'other': 1},
                        dict.fromkeys(MODALITIES, 0)):
            with self.subTest(weights=weights), self.assertRaises(ValueError):
                fuse_features({'rgb': vector(0)}, weights, allow_partial=True)

    def test_cancelled_mean_is_marked_invalid(self):
        result = fuse_features({'rgb': vector(0), 'infrared': -vector(0)}, allow_partial=True)
        self.assertFalse(result['fused_valid'])
        self.assertTrue(np.isfinite(result['fused']).all())

    def test_audio_interval_can_cover_anchor(self):
        anchor = Sample(None, 100, 'rgb_frame', 'rgb-1')
        buffers = {'infrared': [Sample(None, 100.01, 'ir_frame', 'ir-1')],
                   'lidar': [Sample(None, 99.98, 'lidar_frame', 'cloud-1')],
                   'audio': [Sample(None, 98.1, 'array_frame', 'audio-1', duration=2)]}
        matched = match_samples(buffers, anchor)
        self.assertEqual(matched['audio'].gap(100), 0)
        self.assertEqual(matched['infrared'].frame_id, 'ir_frame')

    def test_stale_audio_and_stale_images_never_fill_missing_slots(self):
        anchor = Sample(None, 100, '', 'rgb-1')
        buffers = {'infrared': [Sample(None, 90, '', 'old-ir')],
                   'lidar': [Sample(None, 100, '', 'cloud')],
                   'audio': [Sample(None, 90, '', 'old-audio', duration=2)]}
        self.assertIsNone(match_samples(buffers, anchor))
        matched = match_samples(buffers, anchor, allow_partial=True)
        self.assertEqual(set(matched), {'rgb', 'lidar'})

    def test_non_audio_duration_does_not_make_stale_images_match(self):
        anchor = Sample(None, 100, '', 'rgb')
        buffers = {'infrared': [Sample(None, 90, '', 'old-ir', duration=20)]}
        matched = match_samples(buffers, anchor, allow_partial=True)
        self.assertNotIn('infrared', matched)

    def test_invalid_sample_metadata_rejected(self):
        for kwargs in (dict(stamp=float('nan')), dict(stamp=-1), dict(duration=-1), dict(source_id='')):
            args = dict(payload=None, stamp=1, frame_id='', source_id='x')
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Sample(**args)


class PreprocessingTests(unittest.TestCase):
    def test_nir_is_explicitly_a_repeated_visual_input(self):
        gray = np.arange(64 * 64, dtype=np.uint8).reshape(64, 64)
        np.testing.assert_array_equal(image_tensor(gray, infrared=True).numpy(),
                                      image_tensor(np.repeat(gray[:, :, None], 3, axis=2)).numpy())
        self.assertEqual(tuple(image_tensor(gray, infrared=True, thermal=True).shape), (1, 224, 224))

    def test_invalid_image_dtype_and_channels_rejected(self):
        with self.assertRaises(ValueError):
            image_tensor(np.ones((64, 64), np.uint16), infrared=True)
        with self.assertRaises(ValueError):
            image_tensor(np.ones((64, 64, 3), np.uint8), infrared=True)

    def test_known_fps_and_nearest_neighbors(self):
        import torch
        points = torch.tensor([[[0., 0, 0], [1., 0, 0], [2., 0, 0], [3., 0, 0]]])
        np.testing.assert_array_equal(fps(points, 3).numpy()[0, :, 0], [0, 3, 1])
        distances, indices = TorchKNN(2)(points, torch.tensor([[[1.2, 0, 0]]]))
        np.testing.assert_array_equal(indices.numpy(), [[[1, 2]]])
        np.testing.assert_allclose(distances.numpy(), [[[0.2, 0.8]]], atol=1e-6)

    def test_point_normalization_preserves_origin_and_scale_metadata(self):
        points = np.random.default_rng(12).uniform(-1, 1, (600, 3)).astype(np.float32) + 5
        tensor, metadata = prepare_points(points, count=512)
        self.assertEqual(tuple(tensor.shape), (1, 512, 3))
        restored = tensor.numpy()[0] * metadata['radius_m'] + metadata['origin_xyz_m']
        for p in restored:
            self.assertLess(np.linalg.norm(points - p, axis=1).min(), 2e-6)
        self.assertLessEqual(np.linalg.norm(tensor.numpy(), axis=2).max(), 1.000001)

    def test_insufficient_lidar_points_rejected(self):
        with self.assertRaisesRegex(ValueError, 'at least 512'):
            prepare_points(np.ones((1000, 3), np.float32))

    def test_audio_resampling_and_log_mel_contract(self):
        t = np.arange(16000) / 8000
        samples = (0.1 * np.sin(2 * np.pi * 440 * t)).astype(np.float32)
        tensor, metadata = audio_tensor(samples, 8000, anchor_offset=1)
        self.assertEqual(tuple(tensor.shape), (1, 1, 128, 204))
        self.assertTrue(np.isfinite(tensor.numpy()).all())
        self.assertEqual(metadata['sample_rate_model'], 16000)

    def test_audio_out_of_range_and_nonfinite_rejected(self):
        for bad in (np.full(1600, 2), np.full(1600, np.nan), np.zeros(1)):
            with self.subTest(shape=bad.shape), self.assertRaises(ValueError):
                audio_tensor(bad, 16000)


class ArtifactTests(unittest.TestCase):
    def test_binary_and_ascii_pcd_extra_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            header = 'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA '
            rows = np.array([[1, 2, 3, 4], [5, 6, 7, 8]], dtype='<f4')
            (base / 'binary.pcd').write_bytes((header + 'binary\n').encode() + rows.tobytes())
            (base / 'ascii.pcd').write_text(header + 'ascii\n1 2 3 4\n5 6 7 8\n')
            for file in base.glob('*.pcd'):
                np.testing.assert_array_equal(read_points(file), rows[:, :3])

    def test_unpaired_bundle_requires_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp) / 'manifest.json'
            file.write_text(json.dumps(dict(format_version=1, calibration_version='unknown',
                                             alignment='unpaired_smoke_test', samples={})))
            with self.assertRaisesRegex(ValueError, '--allow-unaligned'):
                load_bundle(file)

    def test_feature_artifacts_keep_layout_and_refuse_overwrite(self):
        result = fuse_features(dict(zip(MODALITIES, [vector(i) for i in range(4)])))
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'result'
            save_features(path, result, dict(format_version=1))
            with np.load(path / 'features.npz', allow_pickle=False) as saved:
                np.testing.assert_array_equal(saved['concat'], result['concat'])
            with self.assertRaises(FileExistsError):
                save_features(path, result, {})


if __name__ == '__main__':
    unittest.main()
