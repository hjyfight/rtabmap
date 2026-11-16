import json
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rtabmap_features'))
from rtabmap_features.fusion import fuse_features
from rtabmap_features.fusion import Sample, match_samples, acquisition_gap
from rtabmap_features.visualization import feature_snapshot, spectrum_rgb


class VisualizationTests(unittest.TestCase):
    def test_visualized_stages_retain_exact_model_values_and_weighted_arithmetic(self):
        names = ('rgb', 'infrared', 'lidar', 'audio')
        raw = {name: np.eye(4, 1024, dtype=np.float32)[i] * (i + 2)
               for i, name in enumerate(names)}
        output = fuse_features(raw, weights=dict(zip(names, [1, 2, 3, 4])))
        metadata = {'samples': {'rgb': {'attributes': {'topic': '/camera/color/image_raw'}}},
                    'preprocessing': {'rgb': {'input_shape': [480, 640, 3]}}}
        view = feature_snapshot(output, metadata, mode='live')
        self.assertEqual(view['flow_version'], 1)
        for i, row in enumerate(view['modalities']):
            np.testing.assert_array_equal(row['encoder_features'], raw[names[i]])
            self.assertAlmostEqual(row['encoder_norm'], i + 2)
            self.assertAlmostEqual(row['normalized_norm'], 1)
        np.testing.assert_array_equal(view['joint_features'], output['concat'])
        mean = np.asarray(view['weighted_mean_features'])
        np.testing.assert_allclose(mean[:4], [.1, .2, .3, .4])
        np.testing.assert_allclose(view['fused_features'], mean / np.linalg.norm(mean), atol=1e-7)
        self.assertEqual(view['modalities'][0]['topic'], '/camera/color/image_raw')
        self.assertEqual(view['modalities'][0]['preprocessing']['input_shape'], [480, 640, 3])
        self.assertLess(len(json.dumps(view)), 650000)
        for key, shape in [('encoder_features', (3, 1024)), ('concat', (4095,))]:
            with self.assertRaises(ValueError):
                feature_snapshot({**output, key: np.zeros(shape)}, {}, mode='live')

    def test_clock_offset_matching_preserves_original_source_stamps(self):
        rgb = Sample(None, 100, 'rgb', 'rgb', clock_offset_s=57.5)
        ir = Sample(None, 100.01, 'ir', 'ir', clock_offset_s=57.5)
        lidar = Sample(None, 99.99, 'lidar', 'lidar', clock_offset_s=57.5)
        audio = Sample(None, 157, 'audio', 'audio', duration=2)
        selected = match_samples({'infrared': [ir], 'lidar': [lidar], 'audio': [audio]}, rgb)
        self.assertIsNotNone(selected)
        self.assertEqual(rgb.stamp, 100)
        self.assertEqual(rgb.metadata()['alignment_stamp'], 157.5)
        self.assertEqual(acquisition_gap('audio', audio, rgb.alignment_stamp), 0)
        self.assertAlmostEqual(acquisition_gap('infrared', ir, rgb.alignment_stamp), .01)
        self.assertIsNone(match_samples({'infrared': [ir], 'lidar': [lidar], 'audio': [audio]},
                                       Sample(None, 100, 'rgb', 'unaligned')))

    def test_missing_modalities_are_json_null_in_cosine_matrix(self):
        output = fuse_features({'rgb': np.ones(1024, np.float32)}, allow_partial=True)
        snapshot = feature_snapshot(output, {}, mode='live')
        encoded = json.dumps(snapshot, allow_nan=False)
        self.assertEqual(json.loads(encoded)['cosine_similarities'][0][1], None)
        self.assertEqual(snapshot['modalities'][1]['weight'], 0)
        self.assertFalse(snapshot['complete'])

    def test_saved_alignment_is_preserved_and_dimensions_are_not_reduced(self):
        output = fuse_features({'rgb': np.ones(1024, np.float32)}, allow_partial=True)
        snapshot = feature_snapshot(output, {'context': {'alignment': 'unpaired_smoke_test'}}, mode='saved')
        self.assertEqual(snapshot['mode'], 'saved')
        self.assertEqual(snapshot['alignment'], 'unpaired_smoke_test')
        self.assertEqual(len(snapshot['fused_features']), 1024)
        with self.assertRaises(ValueError):
            feature_snapshot({**output, 'fused': np.zeros(5)}, {}, mode='saved')

    def test_spectrum_has_fixed_scale_and_reversed_frequency_axis(self):
        values = np.zeros((128, 204), np.float32)
        values[0] = 2; values[-1] = -2
        rgb = spectrum_rgb(values)
        self.assertEqual(rgb.shape, (128, 204, 3))
        self.assertEqual(rgb.dtype, np.uint8)
        np.testing.assert_array_equal(rgb[0, 0], [10, 20, 38])
        np.testing.assert_array_equal(rgb[-1, 0], [250, 194, 96])
        values[2, 4] = np.nan
        with self.assertRaises(ValueError):
            spectrum_rgb(values)


if __name__ == '__main__':
    unittest.main()
