import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rtabmap_features'))
from rtabmap_features.yamnet import (CLASS_COUNT, MODEL_SHA256, PatrolEventTracker,
                                    YamnetClassifier, load_class_map, prepare_waveform)


class AudioInputTests(unittest.TestCase):
    def test_stereo_is_mixed_and_resampled_without_automatic_gain(self):
        phase = np.arange(44100, dtype=np.float32) / 44100
        signal = .1 * np.sin(2 * np.pi * 440 * phase)
        waveform, info = prepare_waveform(np.stack([signal, signal], axis=1), 44100)
        self.assertEqual(waveform.shape, (16000,))
        self.assertEqual(waveform.dtype, np.float32)
        self.assertEqual(info['channels'], 2)
        self.assertAlmostEqual(info['rms_dbfs'], -23.01, places=1)
        self.assertLess(float(np.max(np.abs(waveform))), .11)

    def test_malformed_pcm_and_formats_are_rejected(self):
        cases = [(np.array([]), 16000), (np.array([np.nan]), 16000),
                 (np.array([np.inf]), 16000), (np.array([1.1]), 16000),
                 (np.zeros((1, 65)), 16000), (np.zeros((1, 1, 1)), 16000),
                 (np.zeros(1), 0), (np.zeros(1), 16000.0), (np.zeros(160001), 16000)]
        for samples, rate in cases:
            with self.subTest(shape=samples.shape, rate=rate), self.assertRaises(ValueError):
                prepare_waveform(samples, rate)

    def test_official_map_preserves_all_classes_and_indices(self):
        classes = load_class_map(ROOT / 'config/yamnet_class_map.csv')
        self.assertEqual(len(classes), CLASS_COUNT)
        self.assertEqual(classes[0]['label'], 'Speech')
        self.assertEqual(classes[494]['label'], 'Silence')
        self.assertEqual(classes[48]['label_zh'], '行走／脚步')

    def test_reordered_class_map_is_rejected(self):
        rows = (ROOT / 'config/yamnet_class_map.csv').read_text().splitlines()
        rows[1], rows[2] = rows[2], rows[1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'classes.csv'
            path.write_text('\n'.join(rows))
            with self.assertRaises(ValueError):
                load_class_map(path)


class PatrolTests(unittest.TestCase):
    def setUp(self):
        self.classes = load_class_map(ROOT / 'config/yamnet_class_map.csv')
        self.settings = json.loads((ROOT / 'config/yamnet_patrol.json').read_text())
        self.tracker = PatrolEventTracker(self.settings, self.classes)
        self.scores = np.zeros(CLASS_COUNT)
        self.scores[0] = .8

    def test_consecutive_windows_confirm_once_and_keep_event_identity(self):
        first = self.tracker.update(self.scores, 100, -30)[0]
        self.assertEqual(first['status'], 'candidate')
        self.assertIsNone(first['event_id'])
        second = self.tracker.update(self.scores, 100.5, -30)[0]
        third = self.tracker.update(self.scores, 101, -30)[0]
        self.assertEqual(second['status'], 'confirmed')
        self.assertTrue(second['onset'])
        self.assertEqual(second['event_id'], third['event_id'])
        self.assertFalse(third['onset'])

    def test_silent_audio_cannot_confirm_a_model_candidate(self):
        self.assertEqual(self.tracker.update(self.scores, 1, -90), [])
        first = self.tracker.update(self.scores, 1.5, -30)[0]
        self.assertEqual(first['consecutive_windows'], 1)

    def test_missing_sound_and_long_gap_start_new_observations(self):
        self.tracker.update(self.scores, 1, -30)
        self.tracker.update(self.scores, 1.5, -30)
        self.tracker.update(np.zeros(CLASS_COUNT), 2, -30)
        first = self.tracker.update(self.scores, 2.5, -30)[0]
        self.assertEqual(first['status'], 'candidate')
        second = self.tracker.update(self.scores, 6, -30)[0]
        self.assertEqual(second['status'], 'candidate')
        self.assertEqual(second['consecutive_windows'], 1)

    def test_multiple_sounds_keep_separate_scores(self):
        self.scores[390] = .7
        rows = self.tracker.update(self.scores, 1, -30)
        self.assertEqual({row['key'] for row in rows}, {'speech', 'alarm'})
        self.assertEqual([row['score'] for row in rows], [.8, .7])

    def test_duplicate_timestamps_and_invalid_scores_do_not_advance_confirmation(self):
        self.tracker.update(self.scores, 1, -30)
        for scores, stamp in [(self.scores, 1), (self.scores, .5), (np.zeros(520), 2),
                               (np.full(521, np.nan), 2), (np.full(521, 2), 2)]:
            with self.assertRaises(ValueError):
                self.tracker.update(scores, stamp, -30)
        self.assertEqual(self.tracker.update(self.scores, 1.5, -30)[0]['consecutive_windows'], 2)

    def test_unknown_sound_rules_are_rejected(self):
        settings = copy.deepcopy(self.settings)
        settings['rules'][0]['classes'] = ['a class that does not exist']
        with self.assertRaises(ValueError):
            PatrolEventTracker(settings, self.classes)


@unittest.skipUnless(importlib.util.find_spec('tflite_runtime') and
                     (ROOT / 'models/yamnet/yamnet.tflite').is_file(), 'Install the official YAMNet runtime first')
class ActualModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = YamnetClassifier(ROOT / 'models/yamnet/yamnet.tflite', ROOT / 'config/yamnet_class_map.csv')

    def test_actual_model_has_521_outputs_and_recognizes_digital_silence(self):
        result = self.model.classify(np.zeros(32000, dtype=np.float32), 16000)
        self.assertEqual(len(result['scores']), 521)
        self.assertEqual(result['model']['sha256'], MODEL_SHA256)
        self.assertEqual(result['top_k'][0]['label'], 'Silence')
        self.assertEqual(result['audio']['patches'], 3)

    def test_short_resampled_input_is_padded_and_returns_finite_scores(self):
        result = self.model.classify(np.zeros((8000, 2), dtype=np.float32), 32000)
        self.assertEqual(result['audio']['channels'], 2)
        self.assertEqual(result['audio']['padded_samples'], 11600)
        self.assertTrue(np.isfinite(result['scores']).all())

    def test_modified_model_is_rejected_before_inference(self):
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / 'yamnet.tflite'
            model.write_bytes(b'not the official model')
            with self.assertRaisesRegex(ValueError, 'checksum'):
                YamnetClassifier(model, ROOT / 'config/yamnet_class_map.csv')


if __name__ == '__main__':
    unittest.main()
