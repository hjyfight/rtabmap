import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from rtabmap_semantic.detector import YoloDetector
from rtabmap_semantic.node import store_config


class ArrayResult:
    def __init__(self, values):
        self.values = np.asarray(values)

    def cpu(self):
        return self

    def numpy(self):
        return self.values


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.weights = Path(self.directory.name) / 'model.pt'
        self.weights.write_bytes(b'fixture')

    def test_prompt_free_adapter_keeps_labels_and_masks_without_any_prompts(self):
        calls = []
        mask = np.array([[1., 0., 0.], [1., 1., 0.]], dtype=np.float32)
        result = SimpleNamespace(masks=SimpleNamespace(data=ArrayResult([mask])),
            boxes=SimpleNamespace(cls=ArrayResult([123]), conf=ArrayResult([0.8])),
            names={123: 'fire extinguisher'})
        model = SimpleNamespace(task='segment', model=SimpleNamespace(model=[SimpleNamespace(lrpc=object())]),
            predict=lambda image, **kwargs: calls.append(kwargs) or [result])
        with patch.dict('sys.modules', {'torch': SimpleNamespace(set_num_threads=lambda value: None),
            'ultralytics': SimpleNamespace(YOLOE=lambda path: model)}):
            detector = YoloDetector(self.weights, backend='yoloe_pf')
            detections = detector.detect(np.zeros((2, 3, 3), np.uint8))
        self.assertEqual(detections[0]['label'], 'fire extinguisher')
        np.testing.assert_array_equal(detections[0]['mask'], mask)
        self.assertEqual(detections[0]['mask'].dtype, np.uint8)
        self.assertNotIn('classes', calls[0])
        self.assertNotIn('visual_prompts', calls[0])
        self.assertTrue(calls[0]['retina_masks'])
        self.assertTrue(calls[0]['agnostic_nms'])
        self.assertEqual(calls[0]['iou'], 0.5)

    def test_text_prompt_checkpoint_is_rejected_for_prompt_free_backend(self):
        model = SimpleNamespace(task='segment', model=SimpleNamespace(model=[SimpleNamespace()]))
        with patch.dict('sys.modules', {'torch': SimpleNamespace(set_num_threads=lambda value: None),
            'ultralytics': SimpleNamespace(YOLOE=lambda path: model)}):
            with self.assertRaisesRegex(ValueError, 'prompt-free'):
                YoloDetector(self.weights, backend='yoloe_pf')

    def test_default_backend_still_loads_old_yolo_and_handles_empty_detections(self):
        calls = []
        model = SimpleNamespace(task='segment', predict=lambda image, **kwargs:
                                [SimpleNamespace(masks=None, boxes=None)])
        def load(path, **kwargs):
            calls.append(kwargs)
            return model
        with patch.dict('sys.modules', {'torch': SimpleNamespace(set_num_threads=lambda value: None),
            'ultralytics': SimpleNamespace(YOLO=load)}):
            detector = YoloDetector(self.weights)
            self.assertEqual(detector.detect(np.zeros((2, 3, 3), np.uint8)), [])
        self.assertEqual(calls, [{'task': 'segment'}])
        self.assertEqual(detector.backend, 'yolov8')

    def test_existing_v8_database_fingerprint_is_preserved_and_yoloe_is_distinct(self):
        original = {'model_path': '/models/v8.pt', 'classes': ['*'], 'confidence': 0.5}
        new = {**original, 'detector_backend': 'yolov8', 'device': 'cpu', 'cpu_threads': 2,
               'database_path': '/maps/map.db', 'semantic_database_path': '',
               'confirmation_window': 0, 'dynamic_min_observations': 1, 'label_language': 'zh'}
        self.assertEqual(store_config(new, 'model-hash'), {**original, 'model_sha256': 'model-hash'})
        self.assertEqual(store_config({**new, 'detector_backend': 'yoloe_pf'}, 'other')['detector_backend'], 'yoloe_pf')


if __name__ == '__main__':
    unittest.main()
