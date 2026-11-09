import unittest

import numpy as np

from rtabmap_semantic.emsanet_detector import (
    prepare_rgbd, panoptic_detections, EmsanetDetector, emsanet_chinese_label,
)
from rtabmap_semantic.node import store_config


class EmsanetInputTests(unittest.TestCase):
    def test_ros_metres_and_millimetres_are_equivalent_and_inputs_are_not_mutated(self):
        bgr = np.full((2, 3, 3), [11, 22, 33], dtype=np.uint8)
        mm = np.array([[0, 1200, 2600], [900, 10000, 65535]], np.uint16)
        metres = mm.astype(np.float32) / 1000
        original = metres.copy()
        rgb, converted = prepare_rgbd(bgr, metres)
        np.testing.assert_array_equal(rgb[0, 0], [33, 22, 11])
        np.testing.assert_array_equal(converted, prepare_rgbd(bgr, mm)[1])
        np.testing.assert_array_equal(metres, original)
        np.testing.assert_array_equal(bgr[0, 0], [11, 22, 33])
        self.assertEqual(converted[-1, -1], 0)
        self.assertEqual(converted.dtype, np.float32)

    def test_invalid_depth_is_zero_and_registered_scaling_does_not_interpolate_holes(self):
        bgr = np.zeros((4, 6, 3), np.uint8)
        depth = np.array([[np.nan, np.inf, -1], [2, 0, 4]], np.float32)
        rgb, mm = prepare_rgbd(bgr, depth)
        self.assertEqual(rgb.shape[:2], mm.shape)
        self.assertEqual(set(np.unique(mm)), {0, 2000, 4000})
        self.assertTrue(np.isnan(depth[0, 0]))

    def test_unregistered_shapes_and_ambiguous_encodings_are_rejected(self):
        rgb = np.zeros((4, 6, 3), np.uint8)
        for depth in (None, np.zeros((4, 4), np.uint16), np.zeros((4, 6), np.float64),
                      np.zeros((4, 6, 3), np.uint16)):
            with self.subTest(depth=None if depth is None else (depth.shape, depth.dtype)):
                with self.assertRaises(ValueError):
                    prepare_rgbd(rgb, depth)

    def test_empty_depth_does_not_produce_fabricated_rgb_only_results(self):
        detector = EmsanetDetector.__new__(EmsanetDetector)
        self.assertEqual(detector.detect(np.zeros((4, 6, 3), np.uint8),
                                         np.zeros((4, 6), np.uint16)), [])


class EmsanetOutputTests(unittest.TestCase):
    def test_nyuv2_display_names_keep_existing_yolo_translation_policy(self):
        from rtabmap_semantic.labels import chinese_label
        self.assertEqual(emsanet_chinese_label('refrigerator'), '冰箱')
        self.assertEqual(emsanet_chinese_label('night stand'), '床头柜')
        self.assertEqual(emsanet_chinese_label('chair'), chinese_label('chair'))
        self.assertEqual(chinese_label('night stand'), 'night stand')

    def test_same_category_instances_remain_separate_and_stuff_is_excluded(self):
        labels = [('void', False), ('wall', False), ('chair', True)]
        panoptic = np.array([[2001, 2001, 2002, 2002], [1000, 1001, 2004, 0]])
        instance_ids = {2001: 1, 2002: 2, 1001: 3, 2004: 4}
        metadata = {1: dict(semantic_idx=2, panoptic_score=0.8),
                    2: dict(semantic_idx=2, panoptic_score=0.7),
                    3: dict(semantic_idx=1, panoptic_score=0.99),
                    4: dict(semantic_idx=2, panoptic_score=0.2)}
        results = panoptic_detections(panoptic, instance_ids, metadata, labels, 0.35, 1000)
        self.assertEqual([r['label'] for r in results], ['chair', 'chair'])
        self.assertEqual([r['confidence'] for r in results], [0.8, 0.7])
        self.assertEqual([r['mask'].sum() for r in results], [2, 2])
        self.assertFalse(np.any(results[0]['mask'] & results[1]['mask']))
        self.assertFalse(np.any(results[0]['mask'][1]))

    def test_missing_metadata_unknown_classes_and_nonfinite_confidence_are_ignored(self):
        panoptic = np.array([[1001, 1002, 99003]])
        instance_ids = {1001: 1, 1002: 2, 99003: 3}
        metadata = {2: dict(semantic_idx=1, panoptic_score=float('nan')),
                    3: dict(semantic_idx=99, panoptic_score=0.9)}
        self.assertEqual(panoptic_detections(panoptic, instance_ids, metadata,
                                            [('void', False), ('chair', True)], 0.35, 1000), [])

    def test_cpu_shifted_class_metadata_does_not_turn_sofa_into_chair(self):
        labels = [(name, name not in ('void', 'wall', 'floor')) for name in
                  ('void', 'wall', 'floor', 'cabinet', 'bed', 'chair', 'sofa')]
        encoded_sofa = 6 * 65536 + 1
        panoptic = np.array([[encoded_sofa, encoded_sofa, 65536]])
        # Upstream CPU score computation shifts semantic_idx 6 -> 5 in-place.
        metadata = {7: dict(semantic_idx=5, panoptic_score=0.8)}
        result = panoptic_detections(panoptic, {encoded_sofa: 7}, metadata, labels, 0.35, 65536)
        self.assertEqual(result[0]['label'], 'sofa')
        np.testing.assert_array_equal(result[0]['mask'], [[1, 1, 0]])

    def test_cache_identity_is_separate_from_both_yolo_backends(self):
        config = dict(model_path='/models/model.pth', detector_backend='emsanet')
        ems = store_config(config, 'hash')
        self.assertIn('emsanet_policy_version', ems)
        self.assertNotIn('label_policy_version', ems)
        self.assertNotEqual(ems, store_config({**config, 'detector_backend': 'yolov8'}, 'hash'))
        self.assertNotEqual(ems, store_config({**config, 'detector_backend': 'yoloe_pf'}, 'hash'))


if __name__ == '__main__':
    unittest.main()
