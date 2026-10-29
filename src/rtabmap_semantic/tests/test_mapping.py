import tempfile
import unittest
from pathlib import Path

import numpy as np

from rtabmap_semantic.geometry import lift_mask, rigid_matrix
from rtabmap_semantic.fusion import SemanticState, SemanticStore


def detection(x=0.0, label='chair'):
    return dict(label=label, position=[x, 0, 2], extent=[0.4, 0.4, 0.4],
                sigma=0.02, confidence=0.9)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.k = [100, 0, 49.5, 0, 100, 49.5, 0, 0, 1]
        self.mask = np.zeros((100, 100), np.uint8)
        self.mask[35:65, 35:65] = 1

    def lift(self, depth, **kwargs):
        return lift_mask(self.mask, depth, self.k, (100, 100), np.eye(4), **kwargs)

    def test_millimetres_equal_float_metres(self):
        a = self.lift(np.full((100, 100), 2000, np.uint16))
        b = self.lift(np.full((100, 100), 2, np.float32))
        np.testing.assert_allclose(a['position'], [0, 0, 2], atol=1e-6)
        np.testing.assert_allclose(a['position'], b['position'])

    def test_depth_decimation_scales_intrinsics(self):
        result = self.lift(np.full((50, 50), 2, np.float32))
        np.testing.assert_allclose(result['position'], [0.01, 0.01, 2], atol=0.021)

    def test_invalid_depth_never_produces_label(self):
        self.assertIsNone(self.lift(np.zeros((100, 100), np.uint16)))
        self.assertIsNone(self.lift(np.full((100, 100), np.nan, np.float32)))
        self.assertIsNone(self.lift(np.full((100, 100), 9, np.float32)))
        with self.assertRaises(ValueError):
            self.lift(np.ones((100, 100), np.uint8))

    def test_foreground_cluster_rejects_background(self):
        depth = np.full((100, 100), 2, np.float32)
        depth[35:40, 35:65] = 3.5
        result = self.lift(depth)
        self.assertAlmostEqual(result['position'][2], 2)

    def test_optical_to_node_rotation_and_translation(self):
        transform = rigid_matrix([0.2, 0.1, 0.3], [0, np.sin(np.pi/4), 0, np.cos(np.pi/4)])
        result = lift_mask(self.mask, np.full((100, 100), 2, np.float32),
                           self.k, (100, 100), transform)
        np.testing.assert_allclose(result['position'], [2.2, 0.1, 0.3], atol=1e-6)

    def test_bad_calibration_is_rejected(self):
        with self.assertRaises(ValueError):
            rigid_matrix([0, 0, 0], [0, 0, 0, 0])
        with self.assertRaises(ValueError):
            lift_mask(self.mask, np.ones((50, 100), np.float32),
                      self.k, (100, 100), np.eye(4))


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.state = SemanticState()
        self.state.update_graph({i: np.eye(4) for i in range(1, 10)})

    def test_repeated_node_does_not_confirm_object(self):
        for _ in range(5):
            self.state.add_frame(1, 1.0, [detection()])
        self.assertEqual(len(self.state.objects), 1)
        self.assertEqual(len(self.state.visible_objects()), 0)
        self.state.add_frame(2, 2.0, [detection(0.02)])
        self.state.add_frame(3, 3.0, [detection(-0.02)])
        self.assertEqual(len(self.state.visible_objects()), 1)

    def test_adjacent_same_class_objects_stay_distinct(self):
        for i in range(1, 4):
            self.state.add_frame(i, float(i), [detection(0), detection(0.35)])
        self.assertEqual(len(self.state.visible_objects()), 2)
        self.assertEqual([len(o.observations) for o in self.state.objects], [3, 3])

    def test_loop_closure_reprojects_each_anchor(self):
        for i in range(1, 4):
            self.state.add_frame(i, float(i), [detection()])
        poses = {i: np.eye(4) for i in range(1, 4)}
        poses[1][0, 3], poses[2][0, 3], poses[3][0, 3] = 0.1, 0.2, 0.3
        self.state.update_graph(poses)
        np.testing.assert_allclose(self.state.position(self.state.objects[0]), [0.2, 0, 2])

    def test_dynamic_target_expires_and_is_not_saved(self):
        self.state.add_frame(1, 1.0, [detection(label='person')], now=10)
        self.assertEqual(len(self.state.visible_objects(now=11)), 1)
        self.assertEqual(self.state.to_dict()['objects'], [])
        self.assertEqual(len(self.state.visible_objects(now=14)), 0)

    def test_outlier_anchors_are_excluded_from_display(self):
        for i in range(1, 4):
            self.state.add_frame(i, float(i), [detection()])
        self.state.poses[3][0, 3] = 3
        obj = self.state.objects[0]
        self.assertEqual([o.node_id for o in self.state.inlier_observations(obj)], [1, 2])
        np.testing.assert_allclose(self.state.position(obj), [0, 0, 2])

    def test_loop_closure_merges_duplicate_tracks(self):
        for i in range(1, 4):
            self.state.add_frame(i, float(i), [detection(0)])
        for i in range(4, 7):
            self.state.add_frame(i, float(i), [detection(1)])
        self.assertEqual(len(self.state.objects), 2)
        corrected = {i: np.eye(4) for i in range(1, 7)}
        for i in range(4, 7):
            corrected[i][0, 3] = -1
        self.state.update_graph(corrected)
        self.assertTrue(self.state.reconcile())
        self.assertEqual(len(self.state.objects), 1)
        self.assertEqual(self.state.objects[0].object_id, 1)

    def test_coobserved_objects_never_merge_after_loop(self):
        for i in range(1, 4):
            self.state.add_frame(i, float(i), [detection(0), detection(0.1)])
        self.assertFalse(self.state.reconcile())
        self.assertEqual(len(self.state.objects), 2)

    def test_id_reuse_creates_new_epoch(self):
        self.state.add_frame(1, 1.0, [detection()])
        old_uuid = self.state.map_uuid
        result = self.state.add_frame(1, 100.0, [detection(3)])
        self.assertEqual(result, 'reset')
        self.assertNotEqual(old_uuid, self.state.map_uuid)
        self.assertEqual(len(self.state.objects), 1)

    def test_saved_objects_need_current_graph_after_reload(self):
        for i in range(1, 4):
            self.state.add_frame(i, float(i), [detection()])
        with tempfile.TemporaryDirectory() as directory:
            store = SemanticStore(Path(directory)/'map.semantic.sqlite', Path(directory)/'map.db', {})
            store.save(self.state)
            restored = SemanticState()
            store.load(restored)
            self.assertEqual(restored.visible_objects(), [])
            restored.update_graph(self.state.poses)
            self.assertEqual(restored.visible_objects()[0].object_id, 1)
            store.close()

    def test_map_and_model_mismatch_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'map.semantic.sqlite'
            store = SemanticStore(path, '/tmp/a.db', {'model': 'a'})
            store.save(self.state)
            store.close()
            for key, config in [('/tmp/b.db', {'model': 'a'}), ('/tmp/a.db', {'model': 'b'})]:
                store = SemanticStore(path, key, config)
                with self.assertRaises(ValueError):
                    store.load(SemanticState())
                store.close()


if __name__ == '__main__':
    unittest.main()
