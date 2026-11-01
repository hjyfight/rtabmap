"""Verify projection through optimized anchors, missing data and snapshot deletion."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from rtabmap_semantic_msgs.msg import Observation, SemanticMap, SemanticObject
from visualization_msgs.msg import Marker

spec = importlib.util.spec_from_file_location(
    'miti_scene_adapter', Path(__file__).resolve().parents[1] / 'scripts/miti_scene_adapter.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class SceneAdapterTest(unittest.TestCase):
    def test_optimized_anchors_and_invalid_observations(self):
        pose = Pose()
        pose.position.x = 10.0
        pose.orientation.z = pose.orientation.w = 2 ** -0.5
        anchor = Observation(node_id=1, weight=2.0)
        anchor.position.x = 1.0
        missing = Observation(node_id=9, weight=1.0)
        invalid = Observation(node_id=1, weight=float('nan'))
        obj = SemanticObject(observations=[anchor, missing, invalid])
        np.testing.assert_allclose(adapter.object_position(obj, {1: adapter.pose_matrix(pose)}), [10, 1, 0], atol=1e-12)
        pose.position.x = 8.0  # A loop closure changes the anchor's optimized map pose.
        np.testing.assert_allclose(adapter.object_position(obj, {1: adapter.pose_matrix(pose)}), [8, 1, 0], atol=1e-12)
        self.assertIsNone(adapter.object_position(obj, {}))

    def test_snapshot_removes_disappearing_labels_and_preserves_large_ids(self):
        anchor = Observation(node_id=1, weight=1.0)
        obj = SemanticObject(object_id=2**40, label='椅子', observations=[anchor])
        result = adapter.markers_for(SemanticMap(objects=[obj]), {1: np.eye(4)}, Header(frame_id='map'))
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)
        self.assertEqual(len(result.markers), 3)
        label = result.markers[-1]
        self.assertEqual(label.text, '椅子')
        self.assertEqual(label.header.frame_id, 'map')
        self.assertIn(str(2**40), label.ns)
        cleared = adapter.markers_for(SemanticMap(), {}, Header(frame_id='map'))
        self.assertEqual([marker.action for marker in cleared.markers], [Marker.DELETEALL])

    def test_invalid_pose_is_rejected(self):
        pose = Pose()
        pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            adapter.pose_matrix(pose)


if __name__ == '__main__':
    unittest.main()
