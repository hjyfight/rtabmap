"""Verify projection through optimized anchors, missing data and snapshot deletion."""
import importlib.util
from copy import deepcopy
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace
import unittest

import numpy as np
from geometry_msgs.msg import Pose
from rtabmap_msgs.msg import MapData, MapGraph, RGBDImage
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from rtabmap_semantic_msgs.msg import Observation, SemanticMap, SemanticObject
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
spec = importlib.util.spec_from_file_location(
    'miti_scene_adapter', Path(__file__).resolve().parents[1] / 'scripts/miti_scene_adapter.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class SceneAdapterTest(unittest.TestCase):
    def make_scene(self):
        self.frames, self.paths = [], []
        node = SimpleNamespace(poses={}, graph=None, objects=None, session=None,
            markers=adapter.SemanticMarkers(), cache_epoch=0,
            path_pub=SimpleNamespace(publish=self.paths.append),
            marker_pub=SimpleNamespace(publish=self.frames.append),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))))
        for name in ('on_graph', 'on_objects', 'clear_scene', 'publish_markers', 'restore_graph'):
            setattr(node, name, MethodType(getattr(adapter.MitiSceneAdapter, name), node))
        return node

    def graph(self, node_id=1, x=0.0, frame='map'):
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = x
        return MapGraph(header=Header(frame_id=frame), poses_id=[node_id], poses=[pose])

    def objects(self, anchor_id=1, object_id=1, session='session-one', frame='map'):
        anchor = Observation(node_id=anchor_id, weight=1.0)
        anchor.position.x = 1.0
        return SemanticMap(header=Header(frame_id=frame), map_key='map.db', map_uuid=session,
            objects=[SemanticObject(object_id=object_id, label='椅子', observations=[anchor])])

    def labels(self):
        return [m for m in self.frames[-1].markers if m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD]

    def test_only_valid_aligned_depth_is_forwarded_without_resizing_or_retimestamping(self):
        images, infos, warnings = [], [], []
        node = SimpleNamespace(depth_pub=SimpleNamespace(publish=images.append),
            depth_info_pub=SimpleNamespace(publish=infos.append),
            get_logger=lambda: SimpleNamespace(warning=lambda text, **kwargs: warnings.append(text)))
        header = Header(frame_id='camera_color_optical_frame')
        header.stamp.sec = 123
        rgb = Image(header=deepcopy(header), width=2, height=2, encoding='rgb8', step=6, data=[10] * 12)
        depth = Image(header=deepcopy(header), width=2, height=2, encoding='16UC1', step=4, data=[232, 3] * 4)
        info = CameraInfo(header=deepcopy(header), width=2, height=2, k=[1., 0., 1., 0., 1., 1., 0., 0., 1.])
        message = RGBDImage(rgb=rgb, depth=depth, rgb_camera_info=info)
        adapter.MitiSceneAdapter.on_rgbd(node, message)
        self.assertIs(images[-1], message.depth)
        self.assertIs(infos[-1], message.rgb_camera_info)
        message.depth.width = 640
        adapter.MitiSceneAdapter.on_rgbd(node, message)
        self.assertEqual(len(images), 1)
        self.assertEqual(len(infos), 1)
        message.depth.width = 2
        message.depth.header.stamp.sec = 124
        adapter.MitiSceneAdapter.on_rgbd(node, message)
        self.assertEqual(len(images), 1)
        message.depth.header.stamp.sec = 123
        adapter.MitiSceneAdapter.on_rgbd(node, message)
        self.assertEqual(len(images), 2)
        self.assertEqual(len(warnings), 2)

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
        state = adapter.SemanticMarkers()
        result = state.update(SemanticMap(objects=[obj]), {1: np.eye(4)}, Header(frame_id='map'))
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)
        self.assertEqual(len(result.markers), 3)
        label = result.markers[-1]
        self.assertEqual(label.text, '椅子')
        self.assertEqual(label.header.frame_id, 'map')
        self.assertIn(str(2**40), label.ns)
        cleared = state.update(SemanticMap(), {}, Header(frame_id='map'))
        self.assertEqual([marker.action for marker in cleared.markers], [Marker.DELETE, Marker.DELETE])
        self.assertEqual({m.ns for m in cleared.markers}, {m.ns for m in result.markers[1:]})
        # Last-message-only clients must still receive the deletion after another update.
        again = state.update(SemanticMap(), {}, Header(frame_id='map'))
        self.assertEqual(cleared, again)

    def test_unchanged_objects_are_upserted_with_stable_ids_and_a_complete_snapshot(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        first = state.update(message, {1: np.eye(4)}, message.header)
        second = state.update(message, {1: np.eye(4)}, message.header)
        self.assertEqual(first.markers[1:], second.markers)
        self.assertEqual([m.action for m in second.markers], [Marker.ADD, Marker.ADD])

    def test_local_graph_then_new_semantics_never_removes_an_existing_label(self):
        scene = self.make_scene()
        scene.on_graph(self.graph(1, 10.0))
        scene.on_objects(self.objects(1))
        self.assertEqual(self.labels()[0].pose.position.x, 11.0)
        # Reproduce the live order: the local graph drops anchor 1 before semantics advance.
        scene.on_graph(self.graph(2, 20.0))
        self.assertEqual(self.labels()[0].pose.position.x, 11.0)
        self.assertEqual([m.action for m in self.frames[-1].markers], [Marker.ADD, Marker.ADD])
        scene.on_objects(self.objects(2))
        self.assertEqual(self.labels()[0].pose.position.x, 21.0)
        # Also handle the opposite order: semantics arrive before their new graph anchor.
        scene.on_objects(self.objects(3))
        self.assertEqual(self.labels()[0].pose.position.x, 21.0)
        scene.on_graph(self.graph(3, 30.0))
        self.assertEqual(self.labels()[0].pose.position.x, 31.0)
        self.assertTrue(all(m.action == Marker.ADD for frame in self.frames[1:] for m in frame.markers))

    def test_loop_correction_updates_cached_anchor_without_recreating_identity(self):
        scene = self.make_scene()
        scene.on_graph(self.graph(1, 10.0))
        scene.on_objects(self.objects(1))
        before = self.labels()[0]
        scene.on_graph(self.graph(2, 20.0))
        scene.on_graph(self.graph(1, 5.0))
        after = self.labels()[0]
        self.assertEqual((before.ns, before.id), (after.ns, after.id))
        self.assertEqual(after.pose.position.x, 6.0)
        self.assertIn(2, scene.poses)

    def test_missing_pose_does_not_invent_a_new_object_location(self):
        scene = self.make_scene()
        scene.on_graph(self.graph(1))
        scene.on_objects(self.objects(9))
        self.assertEqual(self.labels(), [])

    def test_removing_one_object_keeps_the_other_and_does_not_clear_the_scene(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        message.objects.append(self.objects(object_id=2).objects[0])
        state.update(message, {1: np.eye(4)}, message.header)
        message.objects = message.objects[1:]
        latest = state.update(message, {1: np.eye(4)}, message.header)
        self.assertEqual([m.action for m in latest.markers], [Marker.DELETE, Marker.DELETE, Marker.ADD, Marker.ADD])
        self.assertTrue(all('/1/' in m.ns for m in latest.markers[:2]))
        self.assertTrue(all('/2/' in m.ns for m in latest.markers[2:]))
        self.assertNotIn(1, state.positions)

    def test_map_reset_clears_old_positions_and_rejects_old_service_response(self):
        scene = self.make_scene()
        scene.on_graph(self.graph(1, 10.0))
        scene.on_objects(self.objects())
        old_epoch = scene.cache_epoch
        old = MapData(header=Header(frame_id='map'), graph=self.graph(1, 10.0))
        scene.on_objects(self.objects(session='session-two'))
        self.assertTrue(any(m.action == Marker.DELETEALL for m in self.frames[-1].markers))
        self.assertEqual(self.labels(), [])
        scene.restore_graph(old_epoch, old)
        self.assertEqual(scene.poses, {})
        scene.on_graph(self.graph(1, 100.0))
        self.assertEqual(self.labels()[0].pose.position.x, 101.0)
        self.assertTrue(all(m.action == Marker.ADD for m in self.frames[-1].markers))

    def test_restoring_missing_poses_never_overwrites_newer_streaming_poses(self):
        scene = self.make_scene()
        scene.on_graph(self.graph(1, 10.0))
        scene.on_objects(self.objects(2))
        graph = self.graph(2, 20.0)
        graph.poses_id.append(1)
        graph.poses.append(self.graph(1, -50.0).poses[0])
        scene.restore_graph(scene.cache_epoch, MapData(header=Header(frame_id='map'), graph=graph))
        self.assertEqual(scene.poses[1][0, 3], 10.0)
        self.assertEqual(self.labels()[0].pose.position.x, 21.0)

    def test_frame_change_never_reuses_positions_from_another_coordinate_frame(self):
        scene = self.make_scene()
        scene.on_graph(self.graph(1, 10.0))
        scene.on_objects(self.objects())
        scene.on_graph(self.graph(1, 200.0, frame='other_map'))
        self.assertEqual(self.labels(), [])
        scene.on_objects(self.objects(frame='other_map'))
        self.assertEqual(self.labels(), [])
        scene.on_graph(self.graph(1, 200.0, frame='other_map'))
        self.assertEqual(self.labels()[0].pose.position.x, 201.0)

    def test_invalid_pose_is_rejected(self):
        pose = Pose()
        pose.orientation.w = 0.0
        with self.assertRaises(ValueError):
            adapter.pose_matrix(pose)

    def test_duplicate_names_share_one_display_identity_without_label_oscillation(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        message.objects[0].label = '电视'
        message.objects[0].confidence = 0.8
        other = self.objects(anchor_id=2, object_id=2).objects[0]
        other.label, other.confidence = '显示器', 0.7
        message.objects.append(other)
        poses = {1: np.eye(4), 2: np.eye(4)}
        poses[2][0, 3] = 0.04
        state.update(message, poses, message.header)
        other.confidence = 0.81
        result = state.update(message, poses, message.header)
        labels = [m for m in result.markers if m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD]
        self.assertEqual([(m.ns, m.text) for m in labels], [('miti/1/label', '电视')])
        self.assertEqual(state.groups, {1: [1, 2]})
        self.assertFalse(any(m.action == Marker.DELETEALL for m in result.markers))

    def test_coobserved_nearby_objects_stay_distinct_even_after_observation_windows_change(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        other = self.objects(object_id=2).objects[0]
        other.observations[0].position.x += 0.02
        message.objects.append(other)
        poses = {1: np.eye(4), 2: np.eye(4)}
        state.update(message, poses, message.header)
        other.observations[0].node_id = 2
        result = state.update(message, poses, message.header)
        self.assertEqual(len([m for m in result.markers if m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD]), 2)

    def test_duplicate_group_cannot_chain_across_separate_objects(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        message.objects.extend([self.objects(anchor_id=i, object_id=i).objects[0] for i in (2, 3)])
        poses = {i: np.eye(4) for i in (1, 2, 3)}
        poses[2][0, 3], poses[3][0, 3] = 0.06, 0.12
        result = state.update(message, poses, message.header)
        self.assertEqual(state.groups, {1: [1, 2], 3: [3]})
        self.assertEqual(len([m for m in result.markers if m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD]), 2)

    def test_small_position_noise_does_not_toggle_an_established_group(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        message.objects.append(self.objects(anchor_id=2, object_id=2).objects[0])
        poses = {1: np.eye(4), 2: np.eye(4)}
        for offset in (0.07, 0.09, 0.075, 0.10):
            poses[2][0, 3] = offset
            state.update(message, poses, message.header)
            self.assertEqual(state.groups, {1: [1, 2]})
        poses[2][0, 3] = 0.13
        state.update(message, poses, message.header)
        self.assertEqual(state.groups, {1: [1], 2: [2]})

    def test_cached_positions_restore_only_for_the_exact_same_map_session(self):
        state = adapter.SemanticMarkers()
        message = self.objects()
        state.update(message, {1: np.eye(4)}, message.header)
        cache = state.cache_snapshot()
        restored = adapter.SemanticMarkers()
        restored.restore_cache(cache)
        result = restored.update(message, {}, message.header)
        self.assertEqual(len([m for m in result.markers if m.type == Marker.TEXT_VIEW_FACING and m.action == Marker.ADD]), 1)
        self.assertEqual(restored.positions[1].tolist(), [1.0, 0.0, 0.0])
        newer_pose = np.eye(4)
        newer_pose[0, 3] = 5.0
        restored.update(message, {1: newer_pose}, message.header)
        self.assertEqual(restored.positions[1].tolist(), [6.0, 0.0, 0.0])
        for field in ('map_key', 'map_uuid'):
            other = deepcopy(message)
            setattr(other, field, 'another-map')
            restored = adapter.SemanticMarkers()
            restored.restore_cache(cache)
            self.assertEqual(len(restored.update(other, {}, other.header).markers), 1)
            self.assertEqual(restored.positions, {})

    def test_invalid_cached_position_is_rejected(self):
        state = adapter.SemanticMarkers()
        for cache in ([], dict(schema=1, session=['map', 'uuid', 'map'], positions=[]),
                      dict(schema=1, session=['map', 'uuid', 'map'], positions={'1': [float('nan'), 0, 0]})):
            with self.subTest(cache=cache), self.assertRaises(ValueError):
                state.restore_cache(cache)


if __name__ == '__main__':
    unittest.main()
