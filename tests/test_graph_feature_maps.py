import sys
import unittest
from pathlib import Path

import numpy as np
from geometry_msgs.msg import Pose
from rtabmap_msgs.msg import Link, MapGraph, Node, Point3f
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from graph_feature_map_node import GraphFeatures
from map_geometry_node import cloud_xyz


def graph(positions, links=()):
    result = MapGraph(header=Header(frame_id='map'))
    for key, xyz in positions.items():
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
        result.poses_id.append(key)
        result.poses.append(pose)
    result.links = [Link(from_id=a, to_id=b, type=kind) for a, b, kind in links]
    return result


def node(key, points, stamp=1.0):
    return Node(id=key, stamp=stamp, word_pts=[Point3f(x=float(x), y=float(y), z=float(z)) for x, y, z in points])


class GraphFeatureTests(unittest.TestCase):
    def test_only_real_constraints_are_drawn_and_bidirectional_links_are_deduplicated(self):
        model = GraphFeatures()
        model.update(graph({1:(0,0,0), 2:(1,0,0), 3:(1,1,0)},
                           [(1,2,0), (2,1,0), (3,1,1), (2,3,2), (3,3,7), (3,99,1)]))
        markers, stats = model.graph_markers(Header(frame_id='map'))
        self.assertEqual(stats['nodes'], 3)
        self.assertEqual(stats['links'], 3)
        self.assertEqual(stats['link_counts']['neighbor'], 1)
        self.assertEqual(stats['link_counts']['global_loop'], 1)
        self.assertEqual(stats['link_counts']['local_loop'], 1)
        self.assertEqual(markers.markers[-1].action, Marker.DELETE)

    def test_features_follow_optimized_translation_and_rotation_with_invalid_points_removed(self):
        model = GraphFeatures(voxel_size=0)
        initial = graph({1:(10,20,30)})
        initial.poses[0].orientation.z = initial.poses[0].orientation.w = np.sqrt(.5)
        model.update(initial, [node(1, [(1,0,0), (0,0,0), (float('nan'),1,2)])])
        cloud, stats = model.feature_cloud(Header(frame_id='map'))
        np.testing.assert_allclose(cloud_xyz(cloud), [[10,21,30]])
        self.assertEqual(stats['points'], 1)
        model.update(graph({1:(2,3,4)}))
        cloud, _ = model.feature_cloud(Header(frame_id='map'))
        np.testing.assert_allclose(cloud_xyz(cloud), [[3,3,4]])

    def test_local_subgraphs_keep_omitted_nodes_but_full_snapshots_remove_them(self):
        model = GraphFeatures()
        model.update(graph({1:(0,0,0), 2:(1,0,0)}, [(1,2,0)]), [node(1, [(1,1,1)]), node(2, [(2,2,2)])])
        model.update(graph({2:(2,0,0)}))
        self.assertEqual(set(model.poses), {1,2})
        revision = model.revision
        model.update(graph({2:(2,0,0)}), snapshot_revision=revision)
        self.assertEqual(set(model.poses), {2})
        self.assertEqual(set(model.features), {2})
        self.assertFalse(model.links)

    def test_delayed_snapshot_keeps_newer_streaming_poses_features_and_links(self):
        model = GraphFeatures(voxel_size=0)
        model.update(graph({1:(0,0,0)}), [node(1, [(1,1,1)])])
        revision = model.revision
        model.update(graph({1:(5,0,0), 2:(10,0,0)}, [(1,2,1)]), [node(2, [(1,1,1)])])
        model.update(graph({1:(0,0,0)}), [node(1, [(1,1,1)])], snapshot_revision=revision)
        self.assertEqual(model.poses[1][0,3], 5)
        self.assertIn(2, model.poses)
        self.assertIn((1,2,1), model.links)

    def test_voxel_deduplication_point_limit_and_empty_snapshot(self):
        model = GraphFeatures(voxel_size=.02, max_points=2)
        model.update(graph({1:(0,0,0), 2:(0,0,0)}),
                     [node(1, [(1,1,1), (2,2,2), (3,3,3)]), node(2, [(1.001,1,1)])])
        cloud, stats = model.feature_cloud(Header(frame_id='map'))
        self.assertEqual(stats['observations'], 4)
        self.assertEqual(stats['unique_points'], 3)
        self.assertEqual(cloud.width, 2)
        self.assertEqual(len(cloud.data), cloud.row_step)
        model.clear()
        markers, stats = model.graph_markers(Header(frame_id='map'))
        self.assertTrue(all(m.action == Marker.DELETE for m in markers.markers))
        self.assertEqual(stats['nodes'], 0)
        cloud, stats = model.feature_cloud(Header(frame_id='map'))
        self.assertEqual(len(cloud.data), 0)
        self.assertIsNone(stats['bounds'])

    def test_new_session_with_reused_node_id_clears_old_features(self):
        model = GraphFeatures()
        model.update(graph({1:(0,0,0), 2:(1,0,0)}), [node(1, [(1,1,1)]), node(2, [(2,2,2)])])
        epoch = model.epoch
        model.update(graph({1:(0,0,0)}), [node(1, [(3,3,3)], stamp=99.)])
        self.assertGreater(model.epoch, epoch)
        self.assertEqual(set(model.features), {1})
        self.assertEqual(set(model.poses), {1})


if __name__ == '__main__':
    unittest.main()
