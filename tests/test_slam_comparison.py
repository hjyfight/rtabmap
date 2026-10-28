"""Check coordinate alignment, saved-map evidence and supported comparison modes."""

import importlib.util
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

import numpy as np
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


compare = load('compare_databases', 'scripts/compare_slam_databases.py')
launcher = load('comparison_launch', 'launch/slam_comparison.launch.py')


class SavedMapTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='slam_compare_test_')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)

    def test_named_ply_fields_and_nonfinite_points(self):
        path = self.root / 'map.ply'
        path.write_text('ply\nformat ascii 1.0\nelement vertex 3\n'
                        'property float z\nproperty uchar intensity\nproperty float x\n'
                        'property float y\nend_header\n3 80 1 2\n6 50 4 5\nnan 60 1 2\n')
        np.testing.assert_allclose(compare.read_ascii_ply(path), [[1, 2, 3], [4, 5, 6]])

    def test_pose_ids_preserved_landmarks_excluded(self):
        path = self.root / 'poses.txt'
        path.write_text('#timestamp x y z qx qy qz qw id\n'
                        '1 3 4 5 0 0 0 1 42\n2 0 0 0 0 0 0 1 -1\n')
        poses = compare.read_poses(path)
        self.assertEqual(set(poses), {42})
        np.testing.assert_allclose(poses[42][1][:3, 3], [3, 4, 5])
        path.write_text('1 0 0 0 0 0 0 1 42\n2 1 0 0 0 0 0 1 42\n')
        with self.assertRaises(ValueError):
            compare.read_poses(path)

    def test_rigid_start_alignment_does_not_scale_trajectory(self):
        start = np.array([[0., -1, 0, 10], [1, 0, 0, 20],
                          [0, 0, 1, 30], [0, 0, 0, 1]])
        xyz = np.array([[10, 20, 30], [10, 22, 30], [7, 20, 30]])
        actual = compare.in_start_frame(xyz, start)
        np.testing.assert_allclose(actual, [[0, 0, 0], [2, 0, 0], [0, 3, 0]])
        self.assertAlmostEqual(np.linalg.norm(actual[1] - actual[2]), np.sqrt(13))

    def test_only_actual_loop_links_count_once(self):
        links = [dict(zip(('from', 'to', 'type'), row)) for row in
                 [(1, 2, 2), (2, 1, 2), (1, 2, 1), (1, 2, 0), (1, 1, 9), (1, 8, 2)]]
        self.assertEqual(compare.unique_loop_links(links, {1, 2}), [(1, 2, 1), (1, 2, 2)])

    def test_unfinished_database_rejected_without_changing_source(self):
        source = self.root / 'unfinished.db'
        with sqlite3.connect(source) as db:
            db.execute('create table Admin (opt_poses blob)')
            db.execute('insert into Admin values (null)')
        before = source.read_bytes()
        with self.assertRaisesRegex(ValueError, '最终优化位姿'):
            compare.export_run(source, self.root / 'export', 'lidar')
        self.assertEqual(source.read_bytes(), before)

    def test_existing_output_is_not_overwritten(self):
        with self.assertRaises(FileExistsError):
            compare.export_run(self.root / 'missing.db', self.root, 'lidar')


class ComparisonLaunchTests(unittest.TestCase):
    def context(self, **overrides):
        context = LaunchContext()
        context.launch_configurations.update(overrides)
        for action in launcher.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return context

    def test_only_camera_switch_differs_between_pipeline_arms(self):
        with tempfile.TemporaryDirectory(prefix='comparison_launch_') as tmp:
            options = []
            for mode in ('lidar', 'rgbd'):
                directory = Path(tmp) / mode
                launcher.setup(self.context(mode=mode, run_dir=str(directory)))
                options.append(json.loads((directory / 'manifest.json').read_text())['launch_arguments'])
            supported = {a.name for a in launcher._base.generate_launch_description().entities
                         if isinstance(a, DeclareLaunchArgument)}
            self.assertFalse(set(options[0]) - supported)
            differences = {k for k in options[0] if options[0][k] != options[1][k]}
            self.assertEqual(differences, {'use_camera', 'database_path'})
            for config in options:
                self.assertEqual(config['use_imu'], 'true')
                self.assertEqual(config['deskewing'], 'true')
                self.assertEqual(config['assemble'], 'false')

    def test_unsupported_depth_mode_fails_before_creating_output(self):
        with tempfile.TemporaryDirectory(prefix='comparison_launch_') as tmp:
            directory = Path(tmp) / 'depth'
            with self.assertRaises(ValueError):
                launcher.setup(self.context(mode='depth', run_dir=str(directory)))
            self.assertFalse(directory.exists())


if __name__ == '__main__':
    unittest.main()
