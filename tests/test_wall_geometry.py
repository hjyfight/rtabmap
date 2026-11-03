import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from wall_geometry import WallConfig, extract_walls, pose_values, transform_matrix


def scene():
    y, z = np.meshgrid(np.arange(-2., 2.01, 0.07), np.arange(-0.5, 2.51, 0.07))
    wall = np.column_stack((np.full(y.size, 3.), y.ravel(), z.ravel()))
    # Door from floor to z=1.5, one metre wide.
    wall = wall[~((np.abs(wall[:, 1]) < 0.55) & (wall[:, 2] < 1.5))]
    x, y = np.meshgrid(np.arange(-2., 3., 0.1), np.arange(-2., 2., 0.1))
    floor = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -0.5)))
    return wall, floor


class WallGeometryTests(unittest.TestCase):
    def test_wall_and_door_hole_survive_noise_without_floor_plane(self):
        wall, floor = scene()
        points = np.vstack((wall, floor))
        points += np.random.default_rng(5).normal(0, 0.006, points.shape)
        found = extract_walls(points)
        self.assertTrue(found)
        triangles = np.vstack([w['triangles'] for w in found])
        self.assertLess(np.abs(triangles[:, 0]-3).max(), 0.03)
        centroids = triangles.reshape(-1, 3, 3).mean(axis=1)
        self.assertFalse(((abs(centroids[:, 1]) < 0.3) & (centroids[:, 2] < 1.2)).any())
        self.assertGreater(sum(w['area'] for w in found), 5)

    def test_empty_floor_and_small_objects_do_not_create_walls(self):
        _, floor = scene()
        self.assertEqual(extract_walls(floor), [])
        self.assertEqual(extract_walls([[np.nan, 0, 0], [0, np.inf, 0]]), [])
        y, z = np.meshgrid(np.arange(0, 0.5, .04), np.arange(0, .7, .04))
        self.assertEqual(extract_walls(np.column_stack((np.ones(y.size), y.ravel(), z.ravel()))), [])

    def test_rotated_walls_follow_new_map_instead_of_appending_old_geometry(self):
        wall, _ = scene()
        rotation = Rotation.from_euler('z', 37, degrees=True).as_matrix()
        moved = wall @ rotation.T + [4, -3, 1]
        found = extract_walls(moved)
        self.assertTrue(found)
        for candidate in found:
            self.assertGreater(abs(candidate['normal'] @ rotation[:, 0]), 0.999)
            back = (candidate['triangles'] - [4, -3, 1]) @ rotation
            self.assertLess(np.abs(back[:, 0]-3).max(), 0.03)

    def test_pose_composes_map_odom_and_real_mount_translation_and_rotation(self):
        rz = Rotation.from_euler('z', 90, degrees=True).as_quat()
        world = transform_matrix([10, 20, 0], rz) @ transform_matrix([1, 0, 0], rz) @ transform_matrix([.2, 0, 0], [0, 0, 0, 1])
        position, quaternion, rpy = pose_values(world)
        np.testing.assert_allclose(position, [9.8, 21, 0], atol=1e-9)
        self.assertAlmostEqual(abs(np.rad2deg(rpy[2])), 180)
        np.testing.assert_allclose(Rotation.from_quat(quaternion).as_matrix(), world[:3, :3], atol=1e-9)

    def test_pose_roll_pitch_yaw_and_gimbal_limit(self):
        for angles in ([25, -40, 170], [0, 90, 70], [0, -90, -70]):
            matrix = transform_matrix([1, 2, 3], Rotation.from_euler('xyz', angles, degrees=True).as_quat())
            _, _, rpy = pose_values(matrix)
            np.testing.assert_allclose(Rotation.from_euler('xyz', rpy).as_matrix(), matrix[:3, :3], atol=1e-7)
        with self.assertRaises(ValueError):
            transform_matrix([0, 0, 0], [0, 0, 0, 0])
        with self.assertRaises(ValueError):
            WallConfig(distance=-1)


if __name__ == '__main__':
    unittest.main()
