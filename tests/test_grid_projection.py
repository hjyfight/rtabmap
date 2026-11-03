import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from grid_projection import GridConfig, project_grid


def floor_cloud():
    x, y = np.meshgrid(np.arange(-2, 2.01, .1), np.arange(-2, 2.01, .1))
    return np.column_stack((x.ravel(), y.ravel(), np.full(x.size, -.5)))


def cell(result, x, y, resolution=.05):
    ix = int(np.floor(x/resolution) - round(result['origin'][0]/resolution))
    iy = int(np.floor(y/resolution) - round(result['origin'][1]/resolution))
    return result['data'][iy, ix]


class GridProjectionTests(unittest.TestCase):
    def test_ceiling_does_not_cover_free_floor_and_walls_remain(self):
        floor = floor_cloud()
        ceiling = floor + [0, 0, 3]
        y, z = np.meshgrid(np.arange(-2, 2.01, .05), np.arange(-.3, 2.6, .05))
        wall = np.column_stack((np.full(y.size, 2), y.ravel(), z.ravel()))
        result = project_grid(np.vstack((floor, ceiling, wall)), [0, 0, 0])
        self.assertEqual(result['state'], 'ready')
        self.assertAlmostEqual(result['floor'][2], -.5, places=5)
        self.assertGreater(result['removed_high_points'], len(ceiling))
        self.assertEqual(cell(result, floor[700, 0], floor[700, 1]), 0)
        self.assertEqual(cell(result, wall[700, 0], wall[700, 1]), 100)

    def test_height_is_relative_to_tilted_floor_not_global_z_zero(self):
        floor = floor_cloud()
        floor[:, 2] = -.8 + .12*floor[:, 0] - .08*floor[:, 1]
        overhead = floor + [0, 0, 2.4]
        result = project_grid(np.vstack((floor, overhead)), [0, 0, 0])
        self.assertEqual(result['state'], 'ready')
        np.testing.assert_allclose(result['floor'], [.12, -.08, -.8], atol=1e-6)
        self.assertEqual(result['obstacle_points'], 0)
        self.assertEqual(result['ground_points'], len(floor))

    def test_table_or_ceiling_without_floor_remains_unknown(self):
        for offset in (1.2, 3.):
            result = project_grid(floor_cloud()+[0, 0, offset], [0, 0, 0])
            self.assertEqual(result['state'], 'waiting_ground')
            self.assertTrue((result['data'] == -1).all())

    def test_unobserved_region_and_doorway_are_not_filled_or_blocked(self):
        floor = floor_cloud()
        floor = floor[np.abs(floor[:, 0]) > .5]
        y, z = np.meshgrid(np.arange(-2, 2.01, .05), np.arange(-.3, 2.6, .05))
        wall = np.column_stack((np.zeros(y.size), y.ravel(), z.ravel()))
        wall = wall[~((np.abs(wall[:, 1]) < .5) & (wall[:, 2] < 1.6))]
        result = project_grid(np.vstack((floor, wall)), [0, 0, 0])
        self.assertEqual(result['state'], 'ready')
        self.assertEqual(cell(result, 0, 0), -1, 'doorway has no observed floor and must remain unknown')
        self.assertEqual(cell(result, 0, -1.5), 100)

    def test_new_snapshot_replaces_removed_obstacle_and_noise_is_not_a_wall(self):
        floor = floor_cloud()
        cluster = np.array([[1., 1., .3], [1., 1., .35], [1., 1., .4]])
        before = project_grid(np.vstack((floor, cluster)), [0, 0, 0])
        self.assertEqual(cell(before, 1, 1), 100)
        after = project_grid(np.vstack((floor, [[-1.5, -1.5, .3]])), [0, 0, 0])
        self.assertEqual(after['obstacle_points'], 0)
        self.assertEqual(after['removed_noise_points'], 1)
        self.assertFalse((after['data'] == 100).any())
        self.assertTrue((before['data'] == 100).any(), 'input result is not mutated')

    def test_empty_malformed_and_excessive_map_inputs(self):
        result = project_grid(np.empty((0, 3)), [0, 0, 0])
        self.assertEqual(result['state'], 'empty')
        self.assertEqual(result['data'].tolist(), [[-1]])
        with self.assertRaises(ValueError):
            project_grid(floor_cloud(), [float('nan'), 0, 0])
        with self.assertRaises(ValueError):
            project_grid(np.vstack((floor_cloud(), [[1e4, 1e4, 0]])), [0, 0, 0])
        with self.assertRaises(ValueError):
            GridConfig(max_obstacle_height=.1)


if __name__ == '__main__':
    unittest.main()
