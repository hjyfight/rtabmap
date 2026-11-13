import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_cloud_filter import DynamicCloudFilter


class DynamicCloudTests(unittest.TestCase):
    def setUp(self):
        self.filter = DynamicCloudFilter()
        self.addCleanup(self.filter.close)
        self.origin = [.05, .05, .05]
        self.old_person = [2.05, .05, .05]
        self.wall = [5.05, .05, .05]
        self.unseen = [2.05, 1.05, .05]
        self.map = np.array([self.old_person, self.wall, self.unseen], dtype=np.float32)
        self.filter.set_map(self.map)

    def test_departed_person_is_cleared_after_three_independent_rays_while_environment_stays(self):
        for stamp in (1., 2.):
            self.filter.scan([self.wall], self.origin, stamp)
            np.testing.assert_array_equal(self.filter.filtered_map(), self.map)
        result = self.filter.scan([self.wall], self.origin, 3.)
        self.assertEqual(result['cleared_voxels'], 1)
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map[1:])

    def test_occluded_and_unobserved_environment_is_never_aged_out(self):
        for stamp in range(1, 20):
            self.filter.scan([[1.05, .05, .05]], self.origin, float(stamp))
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)

    def test_many_rays_in_one_message_and_duplicate_or_reordered_messages_are_only_one_vote(self):
        for stamp in (1., 1., .5):
            self.filter.scan([self.wall]*1000, self.origin, stamp)
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)
        self.filter.scan([self.wall], self.origin, 2.)
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)
        self.filter.scan([self.wall], self.origin, 3.)
        self.assertEqual(len(self.filter.filtered_map()), 2)

    def test_new_hits_restore_a_cleared_voxel_and_repeated_map_snapshots_do_not_restore_ghosts(self):
        for stamp in (1., 2., 3.):
            self.filter.scan([self.wall], self.origin, stamp)
        self.filter.set_map(self.map.copy())
        self.assertEqual(len(self.filter.filtered_map()), 2)
        self.filter.scan([self.old_person], self.origin, 4.)
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)

    def test_surface_neighbors_are_protected_and_old_unconfirmed_evidence_expires(self):
        adjacent = np.array([[5.05, .15, .05], self.wall], dtype=np.float32)
        self.filter.set_map(adjacent)
        for stamp in range(1, 10):
            self.filter.scan([self.wall], self.origin, float(stamp))
        np.testing.assert_array_equal(self.filter.filtered_map(), adjacent)
        self.filter.set_map(self.map)
        for stamp in (20., 30., 40.):
            self.filter.scan([self.wall], self.origin, stamp)
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)

    def test_range_limit_and_epoch_reset_preserve_unproven_geometry(self):
        far = np.array([[15.05, .05, .05]], dtype=np.float32)
        self.filter.set_map(far)
        for stamp in (1., 2., 3.):
            self.filter.scan([[20.05, .05, .05]], self.origin, stamp)
        np.testing.assert_array_equal(self.filter.filtered_map(), far)
        self.filter.set_map(self.map)
        for stamp in (4., 5., 6.):
            self.filter.scan([self.wall], self.origin, stamp)
        self.filter.reset()
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)
        self.assertTrue(self.filter.scan([self.wall], self.origin, 1.)['accepted'])

    def test_invalid_returns_empty_maps_and_nonfinite_inputs_do_not_create_free_evidence(self):
        for stamp in (1., 2., 3.):
            self.filter.scan([[math.nan, 0, 0], [0, 0, 0]], self.origin, stamp)
        np.testing.assert_array_equal(self.filter.filtered_map(), self.map)
        with self.assertRaises(ValueError):
            self.filter.scan([self.wall], [math.nan, 0, 0], 4.)
        self.filter.set_map(np.empty((0, 3)))
        self.assertEqual(self.filter.filtered_map().shape, (0, 3))


if __name__ == '__main__':
    unittest.main()
