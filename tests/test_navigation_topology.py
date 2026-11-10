import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from navigation_topology import NavigationConfig, build_navigation_graph, shortest_route


def two_rooms():
    grid = np.full((100, 210), 100, dtype=np.int8)
    grid[10:85, 10:80] = 0
    grid[10:85, 130:200] = 0
    grid[40:54, 80:130] = 0
    return grid


class NavigationTopologyTests(unittest.TestCase):
    def test_two_rooms_and_narrow_corridor_form_region_portal_navigation_graph(self):
        graph, labels, clearance = build_navigation_graph(two_rooms(), .1)
        kinds = [n['kind'] for n in graph['nodes']]
        self.assertEqual(kinds.count('area'), 2)
        self.assertGreaterEqual(kinds.count('corridor'), 1)
        self.assertGreaterEqual(kinds.count('doorway_candidate'), 2)
        corridor = next(n for n in graph['nodes'] if n['kind'] == 'corridor')
        self.assertTrue(9 < corridor['position'][0] < 12)
        self.assertEqual(len(graph['components']), 1)
        self.assertTrue(all(not n['semantic_confirmed'] for n in graph['nodes']))
        areas = [n['id'] for n in graph['nodes'] if n['kind'] == 'area']
        route = shortest_route(graph, *areas)
        self.assertIsNotNone(route)
        self.assertGreater(route['length_m'], 10)
        self.assertTrue(any(key.startswith('P') for key in route['nodes']))
        for edge in graph['edges']:
            self.assertGreaterEqual(edge['min_clearance_m'], .45)
            length = 0
            for a, b in zip(edge['polyline'], edge['polyline'][1:]):
                a, b = np.array(a), np.array(b)
                length += np.linalg.norm(b-a)
                for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b-a)/.02))):
                    p = a*(1-t)+b*t
                    x, y = np.floor(p[:2]/.1).astype(int)
                    self.assertGreaterEqual(clearance[y, x], .45)
                    self.assertGreater(labels[y, x], 0)
            self.assertAlmostEqual(length, edge['length_m'], places=6)

    def test_unknown_gap_or_closed_door_removes_connection_and_route(self):
        for value in (-1, 100):
            grid = two_rooms()
            grid[:, 105:107] = value
            graph, _, _ = build_navigation_graph(grid, .1)
            self.assertEqual(len(graph['components']), 2)
            areas = [n['id'] for n in graph['nodes'] if n['kind'] == 'area']
            self.assertIsNone(shortest_route(graph, *areas))

    def test_alternative_corridor_keeps_a_route_when_one_connection_closes(self):
        grid = two_rooms()
        grid[65:79, 80:130] = 0
        graph, _, _ = build_navigation_graph(grid, .1)
        self.assertEqual(len(graph['components']), 1)
        self.assertGreaterEqual(len(graph['edges']), len(graph['nodes']))
        grid[40:54, 105:107] = 100
        graph, _, _ = build_navigation_graph(grid, .1)
        areas = [n['id'] for n in graph['nodes'] if n['kind'] == 'area']
        self.assertIsNotNone(shortest_route(graph, *areas))

    def test_narrow_door_does_not_fit_robot_and_diagonal_corners_do_not_connect(self):
        grid = two_rooms()
        grid[40:54, 80:130] = 100
        grid[44:49, 80:130] = 0
        graph, _, _ = build_navigation_graph(grid, .1)
        self.assertEqual(len(graph['components']), 2)
        grid = np.full((70, 70), 100, dtype=np.int8)
        grid[3:35, 3:35] = 0
        grid[35:67, 35:67] = 0
        graph, _, _ = build_navigation_graph(grid, .1)
        self.assertEqual(len(graph['components']), 2)

    def test_rotated_translated_grid_preserves_costs_and_moves_all_vertices(self):
        original, _, _ = build_navigation_graph(two_rooms(), .1)
        moved, _, _ = build_navigation_graph(two_rooms(), .1, (3., -7., .4), math.pi/2)
        for before, after in zip(original['nodes'], moved['nodes']):
            x, y, z = before['position']
            np.testing.assert_allclose(after['position'], [3-y, -7+x, .4], atol=1e-9)
        self.assertEqual([e['length_m'] for e in original['edges']], [e['length_m'] for e in moved['edges']])

    def test_empty_and_fragmentary_floor_explain_missing_navigable_space(self):
        for grid in [np.full((1, 1), -1), np.full((20, 20), 100), np.indices((30, 30)).sum(axis=0)%2*100]:
            graph, labels, _ = build_navigation_graph(grid, .1)
            self.assertEqual(graph['state'], 'insufficient_free_space')
            self.assertFalse(graph['nodes'])
            self.assertFalse(graph['edges'])
            self.assertFalse(labels.any())
        graph, _, _ = build_navigation_graph(np.zeros((40, 40)), .1)
        self.assertEqual(graph['state'], 'ready')
        self.assertEqual(len(graph['components']), 1)

    def test_size_coordinate_and_parameter_validation(self):
        for data in [np.array([[np.nan]]), np.array([[101]]), np.zeros((2, 2, 2))]:
            with self.assertRaises(ValueError):
                build_navigation_graph(data, .1)
        with self.assertRaises(ValueError):
            build_navigation_graph(two_rooms(), .1, yaw=float('nan'))
        with self.assertRaises(ValueError):
            build_navigation_graph(two_rooms(), .1, config=NavigationConfig(max_cells=10))
        with self.assertRaises(ValueError):
            NavigationConfig(robot_radius=1.1)

    def test_shortest_route_uses_metric_weights_and_rejects_unknown_ids(self):
        graph = dict(nodes=[dict(id=x) for x in ('A', 'B', 'C', 'D')], edges=[
            dict(source='A', target='B', length_m=8.), dict(source='A', target='C', length_m=2.),
            dict(source='C', target='B', length_m=3.)])
        self.assertEqual(shortest_route(graph, 'A', 'B'), dict(nodes=['A', 'C', 'B'], length_m=5.))
        self.assertIsNone(shortest_route(graph, 'A', 'D'))
        with self.assertRaises(ValueError):
            shortest_route(graph, 'A', 'missing')


if __name__ == '__main__':
    unittest.main()
