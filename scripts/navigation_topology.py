"""Geometric region/portal graph from an occupancy grid, with checked free paths.

Unknown cells remain blocked. Region and doorway names are geometric candidates,
not semantic detections. This module never commands a robot.
"""
from collections import defaultdict, deque
from dataclasses import dataclass
import heapq
import math

import numpy as np
from scipy import ndimage

CROSS = ndimage.generate_binary_structure(2, 1)
NEIGHBORS = ((-1, 0), (0, -1), (0, 1), (1, 0))


@dataclass(frozen=True)
class NavigationConfig:
    robot_radius: float = 0.40
    safety_margin: float = 0.05
    room_core_clearance: float = 1.0
    min_core_area: float = 0.30
    min_region_area: float = 0.25
    corridor_elongation: float = 3.0
    doorway_max_width: float = 1.6
    free_threshold: int = 25
    max_cells: int = 1000000
    max_regions: int = 256

    def __post_init__(self):
        positive = ('robot_radius', 'room_core_clearance', 'min_core_area',
                    'min_region_area', 'corridor_elongation', 'doorway_max_width')
        if any(not math.isfinite(getattr(self, key)) or getattr(self, key) <= 0 for key in positive):
            raise ValueError('navigation dimensions must be finite and positive')
        if not math.isfinite(self.safety_margin) or self.safety_margin < 0:
            raise ValueError('safety_margin must be finite and nonnegative')
        if self.room_core_clearance <= self.clearance:
            raise ValueError('room_core_clearance must exceed the required robot clearance')
        if not 0 <= self.free_threshold < 100 or not 1 <= self.max_cells <= 4000000 or not 1 <= self.max_regions <= 2000:
            raise ValueError('invalid occupancy or grid size limit')

    @property
    def clearance(self):
        return self.robot_radius + self.safety_margin


def _elongation(cells):
    if len(cells) < 3:
        return 1.0
    eig = np.linalg.eigvalsh(np.cov(cells.T))
    return float(math.sqrt((eig[1] + 1) / (eig[0] + 1)))


def partition_regions(safe, clearance, resolution, config):
    """Grow broad-space cores a bounded distance; keep connecting corridors."""
    cores, count = ndimage.label(safe & (clearance >= config.room_core_clearance), CROSS)
    labels = np.zeros(safe.shape, dtype=np.int32)
    kinds, next_label = {}, 1
    core_sizes = np.bincount(cores.ravel())
    core_boxes = ndimage.find_objects(cores)
    for key in np.flatnonzero(core_sizes[1:]*resolution**2 >= config.min_core_area)+1:
        box = core_boxes[key-1]
        labels[box][cores[box] == key] = next_label
        kinds[next_label] = 'area'
        next_label += 1
    if len(kinds) > config.max_regions:
        raise ValueError('地图过于碎片化，区域数量超过配置上限')
    h, w = safe.shape
    frontier = deque((int(y), int(x), 0) for y, x in np.argwhere(labels > 0))
    steps = int(math.floor((config.room_core_clearance - config.clearance) / resolution))
    while frontier:
        y, x, depth = frontier.popleft()
        if depth >= steps:
            continue
        for dy, dx in NEIGHBORS:
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and safe[ny, nx] and labels[ny, nx] == 0:
                labels[ny, nx] = labels[y, x]
                frontier.append((ny, nx, depth + 1))
    leftover, count = ndimage.label(safe & (labels == 0), CROSS)
    sizes = np.bincount(leftover.ravel())
    touching = np.unique(leftover[ndimage.binary_dilation(labels > 0, structure=CROSS) & (labels == 0)])
    eligible = set((np.flatnonzero(sizes[1:]*resolution**2 >= config.min_region_area)+1).tolist())
    eligible.update(int(key) for key in touching if key > 0)
    if not eligible:
        return labels, kinds
    boxes = ndimage.find_objects(leftover)
    for key in sorted(eligible):
        by, bx = boxes[key-1]
        box = (slice(max(0, by.start-1), min(h, by.stop+1)), slice(max(0, bx.start-1), min(w, bx.stop+1)))
        mask = leftover[box] == key
        cells = np.argwhere(mask)
        adjacent = np.unique(labels[box][ndimage.binary_dilation(mask, structure=CROSS) & ~mask])
        adjacent = adjacent[adjacent > 0]
        long = _elongation(cells) >= config.corridor_elongation
        area = len(cells) * resolution**2
        if len(adjacent) == 1 and (not long or area < config.min_region_area):
            labels[box][mask] = adjacent[0]
        elif area >= config.min_region_area or len(adjacent) >= 2:
            if len(kinds) >= config.max_regions:
                raise ValueError('地图过于碎片化，区域数量超过配置上限')
            labels[box][mask] = next_label
            kinds[next_label] = 'corridor' if long or len(adjacent) >= 2 else 'area'
            next_label += 1
    return labels, kinds


def _simplify(cells):
    """Remove only collinear samples; never replace bends with unchecked chords."""
    if len(cells) < 3:
        return cells
    result = [cells[0]]
    for i in range(1, len(cells) - 1):
        a, b, c = np.asarray(cells[i-1]), np.asarray(cells[i]), np.asarray(cells[i+1])
        if not np.array_equal(b-a, c-b):
            result.append(cells[i])
    return result + [cells[-1]]


def build_navigation_graph(data, resolution, origin=(0., 0., 0.), yaw=0., config=NavigationConfig()):
    grid = np.asarray(data)
    origin = np.asarray(origin, dtype=float)
    if (grid.ndim != 2 or not grid.size or grid.size > config.max_cells or
            not np.issubdtype(grid.dtype, np.number) or not np.isfinite(grid).all() or
            ((grid < -1) | (grid > 100)).any()):
        raise ValueError('invalid occupancy grid')
    if not math.isfinite(resolution) or resolution <= 0 or not math.isfinite(yaw):
        raise ValueError('invalid grid resolution or yaw')
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('invalid map origin')
    free = (grid >= 0) & (grid <= config.free_threshold)
    # Subtract a half-cell diagonal from the center-to-center distance. Padding
    # treats the exterior as unknown and gives a conservative obstacle clearance.
    clearance = np.maximum(0., ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
                           * resolution - resolution / math.sqrt(2))
    safe = free & (clearance >= config.clearance)
    labels, kinds = partition_regions(safe, clearance, resolution, config)
    h, w = grid.shape
    c, s = math.cos(yaw), math.sin(yaw)

    def world(cell):
        y, x = cell
        px, py = (x + .5) * resolution, (y + .5) * resolution
        return [float(origin[0] + c*px - s*py), float(origin[1] + s*px + c*py), float(origin[2])]

    nodes, edges, centers = [], [], {}
    parents = np.full(grid.shape, -1, dtype=np.int64)
    distances = np.full(grid.shape, -1, dtype=np.int32)
    for key in sorted(kinds):
        cells = np.argwhere(labels == key)
        if not len(cells):
            continue
        best = clearance[cells[:, 0], cells[:, 1]]
        # Put a corridor representative in its middle. Its widest cells are
        # often at the room entrance and would overlap the portal marker.
        candidates = cells if kinds[key] == 'corridor' else cells[np.isclose(best, best.max())]
        center = tuple(candidates[np.argmin(np.linalg.norm(candidates - cells.mean(axis=0), axis=1))])
        centers[key] = center
        nodes.append(dict(id=f'R{key}', kind=kinds[key], label=('通道 ' if kinds[key] == 'corridor' else '区域 ') + str(key),
                          position=world(center), area_m2=round(len(cells)*resolution**2, 4),
                          clearance_m=float(clearance[center]), semantic_confirmed=False))
        # One shortest-path tree per region checks all future center-to-portal paths.
        queue = deque([center])
        distances[center] = 0
        while queue:
            y, x = queue.popleft()
            for dy, dx in NEIGHBORS:
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and labels[ny, nx] == key and distances[ny, nx] < 0:
                    distances[ny, nx] = distances[y, x] + 1
                    parents[ny, nx] = y*w + x
                    queue.append((ny, nx))

    boundaries = defaultdict(list)
    for dy, dx in ((0, 1), (1, 0)):
        a = labels[:h-dy, :w-dx]
        b = labels[dy:, dx:]
        ys, xs = np.nonzero((a > 0) & (b > 0) & (a != b))
        for y, x in zip(ys, xs):
            ca, cb = (int(y), int(x)), (int(y+dy), int(x+dx))
            ka, kb = int(labels[ca]), int(labels[cb])
            if ka > kb:
                ka, kb, ca, cb = kb, ka, cb, ca
            boundaries[(ka, kb)].append((ca, cb))

    def path_to(cell):
        result = [cell]
        while parents[cell] >= 0:
            cell = divmod(int(parents[cell]), w)
            result.append(cell)
        return list(reversed(result))

    for (ka, kb), pairs in sorted(boundaries.items()):
        boundary_mask = np.zeros(grid.shape, dtype=bool)
        for a, _ in pairs:
            boundary_mask[a] = True
        clusters, count = ndimage.label(boundary_mask, np.ones((3, 3), dtype=bool))
        for cluster in range(1, count + 1):
            options = [(a, b) for a, b in pairs if clusters[a] == cluster]
            a, b = max(options, key=lambda p: (min(clearance[p[0]], clearance[p[1]]),
                        -int(distances[p[0]] + distances[p[1]])))
            portal_cell = ((a[0]+b[0])/2, (a[1]+b[1])/2)
            width = 2 * min(clearance[a], clearance[b])
            portal_id = 'P' + str(sum(n['id'].startswith('P') for n in nodes) + 1)
            door = width <= config.doorway_max_width
            nodes.append(dict(id=portal_id, kind='doorway_candidate' if door else 'passage',
                              label=('候选门口 ' if door else '连接口 ') + portal_id[1:], position=world(portal_cell),
                              width_lower_bound_m=float(width), semantic_confirmed=False))
            for key, cell in ((ka, a), (kb, b)):
                route = path_to(cell)
                minimum = min(float(clearance[p]) for p in route)
                points = [world(p) for p in _simplify(route)] + [world(portal_cell)]
                edges.append(dict(source=f'R{key}', target=portal_id,
                                  length_m=(len(route)-.5)*resolution,
                                  min_clearance_m=min(minimum, float(width/2)), polyline=points,
                                  traversability='observed_free'))
    components = connected_components(nodes, edges)
    xy = np.array([n['position'] for n in nodes])
    bounds = None if not len(xy) else dict(center=((xy.min(axis=0)+xy.max(axis=0))/2).tolist(),
                                         span=(xy.max(axis=0)-xy.min(axis=0)).tolist())
    corners = np.array([world(p) for p in [(-.5, -.5), (-.5, w-.5), (h-.5, -.5), (h-.5, w-.5)]])
    grid_bounds = dict(center=((corners.min(axis=0)+corners.max(axis=0))/2).tolist(),
                       span=(corners.max(axis=0)-corners.min(axis=0)).tolist())
    graph = dict(schema=1, kind='navigation', state='ready' if nodes else 'insufficient_free_space',
                 reason='' if nodes else '已知空闲区域不足以容纳当前机器人净空，请继续完善地图',
                 nodes=nodes, edges=edges, components=components, bounds=bounds, grid_bounds=grid_bounds,
                 required_clearance_m=config.clearance, robot_radius_m=config.robot_radius,
                 safety_margin_m=config.safety_margin, observed_free_cells=int(free.sum()),
                 traversable_cells=int(safe.sum()), unknown_cells=int((grid < 0).sum()),
                 max_observed_clearance_m=float(clearance.max()), resolution=resolution,
                 origin=origin.tolist(), yaw=yaw, semantic_confirmed=False)
    return graph, labels, clearance


def connected_components(nodes, edges):
    adjacency = {n['id']: set() for n in nodes}
    for e in edges:
        adjacency[e['source']].add(e['target'])
        adjacency[e['target']].add(e['source'])
    groups = []
    unseen = set(adjacency)
    while unseen:
        queue, group = [min(unseen)], []
        while queue:
            key = queue.pop()
            if key not in unseen:
                continue
            unseen.remove(key)
            group.append(key)
            queue.extend(sorted(adjacency[key]))
        groups.append(sorted(group))
    return groups


def shortest_route(graph, start, goal):
    """Plan on one graph snapshot only; returns None for disconnected regions."""
    adjacency = {n['id']: [] for n in graph['nodes']}
    if start not in adjacency or goal not in adjacency:
        raise ValueError('start and goal must be nodes in this graph revision')
    for edge in graph['edges']:
        adjacency[edge['source']].append((edge['target'], edge['length_m']))
        adjacency[edge['target']].append((edge['source'], edge['length_m']))
    queue, best = [(0., start, [start])], {start: 0.}
    while queue:
        cost, key, path = heapq.heappop(queue)
        if cost != best[key]:
            continue
        if key == goal:
            return dict(nodes=path, length_m=cost)
        for other, distance in adjacency[key]:
            updated = cost + distance
            if updated < best.get(other, float('inf')):
                best[other] = updated
                heapq.heappush(queue, (updated, other, path + [other]))
    return None
