"""Project a complete map snapshot onto XY with a measured floor and height band.

Only observed floor cells are free. Unobserved cells are unknown; a global cloud
has no per-point sensor origin, so casting rays from the current pose is invalid.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class GridConfig:
    resolution: float = 0.05
    min_obstacle_height: float = 0.15
    max_obstacle_height: float = 1.6
    ground_tolerance: float = 0.07
    max_ground_tilt: float = 20.0
    min_sensor_height: float = 0.10
    max_sensor_height: float = 1.8
    floor_min_points: int = 100
    floor_min_area: float = 1.0
    noise_radius: float = 0.12
    noise_min_neighbors: int = 3
    max_cells: int = 4000000

    def __post_init__(self):
        positive = ('resolution', 'min_obstacle_height', 'max_obstacle_height', 'ground_tolerance',
                    'max_ground_tilt', 'min_sensor_height', 'max_sensor_height', 'floor_min_area', 'noise_radius')
        if any(not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0 for key in positive):
            raise ValueError('grid dimensions and limits must be finite and positive')
        if not self.ground_tolerance < self.min_obstacle_height < self.max_obstacle_height:
            raise ValueError('ground_tolerance < min_obstacle_height < max_obstacle_height is required')
        if not 0 < self.max_ground_tilt < 45 or self.min_sensor_height >= self.max_sensor_height:
            raise ValueError('invalid ground search bounds')
        if self.floor_min_points < 20 or self.noise_min_neighbors < 1 or not 1 <= self.max_cells <= 16777216:
            raise ValueError('invalid grid processing limits')


def fit_ground(points, reference, config):
    """Find a supported near-horizontal plane below the tracked sensor/reference.

    The reference height check prevents a table/ceiling-only scan becoming free
    floor. Plane coefficients are z = ax + by + c, in the existing map frame.
    """
    rng = np.random.default_rng(42)
    sample = points
    if len(sample) > 12000:
        sample = sample[rng.choice(len(sample), 12000, replace=False)]
    # Ceiling points cannot seed floor hypotheses. The height gate is evaluated
    # at the actual reference XY for tilted maps, not at an assumed z=0 floor.
    seeds = sample[sample[:, 2] <= np.percentile(sample[:, 2], 40)]
    if len(seeds) < config.floor_min_points:
        return None
    best, score = None, 0
    tilt = np.tan(np.deg2rad(config.max_ground_tilt))
    for _ in range(160):
        triangle = seeds[rng.choice(len(seeds), 3, replace=False)]
        design = np.column_stack((triangle[:, :2], np.ones(3)))
        if abs(np.linalg.det(design)) < .03:
            continue
        plane = np.linalg.solve(design, triangle[:, 2])
        if np.linalg.norm(plane[:2]) > tilt:
            continue
        sensor_height = reference[2] - (reference[:2] @ plane[:2] + plane[2])
        if not config.min_sensor_height <= sensor_height <= config.max_sensor_height:
            continue
        heights = sample[:, 2] - (sample[:, 0]*plane[0] + sample[:, 1]*plane[1] + plane[2])
        # Reject elevated surfaces when substantial observed geometry lies below.
        if np.mean(heights < -2*config.ground_tolerance) > .08:
            continue
        count = int((np.abs(heights) <= config.ground_tolerance).sum())
        if count > score:
            best, score = plane, count
    if best is None or score < config.floor_min_points:
        return None
    for _ in range(2):
        heights = sample[:, 2] - (sample[:, 0]*best[0] + sample[:, 1]*best[1] + best[2])
        inliers = sample[np.abs(heights) <= config.ground_tolerance]
        best = np.linalg.lstsq(np.column_stack((inliers[:, :2], np.ones(len(inliers)))), inliers[:, 2], rcond=None)[0]
    heights = sample[:, 2] - (sample[:, 0]*best[0] + sample[:, 1]*best[1] + best[2])
    floor = sample[np.abs(heights) <= config.ground_tolerance]
    sensor_height = reference[2] - (reference[:2] @ best[:2] + best[2])
    area = len(np.unique(np.floor(floor[:, :2] / .20).astype(np.int64), axis=0)) * .20**2
    if (len(floor) < config.floor_min_points or area < config.floor_min_area or
            np.linalg.norm(best[:2]) > tilt or
            not config.min_sensor_height <= sensor_height <= config.max_sensor_height or
            np.mean(heights < -2*config.ground_tolerance) > .08):
        return None
    return best


def project_grid(points, reference, config=GridConfig()):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    reference = np.asarray(reference, dtype=np.float64)
    if reference.shape != (3,) or not np.isfinite(reference).all():
        raise ValueError('a finite map-frame reference position is required')
    if (np.abs(points) > 1e6).any():
        raise ValueError('map coordinates exceed the supported bounds')
    result = dict(data=np.full((1, 1), -1, dtype=np.int8), origin=[0., 0., 0.],
                  state='empty', source_points=len(points), floor=None,
                  ground_points=0, obstacle_points=0, removed_high_points=0, removed_noise_points=0)
    if len(points) < config.floor_min_points:
        return result
    low = np.floor(points[:, :2].min(axis=0) / config.resolution).astype(np.int64) - 1
    high = np.floor(points[:, :2].max(axis=0) / config.resolution).astype(np.int64) + 1
    width, height = map(int, high-low+1)
    if width*height > config.max_cells:
        raise ValueError('projected map exceeds max_cells; increase resolution')
    grid = np.full((height, width), -1, dtype=np.int8)
    result.update(data=grid, origin=[float(low[0]*config.resolution), float(low[1]*config.resolution), 0.],
                  state='waiting_ground')
    plane = fit_ground(points, reference, config)
    if plane is None:
        return result
    heights = points[:, 2] - (points[:, 0]*plane[0] + points[:, 1]*plane[1] + plane[2])
    floor = points[np.abs(heights) <= config.ground_tolerance]
    obstacles = points[(heights >= config.min_obstacle_height) & (heights <= config.max_obstacle_height)]
    before = len(obstacles)
    if len(obstacles) and config.noise_min_neighbors > 1:
        distances, _ = cKDTree(obstacles).query(obstacles, k=config.noise_min_neighbors,
                                               distance_upper_bound=config.noise_radius, workers=1)
        obstacles = obstacles[np.isfinite(distances[:, -1])]
    for cloud, value in ((floor, 0), (obstacles, 100)):
        indices = np.floor(cloud[:, :2] / config.resolution).astype(np.int64) - low
        grid[indices[:, 1], indices[:, 0]] = value
    result.update(state='ready', floor=plane.tolist(), ground_points=len(floor), obstacle_points=len(obstacles),
                  removed_high_points=int((heights > config.max_obstacle_height).sum()),
                  removed_noise_points=before-len(obstacles))
    return result
