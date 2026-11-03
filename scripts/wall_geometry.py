"""Geometry-only wall candidates. Work on each complete map snapshot, never append walls."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class WallConfig:
    voxel_size: float = 0.10
    distance: float = 0.08
    cell_size: float = 0.20
    min_height: float = 1.4
    min_width: float = 0.9
    min_area: float = 1.2
    max_tilt_degrees: float = 15.0
    min_points: int = 60
    max_points: int = 60000
    max_walls: int = 24
    hypotheses: int = 128

    def __post_init__(self):
        for name in ('voxel_size', 'distance', 'cell_size', 'min_height', 'min_width', 'min_area'):
            if not np.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not 0 < self.max_tilt_degrees < 45:
            raise ValueError('max_tilt_degrees must be between 0 and 45')
        if self.min_points < 12 or self.max_points < self.min_points or self.max_walls < 1 or self.hypotheses < 16:
            raise ValueError('invalid wall extraction limits')


def transform_matrix(position, quaternion):
    p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid position or quaternion')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix()
    result[:3, 3] = p
    return result


def pose_values(matrix):
    rotation = matrix[:3, :3]
    # Fixed-axis XYZ: R = Rz(yaw) Ry(pitch) Rx(roll), angles in radians.
    pitch = np.arctan2(-rotation[2, 0], np.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(np.cos(pitch)) > 1e-7:
        roll, yaw = np.arctan2(rotation[2, 1], rotation[2, 2]), np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll, yaw = 0.0, np.arctan2(-rotation[0, 1], rotation[1, 1])
    return matrix[:3, 3].tolist(), Rotation.from_matrix(rotation).as_quat().tolist(), [float(roll), float(pitch), float(yaw)]


def components(cells):
    remaining = set(map(tuple, cells))
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        pending, group = [seed], []
        while pending:
            x, y = pending.pop()
            group.append((x, y))
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                neighbor = (x + dx, y + dy)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    pending.append(neighbor)
        yield np.asarray(group)


def cell_mesh(cells, normal, offset, u, v, size):
    """Tile only observed cells; do not close holes or fill a convex hull over doorways."""
    origin = -offset * normal
    corners = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
    xy = (cells[:, None, :] + corners[None, :, :]) * size
    xyz = origin + xy[:, :, :1] * u + xy[:, :, 1:] * v
    triangles = xyz[:, [0, 1, 2, 0, 2, 3], :].reshape(-1, 3)
    # Boundary ribbons use one triangle batch, avoiding thousands of line draw calls.
    occupied, borders = set(map(tuple, cells)), []
    for x, y in sorted(occupied):
        for delta, a, b, inward in (
                ((0, -1), (x, y), (x+1, y), (0, 0.06)),
                ((1, 0), (x+1, y), (x+1, y+1), (-0.06, 0)),
                ((0, 1), (x+1, y+1), (x, y+1), (0, -0.06)),
                ((-1, 0), (x, y+1), (x, y), (0.06, 0))):
            if (x + delta[0], y + delta[1]) in occupied:
                continue
            a, b, inward = np.asarray(a), np.asarray(b), np.asarray(inward)
            strip = np.array([a, b, b+inward, a, b+inward, a+inward]) * size
            borders.extend(origin + strip[:, :1] * u + strip[:, 1:] * v)
    return triangles, np.asarray(borders).reshape(-1, 3)


def extract_walls(points, config=WallConfig()):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    # Bound malformed coordinates before integer quantization.
    points = points[(np.abs(points) < 1e6).all(axis=1)]
    if len(points) < config.min_points:
        return []
    _, indices = np.unique(np.floor(points / config.voxel_size).astype(np.int64), axis=0, return_index=True)
    points = points[indices]
    rng = np.random.default_rng(42)
    if len(points) > config.max_points:
        points = points[rng.choice(len(points), config.max_points, replace=False)]
    if len(points) < config.min_points:
        return []
    distances, nearest = cKDTree(points).query(points, k=min(16, len(points)), workers=1)
    neighborhoods = points[nearest]
    centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
    covariance = np.einsum('nki,nkj->nij', centered, centered)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normals = eigenvectors[:, :, 0]
    valid = ((np.abs(normals[:, 2]) < np.sin(np.deg2rad(config.max_tilt_degrees))) &
             (eigenvalues[:, 0] / np.maximum(eigenvalues.sum(axis=1), 1e-12) < 0.035) &
             (eigenvalues[:, 1] > 1e-4) & (distances[:, -1] < 0.8))
    points, normals = points[valid], normals[valid]
    walls = []
    for _ in range(config.max_walls):
        if len(points) < config.min_points:
            break
        chosen = rng.choice(len(points), min(config.hypotheses, len(points)), replace=False)
        sample = points[rng.choice(len(points), min(6000, len(points)), replace=False)]
        best_count, best = 0, None
        for begin in range(0, len(chosen), 16):
            take = chosen[begin:begin+16]
            ns = normals[take]
            ds = -np.einsum('ij,ij->i', points[take], ns)
            residual = (sample[:, 0, None]*ns[None, :, 0] + sample[:, 1, None]*ns[None, :, 1] +
                        sample[:, 2, None]*ns[None, :, 2] + ds)
            counts = (np.abs(residual) < config.distance).sum(axis=0)
            index = int(np.argmax(counts))
            if counts[index] > best_count:
                best_count, best = counts[index], (ns[index], ds[index])
        if best is None:
            break
        normal, offset = best
        inliers = np.abs(np.einsum('ij,j->i', points, normal) + offset) < config.distance
        if inliers.sum() < config.min_points:
            break
        fit = points[inliers]
        center = fit.mean(axis=0)
        _, vectors = np.linalg.eigh((fit-center).T @ (fit-center))
        normal = vectors[:, 0]
        offset = -float(center @ normal)
        inliers = np.abs(np.einsum('ij,j->i', points, normal) + offset) < config.distance
        fit = points[inliers]
        points, normals = points[~inliers], normals[~inliers]
        if abs(normal[2]) > np.sin(np.deg2rad(config.max_tilt_degrees)):
            continue
        if normal[np.argmax(np.abs(normal))] < 0:
            normal, offset = -normal, -offset
        u = np.cross([0., 0., 1.], normal)
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        uv = np.column_stack((fit @ u, fit @ v))
        cells = np.unique(np.floor(uv / config.cell_size).astype(np.int64), axis=0)
        for group in sorted(components(cells), key=len, reverse=True):
            width, height = (np.ptp(group, axis=0)+1) * config.cell_size
            area = len(group) * config.cell_size**2
            if width < config.min_width or height < config.min_height or area < config.min_area:
                continue
            triangles, edges = cell_mesh(group, normal, offset, u, v, config.cell_size)
            label = -offset*normal + (group[:, 0].mean()+0.5)*config.cell_size*u + (group[:, 1].max()+1)*config.cell_size*v
            walls.append(dict(normal=normal, offset=offset, triangles=triangles, edges=edges,
                              label=label, area=float(area), width=float(width), height=float(height)))
            if len(walls) >= config.max_walls:
                return walls
    return walls
