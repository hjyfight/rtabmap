"""RGB-D geometry, independent of ROS and the inference runtime."""
import cv2
import numpy as np


def rigid_matrix(translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    t = np.asarray(translation, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or not np.isfinite(t).all() or norm < 1e-8:
        raise ValueError('Invalid rigid transform')
    x, y, z, w = q / norm
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = t
    return result


def transform_point(matrix, point):
    return matrix[:3, :3] @ np.asarray(point) + matrix[:3, 3]


def lift_mask(mask, depth, intrinsics, camera_size, camera_to_node,
              distortion=(), distortion_model='plumb_bob',
              min_depth=0.3, max_depth=4.0, min_points=30):
    """Return a robust visible-surface center in the node frame, or None.

    uint16 depth uses the ROS millimetre convention; float32 depth is metres.
    Masks and depth must be registered to the same optical camera/FOV.
    """
    if depth.ndim != 2 or depth.dtype not in (np.uint16, np.float32):
        raise ValueError('Depth must be 16UC1 (mm) or 32FC1 (m)')
    height, width = depth.shape
    cw, ch = camera_size
    if cw <= 0 or ch <= 0 or abs(width / height - cw / ch) > 0.02:
        raise ValueError('Depth and calibration have incompatible aspect ratios')
    k = np.asarray(intrinsics, dtype=float).reshape(3, 3).copy()
    k[0, :] *= width / cw
    k[1, :] *= height / ch
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid camera intrinsics')
    mask = cv2.resize(np.asarray(mask, dtype=np.uint8), (width, height),
                      interpolation=cv2.INTER_NEAREST)
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8)) > 0
    metres = depth.astype(np.float64) * (0.001 if depth.dtype == np.uint16 else 1.0)
    valid = mask & np.isfinite(metres) & (metres >= min_depth) & (metres <= max_depth)
    if np.count_nonzero(valid) < min_points:
        return None
    # Retain the dominant depth cluster, then the largest connected component.
    values = metres[valid]
    bins = np.arange(min_depth, max_depth + 0.10001, 0.1)
    counts, edges = np.histogram(values, bins=bins)
    peak = int(np.argmax(counts))
    lo = hi = peak
    while lo > 0 and counts[lo-1] >= max(2, counts[peak] * 0.1):
        lo -= 1
    while hi+1 < len(counts) and counts[hi+1] >= max(2, counts[peak] * 0.1):
        hi += 1
    valid &= (metres >= edges[lo]) & (metres <= edges[hi+1])
    n, labels, stats, _ = cv2.connectedComponentsWithStats(valid.astype(np.uint8), connectivity=8)
    if n <= 1:
        return None
    valid = labels == (1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA])))
    v, u = np.nonzero(valid)
    if len(u) < min_points:
        return None
    # Bound projection work for large masks without changing geometric scale.
    stride = max(1, len(u) // 5000)
    u, v = u[::stride], v[::stride]
    z = metres[v, u]
    pixels = np.column_stack((u, v)).astype(np.float64).reshape(-1, 1, 2)
    d = np.asarray(distortion, dtype=float)
    if len(d) and np.any(d):
        if distortion_model == 'equidistant':
            rays = cv2.fisheye.undistortPoints(pixels, k, d).reshape(-1, 2)
        elif distortion_model in ('plumb_bob', 'rational_polynomial'):
            rays = cv2.undistortPoints(pixels, k, d).reshape(-1, 2)
        else:
            raise ValueError('Unsupported distortion model: ' + distortion_model)
    else:
        rays = (pixels.reshape(-1, 2) - [k[0, 2], k[1, 2]]) / [k[0, 0], k[1, 1]]
    camera_points = np.column_stack((rays * z[:, None], z))
    points = camera_points @ camera_to_node[:3, :3].T + camera_to_node[:3, 3]
    center = np.median(points, axis=0)
    extent = np.maximum(0.02, np.percentile(points, 95, axis=0) - np.percentile(points, 5, axis=0))
    sigma = max(0.02, float(np.median(np.abs(z - np.median(z)))) * 1.4826)
    return dict(position=center.tolist(), extent=extent.tolist(), sigma=sigma,
                valid_points=len(u))
