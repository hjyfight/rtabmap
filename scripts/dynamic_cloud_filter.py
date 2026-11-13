"""Bounded, display-only ray evidence; existing map snapshots remain authoritative."""
import ctypes as ct
import math
from pathlib import Path

import numpy as np


def xyz_array(points):
    result = np.asarray(points, dtype=np.float32)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError('Expected Nx3 points')
    return np.ascontiguousarray(result)


class DynamicCloudFilter:
    def __init__(self, resolution=.1, confirmations=3, max_range=12., endpoint_margin=.2, evidence_window=5.):
        if not all(math.isfinite(v) and v > 0 for v in (resolution, max_range, endpoint_margin, evidence_window)) or not 1 <= confirmations <= 20:
            raise ValueError('Invalid ray filter parameters')
        if resolution < .02 or resolution > .5 or endpoint_margin < resolution or max_range <= endpoint_margin:
            raise ValueError('Invalid voxel size, endpoint margin or range')
        path = Path(__file__).resolve().parents[1]/'build/dynamic_cloud/libdynamic_cloud.so'
        self.lib = lib = ct.CDLL(str(path))
        fptr = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
        lib.dc_create.argtypes = [ct.c_double, ct.c_uint, ct.c_double, ct.c_double, ct.c_double]
        lib.dc_create.restype = ct.c_void_p
        lib.dc_destroy.argtypes = lib.dc_reset.argtypes = [ct.c_void_p]
        lib.dc_map.argtypes = [ct.c_void_p, fptr, ct.c_size_t]
        lib.dc_scan.argtypes = [ct.c_void_p, fptr, ct.c_size_t, fptr, ct.c_double,
                               np.ctypeslib.ndpointer(dtype=np.uintp, flags='C_CONTIGUOUS')]
        lib.dc_mask.argtypes = [ct.c_void_p, fptr, ct.c_size_t,
                               np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS')]
        lib.dc_error.restype = ct.c_char_p
        self.handle = lib.dc_create(resolution, confirmations, max_range, endpoint_margin, evidence_window)
        if not self.handle:
            raise RuntimeError(lib.dc_error().decode())
        self.points = np.empty((0, 3), dtype=np.float32)

    def check(self, result):
        if result < 0:
            raise RuntimeError(self.lib.dc_error().decode())
        return result

    def close(self):
        if self.handle:
            self.lib.dc_destroy(self.handle)
            self.handle = None

    def reset(self):
        self.lib.dc_reset(self.handle)

    def set_map(self, points):
        points = xyz_array(points)
        self.check(self.lib.dc_map(self.handle, points, len(points)))
        self.points = points

    def scan(self, points, origin, stamp):
        points = xyz_array(points)
        origin = np.ascontiguousarray(origin, dtype=np.float32)
        if origin.shape != (3,) or not np.isfinite(origin).all() or not math.isfinite(stamp):
            raise ValueError('Invalid sensor origin or acquisition time')
        stats = np.zeros(3, dtype=np.uintp)
        accepted = self.check(self.lib.dc_scan(self.handle, points, len(points), origin, stamp, stats))
        return dict(accepted=bool(accepted), rays=int(stats[0]), contradicted_voxels=int(stats[1]), cleared_voxels=int(stats[2]))

    def filtered_map(self):
        mask = np.empty(len(self.points), dtype=np.uint8)
        self.check(self.lib.dc_mask(self.handle, self.points, len(self.points), mask))
        return self.points[mask.astype(bool)]
