import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts')]
from rtabmap_research import annotations
from research_camera_tf import lidar_from_camera_link


def test_font_fallback_preserves_latin_digits_and_chinese(monkeypatch):
    monkeypatch.setattr(annotations, 'FONT_PATHS', (
        '/missing/NotoSansCJK-Medium.ttc',
        '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'))
    annotations.font.cache_clear()
    try:
        selected = annotations.font(14)
        missing = selected.getmask('\U0010ffff')
        for char in 'RGB123+·墙模型预测':
            mask = selected.getmask(char)
            assert (mask.size, bytes(mask)) != (missing.size, bytes(missing))
    finally:
        annotations.font.cache_clear()


def test_factory_camera_transform_and_calibration_share_lidar_geometry():
    camera_from_lidar = np.eye(4)
    camera_from_lidar[:3, :3] = Rotation.from_euler('xyz', [1.5, .02, -3.]).as_matrix()
    camera_from_lidar[:3, 3] = [.03, -.02, -.11]
    camera_from_link = np.eye(4)
    camera_from_link[:3, :3] = Rotation.from_euler('xyz', [-1.57, 0., -1.57]).as_matrix()
    camera_from_link[:3, 3] = [.001, .02, 0.]
    mount = lidar_from_camera_link(camera_from_lidar, camera_from_link)
    link_points = np.array([[0., 1., 2., 1.], [1., 0., 3., 1.]]).T
    np.testing.assert_allclose(camera_from_lidar @ mount @ link_points,
                               camera_from_link @ link_points, atol=1e-12)
    np.testing.assert_allclose(mount[:3, :3].T @ mount[:3, :3], np.eye(3), atol=1e-12)
    assert np.linalg.det(mount[:3, :3]) > .999
