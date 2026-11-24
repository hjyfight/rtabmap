"""Timestamp pairing and calibrated, occlusion-aware PMF input projection."""
from collections import deque
import json
from pathlib import Path
import numpy as np


class NotReady(ValueError):
    pass


def measured_transform(path):
    cfg = json.loads(Path(path).read_text())
    if cfg.get('status') != 'measured' or not cfg.get('evidence'):
        raise NotReady('待标定：需要实测 camera_from_lidar 外参与标定记录')
    matrix = np.asarray(cfg.get('camera_from_lidar'), dtype=np.float64)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1]) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-3) or
            not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-3)):
        raise ValueError('camera_from_lidar 必须是有效的刚体变换矩阵')
    if not cfg.get('camera_frame') or not cfg.get('lidar_frame'):
        raise ValueError('标定需要 camera_frame 和 lidar_frame')
    scale = cfg.get('intensity_scale')
    if scale is None or not np.isfinite(scale) or scale <= 0:
        raise NotReady('需要确认 Livox reflectivity 到模型 intensity 的转换比例')
    return matrix, cfg


def estimated_transform(path):
    cfg=json.loads(Path(path).read_text())
    validation=cfg.get('validation',{})
    old,new=validation.get('old_common',{}),validation.get('estimated_common',{})
    if (cfg.get('status')!='auto_estimated' or cfg.get('geometry_verified') is not False or
            not cfg.get('evidence') or not validation.get('improvement_observed') or
            new.get('points',0)<100 or new.get('median_m',float('inf'))>.25 or
            new.get('within_20cm',0)<.5 or new.get('median_m',float('inf'))>=old.get('median_m',0)*.5):
        raise NotReady('自动外参估计尚未通过独立帧复核')
    # Validate the estimate while retaining its explicitly unverified status.
    matrix=np.asarray(cfg['camera_from_lidar'],dtype=np.float64)
    if (matrix.shape!=(4,4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3],[0,0,0,1]) or
            not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-3) or
            not np.isclose(np.linalg.det(matrix[:3,:3]),1,atol=1e-3)):
        raise ValueError('自动估计外参不是有效刚体矩阵')
    scale=cfg.get('intensity_scale')
    if (not cfg.get('camera_frame') or not cfg.get('lidar_frame') or scale is None or
            not np.isfinite(scale) or scale<=0):
        raise NotReady('自动外参估计缺少输入坐标系或强度转换记录')
    return matrix,cfg


def tf_preview_transform(translation, quaternion, camera_frame, lidar_frame, scale, evidence):
    """A runtime TF is usable for an experiment, never evidence of calibration."""
    p, q = np.asarray(translation, dtype=float), np.asarray(quaternion, dtype=float)
    if (p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or
            not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1, atol=1e-3)):
        raise ValueError('无效的相机/LiDAR TF')
    if not camera_frame or not lidar_frame or not evidence or scale is None or not np.isfinite(scale) or scale <= 0:
        raise NotReady('实验融合需要明确坐标系及已核对的强度转换')
    x, y, z, w = q / np.linalg.norm(q)
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = p
    return matrix, dict(status='tf_preview', camera_frame=camera_frame, lidar_frame=lidar_frame,
                        intensity_scale=scale, intensity_evidence=evidence,
                        evidence='当前SLAM TF；安装外参尚未实测标定', geometry_verified=False)


def pair_rgb_lidar(frames, clouds, tolerance, now, max_age):
    """Use the newest complete pair when transport delivers LiDAR after RGB."""
    if not frames or not clouds:
        raise NotReady('等待RGB与原始LiDAR')
    nearest_gap = None
    for frame in reversed(frames):
        if not -.5 <= now-frame[0] <= max_age:
            continue
        cloud = min(clouds, key=lambda c: abs(c[0]-frame[0]))
        gap = abs(cloud[0]-frame[0])
        nearest_gap = gap if nearest_gap is None else min(nearest_gap, gap)
        if gap <= tolerance and -.5 <= now-cloud[0] <= max_age:
            return frame, cloud, gap
    if nearest_gap is None:
        raise NotReady('RGB输入过期或采集时间异常；请检查设备时钟')
    raise NotReady(f'RGB/LiDAR未同步：偏差 {nearest_gap:.3f}s')


def project_pmf(points, rgb, intrinsic, distortion, camera_from_lidar, intensity_scale):
    """Project raw XYZ/intensity; keep the closest camera-depth sample per pixel.

    Point features remain in the LiDAR coordinate frame as in SemanticKITTI.
    Right/bottom padding never changes the camera intrinsics.
    """
    import cv2
    points = np.asarray(points, np.float32)
    if points.ndim != 2 or points.shape[1] != 4:
        raise ValueError('PMF requires raw x,y,z,intensity, not a coloured map cloud')
    h, w = rgb.shape[:2]
    k = np.asarray(intrinsic, np.float64).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError('Invalid RGB CameraInfo intrinsics')
    valid = np.isfinite(points).all(1) & (np.linalg.norm(points[:, :3], axis=1) > .05)
    indices = np.flatnonzero(valid)
    xyz = points[indices, :3]
    camera_xyz = xyz @ camera_from_lidar[:3, :3].T + camera_from_lidar[:3, 3]
    front = camera_xyz[:, 2] > .05
    indices, camera_xyz = indices[front], camera_xyz[front]
    if not len(indices):
        raise NotReady('点云未落入相机前方；请检查外参与视野')
    pixels, _ = cv2.projectPoints(camera_xyz, np.zeros(3), np.zeros(3), k,
                                  np.asarray(distortion, np.float64) if len(distortion) else None)
    uv = np.floor(pixels.reshape(-1, 2)).astype(np.int64)
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    indices, uv, camera_xyz = indices[inside], uv[inside], camera_xyz[inside]
    order = np.argsort(camera_xyz[:, 2], kind='stable')
    flat = uv[order, 1] * w + uv[order, 0]
    _, first = np.unique(flat, return_index=True)
    chosen = order[first]
    indices, uv = indices[chosen], uv[chosen]
    if len(indices) < 32:
        raise NotReady('相机视野内有效投影点不足32个')
    ph, pw = (h + 15) // 16 * 16, (w + 15) // 16 * 16
    sparse = np.zeros((5, ph, pw), np.float32)
    mask = np.zeros((ph, pw), bool)
    xyz = points[indices, :3]
    raw = np.column_stack([np.linalg.norm(xyz, axis=1), xyz,
                           np.clip(points[indices, 3] * intensity_scale, 0, 1)])
    mean = np.asarray([12.12, 10.88, .23, -1.04, .21], np.float32)
    std = np.asarray([12.32, 11.47, 6.91, .86, .16], np.float32)
    sparse[:, uv[:, 1], uv[:, 0]] = ((raw - mean) / std).T
    mask[uv[:, 1], uv[:, 0]] = True
    image = np.zeros((3, ph, pw), np.float32)
    image[:, :h, :w] = rgb.transpose(2, 0, 1) / 255.
    return sparse, image, mask, uv, indices


class AudioTimeline:
    """Reconstruct overlapping PCM windows on their original sample timeline.

    The coverage mask rejects gaps; no repeats or zero-filled missing audio are
    used as valid input. Estimated ADC timestamps remain explicitly estimated.
    """
    def __init__(self, max_seconds=14):
        # Go2 publishes 0.5s windows every 0.1s. A 64-window cap retains only
        # 6.8s, so the required 10s AVS input could never become complete.
        # Keep a bounded buffer and prune by the actual sample timeline below.
        self.windows = deque(maxlen=512)
        self.rate = None
        self.max_seconds = max_seconds
        self.last_source = None

    def add(self, start, pcm, rate, source, quality):
        pcm = np.asarray(pcm, np.float32)
        if not np.isfinite(start) or not 8000 <= rate <= 192000 or not len(pcm):
            raise ValueError('Invalid audio timestamp/rate')
        if pcm.ndim != 1 or not np.isfinite(pcm).all() or len(pcm) > rate * 10:
            raise ValueError('Invalid PCM samples')
        if source == self.last_source:
            return
        end = start + len(pcm) / rate
        if self.rate != rate or (self.windows and end < self.windows[-1][1] - .5):
            self.windows.clear()
        self.rate, self.last_source = rate, source
        self.windows.append((start, end, pcm, quality))
        while self.windows and self.windows[0][1] < end - self.max_seconds:
            self.windows.popleft()

    def extract(self, seconds=10, *, end=None):
        if not self.windows:
            raise NotReady('等待麦克风PCM')
        if end is None:
            end = self.windows[-1][1]
        elif not np.isfinite(end) or end > self.windows[-1][1]:
            raise NotReady('音频尚未覆盖请求的采集时间')
        start, count = end - seconds, int(round(seconds * self.rate))
        samples, covered = np.zeros(count, np.float32), np.zeros(count, bool)
        for t, _, pcm, _ in self.windows:
            offset = int(round((t - start) * self.rate))
            a, b = max(0, offset), min(count, offset + len(pcm))
            if b > a:
                samples[a:b] = pcm[a-offset:b-offset]
                covered[a:b] = True
        if not covered.all():
            raise NotReady(f'等待连续10秒音频：当前覆盖 {covered.mean()*100:.1f}%')
        return samples, self.rate, start, end, self.windows[-1][3]


def pair_frames(frames, times, tolerance):
    if not frames:
        raise NotReady('等待RGB图像')
    selected, gaps = [], []
    for stamp in times:
        row = min(frames, key=lambda x: abs(x[0] - stamp))
        gap = abs(row[0] - stamp)
        if gap > tolerance:
            raise NotReady(f'音视频时间未配对：最近帧偏差 {gap:.3f}s')
        selected.append(row)
        gaps.append(gap)
    if len({x[0] for x in selected}) != len(times):
        raise NotReady('RGB帧不足，拒绝重复图像充当10帧视频')
    return selected, gaps
