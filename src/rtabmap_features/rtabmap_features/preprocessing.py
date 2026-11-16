"""Explicit image, point-cloud and audio preprocessing for the frozen backbone."""
import numpy as np


def image_tensor(image, infrared=False, thermal=False):
    import torch
    from PIL import Image
    from torchvision import transforms
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.size == 0:
        raise ValueError('Images must be nonempty uint8 arrays')
    if infrared:
        if image.ndim != 2:
            raise ValueError('Infrared must be a mono8 image')
        pil = Image.fromarray(image, mode='L').convert('L' if thermal else 'RGB')
    else:
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('RGB input must have shape HxWx3, in RGB order')
        pil = Image.fromarray(image, mode='RGB')
    mean, std = ((0.5,), (0.5,)) if thermal else (
        (0.48145466, 0.4578275, 0.40821073), (0.26862954, 0.26130258, 0.27577711))
    transform = transforms.Compose([
        transforms.Resize(224, interpolation=transforms.InterpolationMode.BICUBIC),
        transforms.CenterCrop(224), transforms.ToTensor(), transforms.Normalize(mean, std)])
    return transform(pil).to(dtype=torch.float32)


def prepare_points(points, count=2048, max_range=40.0, units='m'):
    import torch
    from .point_ops import fps
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] < 3 or not 512 <= count <= 8192:
        raise ValueError('Point cloud must be Nx3 or NxC; requested count must be 512..8192')
    if units not in ('m', 'normalized'):
        raise ValueError('Point units must be m, or normalized for an explicit object smoke test')
    xyz = np.asarray(points[:, :3], dtype=np.float32)
    valid = np.isfinite(xyz).all(axis=1)
    ranges = np.linalg.norm(np.where(np.isfinite(xyz), xyz, 0), axis=1)
    if units == 'm':
        valid &= (ranges > 0.05) & (ranges <= max_range)
    xyz = np.unique(xyz[valid], axis=0)
    if len(xyz) < 512:
        raise ValueError(f'LiDAR has only {len(xyz)} valid distinct points; at least 512 required')
    origin = xyz.mean(axis=0, dtype=np.float64).astype(np.float32)
    centered = xyz - origin
    scale = float(np.linalg.norm(centered, axis=1).max())
    if not np.isfinite(scale) or scale <= 1e-8:
        raise ValueError('Degenerate point cloud')
    candidates = centered / scale
    if len(candidates) > 32768:
        candidates = candidates[np.linspace(0, len(candidates) - 1, 32768, dtype=int)]
    normalized = torch.from_numpy(candidates).unsqueeze(0)
    if len(candidates) >= count:
        normalized = fps(normalized, count)
    else:
        index = torch.arange(count) % len(candidates)
        normalized = normalized[:, index].contiguous()
    metadata = dict(input_count=len(points), valid_distinct_count=len(xyz), output_count=count,
                    input_units=units, origin_xyz=origin.tolist(), radius=scale,
                    origin_xyz_m=origin.tolist() if units == 'm' else None,
                    radius_m=scale if units == 'm' else None,
                    fps_candidate_count=len(candidates),
                    policy='unique_lexicographic_cap32768_unit_sphere_fps_start0_repeat_if_short',
                    source_scope='whole_cloud_or_user_supplied_roi',
                    metric_geometry_preserved_in_metadata=units == 'm')
    return normalized, metadata


def audio_tensor(samples, sample_rate, anchor_offset=None):
    import torch
    import torchaudio
    samples = np.asarray(samples, dtype=np.float32)
    if samples.ndim == 1:
        samples = samples[:, None]
    if samples.ndim != 2 or len(samples) < 1 or not np.isfinite(samples).all():
        raise ValueError('Audio must contain finite float32 PCM samples, shape TxC')
    if not 8000 <= sample_rate <= 192000 or np.max(np.abs(samples)) > 1.00001:
        raise ValueError('Audio requires an 8..192 kHz rate and PCM values in [-1, 1]')
    if len(samples) / sample_rate < 0.1:
        raise ValueError('Audio window must be at least 100 ms')
    channels = samples.shape[1]
    waveform = torch.from_numpy(samples.mean(axis=1).copy()).unsqueeze(0)
    if sample_rate != 16000:
        waveform = torchaudio.functional.resample(waveform, sample_rate, 16000)
    duration = waveform.shape[1] / 16000
    middle = duration / 2 if anchor_offset is None else float(anchor_offset)
    start = max(0.0, min(middle - 1.0, max(duration - 2.0, 0.0)))
    waveform = waveform[:, int(round(start * 16000)):int(round((start + 2) * 16000))].clone()
    waveform -= waveform.mean()
    fbank = torchaudio.compliance.kaldi.fbank(
        waveform, htk_compat=True, sample_frequency=16000, use_energy=False,
        window_type='hanning', num_mel_bins=128, dither=0.0,
        frame_length=25, frame_shift=10)
    fbank = fbank.transpose(0, 1)
    fbank = torch.nn.functional.pad(fbank[:, :204], (0, max(204 - fbank.shape[1], 0)))
    fbank = ((fbank - (-4.268)) / 9.138).unsqueeze(0).unsqueeze(0)
    return fbank, dict(sample_rate_in=sample_rate, sample_rate_model=16000,
                       channels_in=channels, channel_policy='mean_to_mono',
                       crop_start_offset_s=start, crop_duration_s=min(duration, 2.0),
                       spectrum='kaldi_log_mel', spectrum_shape=[1, 128, 204],
                       clip_policy='single_up_to_2s_window_containing_anchor')
