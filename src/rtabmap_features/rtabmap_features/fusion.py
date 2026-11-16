"""Fixed-layout fusion and acquisition-time matching, with no learned weights."""
from dataclasses import dataclass, field
import math
import numpy as np

MODALITIES = ('rgb', 'infrared', 'lidar', 'audio')
FEATURE_DIM = 1024


@dataclass(frozen=True)
class Sample:
    payload: object
    stamp: float
    frame_id: str
    source_id: str
    duration: float = 0.0
    attributes: dict = field(default_factory=dict)
    clock_offset_s: float = 0.0

    def __post_init__(self):
        if not math.isfinite(self.stamp) or self.stamp < 0:
            raise ValueError('Sample acquisition stamp must be finite and nonnegative')
        if not math.isfinite(self.duration) or self.duration < 0:
            raise ValueError('Sample duration must be finite and nonnegative')
        if not self.source_id:
            raise ValueError('Sample source_id is required')
        if not math.isfinite(self.clock_offset_s) or self.alignment_stamp < 0:
            raise ValueError('Invalid clock alignment offset')

    @property
    def alignment_stamp(self):
        return self.stamp + self.clock_offset_s

    def gap(self, anchor):
        return max(self.alignment_stamp - anchor, anchor - (self.alignment_stamp + self.duration), 0.0)

    def metadata(self):
        return dict(stamp=self.stamp, duration=self.duration, frame_id=self.frame_id,
                    source_id=self.source_id, attributes=self.attributes,
                    alignment_stamp=self.alignment_stamp, clock_offset_s=self.clock_offset_s)


def acquisition_gap(name, sample, anchor):
    return sample.gap(anchor) if name == 'audio' else abs(sample.alignment_stamp - anchor)


def match_samples(buffers, anchor_sample, tolerance=0.05, allow_partial=False):
    if not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Invalid synchronization tolerance')
    selected = {'rgb': anchor_sample}
    for name in MODALITIES[1:]:
        candidates = [s for s in buffers.get(name, ())
                      if acquisition_gap(name, s, anchor_sample.alignment_stamp) <= tolerance]
        if candidates:
            selected[name] = min(candidates, key=lambda s: (
                acquisition_gap(name, s, anchor_sample.alignment_stamp),
                abs(s.alignment_stamp + s.duration / 2 - anchor_sample.alignment_stamp)))
    if not allow_partial and len(selected) != len(MODALITIES):
        return None
    return selected


def fuse_features(features, weights=None, allow_partial=False):
    unknown = set(features) - set(MODALITIES)
    if unknown:
        raise ValueError(f'Unknown modalities: {sorted(unknown)}')
    missing = set(MODALITIES) - set(features)
    if missing and not allow_partial:
        raise ValueError(f'Missing required modalities: {sorted(missing)}')
    if not features:
        raise ValueError('No features to fuse')
    requested = {name: 1.0 for name in MODALITIES}
    if weights is not None:
        if set(weights) - set(MODALITIES):
            raise ValueError('Unknown modality in fusion weights')
        requested.update(weights)
    if any(not math.isfinite(float(w)) or w < 0 for w in requested.values()):
        raise ValueError('Fusion weights must be finite and nonnegative')
    independent = np.zeros((len(MODALITIES), FEATURE_DIM), np.float32)
    encoder_features = np.zeros_like(independent)
    valid = np.zeros(len(MODALITIES), bool)
    effective = np.zeros(len(MODALITIES), np.float32)
    for i, name in enumerate(MODALITIES):
        if name not in features:
            continue
        vector = np.asarray(features[name], dtype=np.float32)
        if vector.shape != (FEATURE_DIM,) or not np.isfinite(vector).all():
            raise ValueError(f'{name} must contain {FEATURE_DIM} finite feature values')
        norm = np.linalg.norm(vector.astype(np.float64))
        if norm <= 1e-12:
            raise ValueError(f'{name} feature has zero norm')
        encoder_features[i] = vector
        independent[i] = vector / norm
        valid[i] = True
        effective[i] = requested[name]
    if effective.sum() <= 0:
        raise ValueError('At least one valid modality must have positive fusion weight')
    effective /= effective.sum()
    mean = (independent * effective[:, None]).sum(axis=0)
    fused_norm = np.linalg.norm(mean.astype(np.float64))
    fused_valid = bool(fused_norm > 1e-12)
    fused = mean / fused_norm if fused_valid else np.zeros(FEATURE_DIM, np.float32)
    similarities = independent @ independent.T
    similarities[~valid, :] = np.nan
    similarities[:, ~valid] = np.nan
    return dict(encoder_features=encoder_features, independent=independent, concat=independent.reshape(-1).copy(),
                fused=fused.astype(np.float32), valid_mask=valid, weights=effective,
                cosine_similarities=similarities, fused_valid=np.asarray(fused_valid),
                complete=np.asarray(valid.all()))
