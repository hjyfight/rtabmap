"""Standard-message MITI views of frozen features; no model or learned projection."""
import time
import numpy as np
from .fusion import MODALITIES, FEATURE_DIM


def feature_snapshot(output, metadata, *, mode, sequence=0, published_at=None):
    if mode not in ('live', 'saved'):
        raise ValueError('Visualization mode must be live or saved')
    independent = np.asarray(output['independent'], dtype=np.float32)
    encoder_features = np.asarray(output['encoder_features'], dtype=np.float32)
    joint = np.asarray(output['concat'], dtype=np.float32)
    fused = np.asarray(output['fused'], dtype=np.float32)
    valid = np.asarray(output['valid_mask'], dtype=bool)
    weights = np.asarray(output['weights'], dtype=np.float32)
    if (independent.shape != (4, FEATURE_DIM) or encoder_features.shape != independent.shape or
            joint.shape != (4 * FEATURE_DIM,) or fused.shape != (FEATURE_DIM,) or
            valid.shape != (4,) or weights.shape != (4,) or
            not np.isfinite(independent).all() or not np.isfinite(encoder_features).all() or
            not np.isfinite(joint).all() or not np.isfinite(fused).all() or
            not np.isfinite(weights).all()):
        raise ValueError('Invalid feature arrays for visualization')
    similarities = np.clip(independent @ independent.T, -1, 1)
    weighted_mean = (independent * weights[:, None]).sum(axis=0)
    context = metadata.get('context', {})
    provenance = metadata.get('provenance', {})
    samples = metadata.get('samples', {})
    rows = []
    for i, name in enumerate(MODALITIES):
        sample = samples.get(name, {})
        rows.append(dict(name=name, valid=bool(valid[i]), weight=float(weights[i]),
                         stamp=sample.get('stamp'), duration=sample.get('duration', 0),
                         alignment_stamp=sample.get('alignment_stamp', sample.get('stamp')),
                         clock_offset_s=sample.get('clock_offset_s', 0),
                         gap_s=metadata.get('time_gaps_s', {}).get(name),
                         frame_id=sample.get('frame_id', ''),
                         source_id=sample.get('source_id', ''),
                         topic=sample.get('attributes', {}).get('topic', ''),
                         preprocessing=metadata.get('preprocessing', {}).get(name, {}),
                         stamp_quality=sample.get('attributes', {}).get('stamp_quality', ''),
                         encoder=metadata.get('preprocessing', {}).get(name, {}).get('encoder',
                             'point' if name == 'lidar' else 'audio' if name == 'audio' else ''),
                         encoder_features=encoder_features[i].tolist(),
                         encoder_norm=float(np.linalg.norm(encoder_features[i].astype(np.float64))),
                         normalized_norm=float(np.linalg.norm(independent[i].astype(np.float64))),
                         features=independent[i].tolist()))
    return dict(schema=1, flow_version=1, mode=mode, sequence=int(sequence),
                published_at=time.time() if published_at is None else float(published_at),
                alignment=context.get('alignment', 'unknown'),
                calibration_version=context.get('calibration_version', 'unknown'),
                model=provenance.get('backend', 'unknown'),
                model_sha256=provenance.get('checkpoint_sha256', ''),
                device=provenance.get('inference_device', 'unknown'),
                feature_dim=FEATURE_DIM, joint_dim=4 * FEATURE_DIM,
                complete=bool(output['complete']), fused_valid=bool(output['fused_valid']),
                inference_s=metadata.get('total_inference_s', 0),
                modalities=rows, joint_features=joint.tolist(),
                weighted_mean_features=weighted_mean.tolist(),
                weighted_mean_norm=float(np.linalg.norm(weighted_mean.astype(np.float64))),
                fused_features=fused.tolist(),
                fused_norm=float(np.linalg.norm(fused.astype(np.float64))),
                cosine_similarities=[[float(similarities[i, j]) if valid[i] and valid[j] else None
                                      for j in range(4)] for i in range(4)],
                nir_experimental=bool(metadata.get('preprocessing', {}).get('infrared', {}).get(
                    'nir_repeated_to_rgb', False)),
                geometry_alignment_performed=False)


def spectrum_rgb(tensor):
    """Fixed [-2, 2] display range for normalized log-Mel, low Mel bands at bottom."""
    values = np.asarray(tensor, dtype=np.float32).reshape(128, 204)
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite log-Mel spectrum')
    value = np.clip((values[::-1] + 2) / 4, 0, 1)
    stops = np.array([[10, 20, 38], [43, 123, 165], [79, 214, 188], [250, 194, 96]], np.float32)
    position = value * 3
    index = np.minimum(position.astype(int), 2)
    fraction = (position - index)[..., None]
    return np.asarray(stops[index] * (1 - fraction) + stops[index + 1] * fraction, np.uint8)
