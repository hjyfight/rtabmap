"""Offline sensor bundles and self-describing feature artifacts."""
import json
from pathlib import Path
import time
import numpy as np
from .fusion import MODALITIES, Sample, acquisition_gap, fuse_features


def read_points(path):
    path = Path(path)
    if path.suffix == '.npy':
        return np.load(path, allow_pickle=False)
    if path.suffix == '.npz':
        with np.load(path, allow_pickle=False) as archive:
            return archive['points'].copy()
    if path.suffix == '.pt':
        import torch
        tensor = torch.load(path, map_location='cpu', weights_only=True)
        if not isinstance(tensor, torch.Tensor):
            raise ValueError('Point .pt files must contain one tensor')
        return tensor.numpy()
    if path.suffix == '.xyz':
        return np.loadtxt(path, dtype=np.float32)
    if path.suffix != '.pcd':
        raise ValueError('Supported point files: .npy, .npz (points), .pt, .xyz, .pcd')
    with path.open('rb') as stream:
        header = {}
        for _ in range(100):
            line = stream.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key, *values = line.split()
                header[key.upper()] = values
                if key.upper() == 'DATA':
                    break
        else:
            raise ValueError('Invalid PCD header')
        fields = header.get('FIELDS', header.get('FIELD', []))
        if not {'x', 'y', 'z'} <= set(fields):
            raise ValueError('PCD requires x/y/z fields')
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if header['DATA'] == ['ascii']:
            matrix = np.loadtxt(stream, dtype=np.float32, ndmin=2)
            offsets = np.cumsum([0] + counts[:-1])
            return matrix[:, [offsets[fields.index(k)] for k in ('x', 'y', 'z')]]
        if header['DATA'] != ['binary']:
            raise ValueError('Compressed PCD is unsupported; export binary/ascii PCD or NPY')
        types = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([(name, '<' + types[kind] + size, (count,))
                          for name, kind, size, count in zip(fields, header['TYPE'], header['SIZE'], counts)])
        payload = stream.read()
        if len(payload) % dtype.itemsize:
            raise ValueError('Truncated binary PCD')
        records = np.frombuffer(payload, dtype=dtype)
        return np.column_stack([records[k][:, 0] for k in ('x', 'y', 'z')]).astype(np.float32)


def load_bundle(path, allow_unaligned=False):
    from PIL import Image
    import soundfile as sf
    path = Path(path).resolve()
    manifest = json.loads(path.read_text())
    if manifest.get('format_version') != 1 or not manifest.get('calibration_version'):
        raise ValueError('Bundle requires format_version=1 and calibration_version')
    alignment = manifest.get('alignment')
    if alignment not in ('acquisition_timestamps', 'estimated_clock_offsets', 'unpaired_smoke_test'):
        raise ValueError('Bundle must declare its alignment policy')
    if alignment == 'unpaired_smoke_test' and not allow_unaligned:
        raise ValueError('Unpaired historical files require explicit --allow-unaligned')
    if alignment == 'estimated_clock_offsets' and not allow_unaligned:
        raise ValueError('Estimated clock offsets require explicit --allow-unaligned')
    unknown = set(manifest.get('samples', {})) - set(MODALITIES)
    if unknown:
        raise ValueError(f'Unknown sensor inputs: {sorted(unknown)}')
    samples = {}
    for name, info in manifest['samples'].items():
        file = (path.parent / info['path']).resolve()
        attributes = dict(info.get('attributes', {}), file=str(file))
        duration = float(info.get('duration', 0))
        if name != 'audio' and duration != 0:
            raise ValueError('Only audio has a duration window in this bundle interface')
        if name in ('rgb', 'infrared'):
            with Image.open(file) as image:
                payload = np.asarray(image.convert('RGB' if name == 'rgb' else 'L')).copy()
        elif name == 'lidar':
            if attributes.get('units') != 'm' and not (
                    alignment == 'unpaired_smoke_test' and attributes.get('units') == 'normalized'):
                raise ValueError('LiDAR input must explicitly declare attributes.units=m')
            payload = read_points(file)
        else:
            payload, rate = sf.read(file, dtype='float32', always_2d=True)
            actual_duration = len(payload) / rate
            if duration and abs(duration - actual_duration) > 1 / rate:
                raise ValueError('Audio declared duration does not match its PCM data')
            duration = actual_duration
            attributes['sample_rate'] = rate
        offset = float(info.get('clock_offset_s', 0))
        if offset and alignment == 'acquisition_timestamps':
            raise ValueError('Clock offsets must be labeled estimated_clock_offsets')
        samples[name] = Sample(payload, float(info['stamp']), info.get('frame_id', ''),
                               info.get('source_id', str(file)), duration, attributes, offset)
    if 'rgb' not in samples:
        raise ValueError('A RGB anchor sample is required')
    tolerance = float(manifest.get('sync_tolerance_s', 0.05))
    if not np.isfinite(tolerance) or tolerance < 0:
        raise ValueError('Invalid synchronization tolerance')
    if alignment in ('acquisition_timestamps', 'estimated_clock_offsets'):
        bad = [name for name, sample in samples.items()
               if acquisition_gap(name, sample, samples['rgb'].alignment_stamp) > tolerance]
        if bad:
            raise ValueError(f'Inputs exceed acquisition-time tolerance: {bad}')
    return samples, manifest


def extract_bundle(encoder, samples, context, weights=None, allow_partial=False):
    started = time.perf_counter()
    anchor = samples['rgb'].alignment_stamp
    features, preprocessing, timing = encoder.encode(samples, anchor)
    output = fuse_features(features, weights, allow_partial=allow_partial)
    metadata = dict(format_version=1, modality_order=list(MODALITIES), feature_dim=1024,
                    joint_dim=4096, dtype='float32', independent_layout='4x1024_row_major',
                    joint_policy='concat_unit_norm_features_fixed_slots',
                    fused_policy='unit_norm_fixed_weighted_mean',
                    complete=bool(output['complete']), fused_valid=bool(output['fused_valid']),
                    valid_mask=output['valid_mask'].tolist(), weights=output['weights'].tolist(),
                    fusion_scope='global_co_observation_embeddings',
                    geometry_alignment_performed=False,
                    quality_and_health={name: sample.attributes.get('source_record', {}) for name, sample in samples.items()},
                    safety=dict(require_human_review=True, prohibit_automatic_execution=True,
                                restricted_use=['visualization', 'recording', 'offline_analysis']),
                    context=context, provenance=encoder.provenance,
                    samples={name: sample.metadata() for name, sample in samples.items()},
                    preprocessing=preprocessing, encoder_timing_s=timing,
                    total_inference_s=time.perf_counter() - started,
                    time_gaps_s={name: acquisition_gap(name, sample, anchor) for name, sample in samples.items()})
    return output, metadata


def save_features(directory, output, metadata):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(directory / 'features.npz', **output)
    (directory / 'report.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2,
                                                    allow_nan=False) + '\n')
    return directory
