#!/usr/bin/env python3
"""Benchmark 3001 research inference with saved hardware inputs and real weights."""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

os.environ.setdefault('CUMM_CUDA_ARCH_LIST', '8.7')
os.environ.setdefault('CUDA_HOME', '/usr/local/cuda')
os.environ['PATH'] = str(Path(os.environ['CUDA_HOME']) / 'bin') + os.pathsep + os.environ.get('PATH', '')
ROOT = Path(__file__).resolve().parents[1]
CASES = ('rgbnir', 'ptv3', 'odin_compare', 'avs')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def inputs_for(case, cfg):
    import numpy as np
    from PIL import Image
    public = ROOT / '.tools/miti-research/root/usr/lib/miti/public/research'
    if case == 'rgbnir':
        sample = ROOT / 'logs/segmentation_fusion/capture_20261116_204011/samples/046fe947e53a74ecf307'
        metadata = json.loads((sample / 'metadata.json').read_text())
        with np.load(sample / 'arrays.npz', allow_pickle=False) as archive:
            depth = archive['depth_m'].copy()
        rgb = np.asarray(Image.open(sample / 'rgb.png').convert('RGB'))
        nir = np.asarray(Image.open(sample / 'nir.png').convert('L'))
        from rtabmap_semantic.fusion_geometry import projection, project_image
        u, v, valid = projection(depth, metadata['rgb_info'], metadata['nir_info'],
                                  np.asarray(metadata['nir_from_rgb']))
        registered = project_image(nir, u, v)
        sources = [sample / name for name in ('rgb.png', 'nir.png', 'arrays.npz', 'metadata.json')]
        return (rgb, registered, valid), dict(rgb_shape=list(rgb.shape), network_size=cfg['rgbnir_size'],
                                            valid_nir_fraction=float(valid.mean()), frames_per_result=1), sources
    if case in ('ptv3', 'odin_compare'):
        from rtabmap_research.replay import read_bundle
        from rtabmap_research.point_scene import build_scene
        path = public / 'pmf_features.npz'
        _, _, bundle = read_bundle(path)
        scene = build_scene(bundle['observations'], bundle['matrix'])
        latest = bundle['observations'][-1]
        shape = list(latest['rgb'].shape)
        raw_depth = latest['depth'].astype(np.float32)
        if latest['depth'].dtype == np.uint16:
            raw_depth /= 1000
        valid_depth = np.isfinite(raw_depth) & (raw_depth > .1) & (raw_depth < 10)
        description = dict(rgb_shape=shape, context_views=len(bundle['observations']),
                           lidar_points=len(scene['xyz']), valid_current_depth_points=int(valid_depth.sum()),
                           frames_per_result=1)
        data = (scene['xyz'], scene['rgb'], scene['normals']) if case == 'ptv3' else (bundle,)
        return data, description, [path]
    path = public / 'avs_features.npz'
    with np.load(path, allow_pickle=False) as archive:
        frames = archive['input_frames'].copy()
        mel = archive['input_log_mel'].copy()
    rgb = np.rint(np.clip((frames[-1] * np.asarray([.229, .224, .225], np.float32)[:, None, None]
                          + np.asarray([.485, .456, .406], np.float32)[:, None, None]) * 255, 0, 255)
                   ).astype(np.uint8).transpose(1, 2, 0)
    return (frames, mel, rgb), dict(video_shape=list(frames.shape), log_mel_shape=list(mel.shape),
                                   frames_per_result=10, pcm_available=False), [path]


def run_case(args):
    import numpy as np
    import torch
    sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'src/rtabmap_semantic')]
    assert torch.cuda.is_available(), 'CUDA is required for this benchmark'
    torch.set_num_threads(2)
    cfg = json.loads((ROOT / 'config/fusion_research.json').read_text())
    prepared_at = time.perf_counter()
    data, description, sources = inputs_for(args.case, cfg)
    preparation_s = time.perf_counter() - prepared_at
    loaded_at = time.perf_counter()
    if args.case == 'rgbnir':
        from rtabmap_research.rgbnir import RGBNIR
        model = RGBNIR(ROOT / cfg['rgbnir_weights'], 'cuda:0', size=cfg['rgbnir_size'],
                       confidence=cfg['rgbnir_confidence'], nir_weight=cfg['rgbnir_nir_weight'])
        infer = model.infer
        boundary = 'registered RGB/NIR -> shared backbone fusion + RGB baseline + decoding + tensor traces + display images'
    elif args.case == 'ptv3':
        from rtabmap_research.ptv3 import PointTransformer
        model = PointTransformer(ROOT / cfg['ptv3_weights'], 'cuda:0')
        infer = model.infer_points
        boundary = 'prepared coloured LiDAR/normals -> voxel sampling + PTv3 + original-point labels'
    elif args.case == 'odin_compare':
        from rtabmap_research.odin import ODINComparison
        model = ODINComparison(ROOT / cfg['odin_weights'], 'cuda:0')
        infer = model.infer_bundle
        boundary = 'saved posed RGB-D/LiDAR -> scene preparation + ODIN-Swin + PTv3 comparison + tensor traces + display images'
    else:
        from rtabmap_research.models import AVS
        model = AVS(ROOT / cfg['avs_weights'], 'cuda:0')
        infer = model.infer_preprocessed
        boundary = 'ten normalized genuine video/log-Mel inputs -> AVSegFormer + tensor traces + last-frame display'
    torch.cuda.synchronize()
    load_s = time.perf_counter() - loaded_at
    assert all(not value.requires_grad for value in model.net.parameters())
    assert not model.net.training
    torch.cuda.reset_peak_memory_stats()
    warmups, measured = [], []
    result = None
    for iteration in range(args.warmup + args.iterations):
        torch.random.default_generator.manual_seed(123)
        torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode():
            result = infer(*data)
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - started
        if iteration < args.warmup:
            warmups.append(elapsed)
            print(f'{args.case}: warmup {iteration + 1}/{args.warmup}: {elapsed:.4f}s', flush=True)
        else:
            measured.append(elapsed)
            print(f'{args.case}: sample {len(measured)}/{args.iterations}: {elapsed:.4f}s', flush=True)
    values = np.asarray(measured)
    if args.case == 'ptv3':
        labels, stats = result
        output = dict(label_shape=list(labels.shape), voxels=stats['voxels'], labels_sha256=hashlib.sha256(labels.tobytes()).hexdigest())
    else:
        value, labels = result
        if args.case == 'rgbnir':
            labels = model.trace.arrays['instance_masks']
        output = dict(label_shape=list(labels.shape), output_shape=value.get('output_shape'),
                      detections=value.get('detection_count'), classes=value.get('classes'),
                      labels_sha256=hashlib.sha256(labels.tobytes()).hexdigest())
        for name, payload in value.get('images', {}).items():
            (args.output / f'{args.case}_{name}.jpg').write_bytes(base64.b64decode(payload))
    properties = torch.cuda.get_device_properties(0)
    report = dict(case=args.case, passed=True, source_mode='saved_hardware_input_replay',
                  device='cuda:0', gpu=properties.name, torch=torch.__version__, cuda=torch.version.cuda,
                  precision='float32 tensors, framework default TF32 policy',
                  tf32=dict(matmul=torch.backends.cuda.matmul.allow_tf32, cudnn=torch.backends.cudnn.allow_tf32),
                  cpu_threads=2, measurement_boundary=boundary,
                  excluded=['model load', 'input file read', 'ROS/network transfer', 'NPZ compression', 'web refresh'],
                  accuracy_verified=False, input=description,
                  input_sources=[dict(path=str(path.relative_to(ROOT)), sha256=sha256(path)) for path in sources],
                  config_sha256=sha256(ROOT / 'config/fusion_research.json'), model=model.metadata,
                  input_preparation_s=preparation_s, cold_model_load_s=load_s,
                  warmup_s=warmups, samples_s=measured, iterations=args.iterations,
                  latency_s=dict(mean=float(values.mean()), median=float(np.median(values)),
                                 p95=float(np.percentile(values, 95)), minimum=float(values.min()), maximum=float(values.max())),
                  results_per_second=float(1 / np.median(values)),
                  peak_allocated_mb=torch.cuda.max_memory_allocated() / 2**20,
                  peak_reserved_mb=torch.cuda.max_memory_reserved() / 2**20,
                  output=output)
    (args.output / (args.case + '.json')).write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(case=args.case, latency_s=report['latency_s'], peak_allocated_mb=report['peak_allocated_mb']), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES)
    parser.add_argument('--iterations', type=int, default=10)
    parser.add_argument('--warmup', type=int, default=2)
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/orin_research_benchmark/latest')
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 1:
        parser.error('Use at least one warmup and one measured iteration')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.case:
        run_case(args)
        return
    reports, failures = {}, {}
    for case in CASES:
        print('Benchmarking ' + case, flush=True)
        command = [sys.executable, str(Path(__file__).resolve()), '--case', case, '--iterations', str(args.iterations),
                   '--warmup', str(args.warmup), '--output', str(args.output)]
        with (args.output / (case + '.log')).open('w') as log:
            completed = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        if completed.returncode:
            failures[case] = dict(exit_code=completed.returncode, log=str(args.output / (case + '.log')))
            print('Failed: ' + case, flush=True)
        else:
            reports[case] = json.loads((args.output / (case + '.json')).read_text())
            print(f'{case}: median {reports[case]["latency_s"]["median"]:.4f}s', flush=True)
        summary = dict(passed=not failures, complete=len(reports) + len(failures) == len(CASES),
                       source_mode='saved_hardware_input_replay', models=reports, failures=failures)
        (args.output / 'report.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
