#!/usr/bin/env python3
"""Extract frozen RGB/infrared/LiDAR/audio features from an explicit sensor bundle."""
import argparse
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_features'))


def main():
    from rtabmap_features.backbone import FrozenEncoder
    from rtabmap_features.bundle import load_bundle, extract_bundle, save_features
    from rtabmap_features.fusion import MODALITIES
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--source', type=Path, default=PROJECT / '.tools/unibind')
    p.add_argument('--checkpoint', type=Path, default=PROJECT / 'models/unibind/pretrained_weights.pt')
    p.add_argument('--device', default='cpu')
    p.add_argument('--threads', type=int, default=2)
    p.add_argument('--point-count', type=int, default=2048)
    p.add_argument('--infrared-kind', choices=('nir', 'thermal'), default='nir')
    p.add_argument('--weights', type=float, nargs=4, default=[1, 1, 1, 1],
                   metavar=('RGB', 'IR', 'LIDAR', 'AUDIO'))
    p.add_argument('--allow-partial', action='store_true')
    p.add_argument('--allow-unaligned', action='store_true',
                   help='Explicitly permit unpaired historical files for a smoke test')
    args = p.parse_args()
    if args.output.exists():
        p.error('Output already exists; choose a new directory')
    samples, manifest = load_bundle(args.manifest, allow_unaligned=args.allow_unaligned)
    if not args.allow_partial and len(samples) != 4:
        p.error('Four modalities are required; use --allow-partial only for explicit degraded operation')
    print('Loading verified pretrained backbone on ' + args.device, flush=True)
    encoder = FrozenEncoder(args.source, args.checkpoint, args.device, args.infrared_kind, args.threads,
                            args.point_count)
    output, report = extract_bundle(encoder, samples, manifest,
                                    dict(zip(MODALITIES, args.weights)), args.allow_partial)
    save_features(args.output, output, report)
    print(json.dumps(dict(output=str(args.output), complete=report['complete'],
                          independent_shape=list(output['independent'].shape),
                          joint_dim=len(output['concat']), fused_dim=len(output['fused']),
                          inference_s=report['total_inference_s']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
