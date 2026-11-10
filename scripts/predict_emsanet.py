#!/usr/bin/env python3
"""Offline RGB-D instance inference with the same backend used by ROS."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR', str(ROOT / 'models/.matplotlib-emsanet'))
sys.path.insert(0, str(ROOT / 'src/rtabmap_semantic'))
from rtabmap_semantic.emsanet_detector import EmsanetDetector, EMSANET_FUSION_MODES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rgb', type=Path, default=ROOT / '.tools/emsanet/samples/sample_rgb.png')
    parser.add_argument('--depth', type=Path, default=ROOT / '.tools/emsanet/samples/sample_depth.png',
                        help='Registered uint16 PNG in mm, or float32 .npy in metres')
    parser.add_argument('--model', type=Path, default=ROOT / 'models/emsanet/nyuv2_r34_NBt1D_pre.pth')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--fusion', choices=EMSANET_FUSION_MODES, default='se-add-uni-rgb')
    parser.add_argument('--confidence', type=float, default=0.35)
    parser.add_argument('--image-size', type=int, default=640)
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/emsanet/sample')
    args = parser.parse_args()
    rgb = cv2.imread(str(args.rgb), cv2.IMREAD_COLOR)
    depth = np.load(args.depth, allow_pickle=False) if args.depth.suffix == '.npy' \
        else cv2.imread(str(args.depth), cv2.IMREAD_UNCHANGED)
    if rgb is None or depth is None:
        parser.error('Unable to load the supplied RGB/depth images')
    started = time.monotonic()
    model = EmsanetDetector(args.model, args.device, args.confidence, args.image_size, fusion=args.fusion)
    load_seconds = time.monotonic() - started
    started = time.monotonic()
    detections = model.detect(rgb, depth)
    elapsed = time.monotonic() - started
    args.output.mkdir(parents=True, exist_ok=True)
    overlay = rgb.copy()
    records, masks = [], {}
    for index, detection in enumerate(detections, 1):
        mask = detection['mask'].astype(bool)
        y, x = np.nonzero(mask)
        color = np.array([(index * 67) % 205 + 50, (index * 113) % 205 + 50,
                          (index * 149) % 205 + 50], dtype=np.uint8)
        overlay[mask] = (overlay[mask] * 0.5 + color * 0.5).astype(np.uint8)
        cv2.putText(overlay, f"{index}: {detection['label']} {detection['confidence']:.2f}",
                    (int(x.min()), max(16, int(y.min()))), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    tuple(map(int, color)), 1, cv2.LINE_AA)
        records.append(dict(instance_id=index, label=detection['label'],
                            confidence=detection['confidence'], pixels=int(mask.sum())))
        masks[f'instance_{index}'] = detection['mask']
    cv2.imwrite(str(args.output / 'instances.png'), overlay)
    np.savez_compressed(args.output / 'masks.npz', **masks)
    report = dict(backend='emsanet', fusion=model.fusion_name, model_timing=model.last_timing,
                  modalities=list(model.model.args.input_modalities),
                  rgb=str(args.rgb), depth=str(args.depth), depth_encoding=str(depth.dtype),
                  source_shape=list(rgb.shape), model=str(args.model), device=args.device,
                  model_load_seconds=load_seconds, inference_seconds=elapsed, instances=records)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
