#!/usr/bin/env python3
"""Recognize an image without prompts; stdout contains only unique category names."""
import argparse
from contextlib import redirect_stdout
import os
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_semantic'))
os.environ.setdefault('YOLO_CONFIG_DIR', str(PROJECT / 'models/.ultralytics-yoloe'))
os.environ.setdefault('MPLCONFIGDIR', str(PROJECT / 'models/.matplotlib-yoloe'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--model', type=Path, default=PROJECT / 'models/yoloe-11s-seg-pf.pt')
    parser.add_argument('--confidence', type=float, default=0.45)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    with redirect_stdout(sys.stderr):
        import cv2
        from rtabmap_semantic.detector import YoloDetector
        from rtabmap_semantic.labels import chinese_label
        image = cv2.imread(str(args.image))
        if image is None:
            parser.error('Cannot read image: ' + str(args.image))
        detector = YoloDetector(args.model, args.device, args.confidence, backend='yoloe_pf')
        labels = sorted({chinese_label(item['label']) for item in detector.detect(image)})
    for label in labels:
        print(label)


if __name__ == '__main__':
    main()
