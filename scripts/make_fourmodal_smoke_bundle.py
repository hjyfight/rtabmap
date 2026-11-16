#!/usr/bin/env python3
"""Build an explicitly unpaired smoke bundle from installed real/example inputs."""
import argparse
import json
from pathlib import Path
import shutil

PROJECT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    source = PROJECT / '.tools/unibind/assets'
    if args.output.exists():
        p.error('Output already exists')
    image = next(iter(sorted(source.glob('image*'))), None)
    if image is None:
        p.error('Run the installation first; official image asset missing')
    infrared = PROJECT / 'logs/infrared_check_20261104_165340/infra2.png'
    audio = PROJECT / 'logs/audio/m260c_20261113_ly64lvma/input_5s.wav'
    if not infrared.is_file() or not audio.is_file():
        p.error('Historical real NIR/audio files are missing; provide your own bundle manifest')
    args.output.mkdir(parents=True)
    files = dict(rgb=image, infrared=infrared, lidar=source / 'point_car.pt', audio=audio)
    samples = {}
    for name, file in files.items():
        target = name + file.suffix
        shutil.copyfile(file, args.output / target)
        samples[name] = dict(path=target, stamp=0.0, source_id=str(file),
                             frame_id='', attributes={'units': 'normalized',
                                                       'source_kind': 'official_object_point_cloud'}
                             if name == 'lidar' else {})
    manifest = dict(format_version=1, calibration_version='unpaired_not_calibrated',
                    alignment='unpaired_smoke_test', samples=samples,
                    note='Independent historical/example inputs, not synchronized or the same scene. '
                         'Point input is an official object point cloud, not a MID360 scan. '
                         'This validates feature extraction and fusion dataflow only.')
    path = args.output / 'manifest.json'
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(path)


if __name__ == '__main__':
    main()
