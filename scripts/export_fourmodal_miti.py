#!/usr/bin/env python3
"""Export a saved feature result for MITI's explicitly selected offline view."""
import argparse
import json
import shutil
from pathlib import Path
import sys
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'src/rtabmap_features'))
from rtabmap_features.visualization import feature_snapshot, spectrum_rgb


def export(result, public):
    report = json.loads((result / 'report.json').read_text())
    with np.load(result / 'features.npz', allow_pickle=False) as archive:
        snapshot = feature_snapshot(archive, report, mode='saved')
    public.mkdir(parents=True, exist_ok=True)
    audio_path = report.get('samples', {}).get('audio', {}).get('attributes', {}).get('file')
    if (result / 'spectrum.png').is_file():
        temporary = public / 'rtabmap-fourmodal-audio.tmp.png'
        shutil.copyfile(result / 'spectrum.png', temporary)
        temporary.replace(public / 'rtabmap-fourmodal-audio.png')
        snapshot['spectrum_url'] = '/rtabmap-fourmodal-audio.png'
    elif audio_path and Path(audio_path).is_file():
        import soundfile as sf
        from PIL import Image
        from rtabmap_features.preprocessing import audio_tensor
        audio, rate = sf.read(audio_path, dtype='float32', always_2d=True)
        samples = report['samples']
        offset = samples['rgb'].get('alignment_stamp', samples['rgb']['stamp']) - samples['audio'].get('alignment_stamp', samples['audio']['stamp'])
        spectrum, _ = audio_tensor(audio, rate, offset)
        image = public / 'rtabmap-fourmodal-audio.png'
        temporary = image.with_suffix('.tmp.png')
        Image.fromarray(spectrum_rgb(spectrum.cpu().numpy())).save(temporary)
        temporary.replace(image)
        snapshot['spectrum_url'] = '/rtabmap-fourmodal-audio.png'
    target = public / 'rtabmap-fourmodal-demo.json'
    temporary = target.with_suffix('.tmp.json')
    temporary.write_text(json.dumps(snapshot, ensure_ascii=False, allow_nan=False) + '\n')
    temporary.replace(target)
    print('MITI saved feature view: ' + str(result))
    return snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path)
    parser.add_argument('--result-root', type=Path, help='Export the latest report beneath this run directory')
    parser.add_argument('--public', type=Path, default=PROJECT / '.tools/miti/root/usr/lib/miti/public')
    args = parser.parse_args()
    result = args.result
    if args.result_root:
        candidates = [p.parent for p in args.result_root.rglob('report.json')
                      if (p.parent / 'features.npz').is_file()]
        if not candidates:
            parser.error('No completed features beneath --result-root')
        result = max(candidates, key=lambda p: (p / 'report.json').stat().st_mtime)
    if result is None:
        root = PROJECT / 'logs/fourmodal'
        reports = [*root.glob('demo_*/report.json'), *root.glob('hardware_*/inference/report.json'),
                   *root.glob('hardware_live_*/features/*/report.json')]
        candidates = [p.parent for p in reports if (p.parent / 'features.npz').is_file()]
        if not candidates:
            print('No saved four-modal demo; live MITI view is available')
            return
        result = max(candidates, key=lambda p: (p / 'report.json').stat().st_mtime)
    export(result.resolve(), args.public)


if __name__ == '__main__':
    main()
