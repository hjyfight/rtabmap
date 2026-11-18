#!/usr/bin/env python3
"""Check the actual model or recognize a local WAV before starting ROS."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/rtabmap_features'))
from rtabmap_features.yamnet import YamnetClassifier, PatrolEventTracker, make_snapshot, read_wav


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--wav', type=Path)
    inputs.add_argument('--check', action='store_true')
    parser.add_argument('--model', type=Path, default=ROOT / 'models/yamnet/yamnet.tflite')
    parser.add_argument('--class-map', type=Path, default=ROOT / 'config/yamnet_class_map.csv')
    parser.add_argument('--patrol-config', type=Path, default=ROOT / 'config/yamnet_patrol.json')
    parser.add_argument('--cpu-threads', type=int, default=2)
    parser.add_argument('--window-seconds', type=float, default=2.0)
    parser.add_argument('--hop-seconds', type=float, default=.48)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not .975 <= args.window_seconds <= 10 or not .05 <= args.hop_seconds <= args.window_seconds:
        parser.error('Require .975 <= window-seconds <= 10 and .05 <= hop-seconds <= window-seconds')
    model = YamnetClassifier(args.model, args.class_map, args.cpu_threads)
    tracker = PatrolEventTracker(json.loads(args.patrol_config.read_text()), model.classes)
    if args.check:
        result = model.classify(np.zeros(15600, dtype=np.float32), 16000)
        output = dict(check_passed=True, model=result['model'], top_k=result['top_k'],
                      processing_s=result['processing_s'], output_classes=len(result['scores']))
    else:
        rate, samples = read_wav(args.wav)
        if not len(samples):
            parser.error('WAV is empty')
        window, hop = int(round(rate * args.window_seconds)), int(round(rate * args.hop_seconds))
        final_start = max(0, len(samples) - window)
        starts = list(range(0, final_start + 1, hop))
        if starts[-1] != final_start:
            starts.append(final_start)
        windows, all_scores = [], []
        latest = None
        for sequence, start in enumerate(starts, 1):
            chunk = samples[start:start + window]
            result = model.classify(chunk, rate)
            stamp = (start + len(chunk)) / rate
            events = tracker.update(result['scores'], stamp, result['audio']['rms_dbfs'])
            source = dict(source_id=str(args.wav.resolve()), frame_id='wav_file',
                          stamp_quality='file_relative_time', raw_stamp=start / rate,
                          recording_duration_s=len(chunk) / rate, analysis_start_stamp=start / rate, clock_offset_s=0.0)
            latest = make_snapshot(result, sequence=sequence, stamp=stamp, source=source,
                                   patrol_events=events, mode='offline')
            all_scores.append(result['scores'])
            windows.append(dict(start_s=start / rate, end_s=stamp, top_k=result['top_k'],
                                patrol_events=events, processing_s=result['processing_s'], audio=result['audio']))
        output = dict(schema='yamnet_wav_report/1', created_at=time.time(),
                      file=str(args.wav.resolve()), duration_s=len(samples) / rate,
                      model=model.model, top_k=model.ranking(np.mean(all_scores, axis=0)),
                      windows=windows, latest_snapshot=latest)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(output, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
    print(json.dumps({key: value for key, value in output.items() if key not in ('windows', 'latest_snapshot')},
                     ensure_ascii=False, allow_nan=False, indent=2))


if __name__ == '__main__':
    main()
