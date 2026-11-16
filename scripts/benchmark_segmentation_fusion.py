#!/usr/bin/env python3
"""Benchmark unchanged fusion on captured inputs and compare saved CPU features."""
import argparse
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/rtabmap_features'),str(ROOT/'src/rtabmap_semantic')]
from rtabmap_features.backbone import FrozenEncoder
from rtabmap_semantic.object_fusion import ObjectFusionEngine,TaskVocabulary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--samples',type=int,default=3)
    parser.add_argument('--output',type=Path,default=ROOT/'logs/segmentation_validation/fusion_gpu_benchmark.json')
    args=parser.parse_args()
    if not 1<=args.samples<=10:raise ValueError('Use 1..10 representative samples')
    records=[(p,json.loads(p.read_text())) for p in args.capture.glob('samples/*/metadata.json')]
    records.sort(key=lambda r:r[1]['sequence'])
    records=records[-args.samples:]
    if len(records)!=args.samples:raise ValueError('Not enough completed captures')
    policy=records[0][1]['policy']
    engine=ObjectFusionEngine(FrozenEncoder(ROOT/'.tools/unibind',ROOT/'models/unibind/pretrained_weights.pt',
        device=args.device,threads=2,retain_text=True),TaskVocabulary(ROOT/'config/segmentation_fusion_task.json'),
        ROOT/'models/unibind/task_prototypes',max_objects=policy['max_objects'],min_coverage=policy['min_coverage'])
    rows=[]
    for path,metadata in records:
        folder=path.parent
        with np.load(folder/'arrays.npz',allow_pickle=False) as saved:
            bundle=dict(rgb=cv2.cvtColor(cv2.imread(str(folder/'rgb.png')),cv2.COLOR_BGR2RGB),
                nir=cv2.imread(str(folder/'nir.png'),cv2.IMREAD_GRAYSCALE),depth_m=saved['depth_m'].copy(),
                rgb_info=metadata['rgb_info'],nir_info=metadata['nir_info'],transform=np.asarray(metadata['nir_from_rgb']),
                native_names={int(k):v for k,v in metadata['native_names'].items()})
            for kind in ('rgb','nir'):
                bundle[kind+'_instances']=[{**row,'mask':saved[row['mask_key']].copy()}
                    for row in metadata['instances'][kind]]
            start=time.monotonic();result=engine.fuse(bundle);elapsed=time.monotonic()-start
            for key in ('rgb_map','nir_map','fused_map','instance_map','eval_mask'):
                np.testing.assert_array_equal(result[key],saved[key],err_msg=key)
            for key in ('features','fused_features','logits'):
                np.testing.assert_allclose(result[key],saved[key],atol=2e-5,rtol=2e-4,err_msg=key)
            assert [r['predicted_task_id'] for r in result['objects']]==[r['predicted_task_id'] for r in metadata['objects']]
            rows.append(dict(sequence=metadata['sequence'],objects=len(result['objects']),
                original_cpu_s=metadata['inference_s'],accelerated_s=elapsed,timings_s=result['timings_s'],
                max_feature_error=float(np.abs(result['features']-saved['features']).max(initial=0)),
                predictions_and_geometry_equal=True))
    original=float(np.median([r['original_cpu_s'] for r in rows]))
    accelerated=float(np.median([r['accelerated_s'] for r in rows]))
    report=dict(passed=True,device=args.device,precision='float32',real_captured_inputs=True,
        accuracy_verified=False,median_original_cpu_s=original,median_accelerated_s=accelerated,
        median_speedup=original/accelerated,samples=rows)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
