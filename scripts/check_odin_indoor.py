#!/usr/bin/env python3
"""Replay ODIN on archived real RGB-D and Livox; no ROS connection or training."""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from rtabmap_research.odin import ODIN,ODINComparison,NAMES
from rtabmap_research.models import predictions,annotated_predictions
from rtabmap_research.replay import read_bundle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--sparse-only',action='store_true')
    parser.add_argument('--compare',action='store_true')
    parser.add_argument('--backbone',choices=['resnet','swin'],default='resnet')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'logs/research/odin')
    parser.add_argument('--source',type=Path,default=ROOT/'.tools/miti-research/root/usr/lib/miti/public/research/pmf_features.npz')
    args=parser.parse_args();torch.set_num_threads(2)
    directory=args.output_dir;directory.mkdir(parents=True,exist_ok=True)
    arrays,metadata,bundle=read_bundle(args.source);context=metadata.get('context',metadata.get('input'))
    observations=bundle['observations'];calibration=bundle['calibration']
    weights=ROOT/'models'/('odin_scannet_swin_semantic.pth.download' if args.backbone=='swin' else 'odin_scannet_resnet.pth.download')
    model=ODINComparison(weights,args.device,backbone_kind=args.backbone) if args.compare else ODIN(weights,args.device,dense=not args.sparse_only,backbone_kind=args.backbone)
    start=time.perf_counter()
    result,labels=model.infer_bundle(bundle)
    elapsed=time.perf_counter()-start
    uv=model.trace.arrays['projection_uv'];indices=model.trace.arrays['render_indices']
    np.testing.assert_array_equal(labels[uv[:,1],uv[:,0]],model.trace.arrays['native_point_labels'][indices]+1)
    if not args.sparse_only:
        valid=model.trace.arrays['dense_valid'];dense=model.trace.arrays['dense_display_labels']
        assert np.count_nonzero(dense)==np.count_nonzero(valid) and not np.any(dense[~valid])
    original=arrays.get('reference_ptv3_point_labels',arrays.get('point_labels'));current=model.trace.arrays['shared_scannet_labels']
    aligned=model.trace.arrays['point_source_indices'].shape==arrays['point_source_indices'].shape and np.array_equal(
        model.trace.arrays['point_source_indices'],arrays['point_source_indices'])
    report=dict(mode='saved_hardware_input_replay',device=args.device,model=model.metadata,accuracy_verified=False,
        inference_s=elapsed,source=dict(path=str(args.source),sha256=hashlib.sha256(args.source.read_bytes()).hexdigest()),
        input=context,statistics=result['input_statistics'],classes=result['classes'],annotations=result['annotations'],
        projected_points=result['projected_points'],same_target_points_as_ptv3=aligned,
        ptv3_disagreement=float(np.mean(original!=current)) if aligned and original is not None else None,
        note='RGB-D supplies posed context; original Livox supplies queried points. No new parameters or training. No independent ground truth.')
    if 'dense_display_labels' in arrays:
        common=(arrays['dense_display_labels']>0)&(model.trace.arrays['dense_display_labels']>0)
        report['previous_model_dense_disagreement']=float(np.mean(arrays['dense_display_labels'][common]!=model.trace.arrays['dense_display_labels'][common]))
        previous,_=predictions(observations[-1]['rgb'],arrays['dense_display_labels'],['忽略']+NAMES)
        annotated_predictions(previous,observations[-1]['rgb'],arrays['dense_display_labels'],['忽略']+NAMES,task='indoor_rgbd')
        result['images']['previous_model_overlay']=previous['annotated_overlay']
    for name,picture in result['images'].items():(directory/(name+'.jpg')).write_bytes(base64.b64decode(picture))
    report_path=directory/'replay.json';report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    np.savez_compressed(directory/'features.npz',**model.trace.arrays,metadata=np.array(json.dumps(report,ensure_ascii=False)))
    (directory/'snapshot.json').write_text(json.dumps(dict(schema='downstream_fusion/1',session_id='odin-offline',
        source_mode='offline_replay',task='pmf',published_at=time.time(),state='ready',detail='ODIN RGB-D/LiDAR离线回放',
        model=model.metadata,inputs={},ros_error='',result=dict(result,sequence=1,source_stamp=observations[-1]['rgb_stamp'],
        inference_s=elapsed,context=context,accuracy_verified=False)),ensure_ascii=False))
    print(json.dumps({key:report[key] for key in ['statistics','classes','inference_s','ptv3_disagreement']},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
