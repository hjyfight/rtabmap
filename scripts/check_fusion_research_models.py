#!/usr/bin/env python3
"""Synthetic tensor smoke checks, explicitly separate from live/accuracy tests."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from rtabmap_research.models import PMF,AVS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='cuda:0')
    args=parser.parse_args()
    torch.set_num_threads(2)
    cfg=json.loads((ROOT/'config/fusion_research.json').read_text())
    rng=np.random.default_rng(123)
    rgb=rng.integers(0,256,(64,112,3),dtype=np.uint8)
    image=rgb.transpose(2,0,1).astype(np.float32)/255
    sparse=rng.standard_normal((5,64,112)).astype(np.float32)
    mask=np.ones((64,112),bool)
    uv=np.stack(np.meshgrid(np.arange(112),np.arange(64)),axis=-1).reshape(-1,2)
    report=dict(mode='synthetic_tensor_smoke',accuracy_verified=False,models={})
    for key,cls in [('pmf',PMF),('avs',AVS)]:
        model=cls(ROOT/cfg[key+'_weights'],args.device)
        assert all(not p.requires_grad for p in model.net.parameters()) and not model.net.training
        start=time.perf_counter()
        if key=='pmf':
            result,labels=model.infer(rgb,sparse,image,mask,uv)
            assert result['output_shape']==[1,20,64,112]
            assert len(result['features'])==16
            for i in range(1,5):
                a=model.trace.arrays[f'scale{i}_lidar'];b=model.trace.arrays[f'scale{i}_rgb']
                cat=model.trace.arrays[f'scale{i}_concat']
                np.testing.assert_array_equal(cat,np.concatenate([a,b],axis=1))
            camera_labels=model.trace.arrays['rgb_branch_labels'].copy()
            preview,preview_labels=model.rgb_preview(rgb)
            assert preview['mode']=='rgb_only' and len(preview['features'])==4
            np.testing.assert_array_equal(preview_labels,camera_labels)
        else:
            pcm=(.15*np.sin(2*np.pi*440*np.arange(160000)/16000)).astype(np.float32)
            rgbs=[np.roll(rgb,i,axis=1) for i in range(10)]
            result,labels=model.infer(rgbs,pcm,16000)
            assert result['output_shape']==[10,71,224,224]
            assert model.trace.arrays['audio_embedding'].shape== (1,128)
            assert model.trace.arrays['audio_projected'].shape== (1,1,256)
            assert model.trace.arrays['visual_concat_tokens'].shape== (1,1029,256)
            fused=model.trace.arrays['audiovisual_fused'].copy()
            model.infer(rgbs,np.zeros_like(pcm),16000)
            difference=float(np.max(np.abs(fused-model.trace.arrays['audiovisual_fused'])))
            assert difference>1e-7,'Audio changes must affect the actual fused tensor'
            result['audio_change_max_abs']=difference
        report['models'][key]=dict(model=model.metadata,output_shape=result['output_shape'],
            feature_shapes={r['name']:r['shape'] for r in result['features']},elapsed_s=time.perf_counter()-start,
            audio_change_max_abs=result.get('audio_change_max_abs'))
        del model
        if torch.cuda.is_available():torch.cuda.empty_cache()
    path=ROOT/'logs/research/model_smoke.json';path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
