from pathlib import Path
import sys
import numpy as np
import pytest
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from rtabmap_research.rgbnir import RGBNIR
from rtabmap_research.registration import pair_rgbnir,register_nir
from rtabmap_research.inputs import NotReady


def test_pair_selects_complete_frame_without_relaxing_sync_or_age():
    frames=[dict(stamp=10),dict(stamp=10.1)]
    nirs=[dict(stamp=10.01)]
    assert pair_rgbnir(frames,nirs,.05,10.2,3)[0]['stamp']==10
    with pytest.raises(NotReady):pair_rgbnir(frames,nirs,.001,10.2,3)
    with pytest.raises(NotReady):pair_rgbnir(frames,nirs,.05,14,3)


def test_registration_reuses_stereo_projection_and_rejects_wrong_frame():
    camera=dict(width=8,height=6,frame_id='camera',k=[5,0,3,0,5,2,0,0,1],
        p=[5,0,3,-1,0,5,2,0,0,0,1,0],r=np.eye(3).ravel().tolist(),d=[],distortion_model='')
    frame=dict(stamp=10,raw_stamp=10,frame='camera',rgb=np.zeros((6,8,3),np.uint8),
        depth=np.full((6,8),1000,np.uint16),rgb_info=camera)
    nir=dict(stamp=10,raw_stamp=10,frame='camera',image=np.tile(np.arange(8,dtype=np.uint8),(6,1)))
    bundle=register_nir(frame,nir,camera,{})
    assert bundle['valid'][2,3] and bundle['nir'][2,3]==2
    frame['depth'][2,3]=0
    assert not register_nir(frame,nir,camera,{})['valid'][2,3]
    nir['frame']='unknown'
    with pytest.raises(ValueError):register_nir(frame,nir,camera,{})


@pytest.fixture(scope='module')
def model():
    torch.set_num_threads(2)
    return RGBNIR(ROOT/'models/yolov8s-seg.pt','cpu',size=64)


def test_frozen_features_feed_head_and_zero_weight_reproduces_rgb(model):
    generator=torch.Generator().manual_seed(7)
    rgb=torch.rand(1,3,64,64,generator=generator)
    nir=torch.rand(1,1,64,64,generator=generator).repeat(1,3,1,1)
    valid=torch.ones(1,1,64,64);valid[:,:,:,32:]=0
    with torch.inference_mode():baseline=model.prediction_parts(model.net(rgb))[0].clone()
    zero=model.prediction_parts(model.forward_tensors(rgb,nir,valid,nir_weight=0))[0]
    torch.testing.assert_close(zero,baseline,rtol=0,atol=0)
    fused=model.prediction_parts(model.forward_tensors(rgb,nir,valid))[0]
    assert not torch.equal(fused,baseline)
    first={k:v.copy() for k,v in model.trace.arrays.items()}
    model.forward_tensors(rgb,torch.zeros_like(nir),valid)
    for level in ('P3','P4','P5'):
        # NIR affects the head's inputs but never later pure-RGB backbone layers.
        np.testing.assert_array_equal(first[level+'_rgb'],model.trace.arrays[level+'_rgb'])
        alpha=first[level+'_nir_weight']
        np.testing.assert_allclose(first[level+'_fused'],
            (1-alpha)*first[level+'_rgb']+alpha*first[level+'_nir_scaled'],rtol=1e-6,atol=1e-7)
    assert all(not p.requires_grad for p in model.net.parameters())


def test_decode_and_missing_registration(model):
    rgb=np.zeros((64,64,3),np.uint8);nir=np.zeros((64,64),np.uint8)
    with pytest.raises(NotReady):model.infer(rgb,nir,np.zeros(nir.shape,bool))
    result,detections=model.infer(rgb,nir,np.ones(nir.shape,bool))
    assert len(result['features'])==12 and result['output_shape'][1]==116
    assert result['baseline_count']>=0 and isinstance(detections,list)
