import copy
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from research_input_preview import build_input_preview
from export_research_process import build_snapshot_sample


def result(context):
    return dict(sequence=8,source_stamp=1000.,context=context,features=[],images={'raw':'original-jpeg'})


def test_lidar_preview_uses_real_scan_and_query_arrays_with_bounded_point_counts():
    count=1500
    xyz=np.column_stack([np.arange(count)*.01,np.zeros(count),np.full(count,2)]).astype(np.float32)
    raw=np.column_stack([np.arange(2000)*.01,np.zeros(2000),np.full(2000,2),np.full(2000,32)]).astype(np.float32)
    arrays=dict(scene_xyz=xyz,scene_rgb=np.full((count,3),128,np.uint8),scene_normals=np.tile([0,0,-1],(count,1)),
        reference_ptv3_input_features=np.ones((count,6),np.float32),
        point_source_indices=np.column_stack([np.ones(count,int),np.arange(count)]),
        source_lidar_poses=np.tile(np.eye(4),(2,1,1)),source_lidar_1=raw,
        projection_uv=np.column_stack([np.arange(40),np.zeros(40)]),render_indices=np.arange(40),
        target_query_xyz=xyz[None],target_query_sensor=np.zeros(count,np.uint8),
        target_point_to_voxel=np.arange(count)[None],geometry_mask_features=np.ones((1,4,count,1),np.float32))
    context=dict(lidar_frame='livox_frame',world_frame='map',inputs_used=[{'modality':'lidar','shape':[2000,4]}])
    value=result(context)
    preview=build_input_preview('pmf','session',value,arrays,maximum=64)
    visuals=preview['visuals']
    assert len(visuals['cloud']['xyz'])==64 and visuals['cloud']['count']==count
    assert len(visuals['geometry']['xyz'])==64 and visuals['geometry']['count']==count
    assert len(visuals['filter']['before'])==64 and visuals['filter']['before_count']==2000
    assert visuals['filter']['frame']=='livox_frame' and visuals['cloud']['frame']=='map'
    np.testing.assert_allclose(visuals['filter']['before'][0],raw[0,:3])
    value['input_preview']=preview
    sample=build_snapshot_sample('pmf',dict(task='pmf',session_id='session',result=value))
    assert sample['visuals']==visuals
    assert sample['input_preview_source']=='current_inference_memory'
    assert sample['interface_validation']['state']=='missing'
    assert sample['archive_level']=='result_snapshot_only'
    context['inputs_used'][0]['shape'][0]=1
    assert preview['inputs_used'][0]['shape']==[2000,4]


def test_rgbnir_preview_retains_raw_registered_images_and_actual_pixel_correspondence():
    y,x=np.indices((8,10))
    arrays=dict(source_nir=np.arange(320,dtype=np.uint8).reshape(16,20),
        input_registered_nir=np.arange(80,dtype=np.uint8).reshape(8,10),nir_valid=np.ones((8,10),bool),
        projection_u=x.astype(float)*2,projection_v=y.astype(float)*2,nir_from_rgb=np.eye(4))
    preview=build_input_preview('rgbnir','session',result({}),arrays)
    assert all(preview['visuals'][name] for name in ['nir_raw','nir_registered','valid_mask'])
    pairs=preview['visuals']['correspondence']
    assert pairs['count']==80
    np.testing.assert_array_equal(np.asarray(pairs['nir_uv']),np.asarray(pairs['rgb_uv'])*2)


def test_audio_preview_has_original_pcm_waveform_log_mel_and_attention():
    pcm=np.sin(np.arange(160000)*.01).astype(np.float32)
    arrays=dict(input_pcm=pcm,input_sample_rate=np.array(16000,np.int32),
        input_log_mel=np.ones((10,1,96,64),np.float32),
        audio_visual_attention=np.full((1,1,56,56),1/3136,np.float32),
        audio_channel_weights=np.full((1,256,1,1),.5,np.float32),
        queries_visual_attention=np.full((1,300,1029),1/1029,np.float32),
        audio_embedding=np.ones((1,128),np.float32),audio_projected=np.ones((1,256),np.float32))
    preview=build_input_preview('avs','session',result({}),arrays)
    wave=preview['visuals']['waveform']
    assert wave['duration_s']==10 and wave['samples']==len(pcm) and len(wave['envelope'])<=600
    assert all(preview['visuals'][name] for name in ['log_mel','audio_attention','query_attention_28'])
    assert len(preview['tables']['channel_weights'])==256
    assert len(preview['tables']['audio_vectors']['audio_embedding'])==128


@pytest.mark.parametrize('field,value',[('session_id','another'),('sequence',9),('source_stamp',1001.),('task','avs')])
def test_preview_cannot_be_mixed_with_another_inference_batch(field,value):
    value_result=result({})
    preview=build_input_preview('pmf','session',value_result,{})
    preview[field]=value
    value_result['input_preview']=copy.deepcopy(preview)
    with pytest.raises(ValueError,match='preview identity mismatch'):
        build_snapshot_sample('pmf',dict(task='pmf',session_id='session',result=value_result))
