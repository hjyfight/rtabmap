import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from export_research_process import build_sample, timing, waveform


def archive(tmp_path, task, **values):
    metadata=dict(session='sample-session',sequence=4,task=task,source_stamp=100.,context={})
    path=tmp_path/'sample.npz'
    np.savez(path,metadata=np.array(json.dumps(metadata)),**values)
    return np.load(path,allow_pickle=False)


def snapshot(task):
    return dict(task=task,session_id='sample-session',inputs={},result=dict(sequence=4,source_stamp=100.,features=[]))


def test_rejects_another_session_even_with_matching_sequence(tmp_path):
    with archive(tmp_path,'avs') as z:
        wrong=snapshot('avs');wrong['session_id']='other-session'
        with pytest.raises(ValueError,match='identity mismatch'):
            build_sample('avs',wrong,z)


def test_rejects_racing_sequence_and_leaves_no_composite_sample(tmp_path):
    with archive(tmp_path,'avs') as z:
        wrong=snapshot('avs');wrong['result']['sequence']=5
        with pytest.raises(ValueError,match='identity mismatch'):
            build_sample('avs',wrong,z)


def test_old_audio_without_pcm_never_generates_waveform(tmp_path):
    with archive(tmp_path,'avs',input_log_mel=np.ones((10,1,96,64),np.float32)) as z:
        result=build_sample('avs',snapshot('avs'),z)
    assert 'log_mel' in result['visuals']
    assert 'waveform' not in result['visuals']
    assert 'audio_attention' not in result['visuals']
    assert result['evaluation']['verified'] is False


def test_waveform_envelope_uses_actual_samples_and_real_time():
    values=np.array([-.5,.1,-.2,.8],np.float32)
    result=waveform(values,2)
    assert result['duration_s']==2 and result['rate']==2
    np.testing.assert_allclose(np.asarray(result['envelope'])[:,1],values,atol=1e-6)
    assert result['envelope'][-1][0]==1.5


def test_voxel_features_map_to_query_xyz_instead_of_point_index_grid(tmp_path):
    features=np.array([[[[1.],[3.]],[[1.],[3.]]]],np.float32)
    xyz=np.array([[[0,0,0],[1,0,0],[2,0,0]]],np.float32)
    with archive(tmp_path,'pmf',scene_xyz=xyz[0],scene_rgb=np.full((3,3),127,np.uint8),
                 scene_normals=np.tile([0,0,1],(3,1)),target_query_xyz=xyz,
                 target_query_sensor=np.array([0,0,1]),geometry_mask_features=features,
                 target_point_to_voxel=np.array([[0,1,0]])) as z:
        result=build_sample('pmf',snapshot('pmf'),z)
    assert result['visuals']['geometry']['energy']==[1.,3.,1.]
    assert result['visuals']['geometry']['xyz']==xyz[0].tolist()
    assert result['tables']['concatenation']['verified_shape']==[3,6]


def test_raw_and_corrected_audio_times_use_recorded_per_sensor_offsets():
    context=dict(audio_start=100.,frame_stamps=[100.5],sync_gap_s=.0125,
                 clock_offsets_s={'rgb':50.,'audio':0.,'lidar':50.,'nir':50.})
    result=timing('avs',context);row=result['rows'][0]
    assert row['raw_a']==50.5 and row['a']==100.5
    assert row['raw_b']==100.4875 and row['b']==100.4875
    assert row['gap_ms']==pytest.approx(12.5)


def test_old_time_records_do_not_guess_applied_clock_offsets():
    context=dict(source_frames=[dict(rgb_stamp=10.,lidar_stamp=10.03,gap_s=.03)],sync_gap_s=.03,
                 clock_alignment={'workstation_minus_robot_s':55.})
    row=timing('pmf',context)['rows'][0]
    assert row['a']==10. and row['b']==10.03
    assert 'raw_a' not in row
