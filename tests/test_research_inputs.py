import json
import sys
import time
from types import SimpleNamespace
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from rtabmap_research.inputs import (AudioTimeline,NotReady,measured_transform,pair_frames,project_pmf,
                                     tf_preview_transform,pair_rgb_lidar)


def test_unverified_calibration_never_projects(tmp_path):
    path=tmp_path/'calibration.json'
    path.write_text(json.dumps(dict(status='unverified',camera_from_lidar=np.eye(4).tolist())))
    with pytest.raises(NotReady):measured_transform(path)


def test_reject_reflection_and_missing_intensity(tmp_path):
    cfg=dict(status='measured',evidence='calibration-record',camera_frame='rgb',lidar_frame='lidar',
             camera_from_lidar=np.diag([-1,1,1,1]).tolist(),intensity_scale=1/255)
    path=tmp_path/'calibration.json';path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):measured_transform(path)
    cfg['camera_from_lidar']=np.eye(4).tolist();cfg['intensity_scale']=None
    path.write_text(json.dumps(cfg))
    with pytest.raises(NotReady):measured_transform(path)


def test_projection_z_buffer_padding_and_author_channels():
    # Camera K maps x/z into pixel x. 40 distinct pixels + farther duplicate.
    xyz=np.column_stack([np.arange(40),np.zeros(40),np.ones(40)])
    points=np.column_stack([xyz,np.full(40,128)]).astype(np.float32)
    points=np.vstack([points,[0,0,3,255],[0,0,-1,255]])
    sparse,image,mask,uv,indices=project_pmf(points,np.zeros((19,43,3),np.uint8),
        [[1,0,0],[0,10,2],[0,0,1]],[],np.eye(4),1/255)
    assert sparse.shape==(5,32,48) and image.shape==(3,32,48)
    assert mask.sum()==40 and 40 not in indices and 41 not in indices
    pixel=np.flatnonzero(indices==0)[0];u,v=uv[pixel]
    np.testing.assert_allclose(sparse[:,v,u],(np.array([1,0,0,1,128/255])-
        np.array([12.12,10.88,.23,-1.04,.21]))/np.array([12.32,11.47,6.91,.86,.16]),rtol=1e-5)
    assert np.count_nonzero(sparse[:,~mask])==0


def test_pcm_overlap_reconstruction_preserves_samples():
    rate=8000;source=np.sin(np.arange(rate*11)/10).astype(np.float32)
    timeline=AudioTimeline()
    for i in range(19):
        start=i*.5
        timeline.add(start,source[int(start*rate):int((start+2)*rate)],rate,str(i),'estimated')
    samples,_,start,end,_=timeline.extract()
    assert end==11 and start==1
    np.testing.assert_array_equal(samples,source[rate:rate*11])


def test_pcm_gap_and_duplicate_do_not_fabricate_a_window():
    timeline=AudioTimeline();pcm=np.zeros(16000,np.float32)
    timeline.add(0,pcm,8000,'one','estimated')
    timeline.add(8,pcm,8000,'two','estimated')
    timeline.add(8,pcm,8000,'two','estimated')
    assert len(timeline.windows)==2
    with pytest.raises(NotReady):timeline.extract()
    timeline.add(10,np.zeros(32000,np.float32),16000,'new-rate','estimated')
    assert len(timeline.windows)==1


def test_go2_short_windows_cover_ten_seconds_and_prune_old_audio():
    rate=8000
    source=np.sin(np.arange(rate*16)/10).astype(np.float32)
    timeline=AudioTimeline()
    for i in range(151):
        offset=i*(rate//10)
        timeline.add(offset/rate,source[offset:offset+rate//2],rate,str(i),'estimated')
    samples,_,start,end,_=timeline.extract()
    assert end==15.5 and start==5.5
    np.testing.assert_array_equal(samples,source[int(start*rate):int(end*rate)])
    assert 64<len(timeline.windows)<151
    assert timeline.windows[0][1]>=end-timeline.max_seconds


def test_audio_window_can_end_at_delayed_video_time_without_padding():
    rate=8000
    source=np.sin(np.arange(rate*15)/10).astype(np.float32)
    timeline=AudioTimeline()
    for i in range(27):
        offset=i*(rate//2)
        timeline.add(offset/rate,source[offset:offset+2*rate],rate,str(i),'estimated')
    samples,_,start,end,_=timeline.extract(end=13.)
    assert start==3. and end==13.
    np.testing.assert_array_equal(samples,source[3*rate:13*rate])
    with pytest.raises(NotReady):timeline.extract(end=15.1)


def test_avs_pairs_local_audio_with_delayed_wifi_video(monkeypatch):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from run_fusion_research import Research
    research=Research.__new__(Research)
    research.audio=AudioTimeline()
    research.cfg={'max_input_age_s':3.,'avs_sync_tolerance_s':.25}
    rate=8000
    for i in range(27):
        research.audio.add(i*.5,np.full(2*rate,i,np.float32),rate,str(i),'estimated')
    research.frames=[(i*.2,None,'camera') for i in range(69)]
    monkeypatch.setattr(time,'time',lambda:15.)
    bundle=research.avs_bundle()
    assert bundle['audio_end']<15.
    assert bundle['frame_stamps'][-1]==research.frames[-1][0]
    assert len(set(bundle['frame_stamps']))==10
    assert bundle['gap_s']<=.25


def test_frames_require_ten_distinct_matching_exposures():
    frames=[(float(i),None,'camera') for i in range(10)]
    selected,gaps=pair_frames(frames,np.arange(10)+.1,.2)
    assert len(selected)==10 and max(gaps)<.2
    with pytest.raises(NotReady):pair_frames(frames,np.arange(10)+.3,.2)
    with pytest.raises(NotReady):pair_frames(frames,np.zeros(10),.2)


def test_tf_preview_records_unverified_geometry_and_preserves_direction():
    matrix, meta = tf_preview_transform([1,2,3], [0,0,np.sqrt(.5),np.sqrt(.5)],
                                        'rgb', 'lidar', 1/255, 'Livox uint8 reflectivity')
    np.testing.assert_allclose(matrix @ [1,0,0,1], [1,3,3,1], atol=1e-10)
    assert meta['status']=='tf_preview' and meta['geometry_verified'] is False
    with pytest.raises(ValueError):
        tf_preview_transform([1,2,3], [0,0,0,0], 'rgb', 'lidar', 1/255, 'record')
    with pytest.raises(NotReady):
        tf_preview_transform([1,2,3], [0,0,0,1], 'rgb', 'lidar', None, 'record')


def test_rgb_lidar_pair_handles_delivery_lag_without_relaxing_sync():
    frames=[(10.,'older','rgb'), (10.17,'latest','rgb')]
    clouds=[(10.02,'scan')]
    frame,cloud,gap=pair_rgb_lidar(frames,clouds,.1,10.3,3.)
    assert frame[1]=='older' and cloud[1]=='scan' and gap<.1
    with pytest.raises(NotReady):pair_rgb_lidar(frames,clouds,.01,10.3,3.)
    with pytest.raises(NotReady):pair_rgb_lidar(frames,clouds,.1,14.,3.)


def test_measured_calibration_takes_priority_over_tf_preview(tmp_path):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from run_fusion_research import Research
    path=tmp_path/'calibration.json'
    cfg=dict(status='measured',evidence='measurement record',camera_frame='rgb',lidar_frame='lidar',
             camera_from_lidar=np.eye(4).tolist(),intensity_scale=1/255)
    path.write_text(json.dumps(cfg))
    research=Research(dict(public_directory=str(tmp_path/'public'),calibration_path=str(path),
                           pmf_extrinsics_mode='tf_preview',pmf_sync_tolerance_s=.1,max_input_age_s=3))
    stamp=time.time()
    research.frames.append((stamp,np.zeros((19,43,3),np.uint8),'rgb'))
    research.clouds.append((stamp,SimpleNamespace(header=SimpleNamespace(frame_id='lidar'))))
    research.info=dict(frame='rgb',height=19,width=43,distortion_model='plumb_bob')
    bundle=research.pmf_bundle()
    assert bundle['calibration']['status']=='measured' and research.tf_buffer is None
    cfg['camera_from_lidar']=np.diag([-1,1,1,1]).tolist()
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):research.pmf_bundle()
