import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from research_mapping_data import MappingEvidence,transform,residual,performance


def test_constraint_residual_uses_relative_pose_not_global_distance():
    a=transform([10,2,0],[0,0,0,1]);b=transform([11,2,0],[0,0,0,1])
    result=residual(a,b,transform([1,0,0],[0,0,0,1]))
    assert result['translation_m']==pytest.approx(0)
    assert result['rotation_deg']==pytest.approx(0)


def test_raw_node_pose_is_aligned_to_map_before_optimization_comparison():
    evidence=MappingEvidence();raw=np.eye(4);raw[0,3]=1
    evidence.nodes([dict(id=1,map_id=0,stamp=5,pose=raw,points=[[0,0,1]])])
    correction=transform([2,0,0],[0,0,0,1])
    evidence.update_graph({1:correction@raw},[],6,'map',correction)
    sample=evidence.export(now=7)
    assert sample['graph']['comparison'][0]['raw']==[3.,0.,0.]
    assert sample['graph']['comparison'][0]['correction_m']==pytest.approx(0)
    assert sample['visuals']['sparse']['xyz']==[[3.,0.,1.]]


def test_semantic_object_position_uses_real_keyframe_anchors():
    e=MappingEvidence();e.update_graph({1:transform([1,0,0],[0,0,0,1]),2:transform([3,0,0],[0,0,0,1])},[],5,'map',np.eye(4))
    e.semantic([dict(id=1,label='chair',confidence=.8,dynamic=False,anchors=[
        dict(node_id=1,position=[0,0,1],weight=1),dict(node_id=2,position=[0,0,1],weight=3)])],'test.db','map-1')
    obj=e.export(now=7)['associations'][0]
    assert obj['position']==[2.5,0.,1.]
    assert obj['node_ids']==[1,2]


def test_reused_node_identity_clears_old_map_features_and_objects():
    e=MappingEvidence();row=dict(id=1,map_id=0,stamp=5,pose=np.eye(4),points=[[0,0,1]])
    e.nodes([row]);e.update_graph({1:np.eye(4)},[],6,'map',np.eye(4));old=e.session
    e.nodes([dict(row,stamp=100,points=[])])
    assert e.session!=old and not e.graph and not e.features


def test_stale_service_nodes_do_not_replace_new_node_features():
    e=MappingEvidence();row=dict(id=1,map_id=0,stamp=100,pose=np.eye(4),points=[[0,0,2]])
    e.nodes([row]);e.nodes([dict(row,stamp=5,points=[[0,0,1]])],allow_reset=False)
    np.testing.assert_equal(e.features[1],[[0,0,2]])


def test_no_inputs_stays_waiting_and_never_passes_performance():
    sample=MappingEvidence().export(now=7)
    assert sample['state']=='waiting'
    assert all(row['value'] is None and row['state']=='pending' for row in sample['performance']['rows'])


def test_report_needs_matching_map_and_success_trial_counts():
    record=dict(verified=True,evidence='record-1',method='repeated runs',scene_id='scene-1',value=98.,success=49,total=50)
    report=dict(map_uuid='map-1',measurements={'mapping_success_rate':record})
    row=lambda result:next(x for x in result['rows'] if x['key']=='mapping_success_rate')
    assert row(performance(report,'other-map'))['state']=='pending'
    assert row(performance(report,'map-1'))['state']=='pass'
    report['measurements']['mapping_success_rate']['success']=1
    assert row(performance(report,'map-1'))['value'] is None


def test_sync_error_uses_measurement_uncertainty_and_rejects_unverified_numbers():
    record=dict(verified=True,evidence='record-1',method='hardware time capture',scene_id='scene-1',value=9.,uncertainty_ms=2.)
    report=dict(map_uuid='map-1',measurements={'sync_error_ms':record})
    row=lambda result:next(x for x in result['rows'] if x['key']=='sync_error_ms')
    assert row(performance(report,'map-1'))['state']=='fail'
    record['verified']=False
    assert row(performance(report,'map-1'))['state']=='pending'
