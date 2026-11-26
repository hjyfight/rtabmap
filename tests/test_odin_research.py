from pathlib import Path
import sys
import numpy as np
import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from rtabmap_research.odin_runtime import knnquery,queryandgroup,interpolation
from rtabmap_research.odin import NATIVE_TO_SCANNET,NAMES,prepare_depth_context
from rtabmap_research.replay import observations_from_archive
from rtabmap_research.models import Trace


def test_reference_knn_matches_brute_force_and_never_crosses_batches():
    generator=torch.Generator().manual_seed(42)
    points=torch.rand((20,3),generator=generator)
    queries=torch.rand((6,3),generator=generator)
    offset=torch.tensor([10,20]);new_offset=torch.tensor([3,6])
    indices,distance=knnquery(4,points,queries,offset,new_offset)
    expected=[];expected_distance=[]
    for source,target,shift in [(points[:10],queries[:3],0),(points[10:],queries[3:],10)]:
        dist,idx=torch.cdist(target,source).topk(4,largest=False,sorted=True)
        expected.append(idx+shift);expected_distance.append(dist)
    torch.testing.assert_close(indices,torch.cat(expected),rtol=0,atol=0)
    torch.testing.assert_close(distance,torch.cat(expected_distance),rtol=1e-6,atol=1e-6)
    features=torch.rand((20,5),generator=generator)
    grouped,idx=queryandgroup(4,points,queries,features,None,offset,new_offset,return_indx=True)
    torch.testing.assert_close(grouped[:,:,:3],points[idx]-queries[:,None])
    torch.testing.assert_close(grouped[:,:,3:],features[idx])
    result=interpolation(points,queries,features,offset,new_offset,k=4)
    weights=1/(distance+1e-8);weights/=weights.sum(1,keepdim=True)
    torch.testing.assert_close(result,(features[idx]*weights[:,:,None]).sum(1),rtol=1e-6,atol=1e-6)


def test_native_classes_map_explicitly_to_shared_scannet_order():
    assert NAMES[18:]==['墙','地面'] and NAMES[2]=='椅子'
    np.testing.assert_array_equal(np.sort(NATIVE_TO_SCANNET),np.arange(20))
    assert NATIVE_TO_SCANNET[18]==0 and NATIVE_TO_SCANNET[19]==1 and NATIVE_TO_SCANNET[2]==4


def test_both_export_formats_preserve_original_stamps_poses_and_intrinsics():
    shared=dict(source_rgb_frames=np.zeros((2,4,6,3),np.uint8),source_depth_frames=np.ones((2,4,6),np.uint16),
        source_rgb_intrinsics=np.tile(np.eye(3).ravel(),(2,1)),source_lidar_0=np.ones((4,4)),source_lidar_1=np.ones((4,4)))
    stamps=np.asarray([[10,10.01],[11,11.02]])
    poses=np.tile(np.eye(4),(2,1,1));poses[1,0,3]=.1
    old=observations_from_archive(dict(shared,observation_stamps=stamps,observation_poses=poses))
    new=observations_from_archive(dict(shared,source_stamps=stamps,source_lidar_poses=poses,
        source_rgb_intrinsics=shared['source_rgb_intrinsics'].reshape(1,2,3,3)))
    for a,b in zip(old,new):
        assert a['rgb_stamp']==b['rgb_stamp'] and a['lidar_stamp']==b['lidar_stamp']
        np.testing.assert_array_equal(a['world_from_lidar'],b['world_from_lidar'])
        np.testing.assert_array_equal(a['k'],b['k'])


def test_large_point_feature_preview_does_not_exceed_jpeg_dimension_limit():
    trace=Trace();features=torch.ones(1,3,70000,1)
    trace.record('point.features',features,'point features')
    import base64,cv2
    picture=cv2.imdecode(np.frombuffer(base64.b64decode(trace.rows[0]['heatmap']),np.uint8),cv2.IMREAD_COLOR)
    assert picture is not None and max(picture.shape[:2])<65500
    assert trace.rows[0]['heatmap_layout']=='point_index_grid'
    assert trace.arrays['point_features'].shape==(1,3,70000,1)


def test_parallel_depth_preprocessing_preserves_view_order_holes_and_original_inputs():
    observations=[];originals=[]
    for i in range(6):
        depth=np.full((24,32),1000+i*125,np.uint16)
        depth[5:9,7:12]=0
        if i%2:depth=depth.astype(np.float32)/1000;depth[10,10]=np.nan
        originals.append(depth.copy())
        observations.append(dict(depth=depth,rgb=np.zeros((24,32,3),np.uint8),distortion=[]))
    serial=prepare_depth_context(observations,1)
    parallel=prepare_depth_context(observations,4)
    for i,(before,after) in enumerate(zip(serial,parallel)):
        np.testing.assert_array_equal(after[0],before[0])
        np.testing.assert_array_equal(after[1],before[1])
        np.testing.assert_array_equal(observations[i]['depth'],originals[i])


def test_parallel_depth_preprocessing_keeps_registered_depth_validation():
    import pytest
    with pytest.raises(ValueError,match='1..8'):
        prepare_depth_context([],0)
    row=dict(depth=np.zeros((24,32),np.uint16),rgb=np.zeros((24,32,3),np.uint8),distortion=[])
    with pytest.raises(ValueError,match='coverage'):
        prepare_depth_context([row],4)
    row['depth']=np.ones((8,9),np.uint16)
    with pytest.raises(ValueError,match='registered'):
        prepare_depth_context([row],4)
