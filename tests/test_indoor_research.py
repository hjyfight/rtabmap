import json
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from rtabmap_research.inputs import NotReady,estimated_transform
from rtabmap_research.ptv3 import coloured_points,normals_from_lidar
from rtabmap_research.point_scene import build_scene


def fixture():
    x,y=np.meshgrid(np.arange(8)*.025,np.arange(8)*.025)
    points=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,2.),np.full(x.size,32)]).astype(np.float32)
    rgb=np.full((64,80,3),128,np.uint8)
    depth=np.full((64,80),2,np.float32)
    k=np.asarray([[200,0,20],[0,200,20],[0,0,1]],np.float32)
    return points,rgb,depth,k


def test_estimate_never_becomes_measured_and_requires_independent_evidence(tmp_path):
    cfg=dict(status='auto_estimated',geometry_verified=False,evidence='real frame validation',
             camera_frame='rgb',lidar_frame='lidar',camera_from_lidar=np.eye(4).tolist(),intensity_scale=1/255,
             validation=dict(improvement_observed=True,old_common=dict(median_m=.8),
                             estimated_common=dict(points=200,median_m=.14,within_20cm=.6)))
    path=tmp_path/'estimate.json';path.write_text(json.dumps(cfg))
    _,loaded=estimated_transform(path)
    assert loaded['status']=='auto_estimated' and loaded['geometry_verified'] is False
    cfg['validation']['estimated_common']['points']=20
    path.write_text(json.dumps(cfg))
    with pytest.raises(NotReady):estimated_transform(path)
    cfg['validation']['estimated_common']['points']=200
    cfg['camera_from_lidar']=np.diag([-1,1,1,1]).tolist()
    path.write_text(json.dumps(cfg))
    with pytest.raises(ValueError):estimated_transform(path)


def test_colour_association_uses_measured_points_and_rejects_behind_surface():
    points,rgb,depth,k=fixture()
    prepared=coloured_points(rgb,points,np.eye(4),k,[],depth)
    np.testing.assert_array_equal(prepared['xyz'],points[prepared['source_indices'],:3])
    np.testing.assert_array_equal(prepared['rgb'],rgb[prepared['uv'][:,1],prepared['uv'][:,0]])
    assert np.allclose(np.linalg.norm(prepared['normals'],axis=1),1)
    assert np.all(prepared['normals'][:,2]<-.99)
    with pytest.raises(NotReady):coloured_points(rgb,points,np.eye(4),k,[],depth+.5)


def test_scene_uses_timestamped_pose_and_never_counts_repeated_frames():
    points,rgb,depth,k=fixture()
    observations=[]
    for i in range(4):
        pose=np.eye(4);pose[0,3]=i*.01
        local=points.copy();local[:,0]-=i*.01
        observations.append(dict(points=local,rgb=rgb,depth=depth,k=k,distortion=[],world_from_lidar=pose,
                                 rgb_stamp=100+i*.2,lidar_stamp=100+i*.2+.01))
    scene=build_scene(observations,np.eye(4))
    assert scene['xyz'].shape[0]>=32 and scene['source_indices'].shape[0]==len(scene['xyz'])
    assert len(scene['render_indices'])>=32
    np.testing.assert_allclose(scene['xyz'][:,2],2)
    with pytest.raises(NotReady):build_scene([observations[0]]*4,np.eye(4))
