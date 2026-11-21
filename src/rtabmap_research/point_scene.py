"""Combine real timestamped RGB/LiDAR observations using their odometry poses."""
import cv2
import numpy as np
from scipy.spatial import cKDTree

from .inputs import NotReady,project_pmf
from .ptv3 import normals_from_lidar


def project_visible(xyz,camera_from_world,k,distortion,depth):
    h,w=depth.shape
    camera=xyz@camera_from_world[:3,:3].T+camera_from_world[:3,3]
    indices=np.flatnonzero(camera[:,2]>.15)
    pixels=np.floor(cv2.projectPoints(camera[indices],np.zeros(3),np.zeros(3),np.asarray(k).reshape(3,3),
                                     np.asarray(distortion))[0].reshape(-1,2)).astype(int)
    inside=(pixels[:,0]>=0)&(pixels[:,0]<w)&(pixels[:,1]>=0)&(pixels[:,1]<h)
    indices,pixels=indices[inside],pixels[inside]
    values=depth[pixels[:,1],pixels[:,0]].astype(np.float32)
    if depth.dtype==np.uint16:values/=1000
    good=np.isfinite(values)&(values>.15)&(values<10)&(np.abs(values-camera[indices,2])<=.2)
    indices,pixels=indices[good],pixels[good]
    order=np.argsort(camera[indices,2],kind='stable')
    _,first=np.unique(pixels[order,1]*w+pixels[order,0],return_index=True)
    keep=order[first]
    return indices[keep],pixels[keep]


def build_scene(observations,camera_from_lidar,min_frames=4):
    if len(observations)<min_frames:
        raise NotReady(f'等待多帧室内点云：{len(observations)}/{min_frames}组RGB-D/LiDAR')
    if (len({o['rgb_stamp'] for o in observations})!=len(observations) or
            len({o['lidar_stamp'] for o in observations})!=len(observations)):
        raise NotReady('重复曝光或扫描不能充当多帧点云')
    full,xyz,colors,provenance=[],[],[],[]
    for number,obs in enumerate(observations):
        points=obs['points']
        pose=obs['world_from_lidar']
        finite=np.isfinite(points).all(1)&(np.linalg.norm(points[:,:3],axis=1)>.15)&(np.linalg.norm(points[:,:3],axis=1)<12)
        points=points[finite]
        world=points[:,:3]@pose[:3,:3].T+pose[:3,3]
        full.append(world)
        try:
            _,_,_,uv,selected=project_pmf(points,obs['rgb'],obs['k'],obs['distortion'],camera_from_lidar,1/255)
        except NotReady:
            continue
        camera=points[selected,:3]@camera_from_lidar[:3,:3].T+camera_from_lidar[:3,3]
        depth=obs['depth'][uv[:,1],uv[:,0]].astype(np.float32)
        if obs['depth'].dtype==np.uint16:depth/=1000
        valid=np.isfinite(depth)&(depth>.15)&(depth<10)&(np.abs(depth-camera[:,2])<=.2)
        selected,uv=selected[valid],uv[valid]
        xyz.append(world[selected]);colors.append(obs['rgb'][uv[:,1],uv[:,0]])
        provenance.append(np.column_stack([np.full(len(selected),number),np.flatnonzero(finite)[selected]]))
    if not xyz or sum(len(x) for x in xyz)<128:
        raise NotReady('多帧RGB/LiDAR深度一致点不足128个')
    full=np.concatenate(full).astype(np.float32)
    _,unique=np.unique(np.floor(full/.04).astype(np.int64),axis=0,return_index=True)
    full=full[unique]
    xyz=np.concatenate(xyz).astype(np.float32);colors=np.concatenate(colors);provenance=np.concatenate(provenance)
    _,unique=np.unique(np.floor(xyz/.02).astype(np.int64),axis=0,return_index=True)
    xyz,colors,provenance=xyz[unique],colors[unique],provenance[unique]
    latest=observations[-1]
    normal,curvature=normals_from_lidar(full-latest['world_from_lidar'][:3,3])
    _,nearest=cKDTree(full).query(xyz,workers=2)
    camera_from_world=camera_from_lidar@np.linalg.inv(latest['world_from_lidar'])
    render_indices,uv=project_visible(xyz,camera_from_world,latest['k'],latest['distortion'],latest['depth'])
    if len(render_indices)<32:raise NotReady('当前RGB视野内的多帧LiDAR点不足32个')
    return dict(xyz=xyz,rgb=colors,normals=normal[nearest],curvature=curvature[nearest],
                source_indices=provenance,uv=uv,render_indices=render_indices,
                full_geometry_points=len(full),observations=observations,
                camera_from_world=camera_from_world,current_rgb=latest['rgb'])
