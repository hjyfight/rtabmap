"""Reuse the existing calibrated RGB/depth-to-NIR camera projection."""
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'rtabmap_semantic'))
from rtabmap_semantic.fusion_geometry import (projection,project_image,
    rectified_reference_transform,calibration_version)
from .inputs import NotReady


def pair_rgbnir(frames,nirs,tolerance,now,max_age):
    if not frames or not nirs:raise NotReady('等待同步RGB-D与mono8近红外')
    for frame in reversed(frames):
        if not -.5 <= now-frame['stamp'] <= max_age:continue
        nir=min(nirs,key=lambda row:abs(row['stamp']-frame['stamp']))
        if abs(nir['stamp']-frame['stamp'])<=tolerance and -.5<=now-nir['stamp']<=max_age:
            return frame,nir
    raise NotReady('等待新鲜RGB-D/NIR完整配对，采集时间差须在容差内')


def register_nir(frame,nir,nir_info,transforms):
    rgb_info=frame['rgb_info']
    if frame['frame']!=rgb_info['frame_id'] or frame['rgb'].shape[:2]!=frame['depth'].shape:
        raise ValueError('RGB-D图像与RGB CameraInfo不匹配')
    if nir['image'].dtype!=np.uint8 or nir['image'].shape!=(nir_info['height'],nir_info['width']):
        raise ValueError('NIR须为与CameraInfo尺寸匹配的mono8图像')
    matrix=rectified_reference_transform(transforms,frame['frame'],nir_info,nir['frame'])
    depth=frame['depth'].astype(np.float32)
    if frame['depth'].dtype==np.uint16:depth*=.001
    elif frame['depth'].dtype!=np.float32:raise ValueError('未知注册深度单位')
    u,v,valid=projection(depth,rgb_info,nir_info,matrix)
    return dict(rgb=frame['rgb'],nir=project_image(nir['image'],u,v),valid=valid,
        stamp=frame['stamp'],gap_s=abs(frame['stamp']-nir['stamp']),
        rgb_stamp=frame['raw_stamp'],nir_stamp=nir['raw_stamp'],matrix=matrix,
        calibration_version=calibration_version(rgb_info,nir_info,matrix),
        rgb_info=rgb_info,nir_info=nir_info,projection_u=u,projection_v=v,
        source_nir=nir['image'],source_depth=frame['depth'])
