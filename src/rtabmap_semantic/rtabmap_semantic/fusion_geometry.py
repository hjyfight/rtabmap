"""RGB/depth to rectified NIR projection, including right-camera P translation."""
from collections import deque
import hashlib
import json
import cv2
import numpy as np


def transform_matrix(translation, quaternion):
    q=np.asarray(quaternion,dtype=float)
    norm=np.linalg.norm(q)
    if not np.isfinite(q).all() or norm<1e-9:
        raise ValueError('Invalid calibration quaternion')
    x,y,z,w=q/norm
    m=np.eye(4)
    m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    m[:3,3]=translation
    if not np.isfinite(m).all(): raise ValueError('Invalid calibration translation')
    return m


def find_transform(transforms, source, target):
    """Static child->parent transforms. Return target_from_source."""
    graph={}
    for row in transforms.values():
        m=transform_matrix(row['translation'],row['rotation'])
        graph.setdefault(row['child'],[]).append((row['parent'],m))
        graph.setdefault(row['parent'],[]).append((row['child'],np.linalg.inv(m)))
    pending=deque([(source,np.eye(4))]);seen=set()
    while pending:
        frame,m=pending.popleft()
        if frame==target:return m
        if frame in seen:continue
        seen.add(frame)
        pending.extend((other,step@m) for other,step in graph.get(frame,[]) if other not in seen)
    raise ValueError('No camera calibration path: '+source+' -> '+target)


def camera_dict(message):
    return dict(width=message.width,height=message.height,frame_id=message.header.frame_id,
        k=list(message.k),p=list(message.p),r=list(message.r),d=list(message.d),
        distortion_model=message.distortion_model)


def rectified_reference_transform(transforms,rgb_frame,nir_info,image_frame):
    """Use the CameraInfo reference; prove a differing image frame matches stereo P."""
    reference=nir_info['frame_id']
    if image_frame!=reference:
        reference_from_image=find_transform(transforms,image_frame,reference)
        p=np.asarray(nir_info['p'],float).reshape(3,4)
        r=np.asarray(nir_info['r'],float).reshape(3,3)
        expected=-p[:,:3]@r@reference_from_image[:3,3]
        if (not np.allclose(reference_from_image[:3,:3],np.eye(3),atol=1e-3) or
                np.linalg.norm(p[:,3])<1e-9 or not np.allclose(p[:,3],expected,atol=.02)):
            raise ValueError('NIR image frame does not match the calibrated stereo P baseline')
    return find_transform(transforms,rgb_frame,reference)


def projection(depth, rgb_info, nir_info, nir_from_rgb, occlusion_tolerance_m=.04):
    height,width=depth.shape
    if (width,height)!=(rgb_info['width'],rgb_info['height']):
        raise ValueError('Registered depth and RGB calibration dimensions differ')
    k=np.asarray(rgb_info['k'],float).reshape(3,3)
    p=np.asarray(nir_info['p'],float).reshape(3,4)
    r=np.asarray(nir_info['r'],float).reshape(3,3)
    if not all(np.isfinite(m).all() for m in (k,p,r,nir_from_rgb)) or min(k[0,0],k[1,1],p[0,0],p[1,1])<=0:
        raise ValueError('Invalid intrinsics or projection matrix')
    if rgb_info['distortion_model'] not in ('','plumb_bob','rational_polynomial'):
        raise ValueError('Unsupported RGB distortion model')
    yy,xx=np.indices(depth.shape,dtype=np.float32)
    rays=cv2.undistortPoints(np.stack([xx,yy],axis=-1).reshape(-1,1,2),k,np.asarray(rgb_info['d'],float)).reshape(-1,2)
    xyz=np.column_stack([rays,np.ones(len(rays))])*depth.reshape(-1,1)
    transformed=(xyz@nir_from_rgb[:3,:3].T+nir_from_rgb[:3,3])@r.T
    homogeneous=transformed@p[:,:3].T+p[:,3]
    z=homogeneous[:,2]
    safe=np.where(z>1e-6,z,1)
    u,v=homogeneous[:,0]/safe,homogeneous[:,1]/safe
    valid=np.isfinite(depth.ravel())&(depth.ravel()>.1)&(depth.ravel()<40)&(z>1e-6)
    valid &= np.isfinite(u)&np.isfinite(v)&(u>=0)&(u<nir_info['width']-1)&(v>=0)&(v<nir_info['height']-1)
    ix=np.rint(np.where(valid,u,0)).astype(int);iy=np.rint(np.where(valid,v,0)).astype(int)
    index=iy*nir_info['width']+ix
    zbuffer=np.full(nir_info['width']*nir_info['height'],np.inf)
    np.minimum.at(zbuffer,index[valid],z[valid])
    valid &= z<=zbuffer[index]+occlusion_tolerance_m
    u=np.where(valid,u,-1).astype(np.float32).reshape(height,width)
    v=np.where(valid,v,-1).astype(np.float32).reshape(height,width)
    return u,v,valid.reshape(height,width)


def project_image(image, u, v):
    return cv2.remap(image,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=127)


def project_labels(labels, u, v, valid):
    result=np.full(valid.shape,-1,np.int32)
    x=np.rint(u[valid]).astype(int);y=np.rint(v[valid]).astype(int)
    result[valid]=labels[y,x]
    return result


def masked_square_crop(image, mask, box):
    x1,y1,x2,y2=[int(x) for x in box]
    region=image[y1:y2,x1:x2].copy()
    if not region.size:raise ValueError('Empty object crop')
    region[~mask[y1:y2,x1:x2]]=127
    h,w=region.shape[:2];size=max(h,w)
    padded=np.full((size,size)+region.shape[2:],127,np.uint8)
    y,x=(size-h)//2,(size-w)//2
    padded[y:y+h,x:x+w]=region
    return padded


def calibration_version(rgb_info,nir_info,transform):
    payload=dict(rgb=rgb_info,nir=nir_info,nir_from_rgb=np.asarray(transform).tolist())
    return 'realsense_factory_'+hashlib.sha256(json.dumps(payload,sort_keys=True).encode()).hexdigest()[:20]
