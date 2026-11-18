"""Geometric target point selection and GCC-PHAT acoustic bearings with ambiguity retained."""
import numpy as np
import cv2
from .calibration import rigid_matrix


def camera_model(k, distortion=None, distortion_model='plumb_bob'):
    k=np.asarray(k,float).reshape(3,3)
    d=np.asarray(distortion if distortion is not None else [],float)
    if (not np.isfinite(k).all() or min(k[0,0],k[1,1])<=0 or
            not np.allclose(k[2],[0,0,1]) or d.ndim!=1 or not np.isfinite(d).all() or
            len(d) not in (0,4,5,8,12,14) or distortion_model not in ('','plumb_bob','rational_polynomial')):
        raise ValueError('Unsupported or invalid camera model')
    return k,d


def target_geometry(mask, depth, k, distortion=None, distortion_model='plumb_bob'):
    mask = np.asarray(mask, bool); depth = np.asarray(depth, float); k = np.asarray(k, float).reshape(3,3)
    if mask.shape != depth.shape or not np.isfinite(k).all() or min(k[0,0],k[1,1]) <= 0:
        raise ValueError('Invalid target depth/calibration')
    y,x = np.nonzero(mask & np.isfinite(depth) & (depth>.1) & (depth<40))
    if len(x) < 16:
        return None
    z = depth[y,x]
    median = np.median(z); deviation = np.median(np.abs(z-median))
    keep = np.abs(z-median) <= max(.10, 3*deviation)
    x,y,z = x[keep],y[keep],z[keep]
    k,d=camera_model(k,distortion,distortion_model)
    rays=cv2.undistortPoints(np.column_stack([x,y]).astype(float).reshape(-1,1,2),k,d).reshape(-1,2)
    points = np.column_stack([rays,np.ones(len(rays))])*z[:,None]
    low, high = np.quantile(points, [.05,.95], axis=0)
    return dict(position=np.median(points,axis=0).tolist(), extent=(high-low).tolist(),
                depth_sigma_m=float(max(.01,deviation*1.4826)), depth_pixels=len(points),
                bounds_min_m=low.tolist(),bounds_max_m=high.tolist())


def select_target_points(points, camera_from_lidar, k, mask, depth, tolerance_m=.15,
                         distortion=None, distortion_model='plumb_bob'):
    points = np.asarray(points,float)
    if points.ndim != 2 or points.shape[1] < 3 or mask.shape != depth.shape:
        raise ValueError('Invalid point cloud/mask/depth')
    transform=rigid_matrix(camera_from_lidar); k,d=camera_model(k,distortion,distortion_model)
    if not np.isfinite(tolerance_m) or tolerance_m<=0:raise ValueError('Invalid target depth tolerance')
    xyz=points[:,:3];finite=np.isfinite(xyz).all(axis=1)
    xyz=xyz[finite]@transform[:3,:3].T+transform[:3,3]
    z=xyz[:,2];front=z>.1
    visible=xyz.copy();visible[~front]=[0,0,1]
    uv=cv2.projectPoints(visible.reshape(-1,1,3),np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2) if len(xyz) else np.empty((0,2))
    u,v=np.rint(uv[:,0]).astype(int),np.rint(uv[:,1]).astype(int)
    inside=front&(u>=0)&(v>=0)&(u<mask.shape[1])&(v<mask.shape[0])
    indices=np.flatnonzero(inside);x,y=u[indices],v[indices]
    observed=depth[y,x]
    same_surface=(mask[y,x]&np.isfinite(observed)&(observed>.1)&(np.abs(z[indices]-observed)<=tolerance_m))
    selected=np.unique(xyz[indices[same_surface]],axis=0)
    return selected.astype(np.float32),dict(input_points=len(points),projected_points=len(indices),
        target_distinct_points=len(selected),projection='camera_intrinsics_mask_registered_depth',
        centroid_m=np.median(selected,axis=0).tolist() if len(selected) else None,
        range_from_rgb_m=float(np.linalg.norm(np.median(selected,axis=0))) if len(selected) else None,
        extent_m=np.ptp(selected,axis=0).tolist() if len(selected) else None,
        depth_gate_m=tolerance_m, coordinate_frame='rgb_optical')


def acoustic_window(samples, sample_rate, anchor_offset_s, window_s=.25):
    samples=np.asarray(samples)
    if (samples.ndim!=2 or not len(samples) or not np.isfinite(anchor_offset_s) or
            not 8000<=sample_rate<=192000):raise ValueError('Invalid acoustic analysis window')
    duration=len(samples)/sample_rate
    if not 0<=anchor_offset_s<=duration:raise ValueError('Target acquisition is outside audio recording')
    count=min(len(samples),int(sample_rate*window_s))
    start=max(0,min(len(samples)-count,int(round(anchor_offset_s*sample_rate))-count//2))
    return samples[start:start+count],dict(start_offset_s=start/sample_rate,end_offset_s=(start+count)/sample_rate,
        duration_s=count/sample_rate,anchor_offset_s=anchor_offset_s,selection='window_centered_on_target_acquisition')


def target_absence_evidence(tracks, common_from_rgb, k, depth, occupied_mask,
                            distortion=None, distortion_model='plumb_bob', tolerance_m=.20):
    """Depth rays establish absence only when background is beyond the old volume.

    A detector miss, unchanged depth or an unobserved area never proves removal.
    """
    transform=np.linalg.inv(rigid_matrix(common_from_rgb));k,d=camera_model(k,distortion,distortion_model)
    evidence={}
    for identity,track in tracks.items():
        if track.get('position') is None:continue
        center=transform[:3,:3]@np.asarray(track['position'])+transform[:3,3]
        if center[2]<=.1:continue
        uv=cv2.projectPoints(center.reshape(1,1,3),np.zeros(3),np.zeros(3),k,d)[0].reshape(2)
        x,y=np.rint(uv).astype(int)
        if x<3 or y<3 or x>=depth.shape[1]-3 or y>=depth.shape[0]-3:continue
        patch=depth[y-3:y+4,x-3:x+4];valid=np.isfinite(patch)&(patch>.1)&(patch<40)
        if valid.mean()<.8 or occupied_mask[y-3:y+4,x-3:x+4].any():continue
        values=patch[valid];half_extent=np.linalg.norm(track.get('extent') or [.1,.1,.1])/2
        if (values<center[2]-tolerance_m).mean()>=.8:evidence[identity]='occluded'
        elif (values>center[2]+half_extent+tolerance_m).mean()>=.9:evidence[identity]='clear_line_of_sight'
    return evidence


def gcc_delay(reference, signal, sample_rate, max_delay_s, interpolate=8):
    reference=np.asarray(reference,float);signal=np.asarray(signal,float)
    n=1
    while n<len(reference)+len(signal):n*=2
    spectrum=np.fft.rfft(reference,n)*np.conj(np.fft.rfft(signal,n))
    frequencies=np.fft.rfftfreq(n,1/sample_rate)
    spectrum[(frequencies<200)|(frequencies>min(6000,sample_rate*.45))]=0
    spectrum/=np.maximum(np.abs(spectrum),1e-12)
    correlation=np.fft.irfft(spectrum,n*interpolate)
    shift=min(int(max_delay_s*sample_rate*interpolate)+1,len(correlation)//2-1)
    correlation=np.concatenate([correlation[-shift:],correlation[:shift+1]])
    maximum=int(np.argmax(np.abs(correlation)));value=float(np.abs(correlation[maximum]))
    other=np.abs(correlation).copy();other[max(0,maximum-interpolate):maximum+interpolate+1]=0
    ratio=value/max(float(other.max()),1e-12)
    return (maximum-shift)/(sample_rate*interpolate),ratio


def acoustic_bearing(samples, sample_rate, microphone_positions_m, speed_m_s=343.):
    samples=np.asarray(samples,float);positions=np.asarray(microphone_positions_m,float)
    if (samples.ndim!=2 or positions.shape!=(samples.shape[1],3) or samples.shape[1]<3 or
            len(samples)<128 or not np.isfinite(samples).all() or not np.isfinite(positions).all()):
        raise ValueError('Acoustic direction requires PCM channels matching measured microphone geometry')
    if not 8000<=sample_rate<=192000 or not 300<=speed_m_s<=380:
        raise ValueError('Invalid acoustic rate/sound speed')
    rms=float(np.sqrt(np.mean(samples**2)))
    if rms<1e-4:
        return dict(valid=False,reason='silence',directions=[],distance_estimated=False)
    # Bound CPU work and avoid attributing a whole long recording to an instantaneous target.
    samples=samples[:min(len(samples),int(sample_rate*.25))]
    baseline=positions[1:]-positions[0];delays=[];ratios=[]
    for channel,delta in enumerate(baseline,1):
        tau,ratio=gcc_delay(samples[:,0],samples[:,channel],sample_rate,np.linalg.norm(delta)/speed_m_s)
        delays.append(tau);ratios.append(ratio)
    rank=int(np.linalg.matrix_rank(baseline))
    if rank<2:
        return dict(valid=False,reason='linear_array_direction_ambiguous',directions=[],distance_estimated=False)
    rhs=np.asarray(delays)*speed_m_s
    direction=np.linalg.lstsq(baseline,rhs,rcond=None)[0]
    residual=float(np.sqrt(np.mean((baseline@direction-rhs)**2))/speed_m_s)
    norm=float(np.linalg.norm(direction))
    if norm>1.12 or norm<.05 or min(ratios)<1.15 or residual>1/sample_rate:
        return dict(valid=False,reason='ambiguous_or_inconsistent_tdoa',directions=[],
                    residual_s=residual,peak_ratio=min(ratios),distance_estimated=False)
    if rank==2:
        _,_,vh=np.linalg.svd(baseline);normal=vh[-1]
        z=np.sqrt(max(0.,1-norm**2))
        directions=[direction+z*normal,direction-z*normal]
    else:directions=[direction/norm]
    return dict(valid=True,reason='gcc_phat_tdoa',directions=[(d/np.linalg.norm(d)).tolist() for d in directions],
                residual_s=residual,peak_ratio=min(ratios),tdoa_s=delays,geometric_rank=rank,
                planar_reflection_ambiguity=rank==2,distance_estimated=False,
                sample_window_s=len(samples)/sample_rate)


def match_sound_to_target(bearing, common_from_array, targets, max_angle_deg=12., ambiguity_margin_deg=5.):
    if not bearing.get('valid'):
        return dict(target_index=None,reason=bearing.get('reason','invalid_bearing'),candidates=[])
    if bearing.get('planar_reflection_ambiguity') and len(bearing['directions'])>1 and np.dot(*np.asarray(bearing['directions'])[:2])<.98:
        return dict(target_index=None,reason='unresolved_planar_array_reflection',candidates=[])
    transform=rigid_matrix(common_from_array);origin=transform[:3,3]
    rays=[np.asarray(d)@transform[:3,:3].T for d in bearing['directions']]
    candidates=[]
    for index,target in enumerate(targets):
        if target.get('position_common') is None:continue
        delta=np.asarray(target['position_common'])-origin;distance=float(np.linalg.norm(delta))
        if distance<.1:continue
        angle=min(float(np.rad2deg(np.arccos(np.clip(ray@(delta/distance),-1,1)))) for ray in rays)
        if angle<=max_angle_deg:candidates.append(dict(target_index=index,angle_deg=angle))
    candidates.sort(key=lambda row:row['angle_deg'])
    if not candidates:return dict(target_index=None,reason='no_target_in_acoustic_direction',candidates=[])
    if len(candidates)>1 and candidates[1]['angle_deg']-candidates[0]['angle_deg']<ambiguity_margin_deg:
        return dict(target_index=None,reason='multiple_targets_share_acoustic_bearing',candidates=candidates)
    return dict(target_index=candidates[0]['target_index'],reason='unique_spatial_bearing_match',
                angle_deg=candidates[0]['angle_deg'],candidates=candidates)
