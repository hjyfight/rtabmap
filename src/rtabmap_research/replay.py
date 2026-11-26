"""Decode archived real sensor bundles from PTv3 and ODIN feature exports."""
import json
import numpy as np


def observations_from_archive(arrays):
    stamps=np.asarray(arrays['source_stamps'] if 'source_stamps' in arrays else arrays['observation_stamps'])
    poses=np.asarray(arrays['source_lidar_poses'] if 'source_lidar_poses' in arrays else arrays['observation_poses'])
    intrinsics=np.asarray(arrays['source_rgb_intrinsics']).reshape(-1,9)
    if stamps.shape!=(len(poses),2) or intrinsics.shape[0]!=len(poses):raise ValueError('Incomplete archived sensor bundle')
    observations=[]
    for i,(rgb_stamp,lidar_stamp) in enumerate(stamps):
        observations.append(dict(rgb=arrays['source_rgb_frames'][i],depth=arrays['source_depth_frames'][i],
            points=arrays['source_lidar_'+str(i)],k=intrinsics[i],
            distortion=arrays['source_rgb_distortion'][i] if 'source_rgb_distortion' in arrays else [],
            world_from_lidar=poses[i],rgb_stamp=float(rgb_stamp),lidar_stamp=float(lidar_stamp),
            gap_s=abs(float(rgb_stamp-lidar_stamp))))
    return observations


def read_bundle(path):
    with np.load(path,allow_pickle=False) as archive:arrays={key:archive[key].copy() for key in archive.files}
    metadata=json.loads(str(arrays['metadata']));context=metadata.get('context',metadata.get('input'))
    if context is None:raise ValueError('Archive lacks sensor calibration context')
    observations=observations_from_archive(arrays)
    calibration=dict(status=context['calibration'],evidence=context['calibration_evidence'],
        camera_frame=context['camera_frame'],lidar_frame=context['lidar_frame'],intensity_scale=1/255)
    bundle=dict(observations=observations,matrix=arrays['camera_from_lidar'],calibration=calibration,
                enforce_depth_consistency=context.get('depth_consistency_enforced',True),
                gap_s=max(row['gap_s'] for row in observations),stamp=observations[-1]['rgb_stamp'])
    return arrays,metadata,bundle
