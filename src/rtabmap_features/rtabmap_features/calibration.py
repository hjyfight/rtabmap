"""Explicit calibration records, rigid-frame validation and reproducible evidence references."""
import hashlib
import json
from pathlib import Path
import numpy as np


def rigid_matrix(value):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(matrix[:3,:3].T@matrix[:3,:3], np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(matrix[:3,:3]), 1, atol=1e-5)):
        raise ValueError('Expected a finite rigid target_from_source 4x4 transform')
    return matrix


def load_calibration(path):
    path = Path(path)
    value = json.loads(path.read_text())
    if value.get('schema') != 1 or not value.get('common_frame') or not value.get('version'):
        raise ValueError('Invalid calibration schema/frame/version')
    if set(value.get('sensors', {})) != {'rgb','infrared','lidar','audio'}:
        raise ValueError('Calibration must describe all four sensors, including unavailable ones')
    for name, sensor in value['sensors'].items():
        if sensor.get('status') not in ('unverified','measured','factory') or not sensor.get('frame_id'):
            raise ValueError('Missing calibration status or sensor frame: '+name)
        if sensor.get('common_from_sensor') is not None:
            rigid_matrix(sensor['common_from_sensor'])
        offset = sensor.get('clock_offset_s', 0.)
        uncertainty = sensor.get('clock_uncertainty_s')
        if isinstance(offset,bool) or not isinstance(offset,(int,float)) or not np.isfinite(offset) or (uncertainty is not None and
                (isinstance(uncertainty,bool) or not isinstance(uncertainty,(int,float)) or not np.isfinite(uncertainty) or uncertainty < 0)):
            raise ValueError('Invalid clock calibration')
        geometry = sensor.get('microphone_positions_m')
        if geometry is not None:
            geometry = np.asarray(geometry, float)
            if geometry.ndim != 2 or geometry.shape[1] != 3 or len(geometry) < 3 or not np.isfinite(geometry).all():
                raise ValueError('Microphone geometry needs at least three measured xyz positions')
        if sensor['status'] == 'measured':
            evidence = sensor.get('evidence', {})
            filename = (path.parent/evidence.get('path','')).resolve()
            if not filename.is_file() or hashlib.sha256(filename.read_bytes()).hexdigest() != evidence.get('sha256'):
                raise ValueError('Measured calibration requires intact referenced evidence: '+name)
            if sensor.get('common_from_sensor') is None or uncertainty is None:
                raise ValueError('Measured calibration requires frame transform and clock uncertainty')
        period=sensor.get('sampling_period_s')
        if period is not None and (isinstance(period,bool) or not isinstance(period,(int,float)) or not np.isfinite(period) or period<=0):
            raise ValueError('Invalid measured sampling period')
        for key in ('clock_valid_from_s','clock_valid_until_s'):
            v=sensor.get(key)
            if v is not None and (type(v) not in (int,float) or not np.isfinite(v) or v<0):raise ValueError('Invalid clock evidence interval')
        if sensor.get('clock_valid_from_s',0)>sensor.get('clock_valid_until_s',float('inf')):raise ValueError('Clock evidence interval reversed')
    value['profile_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    return value


def sensor_calibration(profile, name, frame_id):
    sensor = profile['sensors'][name]
    if sensor['frame_id'] != frame_id:
        raise ValueError('Sensor frame differs from calibration profile: '+name)
    if sensor.get('common_from_sensor') is None:
        return None
    return rigid_matrix(sensor['common_from_sensor'])


def calibration_assessment(profile, limits_s=.05, shortest_period_s=None,stamp=None,matched_gap_s=None):
    sensors = profile['sensors']
    periods=[s.get('sampling_period_s') for s in sensors.values()]
    if shortest_period_s is None and all(p is not None for p in periods):shortest_period_s=min(periods)
    limit = min(limits_s, shortest_period_s/10) if shortest_period_s is not None else limits_s
    measured = all(s['status']=='measured' for s in sensors.values())
    bounded = all(s.get('clock_uncertainty_s') is not None for s in sensors.values())
    # Pairwise uncertainty budgets require both clocks, not just a matching tolerance.
    uncertainty = max((a.get('clock_uncertainty_s') or 0)+(b.get('clock_uncertainty_s') or 0)
                      for i,a in enumerate(sensors.values()) for j,b in enumerate(sensors.values()) if i<j) if bounded else None
    scope=profile.get('measurement_scope','unverified')
    budget_verified=bool(measured and bounded and shortest_period_s is not None and uncertainty<=limit and scope=='physical_measurements')
    current=bool(stamp is not None and all(s.get('clock_valid_from_s',float('inf'))<=stamp<=s.get('clock_valid_until_s',-float('inf')) for s in sensors.values()))
    return dict(measured_all_sensors=measured, pairwise_uncertainty_bound_s=uncertainty,
                limit_s=limit, measurement_scope=profile.get('measurement_scope','unverified'),
                sampling_periods_known=shortest_period_s is not None,shortest_sampling_period_s=shortest_period_s,
                calibration_sync_budget_verified=budget_verified,clock_evidence_covers_stamp=current,
                matched_gap_s=matched_gap_s,
                synchronization_verified=bool(budget_verified and current and matched_gap_s is not None and matched_gap_s+uncertainty<=limit),
                profile_sha256=profile['profile_sha256'], calibration_version=profile['version'],
                missing=[n for n,s in sensors.items() if s['status']!='measured'])
