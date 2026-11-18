#!/usr/bin/env python3
"""Fit supplied 3D/time correspondences. This program never connects to a sensor or robot."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_features'))
from rtabmap_features.calibration import rigid_matrix,load_calibration,calibration_assessment


def fit_rigid(source,reference):
    source=np.asarray(source,float);reference=np.asarray(reference,float)
    if source.shape!=reference.shape or source.ndim!=2 or source.shape[1]!=3 or len(source)<4:
        raise ValueError('Need at least four corresponding xyz measurements per sensor')
    if not np.isfinite(source).all() or not np.isfinite(reference).all():raise ValueError('Nonfinite calibration measurements')
    x=source-source.mean(axis=0);y=reference-reference.mean(axis=0)
    if np.linalg.matrix_rank(x)<2:raise ValueError('Collinear calibration geometry')
    u,_,vh=np.linalg.svd(x.T@y);rotation=vh.T@u.T
    if np.linalg.det(rotation)<0:vh[-1]*=-1;rotation=vh.T@u.T
    transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=reference.mean(axis=0)-source.mean(axis=0)@rotation.T
    predicted=source@rotation.T+transform[:3,3];errors=np.linalg.norm(predicted-reference,axis=1)
    return rigid_matrix(transform),dict(samples=len(source),rmse_m=float(np.sqrt(np.mean(errors**2))),max_error_m=float(errors.max()))


def fit_clock(sensor_stamps,reference_stamps,instrument_uncertainty_s):
    sensor=np.asarray(sensor_stamps,float);reference=np.asarray(reference_stamps,float)
    if sensor.shape!=reference.shape or sensor.ndim!=1 or len(sensor)<5 or np.ptp(sensor)<=0:
        raise ValueError('Need at least five corresponding acquisition events over time')
    if not np.isfinite(sensor).all() or not np.isfinite(reference).all() or not np.isfinite(instrument_uncertainty_s) or instrument_uncertainty_s<0:
        raise ValueError('Invalid clock measurement')
    offsets=reference-sensor;offset=float(np.median(offsets));residual=offsets-offset
    drift=float(np.polyfit(sensor-sensor.mean(),offsets,1)[0])
    uncertainty=float(np.abs(residual).max()+instrument_uncertainty_s)
    return dict(clock_offset_s=offset,clock_uncertainty_s=uncertainty,events=len(sensor),
                acquisition_span_s=float(np.ptp(sensor)),offset_drift_s_per_s=drift,
                instrument_uncertainty_s=instrument_uncertainty_s,
                limitation='Bound describes supplied event interval; repeat after clock/source changes')


def fit_measurements(measurements,output):
    if measurements.get('schema')!=1 or measurements.get('measurement_scope') not in ('physical_measurements','synthetic_fixture'):
        raise ValueError('Declare physical measurements or synthetic fixture explicitly')
    if not measurements.get('reference_event_method') or not measurements.get('instrument_reference'):
        raise ValueError('Reference event method and instrument calibration reference are required')
    if set(measurements.get('sensors',{}))!={'rgb','infrared','lidar','audio'}:
        raise ValueError('Provide all four calibration sensor records')
    if not measurements.get('version') or not measurements.get('common_frame'):raise ValueError('Calibration identity/frame required')
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    measurement_data=json.dumps(measurements,sort_keys=True,indent=2,allow_nan=False)+'\n'
    (output/'input_measurements.json').write_text(measurement_data)
    profile=dict(schema=1,version=measurements['version'],common_frame=measurements['common_frame'],
                 measurement_scope=measurements['measurement_scope'],sensors={})
    report=dict(schema=1,measurement_scope=measurements['measurement_scope'],reference_event_method=measurements['reference_event_method'],
                instrument_reference=measurements['instrument_reference'],sensors={},hardware_contacted=False,
                input_measurements_sha256=hashlib.sha256(measurement_data.encode()).hexdigest())
    for name,row in measurements['sensors'].items():
        transform,geometry=fit_rigid(row['source_points_m'],row['reference_points_m'])
        clock=fit_clock(row['source_event_stamps'],row['reference_event_stamps'],row['instrument_uncertainty_s'])
        maximum=row.get('max_geometry_rmse_m')
        if maximum is None or not np.isfinite(maximum) or maximum<=0:raise ValueError('Specify the task calibration RMSE acceptance limit for '+name)
        accepted=geometry['rmse_m']<=maximum
        sensor=dict(frame_id=row['frame_id'],status='measured' if accepted else 'unverified',common_from_sensor=transform.tolist(),
                    clock_offset_s=clock['clock_offset_s'],clock_uncertainty_s=clock['clock_uncertainty_s'])
        if name=='audio':sensor['microphone_positions_m']=row.get('microphone_positions_m')
        sensor['sampling_period_s']=row.get('sampling_period_s')
        sensor['clock_valid_from_s']=float(min(row['reference_event_stamps']))
        sensor['clock_valid_until_s']=float(max(row['reference_event_stamps']))
        profile['sensors'][name]=sensor
        report['sensors'][name]=dict(geometry=geometry,clock=clock,geometry_accepted=accepted,
                                    task_geometry_rmse_limit_m=maximum)
    evidence=output/'fit_evidence.json';evidence.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    digest=hashlib.sha256(evidence.read_bytes()).hexdigest()
    for sensor in profile['sensors'].values():sensor['evidence']=dict(path=evidence.name,sha256=digest)
    filename=output/'calibration.json';filename.write_text(json.dumps(profile,indent=2,allow_nan=False)+'\n')
    checked=load_calibration(filename)
    assessment=calibration_assessment(checked)
    (output/'assessment.json').write_text(json.dumps(assessment,indent=2)+'\n')
    return filename,assessment


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurements',type=Path)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--check',type=Path)
    args=parser.parse_args()
    if args.check:
        print(json.dumps(calibration_assessment(load_calibration(args.check)),ensure_ascii=False,indent=2));return
    if args.measurements is None or args.output is None:parser.error('Use --measurements and --output, or --check')
    filename,assessment=fit_measurements(json.loads(args.measurements.read_text()),args.output)
    print(json.dumps(dict(profile=str(filename),assessment=assessment),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
