#!/usr/bin/env python3
"""Offline target pipeline benchmark with explicit synthetic inputs, optionally real UniBind."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_features'));sys.path.insert(0,str(ROOT/'tests'))
from test_object_multimodal_fusion import FixtureEncoder,scene
from test_object_multimodal_evidence import fit,measurements,cycle
from rtabmap_features.backbone import FrozenEncoder
from rtabmap_features.calibration import load_calibration
from rtabmap_features.object_multimodal import ObjectMultimodalEngine

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--real-model',action='store_true')
    p.add_argument('--device',choices=['cpu','cuda:0'],default='cpu');p.add_argument('--samples',type=int,default=3)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if not 1<=args.samples<=30:p.error('samples must be 1..30')
    args.output.mkdir(parents=True,exist_ok=True);measure=measurements()
    for row in measure['sensors'].values():
        row['reference_points_m']=row['source_points_m'];row['reference_event_stamps']=row['source_event_stamps']
    filename,_=fit.fit_measurements(measure,args.output/'synthetic_calibration')
    task=json.loads((ROOT/'config/object_multimodal_task.json').read_text())
    started=time.monotonic()
    encoder=FrozenEncoder(ROOT/'.tools/unibind',ROOT/'models/unibind/pretrained_weights.pt',args.device,retain_text=True) if args.real_model else FixtureEncoder()
    engine=ObjectMultimodalEngine(encoder,task,load_calibration(filename))
    initialization=time.monotonic()-started;results=[];offline_times=[]
    for i in range(args.samples):
        started=time.monotonic();bundle=scene();bundle.update(stamp=100.+i,sequence=i+1)
        bundle['audio']=type(bundle['audio'])(bundle['audio'].payload,99.9+i,'audio','synthetic-audio',bundle['audio'].duration,bundle['audio'].attributes)
        result=engine.compute(bundle);encoded=json.dumps(result,ensure_ascii=False,allow_nan=False)
        offline_times.append(time.monotonic()-started);results.append(result)
        (args.output/f'result_{i+1}.json').write_text(encoded+'\n')
        print(json.dumps(dict(sample=i+1,core_compute_s=result['timing']['processing_s'],offline_prepare_to_serialization_s=offline_times[-1],
            targets=len(result['targets']),complete=result['targets'][0]['features']['complete'] if result['targets'] else False)),flush=True)
    report=cycle.cycle_report(results)
    report.update(model='real_pretrained_unibind' if args.real_model else 'explicit_fixture_encoder',
        inference_device=args.device,scope='synthetic_target_images_point_geometry_multichannel_pcm',
        initialization_s=initialization,offline_prepare_to_serialization_s=offline_times,
        robot_connected=False,accuracy_validation='deferred_no_ground_truth',hardware_synchronization_verified=False)
    (args.output/'cycle_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
