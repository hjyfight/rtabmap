#!/usr/bin/env python3
"""Read live fusion and audit real recorded geometry/features, without accuracy claims."""
import argparse
import base64
import json
from pathlib import Path
import sys
import time
import cv2
import numpy as np
import rclpy
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_semantic'))
from rtabmap_semantic.fusion_geometry import projection,project_labels
from rtabmap_semantic.fusion_preview import build_preview
from rtabmap_semantic.labels import chinese_label
from rtabmap_semantic.segmentation import render_images


def audit_capture(directory):
    folder=Path(directory)
    paths=sorted(folder.glob('samples/*/metadata.json'))
    if not paths:raise ValueError('No completed real capture samples')
    timings=[];coverages=[];stamps=[];objects=0;different_branches=0;pipelines=set()
    for path in paths:
        metadata=json.loads(path.read_text());pipelines.add(metadata['pipeline_sha256'])
        with np.load(path.parent/'arrays.npz',allow_pickle=False) as data:
            nir=cv2.imread(str(path.parent/'nir.png'),cv2.IMREAD_GRAYSCALE)
            rgb=cv2.imread(str(path.parent/'rgb.png'))
            assert nir is not None and rgb is not None and float(nir.std())>2
            assert data['depth_m'].shape==rgb.shape[:2]
            u,v,valid=projection(data['depth_m'],metadata['rgb_info'],metadata['nir_info'],np.asarray(metadata['nir_from_rgb']))
            np.testing.assert_array_equal(valid,data['eval_mask'])
            np.testing.assert_array_equal(project_labels(data['nir_native_map'],u,v,valid),data['nir_map'])
            for i,row in enumerate(metadata['objects']):
                features=data['features'][i]
                assert features.shape==(2,1024) and np.isfinite(features).all()
                np.testing.assert_allclose(np.linalg.norm(features,axis=1),1,atol=1e-5)
                weights=np.asarray(row['weights'][:2]);combined=(features*weights[:,None]).sum(axis=0)
                combined/=np.linalg.norm(combined)
                np.testing.assert_allclose(combined,data['fused_features'][i],atol=1e-5)
                different_branches+=int(not np.allclose(features[0],features[1],atol=1e-5));objects+=1
            skew=max(abs(metadata['stamps'][k]-metadata['stamps']['rgb']) for k in ('nir','depth'))
            assert skew<=.05+1e-6
            stamps.append(skew);timings.append(metadata['inference_s']);coverages.append(metadata['coverage'])
    assert objects>0 and different_branches>0
    return dict(samples=len(paths),pipeline_count=len(pipelines),objects=objects,
        distinct_rgb_nir_features=different_branches,max_capture_skew_s=max(stamps),
        median_inference_s=float(np.median(timings)),mean_valid_coverage=float(np.mean(coverages)),
        human_annotations=len(list((folder/'annotations').glob('*.png'))),
        projection_and_occlusion_reproduced=True,independent_nir_projection_reproduced=True,
        weighted_fusion_reproduced=True,capture_directory=str(folder.resolve()))


def audit_preview(view,directory):
    """Compare the live native display with its exact captured source instances."""
    assert view['image_label_vocabulary']=='native_yolo'
    metadata=None;deadline=time.monotonic()+3
    while metadata is None and time.monotonic()<deadline:
        for path in Path(directory).glob('samples/*/metadata.json'):
            record=json.loads(path.read_text())
            if (record['session_id'],record['sequence'])==(view['session_id'],view['sequence']):
                metadata=record;folder=path.parent;break
        if metadata is None:time.sleep(.05)
    if metadata is None:raise ValueError('No capture for this live preview; restart collection or raise its frame limit')
    rgb=cv2.cvtColor(cv2.imread(str(folder/'rgb.png')),cv2.COLOR_BGR2RGB)
    nir=cv2.imread(str(folder/'nir.png'),cv2.IMREAD_GRAYSCALE)
    with np.load(folder/'arrays.npz',allow_pickle=False) as arrays:
        bundle=dict(rgb=rgb,nir=nir,native_names={int(k):v for k,v in metadata['native_names'].items()})
        for kind in ('rgb','nir'):
            bundle[kind+'_instances']=[{**row,'mask':arrays[row['mask_key']].copy()}
                for row in metadata['instances'][kind]]
        u,v,valid=projection(arrays['depth_m'],metadata['rgb_info'],metadata['nir_info'],np.asarray(metadata['nir_from_rgb']))
    originals={r['instance_id']:r for r in bundle['rgb_instances']}
    fused=view['image_labels']['fused']
    assert len(fused)==len(originals) and {r['instance_id'] for r in fused}==set(originals)
    refinements={r['instance_id']:r['class_id'] for r in fused if r['class_id']!=originals[r['instance_id']]['class_id']}
    eligible={r['instance_id']:r for r in metadata['objects'] if r['eligible']}
    for instance_id,native_id in refinements.items():
        assert eligible[instance_id]['native_class_id']==native_id
        assert eligible[instance_id]['selected_task_id']==eligible[instance_id]['predicted_task_id']
    result=dict(projection_u=u,projection_v=v,eval_mask=valid)
    expected=build_preview(bundle,result,refinements)
    assert view['image_instance_counts']==expected['instance_counts']
    assert view['image_labels']==expected['image_labels']
    assert len(view['image_labels']['rgb'])==len(bundle['rgb_instances'])
    from rtabmap_semantic.fusion_geometry import project_image
    inputs=dict(rgb=rgb,nir=cv2.cvtColor(project_image(nir,u,v),cv2.COLOR_GRAY2RGB),fused=rgb)
    for kind in ('rgb','nir','fused'):
        overlay,_=render_images(inputs[kind],expected['maps'][kind])
        ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,82])
        assert ok and base64.b64encode(encoded).decode()==view['images'][kind]
    return dict(full_rgb_instances_preserved=True,preview_native_vocabulary=True,
        preview_instance_counts=expected['instance_counts'],display_images_reproduced=True,
        same_frame_capture=str(folder),rgb_labels=[chinese_label(r['label']) for r in bundle['rgb_instances']])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=50.)
    parser.add_argument('--capture-only',type=Path,help='Audit saved real inputs without starting DDS')
    parser.add_argument('--verify-native-preview',action='store_true',help='Require an active capture of the exact live preview frame')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/segmentation_validation/fusion_live.json')
    args=parser.parse_args()
    if args.capture_only:
        report=dict(passed=True,real_sensor_and_model_data=True,accuracy_verified=False,
            live_three_view_images=False,live_heartbeat=False,**audit_capture(args.capture_only))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return
    rclpy.init();node=rclpy.create_node('segmentation_fusion_live_check')
    views=[];statuses=[]
    subs=[node.create_subscription(String,'/miti/segmentation/fusion_view',lambda m:views.append(json.loads(m.data)),2),
        node.create_subscription(String,'/miti/segmentation/fusion_status',lambda m:statuses.append(json.loads(m.data)),2)]
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline and (not views or not statuses):rclpy.spin_once(node,timeout_sec=.1)
        if not views or not statuses:raise ValueError('No live fusion view/heartbeat received')
        view=views[-1];status=statuses[-1]
        assert view['modalities']==['rgb','nir'] and view['accuracy_status'] in ('unlabeled','validated_fusion','validated_baseline')
        for key in ('rgb','nir','fused'):
            pixels=cv2.imdecode(np.frombuffer(base64.b64decode(view['images'][key]),np.uint8),cv2.IMREAD_COLOR)
            assert pixels is not None and pixels.shape[:2]==(view['height'],view['width'])
        preview=audit_preview(view,status['capture_directory']) if args.verify_native_preview else {}
        audit=audit_capture(status['capture_directory'])
        report=dict(passed=True,real_sensor_and_model_data=True,accuracy_verified=False,
            sequence=view['sequence'],live_three_view_images=True,live_heartbeat=True,**audit,**preview)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.with_suffix('.view.json').write_text(json.dumps(view,ensure_ascii=False)+'\n')
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
