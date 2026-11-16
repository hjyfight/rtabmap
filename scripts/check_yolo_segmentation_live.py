#!/usr/bin/env python3
"""Read-only validation of real RGB segmentation and its fusion requests."""
import argparse
import base64
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.serialization import serialize_message
from cv_bridge import CvBridge
from std_msgs.msg import String
from rtabmap_semantic_msgs.msg import SegmentationFrame, SegmentationFusionRequest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_semantic'))
from rtabmap_semantic.segmentation import dense_maps, render_images


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=40)
    parser.add_argument('--output',type=Path,default=ROOT/'logs/segmentation_validation/live.json')
    args=parser.parse_args()
    rclpy.init()
    node=rclpy.create_node('yolo_segmentation_live_check')
    bridge=CvBridge()
    frames={}
    statuses=[]
    checked=[]
    subscriptions=[]

    def receive(kind, message):
        if kind=='view':
            message=json.loads(message.data)
            key=(message['session_id'],message['sequence'])
        else:
            target=message.baseline if kind=='request' else message
            key=(target.session_id,target.sequence)
        frames.setdefault(key,{})[kind]=message
        while len(frames)>12:
            frames.pop(next(iter(frames)))

    for kind, suffix, type_ in [('baseline','baseline',SegmentationFrame),('selected','selected',SegmentationFrame),
                              ('request','fusion_request',SegmentationFusionRequest)]:
        subscriptions.append(node.create_subscription(type_,'/perception/segmentation/'+suffix,
            lambda m,k=kind:receive(k,m),4))
    subscriptions.append(node.create_subscription(String,'/miti/segmentation/view',lambda m:receive('view',m),2))
    subscriptions.append(node.create_subscription(String,'/miti/segmentation/status',lambda m:statuses.append(json.loads(m.data)),2))
    deadline=time.monotonic()+args.seconds
    try:
        while time.monotonic()<deadline and len(checked)<3:
            rclpy.spin_once(node,timeout_sec=.05)
            for key, values in list(frames.items()):
                if len(values)!=4 or any(r['sequence']==key[1] and r['session_id']==key[0] for r in checked):
                    continue
                baseline,selected,request,view=(values[k] for k in ('baseline','selected','request','view'))
                assert serialize_message(baseline)==serialize_message(selected), 'Default selected output differs from RGB baseline'
                assert serialize_message(baseline)==serialize_message(request.baseline)
                assert request.rgb.header==baseline.header
                rows=[]
                for row in baseline.instances:
                    mask=bridge.imgmsg_to_cv2(row.mask)
                    rows.append(dict(instance_id=row.instance_id,class_id=row.class_id,confidence=row.confidence,
                        mask_offset=list(row.mask_offset),mask=mask))
                classes,identities=dense_maps(rows,(baseline.height,baseline.width))
                np.testing.assert_array_equal(classes,bridge.imgmsg_to_cv2(baseline.class_map))
                np.testing.assert_array_equal(identities,bridge.imgmsg_to_cv2(baseline.instance_map))
                rgb=bridge.imgmsg_to_cv2(request.rgb,desired_encoding='rgb8')
                overlay,color_image=render_images(rgb,classes)
                assert view['stamp']==dict(sec=baseline.header.stamp.sec,nanosec=baseline.header.stamp.nanosec)
                assert view['revision']==0 and view['width']==baseline.width and view['height']==baseline.height
                assert len(view['instances'])==len(baseline.instances)
                for name,pixels in [('raw',rgb),('overlay',overlay),('classes',color_image)]:
                    ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,82])
                    assert ok and base64.b64encode(encoded).decode()==view['images'][name]
                args.output.parent.mkdir(parents=True,exist_ok=True)
                cv2.imwrite(str(args.output.with_suffix('.overlay.png')),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))
                cv2.imwrite(str(args.output.with_suffix('.rgb.png')),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
                args.output.with_suffix('.view.json').write_text(json.dumps(view,ensure_ascii=False))
                checked.append(dict(session_id=key[0],sequence=key[1],instances=len(baseline.instances),
                    labels=[r.label for r in baseline.instances],inference_seconds=baseline.inference_seconds,
                    width=baseline.width,height=baseline.height,model_sha256=baseline.model_sha256))
        report=dict(passed=len(checked)>=3,real_camera=True,real_yolo=True,frames=checked,
            baseline_selected_identical=True if checked else None,request_rgb_and_masks_verified=bool(checked),
            atomic_display_pixels_verified=bool(checked),last_status=statuses[-1] if statuses else None,
            accuracy_evaluated=False)
        report['nonempty_masks_seen']=any(row['instances'] for row in checked)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2))
        if not report['passed']:
            raise RuntimeError('Did not receive three complete real segmentation frames')
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
