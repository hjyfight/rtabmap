#!/usr/bin/env python3
"""Try a lower stereo exposure, retaining it only if NIR improves and depth holds."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter,parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters,SetParameters
from sensor_msgs.msg import Image


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exposure',type=int,default=2500)
    parser.add_argument('--output',type=Path,default=Path('logs/segmentation_validation/nir_exposure_trial.json'))
    args=parser.parse_args()
    if not 500<=args.exposure<=8500:raise ValueError('Use a conservative 500..8500 us trial exposure')
    rclpy.init();node=rclpy.create_node('nir_exposure_quality_trial');bridge=CvBridge()
    get=node.create_client(GetParameters,'/camera/camera/get_parameters')
    set_=node.create_client(SetParameters,'/camera/camera/set_parameters')
    names=['depth_module.enable_auto_exposure','depth_module.exposure']
    original=None;retain=False;changed=False;latest=None;rows=[];depth=[]
    def call(client,request):
        if not client.wait_for_service(timeout_sec=6):raise RuntimeError('Camera parameter service unavailable')
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=6)
        if not future.done():raise RuntimeError('Camera parameter service timeout')
        return future.result()
    def set_values(values):
        result=call(set_,SetParameters.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()]))
        if len(result.results)!=len(values) or not all(r.successful for r in result.results):
            raise RuntimeError('Camera rejected parameters: '+str([r.reason for r in result.results]))
    def nir(message):
        nonlocal latest
        latest=bridge.imgmsg_to_cv2(message,desired_encoding='mono8').copy()
        rows.append((float((latest>=250).mean()),float(np.median(latest))))
    def raw_depth(message):
        pixels=bridge.imgmsg_to_cv2(message,desired_encoding='passthrough')
        values=pixels*.001 if message.encoding=='16UC1' else pixels
        depth.append(float((np.isfinite(values)&(values>.1)&(values<40)).mean()))
    subs=[node.create_subscription(Image,'/camera/infra2/image_rect_raw',nir,qos_profile_sensor_data),
          node.create_subscription(Image,'/camera/depth/image_rect_raw',raw_depth,qos_profile_sensor_data)]
    def sample(seconds,suffix):
        rows.clear();depth.clear();end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if len(rows)<5 or len(depth)<5:raise RuntimeError('Insufficient simultaneous NIR/depth samples')
        cv2.imwrite(str(args.output.with_suffix(suffix+'.png')),latest)
        return dict(images=len(rows),saturated_fraction=float(np.median([r[0] for r in rows])),
            image_median=float(np.median([r[1] for r in rows])),depth_valid_fraction=float(np.median(depth)))
    def restore():
        set_values({names[0]:False,names[1]:original[names[1]]})
        set_values({names[0]:original[names[0]]})
    try:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        values=call(get,GetParameters.Request(names=names)).values
        if len(values)!=2 or values[0].type!=1 or values[1].type!=2:raise RuntimeError('Unexpected stereo parameter types')
        original={k:parameter_value_to_python(v) for k,v in zip(names,values)}
        before=sample(3,'.before')
        args.output.write_text(json.dumps(dict(state='trial',original=original,before=before),indent=2)+'\n')
        changed=True
        set_values({names[0]:False,names[1]:args.exposure})
        end=time.monotonic()+1
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        after=sample(5,'.after')
        retain=(after['saturated_fraction']<.15 and after['saturated_fraction']<before['saturated_fraction']*.5
            and 25<=after['image_median']<=220
            and after['depth_valid_fraction']>=before['depth_valid_fraction']-.08)
        if not retain:restore();changed=False
        current=call(get,GetParameters.Request(names=names)).values
        report=dict(applied=retain,original=original,parameters_after={k:parameter_value_to_python(v) for k,v in zip(names,current)},
            before=before,after=after,depth_safeguard_passed=after['depth_valid_fraction']>=before['depth_valid_fraction']-.08,
            projector_changed=False,accuracy_verified=False)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,indent=2))
    finally:
        if changed and not retain and original is not None:restore()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
