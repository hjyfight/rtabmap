#!/usr/bin/env python3
"""Read native NIR/depth quality and stereo-camera settings without modifying sensors."""
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from rcl_interfaces.srv import GetParameters,DescribeParameters
from sensor_msgs.msg import Image

PARAMETERS=['depth_module.enable_auto_exposure','depth_module.exposure','depth_module.gain',
    'depth_module.laser_power','depth_module.emitter_enabled','depth_module.auto_exposure_limit',
    'depth_module.auto_gain_limit','depth_module.auto_exposure_mode','depth_module.auto_exposure_priority',
    'stereo_module.enable_auto_exposure','stereo_module.exposure','stereo_module.gain',
    'rgb_camera.enable_auto_exposure','rgb_camera.exposure','rgb_camera.gain']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--camera-node',default='/camera/camera')
    parser.add_argument('--output',type=Path,default=Path('logs/segmentation_validation/nir_input.json'))
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('nir_input_quality_check');bridge=CvBridge()
    rows=[];depth=[];latest=None
    def receive_nir(message):
        nonlocal latest
        latest=bridge.imgmsg_to_cv2(message,desired_encoding='mono8').copy()
        rows.append(dict(stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
            saturated_fraction=float((latest>=250).mean()),
            percentiles=np.percentile(latest,[1,5,50,95,99]).tolist(),std=float(latest.std())))
    def receive_depth(message):
        pixels=bridge.imgmsg_to_cv2(message,desired_encoding='passthrough')
        values=pixels*.001 if message.encoding=='16UC1' else pixels
        depth.append(float((np.isfinite(values)&(values>.1)&(values<40)).mean()))
    subs=[node.create_subscription(Image,'/camera/infra2/image_rect_raw',receive_nir,qos_profile_sensor_data),
        node.create_subscription(Image,'/camera/depth/image_rect_raw',receive_depth,qos_profile_sensor_data)]
    get=node.create_client(GetParameters,args.camera_node+'/get_parameters')
    describe=node.create_client(DescribeParameters,args.camera_node+'/describe_parameters')
    future=descriptions=None;start=time.monotonic()
    try:
        while time.monotonic()-start<args.seconds:
            if future is None and get.service_is_ready():future=get.call_async(GetParameters.Request(names=PARAMETERS))
            if descriptions is None and describe.service_is_ready():
                descriptions=describe.call_async(DescribeParameters.Request(names=PARAMETERS))
            rclpy.spin_once(node,timeout_sec=.1)
        if not rows:raise RuntimeError('No native NIR images received')
        parameters={};ranges={}
        if future is not None and future.done():
            parameters={name:parameter_value_to_python(value) if value.type else None
                for name,value in zip(PARAMETERS,future.result().values)}
        if descriptions is not None and descriptions.done():
            ranges={r.name:dict(description=r.description,integer_ranges=[dict(min=v.from_value,max=v.to_value,step=v.step)
                for v in r.integer_range],floating_ranges=[dict(min=v.from_value,max=v.to_value,step=v.step)
                for v in r.floating_point_range]) for r in descriptions.result().descriptors if r.type}
        elapsed=time.monotonic()-start
        report=dict(native_nir=True,images=len(rows),hz=len(rows)/elapsed,
            median_saturated_fraction=float(np.median([r['saturated_fraction'] for r in rows])),
            median_depth_valid_fraction=float(np.median(depth)) if depth else None,
            camera_parameters=parameters,parameter_ranges=ranges,last_image_statistics=rows[-1],
            publishers=[dict(node=p.node_namespace.rstrip('/')+'/'+p.node_name) for p in
                node.get_publishers_info_by_topic('/camera/infra2/image_rect_raw')])
        args.output.parent.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(args.output.with_suffix('.raw.png')),latest)
        args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(report,ensure_ascii=False,indent=2))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
