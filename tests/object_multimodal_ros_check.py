"""Actual target ROS worker, isolated fixture inputs; never opens sensors/robot sockets."""
import copy
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import CameraInfo,Image,PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String,Header
from cv_bridge import CvBridge
from rtabmap_features_msgs.msg import AudioFrame
from rtabmap_semantic_msgs.msg import SegmentationFrame,SegmentationInstance,SegmentationFusionRequest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_features'));sys.path.insert(0,str(ROOT/'src/rtabmap_semantic'))
from rtabmap_features.object_node import ObjectFusionNode
from rtabmap_features.object_multimodal import ObjectMultimodalEngine
from rtabmap_features.backbone import FrozenEncoder
from rtabmap_features.calibration import load_calibration
from rtabmap_features.standard_interface import validate_result
from rtabmap_features.result_journal import validate_observation
from test_object_multimodal_fusion import scene,FixtureEncoder
from test_object_multimodal_evidence import fit,measurements

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--real-model',action='store_true')
    parser.add_argument('--device',choices=['cpu','cuda:0'],default='cpu')
    parser.add_argument('--output',type=Path,default=ROOT/'logs/object_multimodal_validation/ros_contract.json')
    parser.add_argument('--clock-offset-s',type=float,default=0.);args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='object_fusion_ros_') as directory:
        output=Path(directory);measure=measurements()
        for row in measure['sensors'].values():
            row['reference_points_m']=row['source_points_m']
            row['reference_event_stamps']=[stamp+args.clock_offset_s for stamp in row['source_event_stamps']]
        filename,_=fit.fit_measurements(measure,output)
        profile=load_calibration(filename);task=json.loads((ROOT/'config/object_multimodal_task.json').read_text())
        task['tracking_frame']='base_link';task_path=output/'task.json';task_path.write_text(json.dumps(task))
        rclpy.init(args=['--ros-args','-p','calibration_path:='+str(filename),'-p','task_path:='+str(task_path),
            '-p','journal_path:='+str(output/'results.jsonl'),'-p','min_period_s:=0.05','-p','max_input_age_s:=10.0'])
        encoder=FrozenEncoder(ROOT/'.tools/unibind',ROOT/'models/unibind/pretrained_weights.pt',args.device,retain_text=True) if args.real_model else FixtureEncoder()
        engine=ObjectMultimodalEngine(encoder,task,profile)
        node=ObjectFusionNode(engine);probe=rclpy.create_node('object_fusion_contract_probe');executor=SingleThreadedExecutor()
        executor.add_node(node);executor.add_node(probe);bridge=CvBridge();results=[];statuses=[];timings=[]
        subscriptions=[probe.create_subscription(String,topic,target.append,10) for topic,target in
            [('/perception/fusion/targets',results),('/perception/fusion/status',statuses),('/perception/fusion/timing',timings)]]
        pubs={name:probe.create_publisher(kind,topic,qos_profile_sensor_data) for name,kind,topic in [
            ('infrared',Image,node.config['nir_topic']),('depth',Image,node.config['depth_topic']),
            ('lidar',PointCloud2,node.config['lidar_topic']),('audio',AudioFrame,node.config['audio_topic']),
            ('rgb_info',CameraInfo,node.config['rgb_info_topic']),('nir_info',CameraInfo,node.config['nir_info_topic'])]}
        request_pub=probe.create_publisher(SegmentationFusionRequest,'/perception/segmentation/fusion_request',2)
        tf_pub=probe.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        transform=TransformStamped();transform.header.frame_id='rgb';transform.child_frame_id='infrared';transform.transform.rotation.w=1.
        base_tf=copy.deepcopy(transform);base_tf.header.frame_id='base_link';base_tf.child_frame_id='base_fixture'
        tf_pub.publish(TFMessage(transforms=[transform,base_tf]))

        def until(predicate,seconds=8.,action=None):
            end=time.monotonic()+seconds
            while not predicate() and time.monotonic()<end:
                if action:action()
                executor.spin_once(timeout_sec=.02)
            assert predicate(),f'ROS condition timed out: {node.state}'
        def inputs(sequence=1,include_depth=True,bad_nir=False):
            bundle=scene();stamp=probe.get_clock().now().to_msg()
            stamp.sec,stamp.nanosec=divmod(int((stamp.sec+stamp.nanosec/1e9-args.clock_offset_s)*1e9),1000000000)
            stamp_s=stamp.sec+stamp.nanosec/1e9
            header=Header(stamp=stamp,frame_id='rgb')
            rgb=bridge.cv2_to_imgmsg(bundle['rgb'],'rgb8');rgb.header=copy.deepcopy(header)
            frame=SegmentationFrame();frame.header=copy.deepcopy(header);frame.session_id='isolated_fixture';frame.sequence=sequence;frame.width=frame.height=64
            instance=SegmentationInstance();instance.instance_id=7;instance.class_id=0;instance.label='person';instance.confidence=.9
            instance.mask_offset=[0,0];instance.bbox_xyxy=[16.,16.,48.,48.]
            instance.mask=bridge.cv2_to_imgmsg(bundle['instances'][0]['full_mask'].astype(np.uint8),'mono8');instance.mask.header=copy.deepcopy(header);frame.instances=[instance]
            request=SegmentationFusionRequest(baseline=frame,rgb=rgb,deadline_seconds=10.)
            depth=bridge.cv2_to_imgmsg((bundle['depth_m']*1000).astype(np.uint16),'16UC1');depth.header=copy.deepcopy(header)
            nir=bridge.cv2_to_imgmsg(bundle['nir_warped'],'mono8');nir.header=copy.deepcopy(header);nir.header.frame_id='infrared'
            info=CameraInfo(header=copy.deepcopy(header),width=64,height=64,distortion_model='plumb_bob',k=[float(v) for v in bundle['rgb_k']],
                p=[100.,0.,32.,0.,0.,100.,32.,0.,0.,0.,1.,0.],r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],d=[0.]*5)
            nir_info=copy.deepcopy(info);nir_info.header.frame_id='infrared'
            if bad_nir:nir_info.width=63
            lidar_header=copy.deepcopy(header);lidar_header.frame_id='lidar';lidar=point_cloud2.create_cloud_xyz32(lidar_header,bundle['lidar'].payload)
            audio_header=copy.deepcopy(header);audio_header.frame_id='audio'
            # Center the target anchor within the synthetic multichannel recording.
            audio_header.stamp.sec,audio_header.stamp.nanosec=divmod(int((stamp_s-.1)*1e9),1000000000)
            audio=AudioFrame(header=audio_header,sample_rate=16000,channels=4,samples=bundle['audio'].payload.ravel().tolist(),
                source_id='fixture_audio',stamp_quality='synthetic_acquisition')
            return request,{'rgb_info':info,'nir_info':nir_info,'infrared':nir,'lidar':lidar,'audio':audio,**({'depth':depth} if include_depth else {})},depth
        def publish(data):
            request,values,_=data
            for name,value in values.items():pubs[name].publish(value)
            request_pub.publish(request)
        try:
            # Discovery completes before one-shot acquisition to avoid duplicate rejection.
            until(lambda:request_pub.get_subscription_count()>0 and all(p.get_subscription_count()>0 for p in pubs.values()))
            first=inputs();publish(first)
            until(lambda:len(results)>=1 and timings)
            value=validate_result(json.loads(results[-1].data));target=value['targets'][0]
            assert target['features']['complete'],value
            assert not value['calibration']['synchronization_verified']
            # No repeated result for repeated source/session/sequence.
            publish(first)
            start=time.monotonic()
            while time.monotonic()-start<.3:executor.spin_once(timeout_sec=.02)
            assert len(results)==1
            # Delayed registered depth must resume the pending request.
            late=inputs(2,include_depth=False);publish(late)
            until(lambda:node.pending is not None)
            pubs['depth'].publish(late[2]);until(lambda:len(results)>=2)
            assert json.loads(results[-1].data)['targets'][0]['target_id']==target['target_id']
            # Bad NIR calibration degrades the target instead of blocking valid RGB/LiDAR/audio.
            partial=inputs(3,bad_nir=True);publish(partial);until(lambda:len(results)>=3)
            value=validate_result(json.loads(results[-1].data));assert not value['targets'][0]['features']['valid_mask'][1]
            # Illegal PCM is counted and isolated.
            bad=AudioFrame(header=partial[0].rgb.header,sample_rate=0,channels=4,source_id='bad')
            bad.header.frame_id='audio'
            pubs['audio'].publish(bad);until(lambda:node.state['input_errors']>0)
            observations=[validate_observation(json.loads(line)) for line in (output/'results.jsonl').read_text().splitlines()]
            assert len(observations)==3 and all(r['result']['safety']['prohibit_automatic_execution'] for r in observations)
            report=dict(passed=True,checks=8,scope='isolated_ros_synthetic_inputs_real_unibind' if args.real_model else 'isolated_ros_synthetic_inputs_fixture_encoder',
                real_pretrained_model=args.real_model,inference_device=args.device,results=len(observations),
                estimated_clock_offset_s=args.clock_offset_s,
                robot_connected=False,source_acquisition_preserved=True,hardware_accuracy_claim=False,last_timing=observations[-1]['timing'])
            path=args.output;path.parent.mkdir(parents=True,exist_ok=True)
            (path.parent/'ros_results.jsonl').write_text('\n'.join(json.dumps(r,allow_nan=False) for r in observations)+'\n')
            path.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
        finally:
            executor.remove_node(node);executor.remove_node(probe);executor.shutdown();node.destroy_node();probe.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
