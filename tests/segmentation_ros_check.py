"""Isolated ROS contract test with an explicitly synthetic detector, never a live fusion producer."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/rtabmap_semantic'))
from rtabmap_semantic.segmentation_node import SegmentationNode
from rtabmap_semantic_msgs.msg import (SegmentationFrame, SegmentationFusionRequest,
    SegmentationFusionCandidate, SegmentationRefinement)


class FixtureDetector:
    backend = 'yolov8'
    model = SimpleNamespace(names={0:'person',1:'chair'})

    def detect(self, image):
        mask = np.zeros(image.shape[:2], np.uint8)
        mask[5:20, 10:30] = 1
        return [dict(mask=mask, class_id=0, label='person',confidence=.9,bbox_xyxy=[10,5,30,20])]


def main():
    rclpy.init()
    fixture = tempfile.NamedTemporaryFile()
    fixture.write(b'explicit-synthetic-contract-fixture'); fixture.flush()
    config = dict(model_path=fixture.name, rgb_topic='/segmentation_contract/image',
        fusion_mode='guarded', approved_fusion_providers=['contract-fixture'],
        fusion_timeout_s=1.2, max_input_age_s=2.0, max_inference_hz=10.0)
    config['rgb_clock_offset_s']=60.0
    node = SegmentationNode(FixtureDetector(), parameter_overrides=[Parameter(k,value=v) for k,v in config.items()])
    # Transport fixture only; this is not an accuracy report and is never loaded by production code.
    node.evaluation_report=dict(fusion_enabled=True,report_sha256='synthetic-contract-only',confidence_threshold=.8)
    probe = rclpy.create_node('segmentation_contract_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node); executor.add_node(probe)
    received = {key:[] for key in ('baseline','selected','fusion_request','fusion_decision','view','status')}
    subs=[]
    for key, type_ in [('baseline',SegmentationFrame),('selected',SegmentationFrame),
            ('fusion_request',SegmentationFusionRequest),('fusion_decision',String),('view',String),('status',String)]:
        prefix = '/miti/segmentation/' if key in ('view','status') else '/perception/segmentation/'
        subs.append(probe.create_subscription(type_,prefix+key,received[key].append,10))
    image_pub = probe.create_publisher(Image,config['rgb_topic'],1)
    candidate_pub = probe.create_publisher(SegmentationFusionCandidate,'/perception/segmentation/candidate',4)

    def until(predicate, seconds=4, action=None):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:
            if action: action()
            executor.spin_once(timeout_sec=.02)
        assert predicate(), 'ROS condition timed out'

    try:
        bridge=CvBridge()
        image=bridge.cv2_to_imgmsg(np.full((40,64,3),100,np.uint8),encoding='rgb8')
        image.header.stamp=probe.get_clock().now().to_msg()
        image.header.stamp.sec-=60
        image.header.frame_id='fixture_camera'
        until(lambda:received['fusion_request'] and received['selected'] and received['baseline'],action=lambda:image_pub.publish(image))
        baseline=received['baseline'][0]
        assert serialize_message(baseline)==serialize_message(received['selected'][0])
        request=received['fusion_request'][0]
        assert request.rgb.header==baseline.header
        assert request.class_ids.tolist()==[0,1]
        assert bytes(request.rgb.data)==bytes(image.data)
        classes=bridge.imgmsg_to_cv2(baseline.class_map)
        assert classes.shape==(40,64) and classes[0,0]==-1 and classes[5,10]==0
        candidate=SegmentationFusionCandidate(header=deepcopy(baseline.header),session_id=baseline.session_id,
            sequence=baseline.sequence,model_sha256=baseline.model_sha256,vocabulary_version=baseline.vocabulary_version,
            provider_id='contract-fixture',calibration_version='synthetic-test-only',score_calibration_version='temperature:synthetic-contract-only')
        stamp=baseline.header.stamp.sec+baseline.header.stamp.nanosec/1e9
        candidate.refinements=[SegmentationRefinement(instance_id=baseline.instances[0].instance_id,class_id=1,score=.95,
            valid_mask=[True,True,False,False],weights=[.5,.5,0.,0.],source_stamps=[stamp,stamp,0.,0.],
            time_alignment_valid=True,spatial_alignment_valid=True)]
        wrong=deepcopy(candidate); wrong.header.frame_id='wrong_camera'
        candidate_pub.publish(wrong)
        until(lambda:received['fusion_decision'])
        assert json.loads(received['fusion_decision'][-1].data)['reason']=='mismatched_frame_id'
        assert all(m.result_source=='rgb_yolo' for m in received['selected'])
        fixture_report=node.evaluation_report;node.evaluation_report=None
        candidate_pub.publish(candidate)
        until(lambda:json.loads(received['fusion_decision'][-1].data)['reason']=='heldout_evaluation_required')
        assert all(m.result_source=='rgb_yolo' for m in received['selected'])
        node.evaluation_report=fixture_report
        candidate_pub.publish(candidate)
        until(lambda:any(m.result_source=='fusion' for m in received['selected']))
        fused=next(m for m in received['selected'] if m.result_source=='fusion')
        assert fused.revision==1 and fused.instances[0].class_id==1
        assert bytes(fused.instances[0].mask.data)==bytes(baseline.instances[0].mask.data)
        assert fused.instances[0].confidence==baseline.instances[0].confidence
        assert bytes(fused.instance_map.data)==bytes(baseline.instance_map.data)
        until(lambda:any(json.loads(m.data).get('state')=='input_stale' for m in received['status']))
        candidate_pub.publish(candidate)
        until(lambda:received['fusion_decision'] and json.loads(received['fusion_decision'][-1].data)['reason']=='unknown_or_expired_frame')
        report=dict(passed=True,synthetic_contract_test=True,real_model_accuracy_test=False,
            baseline_passthrough_identical=True,fusion_request_preserves_rgb=True,wrong_frame_rejected=True,
            guarded_label_refinement=True,geometry_and_yolo_score_preserved=True,stale_candidate_rejected=True,
            input_disconnect_reported=True)
        report['guarded_without_evaluation_preserves_baseline']=True
        report['remote_clock_offset_preserves_acquisition_stamp']=True
        target=ROOT/'logs/segmentation_validation/ros_contract.json'
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report))
    finally:
        executor.remove_node(node);executor.remove_node(probe)
        node.destroy_node();probe.destroy_node();executor.shutdown();rclpy.shutdown();fixture.close()


if __name__=='__main__':
    main()
