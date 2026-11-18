"""Isolated end-to-end RGB/NIR fusion/capture transport with explicit synthetic models."""
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
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/rtabmap_semantic'))
from rtabmap_semantic.segmentation_node import SegmentationNode
from rtabmap_semantic.nir_segmentation_node import NirSegmentationNode
from rtabmap_semantic.segmentation_fusion_node import FusionNode
from rtabmap_semantic.object_fusion import TaskVocabulary,ObjectFusionEngine
from rtabmap_semantic_msgs.msg import SegmentationFrame,SegmentationFusionCandidate,SegmentationFusionRequest
from test_segmentation_fusion import FakeEncoder,camera


class Detector:
    backend='yoloe_pf'
    model=SimpleNamespace(names={0:'person',1:'robot'})
    def __init__(self,class_id):self.class_id=class_id;self.images=[]
    def detect(self,image):
        self.images.append(image.copy())
        mask=np.zeros(image.shape[:2],np.uint8);mask[2:14,2:14]=1
        return [dict(mask=mask,bbox_xyxy=[2,2,14,14],class_id=self.class_id,
            label=self.model.names[self.class_id],confidence=.9)]


def main():
    rclpy.init();bridge=CvBridge();executor=SingleThreadedExecutor();nodes=[]
    with tempfile.TemporaryDirectory() as directory:
        root=Path(directory);model=root/'fixture.pt';model.write_bytes(b'explicit-synthetic-fusion-fixture')
        task=TaskVocabulary(ROOT/'config/segmentation_fusion_task.json')
        engine=ObjectFusionEngine(FakeEncoder(),task,root/'cache')
        rgb_detector,nir_detector=Detector(0),Detector(1)
        parameters=lambda c:[Parameter(k,value=v) for k,v in c.items()]
        rgb=SegmentationNode(rgb_detector,parameter_overrides=parameters(dict(model_path=str(model),
            rgb_topic='/fusion_contract/rgb',detector_backend='yoloe_pf',fusion_mode='shadow')))
        nir=NirSegmentationNode(nir_detector,parameter_overrides=parameters(dict(model_path=str(model))))
        # The engine is a fixture; production never loads this test's embeddings.
        fusion=FusionNode(engine)
        fusion.config.update(min_period_s=0.,capture_period_s=0.,capture_max_frames=1)
        fusion.capture_root=root/'capture';fusion.capture_root.mkdir();(fusion.capture_root/'annotations').mkdir()
        probe=rclpy.create_node('segmentation_fusion_contract_probe');nodes=[rgb,nir,fusion,probe]
        for node in nodes:executor.add_node(node)
        received={k:[] for k in ('baseline','selected','nir_baseline','candidate','fusion_view')};subs=[]
        for key in received:
            type_=String if key=='fusion_view' else SegmentationFusionCandidate if key=='candidate' else SegmentationFrame
            topic='/miti/segmentation/fusion_view' if key=='fusion_view' else '/perception/segmentation/'+key
            subs.append(probe.create_subscription(type_,topic,received[key].append,10))
        pubs={key:probe.create_publisher(Image,topic,2) for key,topic in
            [('rgb','/fusion_contract/rgb'),('nir',fusion.config['nir_topic']),('depth',fusion.config['depth_topic'])]}
        infos={key:probe.create_publisher(CameraInfo,fusion.config[key+'_info_topic'],2) for key in ('rgb','nir')}
        stamp=probe.get_clock().now().to_msg();frame='camera_fixture_optical_frame'
        rgb_image=bridge.cv2_to_imgmsg(np.full((18,18,3),100,np.uint8),encoding='rgb8')
        nir_pixels=(np.indices((18,18))[0]*10).astype(np.uint8)
        nir_image=bridge.cv2_to_imgmsg(nir_pixels,encoding='mono8')
        depth_image=bridge.cv2_to_imgmsg(np.full((18,18),1000,np.uint16),encoding='16UC1')
        for image in (rgb_image,nir_image,depth_image):image.header.stamp=deepcopy(stamp);image.header.frame_id=frame
        info=CameraInfo();info.header=deepcopy(rgb_image.header);info.width=info.height=18
        calibration=camera(18,18)
        for key in ('k','p','r','d'):setattr(info,key,[float(v) for v in calibration[key]])
        info.distortion_model='plumb_bob'
        def publish():
            for pub in infos.values():pub.publish(info)
            for key,image in [('rgb',rgb_image),('nir',nir_image),('depth',depth_image)]:pubs[key].publish(image)
        def until(predicate,seconds=5.,action=None):
            deadline=time.monotonic()+seconds
            while not predicate() and time.monotonic()<deadline:
                if action:action()
                executor.spin_once(timeout_sec=.02)
            assert predicate(),'Fusion ROS condition timed out'
        try:
            until(lambda:fusion.completed>=1 and received['candidate'] and received['fusion_view'],action=publish)
            base=received['baseline'][0]
            assert serialize_message(base)==serialize_message(received['selected'][0])
            assert all(m.result_source=='rgb_yolo' for m in received['selected'])
            assert np.array_equal(nir_detector.images[0],np.repeat(nir_pixels[:,:,None],3,axis=2))
            view=json.loads(received['fusion_view'][0].data)
            assert view['accuracy_status']=='unlabeled' and view['objects'][0]['selected_task_id']==1
            assert view['image_labels']['rgb'][0]['class_id']==0
            assert view['image_labels']['nir'][0]['class_id']==1
            assert view['image_labels']['fused'][0]['class_id']==1
            assert 0<=view['source_age_s']<5
            sample=next((fusion.capture_root/'samples').iterdir())
            metadata=json.loads((sample/'metadata.json').read_text())
            with np.load(sample/'arrays.npz',allow_pickle=False) as arrays:
                assert arrays['rgb_map'][5,5]==0 and arrays['nir_map'][5,5]==1 and arrays['fused_map'][5,5]==1
                assert arrays['features'].shape==(1,2,1024)
                assert np.array_equal(arrays['rgb_mask_1'],np.ones((12,12),np.uint8))
            assert metadata['nir_provenance']['input_modality']=='nir'
            assert not list((fusion.capture_root/'annotations').iterdir())
            # A mismatched acquisition time cannot silently produce a second bundle.
            request=SimpleNamespace(rgb=rgb_image,baseline=base)
            original=fusion.buffers['nir'].copy();fusion.buffers['nir'].clear()
            wrong=deepcopy(nir_image);wrong.header.stamp.sec+=1;fusion.buffers['nir'].append(wrong)
            try:fusion.bundle(request)
            except ValueError as error:assert '采集时间差' in str(error)
            else:raise AssertionError('Unsynchronized NIR was accepted')
            fusion.buffers['nir']=original
            fusion.last_inputs['depth']=time.monotonic()-6
            try:fusion.bundle(request)
            except ValueError as error:assert '注册深度输入已停止更新' in str(error)
            else:raise AssertionError('Stopped depth stream was treated as a timestamp mismatch')
            fusion.receive_input('depth',depth_image)
            assert len(fusion.buffers['depth'])==1,'Old depth cache must clear after a stream resumes'
            # Fast RGB must not starve fusion by replacing the only request
            # before that request's registered depth has arrived.
            fusion.pending_requests.clear()
            requests=[]
            for index in range(3):
                stamp_ns=base.header.stamp.sec*1_000_000_000+base.header.stamp.nanosec+index*200_000_000
                paired=deepcopy(base);paired.sequence=10+index
                paired.header.stamp.sec,paired.header.stamp.nanosec=divmod(stamp_ns,1_000_000_000)
                image=deepcopy(rgb_image);image.header=deepcopy(paired.header)
                requests.append(SegmentationFusionRequest(baseline=paired,rgb=image,
                    class_ids=list(rgb.labels),class_names=list(rgb.labels.values()),deadline_seconds=3.))
                fusion.receive(requests[-1])
            fusion.receive(requests[-1])
            assert len(fusion.pending_requests)==3,'Duplicate requests must not grow the cache'
            def sensors_for(request):
                for kind,source in [('nir',nir_image),('depth',depth_image)]:
                    image=deepcopy(source);image.header=deepcopy(request.baseline.header)
                    fusion.buffers[kind].clear();fusion.receive_input(kind,image)
            sensors_for(requests[1])
            matched=fusion.newest_bundle()
            assert matched['request'].baseline.sequence==11,'Use the newest complete sensor pair'
            assert [r.baseline.sequence for r,_ in fusion.pending_requests]==[12]
            assert fusion.newest_bundle() is None,'Never reuse an older sensor image for a fresh RGB frame'
            sensors_for(requests[2])
            assert fusion.newest_bundle()['request'].baseline.sequence==12
            assert not fusion.pending_requests
            fusion.pending_requests.append((requests[2],time.monotonic()-6))
            assert fusion.newest_bundle() is None and not fusion.pending_requests
            # Guarded inference retains the reference despite a newer input. This
            # fixture report tests scheduling only; it cannot enable production.
            rgb.config['fusion_mode']='guarded';rgb.config['max_inference_hz']=30.
            rgb.evaluation_report=dict(fusion_enabled=True)
            newer=deepcopy(rgb_image);newer.header.stamp.nanosec+=1;rgb.on_image(newer)
            rgb.poll();assert rgb.pending is not None and rgb.future is None
            rgb.latest['revision']=1;rgb.poll();assert rgb.pending is None and rgb.future is not None
            report=dict(passed=True,synthetic_contract_test=True,real_model_accuracy_test=False,
                independent_nir_input=True,paired_fusion_transport=True,shadow_preserves_baseline=True,
                raw_capture_and_instance_masks=True,no_pseudo_ground_truth=True,
                unsynchronized_input_rejected=True,guarded_retains_reference_frame=True)
            report['stopped_depth_reported_and_old_cache_cleared']=True
            report['delayed_depth_selects_newest_complete_pair']=True
            target=ROOT/'logs/segmentation_validation/fusion_ros_contract.json'
            target.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
        finally:
            for node in nodes:executor.remove_node(node);node.destroy_node()
            executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
