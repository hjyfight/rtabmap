"""Bounded, asynchronous RGB/NIR UniBind fusion with independent baselines and capture."""
import base64
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rtabmap_semantic_msgs.msg import (SegmentationFusionRequest,SegmentationImageRequest,
    SegmentationFrame,SegmentationFusionCandidate,SegmentationRefinement)
from .fusion_geometry import camera_dict,rectified_reference_transform,calibration_version
from .object_fusion import TaskVocabulary,ObjectFusionEngine,digest
from .segmentation_messages import read_instances
from .segmentation import render_images
from .fusion_preview import build_preview
from .labels import chinese_label
from .fusion_evaluation import load_passed_report,softmax,policy_prediction


def seconds(header):return header.stamp.sec+header.stamp.nanosec/1e9


class FusionNode(Node):
    def __init__(self,engine=None):
        super().__init__('segmentation_fusion')
        defaults=dict(task_path='config/segmentation_fusion_task.json',device='cpu',cpu_threads=2,
            rgb_info_topic='/camera/color/camera_info',nir_topic='/camera/infra2/image_rect_raw',
            nir_info_topic='/camera/infra2/camera_info',depth_topic='/camera/depth_registered/image_rect',
            sync_tolerance_s=.05,min_period_s=5.,max_objects=8,min_coverage=.35,
            buffer_size=64,capture_directory='',capture_max_frames=60,capture_period_s=5.,evaluation_report='')
        self.config={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        c=self.config
        if not 2<=c['buffer_size']<=128 or not 1<=c['max_objects']<=16 or not 0<c['sync_tolerance_s']<=.2:
            raise ValueError('Invalid fusion buffer/object/synchronization limits')
        if not 0<c['min_coverage']<=1 or c['min_period_s']<0 or c['capture_max_frames']<0 or c['capture_period_s']<0:
            raise ValueError('Invalid fusion/capture limits')
        self.task=TaskVocabulary(c['task_path']);self.bridge=CvBridge()
        self.evaluation=load_passed_report(c['evaluation_report']) if c['evaluation_report'] else None
        self.status_pub=self.create_publisher(String,'/miti/segmentation/fusion_status',2)
        self.view_pub=self.create_publisher(String,'/miti/segmentation/fusion_view',1)
        self.candidate_pub=self.create_publisher(SegmentationFusionCandidate,'/perception/segmentation/candidate',2)
        self.nir_request_pub=self.create_publisher(SegmentationImageRequest,'/perception/segmentation/nir_request',1)
        self.buffers={k:deque(maxlen=c['buffer_size']) for k in ('nir','depth')}
        self.last_inputs={};self.last_request=None;self.last_result_at=None
        self.infos={};self.transforms={}
        self.pending_requests=deque(maxlen=c['buffer_size'])
        self.waiting=self.future=None
        self.last_start=self.last_capture=-float('inf')
        self.seen=deque(maxlen=128)
        self.state='loading_model';self.reason='';self.completed=0;self.captured=0
        self.last_result=None;self.last_error=''
        self.capture_root=Path(c['capture_directory']) if c['capture_directory'] else None
        if self.capture_root:
            self.capture_root.mkdir(parents=True,exist_ok=True)
            (self.capture_root/'classes.json').write_text(json.dumps(self.task.spec,ensure_ascii=False,indent=2)+'\n')
            (self.capture_root/'annotations').mkdir(exist_ok=True)
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.engine=engine
        self.load_future=None if engine else self.pool.submit(self.load_engine)
        self.create_subscription(Image,c['nir_topic'],lambda m:self.receive_input('nir',m),qos_profile_sensor_data)
        self.create_subscription(Image,c['depth_topic'],lambda m:self.receive_input('depth',m),qos_profile_sensor_data)
        for key,topic in [('rgb',c['rgb_info_topic']),('nir',c['nir_info_topic'])]:
            self.create_subscription(CameraInfo,topic,lambda m,k=key:self.infos.update({k:camera_dict(m)}),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf_static',self.receive_tf,QoSProfile(depth=100,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(SegmentationFusionRequest,'/perception/segmentation/fusion_request',self.receive,1)
        self.create_subscription(SegmentationFrame,'/perception/segmentation/nir_baseline',self.receive_nir,2)
        self.create_timer(.05,self.poll);self.create_timer(1.,self.status)

    def load_engine(self):
        from rtabmap_features.backbone import FrozenEncoder
        encoder=FrozenEncoder('.tools/unibind','models/unibind/pretrained_weights.pt',
            device=self.config['device'],threads=self.config['cpu_threads'],retain_text=True)
        return ObjectFusionEngine(encoder,self.task,'models/unibind/task_prototypes',
            self.config['max_objects'],self.config['min_coverage'])

    def receive_tf(self,message):
        for t in message.transforms:
            if not (t.header.frame_id.startswith('camera_') and t.child_frame_id.startswith('camera_')):continue
            self.transforms[t.child_frame_id]=dict(parent=t.header.frame_id,child=t.child_frame_id,
                translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                rotation=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])

    def receive(self,message):
        self.last_request=time.monotonic()
        key=(message.baseline.session_id,message.baseline.sequence)
        if key in self.seen or any((r.baseline.session_id,r.baseline.sequence)==key
                for r,_ in self.pending_requests):return
        if self.pending_requests and self.pending_requests[-1][0].baseline.session_id!=key[0]:
            self.pending_requests.clear()
        self.pending_requests.append((message,self.last_request))

    def newest_bundle(self):
        """Select the newest complete pair even when registered depth arrives late."""
        now=time.monotonic()
        while self.pending_requests and now-self.pending_requests[0][1]>5:
            self.pending_requests.popleft()
        newest_error=None
        for index in range(len(self.pending_requests)-1,-1,-1):
            request,_=self.pending_requests[index]
            try:bundle=self.bundle(request)
            except ValueError as error:
                if newest_error is None:newest_error=str(error)
                continue
            # Discard the selected frame and everything it supersedes. Retain
            # newer requests whose same-frame sensors have not arrived yet.
            for _ in range(index+1):self.pending_requests.popleft()
            return bundle
        self.reason=newest_error or '等待新的 RGB 融合请求'
        self.state='waiting_inputs'
        return None

    def receive_input(self,kind,message):
        now=time.monotonic()
        if kind in self.last_inputs and now-self.last_inputs[kind]>5:self.buffers[kind].clear()
        self.buffers[kind].append(message);self.last_inputs[kind]=now

    def bundle(self,request):
        if set(self.infos)!= {'rgb','nir'}:raise ValueError('等待 RGB/近红外相机内参')
        rgb_info,nir_info=self.infos['rgb'],self.infos['nir']
        if request.rgb.header!=request.baseline.header:raise ValueError('RGB request header mismatch')
        if request.rgb.header.frame_id!=rgb_info['frame_id']:raise ValueError('RGB optical frame mismatch')
        anchor=seconds(request.baseline.header);matched={}
        for kind in ('nir','depth'):
            name='注册深度' if kind=='depth' else '近红外'
            if not self.buffers[kind]:raise ValueError('等待'+name+'输入')
            if time.monotonic()-self.last_inputs.get(kind,time.monotonic())>5:
                raise ValueError(name+'输入已停止更新，等待恢复')
            message=min(self.buffers[kind],key=lambda m:abs(seconds(m.header)-anchor))
            if abs(seconds(message.header)-anchor)>self.config['sync_tolerance_s']:
                raise ValueError('等待同帧 '+kind+'：采集时间差超过容差')
            matched[kind]=message
        if matched['depth'].header.frame_id!=rgb_info['frame_id']:
            raise ValueError('Depth must be registered in the RGB optical frame')
        if matched['nir'].encoding!='mono8':raise ValueError('NIR requires mono8')
        if (matched['nir'].width,matched['nir'].height)!=(nir_info['width'],nir_info['height']):
            raise ValueError('NIR image/calibration dimensions mismatch')
        transform=rectified_reference_transform(self.transforms,rgb_info['frame_id'],nir_info,matched['nir'].header.frame_id)
        depth=self.bridge.imgmsg_to_cv2(matched['depth'],desired_encoding='passthrough')
        if matched['depth'].encoding=='16UC1':depth=depth.astype(np.float32)*.001
        elif matched['depth'].encoding!='32FC1':raise ValueError('Unknown depth units')
        return dict(request=request,rgb=self.bridge.imgmsg_to_cv2(request.rgb,desired_encoding='rgb8').copy(),
            nir=self.bridge.imgmsg_to_cv2(matched['nir'],desired_encoding='mono8').copy(),depth_m=depth.copy(),
            nir_message=matched['nir'],rgb_info=deepcopy(rgb_info),nir_info=deepcopy(nir_info),transform=transform,
            rgb_instances=read_instances(request.baseline,self.bridge),native_names=dict(zip(request.class_ids,request.class_names)),
            stamps=dict(rgb=anchor,nir=seconds(matched['nir'].header),depth=seconds(matched['depth'].header)),
            calibration_version=calibration_version(rgb_info,nir_info,transform))

    def receive_nir(self,message):
        if not self.waiting:return
        bundle,started=self.waiting;base=bundle['request'].baseline
        if (message.session_id,message.sequence)!=(base.session_id,base.sequence):return
        if (message.header!=bundle['nir_message'].header or message.vocabulary_version!=base.vocabulary_version or
                (message.width,message.height)!=(bundle['nir'].shape[1],bundle['nir'].shape[0]) or
                message.result_source!='nir_yolo'):
            self.reason='独立近红外结果的来源或词表不匹配';self.waiting=None;return
        bundle['nir_instances']=read_instances(message,self.bridge)
        bundle['nir_baseline']=message
        self.waiting=None;self.state='inferencing';self.reason='UniBind 目标级特征融合'
        self.future=self.pool.submit(self.compute,bundle)

    def compute(self,bundle):
        start=time.monotonic();result=self.engine.fuse(bundle)
        result['inference_s']=time.monotonic()-start
        base=bundle['request'].baseline;nir_base=bundle['nir_baseline']
        result['timings_s']['nir_inference']=nir_base.inference_seconds
        result['runtime_device']=self.engine.encoder.provenance.get('inference_device','cpu')
        rgb_parameters=json.loads(base.provenance_json);nir_parameters=json.loads(nir_base.provenance_json)
        result['pipeline_sha256']=digest(dict(policy=result['policy_sha256'],rgb_model=base.model_sha256,
            nir_model=nir_base.model_sha256,rgb_parameters={k:rgb_parameters.get(k) for k in ('detector_backend','confidence','image_size','device')},
            nir_parameters={k:nir_parameters.get(k) for k in ('confidence','image_size','device')},
            fusion_device=result['runtime_device'],
            sync_tolerance_s=self.config['sync_tolerance_s'],calibration=bundle['calibration_version']))
        return bundle,result

    def poll(self):
        if self.load_future is not None:
            if not self.load_future.done():return
            future,self.load_future=self.load_future,None
            try:self.engine=future.result();self.state='waiting_inputs'
            except Exception as error:self.last_error=str(error);self.state='error';return
        if self.engine is None:return
        if self.future is not None and self.future.done():
            future,self.future=self.future,None
            try:
                bundle,result=future.result();self.publish_result(bundle,result)
                self.completed+=1;self.last_error='';self.state='ready'
                self.reason='真实融合已计算'
            except Exception as error:
                self.last_error=str(error);self.state='error';self.get_logger().error(str(error))
        if self.waiting and time.monotonic()-self.waiting[1]>20:
            self.waiting=None;self.reason='独立近红外分割响应超时'
        if self.future or self.waiting or not self.pending_requests or time.monotonic()-self.last_start<self.config['min_period_s']:return
        bundle=self.newest_bundle()
        if bundle is None:return
        request=bundle['request'];self.last_start=time.monotonic()
        self.seen.append((request.baseline.session_id,request.baseline.sequence))
        self.waiting=(bundle,time.monotonic());self.state='waiting_nir';self.reason='独立近红外分割'
        self.nir_request_pub.publish(SegmentationImageRequest(session_id=request.baseline.session_id,
            sequence=request.baseline.sequence,image=bundle['nir_message']))

    def publish_result(self,bundle,result):
        base=bundle['request'].baseline
        approved=self.evaluation if self.evaluation and self.evaluation['pipeline_sha256']==result['pipeline_sha256'] else None
        if self.evaluation and not approved:
            raise ValueError('Evaluation pipeline differs from the current model, calibration or fusion parameters')
        temperature=approved['temperature'] if approved else .07
        threshold=approved['confidence_threshold'] if approved else 0.
        enabled=approved['fusion_enabled'] if approved else True
        result['fused_map'],_=policy_prediction(result,result['objects'],temperature,threshold,enabled)
        candidate=SegmentationFusionCandidate(header=deepcopy(base.header),session_id=base.session_id,
            sequence=base.sequence,model_sha256=base.model_sha256,vocabulary_version=base.vocabulary_version,
            provider_id='unibind_rgb_nir_v1',calibration_version=bundle['calibration_version'],score_calibration_version='unverified')
        if approved:candidate.score_calibration_version='temperature:'+approved['report_sha256']
        for index,row in enumerate(result['objects']):
            score=float(softmax(result['logits'][index,2],temperature).max())
            eligible=row['eligible'] and enabled and score>=threshold
            row['selected_task_id']=row['predicted_task_id'] if eligible else row['baseline_task_id']
            row['selected_score']=score
            if eligible:
                candidate.refinements.append(SegmentationRefinement(instance_id=row['instance_id'],
                    class_id=row['native_class_id'],score=score,valid_mask=[True,True,False,False],
                    weights=row['weights'],source_stamps=[bundle['stamps']['rgb'],bundle['stamps']['nir'],0.,0.],
                    spatial_alignment_valid=True,time_alignment_valid=True))
        if candidate.refinements:self.candidate_pub.publish(candidate)
        preview=build_preview(bundle,result,{r.instance_id:r.class_id for r in candidate.refinements})
        images={}
        for name,pixels in [('rgb',bundle['rgb']),
                ('nir',cv2.cvtColor(result['nir_warped'],cv2.COLOR_GRAY2RGB)),('fused',bundle['rgb'])]:
            labels=preview['maps'][name]
            overlay,_=render_images(pixels,labels)
            ok,encoded=cv2.imencode('.jpg',cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,82])
            if not ok:raise RuntimeError('Fusion preview encoding failed')
            images[name]=base64.b64encode(encoded).decode()
        accuracy='validated_fusion' if approved and enabled else 'validated_baseline' if approved else 'unlabeled'
        rgb_parameters=json.loads(base.provenance_json)
        source_age=time.time()-seconds(base.header)-rgb_parameters.get('rgb_clock_offset_s',0.)
        view=dict(schema=1,experimental=True,accuracy_status=accuracy,session_id=base.session_id,sequence=base.sequence,
            stamp=dict(sec=base.header.stamp.sec,nanosec=base.header.stamp.nanosec),published_at=time.time(),
            source_age_s=max(0.,source_age),
            width=base.width,height=base.height,inference_s=result['inference_s'],coverage=result['coverage'],
            timings_s=result['timings_s'],runtime_device=result['runtime_device'],
            objects=result['objects'],class_names=[chinese_label(n) for n in self.task.names],images=images,
            image_labels=preview['image_labels'],image_instance_counts=preview['instance_counts'],
            image_label_vocabulary='native_yolo',
            pipeline_sha256=result['pipeline_sha256'],modalities=['rgb','nir'],
            selected_source='guarded_candidate' if approved and enabled else 'rgb_yolo',
            evaluation_report_sha256=approved['report_sha256'] if approved else '')
        self.view_pub.publish(String(data=json.dumps(view,ensure_ascii=False,allow_nan=False)))
        self.last_result=dict(sequence=base.sequence,objects=len(result['objects']),candidate_objects=len(candidate.refinements),
            coverage=result['coverage'],inference_s=result['inference_s'],pipeline_sha256=result['pipeline_sha256'],
            timings_s=result['timings_s'],runtime_device=result['runtime_device'],
            accuracy_status=accuracy,session_id=base.session_id)
        self.last_result_at=time.monotonic()
        self.capture(bundle,result)

    def capture(self,bundle,result):
        if self.capture_root is None or self.captured>=self.config['capture_max_frames'] or time.monotonic()-self.last_capture<self.config['capture_period_s']:return
        base=bundle['request'].baseline
        sample_id=digest([base.session_id,base.sequence])[:20]
        target=self.capture_root/'samples'/sample_id
        target.mkdir(parents=True,exist_ok=False)
        if not cv2.imwrite(str(target/'rgb.png'),cv2.cvtColor(bundle['rgb'],cv2.COLOR_RGB2BGR)) or not cv2.imwrite(str(target/'nir.png'),bundle['nir']):
            raise RuntimeError('Capture image write failed')
        masks={};instance_records={}
        for modality in ('rgb','nir'):
            instance_records[modality]=[]
            for row in bundle[modality+'_instances']:
                key=modality+'_mask_'+str(row['instance_id']);masks[key]=row['mask']
                instance_records[modality].append({k:v for k,v in row.items() if k!='mask'} | dict(mask_key=key))
        np.savez_compressed(target/'arrays.npz',depth_m=bundle['depth_m'],
            **masks,**{k:result[k] for k in ('rgb_map','nir_map','fused_map','eval_mask','features','fused_features','logits','instance_map','nir_native_map')})
        metadata=dict(schema=1,sample_id=sample_id,session_id=base.session_id,sequence=base.sequence,
            group_id=self.capture_root.name,split='unassigned',annotation_status='unlabeled',
            annotation_path='annotations/'+sample_id+'.png',stamps=bundle['stamps'],
            source_images=dict(rgb='rgb.png',nir='nir.png'),rgb_info=bundle['rgb_info'],nir_info=bundle['nir_info'],
            nir_from_rgb=bundle['transform'].tolist(),calibration_version=bundle['calibration_version'],
            task_sha256=self.task.sha256,pipeline_sha256=result['pipeline_sha256'],policy=result['policy'],
            rgb_model=base.model_sha256,nir_model=bundle['nir_baseline'].model_sha256,
            rgb_provenance=json.loads(base.provenance_json),nir_provenance=json.loads(bundle['nir_baseline'].provenance_json),
            native_names=bundle['native_names'],instances=instance_records,
            coverage=result['coverage'],objects=result['objects'],inference_s=result['inference_s'],
            timings_s=result['timings_s'],runtime_device=result['runtime_device'],
            modalities=['rgb','nir'],lidar_used=False,audio_used=False)
        (target/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        with (self.capture_root/'manifest.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(sample_id=sample_id,path='samples/'+sample_id,group_id=self.capture_root.name,
                split='unassigned',annotation='annotations/'+sample_id+'.png'))+'\n')
        self.captured+=1;self.last_capture=time.monotonic()

    def status(self):
        now=time.monotonic()
        stale=any(now-self.last_inputs.get(k,-float('inf'))>5 for k in ('nir','depth'))
        state='input_stale' if stale and self.completed and not self.future else self.state
        self.status_pub.publish(String(data=json.dumps(dict(schema=1,state=state,reason=self.reason,
            last_error=self.last_error,completed=self.completed,captured=self.captured,
            capture_directory=str(self.capture_root or ''),last_result=self.last_result,
            last_result_age_s=None if self.last_result_at is None else now-self.last_result_at,
            accuracy_status=self.last_result['accuracy_status'] if self.last_result else 'unlabeled',
            formal_output='see_segmentation_selected',published_at=time.time()),ensure_ascii=False)))

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init();node=None
    try:node=FusionNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
