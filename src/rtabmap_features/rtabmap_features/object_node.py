"""Bounded target-level multimodal ROS worker; no sensor drivers or robot controls."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import json
import hashlib
import os
from pathlib import Path
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
from rtabmap_features_msgs.msg import AudioFrame
from rtabmap_semantic_msgs.msg import SegmentationFusionRequest
from rtabmap_semantic.fusion_geometry import (camera_dict,rectified_reference_transform,projection,
                                            project_image,transform_matrix)
from rtabmap_semantic.segmentation_messages import read_instances
from .backbone import FrozenEncoder
from .calibration import load_calibration,rigid_matrix
from .fusion import MODALITIES,Sample,acquisition_gap
from .quality import SourceMonitor
from .object_multimodal import ObjectMultimodalEngine
from .object_policy import validate_task
from .result_journal import ResultJournal,publication_observation
from .standard_interface import seal_result,validate_result


def stamp_s(header):return header.stamp.sec+header.stamp.nanosec/1e9


class ObjectFusionNode(Node):
    def __init__(self,engine=None):
        super().__init__('object_multimodal_fusion');self.root=Path(os.environ.get('RTABMAP_PROJECT_ROOT',Path.cwd()))
        defaults=dict(device='cuda:0',cpu_threads=2,task_path='config/object_multimodal_task.json',
            calibration_path='config/object_multimodal_calibration.json',state_path='logs/object_multimodal/target_state.json',
            output_topic='/perception/fusion/targets',status_topic='/perception/fusion/status',timing_topic='/perception/fusion/timing',
            journal_path='logs/object_multimodal/results.jsonl',
            sync_tolerance_s=.05,max_input_age_s=3.,min_period_s=1.,lidar_topic='/livox/lidar',
            audio_topic='/multimodal/audio',nir_topic='/camera/infra2/image_rect_raw',
            nir_info_topic='/camera/infra2/camera_info',rgb_info_topic='/camera/color/camera_info',
            depth_topic='/camera/depth_registered/image_rect')
        self.config={key:self.declare_parameter(key,value).value for key,value in defaults.items()}
        for key in ('sync_tolerance_s','max_input_age_s','min_period_s'):
            value=self.config[key]
            if type(value) not in (int,float) or not np.isfinite(value) or value<=0:raise ValueError('Invalid runtime '+key)
        self.calibration=load_calibration(self.root/self.config['calibration_path'])
        self.task=validate_task(json.loads((self.root/self.config['task_path']).read_text()))
        self.bridge=CvBridge();self.infos={};self.transforms={};self.lock=threading.Lock()
        self.buffers={n:deque(maxlen=64) for n in ('infrared','depth','lidar','audio')}
        self.publisher=self.create_publisher(String,self.config['output_topic'],2)
        self.status_publisher=self.create_publisher(String,self.config['status_topic'],2)
        self.timing_publisher=self.create_publisher(String,self.config['timing_topic'],2)
        self.journal=ResultJournal(self.root/self.config['journal_path']);self.last_published=None;self.active_bundle=None
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        self.monitor=SourceMonitor(self.config['max_input_age_s']);self.last_sources={};self.pending=None;self.seen=deque(maxlen=128)
        self.tracking_reset_pending=False
        self.pool=ThreadPoolExecutor(max_workers=1);self.engine=engine;self.future=None;self.last_start=-float('inf')
        self.state=dict(state='loading_model' if engine is None else 'waiting_inputs',processed=0,dropped_pending=0,input_errors=0,
                        task_period_s=None,robot_control=False)
        self.loading=self.pool.submit(self.load_engine) if engine is None else None
        self.create_subscription(SegmentationFusionRequest,'/perception/segmentation/fusion_request',self.receive_request,1)
        for modality,topic,kind in [('infrared',self.config['nir_topic'],Image),('depth',self.config['depth_topic'],Image),
                ('lidar',self.config['lidar_topic'],PointCloud2),('audio',self.config['audio_topic'],AudioFrame)]:
            self.create_subscription(kind,topic,lambda m,n=modality:self.receive(n,m),qos_profile_sensor_data)
        for name,key in [('rgb','rgb_info_topic'),('infrared','nir_info_topic')]:
            self.create_subscription(CameraInfo,self.config[key],lambda m,n=name:self.infos.update({n:camera_dict(m)}),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/tf_static',self.receive_tf,QoSProfile(depth=100,
            reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05,self.poll);self.create_timer(1.,self.status)

    def load_engine(self):
        encoder=FrozenEncoder(self.root/'.tools/unibind',self.root/'models/unibind/pretrained_weights.pt',
                              self.config['device'],threads=self.config['cpu_threads'],retain_text=True)
        return ObjectMultimodalEngine(encoder,self.task,self.calibration,self.root/self.config['state_path'])

    def receive_tf(self,message):
        for t in message.transforms:
            self.transforms[t.child_frame_id]=dict(parent=t.header.frame_id,child=t.child_frame_id,
                translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                rotation=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w])

    def receive(self,name,message):
        try:
            # Registered depth keeps the camera acquisition clock and RGB frame.
            sensor=self.calibration['sensors'].get('rgb' if name=='depth' else name,{})
            offset=float(sensor.get('clock_offset_s',0.));stamp=stamp_s(message.header)
            if sensor.get('status')=='measured' and message.header.frame_id!=sensor['frame_id']:
                raise ValueError('Input frame differs from measured calibration')
            if name in ('infrared','depth'):
                if name=='infrared' and message.encoding!='mono8':raise ValueError('NIR must be mono8')
                payload=self.bridge.imgmsg_to_cv2(message,'mono8' if name=='infrared' else 'passthrough').copy()
                if name=='depth':
                    if message.encoding=='16UC1':payload=payload.astype(np.float32)*.001
                    elif message.encoding!='32FC1':raise ValueError('Invalid depth encoding')
                attributes={'encoding':message.encoding}
                sample=Sample(payload,stamp,message.header.frame_id,f'{name}:{stamp:.9f}',attributes=attributes,clock_offset_s=offset)
            elif name=='lidar':
                raw=point_cloud2.read_points(message,field_names=('x','y','z'),skip_nans=False)
                dtype=getattr(raw,'dtype',None)
                payload=np.column_stack([raw[n] for n in ('x','y','z')]) if dtype is not None and dtype.names else np.asarray(list(raw))
                sample=Sample(payload,stamp,message.header.frame_id,f'lidar:{stamp:.9f}',attributes={'units':'m'},clock_offset_s=offset)
            else:
                if not 1<=message.channels<=64 or not 8000<=message.sample_rate<=192000 or len(message.samples)%message.channels:
                    raise ValueError('Invalid audio sample layout')
                payload=np.asarray(message.samples,np.float32).reshape(-1,message.channels)
                sample=Sample(payload,stamp,message.header.frame_id,message.source_id,len(payload)/message.sample_rate,
                    attributes={'sample_rate':message.sample_rate,'stamp_quality':message.stamp_quality},clock_offset_s=offset)
            if name!='depth':
                metadata=self.monitor.observe(name,payload,stamp,sample.source_id,sample.frame_id,time.time(),
                    sample.alignment_stamp,sample.attributes.get('stamp_quality','acquisition_header'),duration_s=sample.duration)
                metadata['present']=True;sample.attributes['source_record']=metadata
                self.last_sources[name]=metadata
                if not metadata['quality']['usable']:
                    if 'clock_regression_or_source_restart' in metadata['continuity']:
                        with self.lock:self.buffers[name].clear()
                    return
            with self.lock:self.buffers[name].append(sample)
        except Exception as error:
            self.state.update(input_errors=self.state['input_errors']+1,last_input_error={'modality':name,'reason':str(error)})

    def receive_request(self,message):
        identity=(message.baseline.session_id,message.baseline.sequence)
        if identity in self.seen:return
        self.seen.append(identity)
        if self.pending is not None:self.state['dropped_pending']+=1
        self.pending=(message,time.monotonic())

    def matched(self,name,anchor):
        with self.lock:values=list(self.buffers[name])
        if not values:return None
        best=min(values,key=lambda s:acquisition_gap(name,s,anchor))
        return best if acquisition_gap(name,best,anchor)<=self.config['sync_tolerance_s'] else None

    def prepare(self,message,received):
        prepared_started=time.monotonic()
        if 'rgb' not in self.infos:raise ValueError('Waiting for RGB CameraInfo')
        rgb_info=self.infos['rgb'];stamp=stamp_s(message.rgb.header);offset=self.calibration['sensors']['rgb']['clock_offset_s'];anchor=stamp+offset
        if message.rgb.header!=message.baseline.header or message.rgb.header.frame_id!=rgb_info['frame_id']:
            raise ValueError('RGB instance/image/calibration mismatch')
        if time.time()-anchor>self.config['max_input_age_s']:raise ValueError('RGB request is stale')
        depth=self.matched('depth',anchor)
        if depth is None or depth.frame_id!=rgb_info['frame_id']:raise ValueError('Waiting for same-frame registered depth')
        rgb=self.bridge.imgmsg_to_cv2(message.rgb,'rgb8').copy()
        if rgb.shape[:2]!=depth.payload.shape or rgb.shape[:2]!=(rgb_info['height'],rgb_info['width']):
            raise ValueError('RGB/depth/CameraInfo dimensions differ')
        rows=read_instances(message.baseline,self.bridge)
        for row in rows:
            mask=np.zeros(rgb.shape[:2],bool);x,y=row['mask_offset'];h,w=row['mask'].shape
            if x<0 or y<0 or x+w>rgb.shape[1] or y+h>rgb.shape[0]:raise ValueError('Invalid target mask extent')
            mask[y:y+h,x:x+w]=row['mask'].astype(bool);row['full_mask']=mask
        sources={name:dict(present=False,modality=name,quality={'usable':False,'score':0.,'grade':'unavailable'},sensor_health='unavailable') for name in MODALITIES}
        bundle=dict(stamp=anchor,sequence=message.baseline.sequence,rgb=rgb,depth_m=depth.payload,
                    rgb_k=rgb_info['k'],rgb_d=rgb_info['d'],rgb_distortion_model=rgb_info['distortion_model'],
                    rgb_frame=rgb_info['frame_id'],instances=rows,sources=sources,received_monotonic=received,
                    processing_started_monotonic=prepared_started)
        for name in ('lidar','audio'):
            sample=self.matched(name,anchor)
            if sample is not None:
                bundle[name]=sample;sources[name]=sample.attributes['source_record']
        nir=self.matched('infrared',anchor)
        if nir is not None and 'infrared' in self.infos:
            info=self.infos['infrared']
            sources['infrared']=nir.attributes['source_record']
            try:
                if nir.payload.shape!=(info['height'],info['width']):raise ValueError('NIR image/CameraInfo dimensions differ')
                transform=rectified_reference_transform(self.transforms,rgb_info['frame_id'],info,nir.frame_id)
                u,v,valid=projection(depth.payload,rgb_info,info,transform)
                bundle.update(nir_warped=project_image(nir.payload,u,v),nir_valid=valid,nir_frame=nir.frame_id)
            except ValueError as error:self.state['nir_association_reason']=str(error)
        rgb_sensor=self.calibration['sensors']['rgb'];lidar_sensor=self.calibration['sensors']['lidar']
        if rgb_sensor['status']=='measured' and rgb_sensor.get('common_from_sensor') is not None:
            if rgb_sensor['frame_id']!=rgb_info['frame_id']:
                raise ValueError('RGB frame differs from measured calibration')
            base_from_rgb=rigid_matrix(rgb_sensor['common_from_sensor'])
            if lidar_sensor['status']=='measured' and lidar_sensor.get('common_from_sensor') is not None:
                bundle['camera_from_lidar']=np.linalg.inv(base_from_rgb)@rigid_matrix(lidar_sensor['common_from_sensor'])
            try:
                transform=self.tf_buffer.lookup_transform(self.task['tracking_frame'],self.calibration['common_frame'],Time.from_msg(message.rgb.header.stamp))
                t=transform.transform;world_from_base=transform_matrix([t.translation.x,t.translation.y,t.translation.z],
                    [t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
                bundle['common_from_rgb']=world_from_base@base_from_rgb
                audio_sensor=self.calibration['sensors']['audio']
                if audio_sensor['status']=='measured' and audio_sensor.get('common_from_sensor') is not None:
                    bundle['common_from_audio']=world_from_base@rigid_matrix(audio_sensor['common_from_sensor'])
            except Exception as error:self.state['pose_reason']=str(error)
        # Only consume this acquisition after all required dependencies are available.
        # Retrying a pending request while depth/TF arrives is not a duplicate packet.
        metadata=self.monitor.observe('rgb',rgb,stamp,f'rgb:{stamp:.9f}',message.rgb.header.frame_id,time.time(),anchor)
        metadata['present']=True;sources['rgb']=metadata
        if not metadata['quality']['usable']:
            if 'clock_regression_or_source_restart' in metadata['continuity']:self.tracking_reset_pending=True
            raise ValueError('RGB source is invalid')
        bundle['preparation_s']=time.monotonic()-prepared_started
        times=[anchor,depth.alignment_stamp]
        for name in ('infrared','lidar','audio'):
            sample=nir if name=='infrared' else bundle.get(name)
            if sample is not None:times.append(float(np.clip(anchor,sample.alignment_stamp,sample.alignment_stamp+sample.duration)) if name=='audio' else sample.alignment_stamp)
        bundle['matched_gap_s']=max(times)-min(times) if all(sources[n]['present'] for n in MODALITIES) else None
        bundle['runtime_calibration']=dict(rgb_info=rgb_info,nir_info=self.infos.get('infrared'),
            static_tf_sha256=hashlib.sha256(json.dumps(self.transforms,sort_keys=True).encode()).hexdigest(),
            registered_depth_source=depth.metadata())
        bundle['geometry_scope_sha256']=hashlib.sha256(json.dumps(dict(rgb_info=rgb_info,nir_info=self.infos.get('infrared'),
            static_tf=self.transforms),sort_keys=True).encode()).hexdigest()
        bundle['reset_tracking']=self.tracking_reset_pending
        self.tracking_reset_pending=False
        return bundle

    def poll(self):
        if self.loading is not None:
            if not self.loading.done():return
            future,self.loading=self.loading,None
            try:self.engine=future.result();self.state['state']='waiting_inputs'
            except Exception as error:self.state.update(state='error',last_error=str(error));return
        if self.future is not None:
            if not self.future.done():return
            future,self.future=self.future,None
            try:
                result=future.result();result['published_at']=time.time();validate_result(seal_result(result))
                serialized=json.dumps(result,ensure_ascii=False,allow_nan=False)
                publish_started=time.monotonic();self.publisher.publish(String(data=serialized));published=time.monotonic()
                bundle=self.active_bundle
                observation=publication_observation(result,bundle['received_monotonic'],bundle['processing_started_monotonic'],publish_started,published,self.last_published)
                self.last_published=published;self.journal.append(observation)
                self.timing_publisher.publish(String(data=json.dumps(dict(schema='object_multimodal_timing/1',sequence=result['sequence'],
                    result_sha256=result['integrity']['sha256'],timing=observation['timing']),allow_nan=False)))
                self.state.update(state='ready',processed=self.state['processed']+1,last_timing=observation['timing'],targets=len(result['targets']))
            except Exception as error:self.state.update(state='error',last_error=str(error))
        if self.engine is None or self.pending is None or time.monotonic()-self.last_start<self.config['min_period_s']:return
        message,received=self.pending
        try:bundle=self.prepare(message,received)
        except Exception as error:
            self.state.update(state='waiting_inputs',reason=str(error))
            if time.monotonic()-received>self.config['max_input_age_s']:self.pending=None
            return
        self.pending=None;self.last_start=time.monotonic();self.state['state']='inferencing'
        self.active_bundle=bundle;self.future=self.pool.submit(self.engine.compute,bundle)

    def status(self):
        self.status_publisher.publish(String(data=json.dumps(dict(schema=1,**self.state,published_at=time.time(),
            calibration_version=self.calibration['version'],sources=self.last_sources),ensure_ascii=False,allow_nan=False)))

    def destroy_node(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init();node=None
    try:
        node=ObjectFusionNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
