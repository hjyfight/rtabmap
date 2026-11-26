#!/usr/bin/env python3
"""Read existing ROS streams and run frozen fusion experiments for dashboard 3001."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import traceback
import uuid
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from rtabmap_research.inputs import (NotReady, AudioTimeline, pair_frames, measured_transform,
                                     project_pmf, tf_preview_transform, pair_rgb_lidar,estimated_transform,
                                     current_transform)
from research_validation import input_record
from research_input_preview import build_input_preview


def atomic_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    temp.replace(path)


class Research:
    def __init__(self, config):
        self.cfg = config
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.session = uuid.uuid4().hex
        self.frames = deque(maxlen=240)
        self.clouds = deque(maxlen=80)
        self.rgbd_frames = deque(maxlen=60)
        self.nir_frames = deque(maxlen=80)
        self.nir_info = None
        self.audio = AudioTimeline()
        self.audio_sources = deque(maxlen=512)
        self.info = None
        self.inputs = {}
        self.models = {}
        self.model_errors = {}
        self.results = {}
        self.tasks = ('pmf','avs','rgbnir') if config.get('rgbnir_enabled') else ('pmf','avs')
        self.states = {key: ('loading', '正在加载完整预训练权重') for key in self.tasks}
        self.next_run = {key: 0. for key in self.tasks}
        self.last_stamp = {key: None for key in self.tasks}
        self.sequence = {key: 0 for key in self.tasks}
        self.public = ROOT / config['public_directory']
        self.public.mkdir(parents=True, exist_ok=True)
        self.node = None
        self.publishers = {}
        self.ros_error = '正在连接传感器话题'
        self.tf_buffer = None
        self.tf_listener = None

    def observe(self, name, stamp, **metadata):
        self.inputs[name] = dict(stamp=stamp, received_at=time.time(), **metadata)

    def image_record(self, modality, msg, array, raw_stamp, offset, topic, **extra):
        return input_record(modality, topic, 'sensor_msgs/msg/Image', raw_stamp, offset,
                            frame=msg.header.frame_id, shape=array.shape, dtype=str(array.dtype),
                            format=msg.encoding, **extra)

    @staticmethod
    def with_array(record, key, index=None):
        return dict(record, array_ref=dict(key=key, **({'index': index} if index is not None else {})))

    def receive_rgb(self, msg):
        from cv_bridge import CvBridge
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9 + self.cfg['rgb_clock_offset_s']
        with self.lock:
            if self.frames and stamp-self.frames[-1][0] < .07 and stamp >= self.frames[-1][0]:
                return
            if self.frames and stamp < self.frames[-1][0]-1:
                self.frames.clear()
        rgb = CvBridge().imgmsg_to_cv2(msg, 'rgb8').copy()
        with self.lock:
            source = self.image_record('rgb', msg, rgb, stamp-self.cfg['rgb_clock_offset_s'],
                                       self.cfg['rgb_clock_offset_s'], self.cfg['rgb_topic'])
            self.frames.append((stamp, rgb, msg.header.frame_id, source))
            self.observe('rgb', stamp, width=msg.width, height=msg.height, frame=msg.header.frame_id,
                         raw_stamp=stamp-self.cfg['rgb_clock_offset_s'],encoding=msg.encoding,
                         topic=self.cfg['rgb_topic'])

    def receive_info(self, msg):
        with self.lock:
            self.info = dict(width=msg.width, height=msg.height, k=list(msg.k), d=list(msg.d),
                             frame=msg.header.frame_id, distortion_model=msg.distortion_model)
            self.observe('camera_info', time.time(), frame=msg.header.frame_id)

    def receive_rgbd(self,msg):
        from cv_bridge import CvBridge
        from guard_rgbd_input import validate_rgbd
        error=validate_rgbd(msg)
        if error:raise ValueError(error)
        raw_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        stamp=raw_stamp+self.cfg['rgb_clock_offset_s']
        from rtabmap_research import registration
        from rtabmap_semantic.fusion_geometry import camera_dict
        row=dict(stamp=stamp,raw_stamp=raw_stamp,frame=msg.rgb.header.frame_id,
                 rgb=CvBridge().imgmsg_to_cv2(msg.rgb,'rgb8').copy(),
                 depth=CvBridge().imgmsg_to_cv2(msg.depth,'passthrough').copy(),
                 k=list(msg.rgb_camera_info.k),distortion=list(msg.rgb_camera_info.d),
                 rgb_info=camera_dict(msg.rgb_camera_info))
        row['input_metadata'] = [
            self.image_record('rgb', msg.rgb, row['rgb'], raw_stamp, self.cfg['rgb_clock_offset_s'],
                              self.cfg.get('rgbd_topic', '/rgbd_image'), payload_path='rgb',
                              stamp_quality='rgbd_header_stamp'),
            self.image_record('depth', msg.depth, row['depth'], raw_stamp, self.cfg['rgb_clock_offset_s'],
                              self.cfg.get('rgbd_topic', '/rgbd_image'), payload_path='depth',
                              stamp_quality='rgbd_header_stamp', value_unit='mm' if row['depth'].dtype == np.uint16 else 'm')]
        with self.lock:
            if self.rgbd_frames and stamp<self.rgbd_frames[-1]['stamp']-1:self.rgbd_frames.clear()
            self.rgbd_frames.append(row)
            self.observe('registered_depth',stamp,width=msg.depth.width,height=msg.depth.height,frame=msg.depth.header.frame_id)

    def receive_nir(self,msg):
        from cv_bridge import CvBridge
        if msg.encoding!='mono8':raise ValueError('NIR requires mono8')
        raw_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        stamp=raw_stamp+self.cfg.get('nir_clock_offset_s',self.cfg['rgb_clock_offset_s'])
        image=CvBridge().imgmsg_to_cv2(msg,'mono8').copy()
        with self.lock:
            if self.nir_frames and stamp<self.nir_frames[-1]['stamp']-1:self.nir_frames.clear()
            self.nir_frames.append(dict(stamp=stamp,raw_stamp=raw_stamp,image=image,frame=msg.header.frame_id,
                input_metadata=self.image_record('nir', msg, image, raw_stamp,
                    self.cfg.get('nir_clock_offset_s', self.cfg['rgb_clock_offset_s']), self.cfg['nir_topic'])))
            self.observe('nir',stamp,width=msg.width,height=msg.height,frame=msg.header.frame_id,
                         raw_stamp=raw_stamp,encoding=msg.encoding,topic=self.cfg['nir_topic'])

    def receive_nir_info(self,msg):
        from rtabmap_research import registration
        from rtabmap_semantic.fusion_geometry import camera_dict
        with self.lock:self.nir_info=camera_dict(msg)

    def rgbnir_bundle(self):
        from rtabmap_research.registration import pair_rgbnir,register_nir
        from rclpy.time import Time
        from tf2_ros import TransformException
        if self.nir_info is None or self.tf_buffer is None:raise NotReady('等待NIR CameraInfo与相机工厂TF')
        frame,nir=pair_rgbnir(self.rgbd_frames,self.nir_frames,self.cfg['rgbnir_sync_tolerance_s'],
                             time.time(),self.cfg['max_input_age_s'])
        transforms={}
        reference=self.nir_info['frame_id']
        for source in {frame['frame'],nir['frame']} - {reference}:
            try:tf=self.tf_buffer.lookup_transform(reference,source,Time(seconds=frame['raw_stamp']))
            except TransformException:raise NotReady('等待相机RGB/NIR标定TF')
            transforms[source]=dict(child=source,parent=reference,
                translation=[getattr(tf.transform.translation,k) for k in ('x','y','z')],
                rotation=[getattr(tf.transform.rotation,k) for k in ('x','y','z','w')])
        bundle = register_nir(frame,nir,self.nir_info,transforms)
        bundle['inputs_used'] = [self.with_array(r, 'input_rgb' if r['modality']=='rgb' else 'source_depth')
                                for r in frame.get('input_metadata', [])]
        bundle['inputs_used'].append(self.with_array(nir['input_metadata'], 'source_nir'))
        return bundle

    def file_calibration(self):
        if self.cfg.get('use_current_extrinsics'):
            return current_transform(ROOT/self.cfg['current_calibration_path'])
        if self.cfg.get('camera_lidar_mount_changed'):
            raise NotReady('相机/雷达安装已改变，等待重新标定外参')
        path=ROOT/self.cfg['calibration_path']
        configured=json.loads(path.read_text())
        if configured.get('status')=='measured' or self.cfg.get('pmf_extrinsics_mode')!='tf_preview':
            return measured_transform(path)
        estimate=self.cfg.get('estimated_calibration_path')
        if estimate and (ROOT/estimate).exists():return estimated_transform(ROOT/estimate)
        return None

    def indoor_bundle(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        from sensor_msgs_py import point_cloud2
        from map_geometry_node import ros_matrix
        if not self.rgbd_frames or not self.clouds or self.tf_buffer is None:
            raise NotReady('等待同步RGB-D、原始LiDAR及里程计TF')
        latest=self.rgbd_frames[-1]
        self.fresh(latest['stamp'])
        loaded=self.file_calibration()
        if loaded is None:raise NotReady('室内点云等待已复核的外参估计或实测标定')
        matrix,calibration=loaded
        try:
            correction=self.tf_buffer.lookup_transform('map','icp_odom',Time())
        except TransformException:raise NotReady('等待地图到里程计TF')
        correction_matrix=ros_matrix(correction.transform)
        correction_stamp=correction.header.stamp.sec+correction.header.stamp.nanosec/1e9
        if correction_stamp and min(abs(time.time()-correction_stamp),abs(latest['raw_stamp']-correction_stamp))>3:
            raise NotReady('地图到里程计TF已过期')
        observations=[];used=set()
        for frame in self.rgbd_frames:
            if latest['stamp']-frame['stamp']>self.cfg.get('point_window_s',3.):continue
            cloud=min(self.clouds,key=lambda c:abs(c[0]-frame['stamp']))
            if abs(cloud[0]-frame['stamp'])>self.cfg['pmf_sync_tolerance_s'] or cloud[0] in used:continue
            msg=cloud[1]
            if frame['frame']!=calibration['camera_frame'] or msg.header.frame_id!=calibration['lidar_frame']:
                raise NotReady('RGB-D/LiDAR坐标系与外参记录不匹配')
            try:
                pose=self.tf_buffer.lookup_transform('icp_odom',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            except TransformException:continue
            values=point_cloud2.read_points(msg,field_names=('x','y','z','intensity'),skip_nans=True)
            points=np.column_stack([values[f] for f in ('x','y','z','intensity')]).astype(np.float32)
            observations.append(dict(rgb=frame['rgb'],depth=frame['depth'],k=frame['k'],distortion=frame['distortion'],
                points=points,world_from_lidar=correction_matrix@ros_matrix(pose.transform),
                rgb_stamp=frame['raw_stamp'],lidar_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                corrected_stamp=frame['stamp'],gap_s=abs(cloud[0]-frame['stamp']),
                input_metadata=frame.get('input_metadata', []) + [dict(cloud[2], shape=list(points.shape),
                    dtype=str(points.dtype), format='decoded XYZ/intensity', value_unit='XYZ:m; intensity:raw')]))
            used.add(cloud[0])
        if len(observations)<self.cfg.get('point_min_frames',4):
            raise NotReady(f'等待有同步位姿的多帧点云：{len(observations)}/{self.cfg.get("point_min_frames",4)}组')
        sources = []
        for i, observation in enumerate(observations):
            for record in observation['input_metadata']:
                key = 'source_rgb_frames' if record['modality']=='rgb' else 'source_depth_frames' if record['modality']=='depth' else 'source_lidar_'+str(i)
                sources.append(self.with_array(record, key, i if record['modality'] != 'lidar' else None))
        return dict(stamp=observations[-1]['corrected_stamp'],observations=observations,matrix=matrix,
                    calibration=calibration,gap_s=max(o['gap_s'] for o in observations), inputs_used=sources,
                    enforce_depth_consistency=not self.cfg.get('use_current_extrinsics',False))

    def receive_lidar(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9 + self.cfg['lidar_clock_offset_s']
        with self.lock:
            if self.clouds and stamp < self.clouds[-1][0]-1:
                self.clouds.clear()
            source = input_record('lidar', self.cfg['lidar_topic'], 'sensor_msgs/msg/PointCloud2',
                stamp-self.cfg['lidar_clock_offset_s'], self.cfg['lidar_clock_offset_s'],
                frame=msg.header.frame_id, shape=[msg.width*msg.height], dtype='structured PointCloud2',
                format=', '.join(f.name for f in msg.fields),
                source_fields=[dict(name=f.name, datatype=f.datatype, count=f.count) for f in msg.fields])
            self.clouds.append((stamp, msg, source))
            self.observe('lidar', stamp, points=msg.width*msg.height, fields=[f.name for f in msg.fields],
                         frame=msg.header.frame_id,raw_stamp=stamp-self.cfg['lidar_clock_offset_s'],
                         topic=self.cfg['lidar_topic'])

    def receive_audio(self, msg):
        if msg.channels < 1 or len(msg.samples) % msg.channels:
            raise ValueError('Invalid AudioFrame channels')
        pcm = np.asarray(msg.samples,np.float32).reshape(-1,msg.channels).mean(1)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9 + self.cfg['audio_clock_offset_s']
        with self.lock:
            duplicate = msg.source_id == self.audio.last_source
            reset = self.audio.rate != msg.sample_rate or (self.audio.windows and stamp+len(pcm)/msg.sample_rate < self.audio.windows[-1][1]-.5)
            if reset: self.audio_sources.clear()
            self.audio.add(stamp, pcm, msg.sample_rate, msg.source_id, msg.stamp_quality)
            if not duplicate:
                self.audio_sources.append(input_record('audio', self.cfg['audio_topic'],
                    'rtabmap_features_msgs/msg/AudioFrame', stamp-self.cfg['audio_clock_offset_s'],
                    self.cfg['audio_clock_offset_s'], frame=msg.header.frame_id, shape=pcm.shape,
                    dtype=str(pcm.dtype), format='float32 PCM [-1,1]; channel mean for model',
                    source_id=msg.source_id, stamp_quality=msg.stamp_quality,
                    rate=int(msg.sample_rate), channels=int(msg.channels), model_channels=1,
                    duration_s=len(pcm)/msg.sample_rate))
            self.observe('audio', stamp+len(pcm)/msg.sample_rate, rate=msg.sample_rate, channels=msg.channels,
                         quality=msg.stamp_quality, duration_s=len(pcm)/msg.sample_rate,
                         source_id=msg.source_id,topic=self.cfg['audio_topic'],
                         rms_dbfs=float(20*np.log10(max(1e-8,float(np.sqrt(np.mean(pcm**2)))))))

    def callback(self, fn):
        def wrapped(msg):
            try:
                fn(msg)
            except Exception as error:
                with self.lock:
                    self.ros_error = f'{fn.__name__}: {error}'
        return wrapped

    def fresh(self, stamp):
        age = time.time()-stamp
        if age < -.5 or age > self.cfg['max_input_age_s']:
            raise NotReady(f'采集时间异常或输入过期：帧龄 {age:.2f}s；请检查设备时钟')

    def pmf_bundle(self):
        if not self.frames or not self.clouds or self.info is None:
            raise NotReady('等待RGB、原始LiDAR与RGB CameraInfo')
        frame, cloud, gap = pair_rgb_lidar(self.frames, self.clouds, self.cfg['pmf_sync_tolerance_s'],
                                         time.time(), self.cfg['max_input_age_s'])
        loaded=self.file_calibration()
        if loaded is not None:
            matrix,calibration=loaded
        else:
            from rclpy.time import Time
            from tf2_ros import TransformException
            if self.tf_buffer is None:
                raise NotReady('等待当前SLAM相机/LiDAR TF')
            try:
                # Query the raw sensor epoch, before the host clock pairing offset.
                tf = self.tf_buffer.lookup_transform(frame[2], cloud[1].header.frame_id,
                                                     Time.from_msg(cloud[1].header.stamp))
            except TransformException:
                raise NotReady('等待当前SLAM相机/LiDAR TF')
            matrix, calibration = tf_preview_transform(
                [getattr(tf.transform.translation, k) for k in ('x','y','z')],
                [getattr(tf.transform.rotation, k) for k in ('x','y','z','w')],
                frame[2], cloud[1].header.frame_id, self.cfg.get('pmf_preview_intensity_scale'),
                self.cfg.get('pmf_preview_intensity_evidence'))
        if frame[2] != calibration['camera_frame'] or cloud[1].header.frame_id != calibration['lidar_frame']:
            raise NotReady('当前传感器frame_id与标定不匹配')
        if self.info['frame'] != frame[2] or (self.info['height'],self.info['width']) != frame[1].shape[:2]:
            raise NotReady('RGB CameraInfo与当前图像尺寸/坐标系不匹配')
        if self.info['distortion_model'] not in ('plumb_bob','rational_polynomial',''):
            raise NotReady('当前RGB畸变模型尚不支持投影')
        return dict(stamp=frame[0], rgb=frame[1], cloud=cloud[1], info=dict(self.info), matrix=matrix,
                    intensity_scale=calibration['intensity_scale'], gap_s=gap,
                    calibration=calibration, inputs_used=(
                        [self.with_array(frame[3], 'source_rgb')] if len(frame)>3 else []) + (
                        [self.with_array(cloud[2], 'source_lidar_points')] if len(cloud)>2 else []))

    def avs_bundle(self):
        # Centres of author's 0.96s log-Mel patches (96 STFT frames, 25ms window).
        centres = .4875 + np.arange(10)*.96
        if not self.audio.windows:
            raise NotReady('等待麦克风PCM')
        if not self.frames:
            raise NotReady('等待RGB图像')
        # Local Orin audio can arrive before Go2 video over Wi-Fi. Select an
        # actually covered audio window whose last patch has an available image.
        end = min(self.audio.windows[-1][1], self.frames[-1][0] + 10 - centres[-1])
        pcm, rate, start, end, quality = self.audio.extract(end=end)
        self.fresh(end)
        times = start + centres
        frames, gaps = pair_frames(self.frames, times, self.cfg['avs_sync_tolerance_s'])
        sources = [self.with_array(r[3], 'source_rgb_frames', i) for i, r in enumerate(frames) if len(r)>3]
        audio_sources = [dict(r) for r in getattr(self, 'audio_sources', []) if r['aligned_stamp'] < end and
                         r['aligned_stamp']+r['duration_s'] > start]
        owner = np.full(len(pcm), -1, np.int32)
        for i, record in enumerate(audio_sources):
            offset = int(round((record['aligned_stamp']-start)*rate))
            a, b = max(0, offset), min(len(pcm), offset+record['shape'][0])
            if b > a: owner[a:b] = i
        for i, record in enumerate(audio_sources):
            used = np.flatnonzero(owner == i)
            if not len(used): continue
            breaks = np.flatnonzero(np.diff(used)>1)+1
            spans = np.split(used, breaks)
            sources.append(dict(self.with_array(record, 'input_pcm'),
                used_sample_ranges=[[int(v[0]), int(v[-1])+1] for v in spans],
                archive_relation='overlapping capture windows cropped onto selected mono PCM timeline'))
        return dict(stamp=frames[-1][0], rgbs=[r[1] for r in frames], pcm=pcm, rate=rate,
                    audio_start=start, audio_end=end, frame_stamps=[r[0] for r in frames],
                    gap_s=max(gaps), stamp_quality=quality, inputs_used=sources)

    def project_bundle(self, bundle):
        from sensor_msgs_py import point_cloud2
        fields = [f.name for f in bundle['cloud'].fields]
        intensity = next((f for f in ('intensity','reflectivity') if f in fields),None)
        if intensity is None:
            raise NotReady('原始点云缺少intensity/reflectivity；不能补造PMF第五通道')
        values = point_cloud2.read_points(bundle['cloud'],field_names=('x','y','z',intensity),skip_nans=True)
        if getattr(values.dtype,'names',None):
            points = np.column_stack([values[f] for f in ('x','y','z',intensity)]).astype(np.float32)
        else:
            points = np.asarray(values,np.float32).reshape(-1,4)
        bundle['raw_points'] = points
        for record in bundle.get('inputs_used', []):
            if record['modality']=='lidar': record.update(shape=list(points.shape), dtype=str(points.dtype), format='decoded XYZ/intensity')
        return project_pmf(points,bundle['rgb'],bundle['info']['k'],bundle['info']['d'],
                           bundle['matrix'],bundle['intensity_scale'])

    def worker(self):
        import torch
        from rtabmap_research.models import PMF, AVS
        backend=self.cfg.get('pmf_backend')
        point_backend=backend in ('ptv3_compare','odin_compare')
        if backend=='odin_compare':
            from rtabmap_research.odin import ODINComparison
            pmf_class=ODINComparison
        elif point_backend:
            from rtabmap_research.indoor import IndoorComparison
            pmf_class=IndoorComparison
        else:pmf_class=PMF
        torch.set_num_threads(self.cfg['cpu_threads'])
        from rtabmap_research.rgbnir import RGBNIR
        classes={'pmf':pmf_class,'avs':AVS,'rgbnir':RGBNIR}
        for key in self.tasks:
            if key=='pmf' and self.cfg.get('camera_lidar_mount_changed') and not self.cfg.get('use_current_extrinsics'):
                with self.lock:
                    self.states[key]=('waiting','相机/雷达安装已改变，等待重新标定外参')
                continue
            try:
                weights=self.cfg['odin_weights'] if key=='pmf' and backend=='odin_compare' else self.cfg['ptv3_weights'] if key=='pmf' and point_backend else self.cfg[key+'_weights']
                options=dict(size=self.cfg['rgbnir_size'],confidence=self.cfg['rgbnir_confidence'],
                    nir_weight=self.cfg['rgbnir_nir_weight']) if key=='rgbnir' else {}
                if key=='pmf' and backend=='odin_compare':
                    options['preprocess_workers']=self.cfg.get('odin_preprocess_workers',1)
                model = classes[key](ROOT/weights,self.cfg['device'],**options)
                with self.lock:
                    self.models[key] = model
                    self.states[key] = ('waiting','模型已加载，等待输入')
            except Exception as error:
                traceback.print_exc()
                with self.lock:
                    self.model_errors[key] = str(error)
                    self.states[key] = ('model_error', str(error))
        while not self.stop.wait(.15):
            for key in self.tasks:
                if key not in self.models or time.monotonic() < self.next_run[key]:
                    continue
                try:
                    with self.lock:
                        if key=='pmf':
                            try:bundle=self.indoor_bundle() if point_backend else self.pmf_bundle()
                            except NotReady as reason:
                                if point_backend or not self.cfg.get('allow_pmf_rgb_preview') or not self.frames:raise
                                frame=self.frames[-1];self.fresh(frame[0])
                                bundle=dict(rgb_only=True,rgb=frame[1],stamp=frame[0],waiting_reason=str(reason))
                        elif key=='rgbnir':bundle=self.rgbnir_bundle()
                        else:bundle=self.avs_bundle()
                        if bundle['stamp'] == self.last_stamp[key]:
                            continue
                        self.states[key] = ('running','正在进行冻结模型推理')
                    start = time.perf_counter()
                    model = self.models[key]
                    if key=='pmf' and not point_backend and not bundle.get('rgb_only'):
                        try:
                            sparse,image,mask,uv,indices = self.project_bundle(bundle)
                        except NotReady as reason:
                            if not self.cfg.get('allow_pmf_rgb_preview'):
                                raise
                            bundle = dict(rgb_only=True,rgb=bundle['rgb'],stamp=bundle['stamp'],
                                          waiting_reason=str(reason))
                    if key=='pmf' and point_backend:
                        result,labels=model.infer_bundle(bundle)
                        calibration=bundle['calibration']
                        context=dict(sync_gap_s=bundle['gap_s'],calibration=calibration['status'],
                            calibration_evidence=calibration['evidence'],geometry_verified=calibration['status']=='measured',
                            camera_from_lidar=bundle['matrix'].tolist(),camera_frame=calibration['camera_frame'],
                            lidar_frame=calibration['lidar_frame'],rgb_stamp=bundle['stamp'],lidar_used=True,
                            depth_used=True,
                            lidar_channels=['XYZ query positions'] if backend=='odin_compare' else ['XYZ positions','surface normals'],
                            input_fusion='posed RGB-D context + Livox / valid depth point queries' if backend=='odin_compare' else 'RGB color + LiDAR normals',
                            depth_role='encoder geometry and valid depth queries; holes only inpainted for context' if backend=='odin_compare' else 'occlusion filter',
                            occlusion_filter='registered RealSense depth, residual <=0.2m'
                                if bundle.get('enforce_depth_consistency',True) else 'current-parameter projection preview',
                            depth_consistency_enforced=bundle.get('enforce_depth_consistency',True),
                            source_frames=[dict(rgb_stamp=o['rgb_stamp'],lidar_stamp=o['lidar_stamp'],gap_s=o['gap_s'])
                                           for o in bundle['observations']],
                            world_frame='map',
                            observation_count=len(bundle['observations']),window_s=result['window_s'],
                            motion_deskewed=False)
                    elif key=='pmf' and bundle.get('rgb_only'):
                        result,labels=model.rgb_preview(bundle['rgb'])
                        context=dict(sync_gap_s=None,calibration='unverified',rgb_stamp=bundle['stamp'],
                                     waiting_reason=bundle['waiting_reason'],lidar_used=False)
                    elif key == 'pmf':
                        result, labels = model.infer(bundle['rgb'],sparse,image,mask,uv)
                        model.trace.arrays['selected_point_indices'] = indices
                        calibration = bundle['calibration']
                        model.trace.arrays['camera_from_lidar'] = bundle['matrix']
                        model.trace.arrays['rgb_intrinsic'] = np.asarray(bundle['info']['k']).reshape(3,3)
                        model.trace.arrays['rgb_distortion'] = np.asarray(bundle['info']['d'])
                        context = dict(sync_gap_s=bundle['gap_s'], calibration=calibration['status'],
                                       calibration_evidence=calibration['evidence'],
                                       geometry_verified=calibration['status']=='measured',
                                       intensity_scale=bundle['intensity_scale'],
                                       intensity_evidence=calibration.get('intensity_evidence','标定文件'),
                                       camera_from_lidar=bundle['matrix'].tolist(),
                                       camera_frame=calibration['camera_frame'], lidar_frame=calibration['lidar_frame'],
                                       lidar_stamp=bundle['cloud'].header.stamp.sec+bundle['cloud'].header.stamp.nanosec/1e9,
                                       rgb_stamp=bundle['stamp'], lidar_used=True,
                                       lidar_channels=['range','x','y','z','intensity'])
                    elif key=='rgbnir':
                        result,labels=model.infer(bundle['rgb'],bundle['nir'],bundle['valid'])
                        model.trace.arrays.update(source_nir=bundle['source_nir'],source_depth=bundle['source_depth'],
                            projection_u=bundle['projection_u'],projection_v=bundle['projection_v'],nir_from_rgb=bundle['matrix'])
                        context=dict(sync_gap_s=bundle['gap_s'],rgb_stamp=bundle['rgb_stamp'],nir_stamp=bundle['nir_stamp'],
                            calibration=bundle['calibration_version'],rgb_info=bundle['rgb_info'],nir_info=bundle['nir_info'],
                            nir_from_rgb=bundle['matrix'].tolist(),input_fusion='0.9 RGB + 0.1 RMS-scaled NIR at P3/P4/P5',
                            lidar_used=False,nir_used=True)
                    else:
                        result, labels = model.infer(bundle['rgbs'],bundle['pcm'],bundle['rate'])
                        context = dict(sync_gap_s=bundle['gap_s'], audio_start=bundle['audio_start'],
                            audio_end=bundle['audio_end'],frame_stamps=bundle['frame_stamps'],
                            stamp_quality=bundle['stamp_quality'],time_sampling='centres of 0.96s log-Mel patches',
                            duration_s=10, audio_dim=128, projected_dim=256, sample_rate=bundle['rate'],
                            audio_source='WheelTec; '+self.cfg['audio_topic'])
                    elapsed = time.perf_counter()-start
                    if key == 'avs':
                        model.trace.arrays['source_rgb_frames'] = np.stack(bundle['rgbs'])
                    elif key == 'pmf' and not point_backend:
                        model.trace.arrays['source_rgb'] = bundle['rgb']
                        if 'raw_points' in bundle: model.trace.arrays['source_lidar_points'] = bundle['raw_points']
                    context['inputs_used'] = bundle.get('inputs_used', [])
                    context['runtime_pair_tolerance_ms'] = self.cfg[key+'_sync_tolerance_s']*1000
                    context['output_contract'] = dict(output_shape=result.get('output_shape'), classes=result.get('classes', []))
                    context['clock_alignment']=self.cfg.get('clock_alignment',dict(method='configured_offsets'))
                    context['clock_offsets_s']={name:self.cfg.get(name+'_clock_offset_s',0.)
                                                for name in ('rgb','lidar','nir','audio')}
                    result.update(sequence=self.sequence[key]+1, source_stamp=bundle['stamp'], inference_s=elapsed,
                                  context=context, accuracy_verified=False)
                    if not self.cfg['save_features']:
                        preview_started=time.perf_counter()
                        result['input_preview']=build_input_preview(key,self.session,result,model.trace.arrays)
                        result['preview_s']=time.perf_counter()-preview_started
                    if self.cfg['save_features']:
                        path = self.public/f'{key}_features.npz'
                        temp = path.with_suffix('.tmp')
                        with temp.open('wb') as stream:
                            np.savez_compressed(stream,**model.trace.arrays,
                                metadata=np.array(json.dumps(dict(session=self.session,model=model.metadata,
                                    sequence=result['sequence'],task=key,source_stamp=bundle['stamp'],
                                    context=context,features=result['features'],
                                    annotations=result.get('annotations',[])),ensure_ascii=False)))
                        temp.replace(path)
                        result['feature_download'] = f'/research/{key}_features.npz'
                    with self.lock:
                        self.results[key] = result
                        self.sequence[key] += 1
                        self.last_stamp[key] = bundle['stamp']
                        detail = ('按当前外参参数运行'
                                  if key=='pmf' and context.get('calibration')=='current_parameters' else
                                  '室内实验对照：自动外参估计已做独立帧复核，尚未完成标定验收'
                                  if key=='pmf' and context.get('calibration')=='auto_estimated' else
                                  '实验融合：使用当前SLAM TF，安装外参尚未实测标定'
                                  if key=='pmf' and context.get('calibration')=='tf_preview' else '推理完成')
                        self.states[key] = ('waiting',bundle['waiting_reason']) if bundle.get('rgb_only') else ('ready',detail)
                except NotReady as error:
                    with self.lock:
                        self.states[key] = ('waiting',str(error))
                except Exception as error:
                    traceback.print_exc()
                    with self.lock:
                        self.states[key] = ('error',str(error))
                self.next_run[key] = time.monotonic()+self.cfg[key+'_period_s']

    def snapshot(self, key):
        now = time.time()
        with self.lock:
            inputs = {n:dict(v,age_s=now-v['stamp'],receive_age_s=now-v['received_at'])
                      for n,v in self.inputs.items()}
            state, detail = self.states[key]
            model = self.models.get(key)
            return dict(schema='downstream_fusion/1',session_id=self.session,task=key,published_at=now,
                        state=state,detail=detail,inputs=inputs,ros_error=self.ros_error,
                        offline_report=(self.public/'offline/index.html').exists(),
                        model=model.metadata if model else None,result=self.results.get(key),
                        parameters_mode='current_parameters' if key=='pmf' and self.cfg.get('use_current_extrinsics') else None,
                        evaluation=dict(verified=False,note='尚无人工真值与独立单模态基线，不能据此判定融合精度达标'))

    def ros_loop(self):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Image, CameraInfo, PointCloud2
        from std_msgs.msg import String
        from rtabmap_features_msgs.msg import AudioFrame
        from rtabmap_msgs.msg import RGBDImage
        from tf2_ros import Buffer, TransformListener
        from rclpy.duration import Duration
        try:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node('downstream_fusion_research_3001')
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
            self.tf_listener = TransformListener(self.tf_buffer, self.node)
            for kind,typ,topic,fn in [
                ('rgb',Image,'rgb_topic',self.receive_rgb),('info',CameraInfo,'rgb_info_topic',self.receive_info),
                ('lidar',PointCloud2,'lidar_topic',self.receive_lidar),('audio',AudioFrame,'audio_topic',self.receive_audio)]:
                self.node.create_subscription(typ,self.cfg[topic],self.callback(fn),qos_profile_sensor_data)
            if self.cfg.get('pmf_backend') in ('ptv3_compare','odin_compare') or self.cfg.get('rgbnir_enabled'):
                self.node.create_subscription(RGBDImage,self.cfg.get('rgbd_topic','/rgbd_image'),
                                              self.callback(self.receive_rgbd),qos_profile_sensor_data)
            if self.cfg.get('rgbnir_enabled'):
                for typ,topic,fn in [(Image,'nir_topic',self.receive_nir),(CameraInfo,'nir_info_topic',self.receive_nir_info)]:
                    self.node.create_subscription(typ,self.cfg[topic],self.callback(fn),qos_profile_sensor_data)
            self.publishers = {key:self.node.create_publisher(String,f'/miti/research/{key}',1) for key in self.tasks}
            with self.lock:
                self.ros_error = ''
            while not self.stop.is_set() and rclpy.ok():
                rclpy.spin_once(self.node,timeout_sec=.1)
        except Exception as error:
            if not self.stop.is_set():
                traceback.print_exc()
                with self.lock:
                    self.ros_error = str(error)
        finally:
            self.publishers.clear()
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def run(self):
        from std_msgs.msg import String
        threads=[threading.Thread(target=self.ros_loop,daemon=True),threading.Thread(target=self.worker,daemon=True)]
        for thread in threads:
            thread.start()
        while not self.stop.wait(1):
            for key in self.tasks:
                value=self.snapshot(key)
                atomic_json(self.public/f'{key}.json',value)
                publisher=self.publishers.get(key)
                if publisher and self.cfg.get('publish_ros_snapshots', True):
                    try:
                        publisher.publish(String(data=json.dumps(value,ensure_ascii=False,allow_nan=False)))
                    except Exception:
                        pass
        for thread in threads:
            thread.join(timeout=3)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(ROOT/'config/fusion_research.json'))
    args=parser.parse_args()
    cfg=json.loads(Path(args.config).read_text())
    app=Research(cfg)
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_: app.stop.set())
    app.run()


if __name__=='__main__':
    main()
