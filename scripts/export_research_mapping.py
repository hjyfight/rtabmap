#!/usr/bin/env python3
"""Read-only mapping recorder for 3001; never starts drivers or writes SLAM maps."""
import argparse
import base64
from copy import deepcopy
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,PointCloud2,Imu
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rtabmap_msgs.msg import MapData,MapGraph,RGBDImage,Info
from rtabmap_msgs.srv import GetMap2
from cv_bridge import CvBridge

from map_geometry_node import cloud_xyz
from guard_rgbd_input import validate_rgbd
from navigation_topology import build_navigation_graph,NavigationConfig
from loop_closure_diagnostics import read_stats,detection_snapshot
from research_mapping_data import MappingEvidence,transform,rpy,rounded,sample_points,atomic_json,archived_map_sample

ROOT=Path(__file__).resolve().parents[1]
PREFIX='/miti/research/display'


def stamp(message):return message.header.stamp.sec+message.header.stamp.nanosec/1e9
def pose_matrix(p):return transform([p.position.x,p.position.y,p.position.z],[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
def tf_matrix(t):return transform([t.translation.x,t.translation.y,t.translation.z],[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
def jpeg(a):
    ok,encoded=cv2.imencode('.jpg',a,[cv2.IMWRITE_JPEG_QUALITY,76])
    if not ok:raise ValueError('image export failed')
    return base64.b64encode(encoded).decode()


class MappingRecorder(Node):
    def __init__(self,directory,report):
        super().__init__('research_mapping_recorder_3001')
        self.evidence=MappingEvidence();self.directory=directory;self.report_path=report
        self.bridge=CvBridge();self.last={};self.service_future=None;self.last_service=0.;self.navigation_key=None
        sensor=qos_profile_sensor_data
        retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe('rgbd','/rgbd_image',RGBDImage,self.rgbd,sensor,1.)
        self.subscribe('lidar','/livox/lidar',PointCloud2,self.cloud,sensor,1.)
        self.subscribe('deskewed','/livox/lidar/deskewed',PointCloud2,self.cloud,sensor,1.)
        self.subscribe('map','/cloud_map',PointCloud2,self.cloud,retained,2.)
        self.subscribe('clean_map',PREFIX+'/clean_map',PointCloud2,self.cloud,retained,2.)
        self.subscribe('imu','/imu/data',Imu,self.imu,sensor,.25)
        self.subscribe('odom','/icp_odom',Odometry,self.odom,sensor,.25)
        self.subscribe('pose',PREFIX+'/robot_pose',PoseStamped,self.pose,sensor,.25)
        self.subscribe('grid',PREFIX+'/grid_map',OccupancyGrid,self.grid,retained,4.)
        self.subscribe('walls',PREFIX+'/walls',MarkerArray,self.walls,retained,4.)
        self.subscribe('graph','/mapGraph',MapGraph,self.graph,1,.5)
        self.subscribe('map_data','/mapData',MapData,self.map_data,sensor,1.)
        self.subscribe('loop_info','/info',Info,self.info,sensor,.25)
        for key,topic in [('map_status',PREFIX+'/map_status'),('grid_status',PREFIX+'/grid_status'),
                          ('dynamic_status',PREFIX+'/dynamic_cloud_status'),('loop_status',PREFIX+'/loop_status'),
                          ('semantic_status','/semantic/status')]:
            self.subscribe(key,topic,String,self.status,sensor,1.)
        try:
            from rtabmap_semantic_msgs.msg import SemanticMap
            self.subscribe('semantic','/semantic/objects',SemanticMap,self.semantic,retained,1.)
        except ImportError:self.evidence.failures['semantic']='语义消息类型未加载，等待现有语义模块'
        self.map_client=self.create_client(GetMap2,'/rtabmap/get_map_data2')
        self.create_timer(2.,self.request_words);self.create_timer(4.,self.save)
        self.save()

    def subscribe(self,key,topic,kind,callback,qos,interval):
        def receive(message):
            now=time.monotonic()
            if now-self.last.get(key,-float('inf'))<interval:return
            self.last[key]=now
            try:callback(key,topic,message)
            except (ValueError,KeyError,TypeError,IndexError) as error:
                old=self.evidence.failures.get(key,{});count=old.get('count',0) if isinstance(old,dict) else 0
                self.evidence.failures[key]=dict(count=count+1,last=str(error))
        self.create_subscription(kind,topic,receive,qos)

    def observe(self,key,topic,m,payload):
        self.evidence.observe(key,payload,stamp(m),m.header.frame_id,type(m).__name__,topic)

    def rgbd(self,key,topic,m):
        error=validate_rgbd(m)
        if error:raise ValueError(error)
        rgb=self.bridge.imgmsg_to_cv2(m.rgb,'bgr8');depth=self.bridge.imgmsg_to_cv2(m.depth,'passthrough')
        metres=depth.astype(np.float32)/(1000 if depth.dtype==np.uint16 else 1)
        valid=np.isfinite(metres)&(metres>0)
        preview=(np.clip(metres,0,5)/5*255).astype(np.uint8);preview[~valid]=0
        payload=dict(rgb=jpeg(rgb),depth=jpeg(cv2.applyColorMap(preview,cv2.COLORMAP_TURBO)),
            rgb_shape=list(rgb.shape),depth_shape=list(depth.shape),depth_encoding=m.depth.encoding,
            valid_depth_fraction=float(valid.mean()),intrinsics=list(m.rgb_camera_info.k),
            rgb_stamp=stamp(m.rgb),depth_stamp=stamp(m.depth),pair_gap_ms=abs(stamp(m.rgb)-stamp(m.depth))*1000,
            frame=m.rgb.header.frame_id,preprocessing='现有校验后的RGB-D输入；深度显示范围0–5 m')
        self.observe(key,topic,m,payload)

    def cloud(self,key,topic,m):
        points=cloud_xyz(m,limit=3000)
        self.observe(key,topic,m,dict(xyz=sample_points(points),count=m.width*m.height,frame=m.header.frame_id,
            fields=[f.name for f in m.fields],point_step=m.point_step,finite_sample_count=int(np.isfinite(points).all(1).sum())))

    def imu(self,key,topic,m):
        self.observe(key,topic,m,dict(orientation=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],
            angular_velocity=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
            linear_acceleration=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]))

    def odom(self,key,topic,m):
        matrix=pose_matrix(m.pose.pose)
        self.observe(key,topic,m,dict(position=rounded(matrix[:3,3]),rpy_deg=rpy(matrix),
            covariance=list(m.pose.covariance),child_frame=m.child_frame_id,
            linear_velocity=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z],
            angular_velocity=[m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z]))

    def pose(self,key,topic,m):
        matrix=pose_matrix(m.pose);p=rounded(matrix[:3,3])
        self.observe(key,topic,m,dict(position=p,rpy_deg=rpy(matrix),quaternion=[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]))
        self.evidence.trajectory.append(dict(stamp=stamp(m),position=p,frame=m.header.frame_id))

    def graph(self,key,topic,m):
        poses={k:pose_matrix(p) for k,p in zip(m.poses_id,m.poses)}
        links=[dict(**{'from':l.from_id,'to':l.to_id},type=l.type,matrix=tf_matrix(l.transform)) for l in m.links]
        self.evidence.update_graph(poses,links,stamp(m),m.header.frame_id,tf_matrix(m.map_to_odom))
        self.observe(key,topic,m,dict(nodes=len(poses),links=len(links)))

    def map_data(self,key,topic,m,allow_reset=True):
        nodes=[dict(id=n.id,map_id=n.map_id,stamp=n.stamp,pose=pose_matrix(n.pose),
                    points=[[p.x,p.y,p.z] for p in n.word_pts]) for n in m.nodes if n.id>0]
        self.evidence.nodes(nodes,allow_reset)
        if not self.evidence.graph:
            graph=deepcopy(m.graph)
            # GetMap2 on some RTAB-Map builds leaves the nested graph header empty.
            # The outer MapData header is the actual source coordinate/time record.
            if not graph.header.frame_id:graph.header=deepcopy(m.header)
            self.graph('graph',topic,graph)
        self.observe(key,topic,m,dict(nodes=len(nodes),with_visual_points=sum(bool(n['points']) for n in nodes)))

    def semantic(self,key,topic,m):
        objects=[dict(id=o.object_id,label=o.label,confidence=float(o.confidence),dynamic=bool(o.dynamic),
            anchors=[dict(node_id=a.node_id,stamp=a.node_stamp,position=[a.position.x,a.position.y,a.position.z],weight=a.weight) for a in o.observations]) for o in m.objects]
        self.evidence.semantic(objects,m.map_key,m.map_uuid);self.observe(key,topic,m,dict(objects=len(objects),map_uuid=m.map_uuid))
        path=Path(m.map_key).expanduser()
        try:
            if path.suffix=='.db' and (path.resolve().is_relative_to(ROOT) or path.resolve().is_relative_to(Path.home()/'.ros')):
                stat=path.stat();self.evidence.database=dict(path=str(path),size_bytes=stat.st_size,modified_at=stat.st_mtime,
                    note='仅文件状态；不证明保存完成或加载时间合格')
        except OSError:self.evidence.database=None

    def grid(self,key,topic,m):
        w,h=m.info.width,m.info.height
        if not w or not h or w*h!=len(m.data) or w*h>4000000:raise ValueError('invalid grid shape')
        data=np.asarray(m.data,np.int8).reshape(h,w)
        colors=np.full((h,w,3),100,np.uint8);colors[data==0]=[222,235,240];colors[data>=65]=[23,34,45]
        factor=min(1,640/max(w,h));preview=cv2.resize(colors,(max(1,round(w*factor)),max(1,round(h*factor))),interpolation=cv2.INTER_NEAREST)
        origin=pose_matrix(m.info.origin);yaw=float(np.arctan2(origin[1,0],origin[0,0]))
        self.observe(key,topic,m,dict(image=jpeg(preview),width=w,height=h,resolution=m.info.resolution,frame=m.header.frame_id,
            origin=rounded(origin[:3,3]),yaw=yaw,unknown=int((data<0).sum()),free=int((data==0).sum()),occupied=int((data>=65).sum())))
        if w*h<=1000000:
            graph,labels,clearance=build_navigation_graph(data,m.info.resolution,tuple(origin[:3,3]),yaw,NavigationConfig())
            graph.update(frame_id=m.header.frame_id,source_stamp=stamp(m),source_topic=topic,
                         origin='由本次实际栅格按既有几何分区算法只读推导，未写入导航或运动控制')
            self.evidence.navigation=graph
        else:self.evidence.navigation=dict(state='waiting',nodes=[],edges=[],reason='当前栅格超过有界展示分析上限')

    def walls(self,key,topic,m):
        points=[];labels=[];frame=None;stamp_value=None
        for marker in m.markers:
            frame=marker.header.frame_id;stamp_value=stamp(marker)
            if marker.action!=0:continue
            if marker.ns.endswith('/triangles'):points.extend([[p.x,p.y,p.z] for p in marker.points])
            if marker.type==9:labels.append(dict(label=marker.text,position=[marker.pose.position.x,marker.pose.position.y,marker.pose.position.z]))
        if frame:self.evidence.observe(key,dict(xyz=sample_points(points),count=len(points),labels=labels,frame=frame),stamp_value,frame,'MarkerArray',topic)

    def status(self,key,topic,m):
        data=json.loads(m.data)
        if not isinstance(data,dict):raise ValueError('status must be an object')
        self.evidence.observe(key,data,data.get('published_at'),None,'String / JSON',topic)

    def info(self,key,topic,m):
        data=detection_snapshot(m,read_stats(m));event=dict(data,ref_id=m.ref_id,stamp=stamp(m),
            loop_closure_id=m.loop_closure_id,proximity_detection_id=m.proximity_detection_id)
        identity=(m.ref_id,m.loop_closure_id,m.proximity_detection_id)
        if identity!=self.evidence.last_loop:
            self.evidence.events.append(event);self.evidence.last_loop=identity
        self.observe(key,topic,m,dict(ref_id=m.ref_id,**data))

    def request_words(self):
        if self.service_future:
            if self.service_future.done():
                try:
                    if self.service_session==self.evidence.session:
                        self.map_data('map_data','/rtabmap/get_map_data2',self.service_future.result().data,False)
                except Exception as error:self.evidence.failures['word_snapshot']=str(error)
                self.service_future=None
            elif time.monotonic()-self.last_service>8:
                self.map_client.remove_pending_request(self.service_future);self.service_future.cancel();self.service_future=None
            return
        if time.monotonic()-self.last_service<15 or not self.map_client.service_is_ready():return
        self.last_service=time.monotonic()
        self.service_session=self.evidence.session
        self.service_future=self.map_client.call_async(GetMap2.Request(global_map=True,optimized=True,with_words=True))

    def save(self):
        try:report=json.loads(self.report_path.read_text())
        except (OSError,ValueError):report={}
        sample=self.evidence.export(report)
        no_data=not any(key in self.evidence.rows for key in ('rgbd','lidar','map','grid','graph','pose','semantic'))
        if no_data:
            try:
                previous=json.loads((self.directory/'sample.json').read_text())
                if previous.get('sources'):sample=previous
            except (OSError,ValueError):pass
        atomic_json(self.directory/'sample.json',sample)
        atomic_json(self.directory/'status.json',dict(schema='research_mapping_status/1',published_at=time.time(),
            state='waiting' if no_data else sample['state'],session_id=sample['session_id'],sequence=sample['sequence'],sources=sample['sources'],
            map_uuid=sample['map_uuid'],errors=sample['errors']))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=ROOT/'.tools/miti-research/root/usr/lib/miti/public/research/mapping')
    parser.add_argument('--verification',type=Path,default=ROOT/'config/mapping_research_verification.json')
    parser.add_argument('--seconds',type=float,default=0,help='Bounded read-only capture for development checks')
    args=parser.parse_args();until=time.monotonic()+args.seconds if args.seconds else float('inf')
    while time.monotonic()<until:
        node=None
        try:
            rclpy.init();node=MappingRecorder(args.directory,args.verification)
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
            node.save();break
        except (KeyboardInterrupt,ExternalShutdownException):break
        except Exception as error:
            # A missing wired interface must not terminate the whole 3001 supervisor.
            path=args.directory/'sample.json'
            try:sample=json.loads(path.read_text())
            except (OSError,ValueError):sample=archived_map_sample(ROOT)
            if not sample.get('sources'):sample=archived_map_sample(ROOT)
            sample['state']='saved' if sample.get('sources') else 'waiting'
            sample.setdefault('errors',{})['ros_connection']=str(error)
            atomic_json(path,sample)
            atomic_json(args.directory/'status.json',dict(schema='research_mapping_status/1',published_at=time.time(),
                state=sample['state'],session_id=sample['session_id'],sequence=sample['sequence'],sources=sample.get('sources',[]),
                map_uuid=sample.get('map_uuid'),errors=sample['errors']))
            print('建图只读采集暂不可用，保留历史记录或等待输入：'+str(error),flush=True)
        finally:
            if node is not None:node.destroy_node()
            try:
                if rclpy.ok():rclpy.shutdown()
            except Exception:pass
        if time.monotonic()<until:time.sleep(min(10,max(0,until-time.monotonic())))


if __name__=='__main__':main()
