"""Bounded mapping evidence, independent of ROS and the SLAM database writer."""
from collections import deque
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np


def rounded(a):
    a=np.asarray(a,dtype=float)
    if not np.isfinite(a).all():raise ValueError('nonfinite mapping evidence')
    return np.round(a,5).tolist()


def sample_points(points,limit=2500):
    a=np.asarray(points,dtype=float).reshape(-1,3)
    a=a[np.isfinite(a).all(1)]
    ids=np.linspace(0,len(a)-1,min(limit,len(a)),dtype=int) if len(a) else np.array([],int)
    return rounded(a[ids])


def transform(position,quaternion):
    p=np.asarray(position,float);q=np.asarray(quaternion,float)
    if p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or np.linalg.norm(q)<1e-8:
        raise ValueError('invalid pose')
    x,y,z,w=q/np.linalg.norm(q);m=np.eye(4)
    m[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    m[:3,3]=p
    return m


def rpy(matrix):
    r=matrix[:3,:3];pitch=np.arctan2(-r[2,0],np.hypot(r[0,0],r[1,0]))
    roll=np.arctan2(r[2,1],r[2,2]);yaw=np.arctan2(r[1,0],r[0,0])
    return rounded(np.rad2deg([roll,pitch,yaw]))


def residual(a,b,measurement):
    e=np.linalg.inv(measurement)@np.linalg.inv(a)@b
    return dict(translation_m=float(np.linalg.norm(e[:3,3])),
                rotation_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(e[:3,:3])-1)/2,-1,1)))))


METRICS=[
    ('sync_error_ms','多传感器同步误差','ms','≤',10,True),
    ('mapping_success_rate','一次建图成功率','%','≥',98,True),
    ('map_load_s','地图加载时间','s','≤',20,True),
    ('initial_no_prior_s','无先验初始定位','s','≤',20,True),
    ('initial_with_prior_s','有地图或辅助信息初始定位','s','≤',2,True),
    ('global_localization_success_rate','静态全局定位成功率','%','≥',98,True),
    ('layout_success_drop_percent','局部布局变化≤15%时成功率下降','百分点','≤',5,True),
    ('absolute_trajectory_error_cm','绝对轨迹误差','cm','＜',5,False),
    ('vertical_error_cm','室内垂直误差','cm','＜',2,False),
    ('angle_error_deg','角度误差','°','＜',1,False),
    ('reacquisition_s','重捕获时间','s',None,None,True),
    ('geometry_error_m','建图几何误差','m',None,None,True),
    ('update_frequency_hz','地图更新频率','Hz',None,None,True),
    ('processing_fps','处理帧率','Hz',None,None,True),
    ('end_to_end_ms','端到端时延','ms',None,None,True),
]


def performance(report,map_uuid):
    """Use only scoped, documented measurements; receive rates are not accuracy."""
    if not isinstance(report,dict):report={}
    values=report.get('measurements',{})
    if not isinstance(values,dict):values={}
    scoped=bool(map_uuid and report.get('map_uuid')==map_uuid)
    rows=[]
    for key,label,unit,op,limit,mandatory in METRICS:
        record=values.get(key,{}) if scoped else {}
        state='pending';value=None
        if (isinstance(record,dict) and record.get('verified') is True and
                record.get('evidence') and record.get('method') and record.get('scene_id')):
            candidate=record.get('value')
            if isinstance(candidate,(int,float)) and not isinstance(candidate,bool) and math.isfinite(candidate) and candidate>=0:
                value=candidate
                if key.endswith('success_rate'):
                    total=record.get('total');success=record.get('success')
                    if (not isinstance(total,int) or isinstance(total,bool) or total<=0 or
                            not isinstance(success,int) or isinstance(success,bool) or not 0<=success<=total or
                            abs(success/total*100-value)>1e-5):value=None
                if key=='sync_error_ms':
                    uncertainty=record.get('uncertainty_ms')
                    if not isinstance(uncertainty,(int,float)) or isinstance(uncertainty,bool) or not math.isfinite(uncertainty) or uncertainty<0:value=None
                if value is not None:
                    actual_limit=limit if limit is not None else record.get('task_limit')
                    actual_op=op or record.get('comparison')
                    if isinstance(actual_limit,(int,float)) and actual_op in ('≤','≥','＜'):
                        upper=value+(record.get('uncertainty_ms',0) if key=='sync_error_ms' else 0)
                        good=upper<=actual_limit if actual_op=='≤' else value>=actual_limit if actual_op=='≥' else upper<actual_limit
                        state='pass' if good else 'fail'
        rows.append(dict(key=key,label=label,unit=unit,comparison=op,limit=limit,mandatory=mandatory,
                         value=value,state=state,record=record if scoped else None))
    return dict(rows=rows,state='pending' if any(r['state']=='pending' for r in rows) else
                'fail' if any(r['state']=='fail' and r['mandatory'] for r in rows) else 'pass',
                scope_valid=scoped,note='接收频率、图约束残差及地图截图不代替定位或建图精度；数值须对应当前地图、场景和测量证据。')


class MappingEvidence:
    def __init__(self):
        self.session=uuid.uuid4().hex;self.sequence=0;self.rows={};self.histories={};self.failures={}
        self.graph={};self.raw_nodes={};self.features={};self.node_identity={};self.links=[];self.map_to_odom=np.eye(4)
        self.frame=None;self.graph_stamp=None;self.graph_revision=0;self.objects=[];self.map_key=None;self.map_uuid=None
        self.trajectory=deque(maxlen=250);self.events=deque(maxlen=60);self.last_loop=None
        self.database=None;self.navigation=None;self.scene=None

    def observe(self,key,payload,stamp,frame,kind,topic,now=None):
        now=time.time() if now is None else now
        times=self.histories.setdefault(key,deque(maxlen=150));times.append(now)
        self.rows[key]=dict(payload=payload,stamp=stamp,frame=frame,kind=kind,topic=topic,received_at=now)

    def reset_map(self):
        self.session=uuid.uuid4().hex;self.sequence=0;self.graph.clear();self.raw_nodes.clear();self.features.clear()
        self.node_identity.clear();self.links=[];self.objects=[];self.trajectory.clear();self.events.clear()
        self.map_uuid=None;self.map_key=None;self.database=None;self.navigation=None
        self.frame=None;self.graph_stamp=None;self.graph_revision=0;self.last_loop=None
        self.map_to_odom=np.eye(4)
        for key in ('map','clean_map','grid','walls','semantic','pose','graph','features'):self.rows.pop(key,None)

    def nodes(self,nodes,allow_reset=True):
        if allow_reset and any(n['id'] in self.node_identity and self.node_identity[n['id']]!=(n['map_id'],n['stamp']) for n in nodes):
            self.reset_map()
        for n in nodes:
            key=n['id'];identity=(n['map_id'],n['stamp'])
            if not allow_reset and key in self.node_identity and self.node_identity[key]!=identity:continue
            self.node_identity[key]=identity;self.raw_nodes[key]=np.asarray(n['pose'],float)
            if n.get('points'):self.features[key]=np.asarray(n['points'],float).reshape(-1,3)

    def update_graph(self,poses,links,stamp,frame,map_to_odom):
        if self.frame is not None and self.frame!=frame:self.reset_map()
        self.frame=frame;self.graph_stamp=stamp;self.graph_revision+=1
        self.map_to_odom=np.asarray(map_to_odom,float)
        self.graph.update({int(k):np.asarray(v,float) for k,v in poses.items()})
        self.links=links

    def semantic(self,objects,map_key,map_uuid):
        if self.map_uuid and self.map_uuid!=map_uuid:self.reset_map()
        self.map_key=map_key;self.map_uuid=map_uuid;self.objects=objects

    def export(self,report=None,now=None):
        now=time.time() if now is None else now;self.sequence+=1
        sources=[]
        for key,row in self.rows.items():
            history=[t for t in self.histories[key] if now-t<=15]
            hz=(len(history)-1)/(history[-1]-history[0]) if len(history)>1 and history[-1]>history[0] else None
            sources.append(dict(key=key,stamp=row['stamp'],frame=row['frame'],topic=row['topic'],type=row['kind'],
                                received_at=row['received_at'],receive_age_s=max(0,now-row['received_at']),receive_hz=hz))
        visuals={};states={key:row['payload'] for key,row in self.rows.items() if key.endswith('status')}
        for key in ('rgbd','lidar','deskewed','map','clean_map','grid','walls','pose','odom','imu'):
            if key in self.rows:visuals[key]=self.rows[key]['payload']
        raw=self.rows.get('lidar');processed=self.rows.get('deskewed')
        preprocessing=dict(pair_valid=bool(raw and processed and raw['frame']==processed['frame'] and
            abs(raw['stamp']-processed['stamp'])<=.15),note='同帧点云可对照运动去畸变；显示抽样点数不表示过滤精度。')
        if raw and processed:preprocessing['pair_gap_ms']=abs(raw['stamp']-processed['stamp'])*1000
        graph_nodes=[dict(id=k,position=rounded(m[:3,3])) for k,m in sorted(self.graph.items()) if k>0]
        edges=[];differences=[]
        for link in self.links[:2000]:
            a,b=link['from'],link['to']
            if a in self.graph and b in self.graph:
                edges.append(dict(a=a,b=b,type=link['type'],**residual(self.graph[a],self.graph[b],link['matrix'])))
        for key,m in sorted(self.graph.items()):
            if key in self.raw_nodes:
                raw_map=self.map_to_odom@self.raw_nodes[key]
                differences.append(dict(id=key,raw=rounded(raw_map[:3,3]),optimized=rounded(m[:3,3]),
                    correction_m=float(np.linalg.norm(m[:3,3]-raw_map[:3,3]))))
        visual_points=[];feature_counts=[]
        for key,points in sorted(self.features.items()):
            if key in self.graph:
                m=self.graph[key];transformed=points@m[:3,:3].T+m[:3,3]
                visual_points.append(transformed[np.isfinite(transformed).all(1)])
                feature_counts.append(dict(node_id=key,points=len(points),stamp=self.node_identity.get(key,(None,None))[1]))
        if visual_points:
            points=np.concatenate(visual_points);visuals['sparse']=dict(xyz=sample_points(points),count=len(points),frame=self.frame)
        anchored=[]
        for obj in self.objects[:150]:
            positions=[];weights=[];anchors=[]
            for anchor in obj['anchors']:
                key=anchor['node_id'];weight=anchor['weight'];anchors.append(key)
                if key in self.graph and math.isfinite(weight) and weight>0:
                    local=np.asarray(anchor['position'],float);m=self.graph[key]
                    positions.append(m[:3,:3]@local+m[:3,3]);weights.append(weight)
            anchored.append(dict(id=obj['id'],label=obj['label'],confidence=obj['confidence'],dynamic=obj['dynamic'],
                node_ids=sorted(set(anchors)),position=rounded(np.average(positions,axis=0,weights=weights)) if positions else None,
                association_state='projected' if positions else 'waiting_pose'))
        active=any(r['receive_age_s']<4 and r['key'] in ('odom','lidar','rgbd') for r in sources)
        operations=[dict(name='数据库文件可见',state='observed' if self.database else 'pending',evidence=self.database),
                    dict(name='地图保存完成及恢复',state='pending'),dict(name='地图加载计时',state='pending'),
                    dict(name='持久性变化与瞬态目标区分',state='pending'),dict(name='子地图拼接与多时相对齐',state='pending'),
                    dict(name='地图要素操作、版本回退及兼容',state='pending')]
        return dict(schema='research_mapping/1',session_id=self.session,sequence=self.sequence,published_at=now,
            state='receiving' if active else 'saved' if self.rows else 'waiting',map_uuid=self.map_uuid,map_key=self.map_key,
            frame=self.frame,source_stamp=self.graph_stamp,sources=sources,visuals=visuals,statuses=states,
            graph=dict(nodes=graph_nodes[:2500],edges=edges,revision=self.graph_revision,
                       comparison=differences[:1000],comparison_note='同关键帧里程计位姿经当前map_to_odom变换，与图优化位姿对照；修正量不是绝对轨迹误差。'),
            associations=anchored,feature_nodes=feature_counts[:150],trajectory=list(self.trajectory),
            loop_events=list(self.events),navigation=self.navigation,preprocessing=preprocessing,operations=operations,
            performance=performance(report or {},self.map_uuid),errors=self.failures,
            capture_note='各来源按自己的采集时间保留；地图与传感器快照可能更新于不同时间，不宣称全部模态同帧同步。')


def atomic_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,separators=(',',':')));temporary.replace(path)


def archived_map_sample(root):
    """Read one historical capture; never merge unrelated mapping sessions."""
    directory=Path(root)/'logs/research/live';report_path=directory/'map_status.json'
    evidence=MappingEvidence()
    try:
        report=json.loads(report_path.read_text());captured=report_path.stat().st_mtime
        with np.load(directory/'map_inputs.npz',allow_pickle=False) as arrays:
            for key,name,topic in [('map','raw_xyz','/cloud_map'),('clean_map','clean_xyz','/miti/research/display/clean_map')]:
                if name in arrays:
                    points=arrays[name]
                    evidence.observe(key,dict(xyz=sample_points(points),count=len(points),frame=None),None,None,
                        'PointCloud2 archived XYZ',topic,now=captured)
        for old,new in [('geometry','map_status'),('dynamic','dynamic_status'),('grid','grid_status'),('loops','loop_status')]:
            if old in report.get('statuses',{}):
                evidence.observe(new,report['statuses'][old],None,None,'archived String / JSON',
                                 '/miti/research/display/'+new,now=captured)
        pose=report.get('statuses',{}).get('geometry',{}).get('pose',{})
        if pose.get('position_m') and pose.get('rpy_deg'):
            evidence.frame=pose.get('frame_id')
            evidence.observe('pose',dict(position=pose['position_m'],rpy_deg=pose['rpy_deg'],
                quaternion=pose.get('orientation_xyzw')),pose.get('stamp'),pose.get('frame_id'),
                'archived PoseStamped fields','/miti/research/display/robot_pose',now=captured)
        sample=evidence.export(now=captured+30);sample['published_at']=captured
        sample['capture_mode']='archived';sample['source_stamp']=captured
        sample['archive_source']=str(report_path.relative_to(root))
        sample['capture_note']='历史同次采样记录，非当前场景。点云归档未保存frame_id，因此不与位姿或其他地图叠加判定一致性。'
        return sample
    except (OSError,ValueError,KeyError):
        return evidence.export()
