"""Persistent target identities with motion prediction and explicit lost/reassociated states."""
import json
import uuid
from pathlib import Path
import numpy as np


def unit(vector):
    value=np.asarray(vector,float);norm=np.linalg.norm(value)
    if value.shape!=(1024,) or not np.isfinite(value).all() or norm<1e-12:
        raise ValueError('Invalid target association feature')
    return value/norm


class TargetTracker:
    def __init__(self, frame_id, calibration_version, max_gap_s=3., gate_m=.8, similarity=.65,
                 state_path=None):
        self.frame_id,self.calibration_version=frame_id,calibration_version
        self.max_gap_s,self.gate_m,self.similarity=max_gap_s,gate_m,similarity
        if not frame_id or not calibration_version or not np.isfinite([max_gap_s,gate_m,similarity]).all() or max_gap_s<=0 or gate_m<=0 or not 0<similarity<=1:
            raise ValueError('Invalid tracking policy')
        self.state_path=Path(state_path) if state_path else None
        self.session=uuid.uuid4().hex;self.tracks={};self.counter=0;self.last_stamp=None;self.geometry_scope=None
        if self.state_path and self.state_path.exists():
            record=json.loads(self.state_path.read_text())
            if record.get('schema')==1 and (record['frame_id'],record['calibration_version'])==(frame_id,calibration_version):
                self.session=record['session'];self.counter=record['counter'];self.tracks=record['tracks']
                self.last_stamp=record.get('last_stamp',max((t['last_seen'] for t in self.tracks.values()),default=None))
                self.geometry_scope=record.get('geometry_scope')

    def update(self, observations, stamp, absence_evidence=None):
        if not np.isfinite(stamp) or (self.last_stamp is not None and stamp<=self.last_stamp):
            raise ValueError('Target updates require strictly increasing acquisition stamps')
        self.last_stamp=stamp;absence_evidence=absence_evidence or {}
        available={key:value for key,value in self.tracks.items() if 0<=stamp-value['last_seen']<=self.max_gap_s}
        pairs=[]
        for index,obs in enumerate(observations):
            feature=unit(obs['feature']);position=obs.get('position_common')
            for key,track in available.items():
                similarity=float(feature@unit(track['feature']))
                if similarity<self.similarity:continue
                dt=stamp-track['last_seen'];distance=None
                if position is not None and track['position'] is not None:
                    prediction=np.asarray(track['position'])+np.asarray(track['velocity'])*dt
                    distance=float(np.linalg.norm(np.asarray(position)-prediction))
                    if distance>self.gate_m:continue
                elif obs.get('label')!=track['label']:continue
                pairs.append((similarity-(distance or 0)/max(self.gate_m,1e-6)*.2,index,key))
        matches={};used=set()
        for score,index,key in sorted(pairs,reverse=True):
            if index in matches or key in used:continue
            rivals=[p for p in pairs if p[1]==index and p[2]!=key and p[2] not in used]
            if rivals and score-max(p[0] for p in rivals)<.04:continue
            matches[index]=key;used.add(key)
        output=[];events=[]
        for index,obs in enumerate(observations):
            key=matches.get(index)
            if key is None:
                self.counter+=1;key=f'{self.session}:{self.counter}'
            previous=self.tracks.get(key);position=obs.get('position_common');velocity=np.zeros(3)
            if previous and position is not None and previous['position'] is not None:
                dt=stamp-previous['last_seen']
                velocity=.5*np.asarray(previous['velocity'])+.5*(np.asarray(position)-previous['position'])/dt
            motion_valid=bool(previous and position is not None and previous['position'] is not None)
            speed=float(np.linalg.norm(velocity));state='moving' if motion_valid and speed>.15 else 'observed_static' if motion_valid else 'motion_unverified'
            reassociated=bool(previous and previous.get('missed',0)>0)
            track=dict(label=obs['label'],position=position,velocity=velocity.tolist(),last_seen=stamp,
                       feature=unit(obs['feature']).tolist(),missed=0,state=state,absence_confirmations=0,
                       was_moving=bool(speed>.15 or (previous and previous.get('was_moving'))),extent=obs.get('extent'),
                       static_observations=(previous.get('static_observations',0)+1 if previous and motion_valid and speed<=.15 else 0))
            self.tracks[key]=track
            output.append(dict(target_id=key,state=state,reassociated=reassociated,
                position_common=position,velocity_m_s=velocity.tolist() if position is not None else None,
                speed_m_s=speed if position is not None else None,
                direction_unit=(velocity/speed).tolist() if speed>.01 else None,
                predicted_position_1s=(np.asarray(position)+velocity).tolist() if motion_valid else None,
                frame_id=self.frame_id, motion_valid=motion_valid))
            if previous is None:events.append(dict(type='target_added',target_id=key,stamp=stamp))
            if reassociated:events.append(dict(type='target_reassociated',target_id=key,stamp=stamp))
            if previous and previous['state']!=state:events.append(dict(type='motion_state_changed',target_id=key,stamp=stamp))
            if previous and position is not None and previous['position'] is not None:
                displacement=float(np.linalg.norm(np.asarray(position)-previous['position']))
                if displacement>.2:events.append(dict(type='target_displaced',target_id=key,stamp=stamp,displacement_m=displacement))
            if previous and previous.get('extent') is not None and obs.get('extent') is not None:
                difference=float(np.linalg.norm(np.asarray(obs['extent'])-previous['extent']))
                if difference>.2:events.append(dict(type='structure_geometry_changed',target_id=key,stamp=stamp,extent_change_m=difference))
        for key,track in list(self.tracks.items()):
            if key in {r['target_id'] for r in output}:continue
            track['missed']+=1
            evidence=absence_evidence.get(key,'unverified')
            if evidence=='clear_line_of_sight':track['absence_confirmations']+=1
            else:track['absence_confirmations']=0
            track['state']=('transient_departed' if track.get('was_moving') else 'persistent_removed' if track.get('static_observations',0)>=3
                else 'disappeared_unclassified') if track['absence_confirmations']>=3 else 'occluded' if evidence=='occluded' else 'missing_unverified'
            events.append(dict(type=track['state'],target_id=key,stamp=stamp,evidence=evidence))
            if stamp-track['last_seen']>max(self.max_gap_s,10.):self.tracks.pop(key)
        self.save()
        return output,events

    def reset(self,reason):
        if self.state_path and self.state_path.exists():
            archived=self.state_path.with_name(self.state_path.stem+'.'+self.session+'.json')
            self.state_path.replace(archived)
        self.session=uuid.uuid4().hex;self.tracks={};self.counter=0;self.last_stamp=None
        self.save()

    def save(self):
        if not self.state_path:return
        self.state_path.parent.mkdir(parents=True,exist_ok=True)
        payload=dict(schema=1,frame_id=self.frame_id,calibration_version=self.calibration_version,
                     session=self.session,counter=self.counter,tracks=self.tracks,last_stamp=self.last_stamp,geometry_scope=self.geometry_scope)
        temporary=self.state_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(payload,allow_nan=False)+'\n');temporary.replace(self.state_path)
