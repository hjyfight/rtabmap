"""Versioned, typed fusion contract validated at producer and consumer boundaries."""
import hashlib
import hmac
import json
import math
import numpy as np

MODALITIES=('rgb','infrared','lidar','audio')
SCHEMA='object_multimodal_fusion/1'


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()


def seal_result(value, key=None):
    payload={k:v for k,v in value.items() if k!='integrity'}
    encoded=canonical(payload)
    value['integrity']=dict(sha256=hashlib.sha256(encoded).hexdigest(),
        authentication='hmac_sha256' if key else 'unsigned',
        hmac_sha256=hmac.new(key,encoded,hashlib.sha256).hexdigest() if key else None)
    return value


def number(value, lower=None):
    return type(value) in (int,float) and math.isfinite(value) and (lower is None or value>=lower)


def vector(value, size, nullable=False):
    if value is None and nullable:return
    if not isinstance(value,list) or len(value)!=size or any(not number(v) for v in value):raise ValueError('Invalid numeric vector')


def numeric_tree(value):
    return all(numeric_tree(v) for v in value) if isinstance(value,list) else number(value)


def flags(value):
    names=['require_human_review','require_independent_review','prohibit_automatic_execution']
    if not isinstance(value,dict) or any(type(value.get(k)) is not bool for k in names):raise ValueError('Missing or nonboolean use restrictions')
    uses=value.get('restricted_use')
    if not isinstance(uses,list) or any(not isinstance(v,str) or not v for v in uses):raise ValueError('Invalid restricted use list')
    if (value['require_human_review'] or value['require_independent_review']) and not value['prohibit_automatic_execution']:
        raise ValueError('Pending review must prohibit automatic execution')


def validate_result(value, key=None):
    required=('schema','sequence','stamp','published_at','frame_id','model','calibration','sources',
              'targets','events','timing','safety','integrity')
    if not isinstance(value,dict) or any(k not in value for k in required) or value['schema']!=SCHEMA:
        raise ValueError('Missing or unsupported fusion contract fields')
    if not isinstance(value['frame_id'],str) or not value['frame_id'] or type(value['sequence']) is not int or value['sequence']<0:
        raise ValueError('Invalid result identity/frame')
    if any(not number(value[k],0) for k in ('stamp','published_at')):raise ValueError('Invalid result timestamps')
    for name in ('model','calibration','sources','timing','integrity'):
        if not isinstance(value[name],dict):raise ValueError('Invalid record '+name)
    if not isinstance(value['targets'],list) or len(value['targets'])>16 or not isinstance(value['events'],list):raise ValueError('Invalid target/event collection')
    encoded=canonical({k:v for k,v in value.items() if k!='integrity'})
    integrity=value['integrity']
    if hashlib.sha256(encoded).hexdigest()!=integrity.get('sha256'):raise ValueError('Fusion result integrity failed')
    auth=integrity.get('authentication');signature=integrity.get('hmac_sha256')
    if auth not in ('unsigned','hmac_sha256') or (auth=='unsigned' and signature is not None) or (auth=='hmac_sha256' and
            (not isinstance(signature,str) or len(signature)!=64 or any(c not in '0123456789abcdef' for c in signature))):
        raise ValueError('Invalid authentication envelope')
    if key and (auth!='hmac_sha256' or not hmac.compare_digest(hmac.new(key,encoded,hashlib.sha256).hexdigest(),signature)):
        raise ValueError('Fusion result source authentication failed')
    if set(value['sources'])!=set(MODALITIES):raise ValueError('All source slots must be represented')
    for name,source in value['sources'].items():
        if not isinstance(source,dict) or type(source.get('present')) is not bool or source.get('modality')!=name:
            raise ValueError('Invalid source slot identity')
        quality=source.get('quality')
        if not isinstance(quality,dict) or type(quality.get('usable')) is not bool or not number(quality.get('score'),0) or quality['score']>1 or quality.get('grade') not in ('invalid','unavailable','degraded','nominal'):
            raise ValueError('Invalid source quality')
        if source['present'] and (not source.get('source_id') or not source.get('frame_id') or not number(source.get('stamp'),0)):
            raise ValueError('Missing present-source provenance')
    timing=value['timing']
    for name in ('processing_s','queue_s','receipt_to_result_s'):
        if not number(timing.get(name),0):raise ValueError('Invalid measured timing '+name)
    if not isinstance(timing.get('boundary'),str) or not timing['boundary']:raise ValueError('Timing boundary required')
    for name in ('output_interval_s','task_period_s'):
        if timing.get(name) is not None and not number(timing[name],0):raise ValueError('Invalid timing '+name)
    identities=set()
    for target in value['targets']:
        required=('target_id','label','baseline_label','instance_id','position','motion','modalities','uncertainty','safety','features','quality','preprocessing')
        if not isinstance(target,dict) or any(f not in target for f in required):raise ValueError('Incomplete target record')
        identity=target['target_id']
        if not isinstance(identity,str) or not identity or identity in identities:raise ValueError('Duplicate/invalid target identity')
        identities.add(identity)
        if any(not isinstance(target[n],str) or not target[n] for n in ('label','baseline_label')) or type(target['instance_id']) is not int:
            raise ValueError('Invalid target label/instance identity')
        position=target['position'];motion=target['motion']
        if not isinstance(position,dict) or type(position.get('valid')) is not bool or position.get('frame_id')!=value['frame_id']:
            raise ValueError('Invalid target position frame/validity')
        vector(position.get('xyz_m'),3,nullable=not position['valid']);vector(position.get('extent_m'),3,nullable=True)
        if not position['valid'] and position.get('xyz_m') is not None:raise ValueError('Invalid position must be null')
        if position.get('extent_m') is not None and min(position['extent_m'])<0:raise ValueError('Negative extent')
        if not isinstance(motion,dict) or motion.get('target_id')!=identity or motion.get('frame_id')!=value['frame_id'] or type(motion.get('motion_valid')) is not bool:
            raise ValueError('Invalid target motion identity/frame')
        for n in ('velocity_m_s','direction_unit','predicted_position_1s'):vector(motion.get(n),3,nullable=True)
        if motion['motion_valid'] and (not position['valid'] or motion.get('velocity_m_s') is None or not number(motion.get('speed_m_s'),0)):
            raise ValueError('Invalid motion estimate')
        if not isinstance(target['modalities'],dict) or set(target['modalities'])!=set(MODALITIES):raise ValueError('Missing target modality provenance')
        features=target['features']
        if not isinstance(features,dict):raise ValueError('Target feature record required')
        for n in ('complete','fused_valid'):
            if type(features.get(n)) is not bool:raise ValueError('Invalid feature flags')
        valid=features.get('valid_mask')
        if not isinstance(valid,list) or len(valid)!=4 or any(type(v) is not bool for v in valid):raise ValueError('Invalid modality mask type')
        for name in ('independent','encoder','weights','joint','fused'):
            if not isinstance(features.get(name),list) or not numeric_tree(features[name]):raise ValueError('Non-numeric feature '+name)
        a=np.asarray(features.get('independent'),float);raw=np.asarray(features.get('encoder'),float);w=np.asarray(features.get('weights'),float)
        if a.shape!=(4,1024) or raw.shape!=a.shape or w.shape!=(4,) or not all(np.isfinite(v).all() for v in (a,raw,w)):
            raise ValueError('Invalid target feature layout')
        if (w<0).any() or not np.isclose(w.sum(),1,atol=1e-6):raise ValueError('Invalid target weights')
        joint=np.asarray(features.get('joint'),float);fused=np.asarray(features.get('fused'),float)
        if joint.shape!=(4096,) or fused.shape!=(1024,) or not np.isfinite(joint).all() or not np.isfinite(fused).all():raise ValueError('Invalid joint/fused vector')
        if not np.allclose(joint,a.ravel(),atol=1e-6):raise ValueError('Joint vector does not preserve slots')
        valid=np.asarray(valid,bool)
        if not valid.any() or np.count_nonzero(a[~valid]) or np.count_nonzero(raw[~valid]) or np.count_nonzero(w[~valid]):
            raise ValueError('Absent modalities must have zero slots/weights')
        norms=np.linalg.norm(raw[valid],axis=1)
        if (norms<=1e-12).any() or not np.allclose(a[valid],raw[valid]/norms[:,None],atol=2e-6):raise ValueError('Independent vectors must normalize encoder output')
        mean=(a*w[:,None]).sum(axis=0);norm=np.linalg.norm(mean);expected=mean/norm if norm>1e-12 else np.zeros(1024)
        if features['complete']!=bool(valid.all()) or features['fused_valid']!=bool(norm>1e-12) or not np.allclose(fused,expected,atol=2e-6):
            raise ValueError('Fusion output/flags do not match weighted computation')
        for i,name in enumerate(MODALITIES):
            record=target['modalities'][name]
            if not isinstance(record,dict) or record.get('target_id')!=identity or type(record.get('associated')) is not bool or type(record.get('used')) is not bool:
                raise ValueError('Invalid target modality association')
            if record['used']!=bool(valid[i]) or (record['used'] and (not record['associated'] or not value['sources'][name]['present'] or not value['sources'][name]['quality']['usable'])):
                raise ValueError('Used modality is not associated/present/usable')
            if record.get('source')!=value['sources'][name] or not isinstance(record.get('basis'),str):raise ValueError('Target source provenance changed')
        uncertainty=target['uncertainty']
        if not isinstance(uncertainty,dict) or type(uncertainty.get('accuracy_verified')) is not bool or not isinstance(uncertainty.get('reasons'),list):
            raise ValueError('Invalid target uncertainty')
        if uncertainty.get('missing_modalities')!=[n for i,n in enumerate(MODALITIES) if not valid[i]]:raise ValueError('Missing modalities contradict feature mask')
        conflicts=uncertainty.get('conflicts')
        if not isinstance(conflicts,list):raise ValueError('Conflict records required')
        for conflict in conflicts:
            if not isinstance(conflict,dict) or not conflict.get('type') or not conflict.get('handling') or not isinstance(conflict.get('participating_modalities'),list) or not set(conflict['participating_modalities'])<=set(MODALITIES):
                raise ValueError('Incomplete conflict type/modalities/handling')
        flags(target['safety'])
        if (not uncertainty['accuracy_verified'] or conflicts) and not target['safety']['prohibit_automatic_execution']:
            raise ValueError('Unvalidated/conflicting target cannot allow execution')
        if conflicts and not target['safety']['require_independent_review']:raise ValueError('Conflicts require independent review')
    flags(value['safety'])
    if any(t['safety']['prohibit_automatic_execution'] for t in value['targets']) and not value['safety']['prohibit_automatic_execution']:
        raise ValueError('Result must preserve target use restrictions')
    for event in value['events']:
        if not isinstance(event,dict) or not event.get('target_id') or not event.get('type') or not number(event.get('stamp'),0):raise ValueError('Invalid event record')
    return value


def require_control_approval(value, key=None):
    validate_result(value,key)
    if value['safety']['prohibit_automatic_execution'] or value['safety']['restricted_use'] or any(t['safety']['prohibit_automatic_execution'] or t['safety']['restricted_use'] for t in value['targets']):
        raise ValueError('Fusion result prohibits automatic execution')
    if not key:raise ValueError('Control consumer requires authenticated result')
    return value
