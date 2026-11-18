"""Real target-level four-modal inference with geometric evidence and conservative attribution."""
import hashlib
import base64
import json
import time
from pathlib import Path
import numpy as np
import cv2
from .fusion import MODALITIES,Sample,fuse_features
from .calibration import rigid_matrix,calibration_assessment
from .quality import signal_quality
from .target_association import target_geometry,select_target_points,acoustic_bearing,match_sound_to_target,acoustic_window,target_absence_evidence
from .target_tracking import TargetTracker,unit
from .standard_interface import seal_result,validate_result
from .object_policy import validate_task


def crop(image,mask):
    y,x=np.nonzero(mask)
    if not len(x):raise ValueError('Empty target mask')
    x1,x2,y1,y2=x.min(),x.max()+1,y.min(),y.max()+1
    region=image[y1:y2,x1:x2].copy();region[~mask[y1:y2,x1:x2]]=127
    size=max(region.shape[:2]);output=np.full((size,size)+region.shape[2:],127,np.uint8)
    top=(size-region.shape[0])//2;left=(size-region.shape[1])//2
    output[top:top+region.shape[0],left:left+region.shape[1]]=region
    return output


def preview(image):
    scale=min(1.,224/max(image.shape[:2]));thumbnail=cv2.resize(image,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
    if thumbnail.ndim==3:thumbnail=cv2.cvtColor(thumbnail,cv2.COLOR_RGB2BGR)
    ok,encoded=cv2.imencode('.png',thumbnail)
    if not ok:raise ValueError('Target thumbnail encoding failed')
    return base64.b64encode(encoded).decode('ascii')


class ObjectMultimodalEngine:
    def __init__(self,encoder,task,calibration,state_path=None):
        self.encoder,self.task,self.calibration=encoder,validate_task(task),calibration
        self.names=[row['name'] for row in task['classes']]
        self.policy_sha256=hashlib.sha256(json.dumps(task,sort_keys=True).encode()).hexdigest()
        texts=[template.format(name) for name in self.names for template in task['templates']]
        raw=encoder.encode_text(texts)
        raw=np.stack([unit(v) for v in raw]).reshape(len(self.names),len(task['templates']),1024)
        self.prototypes=np.stack([unit(v) for v in raw.mean(axis=1)])
        self.event_names=task.get('audio_event_classes',['person speaking','dog barking','vehicle engine','mechanical noise','impact','background noise'])
        event_texts=[f'a recording of {name}.' for name in self.event_names]
        self.event_prototypes=np.stack([unit(v) for v in encoder.encode_text(event_texts)])
        self.tracker=TargetTracker(task['tracking_frame'],calibration['version']+':'+calibration['profile_sha256'],
                                  max_gap_s=task.get('reassociation_s',3.),state_path=state_path)
        self.last_publish=None;self.durations=[];self.geometry_scope=self.tracker.geometry_scope

    def compute(self,bundle):
        started=time.monotonic();stamp=float(bundle['stamp']);rgb=bundle['rgb'];depth=bundle['depth_m']
        scope=bundle.get('geometry_scope_sha256')
        if bundle.get('reset_tracking') or (self.geometry_scope is not None and scope!=self.geometry_scope):
            self.tracker.reset('acquisition_timeline_or_runtime_geometry_changed')
        self.geometry_scope=scope
        self.tracker.geometry_scope=scope
        calibration_status=calibration_assessment(self.calibration,stamp=stamp,matched_gap_s=bundle.get('matched_gap_s'))
        common_from_rgb=bundle.get('common_from_rgb')
        if common_from_rgb is not None:common_from_rgb=rigid_matrix(common_from_rgb)
        targets=[]
        for row in bundle['instances'][:self.task.get('max_targets',4)]:
            mask=np.asarray(row['full_mask'],bool)
            if mask.shape!=rgb.shape[:2] or mask.sum()<64:continue
            geometry=target_geometry(mask,depth,bundle['rgb_k'],bundle.get('rgb_d'),bundle.get('rgb_distortion_model','plumb_bob'))
            position=None
            extent=None
            if geometry and common_from_rgb is not None:
                position=(np.asarray(geometry['position'])@common_from_rgb[:3,:3].T+common_from_rgb[:3,3]).tolist()
                extent=(np.abs(common_from_rgb[:3,:3])@np.asarray(geometry['extent'])).tolist()
            target=dict(row=row,mask=mask,geometry=geometry,extent_common=extent,position_common=position,samples={},modality_records={})
            target['samples']['rgb']=Sample(crop(rgb,mask),stamp,bundle['rgb_frame'],f"rgb:{bundle['sequence']}:{row['instance_id']}")
            targets.append(target)
        audio_match=dict(target_index=None,reason='no_audio',candidates=[]);bearing=None;window=None
        audio=bundle.get('audio');audio_sensor=self.calibration['sensors']['audio']
        if audio is not None:
            if audio_sensor['status']=='measured' and audio_sensor.get('microphone_positions_m') is not None and bundle.get('common_from_audio') is not None:
                try:
                    pcm,window=acoustic_window(audio.payload,audio.attributes['sample_rate'],stamp-audio.alignment_stamp)
                    bearing=acoustic_bearing(pcm,audio.attributes['sample_rate'],audio_sensor['microphone_positions_m'])
                    audio_match=match_sound_to_target(bearing,bundle['common_from_audio'],targets,
                        self.task.get('audio_angle_deg',12.),self.task.get('audio_ambiguity_deg',5.))
                except ValueError as error:audio_match['reason']=str(error)
            else:audio_match['reason']='audio_geometry_or_mounting_unverified'
        for index,target in enumerate(targets):
            mask=target['mask'];row=target['row']
            records=target['modality_records']
            records['rgb']=dict(associated=True,basis='original_rgb_instance_mask',source=bundle['sources']['rgb'])
            nir=bundle.get('nir_warped');valid=bundle.get('nir_valid')
            coverage=float((mask&valid).sum()/mask.sum()) if valid is not None else 0.
            nir_ok=nir is not None and coverage>=self.task.get('min_nir_coverage',.35)
            if nir_ok:
                target['samples']['infrared']=Sample(crop(nir,mask&valid),stamp,bundle.get('nir_frame','nir'),f"nir:{bundle['sequence']}:{row['instance_id']}")
            records['infrared']=dict(associated=nir_ok,basis='registered_depth_and_calibrated_stereo_projection' if nir_ok else 'missing_or_insufficient_coverage',
                                    coverage=coverage,source=bundle['sources']['infrared'])
            lidar=bundle.get('lidar');camera_from_lidar=bundle.get('camera_from_lidar')
            lidar_ok=False;lidar_detail={}
            if lidar is not None and camera_from_lidar is not None and self.calibration['sensors']['lidar']['status']=='measured':
                points,lidar_detail=select_target_points(lidar.payload,camera_from_lidar,bundle['rgb_k'],mask,depth,
                    distortion=bundle.get('rgb_d'),distortion_model=bundle.get('rgb_distortion_model','plumb_bob'))
                lidar_ok=len(points)>=512
                if lidar_ok:target['samples']['lidar']=Sample(points,lidar.stamp,bundle['rgb_frame'],lidar.source_id,
                    attributes=dict(units='m',source_frame=lidar.frame_id,target_roi=True),clock_offset_s=lidar.clock_offset_s)
            records['lidar']=dict(associated=lidar_ok,basis='projection_and_surface_depth_gate' if lidar_ok else 'missing_calibration_or_insufficient_target_points',
                                 geometry=lidar_detail,source=bundle['sources']['lidar'])
            sound_ok=(audio_match['target_index']==index and row['label'].lower() in self.task['acoustic_source_classes'])
            if sound_ok:
                target['samples']['audio']=Sample(pcm,audio.stamp+window['start_offset_s'],audio.frame_id,audio.source_id,
                    window['duration_s'],dict(audio.attributes,target_analysis_window=window),audio.clock_offset_s)
            records['audio']=dict(associated=sound_ok,basis=audio_match['reason'] if sound_ok else
                                 'target_not_acoustic_source_class' if audio_match['target_index']==index else audio_match['reason'],
                                 bearing=bearing,spatial_match=audio_match,source=bundle['sources']['audio'],analysis_window=window,
                                 event=dict(type='acoustic_activity' if bearing and bearing.get('valid') else 'unresolved',
                                     classification_verified=False,speech_transcription=None,
                                     command_execution_allowed=False,recording_duration_s=audio.duration if audio else None,
                                     source_position_m=None,position_status='bearing_only_no_acoustic_range'))
            if sound_ok:records['audio']['event']['associated_target_position_m']=target['position_common']
        output=[];track_inputs=[]
        for target in targets:
            samples=target['samples'];features,preprocessing,encoder_timing=self.encoder.encode(samples,stamp)
            quality={name:signal_quality(name,sample.payload) for name,sample in samples.items()}
            weights={name:float(self.task['base_weights'][MODALITIES.index(name)])*quality[name]['score']*
                target['modality_records'][name]['source'].get('quality',{}).get('score',0.) for name in samples}
            for name in list(features):
                source=target['modality_records'][name]['source']
                if source.get('present') is False or not source.get('quality',{}).get('usable',True):
                    features.pop(name);weights[name]=0.
            if not features or not any(weights.get(n,0)>0 for n in features):continue
            fused=fuse_features(features,weights,allow_partial=True)
            logits={name:unit(vector)@self.prototypes.T for name,vector in features.items()}
            predictions={name:self.names[int(score.argmax())] for name,score in logits.items()}
            if 'audio' in features:
                scores=unit(features['audio'])@self.event_prototypes.T
                target['modality_records']['audio']['event'].update(type='acoustic_event_candidate',
                    candidate_label=self.event_names[int(scores.argmax())],
                    candidate_cosine=float(scores.max()),score_kind='uncalibrated_cosine_similarity',
                    analysis_duration_s=window['duration_s'])
            conflicts=[]
            baseline=target['row']['label']
            if len(set(predictions.values()))>1 or any(p!=baseline for p in predictions.values()):
                conflicts.append(dict(type='semantic_label_disagreement',participating_modalities=list(predictions),
                    predictions=predictions,baseline_label=baseline,handling='retain_rgb_detection_and_require_review'))
            semantic=fused['fused']@self.prototypes.T;chosen=self.names[int(semantic.argmax())]
            label=target['row']['label'] if conflicts or not fused['fused_valid'] else chosen
            missing=[n for n in MODALITIES if n not in features]
            ys,xs=np.nonzero(target['mask']);edge=bool(xs.min()==0 or ys.min()==0 or xs.max()==rgb.shape[1]-1 or ys.max()==rgb.shape[0]-1)
            depth_fraction=float((target['mask']&np.isfinite(depth)&(depth>.1)&(depth<40)).sum()/target['mask'].sum())
            uncertainty=dict(reasons=['accuracy_not_validated']+(['missing_or_unassociated_modalities'] if missing else [])+
                (['semantic_conflict'] if conflicts else [])+(['target_pose_unavailable'] if target['position_common'] is None else [])+
                (['field_of_view_edge'] if edge else [])+(['incomplete_target_depth'] if depth_fraction<.8 else [])+
                (['hardware_synchronization_unverified'] if not calibration_status['synchronization_verified'] else []),
                missing_modalities=missing,conflicts=conflicts,semantic_scores_kind='cosine_similarity_not_calibrated_probability',
                depth_sigma_m=target['geometry']['depth_sigma_m'] if target['geometry'] else None,
                boundary_valid_fraction=target['modality_records']['infrared']['coverage'],
                depth_valid_fraction=depth_fraction,touches_field_of_view_edge=edge,
                modality_predictions=predictions,fused_suggestion=chosen,accuracy_verified=False)
            flags=dict(require_human_review=True,require_independent_review=bool(conflicts),
                       restricted_use=['visualization','recording','offline_analysis'],prohibit_automatic_execution=True)
            raw=np.zeros((4,1024),np.float32)
            for name,vector in features.items():raw[MODALITIES.index(name)]=vector
            record=dict(label=label,baseline_label=target['row']['label'],instance_id=target['row']['instance_id'],
                images={name:preview(samples[name].payload) for name in ('rgb','infrared') if name in features},
                target_mask_sha256=hashlib.sha256(np.ascontiguousarray(target['mask']).tobytes()).hexdigest(),
                fusion_level='feature',weight_policy=self.task['weight_policy'],
                structure=dict(state='unverified',basis='tracked_extent_and_position_changes',
                    state_semantics='No absolute open_closed_or_collapse_state_without_task_reference'),
                position=dict(frame_id=self.task['tracking_frame'],xyz_m=target['position_common'],
                              extent_m=target['extent_common'],valid=target['position_common'] is not None,
                              extent_basis='camera_quantile_bounds_rotated_to_common_axis_aligned_extent',
                              camera_geometry=target['geometry'],camera_frame_id=bundle['rgb_frame']),
                modalities=target['modality_records'],quality=quality,preprocessing=preprocessing,
                uncertainty=uncertainty,safety=flags,
                features=dict(encoder=raw.tolist(),independent=fused['independent'].tolist(),joint=fused['concat'].tolist(),
                    fused=fused['fused'].tolist(),valid_mask=fused['valid_mask'].tolist(),weights=fused['weights'].tolist(),
                    complete=bool(fused['complete']),fused_valid=bool(fused['fused_valid'])),encoder_timing_s=encoder_timing)
            for n in MODALITIES:
                record['modalities'][n]['used']=n in features
                record['modalities'][n]['target_id_pending']=True
                if n in samples:
                    record['modalities'][n]['feature_input']=samples[n].metadata()
            output.append(record)
            track_inputs.append(dict(label=record['baseline_label'],position_common=target['position_common'],
                                     feature=features.get('rgb',next(iter(features.values()))),
                                     extent=record['position']['extent_m']))
        absence=bundle.get('absence_evidence')
        if absence is None and common_from_rgb is not None:
            occupied=np.zeros(rgb.shape[:2],bool)
            for target in targets:occupied|=target['mask']
            absence=target_absence_evidence(self.tracker.tracks,common_from_rgb,bundle['rgb_k'],depth,occupied,
                bundle.get('rgb_d'),bundle.get('rgb_distortion_model','plumb_bob'))
        identities,events=self.tracker.update(track_inputs,stamp,absence)
        for record,identity in zip(output,identities):
            record['target_id']=identity['target_id'];record['motion']=identity
            for source in record['modalities'].values():
                source.pop('target_id_pending');source['target_id']=identity['target_id']
        result=dict(schema='object_multimodal_fusion/1',sequence=int(bundle['sequence']),stamp=stamp,
            published_at=time.time(),frame_id=self.task['tracking_frame'],
            model=dict(**self.encoder.provenance,policy_sha256=self.policy_sha256),
            calibration=calibration_status,sources=bundle['sources'],targets=output,
            events=events,safety=dict(require_human_review=True,require_independent_review=any(r['safety']['require_independent_review'] for r in output),
                restricted_use=['visualization','recording','offline_analysis'],prohibit_automatic_execution=True))
        result['calibration']['runtime_geometry']=bundle.get('runtime_calibration')
        finished=time.monotonic();processing=finished-started
        self.durations=(self.durations+[processing])[-256:]
        received=bundle.get('received_monotonic',started)
        timing=dict(processing_s=processing,core_compute_s=processing,preparation_s=bundle.get('preparation_s',0.),queue_s=max(0.,started-received),
            receipt_to_result_s=max(0.,finished-received),output_interval_s=None if self.last_publish is None else finished-self.last_publish,
            task_period_s=self.task.get('task_period_s'),period_verdict='not_assessed_task_period_unspecified' if self.task.get('task_period_s') is None else 'see_publication_timing_report',
            processing_p50_s=float(np.quantile(self.durations,.5)),processing_p95_s=float(np.quantile(self.durations,.95)),
            boundary='core_association_encoding_tracking_result_construction_excludes_node_preparation_seal_validation_serialization_publication',samples=len(self.durations))
        self.last_publish=finished
        result['timing']=timing
        return validate_result(seal_result(result))
