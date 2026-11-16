"""Held-out full-frame and common-FOV mIoU, using only reviewed annotations."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def file_sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def confusion(truth,prediction,valid,classes):
    if truth.shape!=prediction.shape or truth.shape!=valid.shape:raise ValueError('Evaluation shapes differ')
    mask=valid&(truth>=0)&(truth<classes)
    pred=prediction[mask].astype(np.int64)
    if ((pred < -1)|(pred>=classes)).any():raise ValueError('Prediction outside task vocabulary')
    pred=np.where(pred<0,classes,pred)
    return np.bincount(truth[mask].astype(np.int64)*(classes+1)+pred,
        minlength=classes*(classes+1)).reshape(classes,classes+1)


def metrics(matrix):
    classes=matrix.shape[0];tp=np.diag(matrix[:,:classes]).astype(float)
    union=matrix.sum(axis=1)+matrix[:,:classes].sum(axis=0)-tp
    active=union>0
    iou=np.divide(tp,union,out=np.zeros_like(tp),where=active)
    return dict(miou=float(iou[active].mean()) if active.any() else None,
        pixel_accuracy=float(tp.sum()/matrix.sum()) if matrix.sum() else None,
        per_class_iou=[float(iou[i]) if active[i] else None for i in range(classes)],pixels=int(matrix.sum()))


def softmax(logits,temperature):
    if not np.isfinite(temperature) or temperature<=0:raise ValueError('Invalid calibration temperature')
    values=np.asarray(logits,np.float64)/temperature;values-=values.max(axis=-1,keepdims=True)
    if not np.isfinite(values).all():raise ValueError('Invalid fusion logits')
    exp=np.exp(values);return exp/exp.sum(axis=-1,keepdims=True)


def policy_prediction(arrays,objects,temperature,threshold,enabled=True):
    """The identical class-only policy used by evaluation and the ROS producer."""
    if not 0<=threshold<=1:raise ValueError('Invalid fusion confidence threshold')
    result=arrays['rgb_map'].copy();changes=0
    if not enabled:return result,changes
    if len(objects)!=len(arrays['logits']):raise ValueError('Fusion object/logit count mismatch')
    for i,row in enumerate(objects):
        logits=arrays['logits'][i,2]
        probability=softmax(logits,temperature)
        if row['eligible'] and float(probability.max())>=threshold:
            mask=arrays['instance_map']==row['instance_id']
            changes+=int((mask&(result!=int(np.argmax(logits)))).sum())
            result[mask]=int(np.argmax(logits))
    return result,changes


def gated_prediction(sample,temperature,threshold,enabled=True):
    return policy_prediction(sample['arrays'],sample['metadata']['objects'],temperature,threshold,enabled)


def evaluate(index_path,bootstrap_samples=1000,min_test_samples=30,min_test_groups=5):
    if bootstrap_samples<1:raise ValueError('Bootstrap sample count must be positive')
    index_path=Path(index_path);index=json.loads(index_path.read_text())
    classes=len(index['classes']);rows=index['samples']
    missing=[];unreviewed=[];unassigned=[];groups={};identities=set();annotation_paths=set();loaded=[];hashes=[]
    for row in rows:
        if row['sample_id'] in identities:raise ValueError('Duplicate sample ID')
        identities.add(row['sample_id'])
        annotation=(index_path.parent/row['annotation']).resolve()
        if annotation in annotation_paths:raise ValueError('Reused annotation path')
        annotation_paths.add(annotation)
        split=row.get('split','unassigned');group=row.get('group_id','')
        if split not in ('validation','test') or not group:unassigned.append(row['sample_id'])
        else:
            if group in groups and groups[group]!=split:raise ValueError('Scene/group leakage across validation and test')
            groups[group]=split
        if not annotation.is_file():missing.append(row['sample_id'])
        if row.get('annotation_source')!='human_reviewed':unreviewed.append(row['sample_id'])
        if not annotation.is_file() or row.get('annotation_source')!='human_reviewed' or split not in ('validation','test'):continue
        folder=(index_path.parent/row['sample_path']).resolve()
        metadata=json.loads((folder/'metadata.json').read_text())
        if metadata['sample_id']!=row['sample_id']:raise ValueError('Sample metadata identity mismatch')
        if metadata['task_sha256']!=index['task_sha256']:raise ValueError('Task vocabulary mismatch')
        with np.load(folder/'arrays.npz',allow_pickle=False) as source:arrays={k:source[k] for k in source.files}
        truth=cv2.imread(str(annotation),cv2.IMREAD_UNCHANGED)
        if truth is None or truth.ndim!=2:raise ValueError('Annotation must be a single-channel indexed PNG')
        if not np.isin(truth,np.r_[np.arange(classes),65534,65535]).all():raise ValueError('Unknown annotation class ID')
        truth=truth.astype(np.int32);truth[truth>=65534]=-1
        loaded.append(dict(entry=row,metadata=metadata,arrays=arrays,truth=truth))
        hashes.append(dict(sample_id=row['sample_id'],split=split,group_id=group,
            arrays=file_sha(folder/'arrays.npz'),annotation=file_sha(annotation),metadata=file_sha(folder/'metadata.json')))
    report=dict(schema=1,passed=False,status='needs_annotations',scope='paired_inputs_on_full_rgb_reference_frame',
        total_samples=len(rows),human_reviewed_samples=len(loaded),missing_annotations=len(missing),
        unreviewed_annotations=len(unreviewed),unassigned_samples=len(unassigned),classes=index['classes'],
        task_sha256=index['task_sha256'],target='fusion_miou >= max(rgb_miou, nir_miou)',
        test_data_used_for_policy_selection=False)
    if not rows or missing or unreviewed or unassigned:return report
    validation=[r for r in loaded if r['entry']['split']=='validation']
    test=[r for r in loaded if r['entry']['split']=='test']
    if not validation or not test:report['status']='needs_independent_splits';return report
    pipelines={r['metadata']['pipeline_sha256'] for r in loaded}
    if len(pipelines)!=1:raise ValueError('Mixed model/calibration/policy fingerprints: evaluate each fixed pipeline separately')
    logits=[];targets=[]
    for sample in validation:
        for i,row in enumerate(sample['metadata']['objects']):
            mask=(sample['arrays']['instance_map']==row['instance_id'])&(sample['truth']>=0)
            values=sample['truth'][mask]
            if len(values)<20:continue
            counts=np.bincount(values,minlength=classes)
            if counts.max()/len(values)<.5:continue
            logits.append(sample['arrays']['logits'][i,2]);targets.append(int(counts.argmax()))
    if len(logits)<20:report.update(status='needs_more_validation_objects',validation_objects=len(logits));return report
    logits=np.asarray(logits);targets=np.asarray(targets)
    temperatures=np.geomspace(.005,1.,60)
    losses=[float(-np.log(np.maximum(softmax(logits,t)[np.arange(len(targets)),targets],1e-12)).mean()) for t in temperatures]
    temperature=float(temperatures[int(np.argmin(losses))])
    def score(samples,threshold,enabled,common_fov=False):
        matrices={k:np.zeros((classes,classes+1),np.int64) for k in ('rgb','nir','fusion')};changes=0;by_group={}
        for sample in samples:
            arrays=sample['arrays'];pred,n=gated_prediction(sample,temperature,threshold,enabled);changes+=n
            valid=arrays['eval_mask'] if common_fov else np.ones_like(arrays['eval_mask'],dtype=bool)
            local={k:confusion(sample['truth'],p,valid,classes) for k,p in
                [('rgb',arrays['rgb_map']),('nir',arrays['nir_map']),('fusion',pred)]}
            group=sample['entry']['group_id']
            if group not in by_group:by_group[group]={k:np.zeros_like(v) for k,v in local.items()}
            for k in local:matrices[k]+=local[k];by_group[group][k]+=local[k]
        return matrices,changes,by_group
    options=[]
    for enabled,threshold in [(False,1.),*[(True,t) for t in (0.,.5,.7,.8,.9,.95)]]:
        matrices,changes,_=score(validation,threshold,enabled)
        value=metrics(matrices['fusion'])['miou']
        if value is None:raise ValueError('No labeled validation pixels in common FOV')
        options.append((value,-changes,enabled,threshold))
    _,_,enabled,threshold=max(options,key=lambda r:(r[0],r[1],not r[2]))
    matrices,changes,by_group=score(test,threshold,enabled)
    results={k:metrics(v) for k,v in matrices.items()}
    common,_,common_groups=score(test,threshold,enabled,common_fov=True)
    common_results={k:metrics(v) for k,v in common.items()}
    if any(v['miou'] is None for v in results.values()) or any(v['miou'] is None for v in common_results.values()):
        raise ValueError('No labeled test pixels in the full frame or common FOV')
    rng=np.random.default_rng(42);group_names=list(by_group);differences=[];common_differences=[]
    for _ in range(bootstrap_samples):
        totals={k:np.zeros_like(v) for k,v in matrices.items()}
        common_totals={k:np.zeros_like(v) for k,v in common.items()}
        for index_ in rng.integers(0,len(group_names),len(group_names)):
            for k in totals:
                totals[k]+=by_group[group_names[index_]][k]
                common_totals[k]+=common_groups[group_names[index_]][k]
        values={k:metrics(v)['miou'] for k,v in totals.items()}
        if all(v is not None for v in values.values()):differences.append(values['fusion']-max(values['rgb'],values['nir']))
        common_values={k:metrics(v)['miou'] for k,v in common_totals.items()}
        if all(v is not None for v in common_values.values()):
            common_differences.append(common_values['fusion']-max(common_values['rgb'],common_values['nir']))
    lower=float(np.quantile(differences,.05)) if differences else None
    delta=results['fusion']['miou']-max(results['rgb']['miou'],results['nir']['miou'])
    common_lower=float(np.quantile(common_differences,.05)) if common_differences else None
    common_delta=common_results['fusion']['miou']-max(common_results['rgb']['miou'],common_results['nir']['miou'])
    enough=len(test)>=min_test_samples and len(group_names)>=min_test_groups
    passed=bool(enough and lower is not None and lower>=0 and delta>=0 and
        common_lower is not None and common_lower>=0 and common_delta>=0)
    candidate,_,_=score(test,0.,True)
    report.update(status='passed' if passed else 'not_passed',passed=passed,metrics=results,
        delta_vs_best_single=delta,one_sided_95pct_lower_bound=lower,test_samples=len(test),test_groups=len(group_names),
        minimum_test_samples=min_test_samples,minimum_test_groups=min_test_groups,validation_samples=len(validation),
        validation_objects=len(logits),temperature=temperature,validation_nll=float(min(losses)),
        confidence_threshold=float(threshold),fusion_enabled=bool(enabled),changed_test_pixels=changes,
        pipeline_sha256=next(iter(pipelines)),rgb_model_sha256=loaded[0]['metadata']['rgb_model'],
        data_fingerprint=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest(),
        bootstrap_samples=bootstrap_samples,bootstrap_unit='scene_group',
        common_fov_metrics=common_results,common_fov_delta_vs_best_single=common_delta,
        common_fov_one_sided_95pct_lower_bound=common_lower,
        candidate_metrics={k:metrics(v) for k,v in candidate.items()},
        selected_policy='fusion_with_rgb_fallback' if enabled else 'rgb_baseline',
        limitation='Only the human-annotated paired-input test distribution on the full RGB reference grid; not a per-frame, dropout-distribution or future-scene guarantee.')
    report['report_sha256']=hashlib.sha256(json.dumps(report,sort_keys=True).encode()).hexdigest()
    return report


def load_passed_report(path):
    report=json.loads(Path(path).read_text());expected=report.pop('report_sha256',None)
    actual=hashlib.sha256(json.dumps(report,sort_keys=True).encode()).hexdigest()
    if expected!=actual or report.get('passed') is not True or report.get('status')!='passed':
        raise ValueError('No intact passed held-out evaluation report')
    if any(report.get(k,-1) is None or report.get(k,-1)<0 for k in
            ('one_sided_95pct_lower_bound','delta_vs_best_single',
             'common_fov_one_sided_95pct_lower_bound','common_fov_delta_vs_best_single')):
        raise ValueError('Fusion did not meet the noninferiority target')
    if report.get('test_groups',0)<5 or report.get('test_samples',0)<30:
        raise ValueError('Insufficient held-out evaluation coverage')
    if report.get('bootstrap_samples',0)<1000 or report.get('test_data_used_for_policy_selection') is not False:
        raise ValueError('Insufficient resampling or test data used for policy selection')
    report['report_sha256']=expected
    return report
