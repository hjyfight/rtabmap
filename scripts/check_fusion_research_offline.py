#!/usr/bin/env python3
"""Replay saved hardware inputs; never connect to ROS or replace live snapshots."""
import argparse
import base64
import hashlib
import html
import json
from pathlib import Path
import sys
import time
import numpy as np
import torch
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'src/rtabmap_semantic')]
from rtabmap_research.models import AVS
from rtabmap_research.rgbnir import RGBNIR
from rtabmap_research.indoor import IndoorComparison
from rtabmap_research.odin import ODINComparison
from rtabmap_research.replay import read_bundle


def read_npz(path):
    with np.load(path,allow_pickle=False) as archive:return {key:archive[key].copy() for key in archive.files}


def source(path):
    return dict(path=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def save(directory,key,model,result,stamp,context,inputs):
    result.update(sequence=1,source_stamp=stamp,inference_s=context.pop('inference_s'),
        context=context,accuracy_verified=False,feature_download=key+'_features.npz')
    value=dict(schema='downstream_fusion/1',session_id='offline-check',source_mode='offline_replay',
        task=key,published_at=time.time(),state='ready',detail='离线回放：已保存真实输入',
        model=model.metadata,inputs=inputs,result=result,ros_error='')
    (directory/(key+'.json')).write_text(json.dumps(value,ensure_ascii=False,allow_nan=False))
    np.savez_compressed(directory/(key+'_features.npz'),**model.trace.arrays,
        metadata=np.array(json.dumps(dict(source_mode='offline_replay',model=model.metadata,context=context,
            source_stamp=stamp,features=result['features'],annotations=result.get('annotations',[])),ensure_ascii=False)))
    for name,picture in result['images'].items():
        (directory/(key+'_'+name+'.jpg')).write_bytes(base64.b64decode(picture))


def rgbnir(cfg,device,directory,sample):
    metadata=json.loads((sample/'metadata.json').read_text())
    arrays=read_npz(sample/'arrays.npz')
    rgb=np.asarray(Image.open(sample/'rgb.png').convert('RGB'))
    nir=np.asarray(Image.open(sample/'nir.png').convert('L'))
    frame=dict(stamp=metadata['stamps']['rgb'],raw_stamp=metadata['stamps']['rgb'],frame=metadata['rgb_info']['frame_id'],
        rgb=rgb,depth=arrays['depth_m'],rgb_info=metadata['rgb_info'])
    row=dict(stamp=metadata['stamps']['nir'],raw_stamp=metadata['stamps']['nir'],image=nir,frame=metadata['nir_info']['frame_id'])
    # This capture already archives the factory reference transform, including stereo P.
    from rtabmap_semantic.fusion_geometry import projection,project_image
    matrix=np.asarray(metadata['nir_from_rgb'])
    u,v,valid=projection(arrays['depth_m'],metadata['rgb_info'],metadata['nir_info'],matrix)
    registered=project_image(nir,u,v)
    assert abs(row['stamp']-frame['stamp'])<=cfg['rgbnir_sync_tolerance_s']
    model=RGBNIR(ROOT/cfg['rgbnir_weights'],device,size=cfg['rgbnir_size'],confidence=cfg['rgbnir_confidence'],
        nir_weight=cfg['rgbnir_nir_weight'])
    start=time.perf_counter();result,_=model.infer(rgb,registered,valid);elapsed=time.perf_counter()-start
    r,n,m,_=model._tensor_inputs(rgb,registered,valid)
    with torch.inference_mode():pure=model.prediction_parts(model.net(r))[0].clone()
    zero=model.prediction_parts(model.forward_tensors(r,n,m,nir_weight=0))[0]
    torch.testing.assert_close(zero,pure,rtol=0,atol=0)
    mixed=model.prediction_parts(model.forward_tensors(r,n,m))[0]
    change=float((mixed-pure).abs().max())
    assert change>1e-7
    # forward_tensors clears its archive; restore the actual masks and inputs from infer.
    start=time.perf_counter();result,_=model.infer(rgb,registered,valid);elapsed=time.perf_counter()-start
    model.trace.arrays.update(source_nir=nir,source_depth_m=arrays['depth_m'],
        projection_u=u,projection_v=v,nir_from_rgb=matrix)
    context=dict(inference_s=elapsed,sync_gap_s=abs(row['stamp']-frame['stamp']),rgb_stamp=frame['stamp'],
        nir_stamp=row['stamp'],rgb_info=metadata['rgb_info'],nir_info=metadata['nir_info'],
        nir_from_rgb=matrix.tolist(),calibration=metadata['calibration_version'],
        sources=[source(sample/p) for p in ('rgb.png','nir.png','arrays.npz','metadata.json')],
        nir_used=True,lidar_used=False,zero_weight_equal=True,output_change_max_abs=change)
    save(directory,'rgbnir',model,result,frame['stamp'],context,
        dict(rgb=dict(stamp=frame['stamp'],width=rgb.shape[1],height=rgb.shape[0],frame=frame['frame']),
             nir=dict(stamp=row['stamp'],width=nir.shape[1],height=nir.shape[0],frame=row['frame'])))
    return dict(valid_coverage=result['valid_coverage'],instances=result['detection_count'],rgb_instances=result['baseline_count'],
        zero_weight_equal=True,output_change_max_abs=change,annotations=result['annotations'],model=model.metadata)


def indoor(cfg,device,directory):
    path=ROOT/'.tools/miti-research/root/usr/lib/miti/public/research/pmf_features.npz'
    arrays,metadata,bundle=read_bundle(path)
    context=metadata['context'];observations=bundle['observations'];calibration=bundle['calibration']
    odin=cfg.get('pmf_backend')=='odin_compare'
    model=ODINComparison(ROOT/cfg['odin_weights'],device) if odin else IndoorComparison(ROOT/cfg['ptv3_weights'],device)
    start=time.perf_counter();result,labels=model.infer_bundle(bundle);elapsed=time.perf_counter()-start
    # Check that the saved full logits map back to every point label, using archived indices.
    if odin:
        trace=model.trace.arrays
        np.testing.assert_array_equal(trace['semantic_scores'].argmax(0),trace['native_query_labels'])
        uv=trace['projection_uv'];indices=trace['render_indices']
        np.testing.assert_array_equal(labels[uv[:,1],uv[:,0]],trace['native_point_labels'][indices]+1)
        valid=trace['dense_valid'];dense=trace['dense_display_labels']
        assert np.count_nonzero(dense)==np.count_nonzero(valid) and not np.any(dense[~valid])
        for i,observation in enumerate(observations):
            selected=np.flatnonzero(trace['point_source_indices'][:,0]==i)
            raw=observation['points'][trace['point_source_indices'][selected,1],:3]
            pose=observation['world_from_lidar']
            np.testing.assert_allclose(trace['target_query_xyz'][0,selected],raw@pose[:3,:3].T+pose[:3,3],rtol=1e-6,atol=1e-6)
        context=dict(context,depth_used=True,input_fusion=model.metadata['input'])
    else:
        voxel=model.trace.arrays['point_logits'][0].argmax(1)
        np.testing.assert_array_equal(voxel[model.trace.arrays['voxel_inverse']],model.trace.arrays['point_labels'])
        np.testing.assert_array_equal(model.trace.arrays['inputs_concat'],
            np.concatenate([model.trace.arrays['inputs_rgb'],model.trace.arrays['inputs_lidar_normal']],axis=2))
    context=dict(context,inference_s=elapsed,sources=[source(path)],comparison=result['comparison'])
    stamp=float(observations[-1]['rgb_stamp'])
    save(directory,'pmf',model,result,stamp,context,
        dict(rgb=dict(stamp=stamp,width=labels.shape[1],height=labels.shape[0],frame=calibration['camera_frame']),
             lidar=dict(stamp=observations[-1]['lidar_stamp'],points=len(observations[-1]['points']),frame=calibration['lidar_frame'])))
    return dict(scene_points=result['scene_points'],projected_points=result['projected_points'],
        comparison=result['comparison'],classes=result['classes'],input_statistics=result.get('input_statistics'),model=model.metadata)


def audiovisual(cfg,device,directory):
    path=ROOT/'.tools/miti-research/root/usr/lib/miti/public/research/avs_features.npz'
    arrays=read_npz(path);metadata=json.loads(str(arrays['metadata']))
    frames=arrays['input_frames'];mel=arrays['input_log_mel']
    rgb=np.rint(np.clip((frames[-1]*np.asarray([.229,.224,.225],np.float32)[:,None,None]+
        np.asarray([.485,.456,.406],np.float32)[:,None,None])*255,0,255)).astype(np.uint8).transpose(1,2,0)
    model=AVS(ROOT/cfg['avs_weights'],device)
    start=time.perf_counter();result,labels=model.infer_preprocessed(frames,mel,rgb);elapsed=time.perf_counter()-start
    trace_arrays=dict(model.trace.arrays);trace_rows=list(model.trace.rows)
    fused=trace_arrays['audiovisual_fused'].copy()
    model.infer_preprocessed(frames,mel+.5,rgb)
    change=float(np.max(np.abs(fused-model.trace.arrays['audiovisual_fused'])))
    assert change>1e-7
    model.trace.arrays=trace_arrays;model.trace.rows=trace_rows
    context=dict(metadata['context'],inference_s=elapsed,sources=[source(path)],
        input_level='saved normalized video + genuine log-Mel; original PCM is absent',
        audio_log_mel_shift=.5,audio_change_max_abs=change,
        previous_label_agreement=float(np.mean(labels==arrays['output_labels'])))
    save(directory,'avs',model,result,metadata['source_stamp'],context,
        dict(rgb=dict(stamp=metadata['source_stamp'],width=224,height=224,frame='saved normalized RGB'),
             audio=dict(stamp=metadata['context']['audio_end'],rate=16000,channels=1)))
    return dict(output_shape=result['output_shape'],audio_change_max_abs=change,
        previous_label_agreement=context['previous_label_agreement'],classes=result['classes'],model=model.metadata)


def report_page(directory,report):
    cards=[]
    titles={'rgbnir':'RGB 90% + NIR 10% · YOLOv8-seg','pmf':'室内 RGB + 深度 + LiDAR · ODIN','avs':'RGB + 音频 · AVSegFormer'}
    for key in report['tasks']:
        value=json.loads((directory/(key+'.json')).read_text());result=value['result']
        keys=[('annotated_overlay','融合预测')]
        if key=='pmf':
            keys+=[('lidar_overlay','真实LiDAR点预测'),('reference_overlay','原PTv3 · 相同LiDAR点对照')] if value['model'].get('backend_kind')=='rgbd_lidar_odin' else [('no_rgb_overlay','同点云移除RGB输入'),('reference_overlay','原PMF道路类别对照')]
        elif key=='rgbnir':keys+=[('reference_overlay','同帧原RGB基线'),('projection','配准NIR')]
        else:keys+=[('raw','对应采集视频帧')]
        figures=''.join(f'<figure><img src="{key}_{name}.jpg"><figcaption>{caption}</figcaption></figure>' for name,caption in keys)
        details=html.escape(json.dumps(report['tasks'][key],ensure_ascii=False,indent=2))
        cards.append(f'<section><h2>{titles[key]}</h2><div class="pictures">{figures}</div><a href="{key}_features.npz">下载本次NPZ</a><details><summary>检查结果与来源</summary><pre>{details}</pre></details></section>')
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>3001融合离线检查</title>
<style>body{margin:24px;background:#101b27;color:#e2edf6;font:15px/1.6 system-ui}h1{font-size:24px}h2{font-size:19px}section{border:1px solid #395268;border-radius:10px;padding:16px;margin:20px 0}.pictures{display:flex;gap:12px;flex-wrap:wrap}figure{margin:0;flex:1;min-width:240px}img{width:100%;max-height:300px;object-fit:contain;background:#09111a}figcaption{color:#b3c7d8}a{color:#81e3d0}pre{white-space:pre-wrap;overflow-wrap:anywhere}.note{color:#edc98a}</style>
<h1>3001融合 · 离线检查</h1><p class="note">使用已保存的真实输入。当前实机未连接；以下预测与当前画面无关。检查证明模型链路能运行，尚未证明融合精度改善。</p>'''+''.join(cards)+'</html>'
    (directory/'index.html').write_text(page)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='cuda:0' if torch.cuda.is_available() else 'cpu')
    parser.add_argument('--tasks',nargs='+',choices=['rgbnir','pmf','avs'],default=['rgbnir','pmf','avs'])
    parser.add_argument('--sample',type=Path,default=ROOT/'logs/segmentation_fusion/capture_20261116_204011/samples/046fe947e53a74ecf307')
    args=parser.parse_args();torch.set_num_threads(2)
    if 'pmf' in args.tasks and not args.device.startswith('cuda'):
        parser.error('PTv3稀疏卷积需要GPU；仅CPU检查可用 --tasks rgbnir avs')
    directory=ROOT/'logs/research/offline';directory.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((ROOT/'config/fusion_research.json').read_text())
    report=dict(mode='saved_hardware_input_replay',device=args.device,accuracy_verified=False,tasks={})
    for key in args.tasks:
        print('Checking '+key,flush=True)
        fn={'rgbnir':rgbnir,'pmf':indoor,'avs':audiovisual}[key]
        report['tasks'][key]=fn(cfg,args.device,directory,args.sample) if key=='rgbnir' else fn(cfg,args.device,directory)
        (directory/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
        report_page(directory,report)
        if torch.cuda.is_available():torch.cuda.empty_cache()
        print(key+' passed',flush=True)
    print('Report: '+str(directory/'index.html'))


if __name__=='__main__':main()
