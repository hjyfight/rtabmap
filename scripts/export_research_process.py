#!/usr/bin/env python3
"""Bounded, read-only views of coherent 3001 feature archives. No ROS subscription."""
import argparse
import base64
import json
import os
from pathlib import Path
import signal
import time

import numpy as np
from research_validation import interface_audit, sync_audit, load_policy, verification_revision, PROTOCOL

ROOT = Path(__file__).resolve().parents[1]
TASKS = ('pmf', 'rgbnir', 'avs')


def atomic_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False, separators=(',', ':')))
    temporary.replace(path)


def numeric(value, decimals=5):
    a = np.asarray(value)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite archived values')
    return np.round(a.astype(float), decimals).tolist()


def choose(count, maximum=2048):
    return np.linspace(0, count-1, min(count, maximum), dtype=int) if count else np.array([], int)


def picture(value, heat=False):
    import cv2
    a = np.asarray(value)
    if heat:
        a = np.nan_to_num(a.astype(np.float32))
        a = ((a-a.min())/max(float(np.ptp(a)), 1e-8)*255).astype(np.uint8)
        a = cv2.cvtColor(cv2.applyColorMap(a, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
    elif a.dtype == bool:
        a = a.astype(np.uint8)*255
    if a.ndim == 2:
        a = np.repeat(a[:, :, None], 3, 2)
    a = np.asarray(a, np.uint8)
    if max(a.shape[:2]) > 640:
        a = cv2.resize(a, (max(1, round(a.shape[1]*640/max(a.shape[:2]))),
                           max(1, round(a.shape[0]*640/max(a.shape[:2])))))
    ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(a, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 78])
    if not ok:
        raise ValueError('Image export failed')
    return base64.b64encode(encoded).decode('ascii')


def waveform(pcm, rate):
    pcm = np.asarray(pcm).ravel()
    edges = np.linspace(0, len(pcm), min(600, len(pcm))+1, dtype=int)
    return dict(rate=int(rate), duration_s=len(pcm)/float(rate), samples=len(pcm),
                envelope=[numeric([a/float(rate), pcm[a:b].min(), pcm[a:b].max()])
                          for a, b in zip(edges[:-1], edges[1:]) if b > a])


def timing(task, context):
    offsets = context.get('clock_offsets_s')
    rows = []
    if task == 'pmf':
        for i, r in enumerate(context.get('source_frames', [])):
            rows.append(dict(pair=i, a=r['rgb_stamp'], b=r['lidar_stamp'], gap_ms=r['gap_s']*1000))
        a_name, b_name = 'RGB', 'LiDAR'
    elif task == 'rgbnir':
        if context.get('rgb_stamp') is not None and context.get('nir_stamp') is not None:
            rows.append(dict(pair=0, a=context['rgb_stamp'], b=context['nir_stamp'],
                             gap_ms=context.get('sync_gap_s', 0)*1000))
        a_name, b_name = 'RGB', '近红外'
    else:
        for i, frame in enumerate(context.get('frame_stamps', [])):
            centre = context['audio_start']+.4875+i*.96
            rows.append(dict(pair=i, a=frame, b=centre, gap_ms=abs(frame-centre)*1000))
        a_name, b_name = 'RGB（校正后）', '声学窗中心（校正后）'
    if offsets:
        for row in rows:
            if task == 'avs':
                row['raw_a'] = row['a']-offsets['rgb']
                row['raw_b'] = row['b']-offsets['audio']
            else:
                row['raw_a'], row['raw_b'] = row['a'], row['b']
                row['a'] += offsets['rgb']
                row['b'] += offsets['lidar' if task == 'pmf' else 'nir']
    maximum = max((abs(r['a']-r['b'])*1000 for r in rows), default=None)
    return dict(rows=rows, a_name=a_name, b_name=b_name, explicit_offsets=offsets,
                clock=context.get('clock_alignment', {}),
                maximum_gap_ms=maximum,
                note='配对时间差与时钟偏移估计不等同于传感器硬件同步误差；旧归档未记录应用偏移时只展示已有时间。')


def build_sample(task, snapshot, archive):
    """Reject racing JSON/NPZ pairs, including equal sequence from another session."""
    result = snapshot.get('result') or {}
    meta = json.loads(str(archive['metadata']))
    if (meta.get('session'), meta.get('sequence'), meta.get('task')) != (
            snapshot.get('session_id'), result.get('sequence'), task):
        raise ValueError('snapshot/archive identity mismatch')
    context = meta.get('context', {})
    # Input evidence comes only from the selected archive. Live snapshot fields
    # are intentionally excluded, even when they look newer or more complete.
    fields = {}
    for record in context.get('inputs_used', []):
        fields.setdefault(record.get('modality'), dict(record))
    sample = dict(schema='research_process/1', task=task, session_id=meta['session'],
                  sequence=meta['sequence'], source_stamp=meta['source_stamp'], exported_at=time.time(),
                  context=context, model=meta.get('model', {}), timing=timing(task, context),
                  interface_example=fields, features=result.get('features', []),
                  images=result.get('images', {}), output_shape=result.get('output_shape'),
                  classes=result.get('classes', []), inference_ms=result.get('inference_s', 0)*1000,
                  feature_statistics=result.get('feature_statistics', []),
                  comparison=result.get('comparison'), reference_input=result.get('reference_input'),
                  evaluation=dict(verified=False, note='尚未导入同样本标注与性能评测记录；当前展示模型输出与处理机制。'),
                  visuals={}, tables={})
    sample['validation_revision'] = verification_revision(ROOT)
    sample['interface_validation'] = interface_audit(task, snapshot, meta, archive)
    sample['sync_validation'] = sync_audit(task, meta, sample['timing'], load_policy(ROOT), ROOT)
    populate_visuals(sample, task, context, archive)
    return sample


def populate_visuals(sample, task, context, arrays, maximum=2048):
    visuals, tables = sample['visuals'], sample['tables']
    keys = set(arrays.files) if hasattr(arrays, 'files') else set(arrays)
    def get(name):
        return arrays[name] if name in keys else None
    if task == 'pmf' and 'scene_xyz' in keys:
        xyz, rgb, normals = get('scene_xyz'), get('scene_rgb'), get('scene_normals')
        ids = choose(len(xyz), maximum)
        visuals['cloud'] = dict(xyz=numeric(xyz[ids]), rgb=rgb[ids].tolist(), count=len(xyz), frame=context.get('world_frame', 'icp_odom'))
        features = get('reference_ptv3_input_features')
        # Older samples contain exact inputs for this deterministic concatenation.
        reconstructed = features is None
        if reconstructed:
            features = np.concatenate([rgb.astype(np.float32)/127.5-1, normals], axis=1)
        point_ids = choose(len(xyz), 8)
        tables['concatenation'] = dict(indices=point_ids.tolist(), values=numeric(features[point_ids]),
            verified_shape=list(features.shape), origin='由同样本已归档RGB与法向量按原处理公式复算' if reconstructed else '实际对照分支输入归档',
            note='颜色3维＋法向量3维→6维；XYZ单独提供位置。对照分支与主模型分别运行。')
        if 'target_query_xyz' in keys and 'geometry_mask_features' in keys:
            q = get('target_query_xyz').reshape(-1, 3)
            f = get('geometry_mask_features')[0, :, :, 0]
            sensors = get('target_query_sensor')
            ids = choose(len(q), maximum)
            if f.shape[1] == len(q):
                feature_ids = ids
            elif 'target_point_to_voxel' in keys:
                mapping = get('target_point_to_voxel').reshape(-1)
                if len(mapping) != len(q) or mapping.max() >= f.shape[1]:
                    raise ValueError('geometry voxel mapping does not match query positions')
                feature_ids = mapping[ids]
            else:
                raise ValueError('geometry features do not match query positions')
            energy = np.sqrt(np.mean(f[:, feature_ids]**2, axis=0))
            visuals['geometry'] = dict(xyz=numeric(q[ids]), energy=numeric(energy),
                sensor=sensors[ids].tolist() if sensors is not None else [], count=len(q),
                frame=context.get('world_frame', 'icp_odom'), feature_shape=list(f.shape))
        if 'point_source_indices' in keys and 'source_lidar_poses' in keys:
            source = get('point_source_indices')
            poses = get('source_lidar_poses')
            frame = len(poses)-1
            raw = get('source_lidar_'+str(frame))
            if raw is not None:
                valid = np.isfinite(raw[:, :3]).all(1)
                raw_ids = np.flatnonzero(valid)
                chosen = raw_ids[choose(len(raw_ids), maximum)]
                selected_ids = source[source[:, 0] == frame, 1].astype(int)
                kept = selected_ids[choose(len(selected_ids), maximum)]
                visuals['filter'] = dict(before=numeric(raw[chosen, :3]), after=numeric(raw[kept, :3]),
                    before_count=len(raw), after_count=len(selected_ids), frame=context.get('lidar_frame', 'LiDAR'),
                    observation=frame, note='同一原始扫描坐标系；经过有效性、距离、投影、深度一致性与体素筛选后的保留点，数量变化不是降噪精度。'
                    if context.get('depth_consistency_enforced',True) else
                    '同一原始扫描坐标系；按当前参数进行有效性、距离、投影与体素筛选。')
        if 'projection_uv' in keys:
            uv = get('projection_uv'); render = get('render_indices'); ids = choose(len(uv), 24)
            visuals['correspondence'] = dict(uv=numeric(uv[ids]), xyz=numeric(xyz[render[ids]]),
                                              count=len(uv), frame=context.get('world_frame', 'icp_odom'))
        if 'source_rgb_intrinsics' in keys:
            tables['intrinsics'] = numeric(get('source_rgb_intrinsics').reshape(-1, 3, 3)[-1])
        if 'camera_from_lidar' in keys:
            tables['extrinsics'] = numeric(get('camera_from_lidar'))
        if 'source_depth_frames' in keys:
            visuals['depth'] = picture(get('source_depth_frames')[-1], heat=True)
    elif task == 'rgbnir':
        for name, output in [('source_nir', 'nir_raw'), ('input_registered_nir', 'nir_registered'), ('nir_valid', 'valid_mask')]:
            if name in keys:
                visuals[output] = picture(get(name))
        if 'projection_u' in keys and 'projection_v' in keys and 'nir_valid' in keys:
            valid = get('nir_valid'); y, x = np.where(valid); ids = choose(len(x), 24)
            u, v = get('projection_u'), get('projection_v')
            visuals['correspondence'] = dict(rgb_uv=numeric(np.c_[x[ids], y[ids]]),
                nir_uv=numeric(np.c_[u[y[ids], x[ids]], v[y[ids], x[ids]]]), count=len(x))
        if 'nir_from_rgb' in keys:
            tables['extrinsics'] = numeric(get('nir_from_rgb'))
        info = context.get('rgb_info', {})
        if info.get('k'):
            tables['intrinsics'] = numeric(np.asarray(info['k']).reshape(3, 3))
    elif task == 'avs':
        if 'input_pcm' in keys and 'input_sample_rate' in keys:
            visuals['waveform'] = waveform(get('input_pcm'), int(get('input_sample_rate')))
            import scipy.signal
            frequencies, power = scipy.signal.welch(get('input_pcm'), fs=int(get('input_sample_rate')), nperseg=1024)
            visuals['spectrum'] = dict(hz=numeric(frequencies, 1), db=numeric(10*np.log10(np.maximum(power, 1e-16)), 2))
        if 'input_log_mel' in keys:
            visuals['log_mel'] = picture(get('input_log_mel')[-1, 0].T[::-1], heat=True)
        if 'audio_visual_attention' in keys:
            attention = get('audio_visual_attention')[0, 0]
            visuals['audio_attention'] = picture(attention, heat=True)
            tables['audio_attention'] = dict(shape=list(attention.shape), sum=float(attention.sum()))
        if 'audio_channel_weights' in keys:
            tables['channel_weights'] = numeric(get('audio_channel_weights').ravel())
        if 'queries_visual_attention' in keys:
            attention = get('queries_visual_attention')[0].mean(0)
            start = 0
            for side in (28, 14, 7):
                visuals['query_attention_'+str(side)] = picture(attention[start:start+side*side].reshape(side, side), heat=True)
                start += side*side
            tables['query_attention'] = dict(shape=list(get('queries_visual_attention').shape), reduction='300个查询的平均注意力；三个视觉尺度分别显示')
        tables['audio_vectors'] = {name: numeric(get(name).ravel()) for name in ('audio_embedding', 'audio_projected') if name in keys}


def build_snapshot_sample(task, snapshot):
    """Display recorded JSON results without inventing missing NPZ input evidence."""
    result=snapshot.get('result') or {}
    if snapshot.get('task')!=task or not snapshot.get('session_id') or result.get('sequence') is None:
        raise ValueError('invalid result snapshot identity')
    context=result.get('context') or {}
    meta=dict(task=task,session=snapshot['session_id'],sequence=result['sequence'],context=context)
    sample=dict(schema='research_process/1',task=task,session_id=snapshot['session_id'],sequence=result['sequence'],
        source_stamp=result.get('source_stamp'),exported_at=time.time(),context=context,model=snapshot.get('model') or {},
        timing=timing(task,context),interface_example={},features=result.get('features',[]),images=result.get('images',{}),
        output_shape=result.get('output_shape'),classes=result.get('classes',[]),inference_ms=result.get('inference_s',0)*1000,
        feature_statistics=result.get('feature_statistics',[]),comparison=result.get('comparison'),
        reference_input=result.get('reference_input'),visuals={},tables={},archive_level='result_snapshot_only',
        evaluation=dict(verified=False,note='仅有原模型结果快照，输入及完整张量未归档；精度仍待实测。'),
        validation_revision=verification_revision(ROOT))
    sample['interface_validation']=dict(schema='research_interface_check/1',state='missing',
        batch_key=f"{task}:{snapshot['session_id']}:{result['sequence']}",protocol=PROTOCOL,inputs=[],features=[],
        checks=[dict(layer='输入—特征—输出',name='完整特征归档',state='missing',
                     detail='当前运行未保存NPZ，无法核对输入与数组；只展示JSON中的原模型输出。')],
        counts={'pass':0,'fail':0,'missing':1},note='没有将结果快照转写为已验证的输入或张量归档。')
    sample['sync_validation']=sync_audit(task,meta,sample['timing'],load_policy(ROOT),ROOT)
    if task=='avs':
        sample['tables']['audio_vectors']={f['name'].replace('.','_'):f['vector'] for f in sample['features']
            if f.get('name') in ('audio.embedding','audio.projected') and f.get('vector')}
    if task=='rgbnir' and context.get('rgb_info',{}).get('k'):
        sample['tables']['intrinsics']=numeric(np.asarray(context['rgb_info']['k']).reshape(3,3))
    matrix=context.get('camera_from_lidar') if task=='pmf' else context.get('nir_from_rgb')
    if matrix is not None:sample['tables']['extrinsics']=numeric(matrix)
    preview=result.get('input_preview')
    if preview is not None:
        expected=(task,snapshot['session_id'],result['sequence'],result.get('source_stamp'))
        actual=(preview.get('task'),preview.get('session_id'),preview.get('sequence'),preview.get('source_stamp'))
        if preview.get('schema')!='research_input_preview/1' or actual!=expected:
            raise ValueError('input preview identity mismatch')
        sample['visuals'].update(preview.get('visuals') or {})
        sample['tables'].update(preview.get('tables') or {})
        sample['input_preview_source']=preview.get('source')
        sample['input_preview_max_points']=preview.get('max_points')
        for record in preview.get('inputs_used',[]):
            sample['interface_example'][record.get('modality')]=dict(record)
        sample['interface_validation']['inputs']=preview.get('inputs_used',[])
        sample['interface_validation']['checks'][0]['detail']='当前提供同批次真实输入的抽样预览；完整输入与张量未保存，离线数组复核不可用。'
    return sample


def export_once(directory, cache, throttle, interval=6, allow_snapshot_only=False):
    output = directory/'process'; output.mkdir(exist_ok=True)
    status = dict(schema='research_process_status/1', published_at=time.time(), branches={})
    for task in TASKS:
        try:
            snapshot = json.loads((directory/(task+'.json')).read_text())
            result = snapshot.get('result') or {}
            identity = [snapshot.get('session_id'), result.get('sequence'), verification_revision(ROOT)]
            row = dict(state=snapshot.get('state'), detail=snapshot.get('detail'),
                       published_at=snapshot.get('published_at'), inputs=snapshot.get('inputs', {}),
                       sample_identity=cache.get(task), exporter_error=None)
            if result and identity != cache.get(task) and time.monotonic()-throttle.get(task, -interval) >= interval:
                throttle[task] = time.monotonic()
                try:
                    path=directory/(task+'_features.npz')
                    if allow_snapshot_only and (not path.exists() or path.stat().st_size==0):
                        sample=build_snapshot_sample(task,snapshot)
                    else:
                        with np.load(path, allow_pickle=False) as archive:
                            sample = build_sample(task, snapshot, archive)
                    atomic_json(output/(task+'.json'), sample)
                    cache[task] = identity
                except (ValueError, OSError, KeyError, EOFError) as error:
                    row['exporter_error'] = str(error)
            row['sample_identity'] = cache.get(task)
            status['branches'][task] = row
        except (OSError, ValueError) as error:
            status['branches'][task] = dict(state='waiting', detail='等待输入记录', exporter_error=str(error), sample_identity=cache.get(task))
    atomic_json(output/'status.json', status)
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=ROOT/'.tools/miti-research/root/usr/lib/miti/public/research')
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--interval', type=float, default=6.)
    parser.add_argument('--allow-snapshot-only',action='store_true',help='Display existing JSON results when no NPZ was saved; keep missing evidence explicit')
    args = parser.parse_args()
    os.nice(10)
    cache, throttle = {}, {}
    for task in TASKS:
        path = args.directory/'process'/(task+'.json')
        try:
            saved = json.loads(path.read_text())
            if saved.get('sequence', 0) > 0:
                cache[task] = [saved['session_id'], saved['sequence'], saved.get('validation_revision')]
        except (OSError, ValueError, KeyError):
            pass
    if args.once:
        result=export_once(args.directory, cache, throttle, args.interval,args.allow_snapshot_only)
        print(json.dumps({key:dict(sample=row.get('sample_identity'),error=row.get('exporter_error'))
                          for key,row in result['branches'].items()},ensure_ascii=False))
        return
    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    while True:
        export_once(args.directory, cache, throttle, args.interval,args.allow_snapshot_only)
        time.sleep(3)


if __name__ == '__main__':
    main()
