"""Contracts and measurement checks for the selected archived research sample."""
import json
import hashlib
import math
from pathlib import Path

import numpy as np

INPUT_FIELDS = [
    ('modality', '模态', 'string', '—', '本次实际参与处理的模态'),
    ('topic', '来源话题', 'string', '—', '原数据所在ROS话题'),
    ('message_type', '消息类型', 'string', '—', '实际ROS消息类型'),
    ('source_id', '来源标识', 'string', '—', '设备给出的标识或话题、坐标系与原始时间组成的采集标识'),
    ('raw_stamp', '原始采集时间', 'float64', 's', '原消息时间；估计采集时间必须另标时间质量'),
    ('aligned_stamp', '对齐时间', 'float64', 's', '原始时间加所记录的偏移'),
    ('clock_offset_s', '应用偏移', 'float64', 's', '本次配对实际使用的时钟偏移'),
    ('stamp_quality', '时间质量', 'string', '—', '区分传感器消息时间、主机估计时间与测量时间'),
    ('format', '数据格式', 'string', '—', '图像编码、点字段或PCM格式'),
    ('shape', '输入形状', 'int[]', 'px / 点 / 样本', '实际解码数据的形状'),
    ('dtype', '数据类型', 'string', '—', '实际解码数组类型'),
]
INPUT_CONDITIONAL = [
    ('frame', '空间坐标系', 'string', '—', '空间数据必需；纯声学嵌入不编造空间坐标'),
    ('rate', '声学采样率', 'uint32', 'Hz', '声学输入必需'),
    ('channels', '采集通道', 'uint16', '通道', '声学输入必需，模型的单通道转换另行注明'),
    ('duration_s', '声学窗口长度', 'float64', 's', '声学输入必需'),
]
PROTOCOL = {
    'schema': 'research_interfaces/1',
    'batch_key': 'task + session_id + sequence（NPZ metadata.session映射到session_id）',
    'layers': [
        {'name': '输入接口', 'transport': 'ROS 2 Image / PointCloud2 / AudioFrame；RGB-D按嵌套载荷注明来源',
         'fields': [dict(key=k, label=l, type=t, unit=u, required='必需', meaning=m)
                    for k, l, t, u, m in INPUT_FIELDS] +
                   [dict(key=k, label=l, type=t, unit=u, required='适用时必需', meaning=m)
                    for k, l, t, u, m in INPUT_CONDITIONAL]},
        {'name': '中间特征接口', 'transport': 'JSON描述 + NPZ数组', 'fields': [
            dict(key='name / archive_key', label='特征与数组标识', type='string', unit='—', required='必需', meaning='对应实际归档数组'),
            dict(key='shape / dtype', label='形状与类型', type='int[] / string', unit='维度', required='必需', meaning='完整模型形状与实际保存形状分别记录'),
            dict(key='operation', label='处理模块或操作', type='string', unit='—', required='必需', meaning='实际特征提取或交互位置'),
            dict(key='batch_key', label='输入批次', type='string', unit='—', required='必需', meaning='继承本次归档的模态、会话、序号与输入记录') ]},
        {'name': '融合结果接口', 'transport': 'ROS 2 std_msgs/msg/String承载JSON + 结果快照', 'fields': [
            dict(key='task / session_id / sequence', label='分支与批次', type='string / string / uint64', unit='—', required='必需', meaning='与特征NPZ身份一致'),
            dict(key='source_stamp', label='结果关联时间', type='float64', unit='s', required='必需', meaning='关联输入锚点；不是结果发布时间'),
            dict(key='model.sha256', label='模型标识', type='string', unit='—', required='必需', meaning='实际加载权重的哈希'),
            dict(key='output_shape / classes', label='输出结构与类别', type='int[] / object[]', unit='维度', required='必需', meaning='实际模型输出形状与类别定义'),
            dict(key='context', label='来源及坐标关系', type='object', unit='s / m（适用时）', required='必需', meaning='关联使用过的输入、时间与标定上下文') ]},
    ],
}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def array_info(archive, key):
    """Read NPY headers without decompressing full feature tensors."""
    if key not in archive.files:
        return None
    with archive.zip.open(key + '.npy') as stream:
        version = np.lib.format.read_magic(stream)
        if version == (1, 0):
            shape, _, dtype = np.lib.format.read_array_header_1_0(stream)
        elif version == (2, 0):
            shape, _, dtype = np.lib.format.read_array_header_2_0(stream)
        else:
            value = archive[key]
            return dict(shape=list(value.shape), dtype=str(value.dtype))
    return dict(shape=list(shape), dtype=str(dtype))


def input_record(modality, topic, message_type, raw_stamp, offset, *, frame='',
                 shape=None, dtype='', format='', source_id='', stamp_quality='sensor_message_stamp', **extra):
    return dict(modality=modality, topic=topic, message_type=message_type,
                raw_stamp=float(raw_stamp), aligned_stamp=float(raw_stamp + offset),
                clock_offset_s=float(offset), frame=frame, shape=list(shape or []),
                dtype=dtype, format=format,
                source_id=source_id or f'{topic}:{frame}:{raw_stamp:.9f}',
                stamp_quality=stamp_quality, **extra)


def load_policy(root):
    path = Path(root) / 'config/research_verification.json'
    if not path.exists():
        return dict(reference='未加载指标规范', sync_limit_ms=None, pair_check_limit_ms=None)
    policy = json.loads(path.read_text())
    if policy.get('schema') != 'research_verification/1':
        raise ValueError('Invalid verification policy schema')
    for key in ('sync_limit_ms', 'pair_check_limit_ms'):
        if not finite(policy.get(key)) or policy[key] <= 0:
            raise ValueError(f'Invalid {key}')
    return policy


def verification_revision(root):
    root = Path(root)
    policy = load_policy(root)
    content = json.dumps(policy, sort_keys=True).encode()
    path = root / policy.get('measurement_file', 'logs/research/sync_measurements.json')
    if path.is_file(): content += path.read_bytes()
    return hashlib.sha256(content).hexdigest()[:12]


def interface_audit(task, snapshot, meta, archive):
    checks = []
    def check(layer, name, state, detail):
        checks.append(dict(layer=layer, name=name, state=state, detail=str(detail)))
    batch = f'{task}:{meta.get("session")}:{meta.get("sequence")}'
    context = meta.get('context', {})
    inputs = context.get('inputs_used', [])
    if not inputs:
        check('输入', '实际输入归档', 'missing', '旧归档未保存选中输入字段；不使用最新快照补填')
    required_modalities = {'pmf': {'rgb', 'lidar'}, 'rgbnir': {'rgb', 'nir'}, 'avs': {'rgb', 'audio'}}[task]
    present = {r.get('modality') for r in inputs}
    check('输入', '参与模态', 'pass' if required_modalities <= present else 'missing',
          '、'.join(sorted(present)) or '未归档')
    records = []
    for i, row in enumerate(inputs):
        r = dict(row)
        label = f'{r.get("modality", "未命名")}[{i + 1}]'
        required = [k for k, *_ in INPUT_FIELDS]
        if r.get('modality') in ('rgb', 'nir', 'lidar', 'depth'):
            required.append('frame')
        if r.get('modality') == 'audio':
            required += ['rate', 'channels', 'duration_s']
        missing = [k for k in required if k not in r or r[k] is None or r[k] == '' or r[k] == []]
        invalid = []
        for k in ('raw_stamp', 'aligned_stamp', 'clock_offset_s'):
            if not finite(r.get(k)):
                invalid.append(k)
        shape = r.get('shape')
        if not isinstance(shape, list) or not shape or any(not isinstance(v, int) or v <= 0 for v in shape):
            invalid.append('shape')
        if r.get('modality') == 'audio':
            for k in ('rate', 'channels', 'duration_s'):
                if not finite(r.get(k)) or r[k] <= 0:
                    invalid.append(k)
        check('输入', label + ' 必需字段', 'missing' if missing else 'fail' if invalid else 'pass',
              '缺失：' + ', '.join(missing) if missing else '类型或范围错误：' + ', '.join(invalid) if invalid else '字段、类型及单位约定已核对')
        if not any(k in invalid for k in ('raw_stamp', 'aligned_stamp', 'clock_offset_s')):
            error = abs(r['aligned_stamp'] - (r['raw_stamp'] + r['clock_offset_s']))
            check('同步关系', label + ' 原始时间＋偏移', 'pass' if error <= 1e-6 else 'fail', f'差值 {error * 1000:.6f} ms')
        ref = r.get('array_ref', {})
        info = array_info(archive, ref.get('key', ''))
        if info is None:
            check('输入', label + ' 数据归档', 'missing', ref.get('key') or '没有数组关联')
        else:
            index = ref.get('index')
            valid_index = index is None or isinstance(index, int) and 0 <= index < info['shape'][0]
            check('输入', label + ' 数据归档', 'pass' if valid_index else 'fail',
                  f'{ref.get("key")} / {info["shape"]} / {info["dtype"]}' + (f' / 索引 {index}' if index is not None else ''))
            r['archive_data'] = info
            expected_shape = info['shape'][1:] if index is not None and valid_index else info['shape']
            if not r.get('used_sample_ranges'):
                check('输入', label + ' 解码形状及类型', 'pass' if r.get('shape') == expected_shape and r.get('dtype') == info['dtype'] else 'fail',
                      f'记录 {r.get("shape")} / {r.get("dtype")}；归档 {expected_shape} / {info["dtype"]}')
            else:
                ranges = r['used_sample_ranges']
                valid_ranges = all(isinstance(span, list) and len(span)==2 and
                    all(isinstance(v, int) for v in span) and 0 <= span[0] < span[1] <= info['shape'][0] for span in ranges)
                check('输入', label + ' 声学采样区间', 'pass' if valid_ranges and r.get('dtype') == info['dtype'] else 'fail',
                      f'实际贡献区间 {ranges}，采集通道 {r.get("channels")}，模型通道 {r.get("model_channels")}')
        r['batch_key'] = batch
        records.append(r)
    features = []
    for f in meta.get('features', []):
        name = f.get('name', '')
        key = name.replace('.', '_')
        info = array_info(archive, key)
        descriptor = dict(name=name, archive_key=key, full_shape=f.get('shape'),
                          saved_shape=info['shape'] if info else None, dtype=info['dtype'] if info else None,
                          operation=f.get('operation'), batch_key=batch)
        expected = f.get('saved_shape')
        state = 'missing' if not info or not name or not f.get('operation') else 'fail' if expected != info['shape'] else 'pass'
        check('特征', name or '未命名特征', state,
              '归档缺失' if not info else f'{info["shape"]} / {info["dtype"]}')
        features.append(descriptor)
    if not features:
        check('特征', '特征描述', 'missing', '没有特征描述记录')
    result = snapshot.get('result') or {}
    pairs = [('分支', task, meta.get('task')), ('会话', snapshot.get('session_id'), meta.get('session')),
             ('序号', result.get('sequence'), meta.get('sequence')),
             ('关联时间', result.get('source_stamp'), meta.get('source_stamp')),
             ('模型哈希', (snapshot.get('model') or {}).get('sha256'), (meta.get('model') or {}).get('sha256'))]
    for name, left, right in pairs:
        state = 'missing' if left is None or right is None or left == '' or right == '' else 'pass' if left == right else 'fail'
        check('输入—特征—输出', name, state, f'结果 {left} / 归档 {right}')
    output = context.get('output_contract')
    if not output:
        check('结果', '输出结构归档', 'missing', '旧归档没有输出结构描述')
    else:
        for name in ('output_shape', 'classes'):
            exists = name in output and name in result
            state = 'missing' if not exists else 'pass' if output[name] == result[name] else 'fail'
            check('结果', name, state, '归档与结果快照一致' if state == 'pass' else '缺失或不一致')
    state = 'fail' if any(c['state'] == 'fail' for c in checks) else 'missing' if any(c['state'] == 'missing' for c in checks) else 'pass'
    return dict(schema='research_interface_check/1', state=state, batch_key=batch,
                protocol=PROTOCOL, inputs=records, features=features, checks=checks,
                counts={key: sum(c['state'] == key for c in checks) for key in ('pass', 'fail', 'missing')},
                note='此结果仅表示字段及归档对应核查；现场功能合格结论仍对应验证记录。')


def sync_audit(task, meta, timing, policy, root=None):
    context = meta.get('context', {})
    rows = timing.get('rows', [])
    limit = policy.get('pair_check_limit_ms')
    gaps = [abs(r['a'] - r['b']) * 1000 for r in rows if finite(r.get('a')) and finite(r.get('b'))]
    statistics = dict(count=len(gaps), mean_ms=float(np.mean(gaps)) if gaps else None,
                      p95_ms=float(np.percentile(gaps, 95)) if gaps else None,
                      maximum_ms=max(gaps) if gaps else None,
                      over_limit_count=sum(g > limit + 1e-6 for g in gaps) if finite(limit) else None)
    pair_state = 'pending' if not gaps or not finite(limit) else 'fail' if statistics['over_limit_count'] else 'pass'
    effective_limit = policy.get('sync_limit_ms')
    clock = context.get('clock_alignment', {})
    measured = None
    measurement_error = None
    path = policy.get('measurement_file')
    if root and path:
        path = (Path(root) / path).resolve()
        if path.exists():
            try:
                data = json.loads(path.read_text())
                if data.get('schema') != 'research_sync_measurements/1':
                    raise ValueError('测量记录版本错误')
                matches = [r for r in data.get('records', []) if
                           (r.get('task'), r.get('session_id'), r.get('sequence')) ==
                           (task, meta.get('session'), meta.get('sequence'))]
                if len(matches) > 1:
                    raise ValueError('同批次存在多条同步测量记录')
                if matches:
                    measured = matches[0]
                    for k in ('max_error_ms', 'uncertainty_ms'):
                        if not finite(measured.get(k)) or measured[k] < 0:
                            raise ValueError('同步测量值或不确定度无效')
                    if not measured.get('method') or not measured.get('evidence') or measured.get('covers_acquisition_time') is not True:
                        raise ValueError('测量记录缺少方法、证据或采集时间覆盖说明')
            except (ValueError, TypeError, OSError) as error:
                measured = None
                measurement_error = str(error)
    measured_state = 'pending'
    if measured and finite(effective_limit):
        measured_state = 'pass' if measured['max_error_ms'] + measured['uncertainty_ms'] <= effective_limit else 'fail'
    overall = measured_state
    return dict(schema='research_sync_check/1', reference=policy.get('reference'),
                timebase='原始消息时间与本次归档偏移；各输入时间质量分别保留',
                formula='Δt_pair = |(tA + δA) − (tB + δB)|',
                pair_limit_ms=limit, runtime_tolerance_ms=context.get('runtime_pair_tolerance_ms'),
                statistics=statistics, pair_state=pair_state,
                sync_limit_ms=policy.get('sync_limit_ms'),
                effective_sync_limit_ms=effective_limit, measurement=measured,
                measurement_error=measurement_error, measurement_state=measured_state, state=overall,
                clock_method=clock.get('method', '未记录'),
                clock_uncertainty_ms=clock.get('round_trip_uncertainty_s') * 1000 if finite(clock.get('round_trip_uncertainty_s')) else None,
                clock_spread_ms=clock.get('offset_observed_spread_s') * 1000 if finite(clock.get('offset_observed_spread_s')) else None,
                acquisition_timestamp_quality=context.get('stamp_quality', '各传感器消息时间'),
                note='配对门限核查与同步精度测量分开判定；往返或接收延迟估计不能代替覆盖采集时间的同步测量。')
