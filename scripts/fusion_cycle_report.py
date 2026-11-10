"""ROS-independent timing aggregation. No uncorrelated stage times are summed."""
from collections import Counter, defaultdict
import math
import statistics


MODEL_METRICS = {
    'rgb_backbone': 'emsanet_rgb_features',
    'depth_backbone': 'emsanet_depth_features',
    'feature_extraction': 'emsanet_feature_extraction',
    'rgbd_fusion': 'emsanet_feature_fusion',
    'context': 'emsanet_context',
    'decoders_and_postprocessing': 'emsanet_decode_and_postprocess',
    'model_forward': 'emsanet_model_forward',
    'model_other': 'emsanet_model_other',
}


def model_metrics(profile):
    """Validate coverage/accounting before exposing numbers as separate stages."""
    if profile.get('state') != 'valid' or profile.get('device') != 'cpu':
        raise ValueError('invalid/unsupported model profile')
    groups = profile['groups_ms']
    values = {name: groups[key] for key, name in MODEL_METRICS.items()}
    if any(not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 for v in values.values()):
        raise ValueError('invalid model stage duration')
    if not math.isclose(groups['feature_extraction'], groups['rgb_backbone']+groups['depth_backbone'], abs_tol=1e-6):
        raise ValueError('feature extraction accounting mismatch')
    parts = sum(groups[k] for k in ('rgb_backbone', 'depth_backbone', 'rgbd_fusion',
                                   'context', 'decoders_and_postprocessing', 'model_other'))
    if not math.isclose(parts, groups['model_forward'], abs_tol=1e-6):
        raise ValueError('model forward accounting mismatch')
    return values


def distribution(values):
    values = sorted(values)
    if not values:
        return dict(count=0, mean_ms=None, min_ms=None, p50_ms=None, p95_ms=None,
                    p99_ms=None, max_ms=None)
    def percentile(p):
        index = (len(values)-1)*p
        low = math.floor(index)
        return values[low] + (values[math.ceil(index)]-values[low])*(index-low)
    return dict(count=len(values), mean_ms=statistics.mean(values), min_ms=values[0],
                p50_ms=percentile(.5), p95_ms=percentile(.95), p99_ms=percentile(.99), max_ms=values[-1])


def identity(row):
    # node_stamp is originally a double in RTAB-Map. Preserve its exact value.
    stamp = float(row['node_stamp'])
    if not math.isfinite(stamp) or row['node_id'] <= 0:
        raise ValueError('invalid frame identity')
    return row['session_id'], row['node_id'], stamp.hex()


def milliseconds(end, start):
    if not isinstance(start, int) or not isinstance(end, int) or start <= 0 or end < start:
        raise ValueError('invalid/non-monotonic timing boundaries')
    return (end-start)/1e6


def summarize(events, period_ms=None, warmup=0, minimum_samples=30):
    if period_ms is not None and (not math.isfinite(period_ms) or period_ms <= 0):
        raise ValueError('period_ms must be finite and positive')
    if warmup < 0 or minimum_samples < 1:
        raise ValueError('invalid warmup/minimum_samples')
    unique, duplicates, invalid = {}, 0, 0
    for row in events:
        try:
            key = row['source'], row['run_id'], int(row['sequence'])
            if key in unique:
                duplicates += 1
            else:
                unique[key] = row
        except (KeyError, TypeError, ValueError):
            invalid += 1
    rows = sorted(unique.values(), key=lambda row: row.get('event_ns', 0))
    counts = Counter(f"{r['source']}/{r.get('event')}" for r in rows)
    sequences = defaultdict(set)
    for r in rows:
        sequences[r['source'], r['run_id']].add(int(r['sequence']))
    sequence_gaps = sum(max(s)-min(s)+1-len(s) for s in sequences.values() if s)
    dispatched = defaultdict(list)
    for r in rows:
        if r['source'] == 'keyframe_bridge' and r.get('event') == 'dispatched':
            try:
                dispatched[identity(r)].append(r)
            except (KeyError, ValueError, TypeError):
                invalid += 1
    metrics, cycles, first_cycles, run_counts = defaultdict(list), [], [], Counter()
    starts, terminal, matched_dispatches = set(), set(), set()
    bridge_queued, bridge_terminal = set(), set()
    faults = sum(n for k, n in counts.items() if any(
        s in k for s in ('/failed', '/dropped', '/decode_failed', '/invalid_input')))
    missing_bridge = 0
    model_coverage = Counter()
    for r in rows:
        source, event = r['source'], r.get('event')
        if source == 'native' and event == 'sample':
            for name, value in r.get('metrics_ms', {}).items():
                if isinstance(value, (int, float)) and math.isfinite(value) and value >= 0:
                    metrics[name].append(value)
                else:
                    invalid += 1
            continue
        if source not in ('keyframe_bridge', 'semantic_mapper'):
            continue
        try:
            key = (r['run_id'], identity(r))
            if source == 'keyframe_bridge':
                if event == 'queued':
                    bridge_queued.add(key)
                if event in ('decode_failed', 'invalid_input', 'dropped_overflow', 'dropped_reset'):
                    bridge_terminal.add(key)
                continue
            if source != 'semantic_mapper':
                continue
            if event == 'accepted':
                starts.add(key)
            if event in ('completed', 'failed', 'dropped', 'already_processed'):
                terminal.add(key)
                if event != 'completed':
                    candidates = [b for b in dispatched.get(identity(r), []) if r.get('clock_id') and
                                  r['clock_id'] == b.get('clock_id') and b['dispatch_ns'] <= r['event_ns']]
                    if candidates:
                        b = max(candidates, key=lambda b: b['dispatch_ns'])
                        bridge_terminal.add((b['run_id'], identity(b)))
            if event != 'completed':
                continue
            s = r['stages']
            boundaries = [r['received_ns'], r['submitted_ns'], s['worker_start_ns'], s['prepared_ns'],
                          s['detected_ns'], s['worker_end_ns'], s['fusion_start_ns'], s['fusion_end_ns'],
                          s['persist_end_ns'], s['publish_start_ns'], s['publish_end_ns']]
            for a, b in zip(boundaries, boundaries[1:]):
                milliseconds(b, a)
            values = {
                'mapper_cycle': milliseconds(s['publish_end_ns'], r['received_ns']),
                'mapper_wait': milliseconds(s['worker_start_ns'], r['received_ns']),
                'prepare': milliseconds(s['prepared_ns'], s['worker_start_ns']),
                'network_and_masks': milliseconds(s['detected_ns'], s['prepared_ns']),
                'projection_3d': milliseconds(s['worker_end_ns'], s['detected_ns']),
                'executor_wait': milliseconds(s['fusion_start_ns'], s['worker_end_ns']),
                'object_fusion': milliseconds(s['fusion_end_ns'], s['fusion_start_ns']),
                'persistence': milliseconds(s['persist_end_ns'], s['fusion_end_ns']),
                'publish': milliseconds(s['publish_end_ns'], s['publish_start_ns']),
            }
            candidates = [b for b in dispatched.get(identity(r), []) if r.get('clock_id') and
                          r['clock_id'] == b.get('clock_id') and
                          b['decoded_ns'] <= r['received_ns'] and
                          (b['run_id'], identity(b)) not in matched_dispatches]
            bridge = max(candidates, key=lambda b: b['dispatch_ns']) if candidates else None
            if bridge:
                milliseconds(bridge['dispatch_ns'], bridge['received_ns'])
                milliseconds(bridge['decoded_ns'], bridge['dispatch_ns'])
                milliseconds(bridge['published_ns'], bridge['decoded_ns'])
                values.update(bridge_to_semantic=milliseconds(s['publish_end_ns'], bridge['received_ns']),
                              bridge_wait=milliseconds(bridge['dispatch_ns'], bridge['received_ns']),
                              bridge_decode=milliseconds(bridge['decoded_ns'], bridge['dispatch_ns']))
                matched_dispatches.add((bridge['run_id'], identity(bridge)))
                bridge_terminal.add((bridge['run_id'], identity(bridge)))
            else:
                missing_bridge += 1
            profile = s.get('model_timing')
            if profile is None:
                model_coverage['not_available'] += 1
            else:
                try:
                    detail = model_metrics(profile)
                    if detail['emsanet_model_forward'] > values['network_and_masks'] + .01:
                        raise ValueError('model forward exceeds containing detector interval')
                    values.update(detail)
                    model_coverage['valid'] += 1
                except (KeyError, TypeError, ValueError):
                    model_coverage['invalid_or_unsupported'] += 1
            run_counts[r['run_id']] += 1
            is_warmup = run_counts[r['run_id']] <= warmup
            if run_counts[r['run_id']] == 1:
                first_cycles.append(dict(run_id=r['run_id'], values_ms=values))
            cycles.append(dict(run_id=r['run_id'], node_id=r['node_id'], node_stamp=r['node_stamp'],
                               session_id=r['session_id'], values_ms=values, warmup=is_warmup,
                               replay=bridge.get('replay') if bridge else None,
                               prediction_count=s.get('prediction_count'), valid_3d_count=s.get('valid_3d_count'),
                               objects_in_state=r.get('objects_in_state'),
                               model_timing_state=profile.get('state') if profile else None,
                               completed_ns=s['publish_end_ns']))
            if not is_warmup:
                for name, value in values.items():
                    metrics[name].append(value)
        except (KeyError, TypeError, ValueError, OverflowError):
            invalid += 1
    for run in run_counts:
        times = sorted(c['completed_ns'] for c in cycles if c['run_id'] == run and not c['warmup'])
        metrics['result_interval'].extend((b-a)/1e6 for a, b in zip(times, times[1:]))
    queue_trends = []
    for run in run_counts:
        measured = [c for c in cycles if c['run_id'] == run and 'bridge_wait' in c['values_ms']]
        if len(measured) > 1:
            queue_trends.append(dict(run_id=run, first_wait_ms=measured[0]['values_ms']['bridge_wait'],
                last_wait_ms=measured[-1]['values_ms']['bridge_wait'],
                growth_ms=measured[-1]['values_ms']['bridge_wait']-measured[0]['values_ms']['bridge_wait']))
    unfinished = len(starts-terminal)
    bridge_unfinished = len(bridge_queued-bridge_terminal)
    quality = dict(duplicate_events=duplicates, invalid_records=invalid, sequence_gaps=sequence_gaps,
                   failures_or_drops=faults, mapper_unfinished_at_end=unfinished,
                   bridge_unfinished_at_end=bridge_unfinished, completed_without_bridge=missing_bridge)
    stats = {name: distribution(values) for name, values in sorted(metrics.items())}
    for name in ('bridge_to_semantic', 'mapper_cycle', 'result_interval'):
        stats.setdefault(name, distribution([]))
    primary = metrics['bridge_to_semantic']
    if period_ms is None:
        verdict = 'not_configured'
    elif not primary:
        verdict = 'insufficient_data'
    elif max(primary) > period_ms:
        verdict = 'exceeded'
    elif len(primary) < minimum_samples or any(quality[k] for k in quality if k != 'duplicate_events'):
        verdict = 'inconclusive'
    else:
        verdict = 'within_limit_for_measured_scope'
    return dict(schema=1, primary_scope='mapData bridge callback entry -> semantic result publish return',
                clock='CLOCK_MONOTONIC; same boot_id required across processes',
                excluded_scope=['sensor exposure/acquisition and network transport',
                                'upstream synchronization, LiDAR/IMU processing and SLAM before mapData',
                                'consumer/display latency and multi-frame first-confirmation latency'],
                whole_system_compliance_proven=False, task_period_ms=period_ms,
                evaluation=verdict, minimum_samples=minimum_samples, warmup_per_mapper_run=warmup,
                threshold_exceedances=(sum(v > period_ms for v in primary) if period_ms is not None else None),
                sample_counts=dict(counts), quality=quality, metrics_ms=stats,
                model_timing_coverage=dict(model_coverage),
                first_cycles=first_cycles, queue_trends=queue_trends, cycles=cycles)


def markdown(report):
    names = {'bridge_to_semantic':'关键帧进入桥接器至语义结果发布（含排队）',
             'mapper_cycle':'语义节点接收至结果发布', 'bridge_wait':'桥接器排队',
             'bridge_decode':'关键帧解码', 'mapper_wait':'语义节点等待/排队',
             'prepare':'图像转换与准备', 'network_and_masks':'特征提取、网络融合与掩膜后处理',
             'projection_3d':'深度三维投影', 'executor_wait':'等待执行器取回推理结果',
             'object_fusion':'跨帧物体关联与融合', 'persistence':'语义数据库保存',
             'publish':'构建并提交结果发布', 'result_interval':'相邻新结果间隔（单独统计）',
             'icp_estimation':'原节点报告的 ICP 估计耗时', 'slam_total':'原节点报告的 SLAM 总处理耗时',
             'legacy_inference_projection':'语义状态中的推理与三维投影耗时（兼容字段，局部）'}
    names.update(icp_receive_interval='ICP 消息接收间隔（非处理耗时）',
                 slam_receive_interval='SLAM 消息接收间隔（非处理耗时）')
    names.update(emsanet_rgb_features='RGB 分支特征提取（各骨干阶段合计）',
                 emsanet_depth_features='深度分支特征提取（各骨干阶段合计）',
                 emsanet_feature_extraction='多源特征提取（RGB＋深度，不含融合层）',
                 emsanet_feature_fusion='RGB-D 特征融合层（含 SE 加权）',
                 emsanet_context='上下文模块',
                 emsanet_decode_and_postprocess='网络解码器与其后处理',
                 emsanet_model_forward='模型完整前向', emsanet_model_other='模型前向其余开销')
    names['TimingMem/Keypoints_detection/ms'] = 'SLAM 视觉关键点检测'
    names['TimingMem/Descriptors_extraction/ms'] = 'SLAM 视觉描述子提取'
    period = '未指定' if report['task_period_ms'] is None else f"{report['task_period_ms']} ms"
    lines = ['# 多源特征提取与融合周期测量', '',
             f"任务需求周期：{period}；判定：`{report['evaluation']}`。",
             '', '主口径为关键帧进入桥接器至语义结果发布，含排队。不是传感器采集到最终结果的全系统时延。',
             '雷达/IMU 里程计与 SLAM 原生耗时单独列出，未将并行、未关联的耗时相加。', '',
             '| 测量项 | 样本数 | 均值 ms | P50 ms | P95 ms | P99 ms | 最大 ms |',
             '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    def fmt(v):
        return '—' if v is None else f'{v:.3f}'
    for name, s in report['metrics_ms'].items():
        lines.append(f"| {names.get(name, name)} | {s['count']} | " +
                     ' | '.join(fmt(s[k]) for k in ('mean_ms','p50_ms','p95_ms','p99_ms','max_ms')) + ' |')
    lines.extend(['', f"每个语义进程排除的首批成功样本数：{report['warmup_per_mapper_run']}；"
                  '首帧计时仍保留在 JSON 的 first_cycles 中。', '',
                  '质量与覆盖：`' + str(report['quality']) + '`。', '',
                  '没有任务阈值时只报告测量值；无新样本不能判定通过。时间戳用于逐帧关联，不与主机墙上时间相减。'])
    if report.get('model_timing_coverage', {}).get('valid'):
        lines.extend(['', '模型分层计时覆盖：`' + str(report['model_timing_coverage']) + '`。',
            '多源特征提取为 RGB 与深度骨干各阶段耗时之和；RGB-D 特征融合为编码器各融合层耗时之和。',
            '当前 CPU 上这些模块串行交替执行，模型完整前向还包含上下文、解码/后处理及其余开销。',
            '分支特征提取、特征提取合计、完整前向是不同层级，不能将所有表格行再次相加。'])
    context = report.get('measurement', {})
    if context:
        lines.extend(['', '测量条件：`' + str(context.get('mode')) + '`。'])
        if context.get('configured_input_period_ms'):
            lines.append(f"回放输入周期 {context['configured_input_period_ms']} ms；"
                         f"发送 {context.get('sent_frames')} 帧；初始化耗时 {context.get('mapper_initialization_ms', 0):.3f} ms。")
        if context.get('objects_in_final_state') == 0:
            lines.append('本批回放结束时无三维物体对象，不能代表物体密集场景的投影与跨帧融合负载。')
        if context.get('limitations'):
            lines.append('')
        for note in context.get('limitations', []):
            lines.append('- ' + note)
    for trend in report.get('queue_trends', []):
        lines.append(f"\n该进程首末帧桥接排队时间：{trend['first_wait_ms']:.3f} → {trend['last_wait_ms']:.3f} ms。"
                     '预热排除不表示队列已达到稳态。')
    return '\n'.join(lines)+'\n'
