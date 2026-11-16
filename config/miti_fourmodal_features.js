/* Native MITI feature panel and one-time layout addition. Uses MITI's React instance. */
function rtabmapFourmodalWidget(id = 'rtabmap-fourmodal') {
  return {i: id, type: 'fourmodal-features', title: '四模态 · 输入 → 特征 → 融合',
    x: 0, y: 24, w: 8, h: 10, props: {topic: '/miti/fourmodal_features'}};
}

function rtabmapAudioSpectrumWidget(id = 'rtabmap-audio-spectrum') {
  return {i: id, type: 'camera-viewer', title: '声学 · log-Mel 声谱',
    x: 8, y: 24, w: 4, h: 10, props: {topic: '/miti/audio_spectrum/image_raw', maxFps: 2}};
}

function rtabmapMigrateFourmodalLayout(layout) {
  if (!layout || !Array.isArray(layout.widgets)) return layout;
  const leaves = tree => typeof tree === 'string' ? [tree] :
    tree ? [...leaves(tree.first), ...leaves(tree.second)] : [];
  const anchor = layout.widgets.find(w => w.i === 'rtabmap-scene' && w.type === 'urdf-viewer' &&
    leaves(layout.mosaic).includes(w.i));
  if (!anchor || anchor.rtabmapFourmodalLayoutVersion === 1) return layout;
  const widgets = layout.widgets.map(w => w === anchor ? {...w, rtabmapFourmodalLayoutVersion: 1} : w);
  const added = [];
  for (const [factory, present] of [
    [rtabmapFourmodalWidget, w => w.type === 'fourmodal-features'],
    [rtabmapAudioSpectrumWidget, w => w.type === 'camera-viewer' && w.props?.topic === '/miti/audio_spectrum/image_raw']
  ]) {
    if (widgets.some(present)) continue;
    let id = factory().i, suffix = 2;
    while (widgets.some(w => w.i === id)) id = factory().i + '-' + suffix++;
    const widget = factory(id);
    widgets.push(widget); added.push(id);
  }
  if (!added.length) return {...layout, widgets};
  const extra = added.length === 1 ? added[0] :
    {direction: 'row', first: added[0], second: added[1], splitPercentage: 70};
  return {...layout, widgets, mosaic: {direction: 'column', first: layout.mosaic,
    second: extra, splitPercentage: 70}};
}

function rtabmapFourmodalFactory(React) {
  'use strict';
  const h = React.createElement;
  const order = ['rgb', 'infrared', 'lidar', 'audio'];
  const names = ['可见光 RGB', '近红外 NIR', 'LiDAR 点云', '声学 PCM'];
  const colors = ['#6bb8fa', '#c3a4fa', '#60d8bd', '#f3bc6b', '#f3e3a4'];
  const finite = value => typeof value === 'number' && Number.isFinite(value);
  const vector = (value, size = 1024) => Array.isArray(value) && value.length === size && value.every(finite);
  function parseSnapshot(message) {
    try {
      if (typeof message?.data !== 'string' || message.data.length > 650000) return null;
      const v = JSON.parse(message.data);
      if (v?.schema !== 1 || !['live', 'saved'].includes(v.mode) ||
          !finite(v.published_at) || !Number.isSafeInteger(v.sequence) || v.sequence < 0 ||
          v.feature_dim !== 1024 || v.joint_dim !== 4096 ||
          typeof v.complete !== 'boolean' || typeof v.fused_valid !== 'boolean' ||
          !finite(v.inference_s) || v.inference_s < 0 || !vector(v.fused_features) ||
          !Array.isArray(v.modalities) || v.modalities.length !== 4 ||
          !v.modalities.every((row, i) => row?.name === order[i] &&
            typeof row.valid === 'boolean' && finite(row.weight) && row.weight >= 0 && row.weight <= 1 &&
            (!row.valid ? row.weight === 0 : true) && vector(row.features) &&
            (row.gap_s === null || (finite(row.gap_s) && row.gap_s >= 0))) ||
          v.complete !== v.modalities.every(row => row.valid) ||
          !Array.isArray(v.cosine_similarities) || v.cosine_similarities.length !== 4 ||
          !v.cosine_similarities.every((row, i) => Array.isArray(row) && row.length === 4 &&
            row.every((x, j) => v.modalities[i].valid && v.modalities[j].valid ?
              finite(x) && x >= -1 && x <= 1 : x === null))) return null;
      if (v.flow_version !== undefined && (v.flow_version !== 1 ||
          !vector(v.joint_features, 4096) || !vector(v.weighted_mean_features) ||
          !finite(v.weighted_mean_norm) || v.weighted_mean_norm < 0 ||
          !finite(v.fused_norm) || v.fused_norm < 0 ||
          !v.modalities.every(row => vector(row.encoder_features) &&
            finite(row.encoder_norm) && row.encoder_norm >= 0 &&
            finite(row.normalized_norm) && row.normalized_norm >= 0))) return null;
      return v;
    } catch (_) { return null; }
  }
  function parseStatus(message) {
    try {
      const v = typeof message?.data === 'string' && message.data.length < 16384 ? JSON.parse(message.data) : null;
      return v?.schema === 1 && finite(v.published_at) && typeof v.state === 'string' &&
        finite(v.max_input_age_s) && v.max_input_age_s > 0 &&
        order.every(name => v.input_ages_s?.[name] === null ||
          (finite(v.input_ages_s?.[name]) && v.input_ages_s[name] >= 0)) ? v : null;
    } catch (_) { return null; }
  }
  function deriveView(sample, status, now, connected, error = '') {
    const data = sample?.snapshot;
    if (data?.mode === 'saved') return {data, mode: 'saved', title: data.alignment === 'unpaired_smoke_test' ?
      '离线样本 · 未配对输入' : '离线采集结果', detail: '已保存的模型输出，不代表当前传感器状态'};
    if (error) return {data, mode: 'stale', title: '显示数据不可用', detail: error};
    if (!connected) return {data, mode: 'waiting', title: '等待 MITI 数据源', detail: '也可点击“离线结果”查看已验证样本'};
    const heartbeat = status && now - status.receivedAt < 4000 ? status.snapshot : null;
    const elapsed = data ? Math.max(0, (now - sample.receivedAt) / 1000) : Infinity;
    const fresh = data && elapsed < Math.max(12, data.inference_s * 3 + 3);
    if (heartbeat?.state === 'inference_error') return {data, mode: 'stale', title: '特征推理异常', detail: heartbeat.last_error || '检查特征节点日志', heartbeat};
    if (heartbeat?.state === 'inferencing') return {data, mode: 'working', title: 'CPU 特征提取中', detail: '显示上一份结果，等待新观测完成', heartbeat};
    const allInputs = heartbeat && order.every(name => finite(heartbeat.input_ages_s[name]) &&
      heartbeat.input_ages_s[name] <= heartbeat.max_input_age_s);
    if (fresh && allInputs) return {data, mode: data.complete ? 'live' : 'working', title: data.complete ?
      '四模态特征已更新' : '部分模态特征已更新', detail: data.alignment === 'estimated_clock_offsets' ?
        '使用时钟偏移估计匹配；未验证硬件同步精度' : '按采集时间匹配；硬件同步与几何标定仍需实测', heartbeat};
    return {data, mode: data ? 'stale' : 'waiting', title: data ? '保留最近特征 · 等待输入' : '等待四模态输入',
      detail: heartbeat ? '检查缺失、过期或采集时间不匹配的输入' : '启动四模态特征节点后自动更新', heartbeat};
  }
  function subscribe(client, topic, parser, onData, onError, clock = () => performance.now()) {
    let disposed = false;
    const receive = message => {
      if (disposed) return;
      const data = parser(message);
      if (data) onData({snapshot: data, receivedAt: clock()});
      else onError('收到不兼容的四模态消息');
    };
    client.subscribe(topic, receive);
    return () => { disposed = true; client.unsubscribe(topic, receive); };
  }
  function pathFor(values, maximum) {
    return values.map((v, i) => (i ? 'L' : 'M') + i + ',' + (25 - 23 * v / maximum).toFixed(2)).join(' ');
  }
  const shortNames = ['RGB', 'NIR', 'LiDAR', 'Audio'];
  const fmt = (value, digits = 4) => finite(value) ? value.toFixed(digits) : '—';
  function inputInfo(row, i) {
    const p = row?.preprocessing || {};
    if (!row?.valid) return '本次缺失';
    if (i < 2) return (Array.isArray(p.input_shape) ? p.input_shape.join(' × ') : '尺寸未记录') + (i === 0 ? ' · RGB8' : ' · mono8');
    if (i === 2) return (p.input_count ?? '—') + ' 点 · XYZ · ' + (p.input_units || '单位未记录');
    return (p.sample_rate_in ?? '—') + ' Hz · ' + (p.channels_in ?? '—') + ' 声道 · ' + fmt(row.duration, 2) + ' s PCM';
  }
  function preprocessingInfo(row, i) {
    const p = row?.preprocessing || {};
    if (i === 0) return ['缩放 / 裁剪 / 像素标准化 → 3 × 224 × 224', 'ImageBind vision · 冻结'];
    if (i === 1) return p.encoder === 'thermal' ?
      ['单通道 → 1 × 224 × 224', 'ImageBind thermal · 冻结'] :
      ['灰度复制 3 通道 / 像素标准化 → 3 × 224 × 224', '共享 ImageBind vision · NIR 实验适配'];
    if (i === 2) return ['过滤 / 去重 / 中心化 / 单位球 / 采样 → ' + (p.output_count ?? '—') + ' × 3',
      'PointBind I2P-MAE + 预训练投影 · 冻结'];
    return ['单声道 / 16 kHz / 至多 2 s → 128 × 204 log-Mel', 'ImageBind audio · 冻结'];
  }
  function inspectVector(data, selected = 'fused') {
    if (!data) return {values: null, label: '等待特征', color: colors[4]};
    const match = /^(raw|norm)([0-3])$/.exec(selected);
    if (match) {
      const i = Number(match[2]), raw = match[1] === 'raw', row = data.modalities[i];
      return {values: raw ? (data.flow_version === 1 ? row.encoder_features : null) : row.features,
        label: shortNames[i] + (raw ? ' · 编码器输出 h' : ' · 归一化特征 z'),
        color: colors[i], valid: row.valid};
    }
    return selected === 'joint' ? {values: data.flow_version === 1 ? data.joint_features : null,
      label: '拼接输出 J · 4096 维', color: colors[4], valid: data.complete} :
      selected === 'mean' ? {values: data.flow_version === 1 ? data.weighted_mean_features : null,
      label: '加权中间向量 m · 1024 维', color: colors[4], valid: data.fused_valid} :
      {values: data.fused_features, label: '融合输出 f · 1024 维', color: colors[4], valid: data.fused_valid};
  }
  const css = `
    .rtab-four{height:100%;overflow:auto;min-width:0;padding:14px 16px;color:#dfebf4;background:#111c28;font:12px/1.5 system-ui,sans-serif;container-type:inline-size}
    .rtab-four *{box-sizing:border-box}.rtab-four header,.rtab-four-controls{display:flex;gap:8px;justify-content:space-between;align-items:center;flex-wrap:wrap}
    .rtab-four h2{margin:0;font-size:17px;font-weight:600}.rtab-four h3{margin:0 0 8px;font-size:13px}.rtab-four p{margin:5px 0 10px;color:#98aec1}
    .rtab-four button{border:1px solid #3b5267;border-radius:5px;background:#213246;color:#dcebf5;padding:5px 9px;cursor:pointer;font:inherit}
    .rtab-four button[aria-pressed=true]{border-color:#60d8bd;background:#203c43}.rtab-four button:disabled{opacity:.5;cursor:default}
    .rtab-four-live{color:#60d8bd}.rtab-four-working{color:#6bb8fa}.rtab-four-saved,.rtab-four-waiting,.rtab-four-held{color:#f3bc6b}.rtab-four-stale{color:#ed928d}
    .rtab-four-flow{overflow-x:auto;margin:10px 0}.rtab-four-flow-head,.rtab-four-lane{display:grid;grid-template-columns:1.1fr 1.5fr 1fr 1fr;gap:8px;min-width:660px}
    .rtab-four-flow-head{color:#97afc3;font-size:11px;padding:4px 0}.rtab-four-lane{margin:7px 0}.rtab-four-cell{min-width:0;padding:8px 10px;background:#192939;border-radius:6px}
    .rtab-four-cell b{display:block}.rtab-four small{display:block;color:#94aabe;overflow-wrap:anywhere;font-variant-numeric:tabular-nums}
    .rtab-four-topic{font:10px/1.4 ui-monospace,monospace;margin-top:4px}.rtab-four button.rtab-four-cell{text-align:left;width:100%}
    .rtab-four svg{display:block;width:100%;height:34px;background:#132231;border-radius:3px;margin:4px 0}.rtab-four-output-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
    .rtab-four-output{background:#192939;padding:10px;border:1px solid #30485b;border-radius:6px}.rtab-four-segments{display:flex;gap:3px;margin:8px 0}.rtab-four-segments span{flex:1;text-align:center;font-size:10px;border-top:4px solid;padding-top:3px}
    .rtab-four-meta{display:flex;gap:14px;flex-wrap:wrap;color:#b6cbdc;font-size:11px;margin-bottom:6px}
    .rtab-four-inspector{margin:12px 0;padding:12px;background:#162536;border-radius:6px}.rtab-four-inspector svg{height:86px}
    .rtab-four-overview{padding:8px 10px;border:1px solid #30485b;border-radius:6px;background:#162536;color:#c6d9e7}
    .rtab-four-contribution{display:flex;align-items:center;gap:8px}.rtab-four-contribution svg{flex:1;min-width:90px;height:16px;margin:0}.rtab-four-contribution span{min-width:72px}
    .rtab-four-inspector input[type=range]{flex:1;min-width:120px;accent-color:#60d8bd}.rtab-four input[type=number]{width:75px}
    .rtab-four select,.rtab-four input[type=number]{background:#203448;color:#dcebf5;border:1px solid #496074;border-radius:3px;font:inherit;padding:4px}
    .rtab-four table{width:100%;border-collapse:separate;border-spacing:3px;text-align:right;font-variant-numeric:tabular-nums;font-size:11px}
    .rtab-four th{font-weight:400;color:#a2b8ca}.rtab-four td{border-radius:4px;padding:4px 6px;background:#1b2c3d}
    .rtab-four-note{border-top:1px solid #2a3d50;margin-top:12px;padding-top:9px;font-size:11px;color:#98aec1}
    .rtab-four details{margin-top:10px}.rtab-four summary{cursor:pointer;color:#b4c8d8}.rtab-four img{width:100%;max-height:130px;object-fit:contain;background:#101a28}
    @container(max-width:600px){.rtab-four-output-grid{grid-template-columns:1fr}.rtab-four{padding:10px}.rtab-four-flow-head,.rtab-four-lane{min-width:640px}}
  `;
  function Spark({values, color, label, index, sharedMaximum, emptyLabel = '等待此阶段特征'}) {
    if (!values) return h('small', null, emptyLabel);
    const maximum = sharedMaximum || Math.max(0.001, ...values.map(Math.abs));
    const end = values.length - 1;
    return h('svg', {viewBox: '0 0 ' + end + ' 50', preserveAspectRatio: 'none', role: 'img', 'aria-label': label},
      h('line', {x1: 0, y1: 25, x2: end, y2: 25, stroke: '#344c62', strokeWidth: 0.5}),
      h('path', {d: pathFor(values, maximum), stroke: color, strokeWidth: 0.8, fill: 'none'}),
      index !== undefined ? h('line', {x1: index, x2: index, y1: 0, y2: 50, stroke: '#ffffff', strokeWidth: 1}) : null);
  }
  function View({view, onLive, onDemo, onHold, loading = false, selected = 'fused', index = 0, onSelected, onIndex}) {
    const data = view.data, flow = data?.flow_version === 1;
    const inspected = inspectVector(data, selected);
    const limit = inspected.values?.length || (selected === 'joint' ? 4096 : 1024);
    const cursor = Math.max(0, Math.min(limit - 1, Math.trunc(index) || 0)), j = cursor % 1024;
    const choose = value => onSelected?.(value);
    const normMaximum = data ? Math.max(0.001, ...data.modalities.flatMap(row => row.features.map(Math.abs))) : 1;
    const contributionMaximum = data ? Math.max(1e-12, ...data.modalities.map(row => Math.abs(row.weight * row.features[j]))) : 1;
    const option = (value, label) => h('option', {key: value, value}, label);
    return h('section', {className: 'rtab-four', 'aria-label': '四模态统一特征'},
      h('style', null, css),
      h('header', null, h('h2', {className: 'rtab-four-' + view.mode, role: 'status'}, view.title),
        h('div', {className: 'rtab-four-controls'},
          h('button', {onClick: onLive, 'aria-pressed': !['saved', 'held'].includes(view.mode)}, '实时输入'),
          h('button', {onClick: onHold, disabled: !data || view.mode === 'saved', 'aria-pressed': view.mode === 'held'}, view.mode === 'held' ? '继续显示' : '冻结当前帧'),
          h('button', {onClick: onDemo, disabled: loading, 'aria-pressed': view.mode === 'saved'}, loading ? '读取中…' : '离线结果'))),
      h('p', null, view.detail),
      h('div', {className: 'rtab-four-meta'},
        h('span', null, '四路全局特征 · 冻结预训练模型 · 无需训练'),
        data ? h('span', null, '第 ' + data.sequence + ' 帧 · ' + data.inference_s.toFixed(2) + ' s / 次 · ' + data.device) : null),
      h('div', {className: 'rtab-four-overview'},
        '四路输入 → 各自编码为 1024 维 → L2 归一化 → 拼接 J（4096 维） / 加权归一化 f（1024 维）',
        h('small', null, '点击 h、z 或输出按钮，检查同一帧的特征。默认四路各占 25%；实际权重以当前结果为准。')),
      h('div', {className: 'rtab-four-flow'},
        h('div', {className: 'rtab-four-flow-head'}, ...['① 传感器输入 →', '② 预处理与编码器 →', '③ 编码器输出 h →', '④ L2 归一化特征 z'].map(text => h('span', {key: text}, text))),
        ...order.map((name, i) => {
          const row = data?.modalities[i], age = view.heartbeat?.input_ages_s[name];
          const inputState = view.mode === 'saved' ? '离线样本' : view.mode === 'held' ? '冻结样本' : view.heartbeat ?
            age === null ? '未收到输入' : age > view.heartbeat.max_input_age_s ? '输入已过期' : '输入已收到' : '输入状态待确认';
          const pre = preprocessingInfo(row, i);
          return h('div', {className: 'rtab-four-lane', key: name},
            h('div', {className: 'rtab-four-cell', style: {borderLeft: '3px solid ' + colors[i]}},
              h('b', {style: {color: colors[i]}}, i === 1 && data?.nir_experimental === false ? '红外 IR' : names[i]),
              h('small', null, row ? inputInfo(row, i) : '等待输入'),
              h('small', null, inputState + ' · ' + (row?.valid ? '参与本次编码' : '本次缺失')),
              h('small', null, '时间差 ' + (view.mode === 'saved' && data.alignment === 'unpaired_smoke_test' ? '不适用' : finite(row?.gap_s) ? (row.gap_s * 1000).toFixed(0) + ' ms' : '—')),
              h('small', {className: 'rtab-four-topic'}, row?.topic || row?.source_id || '来源待记录')),
            h('div', {className: 'rtab-four-cell'}, h('b', null, pre[1]), h('small', null, pre[0])),
            h('button', {className: 'rtab-four-cell', onClick: () => choose('raw' + i), 'aria-pressed': selected === 'raw' + i},
              h('b', null, 'h · 1024 维'),
              h(Spark, {values: flow ? row.encoder_features : null, color: colors[i], label: shortNames[i] + ' 编码器输出',
                emptyLabel: data ? '旧记录未保存此阶段' : '等待此阶段特征'}),
              h('small', null, '范数 ' + fmt(row?.encoder_norm))),
            h('button', {className: 'rtab-four-cell', onClick: () => choose('norm' + i), 'aria-pressed': selected === 'norm' + i},
              h('b', null, 'z = h / ‖h‖₂'),
              h(Spark, {values: row?.features, color: colors[i], label: shortNames[i] + ' 归一化特征', sharedMaximum: normMaximum}),
              h('small', null, '1024 维 · 范数 ' + fmt(row?.normalized_norm))));
        })),
      data ? h(React.Fragment, null,
        h('h3', null, '⑤ 两种融合输出 · 使用同一组归一化特征 z'),
        h('div', {className: 'rtab-four-output-grid'},
          h('div', {className: 'rtab-four-output'},
            h('b', null, '拼接 J = [zRGB, zNIR, zLiDAR, zAudio]'),
            h('div', {className: 'rtab-four-segments'}, ...shortNames.map((name, i) =>
              h('span', {key: name, style: {color: colors[i], borderColor: colors[i]}}, name + ' · ' + (i * 1024) + '–' + (i * 1024 + 1023)))),
            h('small', null, '4096 维 · 保留四路独立槽位；缺失槽位填 0'),
            h('button', {onClick: () => choose('joint'), 'aria-pressed': selected === 'joint'}, '查看拼接输出')),
          h('div', {className: 'rtab-four-output'},
            h('b', null, '加权 m = Σ wᵢzᵢ → f = m / ‖m‖₂'),
            h('p', null, data.modalities.map((row, i) => shortNames[i] + ' ' + fmt(row.weight * 100, 1) + '%').join(' · ')),
            h('small', null, '1024 维 → 1024 维 · ‖m‖₂ ' + fmt(data.weighted_mean_norm) + ' → ‖f‖₂ ' + fmt(data.fused_norm)),
            h('button', {onClick: () => choose('mean'), 'aria-pressed': selected === 'mean'}, '查看加权中间值'), ' ',
            h('button', {onClick: () => choose('fused'), 'aria-pressed': selected === 'fused'}, '查看融合输出'),
            h('small', null, data.fused_valid ? '加权向量有效' : '加权向量无效：零向量或无有效模态'))),
        h('div', {className: 'rtab-four-inspector'},
          h('h3', null, '逐维检查 · ' + inspected.label),
          h('div', {className: 'rtab-four-controls'},
            h('select', {value: selected, onChange: e => choose(e.target.value), 'aria-label': '选择特征阶段'},
              ...shortNames.flatMap((name, i) => [option('raw' + i, name + ' 编码器 h'), option('norm' + i, name + ' 归一化 z')]),
              option('mean', '加权中间向量 m'), option('joint', '拼接输出 J'), option('fused', '融合输出 f')),
            h('input', {type: 'range', min: 0, max: limit - 1, value: cursor, onChange: e => onIndex?.(Number(e.target.value)), 'aria-label': '特征维度索引'}),
            h('input', {type: 'number', min: 0, max: limit - 1, value: cursor, onChange: e => onIndex?.(Math.max(0, Math.min(limit - 1, Number(e.target.value)))), 'aria-label': '输入维度索引'}),
            h('b', null, '维度 ' + cursor + ' = ' + fmt(inspected.values?.[cursor], 6))),
          h(Spark, {values: inspected.values, color: inspected.color, label: inspected.label, index: cursor}),
          h('small', null, '横轴：全部维度 0–' + (limit - 1) + '；纵轴：当前向量幅值 ±' +
            fmt(inspected.values ? Math.max(0.001, ...inspected.values.map(Math.abs)) : null) +
            (inspected.valid === false ? ' · 本向量含缺失槽位或当前无效' : '')),
          h('h3', {style: {marginTop: 12}}, '同一维度 j = ' + j + ' 的融合计算'),
          h('table', {'aria-label': '每路特征与加权贡献'},
            h('thead', null, h('tr', null, ...['模态', 'h[j]', 'z[j]', '权重 w', 'w × z[j]'].map(text => h('th', {key: text}, text)))),
            h('tbody', null, ...data.modalities.map((row, i) => h('tr', {key: row.name},
              h('th', {style: {color: colors[i]}}, shortNames[i]),
              h('td', null, fmt(flow ? row.encoder_features[j] : null, 6)),
              h('td', null, fmt(row.features[j], 6)),
              h('td', null, fmt(row.weight, 3)),
              h('td', null, h('div', {className: 'rtab-four-contribution'},
                h('svg', {viewBox: '0 0 100 16', role: 'img', 'aria-label': shortNames[i] + ' 加权贡献 ' + fmt(row.weight * row.features[j], 6)},
                  h('line', {x1: 50, x2: 50, y1: 0, y2: 16, stroke: '#8fa6bb'}),
                  h('rect', {x: row.features[j] < 0 ? 50 - 48 * Math.abs(row.weight * row.features[j]) / contributionMaximum : 50,
                    y: 3, width: 48 * Math.abs(row.weight * row.features[j]) / contributionMaximum,
                    height: 10, rx: 2, fill: colors[i]})),
                h('span', null, fmt(row.weight * row.features[j], 6)))))))),
          h('small', null, '贡献条使用同一尺度；中心为 0，向左为负、向右为正。'),
          h('p', null, '四路贡献求和 m[' + j + '] = ' + fmt(data.weighted_mean_features?.[j], 6) +
            '；除以 ‖m‖₂ = ' + fmt(data.weighted_mean_norm, 6) +
            ' → f[' + j + '] = ' + fmt(data.fused_features[j], 6)),
          !flow ? h('small', null, '旧结果只包含归一化和融合特征；编码器原值、拼接值及加权中间值未记录。') : null,
          h('small', null, '曲线表示全局嵌入向量；单个维度没有固定的物体、位置或距离含义。')),
        h('details', null, h('summary', null, '跨模态余弦相似度 · −1 到 +1'),
          h('table', {'aria-label': '四模态余弦相似度'},
            h('thead', null, h('tr', null, h('th'), ...shortNames.map(name => h('th', {key: name}, name)))),
            h('tbody', null, ...data.cosine_similarities.map((row, i) => h('tr', {key: i},
              h('th', null, shortNames[i]), ...row.map((value, k) => h('td', {key: k}, fmt(value, 3))))))),
          h('p', null, '相似度是向量关系，不是识别准确率或可靠性分数。')),
        view.mode === 'saved' && data.spectrum_url ? h('details', null,
          h('summary', null, '离线音频声谱'), h('img', {src: '/rtabmap-fourmodal-audio.png', alt: '离线样本 log-Mel 声谱'})) : null,
        h('div', {className: 'rtab-four-note'},
          h('div', null, '深度图是辅助预览，不参与这四路融合。此流程不更新地图。'),
          h('div', null, '下方传感器预览持续更新；冻结当前帧只固定特征，不冻结相机和点云画面。'),
          data.nir_experimental ? h('div', null, '近红外复用视觉编码器，NIR 分布适配效果尚未验证。') : null,
          h('div', null, '编码器输出包含预训练投影与缩放；本面板未提取 Transformer 内部 token / 注意力。'),
          h('div', null, '当前未做图像与点云的几何配准或物体关联；加权系数为配置值，非学习的注意力。'),
          data.alignment === 'estimated_clock_offsets' ? h('div', null, '按估计时钟偏移匹配；硬件同步精度尚未验证。') : null,
          data.modalities[3].stamp_quality === 'host_read_end_estimate' ? h('div', null, '音频来自工作站麦克风，时间为主机读取估计。') : null,
          h('div', {title: data.model_sha256}, data.model + ' · 标定记录 ' + data.calibration_version))) : null);
  }
  function Component({client, topic = '/miti/fourmodal_features'}) {
    const [sample, setSample] = React.useState(null), [status, setStatus] = React.useState(null);
    const [demo, setDemo] = React.useState(null), [error, setError] = React.useState('');
    const [loading, setLoading] = React.useState(false), [now, setNow] = React.useState(() => performance.now());
    const [selected, setSelected] = React.useState('fused'), [index, setIndex] = React.useState(0);
    const [held, setHeld] = React.useState(null);
    const demoRequest = React.useRef(0);
    React.useEffect(() => {
      setSample(null); setStatus(null); setError('');
      if (!client) return;
      const disposers = [];
      try {
        disposers.push(subscribe(client, topic, parseSnapshot, value => {
          if (value.snapshot.mode !== 'live') { setError('实时话题收到离线数据'); return; }
          setSample(value); setError('');
        }, setError));
        disposers.push(subscribe(client, '/miti/fourmodal_status', parseStatus, setStatus, setError));
      } catch (_) { setError('无法订阅四模态话题'); }
      return () => disposers.forEach(dispose => dispose());
    }, [client, topic]);
    React.useEffect(() => {
      const timer = setInterval(() => setNow(performance.now()), 500);
      return () => { clearInterval(timer); demoRequest.current++; };
    }, []);
    async function loadDemo() {
      const request = ++demoRequest.current;
      setLoading(true); setError(''); setHeld(null);
      try {
        const response = await fetch('/rtabmap-fourmodal-demo.json', {cache: 'no-store'});
        if (!response.ok) throw new Error('尚无离线结果，请运行四模态 MITI 安装脚本');
        const snapshot = parseSnapshot({data: await response.text()});
        if (!snapshot || snapshot.mode !== 'saved') throw new Error('离线结果格式不兼容');
        if (request === demoRequest.current) setDemo({snapshot, receivedAt: performance.now()});
      } catch (e) { if (request === demoRequest.current) { setDemo(null); setError(e.message); } }
      finally { if (request === demoRequest.current) setLoading(false); }
    }
    const view = held ? {data: held, mode: 'held', title: '冻结第 ' + held.sequence + ' 帧',
      detail: '固定当前特征供逐维检查；传感器和后台推理继续运行'} :
      deriveView(demo || sample, status, now, Boolean(client), error);
    return h(View, {view, loading, selected, index,
      onLive: () => { demoRequest.current++; setLoading(false); setDemo(null); setHeld(null); setError(''); },
      onHold: () => setHeld(held ? null : sample?.snapshot || null),
      onDemo: loadDemo, onSelected: value => { setSelected(value); setIndex(0); },
      onIndex: setIndex});
  }
  return {Component, View, parseSnapshot, parseStatus, deriveView, subscribe, pathFor, inspectVector};
}
