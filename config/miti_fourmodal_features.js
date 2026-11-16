/* Native MITI feature panel and one-time layout addition. Uses MITI's React instance. */
function rtabmapFourmodalWidget(id = 'rtabmap-fourmodal') {
  return {i: id, type: 'fourmodal-features', title: '四模态 · 特征融合',
    x: 0, y: 24, w: 8, h: 10, props: {topic: '/miti/fourmodal_features'}};
}

function rtabmapAudioSpectrumWidget(id = 'rtabmap-audio-spectrum') {
  return {i: id, type: 'camera-viewer', title: '声学 · log-Mel 声谱',
    x: 8, y: 24, w: 4, h: 10, props: {topic: '/miti/audio_spectrum/image_raw', maxFps: 2}};
}

function rtabmapMigrateFourmodalLayout(layout) {
  if (!layout || !Array.isArray(layout.widgets)) return layout;
  const renamed = layout.widgets.map(widget => widget.type === 'fourmodal-features' &&
    ['四模态 · 统一特征', '四模态统一特征', '四模态 · 输入 → 特征 → 融合'].includes(widget.title)
    ? {...widget, title: '四模态 · 特征融合'} : widget);
  if (renamed.some((widget, i) => widget !== layout.widgets[i])) layout = {...layout, widgets: renamed};
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
    const missing = heartbeat ? order.filter(name => !finite(heartbeat.input_ages_s[name]) ||
      heartbeat.input_ages_s[name] > heartbeat.max_input_age_s).map(name =>
        ({rgb: 'RGB', infrared: '近红外', lidar: 'LiDAR', audio: '音频'})[name]) : [];
    return {data, mode: data ? 'stale' : 'waiting', title: data ? '保留最近特征 · 等待输入' : '等待四模态输入',
      detail: heartbeat ? missing.length ? '等待输入：' + missing.join('、') : '等待时间对齐与特征输出' : '未收到特征节点状态', heartbeat};
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
    .rtab-four{height:100%;min-width:0;overflow:auto;padding:16px;color:#e5edf5;background:#0e1824;font:13px/1.5 system-ui,sans-serif;container-type:inline-size;color-scheme:dark}
    .rtab-four *{box-sizing:border-box}.rtab-four button,.rtab-four input,.rtab-four select{font:inherit}
    .rtab-four button{cursor:pointer}.rtab-four button:disabled{cursor:default;opacity:.45}
    .rtab-four button:focus-visible,.rtab-four summary:focus-visible,.rtab-four input:focus-visible,.rtab-four select:focus-visible{outline:2px solid #71d8c2;outline-offset:3px}
    .rtab-four button{color:inherit;border:1px solid #304253;border-radius:7px;background:transparent;padding:6px 11px;transition:background .15s,border-color .15s}
    .rtab-four button:hover:not(:disabled){background:#213244}.rtab-four button[aria-pressed=true]{background:#233f44;border-color:#66cbb8;color:#b3f5df}
    .rtab-four small{color:#a2b4c6;font-size:11px}.rtab-four p{margin:8px 0;color:#a2b4c6}.rtab-four h3{font-size:13px;font-weight:500;margin:0}
    .rtab-four-header,.rtab-four-toolbar,.rtab-four-chart-head,.rtab-four-card-head,.rtab-four-card-foot,.rtab-four-inspect-controls{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}
    .rtab-four-header{margin-bottom:16px}.rtab-four-status-group{display:flex;align-items:center;gap:9px;flex-wrap:wrap}
    .rtab-four-status{display:inline-flex;align-items:center;gap:7px;font-weight:500}.rtab-four-status:before{content:'';width:7px;height:7px;border-radius:50%;background:currentColor}
    .rtab-four-live{color:#77dbc0}.rtab-four-working{color:#84b8e8}.rtab-four-saved,.rtab-four-held,.rtab-four-waiting{color:#dbbd84}.rtab-four-stale{color:#f0a39b}
    .rtab-four-tag{font-size:11px;color:#a2b4c6;border-left:1px solid #344658;padding-left:9px}.rtab-four-toolbar{gap:6px}
    .rtab-four-modes{display:flex;padding:2px;background:#172536;border-radius:8px;gap:2px}.rtab-four-modes button{padding:4px 10px;border-color:transparent}
    .rtab-four-modes button[aria-pressed=true]{background:#2b4054;color:#edf4fa;border-color:transparent}
    .rtab-four-notice{font-size:12px;padding:9px 11px;background:#26303a;border-radius:7px;margin-bottom:13px;color:#d9c39f;overflow-wrap:anywhere}
    .rtab-four-section-label{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:8px;color:#c4d2df;font-size:12px}
    .rtab-four-inputs{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
    .rtab-four button.rtab-four-input{text-align:left;padding:12px;min-width:0;border:1px solid #2b3d50;border-radius:10px;background:#172536}
    .rtab-four button.rtab-four-input:hover{background:#1c3043;border-color:#476073}.rtab-four button.rtab-four-input[aria-pressed=true]{border-color:var(--modality-color);background:#1a3040;color:#e5edf5}
    .rtab-four-card-head{gap:4px;flex-wrap:nowrap}.rtab-four-name{font-size:14px;font-weight:600;display:flex;align-items:center;gap:7px}.rtab-four-name:before{content:'';width:6px;height:6px;background:var(--modality-color);border-radius:50%;flex:none}
    .rtab-four-card-state{color:#9eb1c4;font-size:11px;white-space:nowrap}.rtab-four-card-state[data-state=missing],.rtab-four-card-state[data-state=stale]{color:#e2ab95}
    .rtab-four-input-size{font-size:11px;color:#a2b4c6;margin-top:5px;overflow-wrap:anywhere;min-height:17px}
    .rtab-four svg.rtab-four-spark{display:block;width:100%;height:34px;margin:10px 0;background:transparent;overflow:hidden}
    .rtab-four-card-foot{font-size:11px;color:#a2b4c6;gap:4px}.rtab-four-card-foot b{font-weight:500;color:#d8e5ef;font-variant-numeric:tabular-nums}
    .rtab-four-empty-spark{height:34px;display:flex;align-items:center;margin:10px 0;color:#8198af;font-size:11px}
    .rtab-four-flow-link{display:flex;align-items:center;justify-content:center;gap:12px;margin:12px 0;color:#93aabf;font-size:11px}
    .rtab-four-flow-link:before,.rtab-four-flow-link:after{content:'';height:1px;flex:1;background:#2a3c4d}
    .rtab-four-chart{background:#132231;border:1px solid #2b3d50;border-radius:10px;padding:14px}
    .rtab-four-chart-head{gap:8px}.rtab-four-chart-title{display:flex;align-items:center;gap:8px}.rtab-four-chart-title b{font-weight:500}
    .rtab-four-chart-head select{max-width:100%;background:#182b3e;color:#c4d7e8;border:1px solid #354b5f;border-radius:6px;padding:5px 7px;font-size:12px}
    .rtab-four-output-tabs{display:flex;gap:5px;flex-wrap:wrap}.rtab-four-output-tabs button{font-size:12px;padding:5px 10px}
    .rtab-four-chart svg.rtab-four-spark{height:100px;margin:15px 0 4px}.rtab-four-chart .rtab-four-empty-spark{height:100px;justify-content:center;margin:15px 0 4px;font-size:13px}
    .rtab-four-axis{display:flex;justify-content:space-between;gap:8px;font-size:11px;color:#829aaf;font-variant-numeric:tabular-nums}
    .rtab-four-folds{margin-top:12px}.rtab-four details{border-bottom:1px solid #293a4c}.rtab-four details:last-child{border-bottom:0}
    .rtab-four summary{cursor:pointer;padding:11px 0;font-size:12px;color:#afc3d5}.rtab-four details[open]>summary{color:#e1edf6}.rtab-four summary::marker{color:#6f899f}
    .rtab-four-detail-body{padding:0 0 14px;min-width:0}.rtab-four-inspect-controls{margin:3px 0 12px;font-variant-numeric:tabular-nums}.rtab-four-inspect-controls label{display:flex;gap:7px;align-items:center}
    .rtab-four input[type=range]{accent-color:#77dbc0;flex:1;min-width:95px}.rtab-four input[type=number]{width:74px;background:#182b3e;color:#e1edf6;border:1px solid #40586d;border-radius:5px;padding:4px}
    .rtab-four-table-wrap{overflow-x:auto}.rtab-four table{width:100%;border-collapse:collapse;font-size:11px;font-variant-numeric:tabular-nums;text-align:right}
    .rtab-four th{color:#a2b4c6;font-weight:400}.rtab-four th,.rtab-four td{padding:6px;border-bottom:1px solid #25394b;white-space:nowrap}.rtab-four th:first-child{text-align:left}
    .rtab-four-contribution{display:flex;align-items:center;gap:8px}.rtab-four-contribution svg{width:74px;height:14px;flex:1;min-width:40px}.rtab-four-contribution span{min-width:64px}
    .rtab-four-formula{font:11px/1.6 ui-monospace,monospace;color:#b6cadd;margin-top:9px;overflow-wrap:anywhere}
    .rtab-four-source-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}.rtab-four-source{min-width:0}.rtab-four-source b{font-weight:500}.rtab-four-source small{display:block;overflow-wrap:anywhere;margin-top:3px}
    .rtab-four-note{font-size:11px;border-top:1px solid #2a3d50;padding-top:10px;margin-top:12px;color:#9db1c3}.rtab-four-note div{margin:3px 0}
    .rtab-four-matrix{max-width:430px}.rtab-four-matrix td{border:3px solid #0e1824;border-radius:6px;padding:9px}.rtab-four-spectrum{width:100%;max-height:130px;object-fit:contain;background:#101a28;margin-top:10px}
    .rtab-four-sr{position:absolute;width:1px;height:1px;padding:0;margin:-1px;overflow:hidden;clip:rect(0,0,0,0);white-space:nowrap;border:0}
    @container(max-width:600px){.rtab-four-inputs{grid-template-columns:repeat(2,minmax(0,1fr))}.rtab-four-source-grid{grid-template-columns:1fr}.rtab-four-tag{display:none}.rtab-four-chart-head{align-items:flex-start}.rtab-four-chart-title{width:100%;justify-content:space-between}}
    @container(max-width:330px){.rtab-four-header{align-items:flex-start}.rtab-four-toolbar{width:100%;justify-content:space-between}.rtab-four-inputs{gap:7px}.rtab-four button.rtab-four-input{padding:9px}.rtab-four-name{font-size:13px}.rtab-four-card-head{flex-wrap:wrap}.rtab-four-chart{padding:10px}.rtab-four-inspect-controls{gap:7px}.rtab-four-contribution svg{display:none}}
    @media(prefers-reduced-motion:reduce){.rtab-four button{transition:none}}
  `;
  function Spark({values, color, label, index, sharedMaximum, emptyLabel = '等待输入'}) {
    if (!values) return h('div', {className: 'rtab-four-empty-spark'}, emptyLabel);
    const maximum = sharedMaximum || Math.max(0.001, ...values.map(Math.abs));
    const end = values.length - 1;
    return h('svg', {className: 'rtab-four-spark', viewBox: '0 0 ' + end + ' 50', preserveAspectRatio: 'none', role: 'img', 'aria-label': label},
      h('line', {x1: 0, y1: 25, x2: end, y2: 25, stroke: '#30495e', vectorEffect: 'non-scaling-stroke', strokeWidth: 1}),
      h('path', {d: pathFor(values, maximum), stroke: color, strokeWidth: 1, vectorEffect: 'non-scaling-stroke', fill: 'none'}),
      index !== undefined ? h('line', {x1: index, x2: index, y1: 0, y2: 50, stroke: '#d9eaf6', vectorEffect: 'non-scaling-stroke', strokeWidth: 1}) : null);
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
    const saved = view.mode === 'saved', held = view.mode === 'held';
    const status = {live: '实时更新', working: '提取中', saved: '离线结果', held: '已冻结', stale: '历史结果', waiting: '等待输入'}[view.mode] || view.title;
    const unpaired = saved && data?.alignment === 'unpaired_smoke_test';
    const notice = view.mode === 'stale' ? view.title + ' · ' + view.detail :
      unpaired ? '未配对样本 · 仅供流程查看' : saved ? '已保存结果 · 非实时输入' :
      !data ? view.detail : null;
    const sourceState = (row, name) => {
      if (saved || held) return row?.valid ? {label: saved ? '离线' : '冻结', state: 'saved'} : {label: '缺失', state: 'missing'};
      const age = view.heartbeat?.input_ages_s[name];
      if (age === null) return {label: '未收到', state: 'missing'};
      if (!finite(age)) return {label: data ? '待确认' : '等待', state: 'waiting'};
      return age > view.heartbeat.max_input_age_s ? {label: '过期', state: 'stale'} : {label: '在线', state: 'live'};
    };
    const compactInput = (row, i) => {
      if (!row?.valid) return data ? '本次缺失' : '—';
      const p = row.preprocessing || {};
      if (i < 2) return Array.isArray(p.input_shape) ? p.input_shape.slice(0, 2).reverse().join(' × ') : '尺寸未记录';
      if (i === 2) return finite(p.input_count) ? p.input_count.toLocaleString('en-US') + ' 点' : 'XYZ 点云';
      return finite(p.sample_rate_in) ? fmt(p.sample_rate_in / 1000, 0) + ' kHz · ' + fmt(row.duration, 1) + ' s' : 'PCM 音频';
    };
    return h('section', {className: 'rtab-four', 'aria-label': '四模态特征融合'},
      h('style', null, css),
      h('header', {className: 'rtab-four-header'},
        h('div', {className: 'rtab-four-status-group'},
          h('span', {className: 'rtab-four-status rtab-four-' + view.mode, role: 'status', 'aria-label': view.title}, status),
          data ? h('span', {className: 'rtab-four-tag'}, '#' + data.sequence + ' · ' + fmt(data.inference_s, 1) + ' s') : null,
          data?.alignment === 'estimated_clock_offsets' ? h('span', {className: 'rtab-four-tag'}, '估计时钟') : null),
        h('div', {className: 'rtab-four-toolbar'},
          h('div', {className: 'rtab-four-modes', role: 'group', 'aria-label': '数据来源'},
            h('button', {onClick: onLive, 'aria-pressed': !saved, 'aria-label': '实时输入'}, '实时'),
            h('button', {onClick: onDemo, disabled: loading, 'aria-pressed': saved, 'aria-label': '读取离线结果'}, loading ? '读取中' : '离线')),
          h('button', {onClick: onHold, disabled: !data || saved, 'aria-pressed': held, 'aria-label': held ? '继续显示' : '冻结当前帧'}, held ? '继续' : '冻结'))),
      notice ? h('div', {className: 'rtab-four-notice'}, notice) : null,
      h('div', {className: 'rtab-four-section-label'}, h('span', null, '输入特征'),
        h('small', null, data ? data.modalities.filter(row => row.valid).length + ' / 4 参与' : '4 路输入')),
      h('div', {className: 'rtab-four-inputs'}, ...order.map((name, i) => {
        const row = data?.modalities[i], state = sourceState(row, name);
        return h('button', {key: name, className: 'rtab-four-input', style: {'--modality-color': colors[i]},
          onClick: () => choose('norm' + i), 'aria-pressed': selected === 'norm' + i || selected === 'raw' + i,
          'aria-label': names[i] + ' · 查看归一化特征'},
          h('div', {className: 'rtab-four-card-head'},
            h('span', {className: 'rtab-four-name'}, i === 1 && row?.preprocessing?.encoder === 'thermal' ? 'IR' : shortNames[i]),
            h('span', {className: 'rtab-four-card-state', 'data-state': state.state}, state.label)),
          h('div', {className: 'rtab-four-input-size'}, compactInput(row, i)),
          h(Spark, {values: row?.valid ? row.features : null, color: colors[i], label: shortNames[i] + ' 归一化特征', sharedMaximum: normMaximum,
            emptyLabel: data && !row.valid ? '本次缺失' : '等待输入'}),
          h('div', {className: 'rtab-four-card-foot'}, h('span', null, '1024 维'), h('b', null, row ? fmt(row.weight * 100, 0) + '%' : '—')));
      })),
      h('div', {className: 'rtab-four-flow-link'}, '编码 → 归一化 → 融合'),
      h('div', {className: 'rtab-four-chart'},
        h('div', {className: 'rtab-four-chart-head'},
          h('div', {className: 'rtab-four-chart-title'}, h('b', null, '特征视图'),
            h('select', {value: selected, onChange: e => choose(e.target.value), 'aria-label': '选择特征阶段'},
              option('fused', '融合 f'), option('joint', '拼接 J'), option('mean', '加权中间值 m'),
              ...shortNames.flatMap((name, i) => [option('raw' + i, name + ' 编码器 h'), option('norm' + i, name + ' 归一化 z')]))),
          h('div', {className: 'rtab-four-output-tabs', role: 'group', 'aria-label': '融合输出'},
            h('button', {onClick: () => choose('fused'), 'aria-pressed': selected === 'fused'}, '融合 · 1024'),
            h('button', {onClick: () => choose('joint'), 'aria-pressed': selected === 'joint'}, '拼接 · 4096'))),
        h(Spark, {values: inspected.values, color: inspected.color, label: inspected.label, index: cursor,
          emptyLabel: data ? '此阶段未记录' : '等待特征数据'}),
        h('div', {className: 'rtab-four-axis'}, h('span', null, '0'),
          h('span', null, '维度 · 幅值 ±' + fmt(inspected.values ? Math.max(0.001, ...inspected.values.map(Math.abs)) : null, 3)),
          h('span', null, limit - 1)),
        inspected.valid === false ? h('small', null, selected === 'joint' ? '含缺失模态的零槽位' : '本次向量无效') : null),
      data ? h('div', {className: 'rtab-four-folds'},
        h('details', null, h('summary', null, '逐维检查'),
          h('div', {className: 'rtab-four-detail-body'},
            h('div', {className: 'rtab-four-inspect-controls'},
              h('label', null, '维度', h('input', {type: 'number', min: 0, max: limit - 1, value: cursor,
                onChange: e => onIndex?.(Math.max(0, Math.min(limit - 1, Number(e.target.value)))), 'aria-label': '输入维度索引'})),
              h('input', {type: 'range', min: 0, max: limit - 1, value: cursor, onChange: e => onIndex?.(Number(e.target.value)), 'aria-label': '特征维度索引'}),
              h('b', null, fmt(inspected.values?.[cursor], 6)),
              h('span', {className: 'rtab-four-sr'}, '维度 ' + cursor + ' = ' + fmt(inspected.values?.[cursor], 6))),
            h('h3', null, '同一维度 j = ' + j + ' 的贡献'),
            h('div', {className: 'rtab-four-table-wrap'}, h('table', {'aria-label': '每路特征与加权贡献'},
              h('thead', null, h('tr', null, ...['模态', 'h[j]', 'z[j]', '权重', 'w × z[j]'].map(text => h('th', {key: text}, text)))),
              h('tbody', null, ...data.modalities.map((row, i) => h('tr', {key: row.name},
                h('th', null, shortNames[i]), h('td', null, fmt(flow ? row.encoder_features[j] : null, 6)),
                h('td', null, fmt(row.features[j], 6)), h('td', null, fmt(row.weight, 3)),
                h('td', null, h('div', {className: 'rtab-four-contribution'},
                  h('svg', {viewBox: '0 0 100 14', role: 'img', 'aria-label': shortNames[i] + ' 加权贡献 ' + fmt(row.weight * row.features[j], 6)},
                    h('line', {x1: 50, x2: 50, y1: 0, y2: 14, stroke: '#718a9d'}),
                    h('rect', {x: row.features[j] < 0 ? 50 - 48 * Math.abs(row.weight * row.features[j]) / contributionMaximum : 50,
                      y: 3, width: 48 * Math.abs(row.weight * row.features[j]) / contributionMaximum, height: 8, rx: 2, fill: colors[i]})),
                  h('span', null, fmt(row.weight * row.features[j], 6))))))))),
            h('div', {className: 'rtab-four-formula'}, '四路贡献求和 m[' + j + '] = ' + fmt(data.weighted_mean_features?.[j], 6) +
              '；‖m‖₂ = ' + fmt(data.weighted_mean_norm, 6) + ' → f[' + j + '] = ' + fmt(data.fused_features[j], 6)),
            h('small', null, '贡献条同尺度，左负右正。单维数值没有固定的物体或距离含义。'),
            !flow ? h('p', null, '旧结果只包含归一化和融合特征；其余阶段未记录。') : null)),
        h('details', null, h('summary', null, '输入与模型'),
          h('div', {className: 'rtab-four-detail-body'},
            h('div', {className: 'rtab-four-source-grid'}, ...data.modalities.map((row, i) => {
              const pre = preprocessingInfo(row, i);
              return h('div', {className: 'rtab-four-source', key: row.name},
                h('b', null, names[i]), h('small', null, '传感器输入 · ' + inputInfo(row, i)),
                h('small', null, '预处理与编码器 · ' + pre.join(' · ')),
                h('small', null, '时间差 ' + (unpaired ? '不适用' : finite(row.gap_s) ? fmt(row.gap_s * 1000, 2) + ' ms' : '—')),
                h('small', null, row.topic || row.source_id || '来源未记录'),
                h('small', null, '编码器输出 ‖h‖₂ ' + fmt(row.encoder_norm) + ' → L2 归一化特征 ‖z‖₂ ' + fmt(row.normalized_norm)));
            })),
            h('div', {className: 'rtab-four-formula'}, 'zᵢ = hᵢ / ‖hᵢ‖₂ · 拼接输出 J = [zRGB, zNIR, zLiDAR, zAudio]'),
            h('div', {className: 'rtab-four-formula'}, 'm = Σ wᵢzᵢ · f = m / ‖m‖₂ · ‖f‖₂ = ' + fmt(data.fused_norm)),
            h('div', {className: 'rtab-four-note'},
              h('div', null, '全局特征 · 冻结预训练模型 · ' + data.device),
              h('div', null, '深度图是辅助预览，不参与这四路融合。此流程不更新地图。'),
              h('div', null, '冻结仅固定特征，下方传感器预览持续更新。'),
              h('div', null, '未做图像与点云的几何配准；权重为配置值。'),
              data.nir_experimental ? h('div', null, '近红外复用视觉编码器，适配效果待验证。') : null,
              data.alignment === 'estimated_clock_offsets' ? h('div', null, '按估计时钟匹配，硬件同步精度未验证。') : null,
              data.modalities[3].stamp_quality === 'host_read_end_estimate' ? h('div', null, '音频采集时间为主机读取估计。') : null,
              h('div', null, data.model + ' · ' + data.calibration_version)))),
        h('details', null, h('summary', null, '模态相似度'),
          h('div', {className: 'rtab-four-detail-body'},
            h('table', {className: 'rtab-four-matrix', 'aria-label': '四模态余弦相似度'},
              h('thead', null, h('tr', null, h('th'), ...shortNames.map(name => h('th', {key: name}, name)))),
              h('tbody', null, ...data.cosine_similarities.map((row, i) => h('tr', {key: i},
                h('th', null, shortNames[i]), ...row.map((value, k) => h('td', {key: k, style: finite(value) ?
                  {background: 'rgba(' + (value < 0 ? '196,137,113,' : '76,167,154,') + (0.08 + 0.3 * Math.abs(value)) + ')'} : {}}, fmt(value, 3))))))),
            h('p', null, '余弦相似度 −1 到 +1；不是识别准确率。'))),
        saved && data.spectrum_url ? h('details', null, h('summary', null, '离线音频声谱'),
          h('img', {className: 'rtab-four-spectrum', src: '/rtabmap-fourmodal-audio.png', alt: '离线样本 log-Mel 声谱'})) : null) : null);
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
