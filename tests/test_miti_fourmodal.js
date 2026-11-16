const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {createRequire} = require('node:module');
const root = path.resolve(__dirname, '..');
const vendorRequire = createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const React = vendorRequire('react');
const {renderToStaticMarkup} = vendorRequire('react-dom/server.browser');
const source = fs.readFileSync(path.join(root, 'config/miti_fourmodal_features.js'), 'utf8');
const context = vm.createContext({});
vm.runInContext(source, context);
const panel = context.rtabmapFourmodalFactory(React);
const copy = value => JSON.parse(JSON.stringify(value));
function snapshot(overrides = {}) {
  const modalities = ['rgb', 'infrared', 'lidar', 'audio'].map((name, i) => ({name,
    valid: true, weight: 0.25, gap_s: 0, stamp: 10, duration: i === 3 ? 2 : 0,
    features: Array.from({length: 1024}, (_, j) => i === j ? 1 : 0)}));
  return {schema: 1, mode: 'live', sequence: 1, published_at: 20, feature_dim: 1024,
    joint_dim: 4096, complete: true, fused_valid: true, inference_s: 4.2,
    modalities, fused_features: Array.from({length: 1024}, (_, j) => j < 4 ? 0.5 : 0),
    cosine_similarities: modalities.map((_, i) => modalities.map((_, j) => i === j ? 1 : 0)),
    alignment: 'acquisition_timestamps', nir_experimental: true, device: 'cpu',
    model: 'frozen-test', model_sha256: 'test', calibration_version: 'unverified', ...overrides};
}
const parse = data => panel.parseSnapshot({data: JSON.stringify(data)});
function expandElements(element) {
  if (!element || typeof element !== 'object') return element;
  if (Array.isArray(element)) return element.map(expandElements);
  if (typeof element.type === 'function') return expandElements(element.type(element.props));
  return {...element, props: {...element.props, children: React.Children.toArray(element.props.children).map(expandElements)}};
}
function findElements(element, predicate) {
  if (!element || typeof element !== 'object') return [];
  if (Array.isArray(element)) return element.flatMap(e => findElements(e, predicate));
  return [...(predicate(element) ? [element] : []), ...findElements(element.props?.children, predicate)];
}
function flowSnapshot() {
  const v = snapshot(); v.flow_version = 1;
  for (let i = 0; i < 4; i++) {
    const row = v.modalities[i];
    row.encoder_features = row.features.map(x => x * (i + 2));
    row.encoder_norm = i + 2; row.normalized_norm = 1;
    row.topic = '/input/' + row.name;
    row.preprocessing = i < 2 ? {input_shape: i ? [480, 640] : [480, 640, 3]} :
      i === 2 ? {input_count: 10000, output_count: 2048, input_units: 'm'} :
      {sample_rate_in: 48000, channels_in: 2};
  }
  v.joint_features = v.modalities.flatMap(row => row.features);
  v.weighted_mean_features = v.fused_features.map(x => x / 2);
  v.weighted_mean_norm = 0.5; v.fused_norm = 1;
  return v;
}
test('flow protocol validates raw, intermediate and joint vectors; old snapshots remain honest', () => {
  assert.ok(parse(flowSnapshot()));
  for (const mutate of [v => v.joint_features.pop(), v => v.weighted_mean_features[0] = NaN,
    v => delete v.modalities[2].encoder_features, v => v.modalities[0].encoder_norm = -1]) {
    const v = flowSnapshot(); mutate(v); assert.equal(parse(v),null);
  }
  assert.equal(panel.inspectVector(snapshot(),'raw0').values,null);
  const html = renderToStaticMarkup(React.createElement(panel.View, {
    view: {data:snapshot(),mode:'saved',title:'旧记录',detail:''},selected:'raw0'}));
  assert.match(html,/旧结果只包含归一化和融合特征/);
  assert.match(html,/维度 0 = —/);
});
test('each stage inspects the actual corresponding array and the full 4096-dimensional joint', () => {
  const v = flowSnapshot();
  for (let i=0;i<4;i++) {
    assert.equal(panel.inspectVector(v,'raw'+i).values,v.modalities[i].encoder_features);
    assert.equal(panel.inspectVector(v,'norm'+i).values,v.modalities[i].features);
  }
  assert.equal(panel.inspectVector(v,'mean').values,v.weighted_mean_features);
  assert.equal(panel.inspectVector(v,'joint').values,v.joint_features);
  assert.equal(panel.inspectVector(v,'fused').values,v.fused_features);
  const html = renderToStaticMarkup(React.createElement(panel.View, {
    view:{data:v,mode:'live',title:'测试',detail:''},selected:'joint',index:1025}));
  for (const label of ['传感器输入','预处理与编码器','编码器输出','L2 归一化特征','同一维度 j = 1',
    '640','10000 点','2048','48000 Hz','/input/rgb','维度 1025 = 1.000000','四路贡献求和',
    '深度图是辅助预览','max="4095"']) assert.ok(html.includes(label),label);
});
test('compact panel collapses technical details and retains stage, source and freeze controls', () => {
  const selected = [], actions = [];
  const tree = expandElements(panel.View({view: {data: flowSnapshot(), mode:'live', title:'测试', detail:''},
    onSelected: value => selected.push(value), onLive: () => actions.push('live'),
    onDemo: () => actions.push('saved'), onHold: () => actions.push('held')}));
  const folds = findElements(tree, e => e.type === 'details');
  assert.equal(folds.length, 3); assert.ok(folds.every(e => !e.props.open));
  const sources = findElements(tree, e => e.type === 'button' && e.props.className === 'rtab-four-input');
  assert.equal(sources.length, 4);
  sources[2].props.onClick();
  findElements(tree, e => e.type === 'select')[0].props.onChange({target:{value:'raw2'}});
  for (const label of ['实时输入','读取离线结果','冻结当前帧']) {
    findElements(tree, e => e.type === 'button' && e.props['aria-label'] === label)[0].props.onClick();
  }
  assert.deepEqual(selected, ['norm2','raw2']); assert.deepEqual(actions, ['live','saved','held']);
  const waiting = renderToStaticMarkup(React.createElement(panel.View, {
    view:{mode:'waiting',title:'等待输入',detail:'等待连接'}}));
  assert.match(waiting, /等待特征数据/); assert.doesNotMatch(waiting, /旧记录|<path/);
});
test('accepts complete vectors and rejects malformed, missing and nonfinite dimensions', () => {
  assert.equal(parse(snapshot()).modalities.length, 4);
  for (const change of [v => v.modalities[0].features.pop(), v => v.fused_features[0] = Infinity,
    v => v.modalities[1].name = 'thermal', v => v.cosine_similarities[0][1] = 2,
    v => v.complete = false, v => v.modalities[0].weight = -1]) {
    const v = snapshot(); change(v); assert.equal(parse(v), null);
  }
});
test('missing modality has null cosine cells and zero weight', () => {
  const v = snapshot({complete: false}); v.modalities[3].valid = false; v.modalities[3].weight = 0;
  v.modalities[3].features.fill(0);
  for (let i = 0; i < 4; i++) { v.cosine_similarities[i][3] = null; v.cosine_similarities[3][i] = null; }
  assert.ok(parse(v));
  const html = renderToStaticMarkup(React.createElement(panel.View, {
    view: {data: v, mode: 'saved', title: '离线采集结果', detail: 'test'}}));
  assert.match(html, /本次缺失/); assert.match(html, /—/);
});
test('unpaired saved files never become a live synchronized status even with fresh timestamps', () => {
  const v = snapshot({mode: 'saved', alignment: 'unpaired_smoke_test', spectrum_url: '/ignored'});
  const view = panel.deriveView({snapshot: v, receivedAt: 0}, null, 100000, true);
  assert.equal(view.mode, 'saved'); assert.match(view.title, /未配对输入/);
  const html = renderToStaticMarkup(React.createElement(panel.View, {view}));
  for (const label of ['可见光 RGB', '近红外 NIR', 'LiDAR 点云', '声学 PCM', 'log-Mel',
    '1024', '4096', '余弦相似度', '不是识别准确率', '时间差 不适用', '离线音频声谱']) assert.ok(html.includes(label), label);
  assert.doesNotMatch(html, /时间差 0 ms/);
  assert.match(html, /src="\/rtabmap-fourmodal-audio.png"/);
});
test('live status requires fresh features and fresh input heartbeat; stopped sources retain history', () => {
  const sample = {snapshot: snapshot(), receivedAt: 0};
  const heartbeat = {snapshot: {state: 'ready', max_input_age_s: 3,
    input_ages_s: {rgb: 0, infrared: 0, lidar: 0, audio: 0}}, receivedAt: 0};
  assert.equal(panel.deriveView(sample, heartbeat, 1000, true).mode, 'live');
  assert.equal(panel.deriveView(sample, heartbeat, 5000, true).mode, 'stale');
  const stale = copy(heartbeat); stale.snapshot.input_ages_s.audio = 5;
  assert.equal(panel.deriveView(sample, stale, 1000, true).mode, 'stale');
  const working = copy(heartbeat); working.snapshot.state = 'inferencing';
  assert.equal(panel.deriveView(sample, working, 1000, true).mode, 'working');
  assert.equal(panel.deriveView(sample, heartbeat, 1000, false).mode, 'waiting');
});
test('before first features, heartbeat identifies the missing input instead of showing all sources as waiting', () => {
  const heartbeat = {snapshot:{state:'waiting_inputs',max_input_age_s:3,
    input_ages_s:{rgb:.1,infrared:.1,lidar:.1,audio:null}},receivedAt:0};
  const view = panel.deriveView(null, heartbeat, 1000, true);
  assert.equal(view.mode, 'waiting'); assert.equal(view.detail, '等待输入：音频');
  const tree = expandElements(panel.View({view}));
  const labels = findElements(tree, e => e.props?.className === 'rtab-four-card-state');
  assert.deepEqual(labels.map(e => e.props.children.join('')), ['在线','在线','在线','未收到']);
  const absent = panel.deriveView(null, null, 1000, true);
  assert.equal(absent.detail, '未收到特征节点状态');
});
test('MITI subscription removes its precise listener and ignores later deliveries', () => {
  let listener, removed, samples = 0, errors = 0;
  const client = {subscribe(topic, fn) { listener = fn; }, unsubscribe(topic, fn) { removed = fn; }};
  const dispose = panel.subscribe(client, '/miti/fourmodal_features', panel.parseSnapshot,
    () => samples++, () => errors++, () => 12);
  listener({data: JSON.stringify(snapshot())}); listener({data: '{}'}); dispose();
  listener({data: JSON.stringify(snapshot())});
  assert.equal(samples, 1); assert.equal(errors, 1); assert.equal(removed, listener);
});
test('one-time migration keeps original panels/settings and respects deliberate panel removal', () => {
  const anchor = {i: 'rtabmap-scene', type: 'urdf-viewer', props: {view: {distance: 44}}};
  const layout = {widgets: [anchor, {i: 'custom', type: 'topics'}],
    mosaic: {direction: 'row', first: anchor.i, second: 'custom', splitPercentage: 72}};
  const upgraded = context.rtabmapMigrateFourmodalLayout(layout);
  assert.equal(upgraded.widgets.length, 4); assert.equal(upgraded.mosaic.first, layout.mosaic);
  assert.equal(upgraded.widgets[0].props, anchor.props);
  assert.equal(context.rtabmapMigrateFourmodalLayout(upgraded), upgraded);
  const removed = {...upgraded, widgets: upgraded.widgets.slice(0, 2)};
  assert.equal(context.rtabmapMigrateFourmodalLayout(removed), removed);
  assert.equal(context.rtabmapMigrateFourmodalLayout({widgets: [{i: 'custom'}], mosaic: 'custom'}).widgets.length, 1);
});
test('project default layout references each panel exactly once and needs no migration', () => {
  const layout = JSON.parse(fs.readFileSync(path.join(root, 'config/miti_live_layout.json'), 'utf8'));
  const leaves = tree => typeof tree === 'string' ? [tree] : [...leaves(tree.first), ...leaves(tree.second)];
  const ids = leaves(layout.mosaic);
  assert.equal(new Set(ids).size, ids.length);
  assert.equal(ids.length, layout.widgets.length);
  assert.equal(context.rtabmapMigrateFourmodalLayout(layout), layout);
});
