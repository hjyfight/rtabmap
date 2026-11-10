const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {createRequire} = require('node:module');
const root = path.resolve(__dirname, '..');
const vendor = createRequire(path.join(root, '.tools/miti/root/usr/lib/miti/package.json'));
const React = vendor('react');
const {renderToStaticMarkup} = vendor('react-dom/server.browser');
let source = fs.readFileSync(path.join(root, 'config/miti_semantic_labels.js'), 'utf8');
if (process.env.MITI_TEST_RENDERER_BUNDLE) {
  const bundle = fs.readFileSync(process.env.MITI_TEST_RENDERER_BUNDLE, 'utf8');
  const begin = bundle.indexOf('function rtabmapSemanticLabelsFactory(');
  const end = bundle.indexOf('const RtabmapSemanticLabels = ', begin);
  assert.ok(begin >= 0 && end > begin);
  source = bundle.slice(begin, end);
}
const factory = vm.runInNewContext(source + ';rtabmapSemanticLabelsFactory');
const htmlCalls = [];
const ui = factory(React, props => {
  htmlCalls.push(props);
  return React.createElement('div', null, props.children);
});
const marker = (id, text) => ({type: 9, ns: `miti/${id}/label`, text, scale: {z: .16}});
// Reproduce "television +5 / box +3": every record must keep its own stable
// representation instead of letting unrelated projected neighbors rename it.
const markers = [marker(5, '电视'), ...[10, 22, 27, 64, 90].map(id => marker(id, '其他物品')),
  marker(4, '盒子'), ...[76, 97, 155].map(id => marker(id, '其他家具'))];
function render(input = markers, settings = {}, labelScale = 6) {
  htmlCalls.length = 0;
  return renderToStaticMarkup(React.createElement(ui.Provider, {settings},
    input.map(m => React.createElement(ui.Label, {key: m.ns, marker: m,
      color: '#fff', opacity: 1, labelScale}))));
}
function texts(html) {
  return Object.fromEntries([...html.matchAll(/<span[^>]*data-semantic-id="([^"]+)"[^>]*>(.*?)<\/span>/g)]
    .map(match => [match[1], match[2]]));
}

test('legacy declutter layouts render TV/box immediately with no aggregate counts or initial hiding', () => {
  const html = render(markers, {rtabmapSemanticLabels: 'declutter'});
  const labels = texts(html);
  assert.equal(labels['miti/5/label'], '电视');
  assert.equal(labels['miti/4/label'], '盒子');
  assert.equal(Object.keys(labels).length, 10);
  assert.doesNotMatch(html, /\+\d|visibility:hidden|aria-hidden="true"/);
  for (const id of [10, 22, 27, 64, 90, 76, 97, 155]) assert.equal(labels[`miti/${id}/label`], '·');
  assert.match(html, /预测类别：其他物品 · 对象 #10/);
  assert.equal(ui.modeOf({}), 'stable');
  assert.equal(ui.modeOf({rtabmapSemanticLabels: 'declutter'}), 'stable');
});

test('orbit, zoom, changed projected neighbors and input order do not change per-object text or visibility', () => {
  const baseline = texts(render());
  for (const theta of [0, 30, 90, 180, 270, 359]) {
    for (const distance of [2, 12, 50]) {
      const projected = markers.slice().reverse().map((m, i) => ({...m,
        pose: {position: {x: Math.cos(theta)*i, y: Math.sin(theta)*i, z: i / distance}},
        rect: {left: i % 2 ? 100 : 500, right: 600, top: 100, bottom: 125}}));
      assert.deepEqual(texts(render(projected, {rtabmapSemanticLabels: 'stable', camera: {theta, distance}})), baseline);
    }
  }
});

test('all and hidden modes are reversible and each scene has an independent setting', () => {
  assert.equal(texts(render(markers, {rtabmapSemanticLabels: 'all'}))['miti/10/label'], '其他物品');
  assert.deepEqual(texts(render(markers, {rtabmapSemanticLabels: 'hidden'})), {});
  assert.equal(texts(render())['miti/10/label'], '·');
  const scene = mode => React.createElement(ui.Provider, {settings: {rtabmapSemanticLabels: mode}},
    React.createElement(ui.Label, {marker: marker(10, '其他物品'), color: '#fff', opacity: 1, labelScale: 1}));
  const html = renderToStaticMarkup(React.createElement(React.Fragment, null, scene('all'), scene('stable'), scene('hidden')));
  assert.match(html, />其他物品<\/span>.*>·<\/span>/);
  assert.equal((html.match(/data-semantic-id=/g) || []).length, 2);
});

test('adding, deleting or updating one record never changes another record, and same-class neighbors remain distinct', () => {
  const baseline = texts(render()), chair = marker(200, '椅子'), another = marker(201, '椅子');
  const withChairs = texts(render([...markers, chair, another]));
  assert.equal(withChairs[chair.ns], '椅子');
  assert.equal(withChairs[another.ns], '椅子');
  for (const [id, text] of Object.entries(baseline)) assert.equal(withChairs[id], text);
  const after = texts(render(markers.filter(m => m.ns !== 'miti/10/label').map(m =>
    m.ns === 'miti/4/label' ? {...m, text: '柜台'} : m)));
  assert.equal(after['miti/10/label'], undefined);
  assert.equal(after['miti/4/label'], '柜台');
  for (const [id, text] of Object.entries(baseline)) if (!['miti/10/label', 'miti/4/label'].includes(id)) assert.equal(after[id], text);
});

test('semantic text has bounded pixel size and does not enable distance scaling or geometry occlusion', () => {
  render();
  for (const call of htmlCalls) {
    assert.equal(call.distanceFactor, undefined);
    assert.equal(call.transform, undefined);
    assert.equal(call.occlude, false);
  }
  for (const scale of [undefined, 0, NaN, Infinity, -1, .01, 1, 6, 100]) {
    assert.ok(ui.fontSize(scale) >= 10 && ui.fontSize(scale) <= 16);
  }
});

test('controls keep user modes and unrelated map settings; they offer no camera-dependent mode', () => {
  const settings = {topics: {'/miti/semantic_markers': {visible: true}},
    scene: {heightClip: {enabled: true}, rtabmapSemanticLabels: 'declutter'}, view: {distance: 4}};
  let changed;
  const tree = ui.Controls({settings, onChange: value => {changed = value;}});
  const select = tree.props.children[1];
  assert.equal(select.props.value, 'stable');
  select.props.onChange({target: {value: 'all'}});
  assert.equal(changed.scene.rtabmapSemanticLabels, 'all');
  assert.equal(changed.scene.heightClip, settings.scene.heightClip);
  assert.equal(changed.view, settings.view);
  assert.equal(changed.topics, settings.topics);
  const html = renderToStaticMarkup(tree);
  for (const text of ['物体标签', '稳定标签', '全部标签', '隐藏标签']) assert.ok(html.includes(text));
  assert.doesNotMatch(html, /自动避让|value="declutter"/);
  assert.equal(ui.Controls({settings: {topics: {}}, onChange() {}}), null);
});

test('only project semantic text labels use this renderer and prediction text is escaped', () => {
  for (const m of [{type: 2, ns: 'miti/5/point'}, {type: 9, ns: 'rtabmap/navigation/region'},
                   {type: 9, ns: 'another/label'}]) assert.equal(ui.isSemantic(m), false);
  assert.equal(ui.isSemantic(marker(5, '电视')), true);
  const html = render([marker(5, '<script>电视</script>')]);
  assert.ok(html.includes('&lt;script&gt;电视&lt;/script&gt;'));
});
