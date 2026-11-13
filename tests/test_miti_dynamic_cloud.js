const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const source = fs.readFileSync(path.join(root, 'config/miti_dynamic_cloud.js'), 'utf8');
const {migrate, factory, visible} = vm.runInNewContext(source + ';({migrate:rtabmapMigrateDynamicCloudWidget,factory:rtabmapDynamicCloudFactory,visible:rtabmapLiveScanVisible})');
const ui = factory({});
const layout = JSON.parse(fs.readFileSync(path.join(root, 'config/miti_live_layout.json'), 'utf8'));

test('existing 3D map switches to cleaned history plus replace-only live cloud and preserves view/labels', () => {
  const original = layout.widgets.find(w => w.i === 'rtabmap-scene');
  const upgraded = migrate(original);
  assert.equal(upgraded.props.topics['/cloud_map'].visible, false);
  assert.equal(upgraded.props.topics['/miti/clean_map'].visible, true);
  assert.equal(upgraded.props.topics['/miti/live_scan'].visible, true);
  assert.equal(upgraded.props.topics['/miti/live_scan'].staleAfter, 1.5);
  assert.equal(upgraded.props.view, original.props.view);
  assert.equal(upgraded.props.frame, original.props.frame);
  assert.equal(upgraded.props.scene.heightClip, original.props.scene.heightClip);
  assert.equal(upgraded.props.topics['/miti/semantic_markers'], original.props.topics['/miti/semantic_markers']);
  assert.equal(original.props.topics['/cloud_map'].visible, true);
});

test('raw/cleaned comparison switches exactly these clouds and survives future migrations', () => {
  const upgraded = migrate(layout.widgets[0]);
  const raw = ui.toggle(upgraded.props);
  assert.equal(raw.scene.rtabmapDynamicCloud, false);
  assert.equal(raw.topics['/cloud_map'].visible, true);
  assert.equal(raw.topics['/miti/clean_map'].visible, false);
  assert.equal(raw.topics['/miti/live_scan'].visible, false);
  const saved = {...upgraded, props: raw};
  assert.equal(migrate(saved), saved);
  const clean = ui.toggle(raw);
  assert.equal(clean.topics['/cloud_map'].visible, false);
  assert.equal(clean.topics['/miti/clean_map'].visible, true);
  assert.equal(clean.topics['/miti/live_scan'].visible, true);
  assert.equal(clean.view, raw.view);
  assert.equal(clean.topics['/robot_pose'], raw.topics['/robot_pose']);
});

test('hidden source clouds remain hidden during migration and other map panels are untouched', () => {
  const original = layout.widgets[0];
  const hidden = {...original, props: {...original.props, topics: {...original.props.topics,
    '/cloud_map': {...original.props.topics['/cloud_map'], visible: false}}}};
  const upgraded = migrate(hidden);
  assert.equal(upgraded.props.topics['/miti/clean_map'].visible, false);
  assert.equal(upgraded.props.topics['/miti/live_scan'].visible, false);
  for (const widget of layout.widgets.slice(1)) assert.equal(migrate(widget), widget);
});

test('a stopped live scan is removed after 1.5 seconds while the environment remains visible', () => {
  const live = {topic: '/miti/live_scan'}, map = {topic: '/miti/clean_map'};
  assert.equal(visible(live, new Date(1000), 2499), true);
  assert.equal(visible(live, new Date(1000), 2501), false);
  assert.equal(visible(live, null, 2000), false);
  assert.equal(visible(map, new Date(1000), 999999), true);
  assert.equal(visible({topic: '/cloud_map'}, null, 999999), true);
});
