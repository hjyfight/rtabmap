const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const migrate = vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../config/miti_depth_display.js'), 'utf8') +
  '; rtabmapMigrateDepthWidget');

test('migrates the legacy depth source while preserving panel settings and input objects', () => {
  const old = {i: 'rtabmap-depth', type: 'camera-viewer', title: '我的深度', x: 8, y: 3, locked: true,
    props: {topic: '/camera/aligned_depth_to_color/image_raw', maxFps: 5, valueMin: 800, valueMax: 2300}};
  const snapshot = JSON.stringify(old), updated = migrate(old);
  assert.equal(updated.props.topic, '/miti/aligned_depth/image_raw');
  assert.equal(updated.props.valueMin, 800);
  assert.equal(updated.props.valueMax, 2300);
  assert.equal(updated.props.maxFps, 5);
  assert.equal(updated.title, old.title);
  assert.equal(updated.locked, true);
  assert.equal(JSON.stringify(old), snapshot);
  assert.equal(migrate(updated), updated);
  const manuallyChanged = {...updated, props: {...updated.props, topic: old.props.topic}};
  assert.equal(migrate(manuallyChanged), manuallyChanged);
});

test('preserves custom depth topics and all other panels including saved clipping', () => {
  for (const widget of [
    {i: 'rtabmap-depth', type: 'camera-viewer', props: {topic: '/custom_depth'}},
    {i: 'custom-camera', type: 'camera-viewer', props: {topic: '/camera/aligned_depth_to_color/image_raw'}},
    {i: 'rtabmap-scene', type: 'urdf-viewer', props: {scene: {heightClip: {enabled: true, height: 1.2}}}}
  ]) assert.equal(migrate(widget), widget);
});
