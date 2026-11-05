const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const {migrate, widget} = vm.runInNewContext(
  fs.readFileSync(path.join(root, 'config/miti_arc_camera_display.js'), 'utf8') +
  '; ({migrate:rtabmapMigrateArcCameraLayout,widget:rtabmapArcCameraWidget})');
const scene = {i:'rtabmap-scene',type:'urdf-viewer',props:{view:{distance:17}}};
const infrared = {i:'rtabmap-infrared',type:'camera-viewer',title:'我的红外',
  props:{topic:'/camera/infra2/image_rect_raw',maxFps:5,valueMin:20}};
const leaves = tree => typeof tree === 'string' ? [tree] :
  tree ? [...leaves(tree.first),...leaves(tree.second)] : [];

test('adds ARC below infrared once and preserves saved settings and deliberate removal', () => {
  const before = {widgets:[scene,infrared],mosaic:{direction:'row',first:scene.i,second:infrared.i,splitPercentage:73}};
  const snapshot = JSON.stringify(before), after = migrate(before);
  assert.equal(JSON.stringify(before),snapshot);
  assert.deepEqual(leaves(after.mosaic),[scene.i,infrared.i,'rtabmap-arc-camera']);
  assert.equal(after.mosaic.splitPercentage,73);
  assert.equal(after.widgets[0].props,scene.props);
  assert.equal(after.widgets[1],infrared);
  assert.equal(after.widgets[2].props.topic,'/arc_camera/image_raw');
  assert.equal(migrate(after),after);
  const removed = {...after,widgets:after.widgets.filter(w=>w.i!=='rtabmap-arc-camera'),mosaic:before.mosaic};
  assert.equal(migrate(removed),removed);
});

test('retains custom ARC panes and leaves unrelated or hidden project layouts intact', () => {
  const custom = {i:'custom-arc',type:'camera-viewer',title:'新相机',props:{topic:'/arc_camera/image_raw',maxFps:3}};
  const before = {widgets:[scene,custom],mosaic:{direction:'row',first:scene.i,second:custom.i}};
  const after = migrate(before);
  assert.equal(after.widgets.length,2);
  assert.equal(after.widgets[1],custom);
  assert.equal(after.mosaic,before.mosaic);
  assert.equal(migrate(after),after);
  for (const layout of [
    {widgets:[custom],mosaic:custom.i},
    {widgets:[scene,infrared,custom],mosaic:custom.i},
    {widgets:[],mosaic:null}
  ]) assert.equal(migrate(layout),layout);
});

test('falls back to a visible project depth, color, or scene panel', () => {
  for (const anchor of [scene,{...infrared,i:'rtabmap-depth'},{...infrared,i:'rtabmap-color'}]) {
    const before = {widgets:[anchor],mosaic:anchor.i}, after = migrate(before);
    assert.deepEqual(leaves(after.mosaic),[anchor.i,'rtabmap-arc-camera']);
    assert.equal(migrate(after),after);
  }
});

test('default layout includes one ARC panel and matches its visible mosaic', () => {
  const layout = JSON.parse(fs.readFileSync(path.join(root,'config/miti_live_layout.json')));
  assert.deepEqual(layout.widgets.find(w=>w.i==='rtabmap-arc-camera'),JSON.parse(JSON.stringify(widget())));
  assert.equal(layout.widgets.filter(w=>w.props?.topic==='/arc_camera/image_raw').length,1);
  assert.equal(migrate(layout),layout);
  assert.equal(new Set(leaves(layout.mosaic)).size,layout.widgets.length);
  assert.deepEqual(new Set(leaves(layout.mosaic)),new Set(layout.widgets.map(w=>w.i)));
});
