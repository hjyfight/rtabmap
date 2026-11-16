const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const migrate = vm.runInNewContext(fs.readFileSync(path.join(root, 'config/miti_arc_camera_display.js'), 'utf8') +
  '; rtabmapMigrateArcCameraLayout');
const scene = {i:'rtabmap-scene',type:'urdf-viewer',props:{view:{distance:17}},rtabmapArcCameraLayoutVersion:1};
const infrared = {i:'rtabmap-infrared',type:'camera-viewer',title:'我的红外',
  props:{topic:'/camera/infra2/image_rect_raw',maxFps:5,valueMin:20}};
const arc = {i:'rtabmap-arc-camera',type:'camera-viewer',props:{topic:'/arc_camera/image_raw'}};
const leaves = tree => typeof tree === 'string' ? [tree] : tree ? [...leaves(tree.first),...leaves(tree.second)] : [];

test('removes ARC from saved nested layouts and collapses split without resetting views', () => {
  const before = {widgets:[scene,infrared,arc],mosaic:{direction:'row',first:scene.i,
    second:{direction:'column',first:infrared.i,second:arc.i,splitPercentage:40},splitPercentage:73}};
  const snapshot = JSON.stringify(before), after = migrate(before);
  assert.equal(JSON.stringify(before),snapshot);
  assert.deepEqual(leaves(after.mosaic),[scene.i,infrared.i]);
  assert.equal(after.mosaic.splitPercentage,73);
  assert.equal(after.widgets[0],scene); assert.equal(after.widgets[1],infrared);
  assert.equal(migrate(after),after);
});
test('removes custom ARC ids and dangling default leaves, handles empty layout', () => {
  const custom = {...arc,i:'custom-arc'};
  const after = migrate({widgets:[scene,custom],mosaic:{direction:'column',first:custom.i,second:scene.i}});
  assert.equal(after.mosaic,scene.i); assert.equal(after.widgets.length,1);
  assert.equal(migrate({widgets:[arc],mosaic:arc.i}).mosaic,null);
  assert.equal(migrate({widgets:[scene],mosaic:{first:scene.i,second:arc.i}}).mosaic,scene.i);
  for (const layout of [{widgets:[],mosaic:null},{widgets:[scene,infrared],mosaic:infrared.i},null])
    assert.equal(migrate(layout),layout);
});
test('project and fusion layouts have no ARC and one visible leaf per widget', () => {
  for (const file of ['miti_live_layout.json','miti_fusion_layout.json']) {
    const layout = JSON.parse(fs.readFileSync(path.join(root,'config',file)));
    assert.equal(layout.widgets.filter(w=>w.props?.topic==='/arc_camera/image_raw').length,0);
    assert.equal(migrate(layout),layout);
    assert.equal(new Set(leaves(layout.mosaic)).size,layout.widgets.length);
    assert.deepEqual(new Set(leaves(layout.mosaic)),new Set(layout.widgets.map(w=>w.i)));
  }
});
