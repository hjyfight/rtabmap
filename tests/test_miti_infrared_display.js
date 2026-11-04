const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname, '..');
const {migrate, widget} = vm.runInNewContext(
  fs.readFileSync(path.join(root, 'config/miti_infrared_display.js'), 'utf8') +
  '; ({migrate:rtabmapMigrateInfraredLayout,widget:rtabmapInfraredWidget})');
const depth = {i:'rtabmap-depth',type:'camera-viewer',title:'我的深度',
  props:{topic:'/miti/aligned_depth/image_raw',maxFps:5,valueMin:500}};
const scene = {i:'rtabmap-scene',type:'urdf-viewer',props:{scene:{heightClip:{enabled:true,height:1.8}}}};
const leaves = tree => typeof tree === 'string' ? [tree] : [...leaves(tree.first),...leaves(tree.second)];

test('inserts infrared beside depth once and preserves custom settings and other panes', () => {
  const before = {widgets:[scene,depth],mosaic:{direction:'row',first:scene.i,second:depth.i,splitPercentage:73}};
  const snapshot = JSON.stringify(before), after = migrate(before);
  assert.equal(JSON.stringify(before),snapshot);
  assert.deepEqual(leaves(after.mosaic),[scene.i,depth.i,'rtabmap-infrared']);
  assert.equal(after.mosaic.splitPercentage,73);
  assert.equal(after.widgets[0],scene);
  assert.equal(after.widgets[1].props,depth.props);
  assert.equal(after.widgets[1].title,depth.title);
  assert.equal(migrate(after),after);
  const removed = {...after,widgets:after.widgets.filter(w=>w.i!=='rtabmap-infrared'),mosaic:before.mosaic};
  assert.equal(migrate(removed),removed,'a user removal must persist');
});

test('keeps an existing infrared pane, unrelated layouts, and hidden anchors intact', () => {
  for (const topic of ['/camera/infra1/image_rect_raw','/camera/infra2/image_rect_raw']) {
    const custom = {i:'custom-ir',type:'camera-viewer',title:'我的红外',props:{topic,maxFps:3}};
    const before = {widgets:[depth,custom],mosaic:{direction:'row',first:depth.i,second:custom.i}};
    const after = migrate(before);
    assert.equal(after.widgets.length,2);
    assert.equal(after.widgets[1],custom);
    assert.equal(after.mosaic,before.mosaic);
    assert.equal(migrate(after),after);
  }
  for (const before of [{widgets:[scene],mosaic:scene.i},{widgets:[scene,depth],mosaic:scene.i}])
    assert.equal(migrate(before),before);
  const color = {...depth,i:'rtabmap-color'};
  assert.deepEqual(leaves(migrate({widgets:[color],mosaic:color.i}).mosaic),[color.i,'rtabmap-infrared']);
});

test('default dashboard contains one native infrared viewer and all mosaic leaves match widgets', () => {
  const layout = JSON.parse(fs.readFileSync(path.join(root,'config/miti_live_layout.json')));
  const infrared = layout.widgets.find(w=>w.i==='rtabmap-infrared');
  assert.deepEqual(infrared,JSON.parse(JSON.stringify(widget())));
  assert.equal(migrate(layout),layout);
  assert.equal(new Set(leaves(layout.mosaic)).size,layout.widgets.length);
  assert.deepEqual(new Set(leaves(layout.mosaic)),new Set(layout.widgets.map(w=>w.i)));
});
