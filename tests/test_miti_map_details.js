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
const source = fs.readFileSync(path.join(root, 'config/miti_map_details.js'), 'utf8');
const {factory, migrate} = vm.runInNewContext(source + '; ({factory:rtabmapMapDetailsFactory,migrate:rtabmapMigrateWallWidget})');
const ui = factory(React);
const pose = {valid:true,frame_id:'map',tracked_frame:'livox_frame',body_frame:false,
  position_m:[1.234,-2,0.5],orientation_xyzw:[0,0,0,1],rpy_deg:[1,2,90]};
const data = {schema:1,pose,walls:{state:'ready',count:2}};

test('renders real numeric units, reference frame and missing body calibration', () => {
  const html = renderToStaticMarkup(React.createElement(ui.View, {
    data, pose, connected:true,expanded:true,wallsVisible:true
  }));
  for (const text of ['1.234','-2.000','90.0','map','livox_frame','base_link','显示墙面','横滚','航向','/ m','/ °']) assert.ok(html.includes(text),text);
  assert.match(html,/aria-pressed="true"/);
});

test('disconnect, stale status and invalid pose do not display old numeric coordinates', () => {
  const sample = {data,received:0};
  assert.equal(ui.freshPose(sample,1000,true),pose);
  assert.equal(ui.freshPose(sample,2000,true),null);
  assert.equal(ui.freshPose(sample,0,false),null);
  assert.equal(ui.freshPose({data:{...data,pose:{valid:false}},received:0},0,true),null);
  const html = renderToStaticMarkup(React.createElement(ui.View, {data,pose:null,connected:true,stale:true,expanded:true}));
  assert.match(html,/位姿更新已中断/);
  assert.doesNotMatch(html,/1\.234/);
});

test('wall migration and toggle preserve custom layers, clipping, and user visibility', () => {
  const widget = {i:'rtabmap-scene',type:'urdf-viewer',props:{topics:{'/cloud_map':{visible:true}},scene:{heightClip:{enabled:true,height:1.5}}}};
  const updated = migrate(widget);
  assert.equal(updated.props.topics['/miti/walls'].visible,true);
  const hidden = ui.toggleWalls(updated.props);
  assert.equal(hidden.topics['/miti/walls'].visible,false);
  assert.equal(hidden.topics['/cloud_map'],widget.props.topics['/cloud_map']);
  assert.equal(hidden.scene,widget.props.scene);
  assert.equal(ui.toggleWalls(hidden).topics['/miti/walls'].visible,true);
  assert.equal(migrate(updated),updated);
  assert.equal(widget.props.topics['/miti/walls'],undefined);
});

test('repairs the already-migrated wall layout to show robot pose, without resetting styles', () => {
  const widget = {i:'rtabmap-scene',type:'urdf-viewer',rtabmapWallsVersion:1,
    props:{topics:{'/robot_pose':{visible:false,axisScale:1.2},'/miti/walls':{visible:false,type:'markers'}}}};
  const updated = migrate(widget);
  assert.equal(updated.props.topics['/robot_pose'].visible,true);
  assert.equal(updated.props.topics['/robot_pose'].type,'pose');
  assert.equal(updated.props.topics['/robot_pose'].axisScale,1.2);
  assert.equal(updated.props.topics['/miti/walls'].visible,false);
  const userHidden = {...updated,props:{...updated.props,topics:{...updated.props.topics,
    '/robot_pose':{...updated.props.topics['/robot_pose'],visible:false}}}};
  assert.equal(migrate(userHidden),userHidden);
  assert.equal(widget.props.topics['/robot_pose'].visible,false);
});

test('settings readout shows the six live coordinates in normal panel flow', () => {
  const html = renderToStaticMarkup(React.createElement(ui.View, {
    data,pose,connected:true,expanded:true,wallsVisible:true,inline:true
  }));
  assert.match(html,/class="rtab-map-details rtab-map-inline"/);
  for (const text of ['机器人位姿','1.234','-2.000','0.500','1.0','2.0','90.0']) assert.ok(html.includes(text),text);
  assert.match(source,/\.rtab-map-details\.rtab-map-inline \{position:relative/);
});

test('validates status and rejects nonfinite or malformed pose data', () => {
  assert.equal(ui.parse({data:JSON.stringify(data)}).walls.count,2);
  assert.equal(ui.parse({data:'{}'}),null);
  assert.equal(ui.parse({data:JSON.stringify({...data,pose:{...pose,position_m:[null,2,3]}})}),null);
});

test('wall transparency and height clipping update without mutating other materials', () => {
  const material = {userData:{}};
  ui.configureWallMaterial(material,{enabled:true,plane:[0,0,1,-2]});
  const shader = {uniforms:{},vertexShader:'#include <begin_vertex>',fragmentShader:'void main() { }'};
  material.onBeforeCompile(shader);
  assert.match(shader.vertexShader,/modelMatrix/);
  assert.match(shader.fragmentShader,/discard/);
  ui.configureWallMaterial(material,{enabled:false,plane:[0,0,1,-1]});
  assert.equal(shader.uniforms.rtabmapClipEnabled.value,false);
  assert.equal(shader.uniforms.rtabmapClipPlane.value[3],-1);
});
