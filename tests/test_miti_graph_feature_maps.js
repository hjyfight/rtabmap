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
let source = fs.readFileSync(path.join(root, 'config/miti_graph_feature_maps.js'), 'utf8');
const renderer = process.env.MITI_TEST_RENDERER_BUNDLE
  ? fs.readFileSync(process.env.MITI_TEST_RENDERER_BUNDLE, 'utf8') : null;
if (renderer) {
  const begin = renderer.indexOf('function rtabmapVersionPanelChunk(');
  const end = renderer.indexOf('const RtabmapGraphFeatureMaps = ', begin);
  assert.ok(begin >= 0 && end > begin);
  source = renderer.slice(begin, end);
}
const {migrate, widget, factory, kindOf, versionChunk, setMode} = vm.runInNewContext(source +
  '; ({migrate:rtabmapMigrateGraphFeatureLayout,widget:rtabmapGraphFeatureWidget,factory:rtabmapGraphFeatureMapsFactory,kindOf:rtabmapGraphMapKind,versionChunk:rtabmapVersionPanelChunk,setMode:rtabmapSetTopologyMode})',
  {setInterval, clearInterval});
const ui = factory(React);
const leaves = tree => typeof tree === 'string' ? [tree] : tree ? [...leaves(tree.first), ...leaves(tree.second)] : [];
const primary = {i:'rtabmap-scene',type:'urdf-viewer',title:'我的三维地图',props:{view:{distance:17}}};
const grid = {i:'grid',type:'urdf-viewer',props:{scene:{rtabmapGrid:true}}};
const camera = {i:'camera',type:'camera-viewer',props:{topic:'/camera/color/image_raw'}};
const snapshot = {schema:1,state:'ready',frame_id:'map',epoch:1,updated_at:1794220300,
  topology:{nodes:12,links:14,bounds:{center:[1,2,3],span:[10,8,2]}},
  features:{points:1500,feature_nodes:10,bounds:{center:[1,2,3],span:[10,8,2]}}};

test('navigation labels remain small in orthographic views regardless of marker or scene scale', () => {
  const calls = [];
  const labels = factory(React, props => {
    calls.push(props);
    return React.createElement('div', null, props.children);
  });
  for (const scale of [.01, .22, 6, 100]) {
    const marker = {type:9, ns:'navigation/labels', id:1, text:'区域 1', scale:{z:scale}};
    assert.equal(labels.isNavigationLabel(marker), true);
    const html = renderToStaticMarkup(React.createElement(labels.NavigationLabel,
      {marker, color:'#fff', opacity:1, labelScale:scale}));
    assert.match(html, /font:12px/);
    assert.match(html, /区域 1/);
  }
  for (const call of calls) {
    assert.equal(call.distanceFactor, undefined);
    assert.equal(call.transform, undefined);
  }
  for (const marker of [{type:2,ns:'navigation/labels'}, {type:9,ns:'miti/1/label'},
    {type:9,ns:'another/label'}]) assert.equal(labels.isNavigationLabel(marker), false);
});

test('installed marker dispatcher selects fixed-size navigation text and retains native text for other markers',
  {skip: !renderer}, () => {
    const Html = props => React.createElement('div', null, props.children);
    const labels = factory(React, Html);
    const begin = renderer.indexOf('if(RtabmapSemanticLabels.isSemantic(n))');
    const end = renderer.indexOf('if(11===n.type)', begin);
    assert.ok(begin >= 0 && end > begin);
    const dispatch = vm.runInNewContext('(function(n){const d=1,c=100,u={getHexString:()=>"ffffff"};' +
      renderer.slice(begin, end) + '})', {r:{jsx:React.createElement}, ta:{V:Html},
      RtabmapSemanticLabels:{isSemantic:()=>false}, RtabmapGraphFeatureMaps:labels});
    const marker = {type:9,ns:'navigation/labels',id:1,text:'区域 1',scale:{z:.22}};
    assert.equal(dispatch(marker).type, labels.NavigationLabel);
    assert.match(renderToStaticMarkup(dispatch(marker)), /font:12px/);
    assert.equal(dispatch({...marker,ns:'another/label'}).type, Html);
  });

test('saved feature panels get smaller legacy defaults once while keeping custom settings and views', () => {
  const old = widget('features');
  delete old.rtabmapFeatureDisplayVersion;
  const topic = old.props.topics['/miti/sparse_features'];
  topic.pointSize = 4;
  topic.maxPoints = 150000;
  const before = {widgets:[{...primary,rtabmapGraphFeatureLayoutVersion:1},old],mosaic:old.i};
  const after = migrate(before), fixed = after.widgets[1];
  assert.equal(fixed.props.topics['/miti/sparse_features'].pointSize, 2);
  assert.equal(fixed.props.topics['/miti/sparse_features'].maxPoints, 15000);
  assert.equal(fixed.props.view, old.props.view);
  assert.equal(fixed.props.topics['/robot_pose'], old.props.topics['/robot_pose']);
  assert.equal(after.mosaic, before.mosaic);
  assert.equal(topic.pointSize, 4);
  assert.equal(migrate(after), after);
  topic.pointSize = 1;
  topic.maxPoints = 8000;
  const custom = migrate(before).widgets[1].props.topics['/miti/sparse_features'];
  assert.equal(custom.pointSize, 1);
  assert.equal(custom.maxPoints, 8000);
});

test('updated pages request a distinct renderer URL even with an immutable old chunk cached', () => {
  const oldUrl = 'static/chunks/453.e8d48709a26a0ed3.js';
  const cache = new Map([[oldUrl, 'old renderer without graph controls']]);
  const runtime = {u: id => id === 453 ? oldUrl : `static/chunks/${id}.original.js`};
  versionChunk(runtime, 'revision-2');
  assert.equal(cache.has(runtime.u(453)), false);
  assert.equal(runtime.u(453), oldUrl + '?rtabmap_panel=revision-2');
  assert.equal(runtime.u(973), 'static/chunks/973.original.js');
  const nextRuntime = {u: id => id === 453 ? oldUrl : String(id)};
  versionChunk(nextRuntime, 'revision-3');
  assert.notEqual(nextRuntime.u(453), runtime.u(453));
});

test('adds maps under the existing map pair once and preserves views and camera position', () => {
  const before = {widgets:[primary,grid,camera],mosaic:{direction:'row',first:{direction:'row',first:primary.i,second:grid.i,splitPercentage:60},second:camera.i}};
  const after = migrate(before);
  assert.equal(after.widgets.length,5);
  assert.equal(after.widgets[0].props,primary.props);
  assert.equal(after.widgets[1],grid);
  assert.equal(after.widgets[2],camera);
  assert.equal(after.mosaic.first.first,before.mosaic.first);
  assert.deepEqual(leaves(after.mosaic),[primary.i,grid.i,'rtabmap-topology-scene','rtabmap-features-scene',camera.i]);
  assert.equal(migrate(after),after);
  const removed={...after,widgets:after.widgets.filter(w=>w.i!=='rtabmap-topology-scene')};
  assert.equal(migrate(removed),removed);
  assert.equal(migrate({widgets:[camera]}).widgets[0],camera);
});

test('configured map panels are reused and generated ids avoid collisions', () => {
  const existing=widget('topology','mine');
  const before={widgets:[primary,existing,{i:'rtabmap-features-scene',type:'log'}],mosaic:{direction:'row',first:primary.i,second:existing.i}};
  const after=migrate(before);
  assert.equal(after.widgets.length,4);
  assert.equal(after.widgets[1],existing);
  assert.ok(after.widgets.some(w=>w.i==='rtabmap-features-scene-2'));
});

test('upgrades the old pose graph in place and preserves later mode and view choices', () => {
  const old = widget('topology');
  old.props = setMode(old.props, 'pose_graph');
  old.title = '拓扑地图 · 关键帧与回环';
  delete old.rtabmapNavigationVersion;
  const before = {widgets: [{...primary, rtabmapGraphFeatureLayoutVersion: 1}, old, widget('features')],
    mosaic: {direction: 'row', first: primary.i, second: old.i}};
  const after = migrate(before), updated = after.widgets[1];
  assert.equal(after.widgets.length, before.widgets.length);
  assert.equal(after.mosaic, before.mosaic);
  assert.equal(updated.title, '导航连通图 · 区域与通道');
  assert.equal(updated.props.view, old.props.view);
  assert.equal(updated.props.topics['/miti/navigation_map'].visible, true);
  assert.equal(updated.props.topics['/miti/topological_map'].visible, false);
  const changed = {...after, widgets: after.widgets.map(w => w === updated ? {...w, props: setMode(w.props, 'pose_graph')} : w)};
  assert.equal(migrate(changed), changed);
  assert.equal(changed.widgets[1].props.topics['/miti/navigation_map'].visible, false);
  assert.equal(changed.widgets[1].props.topics['/miti/grid_map'].visible, false);
});

test('navigation status shows geometric candidates and an explicit insufficient-space state', () => {
  const message = {schema:1, kind:'navigation', state:'ready', frame_id:'map', epoch:1,
    topology:{nodes:5,links:4,regions:2,corridors:1,doorways:2,components:1,required_clearance_m:.45,
      bounds:{center:[1,2,0],span:[10,5,0]}}};
  assert.ok(ui.parse({data:JSON.stringify(message)}));
  const html = renderToStaticMarkup(React.createElement(ui.View,{kind:'topology',mode:'navigation',data:message,connected:true}));
  for (const text of ['2 个区域','1 条通道','2 个候选门口','几何候选','0.45','关键帧位姿图','适应地图']) assert.ok(html.includes(text),text);
  assert.equal(html.includes('显示墙面'),false);
  const waiting = {...message,state:'waiting',reason:'已知空闲区域不足以容纳当前机器人净空'};
  const empty = renderToStaticMarkup(React.createElement(ui.View,{kind:'topology',mode:'navigation',data:waiting,connected:true}));
  assert.ok(empty.includes(waiting.reason));
  assert.equal(empty.includes('2 个区域'),false);
  const effects=[], subscriptions=[];
  const hooks={createElement:React.createElement,useState:v=>[v,()=>{}],useRef:v=>({current:v}),useEffect:fn=>effects.push(fn)};
  const client={subscribe:topic=>subscriptions.push(topic),unsubscribe(){}};
  factory(hooks).Controls({client,settings:widget('topology').props,onChange(){}});
  const cleanups=effects.map(f=>f());
  assert.deepEqual(subscriptions,['/miti/navigation_status']);
  cleanups.forEach(f=>f?.());
});

test('navigation information can be hidden and restored without changing map or mode', () => {
  const settings = widget('topology').props;
  const waiting = {schema:1,kind:'navigation',state:'waiting',frame_id:'map',epoch:1,
    reason:'已知空闲区域不足以容纳当前机器人净空',
    topology:{nodes:0,links:0,regions:0,corridors:0,doorways:0,components:0}};
  const expanded = renderToStaticMarkup(React.createElement(ui.View,
    {kind:'topology',mode:'navigation',data:waiting,connected:true}));
  assert.ok(expanded.includes('隐藏导航信息'));
  assert.ok(expanded.includes('当前仅显示栅格背景，尚未生成导航节点和连线'));
  const hidden = renderToStaticMarkup(React.createElement(ui.View,
    {kind:'topology',mode:'navigation',data:waiting,connected:true,collapsed:true}));
  assert.ok(hidden.includes('展开导航信息'));
  assert.ok(hidden.includes('aria-expanded="false"'));
  for (const text of ['净空半径','候选门口','适应地图',waiting.reason]) assert.equal(hidden.includes(text),false);
  let changed;
  const hooks = {createElement:React.createElement,useState:v=>[v,()=>{}],useRef:v=>({current:v}),useEffect(){}};
  const controls = factory(hooks).Controls({client:null,settings,onChange:value=>{changed=value;}});
  controls.props.onCollapse();
  assert.equal(changed.scene.rtabmapGraphControlsCollapsed,true);
  assert.equal(changed.view,settings.view);
  assert.equal(changed.topics,settings.topics);
  assert.equal(changed.scene.rtabmapTopologyMode,'navigation');
  const switched = setMode(changed,'pose_graph');
  assert.equal(switched.scene.rtabmapGraphControlsCollapsed,true);
  const restored = factory(hooks).Controls({client:null,settings:changed,onChange:value=>{changed=value;}});
  assert.equal(restored.props.collapsed,true);
  restored.props.onCollapse();
  assert.equal(changed.scene.rtabmapGraphControlsCollapsed,false);
});

test('repairs saved map scenes with missing tags even after layout migration', () => {
  const maps = ['topology', 'features'].map(kind => {
    const scene = widget(kind);
    delete scene.props.scene.rtabmapGraphMap;
    scene.props.view.distance = 7;
    return scene;
  });
  const before = {widgets: [{...primary, rtabmapGraphFeatureLayoutVersion: 1}, ...maps, camera],
    mosaic: {direction: 'row', first: maps[0].i, second: maps[1].i}};
  const after = migrate(before);
  assert.equal(after.widgets.length, before.widgets.length);
  assert.equal(after.mosaic, before.mosaic);
  assert.equal(after.widgets[0], before.widgets[0]);
  assert.equal(after.widgets[3], camera);
  for (let i = 0; i < maps.length; i++) {
    const kind = i === 0 ? 'topology' : 'features';
    assert.equal(kindOf(maps[i].props), kind);
    assert.equal(after.widgets[i + 1].props.scene.rtabmapGraphMap, kind);
    assert.equal(after.widgets[i + 1].props.view, maps[i].props.view);
    assert.equal(after.widgets[i + 1].props.topics, maps[i].props.topics);
  }
  assert.equal(migrate(after), after);
  assert.equal(kindOf(primary.props), null);
  assert.equal(kindOf({topics: {'/miti/topological_map': {visible: false}}}), null);
  assert.equal(kindOf({topics: {'/miti/topological_map': {visible: true},
    '/miti/sparse_features': {visible: true}}}), null);
});

test('project layout includes all four maps with separate data topics', () => {
  const layout=JSON.parse(fs.readFileSync(path.join(root,'config/miti_live_layout.json')));
  for(const kind of ['topology','features']) {
    const panel=layout.widgets.find(w=>w.props?.scene?.rtabmapGraphMap===kind);
    assert.deepEqual(panel.props,JSON.parse(JSON.stringify(widget(kind).props)));
    assert.equal(panel.props.topics['/cloud_map'],undefined);
    assert.equal(panel.props.topics['/miti/walls'],undefined);
  }
  assert.equal(new Set(leaves(layout.mosaic)).size,layout.widgets.length);
});

test('parses counts and bounds, renders the legend and explicitly explains missing features', () => {
  assert.equal(ui.parse({data:JSON.stringify(snapshot)}).features.points,1500);
  for(const bad of [{}, {...snapshot,schema:2}, {...snapshot,features:{points:NaN}},
    {...snapshot,topology:{...snapshot.topology,bounds:{center:[0,0],span:[1,1,1]}}}]) {
    assert.equal(ui.parse({data:JSON.stringify(bad)}),null);
  }
  const html=renderToStaticMarkup(React.createElement(ui.View,{kind:'topology',data:snapshot,connected:true}));
  for(const label of ['12 个关键帧','14 条约束','相邻约束','全局回环','邻近回环','适应地图']) assert.ok(html.includes(label),label);
  const empty=renderToStaticMarkup(React.createElement(ui.View,{kind:'features',data:{...snapshot,features:{points:0,reason:'等待 RGB-D 关键帧'}},connected:true}));
  assert.match(empty,/等待 RGB-D 关键帧/);
  const stale=renderToStaticMarkup(React.createElement(ui.View,{kind:'features',data:snapshot,connected:true,stale:true}));
  assert.match(stale,/更新已中断/);
});

test('fit keeps metric bounds visible in narrow top-down and 3D panels', () => {
  for(const topDown of [true,false]) {
    const view=ui.fitView({fovy:45},snapshot.topology.bounds,.5,topDown);
    const height=2*view.distance*Math.tan(Math.PI/8);
    assert.ok(height>8);
    assert.ok(height*.5>10);
    assert.equal(view.rtabmapTopDown,topDown);
    assert.equal(view.perspective,!topDown);
  }
});

test('information bars subscribe only to status, preserving retained maps for native renderers', () => {
  const effects=[],subscribed=[],removed=[];
  const hooks={createElement:React.createElement,useState:v=>[v,()=>{}],useRef:v=>({current:v}),useEffect:fn=>effects.push(fn)};
  const client={subscribe:(topic,callback)=>subscribed.push({topic,callback}),unsubscribe:(topic,callback)=>removed.push({topic,callback})};
  factory(hooks).Controls({client,settings:widget('features').props,onChange(){}});
  const cleanups=effects.map(fn=>fn());
  assert.deepEqual(subscribed.map(x=>x.topic),['/miti/graph_feature_status']);
  cleanups.forEach(fn=>fn?.());
  assert.equal(removed[0].callback,subscribed[0].callback);
});
