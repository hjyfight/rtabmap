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
const source = fs.readFileSync(path.join(root, 'config/miti_grid_map.js'), 'utf8');
const {migrate, widget, factory} = vm.runInNewContext(source +
  '; ({migrate:rtabmapMigrateGridLayout,widget:rtabmapGridWidget,factory:rtabmapGridMapFactory})');
const ui = factory(React);
const primary = {i:'rtabmap-scene',type:'urdf-viewer',title:'我的三维地图',props:{
  scene:{heightClip:{enabled:true,height:2.25}},view:{phi:58},topics:{'/cloud_map':{visible:true}}}};
const camera = {i:'camera',type:'camera-viewer',title:'相机',props:{topic:'/camera/color/image_raw'}};
const leaves = tree => typeof tree === 'string' ? [tree] : [...leaves(tree.first),...leaves(tree.second)];

test('reuses the adjacent empty scene and preserves the existing map, mosaic and camera', () => {
  const empty = {i:'urdf-viewer-2',type:'urdf-viewer',title:'3D Scene',w:4,h:8,props:{
    version:2,topics:{'/cloud_map':{visible:false}},layers:[{type:'grid',visible:true}]}};
  const before = {widgets:[primary,empty,camera],mosaic:{direction:'row',first:primary.i,
    second:{direction:'row',first:empty.i,second:camera.i,splitPercentage:60}}};
  const after = migrate(before);
  assert.equal(after.widgets.length,3);
  assert.equal(after.mosaic,before.mosaic);
  assert.equal(after.widgets[0].props,primary.props);
  assert.equal(after.widgets[2],camera);
  assert.equal(after.widgets[1].i,empty.i);
  assert.equal(after.widgets[1].w,empty.w);
  assert.equal(after.widgets[1].props.topics['/miti/grid_map'].type,'map');
  assert.equal(after.widgets[1].props.scene.rtabmapGrid,true);
  assert.equal(empty.props.scene,undefined);
  assert.equal(migrate(after),after);
  const removed = {...after,widgets:after.widgets.filter(w=>w.i!==empty.i)};
  assert.equal(migrate(removed),removed,'deliberate removal must persist');
});

test('adds a second map without replacing configured or locked scenes', () => {
  for (const other of [camera,
    {i:'custom',type:'urdf-viewer',title:'3D Scene',props:{topics:{'/cloud_map':{visible:true}}}},
    {i:'locked',type:'urdf-viewer',title:'3D Scene',locked:true}]) {
    const before = {widgets:[primary,other],mosaic:{direction:'row',first:primary.i,second:other.i}};
    const after = migrate(before);
    assert.equal(after.widgets.length,3);
    assert.equal(after.widgets[1],other);
    assert.deepEqual(leaves(after.mosaic),[primary.i,'rtabmap-grid-scene',other.i]);
    assert.equal(after.widgets[0].props,primary.props);
  }
  const unrelated = {widgets:[camera],mosaic:camera.i};
  assert.equal(migrate(unrelated),unrelated);
});

test('project layout uses two native maps with a top-down occupancy grid and isolated display controls', () => {
  const layout = JSON.parse(fs.readFileSync(path.join(root,'config/miti_live_layout.json')));
  const grid = layout.widgets.find(w=>w.i==='rtabmap-grid-scene');
  assert.deepEqual(grid.props,JSON.parse(JSON.stringify(widget().props)));
  assert.equal(grid.props.view.perspective,false);
  assert.equal(grid.props.view.rtabmapTopDown,true);
  assert.equal(grid.props.topics['/miti/grid_map'].visible,true);
  assert.equal(grid.props.topics['/cloud_map'],undefined);
  assert.equal(grid.props.topics['/miti/walls'],undefined);
  assert.equal(layout.widgets[0].props.topics['/cloud_map'].visible,true);
  assert.deepEqual(leaves(layout.mosaic).slice(0,2),['rtabmap-scene','rtabmap-grid-scene']);
  assert.equal(new Set(leaves(layout.mosaic)).size,layout.widgets.length);
});

test('upgrades the existing second map source once while preserving layout, zoom and custom colors', () => {
  const old = {i:'user-second-map',type:'urdf-viewer',title:'二维栅格地图',props:{
    scene:{rtabmapGrid:true},view:{distance:17,targetOffset:[1,2,0]},
    topics:{'/map':{visible:true,type:'map',unknownColor:'#707070'},'/robot_pose':{visible:false}}}};
  const before = {widgets:[{...primary,rtabmapGridLayoutVersion:1},old,camera],mosaic:{
    direction:'row',first:primary.i,second:{direction:'row',first:old.i,second:camera.i}}};
  const after = migrate(before), changed = after.widgets[1];
  assert.equal(after.mosaic,before.mosaic);
  assert.equal(after.widgets[0],before.widgets[0]);
  assert.equal(changed.props.view,old.props.view);
  assert.equal(changed.props.topics['/map'].visible,false);
  assert.equal(changed.props.topics['/miti/grid_map'].visible,true);
  assert.equal(changed.props.topics['/miti/grid_map'].unknownColor,'#707070');
  assert.equal(changed.props.topics['/robot_pose'].visible,false);
  assert.equal(old.props.topics['/map'].visible,true);
  assert.equal(migrate(after),after);
  const html = renderToStaticMarkup(React.createElement(ui.View,{map:null,connected:true,
    status:{state:'ready',min_obstacle_height:.15,max_obstacle_height:1.6}}));
  assert.match(html,/距地面 0.15–1.60 m/);
});

test('map fitting respects metric resolution, translated and rotated origins, and narrow viewports', () => {
  const message = {header:{frame_id:'map'},info:{width:100,height:200,resolution:.05,
    origin:{position:{x:-4,y:3,z:0},orientation:{x:0,y:0,z:Math.SQRT1_2,w:Math.SQRT1_2}}}};
  const map = ui.metadata(message);
  assert.equal(map.width,5);
  assert.equal(map.height,10);
  assert.ok(Math.abs(map.center[0]+9)<1e-10);
  assert.ok(Math.abs(map.center[1]-5.5)<1e-10);
  const view = ui.fitView({fovy:45},map,.5);
  const visibleHeight = 2*view.distance*Math.tan(Math.PI/8);
  assert.ok(visibleHeight>map.span[1]);
  assert.ok(visibleHeight*.5>map.span[0]);
  assert.equal(view.targetOffset,map.center);
  const html = renderToStaticMarkup(React.createElement(ui.View,{map,connected:true}));
  for (const text of ['5.0 cm / 格','100 × 200','障碍','空闲','未知','适应地图','5.0 × 10.0 m']) assert.ok(html.includes(text),text);
  for (const info of [{...message.info,width:0},{...message.info,resolution:NaN},
    {...message.info,width:1e6,height:1e6},
    {...message.info,origin:{...message.info.origin,orientation:{x:0,y:0,z:0,w:0}}}]) {
    assert.equal(ui.metadata({...message,info}),null);
  }
});

test('refresh-only entry replaces cached assets before navigation and preserves user settings', async () => {
  const html = fs.readFileSync(path.join(root,'config/miti_live.html'),'utf8');
  const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
  const initial = {miti_layout_config:JSON.stringify({widgets:[primary,camera]}),
    miti_bridge_url:'"ws://my-robot:8765"',miti_stream_mode:'"latest"'};
  for (const failAsset of [false,true]) {
    const values = new Map(Object.entries(initial)), elements = new Map(), requests = [], navigations = [];
    const sandbox = {
      URLSearchParams,
      document:{getElementById:id=>{if(!elements.has(id)) elements.set(id,{addEventListener(){}});return elements.get(id);}},
      location:{search:'',assign:url=>navigations.push(url)},
      localStorage:{getItem:key=>values.get(key)??null,setItem:(key,value)=>values.set(key,value),removeItem:key=>values.delete(key)},
      fetch:async (url,options)=>{requests.push({url,options});return {
        ok:!(failAsset && url.endsWith('.js')),
        json:async()=>({version:'test-grid',assets:['/_next/static/chunks/grid.js']}),
        arrayBuffer:async()=>new ArrayBuffer(0)
      };}
    };
    vm.createContext(sandbox);
    await vm.runInContext(script+'; enter(false)',sandbox);
    for (const [key,value] of Object.entries(initial)) assert.equal(values.get(key),value);
    assert.equal(requests.some(r=>r.url==='/rtabmap-layout.json'),false);
    assert.equal(requests[1].options.cache,'reload');
    assert.equal(navigations.length,failAsset?0:1);
    if(failAsset) assert.match(elements.get('status').textContent,/显示未更新/);
    else assert.ok([...values.keys()].some(key=>key.startsWith('rtabmap_miti_backup_')));
  }
});

test('a static latched grid reaches the renderer even when its information bar mounts first', () => {
  // Exercise MITI's actual subscribe method: ordinary topics have no replay
  // cache for late callbacks sharing an already-bound ROS subscription.
  const page = fs.readFileSync(path.join(root,
    '.tools/miti/root/usr/lib/miti/rtabmap-panel-originals/.next/static/chunks/app/page-a3758698d17bc097.js'),'utf8');
  const begin = page.indexOf('subscribe(e,t,r,n){');
  const end = page.indexOf('unsubscribe(e,t){',begin);
  assert.ok(begin>0 && end>begin);
  const subscribe = vm.runInNewContext('({'+page.slice(begin,end)+'}).subscribe');
  const grid = {header:{frame_id:'map'},info:{width:3,height:2,resolution:.05,
    origin:{position:{x:-1,y:2,z:0},orientation:{x:0,y:0,z:0,w:1}}},data:[-1,0,100,100,0,-1]};
  const status = {schema:1,state:'ready',width:3,height:2,resolution:.05,
    origin:[-1,2,0],min_obstacle_height:.15,max_obstacle_height:1.6};
  const retained = {'/miti/grid_map':grid,'/miti/grid_status':{data:JSON.stringify(status)}};
  function clientFixture() {
    const bound = new Set(), pending = [];
    return {
      subscriptions:new Map(),retainedTfTopics:new Set(),staticTransformCache:new Map(),subscribe,
      bind(topic) {if(!bound.has(topic)){bound.add(topic);pending.push(topic);}},
      unsubscribe(topic, callback) {this.subscriptions.get(topic)?.callbacks.delete(callback);},
      flush() {while(pending.length){const topic=pending.shift();for(const callback of this.subscriptions.get(topic).callbacks.keys()) callback(retained[topic]);}}
    };
  }
  const previous = clientFixture();
  let lost;
  previous.subscribe('/miti/grid_map',()=>{});
  previous.flush();
  previous.subscribe('/miti/grid_map',message=>{lost=message;});
  previous.flush();
  assert.equal(lost,undefined,'upstream does not replay the retained grid to late callbacks');

  const effects = [], cleanups = [], client = clientFixture();
  const hooks = {createElement:React.createElement,
    useState:value=>[value,()=>{}],useRef:value=>({current:value}),useEffect:effect=>effects.push(effect)};
  const controls = factory(hooks);
  controls.Controls({client,settings:widget().props,onChange(){}});
  for(const effect of effects) cleanups.push(effect());
  client.flush();
  let rendered;
  client.subscribe('/miti/grid_map',message=>{rendered=message;},'nav_msgs/msg/OccupancyGrid');
  client.flush();
  assert.equal(rendered,grid,'native map renderer must receive the one retained grid without robot motion');
  assert.deepEqual(rendered.data,[-1,0,100,100,0,-1]);
  const a=ui.statusMetadata(status),b=ui.metadata(grid);
  assert.deepEqual(JSON.parse(JSON.stringify(a)),JSON.parse(JSON.stringify(b)));
  assert.equal(ui.statusMetadata({origin:[0,0]}),null);
  for(const cleanup of cleanups) cleanup?.();
});
