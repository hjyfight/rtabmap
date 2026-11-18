const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const root=path.resolve(__dirname,'..'),context=vm.createContext({});
vm.runInContext(fs.readFileSync(path.join(root,'config/miti_workspace_sync.js'),'utf8'),context);
const settings=JSON.parse(fs.readFileSync(path.join(root,'config/miti_workspace_sync.json')));
settings.layout=JSON.parse(fs.readFileSync(path.join(root,'config',settings.defaultLayout)));
settings.mapWidget=JSON.parse(fs.readFileSync(path.join(root,'config',settings.mapLayout))).widgets.find(w=>w.i===settings.mapWidgetId);
function storage(initial={}){const values=new Map(Object.entries(initial));return {values,getItem:key=>values.get(key)??null,
  setItem:(key,value)=>values.set(key,value),removeItem:key=>values.delete(key)};}
const target={port:'3002',hostname:'localhost',protocol:'http:'};
function leaves(tree){return typeof tree==='string'?[tree]:[...leaves(tree.first),...leaves(tree.second)];}
test('each empty browser receives the same map, fusion and sound layout on port 3002',()=>{
  const first=storage(),second=storage();
  for(const local of [first,second]){
    const result=context.rtabmapSyncWorkspaceStorage(local,target,settings);assert.equal(result.applied,true);
    const layout=JSON.parse(local.getItem('miti_layout_config'));
    assert.ok(layout.widgets.some(w=>w.type==='audio-events'));
    assert.ok(layout.widgets.some(w=>w.i==='rtabmap-scene'&&w.props.frame.fixedFrame==='map'));
    assert.ok(layout.widgets.some(w=>w.type==='fourmodal-features'));
    const visible=leaves(layout.mosaic);assert.ok(visible.includes(result.mapId));assert.ok(visible.includes(result.audioId));
    assert.equal(JSON.parse(local.getItem('miti_bridge_url')),'ws://localhost:8768');
  }
  assert.equal(first.getItem('miti_layout_config'),second.getItem('miti_layout_config'));
});
test('saved fusion panels and camera settings survive the one-time synchronization',()=>{
  const layout=JSON.parse(JSON.stringify(settings.layout));layout.widgets.find(w=>w.type==='camera-viewer').props.maxFps=7;
  const local=storage({miti_layout_config:JSON.stringify(layout),miti_bridge_url:'"ws://localhost:8765"',miti_stream_mode:'"latest"'});
  const result=context.rtabmapSyncWorkspaceStorage(local,target,settings),updated=JSON.parse(local.getItem('miti_layout_config'));
  assert.equal(updated.widgets.find(w=>w.type==='camera-viewer').props.maxFps,7);
  assert.equal(local.getItem('miti_stream_mode'),'"latest"');assert.ok(local.getItem(result.backup));
  const original=JSON.parse(local.getItem(result.backup));assert.equal(original.miti_layout_config,JSON.stringify(layout));
  assert.equal(context.rtabmapSyncWorkspaceStorage(local,target,settings).applied,false);
});
test('vendor default layouts gain a real map and sound while other ports stay unchanged',()=>{
  const vendor={widgets:[{i:'urdf-viewer-1',type:'urdf-viewer'},{i:'camera-viewer-1',type:'camera-viewer'}],
    mosaic:{direction:'row',first:'urdf-viewer-1',second:'camera-viewer-1'}};
  const local=storage({miti_layout_config:JSON.stringify(vendor)});
  context.rtabmapSyncWorkspaceStorage(local,target,settings);
  assert.equal(JSON.parse(local.getItem('miti_layout_config')).widgets.some(w=>w.i==='urdf-viewer-1'),false);
  const other=storage({miti_layout_config:JSON.stringify(vendor)});
  assert.equal(context.rtabmapSyncWorkspaceStorage(other,{...target,port:'3004'},settings).applied,false);
  assert.equal(other.getItem('miti_layout_config'),JSON.stringify(vendor));
});
test('an existing map keeps its view, and hidden sound panels become visible',()=>{
  const map=JSON.parse(JSON.stringify(settings.mapWidget));map.props.frame.fixedFrame='livox_frame';map.props.view.distance=23;
  const layout={widgets:[map,{i:'audio',type:'audio-events'}],mosaic:map.i};
  const local=storage({miti_layout_config:JSON.stringify(layout)});
  context.rtabmapSyncWorkspaceStorage(local,target,settings);const updated=JSON.parse(local.getItem('miti_layout_config'));
  const actual=updated.widgets.find(w=>w.i===map.i);assert.equal(actual.props.view.distance,23);assert.equal(actual.props.frame.fixedFrame,'map');
  assert.ok(leaves(updated.mosaic).includes('audio'));
});
test('storage write failure restores the prior layout and bridge',()=>{
  const initial={miti_layout_config:JSON.stringify(settings.layout),miti_bridge_url:'"ws://old:8765"'};
  const local=storage(initial),write=local.setItem;let failed=false;
  local.setItem=(key,value)=>{if(key==='miti_bridge_url'&&!failed){failed=true;throw new Error('quota');}return write(key,value);};
  assert.throws(()=>context.rtabmapSyncWorkspaceStorage(local,target,settings),/quota/);
  for(const [key,value] of Object.entries(initial))assert.equal(local.getItem(key),value);
  assert.equal(local.getItem('rtabmap_workspace_sync_3002'),null);
});
