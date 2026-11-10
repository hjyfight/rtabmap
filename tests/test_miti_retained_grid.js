/* Exercise the installed MITI client's real subscription/lifecycle methods. */
const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const root = path.resolve(__dirname,'..');
const page = fs.readFileSync(process.env.MITI_TEST_CLIENT_BUNDLE || path.join(root,
  '.tools/miti/root/usr/lib/miti/.next/static/chunks/app/page-a3758698d17bc097.js'),'utf8');
const begin = page.indexOf('function rtabmapRememberGridMessage(');
const end = page.indexOf('class S{connect(){return this.control.connect()}',begin);
assert.ok(begin>=0 && end>begin,'client cache helpers must be inside the client webpack module');
const method = (start,next) => {
  const first=page.indexOf(start,end), last=page.indexOf(next,first);
  assert.ok(first>=end && last>first,`${start} method exists`);
  return page.slice(first,last);
};
const methods = vm.runInNewContext(page.slice(begin,end)+'\n({'+[
  method('subscribe(e,t,r,n){','unsubscribe(e,t){'),
  method('unsubscribe(e,t){','advertise(e,t){'),
  method('deliver(e,t,r){','workers(){'),
  method('unbind(e){','bindAll(){'),
  method('handleClosed(e,t){','handleUnadvertise(e,t){'),
  method('handleUnadvertise(e,t){','handleMessage(e,t,r){'),
].join(',')+'})',{clearTimeout});
const grid = {info:{width:3,height:2,resolution:.05},data:new Int8Array([-1,0,100,100,0,-1])};
function fixture() {
  const lane={subscriptionTopics:new Map(),unsubscribe(){}};
  const client={...methods,control:lane,connected:true,subscriptions:new Map(),
    retainedTfTopics:new Set(),staticTransformCache:new Map(),lanes:new Map(),
    services:new Map(),clientChannels:new Map(),serviceCalls:new Map(),parameterRequests:new Map(),
    assetRequests:new Map(),assetQueue:[],assetCache:new Map(),disconnectionCallbacks:new Set(),
    emitTopics(){},bindAll(){},retainTransforms(){},binds:0,
    bind(topic){const s=this.subscriptions.get(topic);if(!s.binding){this.binds++;
      s.binding={lane,channelId:this.binds,subscriptionId:this.binds};}},
    send(topic,message){this.deliver(topic,this.subscriptions.get(topic),message);}};
  return client;
}

test('clicking navigation after the 2D panel gets a static grid immediately',()=>{
  const client=fixture(), primary=[], navigation=[];
  const a=m=>primary.push(m),b=m=>navigation.push(m);
  client.subscribe('/miti/grid_map',a);
  client.send('/miti/grid_map',grid);
  client.subscribe('/miti/grid_map',b);
  assert.equal(client.binds,1,'share the existing ROS subscription');
  assert.deepEqual(primary,[grid]);
  assert.deepEqual(navigation,[grid],'no second map update or robot movement needed');
  client.unsubscribe('/miti/grid_map',b);
  client.subscribe('/miti/grid_map',b);
  assert.deepEqual(navigation,[grid,grid]);
});

test('a reset replaces the old grid and last unsubscribe releases it',()=>{
  const client=fixture(),a=()=>{};
  client.subscribe('/miti/grid_map',a);client.send('/miti/grid_map',grid);
  const reset={info:{width:1,height:1,resolution:.05},data:[-1]};
  client.send('/miti/grid_map',reset);
  let shown;
  const b=m=>{shown=m;};client.subscribe('/miti/grid_map',b);
  assert.equal(shown,reset);
  const previous=client.subscriptions.get('/miti/grid_map');
  client.unsubscribe('/miti/grid_map',a);client.unsubscribe('/miti/grid_map',b);
  assert.equal(previous.rtabmapGridMessage,undefined);
  assert.equal(client.subscriptions.has('/miti/grid_map'),false);
  shown=undefined;client.subscribe('/miti/grid_map',b);
  assert.equal(shown,undefined,'a fresh connection must get a fresh snapshot');
});

test('invalid grids remove cache and unrelated large sensor streams are not retained',()=>{
  const client=fixture();client.subscribe('/miti/grid_map',()=>{});client.send('/miti/grid_map',grid);
  client.send('/miti/grid_map',{info:grid.info,data:[0]});
  assert.equal(client.subscriptions.get('/miti/grid_map').rtabmapGridMessage,undefined);
  for(const topic of ['/cloud_map','/camera/color/image_raw','/miti/navigation_map']) {
    client.subscribe(topic,()=>{});client.send(topic,grid);
    assert.equal(client.subscriptions.get(topic).rtabmapGridMessage,undefined);
  }
});

test('channel withdrawal and both data/control disconnects invalidate retained grids',()=>{
  for(const event of ['unadvertise','control-close','data-close','unbind']) {
    const client=fixture();client.subscribe('/miti/grid_map',()=>{});client.send('/miti/grid_map',grid);
    const sub=client.subscriptions.get('/miti/grid_map'),binding=sub.binding;
    if(event==='unadvertise')client.handleUnadvertise(binding.lane,[binding.channelId]);
    if(event==='control-close')client.handleClosed(client.control,true);
    if(event==='data-close'){sub.binding.lane={};client.handleClosed(sub.binding.lane,true);}
    if(event==='unbind')client.unbind(sub);
    assert.equal(sub.rtabmapGridMessage,undefined,event);
    let replayed=false;client.subscribe('/miti/grid_map',()=>{replayed=true;});
    assert.equal(replayed,false,event+' must not replay stale map');
  }
});
