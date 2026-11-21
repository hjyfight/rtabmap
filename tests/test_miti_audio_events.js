const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.resolve(__dirname,'..'),vendor=require('node:module').createRequire(path.join(root,'.tools/miti/root/usr/lib/miti/package.json'));
const React=vendor('react'),{renderToStaticMarkup}=vendor('react-dom/server.browser');
const context=vm.createContext({});vm.runInContext(fs.readFileSync(path.join(root,'config/miti_audio_events.js'),'utf8'),context);
const panel=context.rtabmapAudioEventsFactory(React);
function snapshot(){
  const scores=Array(521).fill(0);scores[0]=.8;
  return {schema:'yamnet_audio_events/1',mode:'live',sequence:1,stamp:100,published_at:100,
    model:{name:'YAMNet',classes:521,sample_rate_hz:16000,score_kind:'independent_class_scores'},scores,
    top_k:[{index:0,mid:'/m/09x0r',label:'Speech',label_zh:'人声',score:.8}],processing_s:.02,
    audio:{rms_dbfs:-30,duration_s:2},source:{source_id:'microphone:1',stamp_quality:'host_read_end_estimate'},
    input_age_s:.1,source_localization:{available:false,target_id:null,position_m:null},
    patrol_events:[{key:'speech',label_zh:'听到人声',score:.8,threshold:.35,status:'confirmed',
      consecutive_windows:2,event_id:'audio-000001',onset:true,source_class:'Speech',observation:'sound_event',stamp:100}]};
}
test('live audio panel renders class scores and confirmed patrol observations',()=>{
  const v=panel.parseSnapshot({data:JSON.stringify(snapshot())});assert.ok(v);
  const html=renderToStaticMarkup(React.createElement(panel.View,{snapshot:v,history:v.patrol_events,receivedAt:100000,now:100100}));
  assert.match(html,/声音识别已更新/);assert.match(html,/521 类/);assert.match(html,/人声/);assert.match(html,/0.800/);
  assert.match(html,/最近巡逻记录/);assert.match(html,/听到人声/);assert.match(html,/模型分数/);
});
test('old retained messages cannot be displayed as current audio',()=>{
  const html=renderToStaticMarkup(React.createElement(panel.View,{snapshot:snapshot(),receivedAt:100000,now:110000}));
  assert.match(html,/音频已中断/);assert.match(html,/上一帧声音候选/);assert.doesNotMatch(html,/声音识别已更新/);
  const replay=renderToStaticMarkup(React.createElement(panel.View,{snapshot:snapshot(),receivedAt:110000,now:110001}));
  assert.match(replay,/音频已中断/);
});
test('sound candidates follow live ranking without fixed animal categories or invented events',()=>{
  const v=snapshot();v.scores[78]=.27;v.scores[70]=.21;v.patrol_events=[];
  const original=JSON.stringify(v);
  const html=renderToStaticMarkup(React.createElement(panel.View,{snapshot:v,receivedAt:100000,now:100100}));
  assert.match(html,/当前声音候选/);assert.match(html,/人声/);assert.match(html,/0.800/);
  assert.doesNotMatch(html,/猫狗|猫叫／呼噜|犬吠／低吼|0.270|0.210/);assert.match(html,/前 1 名/);
  assert.match(html,/尚无已确认的声音事件/);assert.equal(JSON.stringify(v),original);
  const stopped=renderToStaticMarkup(React.createElement(panel.View,{snapshot:v,receivedAt:100000,now:105000}));
  assert.match(stopped,/上一帧声音候选/);assert.doesNotMatch(stopped,/当前声音候选/);
});
test('remote heartbeat handles clock skew while rejecting old results and stopped streams',()=>{
  for(const skew of [-60,60]){
    const v=snapshot();v.source.frame_id='orin_microphone_array';v.published_at=100+skew;
    const status={state:'recognizing',published_at:v.published_at+.2,last_result_at:v.published_at};
    const props={snapshot:v,status,receivedAt:100000,statusReceivedAt:100000,now:100100};
    const render=p=>renderToStaticMarkup(React.createElement(panel.View,p));
    assert.match(render(props),/声音识别已更新/);assert.match(render(props),/Orin 实时麦克风/);
    assert.match(render({...props,now:105000}),/音频已中断/);
    assert.match(render({...props,statusReceivedAt:95000}),/音频已中断/);
    assert.match(render({...props,snapshot:{...v,published_at:v.published_at-10}}),/音频已中断/);
    assert.match(render({...props,snapshot:{...v,input_age_s:10}}),/音频已中断/);
  }
});
test('malformed classes, inconsistent scores and invented locations are rejected',()=>{
  for(const mutate of [v=>v.scores.pop(),v=>v.scores[0]=NaN,v=>v.top_k[0].score=.4,
    v=>v.top_k.push(v.top_k[0]),v=>v.source_localization.available=true,
    v=>v.patrol_events[0].event_id=null,v=>v.patrol_events[0].score=.1,v=>v.sequence=0]){
    const v=snapshot();mutate(v);assert.equal(panel.parseSnapshot({data:JSON.stringify(v)}),null);
  }
});
test('offline recordings and loading/error status have explicit banners',()=>{
  const v=snapshot();v.mode='offline';const recording=renderToStaticMarkup(React.createElement(panel.View,{snapshot:v}));
  assert.match(recording,/离线录音结果/);assert.match(recording,/录音声音候选/);assert.doesNotMatch(recording,/当前声音候选/);
  const html=renderToStaticMarkup(React.createElement(panel.View,{now:100,statusReceivedAt:99,
    status:{state:'model_error',last_error:'missing weights'}}));assert.match(html,/声音识别异常/);assert.match(html,/missing weights/);
  assert.equal(panel.parseStatus({data:JSON.stringify({schema:'yamnet_audio_events/1',classes:521,state:'waiting_audio',published_at:100,last_error:''})}).state,'waiting_audio');
});
test('project layouts gain one audio panel and unrelated layouts are preserved',()=>{
  const layout={widgets:[{i:'rtabmap-scene',type:'urdf-viewer'}],mosaic:'rtabmap-scene'};
  const upgraded=context.rtabmapMigrateAudioEventsLayout(layout);
  assert.equal(layout.widgets.length,1);assert.equal(upgraded.widgets.length,2);
  assert.equal(context.rtabmapMigrateAudioEventsLayout(upgraded),upgraded);
  const unrelated={widgets:[{i:'user-map',type:'urdf-viewer'}],mosaic:'user-map'};
  assert.equal(context.rtabmapMigrateAudioEventsLayout(unrelated),unrelated);
});
test('direction heartbeats cannot make stale or disconnected bearings look live',()=>{
  const direction={schema:'m260c_sound_direction/1',sequence:1,published_at:100,state:'tracking',
    angle_source:'speech_wakeup',angle_deg:115,age_s:1,calibration_verified:false,
    last_error:'',distance_m:null,target_id:null};
  assert.ok(panel.parseDirection({data:JSON.stringify(direction)}));
  const render=extra=>renderToStaticMarkup(React.createElement(panel.DirectionView,{
    direction,directionReceivedAt:100000,now:100100,...extra}));
  assert.match(render({}),/阵列方向 115 度/);
  assert.match(render({}),/地图方向待标定/);
  assert.doesNotMatch(render({direction:{...direction,age_s:10}}),/阵列方向 115 度/);
  assert.doesNotMatch(render({now:104000}),/阵列方向 115 度/);
  assert.equal(panel.parseDirection({data:JSON.stringify({...direction,angle_deg:360})}),null);
  assert.equal(panel.parseDirection({data:JSON.stringify({...direction,angle_source:'manual_beam'})}),null);
  assert.equal(panel.parseDirection({data:JSON.stringify({...direction,distance_m:2})}),null);
});
