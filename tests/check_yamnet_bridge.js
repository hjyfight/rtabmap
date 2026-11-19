/* Read-only bridge check and render of actual live YAMNet messages. */
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const {createRequire}=require('node:module'),root=path.resolve(__dirname,'..');
const vendor=createRequire(path.join(root,'.tools/miti/root/usr/lib/miti/package.json'));
const WS=vendor('next/dist/compiled/ws'),React=vendor('react'),{renderToStaticMarkup}=vendor('react-dom/server.browser');
const output=process.argv[2]||path.join(root,'logs/audio/yamnet/bridge_check.json');
const web=process.argv[3]||'http://127.0.0.1:3004',bridge=process.argv[4]||'ws://127.0.0.1:8768';
const expectedFrame=process.argv[5];

async function read(url,json=false){const response=await fetch(url,{signal:AbortSignal.timeout(5000)});assert.ok(response.ok,url);return json?response.json():response.text();}
async function main(){
  const entry=await read(web+'/rtabmap.html'),layout=await read(web+'/rtabmap-audio-layout.json',true);
  assert.match(entry,/打开哨兵巡逻声音识别/);
  assert.ok(layout.widgets.some(w=>w.type==='audio-events'&&w.props.topic==='/perception/audio/events'));
  const manifest=await read(web+'/rtabmap-panel-assets.json',true);
  const renderer=await read(web+manifest.assets[0]),page=await read(web+manifest.assets[1]);
  assert.match(renderer,/const RtabmapAudioEventsPanel = rtabmapAudioEventsFactory/);
  assert.match(page,/rtabmapMigrateAudioEventsLayout/);
  assert.ok(renderer.includes(fs.readFileSync(path.join(root,'config/miti_audio_events.js'),'utf8')),
    'Served audio panel differs from the current project source');
  const panelStart=renderer.indexOf('function rtabmapAudioEventsFactory(');
  const panelEnd=renderer.indexOf('\nconst RtabmapAudioEventsPanel',panelStart);
  assert.ok(panelStart>=0&&panelEnd>panelStart);
  const context=vm.createContext({});vm.runInContext(renderer.slice(panelStart,panelEnd),context);
  const panel=context.rtabmapAudioEventsFactory(React);
  let workspace=null;
  if(new URL(web).port==='3002'){
    const settingsMatch=page.match(/const RtabmapWorkspaceSettings = (\{[^\n]+\});/);
    assert.ok(settingsMatch,'Served page lacks workspace synchronization settings');
    const settings=JSON.parse(settingsMatch[1]);
    const start=page.indexOf('function rtabmapSyncWorkspaceStorage('),end=page.indexOf('\nconst RtabmapWorkspaceSettings',start);
    assert.ok(start>=0&&end>start);const runtime=vm.createContext({});vm.runInContext(page.slice(start,end),runtime);
    const values=new Map(),storage={getItem:key=>values.get(key)??null,setItem:(key,value)=>values.set(key,value),removeItem:key=>values.delete(key)};
    workspace=runtime.rtabmapSyncWorkspaceStorage(storage,{port:'3002',hostname:'localhost',protocol:'http:'},settings);
    assert.equal(workspace.applied,true);assert.equal(workspace.bridge,'ws://localhost:8768');
    const synced=JSON.parse(values.get('miti_layout_config'));
    assert.equal(synced.widgets.find(w=>w.i===workspace.mapId).props.frame.fixedFrame,'map');
    assert.equal(synced.widgets.find(w=>w.i===workspace.audioId).type,'audio-events');
    workspace={verified:true,bridge:workspace.bridge,fixed_frame:'map',audio_panel_id:workspace.audioId};
  }
  const received=await new Promise((resolve,reject)=>{
    const socket=new WS(bridge,['foxglove.sdk.v1','foxglove.websocket.v1']);
    const subscriptions=new Map(),results=[];let status=null;
    const timer=setTimeout(()=>{socket.terminate();reject(new Error('Missing live audio bridge messages'));},10000);
    const fail=error=>{clearTimeout(timer);socket.terminate();reject(error);};
    socket.on('error',fail);
    socket.on('message',(data,binary)=>{
      try{
        if(!binary){
          const message=JSON.parse(data.toString());if(message.op!=='advertise')return;
          const added=[];
          for(const channel of message.channels){
            if(!['/perception/audio/events','/perception/audio/status'].includes(channel.topic)||
              [...subscriptions.values()].includes(channel.topic))continue;
            assert.equal(channel.encoding,'cdr');
            const id=subscriptions.size+1;subscriptions.set(id,channel.topic);added.push({id,channelId:channel.id});
          }
          if(added.length)socket.send(JSON.stringify({op:'subscribe',subscriptions:added}));
          return;
        }
        if(data[0]!==1||data.length<21)return;
        const topic=subscriptions.get(data.readUInt32LE(1)),payload=data.subarray(13);
        const length=payload[1]===1?payload.readUInt32LE(4):payload.readUInt32BE(4);
        assert.ok(length>0&&length<=payload.length-8);
        const message={data:payload.subarray(8,8+length-1).toString('utf8')};
        if(topic==='/perception/audio/events'){
          const parsed=panel.parseSnapshot(message);assert.ok(parsed,'Actual bridge frame rejected by the audio panel');results.push(parsed);
        }else if(topic==='/perception/audio/status')status=panel.parseStatus(message);
        if(results.length>=3&&status){clearTimeout(timer);socket.close();resolve({results,status});}
      }catch(error){fail(error);}
    });
  });
  const latest=received.results.at(-1),now=Date.now();
  if(expectedFrame)for(const result of received.results)assert.equal(result.source.frame_id,expectedFrame);
  const html=renderToStaticMarkup(React.createElement(panel.View,{snapshot:latest,receivedAt:now,now,
    status:received.status,statusReceivedAt:now,history:latest.patrol_events.filter(e=>e.onset)}));
  assert.match(html,/声音识别已更新/);assert.match(html,/521 类/);assert.ok(html.includes(latest.top_k[0].label_zh));
  assert.match(html,/猫狗实时分数/);assert.match(html,/猫叫／呼噜/);assert.match(html,/犬吠／低吼/);
  const animals=panel.animalScores(latest).map(({key,score})=>({key,score}));
  for(const row of animals)assert.ok(html.includes(row.score.toFixed(3)));
  if(expectedFrame==='orin_microphone_array')assert.match(html,/Orin 实时麦克风/);
  fs.mkdirSync(path.dirname(output),{recursive:true});
  fs.writeFileSync(path.join(path.dirname(output),'bridge_render.html'),html);
  const report={passed:true,web,bridge,publications:received.results.length,classes:latest.scores.length,
    microphone_source:latest.source.source_id,microphone_frame:latest.source.frame_id,
    top_k:latest.top_k,animal_scores:animals,processing_s:latest.processing_s,
    served_audio_panel_matches:true,
    published_at:latest.published_at,checked_at:now/1000,publisher_minus_host_s:latest.published_at-now/1000,
    latest_sequence:latest.sequence,workspace_sync:workspace,rendered_html:path.join(path.dirname(output),'bridge_render.html')};
  fs.writeFileSync(output,JSON.stringify(report,null,2)+'\n');console.log(JSON.stringify(report));
}
main().catch(error=>{console.error(error);process.exitCode=1;});
