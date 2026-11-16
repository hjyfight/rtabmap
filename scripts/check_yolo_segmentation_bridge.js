/* Read-only Foxglove -> CDR -> native React panel verification. */
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..');
const vendor=require('node:module').createRequire(path.join(root,'.tools/miti/root/usr/lib/miti/package.json'));
const WS=vendor('next/dist/compiled/ws'),React=vendor('react');
const {renderToStaticMarkup}=vendor('react-dom/server.browser');
const factory=vm.runInNewContext(fs.readFileSync(path.join(root,'config/miti_segmentation.js'),'utf8')+';rtabmapSegmentationFactory');
const panel=factory(React),subscriptions=new Map(),received={};
const socket=new WS('ws://127.0.0.1:'+(process.argv[2]||8765),['foxglove.sdk.v1','foxglove.websocket.v1']);
const target=path.join(root,'logs/segmentation_validation');
const timer=setTimeout(()=>{console.error('Missing segmentation bridge data:',Object.keys(received));socket.terminate();process.exit(1);},25000);
socket.on('error',error=>{console.error(error);process.exit(1);});
socket.on('message',(data,binary)=>{
  if(!binary){
    const message=JSON.parse(data.toString());if(message.op!=='advertise')return;
    const added=[];
    for(const channel of message.channels){
      if(['/miti/segmentation/view','/miti/segmentation/status'].includes(channel.topic)){
        const id=subscriptions.size+1;subscriptions.set(id,channel.topic);added.push({id,channelId:channel.id});
      }
    }
    if(added.length)socket.send(JSON.stringify({op:'subscribe',subscriptions:added}));
    return;
  }
  if(data[0]!==1||data.length<21)return;
  const topic=subscriptions.get(data.readUInt32LE(1));if(!topic)return;
  const payload=data.subarray(13),little=Boolean(payload[1]&1);
  const length=little?payload.readUInt32LE(4):payload.readUInt32BE(4);
  assert.ok(length>0&&length<=payload.length-8);
  const message={data:payload.subarray(8,8+length-1).toString('utf8')};
  const key=topic.endsWith('/view')?'view':'status';
  received[key]=key==='view'?panel.parseSnapshot(message):panel.parseStatus(message);
  assert.ok(received[key],'Native panel rejected bridged '+key);
  if(received.view&&received.status&&received.view.session_id===received.status.session_id){
    const state=panel.deriveState({snapshot:received.view,receivedAt:0},{snapshot:received.status,receivedAt:0},0,true);
    assert.equal(state.stale,false,'Live segmentation is stale');
    const html=renderToStaticMarkup(React.createElement(panel.View,{sample:{snapshot:received.view},status:received.status,state}));
    assert.match(html,/data:image\/jpeg;base64,/);assert.match(html,/目标/);assert.match(html,/候选仅对比/);
    fs.mkdirSync(target,{recursive:true});
    fs.writeFileSync(path.join(target,'panel.html'),html);
    const report={passed:true,transport:'foxglove_cdr',native_parser:true,react_render:true,
      browser_visual_verified:false,sequence:received.view.sequence,instances:received.view.instances.length,state:state.label};
    fs.writeFileSync(path.join(target,'bridge.json'),JSON.stringify(report,null,2)+'\n');
    console.log(JSON.stringify(report));clearTimeout(timer);socket.close();process.exit(0);
  }
});
