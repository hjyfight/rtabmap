/* Live sound events and patrol observations using MITI's React instance. */
function rtabmapAudioEventsWidget(id = 'rtabmap-audio-events') {
  return {i:id,type:'audio-events',title:'哨兵巡逻 · 声音事件',x:0,y:36,w:8,h:10,
    props:{topic:'/perception/audio/events'}};
}

function rtabmapMigrateAudioEventsLayout(layout) {
  if(!layout||!Array.isArray(layout.widgets)||!layout.mosaic||
    layout.widgets.some(w=>w.type==='audio-events'))return layout;
  // Only extend project layouts, preserving unrelated user dashboards.
  if(!layout.widgets.some(w=>['rtabmap-scene','rtabmap-fourmodal','rtabmap-object-multimodal'].includes(w.i)))return layout;
  let id='rtabmap-audio-events',suffix=2;
  while(layout.widgets.some(w=>w.i===id))id='rtabmap-audio-events-'+suffix++;
  return {...layout,widgets:[...layout.widgets,rtabmapAudioEventsWidget(id)],
    mosaic:{direction:'column',first:layout.mosaic,second:id,splitPercentage:75}};
}

function rtabmapAudioEventsFactory(React) {
  'use strict';
  const h=React.createElement,finite=x=>typeof x==='number'&&Number.isFinite(x);
  const score=x=>finite(x)&&x>=0&&x<=1;
  function parseSnapshot(message) {
    try {
      if(typeof message?.data!=='string'||message.data.length>100000)return null;
      const v=JSON.parse(message.data);
      if(v?.schema!=='yamnet_audio_events/1'||!['live','offline'].includes(v.mode)||
        !Number.isSafeInteger(v.sequence)||v.sequence<1||!finite(v.stamp)||!finite(v.published_at)||
        v.model?.name!=='YAMNet'||v.model?.classes!==521||v.model?.sample_rate_hz!==16000||
        v.model?.score_kind!=='independent_class_scores'||
        !Array.isArray(v.scores)||v.scores.length!==521||!v.scores.every(score)||
        !Array.isArray(v.top_k)||!v.top_k.length||v.top_k.length>20||
        !finite(v.processing_s)||v.processing_s<0||!finite(v.audio?.rms_dbfs)||
        !finite(v.audio?.duration_s)||v.audio.duration_s<=0||
        typeof v.source?.source_id!=='string'||!v.source.source_id||
        typeof v.source?.stamp_quality!=='string'||
        (v.input_age_s!==null&&!finite(v.input_age_s))||
        v.source_localization?.available!==false||v.source_localization?.target_id!==null||
        v.source_localization?.position_m!==null||!Array.isArray(v.patrol_events)||v.patrol_events.length>32)return null;
      const ids=new Set();
      for(const row of v.top_k) {
        if(!Number.isInteger(row?.index)||row.index<0||row.index>=521||ids.has(row.index)||
          typeof row.mid!=='string'||typeof row.label!=='string'||!row.label||
          typeof row.label_zh!=='string'||!row.label_zh||!score(row.score)||
          Math.abs(row.score-v.scores[row.index])>1e-6)return null;
        ids.add(row.index);
      }
      const keys=new Set();
      for(const event of v.patrol_events) {
        if(typeof event?.key!=='string'||!event.key||keys.has(event.key)||
          typeof event.label_zh!=='string'||!event.label_zh||!score(event.score)||!score(event.threshold)||
          event.score<event.threshold||!['candidate','confirmed'].includes(event.status)||
          !Number.isSafeInteger(event.consecutive_windows)||event.consecutive_windows<1||
          typeof event.onset!=='boolean'||!finite(event.stamp)||
          typeof event.source_class!=='string'||event.observation!=='sound_event'||
          (event.status==='confirmed'?typeof event.event_id!=='string'||!event.event_id:event.event_id!==null)||
          (event.onset&&event.status!=='confirmed'))return null;
        keys.add(event.key);
      }
      return v;
    }catch(_){return null;}
  }
  function parseStatus(message) {
    try {
      const v=JSON.parse(message.data);
      return v?.schema==='yamnet_audio_events/1'&&v.classes===521&&
        ['loading_model','waiting_audio','recognizing','inferencing','model_error','inference_error'].includes(v.state)&&
        finite(v.published_at)&&typeof v.last_error==='string'?v:null;
    }catch(_){return null;}
  }
  const fmt=x=>finite(x)?x.toFixed(3):'—';
  const css=`
    .rtab-audio{height:100%;overflow:auto;min-width:0;padding:12px;background:#101b27;color:#dce8f2;font:12px/1.5 system-ui,sans-serif;container-type:inline-size}
    .rtab-audio *{box-sizing:border-box}.rtab-audio h4,.rtab-audio p{margin:0}.rtab-audio small{font-size:10px;color:#93aabf}
    .rtab-audio-header{display:flex;align-items:flex-start;justify-content:space-between;gap:10px;margin-bottom:10px}.rtab-audio-header b{font-size:14px}.rtab-audio-header small{display:block;margin-top:3px}
    .rtab-audio-status{max-width:55%;padding:4px 7px;border:1px solid #395264;border-radius:20px;font-size:10px}.rtab-audio-status.is-live{color:#79ddc8;border-color:#2c625c;background:#133833}.rtab-audio-status.is-waiting{color:#e1bd80;background:#342c1b}
    .rtab-audio-metrics{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-bottom:10px}.rtab-audio-metrics>div{padding:8px 10px;background:#182a3a;border-radius:8px}.rtab-audio-metrics b{display:block;font:600 17px/1.4 system-ui,sans-serif;font-variant-numeric:tabular-nums}.rtab-audio-metrics b span{font-size:10px;font-weight:400;color:#93aabf;margin-left:4px}
    .rtab-audio-body{display:grid;grid-template-columns:1fr 1fr;gap:9px;align-items:start}.rtab-audio-card{min-width:0;border:1px solid #2a4054;background:#142331;border-radius:9px;padding:10px}.rtab-audio-card h4{color:#c0d5e5;font-size:11px;font-weight:600;margin-bottom:9px}.rtab-audio-empty{color:#849db3;font-size:11px;padding:8px 0}
    .rtab-audio-rank{display:grid;grid-template-columns:minmax(0,1fr) 50px 36px;gap:7px;align-items:center;padding:5px 0}.rtab-audio-rank-label{overflow:hidden;white-space:nowrap;text-overflow:ellipsis}.rtab-audio-rank b{font:11px ui-monospace,monospace;color:#b7d3e8;text-align:right}.rtab-audio-track{height:4px;border-radius:4px;background:#2a4051;overflow:hidden}.rtab-audio-track i{display:block;height:100%;background:#61d2bd;border-radius:4px}
    .rtab-audio-history ol{list-style:none;padding:0;margin:0}.rtab-audio-history li{display:grid;grid-template-columns:54px 1fr 36px;gap:7px;padding:5px 0;border-bottom:1px solid #263c4e;font-size:10px}.rtab-audio-history li:last-child{border:0}.rtab-audio-history time{color:#819eb6}.rtab-audio-history b{font-weight:400;text-align:right;color:#78d4c2}
    .rtab-audio-foot{color:#93aabf;font-size:10px;padding-top:9px}.rtab-audio-alert{color:#f2ac98;padding:8px;background:#3b2425;border-radius:6px}
    .rtab-direction-content{display:flex;align-items:center;gap:10px}.rtab-direction-compass{width:88px;height:88px;flex:none}.rtab-direction-info b{font-size:23px;color:#79ddc8}.rtab-direction-info small{display:block;font-size:10px}.rtab-direction-note{color:#93aabf;font-size:10px;margin-top:7px!important}
    @container(max-width:480px){.rtab-audio-body{grid-template-columns:1fr}.rtab-audio-rank{grid-template-columns:minmax(0,1fr) 80px 40px}.rtab-direction-compass{width:70px;height:70px}.rtab-audio-header{flex-wrap:wrap}.rtab-audio-status{max-width:100%}}
  `;
  function parseDirection(message) {
    try {
      if(typeof message?.data!=='string'||message.data.length>20000)return null;
      const v=JSON.parse(message.data);
      if(v.schema!=='m260c_sound_direction/1'||v.angle_source!=='speech_wakeup'||
        !['waiting_wakeup','tracking','stale','serial_error','connecting','stopped'].includes(v.state)||
        !Number.isSafeInteger(v.sequence)||v.sequence<0||!finite(v.published_at)||
        (v.angle_deg!==null&&(!finite(v.angle_deg)||v.angle_deg<0||v.angle_deg>=360))||
        (v.age_s!==null&&(!finite(v.age_s)||v.age_s<0))||
        typeof v.calibration_verified!=='boolean'||typeof v.last_error!=='string'||
        v.distance_m!==null||v.target_id!==null)return null;
      return v;
    }catch(_){return null;}
  }
  function DirectionView({direction=null,directionReceivedAt=0,now=Date.now()}) {
    const connected=direction&&directionReceivedAt>0&&now-directionReceivedAt<3500;
    const age=finite(direction?.age_s)?direction.age_s+Math.max(0,now-directionReceivedAt)/1000:null;
    const active=connected&&direction.state==='tracking'&&age!==null&&age<5&&finite(direction.angle_deg);
    const labels={connecting:'正在连接阵列',waiting_wakeup:'等待语音唤醒',serial_error:'串口连接异常',stopped:'定位服务已停止',stale:'上次唤醒方向',tracking:'语音唤醒方向'};
    return h('div',{className:'rtab-audio-card','aria-label':'M260C 声源方位'},
      h('h4',null,'声源方位 · M260C 六麦'),
      h('div',{className:'rtab-direction-content'},
        h('svg',{className:'rtab-direction-compass',viewBox:'0 0 100 100',role:'img','aria-label':active?'阵列方向 '+direction.angle_deg+' 度':'等待有效声源方向'},
          h('circle',{cx:50,cy:50,r:35,fill:'#10202d',stroke:'#365064'}),
          ...[0,90,180,270].map(a=>h('text',{key:a,x:50+43*Math.sin(a*Math.PI/180),y:53-43*Math.cos(a*Math.PI/180),fill:'#819db4',fontSize:8,textAnchor:'middle'},a+'°')),
          active?h('g',{transform:'rotate('+direction.angle_deg+' 50 50)'},h('path',{d:'M50 50 L44 17 Q50 14 56 17 Z',fill:'#62d5be44'}),h('path',{d:'M50 48 L50 18 M46 23 L50 18 L54 23',stroke:'#79ddc8',strokeWidth:2,fill:'none'})):null,
          h('circle',{cx:50,cy:50,r:3,fill:active?'#79ddc8':'#68859b'})),
        h('div',{className:'rtab-direction-info'},h('b',null,active?direction.angle_deg.toFixed(0)+'°':'—'),
          h('small',null,!connected?'等待定位服务':labels[direction.state]),
          h('small',null,age!==null?'上次角度 '+age.toFixed(1)+' s 前':'360° 水平方位'))),
      h('p',{className:'rtab-direction-note'},direction?.last_error||
        '角度来自语音唤醒；环境声音方向需另行验证。'),
      h('small',null,direction?.calibration_verified?'安装朝向已配置':'阵列坐标 · 地图方向待标定'));
  }
  function View({snapshot=null,history=[],status=null,error='',now=Date.now(),receivedAt=0,statusReceivedAt=0,direction=null,directionReceivedAt=0}) {
    const offline=snapshot?.mode==='offline';
    const currentStatus=status&&now-statusReceivedAt<4000?status:null;
    // Results and heartbeat share the producer's clock, which can differ from this browser's.
    const producerFresh=currentStatus&&['recognizing','inferencing'].includes(currentStatus.state)&&
      finite(snapshot?.input_age_s)&&snapshot.input_age_s>=0&&snapshot.input_age_s<=3&&
      finite(currentStatus.published_at)&&finite(currentStatus.last_result_at)&&
      Math.abs(currentStatus.published_at-snapshot.published_at)<4&&
      Math.abs(currentStatus.last_result_at-snapshot.published_at)<4;
    const fresh=offline||receivedAt>0&&now-receivedAt<4000&&snapshot&&
      (producerFresh||Math.abs(now-snapshot.published_at*1000)<5000);
    const failed=['model_error','inference_error'].includes(currentStatus?.state);
    const banner=error?'声音数据校验失败':failed?'声音识别异常':offline?'离线录音结果':
      fresh?'声音识别已更新':currentStatus?.state==='loading_model'?'正在加载 YAMNet':
      snapshot?'音频已中断，等待实时输入':'等待实时音频';
    const table=(rows,label)=>h('div',{className:'rtab-audio-ranking','aria-label':label},...rows.map(row=>
      h('div',{className:'rtab-audio-rank',key:row.key||row.index},
        h('span',{className:'rtab-audio-rank-label',title:row.label},row.label_zh||row.label),
        h('span',{className:'rtab-audio-track','aria-hidden':true},h('i',{style:{width:(row.score*100).toFixed(1)+'%'}})),
        h('b',null,fmt(row.score)))));
    return h('section',{className:'rtab-audio','aria-label':'声学感知'},
      h('style',null,css),
      h('header',{className:'rtab-audio-header'},
        h('div',null,h('b',null,'声音事件'),h('small',null,'YAMNet · 521 类 · '+
          (snapshot?.source?.frame_id==='orin_microphone_array'?'Orin 实时麦克风':
            snapshot?.source?.frame_id==='host_microphone_array'?'主机 M260C':'实时麦克风'))),
        h('span',{role:'status',className:'rtab-audio-status '+(fresh?'is-live':'is-waiting')},banner)),
      error||failed?h('p',{className:'rtab-audio-alert'},error||currentStatus.last_error):null,
      snapshot?h('div',{className:'rtab-audio-metrics'},...[
        ['音量',snapshot.audio.rms_dbfs.toFixed(1),'dBFS'],
        ['分析窗口',snapshot.audio.duration_s.toFixed(2),'s'],
        ['计算耗时',fmt(snapshot.processing_s),'s']
      ].map(([label,value,unit])=>h('div',{key:label},h('small',null,label),h('b',null,value,h('span',null,unit))))) : null,
      h('div',{className:'rtab-audio-body'},
        h('div',{className:'rtab-audio-card'},
          h('h4',null,offline?'录音声音候选':fresh?'当前声音候选（前 '+snapshot.top_k.length+' 名）':'上一帧声音候选'),
          snapshot?table(snapshot.top_k,'声音候选模型分数'):h('p',{className:'rtab-audio-empty'},'等待音频输入'),
          h('small',null,'模型分数持续更新；连续达到阈值后记录声音事件。')),
        h(DirectionView,{direction,directionReceivedAt,now}),
        h('div',{className:'rtab-audio-card rtab-audio-history'},h('h4',null,'最近巡逻记录'),
          history.length?h('ol',null,...history.slice(0,8).map(event=>h('li',{key:event.event_id+'-'+event.stamp},
            h('time',null,new Date(event.stamp*1000).toLocaleTimeString('zh-CN',{hour12:false})),
            h('span',null,event.label_zh),h('b',null,fmt(event.score))))):h('p',{className:'rtab-audio-empty'},'尚无已确认的声音事件'))),
      snapshot?h('p',{className:'rtab-audio-foot'},snapshot.patrol_events.length?snapshot.patrol_events.map(event=>
        event.label_zh+'（'+(event.status==='confirmed'?'连续观测已确认':'待连续观测')+'）').join('；'):'本帧没有达到巡逻事件阈值'):null);
  }

  function Component({client,topic='/perception/audio/events'}) {
    const [sample,setSample]=React.useState(null),[history,setHistory]=React.useState([]);
    const [status,setStatus]=React.useState(null),[error,setError]=React.useState('');
    const [now,setNow]=React.useState(Date.now()),[bearing,setBearing]=React.useState(null);
    React.useEffect(()=>{
      if(!client)return;
      const receive=message=>{
        const snapshot=parseSnapshot(message);
        if(!snapshot){setError('收到不兼容的 YAMNet 消息');return;}
        setSample({snapshot,receivedAt:Date.now()});setError('');
        const onsets=snapshot.patrol_events.filter(e=>e.onset&&e.status==='confirmed');
        if(onsets.length)setHistory(previous=>{
          const added=onsets.filter(e=>!previous.some(p=>p.event_id===e.event_id&&p.stamp===e.stamp));
          return [...added.reverse(),...previous].slice(0,20);
        });
      };
      const heartbeat=message=>{const value=parseStatus(message);if(value)setStatus({value,receivedAt:Date.now()});};
      const locate=message=>{const value=parseDirection(message);if(value)setBearing({value,receivedAt:Date.now()});};
      client.subscribe(topic,receive);client.subscribe('/perception/audio/status',heartbeat);
      client.subscribe('/perception/audio/direction',locate);
      return()=>{client.unsubscribe(topic,receive);client.unsubscribe('/perception/audio/status',heartbeat);client.unsubscribe('/perception/audio/direction',locate);};
    },[client,topic]);
    React.useEffect(()=>{const timer=setInterval(()=>setNow(Date.now()),1000);return()=>clearInterval(timer);},[]);
    return h(View,{snapshot:sample?.snapshot,receivedAt:sample?.receivedAt||0,history,
      status:status?.value,statusReceivedAt:status?.receivedAt||0,direction:bearing?.value,directionReceivedAt:bearing?.receivedAt||0,error,now});
  }
  return {Component,View,parseSnapshot,parseStatus,parseDirection,DirectionView};
}
