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
  // Indices follow the pinned official 521-class map, including classes outside Top-5.
  const animalGroups=[
    {key:'cat',label:'猫叫／呼噜',indices:[76,78,77,80]},
    {key:'dog',label:'犬吠／低吼',indices:[70,69,73,74,72]}
  ];
  function animalScores(snapshot) {
    return animalGroups.map(group=>({...group,
      score:Math.max(...group.indices.map(i=>snapshot.scores[i]))}));
  }
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
  function View({snapshot=null,history=[],status=null,error='',now=Date.now(),receivedAt=0,statusReceivedAt=0}) {
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
    return h('div',{style:{padding:12,overflow:'auto',height:'100%',background:'#101923',color:'#dce7f0',fontSize:13}},
      h('h3',null,'哨兵巡逻 · 声音事件'),
      h('p',{role:'status',style:{color:fresh?'#70ddc4':'#ffc577'}},banner),
      h('p',null,'YAMNet · 521 类声音 · '+
        (snapshot?.source?.frame_id==='orin_microphone_array'?'Orin 实时麦克风':'实时麦克风')),
      error||failed?h('p',null,error||currentStatus.last_error):null,
      snapshot?h(React.Fragment,null,
        h('p',null,'音量 '+snapshot.audio.rms_dbfs.toFixed(1)+' dBFS · 分析窗口 '+snapshot.audio.duration_s.toFixed(2)+
          ' s · 计算 '+fmt(snapshot.processing_s)+' s'),
        h('h4',null,offline?'猫狗录音分数':fresh?'猫狗实时分数':'猫狗上一帧分数'),
        h('table',{'aria-label':'猫狗声音分数',style:{width:'100%',opacity:fresh?1:.55}},
          h('thead',null,h('tr',null,h('th',null,'声音类别'),h('th',null,'模型分数'))),
          h('tbody',null,...animalScores(snapshot).map(row=>h('tr',{key:row.key},
            h('td',null,row.label),h('td',null,fmt(row.score)))))),
        h('small',null,offline?'录音模型分数；确认事件见下方巡逻记录。':
          '分数持续更新；连续达到阈值后进入下方巡逻记录。'),
        h('p',null,h('strong',null,fresh?'本帧声音候选（前 '+snapshot.top_k.length+' 名）':'上一帧声音候选')),
        h('table',{style:{width:'100%',opacity:fresh?1:.55}},
          h('thead',null,h('tr',null,h('th',null,'声音类别'),h('th',null,'模型分数'))),
          h('tbody',null,...snapshot.top_k.map(row=>h('tr',{key:row.index},
            h('td',null,row.label_zh+(row.label_zh===row.label?'':' · '+row.label)),h('td',null,fmt(row.score)))))),
        h('p',null,snapshot.patrol_events.length?snapshot.patrol_events.map(event=>
          event.label_zh+'（'+(event.status==='confirmed'?'连续观测已确认':'待连续观测')+'）').join('；'):'本帧没有达到巡逻事件阈值'),
        h('small',null,'采集时间来源：'+snapshot.source.stamp_quality)):
        h('p',null,'说话、脚步、猫叫、犬吠、车辆、警报和撞击声会在收到音频后显示。'),
      h('h4',null,'最近巡逻记录'),
      history.length?h('ol',null,...history.slice(0,20).map(event=>h('li',{key:event.event_id+'-'+event.stamp},
        new Date(event.stamp*1000).toLocaleTimeString('zh-CN',{hour12:false})+' · '+event.label_zh+' · 分数 '+fmt(event.score)))):
        h('p',null,'尚无已确认的声音事件'),
      h('p',{style:{fontSize:12,color:'#9baec0'}},'模型分数分别对应各声音类别。此面板记录声音事件，声源位置与目标关联由其他模块提供。'));
  }
  function Component({client,topic='/perception/audio/events'}) {
    const [sample,setSample]=React.useState(null),[history,setHistory]=React.useState([]);
    const [status,setStatus]=React.useState(null),[error,setError]=React.useState('');
    const [now,setNow]=React.useState(Date.now());
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
      client.subscribe(topic,receive);client.subscribe('/perception/audio/status',heartbeat);
      return()=>{client.unsubscribe(topic,receive);client.unsubscribe('/perception/audio/status',heartbeat);};
    },[client,topic]);
    React.useEffect(()=>{const timer=setInterval(()=>setNow(Date.now()),1000);return()=>clearInterval(timer);},[]);
    return h(View,{snapshot:sample?.snapshot,receivedAt:sample?.receivedAt||0,history,
      status:status?.value,statusReceivedAt:status?.receivedAt||0,error,now});
  }
  return {Component,View,parseSnapshot,parseStatus,animalScores};
}
