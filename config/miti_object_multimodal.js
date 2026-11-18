/* Independent target fusion view. Existing RGB/NIR comparison stays separate. */
function rtabmapObjectMultimodalFactory(React) {
  'use strict';
  const h=React.createElement, order=['rgb','infrared','lidar','audio'];
  const names=['RGB','NIR','LiDAR','Audio'];
  const finite=x=>typeof x==='number'&&Number.isFinite(x);
  const vector=(x,n)=>Array.isArray(x)&&x.length===n&&x.every(finite);
  function parseSnapshot(message) {
    try {
      if(typeof message?.data!=='string'||message.data.length>6000000)return null;
      const v=JSON.parse(message.data);
      if(v?.schema!=='object_multimodal_fusion/1'||!Number.isSafeInteger(v.sequence)||v.sequence<0||
        !finite(v.stamp)||!finite(v.published_at)||typeof v.frame_id!=='string'||!v.frame_id||
        !Array.isArray(v.targets)||v.targets.length>16||!Array.isArray(v.events)||
        !finite(v.timing?.processing_s)||v.timing.processing_s<0||
        typeof v.calibration?.synchronization_verified!=='boolean'||
        v.safety?.prohibit_automatic_execution!==true||typeof v.integrity?.sha256!=='string'||
        !order.every(n=>typeof v.sources?.[n]?.present==='boolean'))return null;
      const ids=new Set();
      for(const t of v.targets) {
        const f=t?.features;
        if(typeof t?.target_id!=='string'||!t.target_id||ids.has(t.target_id)||typeof t.label!=='string'||
          typeof t.baseline_label!=='string'||t.position?.frame_id!==v.frame_id||
          typeof t.position?.valid!=='boolean'||(t.position.valid&&!vector(t.position.xyz_m,3))||
          t.safety?.prohibit_automatic_execution!==true||typeof t.safety?.require_independent_review!=='boolean'||
          !Array.isArray(t.uncertainty?.conflicts)||!Array.isArray(t.uncertainty?.missing_modalities)||
          !Array.isArray(f?.valid_mask)||f.valid_mask.length!==4||!f.valid_mask.every(x=>typeof x==='boolean')||
          !vector(f.weights,4)||f.weights.some(x=>x<0)||Math.abs(f.weights.reduce((a,b)=>a+b,0)-1)>1e-5||
          f.complete!==f.valid_mask.every(Boolean)||typeof f.fused_valid!=='boolean'||
          !vector(f.joint,4096)||!vector(f.fused,1024)||!Array.isArray(f.independent)||f.independent.length!==4||
          !f.independent.every(row=>vector(row,1024))||
          !order.every((n,i)=>t.modalities?.[n]?.target_id===t.target_id&&
            typeof t.modalities[n].associated==='boolean'&&t.modalities[n].used===f.valid_mask[i]&&
            (!f.valid_mask[i] ? f.weights[i]===0&&f.independent[i].every(x=>x===0) : t.modalities[n].associated&&v.sources[n].present)))return null;
        if(!f.joint.every((x,i)=>Math.abs(x-f.independent[Math.floor(i/1024)][i%1024])<1e-6))return null;
        ids.add(t.target_id);
      }
      return v;
    }catch(_){return null;}
  }
  const fmt=x=>finite(x)?x.toFixed(3):'—';
  function View({snapshot,now=Date.now(),receivedAt=0,offline=false,error='',status=null}) {
    const stale=!offline&&(!receivedAt||now-receivedAt>12000);
    const banner=offline?'离线模型结果':error?'数据校验失败':stale?'等待目标融合结果':status?.state==='inferencing'?'目标融合计算中':'目标融合已更新';
    return h('div',{style:{padding:12,overflow:'auto',height:'100%',background:'#101923',color:'#dce7f0',fontSize:13}},
      h('h3',null,'单目标四模态融合'),h('p',{role:'status'},banner+(error?' · '+error:'')),
      h('p',null,'按同一目标关联 RGB、NIR、点云和声源；按实际参与模态显示。'),
      snapshot?h(React.Fragment,null,
        h('p',null,'核心计算 '+fmt(snapshot.timing.processing_s)+' s · 任务周期 '+(snapshot.timing.task_period_s===null?'未规定':fmt(snapshot.timing.task_period_s)+' s')),
        h('p',null,'同步 '+(snapshot.calibration.synchronization_verified?'实测验证':'待实测验证')+' · 精度待验证 · 仅显示、记录与离线分析'),
        ...snapshot.targets.map(t=>h('article',{key:t.target_id,style:{padding:10,border:'1px solid #405064',marginBottom:10}},
          h('strong',null,t.label+' · '+(t.features.complete?'四模态齐全':(4-t.uncertainty.missing_modalities.length)+' 模态参与')),
          h('p',null,'目标 ID：'+t.target_id),
          h('p',null,'RGB 标签：'+t.baseline_label+' · 融合建议：'+t.uncertainty.fused_suggestion),
          h('div',{style:{display:'flex',gap:8}},...['rgb','infrared'].filter(n=>typeof t.images?.[n]==='string'&&t.images[n].length<400000&&/^[A-Za-z0-9+/=]+$/.test(t.images[n])).map(n=>
            h('figure',{key:n,style:{margin:0}},h('img',{src:'data:image/png;base64,'+t.images[n],alt:n+' 目标区域',style:{maxWidth:112,maxHeight:112}}),h('figcaption',null,n==='rgb'?'RGB 目标':'NIR 关联区域')))),
          h('p',null,t.position.valid?'位置 ['+t.position.xyz_m.map(fmt).join(', ')+'] m · '+snapshot.frame_id:'目标公共坐标未验证'),
          h('p',null,t.motion.motion_valid?'速度 '+fmt(t.motion.speed_m_s)+' m/s · '+t.motion.state:'运动待连续观测'),
          h('table',{style:{width:'100%'}},h('thead',null,h('tr',null,...['模态','参与','权重','关联依据'].map(n=>h('th',{key:n},n)))),
            h('tbody',null,...order.map((n,i)=>h('tr',{key:n},h('td',null,names[i]),h('td',null,t.modalities[n].used?'是':'否'),
              h('td',null,fmt(t.features.weights[i])),h('td',null,t.modalities[n].basis))))),
          t.uncertainty.conflicts.length?h('p',{style:{color:'#ffc577'}},'类别冲突，保留 RGB 标签，需独立复核'):null,
          h('p',null,'声源 '+(t.modalities.audio.associated?'方向匹配；距离未估计':'未唯一关联')),
          h('p',null,'不确定性：'+t.uncertainty.reasons.join('、')))),
        snapshot.targets.length===0?h('p',null,'本帧无可用目标'):null,
        h('p',null,'最近事件：'+snapshot.events.slice(-5).map(e=>e.type).join('、'))):null);
  }
  function Component({client,topic='/perception/fusion/targets'}) {
    const [sample,setSample]=React.useState(null),[error,setError]=React.useState(''),[now,setNow]=React.useState(Date.now());
    const [status,setStatus]=React.useState(null);
    React.useEffect(()=>{
      if(!client)return;
      const receive=m=>{const v=parseSnapshot(m);if(v){setSample({snapshot:v,receivedAt:Date.now()});setError('');}else setError('收到不兼容的目标融合消息');};
      const heartbeat=m=>{try{const s=JSON.parse(m.data);if(s.schema===1&&typeof s.state==='string')setStatus(s);}catch(_){}};
      client.subscribe(topic,receive);client.subscribe('/perception/fusion/status',heartbeat);
      return()=>{client.unsubscribe(topic,receive);client.unsubscribe('/perception/fusion/status',heartbeat);};
    },[client,topic]);
    React.useEffect(()=>{const timer=setInterval(()=>setNow(Date.now()),1000);return()=>clearInterval(timer);},[]);
    return h(View,{snapshot:sample?.snapshot,receivedAt:sample?.receivedAt||0,now,error,status});
  }
  return {Component,View,parseSnapshot};
}
