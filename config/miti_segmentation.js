/* Independent segmentation display. JPEGs and labels arrive atomically from one source frame. */
function rtabmapSegmentationWidget(id = 'rtabmap-segmentation') {
  return {i:id, type:'yolo-segmentation', title:'目标分割 · YOLO', x:6, y:24, w:6, h:10,
    props:{topic:'/miti/segmentation/view'}};
}
function rtabmapMigrateSegmentationLayout(layout) {
  if (!layout || !Array.isArray(layout.widgets)) return layout;
  const leaves = t => typeof t === 'string' ? [t] : t ? [...leaves(t.first),...leaves(t.second)] : [];
  const visible = leaves(layout.mosaic);
  const anchor = layout.widgets.find(w => w.type === 'fourmodal-features' && visible.includes(w.i));
  if (!anchor || anchor.rtabmapSegmentationLayoutVersion === 1) return layout;
  const widgets = layout.widgets.map(w => w === anchor ? {...w,rtabmapSegmentationLayoutVersion:1} : w);
  if (widgets.some(w => w.type === 'yolo-segmentation')) return {...layout,widgets};
  let id = 'rtabmap-segmentation', suffix = 2;
  while (widgets.some(w => w.i === id)) id = 'rtabmap-segmentation-' + suffix++;
  widgets.push(rtabmapSegmentationWidget(id));
  const insert = t => typeof t === 'string' ? t === anchor.i ?
    {direction:'row',first:t,second:id,splitPercentage:50} : t :
    t ? {...t,first:insert(t.first),second:insert(t.second)} : t;
  return {...layout,widgets,mosaic:insert(layout.mosaic)};
}

function rtabmapSegmentationFactory(React) {
  'use strict';
  const h = React.createElement;
  const finite = n => typeof n === 'number' && Number.isFinite(n);
  function parseSnapshot(message) {
    try {
      if (typeof message?.data !== 'string' || message.data.length > 8000000) return null;
      const v = JSON.parse(message.data);
      if (v.schema !== 1 || typeof v.session_id !== 'string' || !Number.isSafeInteger(v.sequence) ||
          v.sequence < 1 || ![0,1].includes(v.revision) || !['rgb_yolo','fusion'].includes(v.result_source) ||
          !Number.isInteger(v.width) || !Number.isInteger(v.height) || v.width < 1 || v.height < 1 ||
          v.width > 8192 || v.height > 8192 || !finite(v.inference_s) || v.inference_s < 0 ||
          (v.source_age_s !== undefined && (!finite(v.source_age_s) || v.source_age_s < 0)) ||
          !Number.isInteger(v.stamp?.sec) || !Number.isInteger(v.stamp?.nanosec) ||
          !['raw','overlay','classes'].every(k => typeof v.images?.[k] === 'string' &&
            v.images[k].length > 0 && /^[A-Za-z0-9+/=]+$/.test(v.images[k])) ||
          !Array.isArray(v.instances) || v.instances.length > 500 ||
          !v.instances.every(r => Number.isInteger(r.instance_id) && r.instance_id > 0 &&
            Number.isInteger(r.class_id) && r.class_id >= 0 && typeof r.label === 'string' && r.label.length < 160 &&
            finite(r.confidence) && r.confidence >= 0 && r.confidence <= 1 &&
            Array.isArray(r.bbox_xyxy) && r.bbox_xyxy.length === 4 && r.bbox_xyxy.every(finite) &&
            Array.isArray(r.color) && r.color.length === 3 && r.color.every(c => Number.isInteger(c) && c >= 0 && c <= 255))) return null;
      return v;
    } catch (_) { return null; }
  }
  function parseStatus(message) {
    try {
      const v = typeof message?.data === 'string' && message.data.length < 16000 ? JSON.parse(message.data) : null;
      return v?.schema === 1 && typeof v.state === 'string' && typeof v.session_id === 'string' ? v : null;
    } catch (_) { return null; }
  }
  function parseFusion(message) {
    try {
      if (typeof message?.data !== 'string' || message.data.length > 8000000) return null;
      const v=JSON.parse(message.data);
      if(v.schema!==1 || v.experimental!==true || typeof v.session_id!=='string' || !Number.isSafeInteger(v.sequence) || v.sequence<1 ||
        !['unlabeled','validated_fusion','validated_baseline'].includes(v.accuracy_status)||
        !Number.isInteger(v.width)||!Number.isInteger(v.height)||v.width<1||v.height<1||v.width>8192||v.height>8192||
        !finite(v.inference_s)||v.inference_s<0||!finite(v.coverage)||v.coverage<0||v.coverage>1||
        (v.source_age_s!==undefined && (!finite(v.source_age_s)||v.source_age_s<0))||
        !Array.isArray(v.objects)||v.objects.length>16||!Array.isArray(v.class_names)||v.class_names.length>512||
        !v.class_names.every(n=>typeof n==='string'&&n.length<160)||
        !v.objects.every(r=>Number.isInteger(r.instance_id)&&r.instance_id>0&&Number.isInteger(r.task_id))||
        !['rgb','nir','fused'].every(k=>typeof v.images?.[k]==='string' && v.images[k].length>0 && /^[A-Za-z0-9+/=]+$/.test(v.images[k])))return null;
      if(v.image_labels && !['rgb','nir','fused'].every(k=>Array.isArray(v.image_labels[k]) &&
        v.image_labels[k].length<=500 && v.image_labels[k].every(r=>Number.isInteger(r.class_id) &&
          r.class_id>=0 && (typeof r.label==='string' ? r.label.length>0 && r.label.length<160 : r.class_id<v.class_names.length) && finite(r.x) && finite(r.y) &&
          r.x>=0 && r.x<v.width && r.y>=0 && r.y<v.height)))return null;
      return v;
    } catch(_){return null;}
  }
  function parseFusionStatus(message) {
    try {
      const v=typeof message?.data==='string' && message.data.length<20000 ? JSON.parse(message.data) : null;
      return v?.schema===1 && typeof v.state==='string' && Number.isInteger(v.completed) ? v:null;
    }catch(_){return null;}
  }
  function FusionPicture({data,kind,title}) {
    const picture=React.useRef(null),[width,setWidth]=React.useState(350);
    React.useEffect(()=>{
      if(!picture.current || typeof ResizeObserver==='undefined')return;
      const observer=new ResizeObserver(entries=>{
        const next=entries[0]?.contentRect?.width;if(next>0)setWidth(next);
      });
      observer.observe(picture.current);return ()=>observer.disconnect();
    },[]);
    const size=Math.max(14,14*data.width/Math.max(width,1)),padding=size*.28;
    const count=data.image_instance_counts?.[kind];
    return h('figure',null,h('figcaption',null,title+(Number.isInteger(count)?` · ${count} 个目标`:'')),
      h('svg',{ref:picture,className:'rtab-fusion-picture',viewBox:`0 0 ${data.width} ${data.height}`,
        role:'img','aria-label':title},
        h('image',{href:'data:image/jpeg;base64,'+data.images[kind],width:data.width,height:data.height}),
        ...(data.image_labels?.[kind]||[]).map((row,index)=>{
          const text=row.label||data.class_names[row.class_id]||'未知';
          const boxWidth=Math.min(data.width-6,text.length*size+padding*2),boxHeight=size+padding*2;
          const x=Math.max(3,Math.min(data.width-boxWidth-3,row.x-boxWidth/2));
          const y=Math.max(3,Math.min(data.height-boxHeight-3,row.y-boxHeight/2));
          return h('g',{key:index,className:'rtab-fusion-class-label',pointerEvents:'none'},
            h('rect',{x,y,width:boxWidth,height:boxHeight,rx:padding,fill:'#07131d',fillOpacity:.85}),
            h('text',{x:x+padding,y:y+padding+size*.83,fill:'#fff',fontSize:size,fontWeight:600},text));
        })));
  }
  function FusionComparison({sample,status,now=0}) {
    const data=sample?.snapshot, heartbeat=status && now-status.receivedAt<4000 ? status.snapshot:null;
    const stopped=!heartbeat;
    const stale=data && (stopped || heartbeat.state==='input_stale' || heartbeat.state==='error' ||
      (heartbeat.last_result && heartbeat.last_result.session_id!==data.session_id) ||
      now-sample.receivedAt>Math.max(10000,data.inference_s*2500+5000));
    const accuracy=data?.accuracy_status==='validated_fusion'?'已通过留出集评估':
      data?.accuracy_status==='validated_baseline'?'评估选择 RGB 基线':'精度未验证';
    return h('div',{className:'rtab-fusion-compare'},
      h('div',{className:'rtab-fusion-note'},data ? `同一采集帧 #${data.sequence} · RGB + 近红外 · ${accuracy}` : '等待真实目标级融合'),
      !data ? h('div',{className:'rtab-seg-empty'},heartbeat?.last_error||heartbeat?.reason||'等待融合节点') :
        h('div',{className:'rtab-fusion-images'},...[
          ['rgb','RGB 单模态'],['nir','近红外单模态 · 投影到 RGB'],['fused','UniBind 融合预览']
        ].map(([key,label])=>h(FusionPicture,{key,data,kind:key,title:label}))),
      data ? h('div',{className:'rtab-seg-foot'},
        h('span',null,`${data.objects.length} 个目标参与 · 融合计算 ${data.inference_s.toFixed(2)} s`),
        finite(data.source_age_s) ? h('span',null,`画面帧龄 ${(data.source_age_s+Math.max(0,now-(sample.receivedAt??now))/1000).toFixed(1)} s`) : null,
        h('span',null,`几何有效区域 ${Math.round(data.coverage*100)}%`),
        h('span',null,stale?'最近结果 · 输入或节点未更新':heartbeat.state==='inferencing'?'正在计算下一帧':'异步融合结果')) : null,
      data && heartbeat && ['error','waiting_inputs','input_stale'].includes(heartbeat.state) ?
        h('div',{className:'rtab-seg-error'},heartbeat.last_error||heartbeat.reason||'等待同步输入') : null,
      heartbeat ? h('div',{className:'rtab-fusion-note'},`已采集 ${heartbeat.captured||0} 组 · 待人工标注`) : null,
      data ? h('details',null,h('summary',null,'目标级特征复核'),
        h('p',null,'下列为 UniBind 的目标分类建议；上方近红外图是独立 YOLO 分割。'),
        h('div',{className:'rtab-seg-list'},...data.objects.map(row=>h('span',{key:row.instance_id},
          `#${row.instance_id} RGB：${data.class_names[row.rgb_task_id]||'未知'} · NIR：${data.class_names[row.nir_task_id]||'未知'} → 融合：${data.class_names[row.selected_task_id??row.task_id]||'未知'}`)))) : null);
  }
  function deriveState(sample, status, now, connected, error = '') {
    if (error) return {label:'显示异常', stale:true, detail:error};
    if (!connected) return {label:'等待连接',stale:true};
    const heartbeat = status && now-status.receivedAt < 3500 ? status.snapshot : null;
    if (heartbeat?.state === 'loading_model') return {label:'模型加载中',stale:true};
    if (heartbeat?.state === 'inference_error') return {label:'推理异常',stale:true,detail:heartbeat.last_error};
    if (heartbeat?.state === 'input_stale') return {label:'相机已断流 · 最近结果',stale:true};
    if (heartbeat?.state === 'waiting_image') return {label:'等待 RGB 图像',stale:true};
    if (heartbeat?.state === 'timestamp_rejected') return {label:'相机时间戳未对齐',stale:true};
    if (!sample) return {label:heartbeat ? '分割计算中' : '等待分割节点',stale:true};
    if (!heartbeat || heartbeat.session_id !== sample.snapshot.session_id ||
        now-sample.receivedAt > Math.max(4000, sample.snapshot.inference_s*3000+1500)) {
      return {label:'结果已过期',stale:true};
    }
    if (finite(heartbeat.frame_age_s) && heartbeat.frame_age_s>Math.max(4,sample.snapshot.inference_s*3+1.5)) {
      return {label:'采集结果有延迟',stale:true};
    }
    return {label:'实时',stale:false};
  }
  function subscribe(client, topic, parser, setter, onError, clock = () => performance.now()) {
    let disposed = false;
    const receive = message => {
      if (disposed) return;
      const snapshot = parser(message);
      if (snapshot) setter({snapshot,receivedAt:clock()});
      else onError('分割消息格式不兼容');
    };
    client.subscribe(topic,receive);
    return () => { disposed=true; client.unsubscribe(topic,receive); };
  }
  const css = `
    .rtab-seg{height:100%;min-height:0;display:flex;flex-direction:column;background:#111923;color:#e0eaf2;font:13px/1.4 system-ui,sans-serif;overflow:auto}
    .rtab-seg *{box-sizing:border-box}.rtab-seg-bar{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:10px 12px;flex-wrap:wrap}
    .rtab-seg-tabs{display:flex;gap:3px;background:#1a2632;border-radius:8px;padding:3px}.rtab-seg button{font:inherit;border:0;color:#9fb2c4;background:transparent;border-radius:5px;padding:5px 10px;cursor:pointer}
    .rtab-seg button[aria-pressed=true]{background:#304456;color:#f0f5f9}.rtab-seg-state{font-size:12px;color:#65d1b6}.rtab-seg-state[data-stale=true]{color:#e0b982}
    .rtab-seg-picture{position:relative;flex:1;min-height:180px;display:flex;align-items:center;justify-content:center;background:#0c1219;overflow:hidden}
    .rtab-seg-picture svg{position:absolute;inset:0;width:100%;height:100%}.rtab-seg-empty{color:#7d93a6;padding:30px;text-align:center}
    .rtab-seg-foot{display:flex;flex-wrap:wrap;gap:10px;justify-content:space-between;padding:9px 12px;color:#9fb2c4;font-size:12px}
    .rtab-seg details{margin:0 12px 10px;font-size:12px;color:#9fb2c4}.rtab-seg summary{cursor:pointer}.rtab-seg-list{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px}.rtab-seg-list span{background:#1b2936;border-radius:5px;padding:4px 7px}
    .rtab-seg-error{color:#e0b982;margin:0 12px 8px;font-size:12px}
    .rtab-fusion-compare{padding:0 10px 10px}.rtab-fusion-images{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}.rtab-fusion-images figure{margin:0;background:#0c1219;border-radius:7px;overflow:hidden}.rtab-fusion-picture{display:block;width:100%;height:auto}.rtab-fusion-images figcaption{font-size:12px;color:#c5d4df;padding:7px 9px}.rtab-fusion-note{font-size:12px;color:#dfbd82;padding:8px 2px}
  `;
  function View({sample=null,state={label:'等待分割节点',stale:true},status=null,mode='overlay',onMode,pictureRef,pictureSize,fusion,fusionStatus,now}) {
    const data = sample?.snapshot;
    const tabs=h('div',{className:'rtab-seg-tabs',role:'group','aria-label':'分割显示方式'},
      ...[['raw','原图'],['overlay','叠加'],['classes','类别图'],['fusion','融合对比']].map(([key,label])=>
        h('button',{key,onClick:()=>onMode?.(key),'aria-pressed':mode===key},label)));
    if(mode==='fusion')return h('div',{className:'rtab-seg'},h('style',null,css),
      h('div',{className:'rtab-seg-bar'},tabs,h('span',{className:'rtab-seg-state','data-stale':true},'实验预览')),
      h(FusionComparison,{sample:fusion,status:fusionStatus,now}));
    const scale = data && pictureSize?.width && pictureSize?.height ?
      Math.min(pictureSize.width/data.width,pictureSize.height/data.height) : 1;
    return h('div',{className:'rtab-seg'}, h('style',null,css),
      h('div',{className:'rtab-seg-bar'},
        tabs,
        h('span',{className:'rtab-seg-state','data-stale':state.stale,role:'status'},state.label)),
      h('div',{className:'rtab-seg-picture',ref:pictureRef},data ?
        h('svg',{viewBox:`0 0 ${data.width} ${data.height}`,role:'img','aria-label':'YOLO 分割结果'},
          h('image',{href:'data:image/jpeg;base64,'+data.images[mode],width:data.width,height:data.height}),
          mode !== 'raw' ? data.instances.map(row => {
            const size = Math.max(14,12/scale);
            const x = Math.max(3,Math.min(data.width-row.label.length*size-8,row.bbox_xyxy[0]));
            const y = Math.max(size+5,Math.min(data.height-5,row.bbox_xyxy[1]+size));
            return h('text',{key:row.instance_id,x,y,fill:'#f8fbff',stroke:'#15222d',strokeWidth:3,
              paintOrder:'stroke',fontSize:size,fontWeight:500},row.label);
          }) : null) : h('div',{className:'rtab-seg-empty'},'等待相机与分割结果')),
      h('div',{className:'rtab-seg-foot'},
        h('span',null,data ? `采集帧 #${data.sequence} · ${data.instances.length} 个目标 · ${Math.round(data.inference_s*1000)} ms` : '—'),
        finite(data?.source_age_s) ? h('span',null,`画面帧龄 ${(data.source_age_s+Math.max(0,(now??0)-(sample.receivedAt??now??0))/1000).toFixed(1)} s`) : null,
        h('span',null,data?.result_source === 'fusion' ? '融合复核' : data?.backend === 'yoloe_pf' ? 'RGB · YOLOE' : 'RGB · YOLOv8'),
        mode === 'classes' ? h('span',null,'未覆盖区域为未知') : null),
      state.detail ? h('p',{className:'rtab-seg-error'},state.detail) : null,
      h('details',null,h('summary',null,'结果详情'),
        h('p',null,status?.fusion_mode === 'shadow' ? '融合接口已启用 · 候选仅对比' :
          status?.fusion_mode === 'guarded' ? '通过校验的候选可参与类别复核' : '采用 RGB 分割结果'),
        data ? h('div',{className:'rtab-seg-list'},...data.instances.map(row =>
          h('span',{key:row.instance_id},row.label+' · YOLO '+Math.round(row.confidence*100)+'%'+
            (row.classification_score >= 0 ? ' · 复核 '+Math.round(row.classification_score*100)+'%' : '')))) : null));
  }
  function Component({client,topic='/miti/segmentation/view'}) {
    const [sample,setSample]=React.useState(null),[status,setStatus]=React.useState(null);
    const [fusion,setFusion]=React.useState(null),[fusionStatus,setFusionStatus]=React.useState(null);
    const [mode,setMode]=React.useState('overlay'),[error,setError]=React.useState('');
    const [now,setNow]=React.useState(()=>performance.now());
    const pictureRef=React.useRef(null),[pictureSize,setPictureSize]=React.useState(null);
    React.useEffect(()=>{
      if (!pictureRef.current || typeof ResizeObserver === 'undefined') return;
      const observer=new ResizeObserver(entries=>{
        const r=entries[0]?.contentRect;if(r)setPictureSize({width:r.width,height:r.height});
      });
      observer.observe(pictureRef.current);return ()=>observer.disconnect();
    },[]);
    React.useEffect(()=>{
      setSample(null);setStatus(null);setError('');
      setFusion(null);setFusionStatus(null);
      if (!client) return;
      const disposers=[];
      try {
        disposers.push(subscribe(client,topic,parseSnapshot,value=>{setSample(value);setError('');},setError));
        disposers.push(subscribe(client,'/miti/segmentation/status',parseStatus,setStatus,setError));
        disposers.push(subscribe(client,'/miti/segmentation/fusion_view',parseFusion,setFusion,setError));
        disposers.push(subscribe(client,'/miti/segmentation/fusion_status',parseFusionStatus,setFusionStatus,setError));
      } catch (_) { setError('无法订阅分割话题'); }
      return ()=>disposers.forEach(fn=>fn());
    },[client,topic]);
    React.useEffect(()=>{const timer=setInterval(()=>setNow(performance.now()),500);return ()=>clearInterval(timer);},[]);
    return h(View,{sample,status:status?.snapshot,mode,onMode:setMode,pictureRef,pictureSize,fusion,fusionStatus,now,
      state:deriveState(sample,status,now,Boolean(client),error)});
  }
  return {Component,View,parseSnapshot,parseStatus,parseFusion,parseFusionStatus,FusionPicture,FusionComparison,deriveState,subscribe};
}
