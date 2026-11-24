/* Research dashboard 3001. Snapshots come from its own frozen inference worker. */
function rtabmapResearchFactory(React) {
  'use strict';
  const h=React.createElement, finite=x=>typeof x==='number'&&Number.isFinite(x);
  const shape=x=>Array.isArray(x)?x.join(' × '):'—';
  function browserClockOffset(date,started,received) {
    const server=Date.parse(date);
    return Number.isFinite(server)&&received>=started&&received-started<3000
      ?((started+received)/2-server-500)/1000:null;
  }
  const titles={pmf:'PMF · RGB + LiDAR',avs:'AVSegFormer · RGB + 音频'};
  function parse(value) {
    if(value?.schema!=='downstream_fusion/1'||!['pmf','avs','rgbnir'].includes(value.task)||
      typeof value.session_id!=='string'||!finite(value.published_at)||typeof value.state!=='string'||
      !value.inputs||typeof value.detail!=='string')return null;
    if(value.result&&(!Number.isSafeInteger(value.result.sequence)||!finite(value.result.inference_s)||
      !finite(value.result.source_stamp)||!Array.isArray(value.result.features)||
      !value.result.features.every(r=>typeof r.name==='string'&&Array.isArray(r.shape)&&
        Array.isArray(r.vector)&&r.vector.length<=2048&&r.vector.every(finite))))return null;
    return value;
  }
  const css=`
    .fusion-research{height:100%;min-height:0;min-width:0;box-sizing:border-box;overflow-y:auto;overflow-x:hidden;background:#101b27;color:#dce8f2;font:12px/1.5 system-ui,sans-serif;padding:10px 12px}
    .fusion-research *{box-sizing:border-box}.fr-bar{display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:8px}
    .fr-tabs{display:flex;gap:4px}.fusion-research button{border:1px solid #2a4054;border-radius:6px;background:#172a3a;color:#9db4c7;padding:4px 10px;font:inherit;cursor:pointer}
    .fusion-research button[aria-pressed=true]{color:#71e5cb;background:#1b3e3d;border-color:#396e65}.fr-status{font-size:11px;color:#79e1ca}.fr-status[data-wait=true]{color:#ebc68a}
    .fr-inputs{display:flex;gap:8px;margin-bottom:9px}.fr-input{flex:1;border:1px solid #2a4054;border-radius:7px;padding:6px 9px;background:#142432;min-width:0}
    .fr-input strong{font-weight:500;color:#c8dbe8}.fr-input span{float:right;color:#72d6bb}.fr-input[data-stale=true] span{color:#cf9f72}.fr-input small{display:block;color:#7f9db4;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .fr-path{display:flex;align-items:center;gap:5px;color:#8ab7c8;font-size:11px;flex-wrap:wrap;margin:7px 0}.fr-path span{border:1px solid #2b4558;border-radius:4px;padding:3px 6px;background:#132432}.fr-path b{font-weight:400;color:#6a8ba3}
    .fr-pictures{display:grid;grid-template-columns:1fr 1fr;gap:8px}.fr-pictures figure{margin:0;border:1px solid #273b4c;border-radius:7px;overflow:hidden;background:#0b141e;min-width:0}
    .fr-pictures img{display:block;width:100%;height:155px;object-fit:contain}.fr-pictures figcaption{padding:4px 8px;color:#91acc1;font-size:11px;border-top:1px solid #233649}
    .fr-empty{min-height:150px;display:grid;place-content:center;text-align:center;gap:8px;border:1px dashed #2c4358;border-radius:8px;background:#0e1924;color:#91adbf;padding:20px}.fr-empty small{color:#58798f}
    .fr-note{color:#d4b888;font-size:11px;margin:7px 0;overflow-wrap:anywhere}.fr-foot{display:flex;flex-wrap:wrap;gap:8px;color:#7797ac;font-size:11px;margin-top:7px}.fr-foot a{color:#76c8dd;text-decoration:none}
    .fr-classes{display:flex;gap:5px;flex-wrap:wrap;margin-top:7px}.fr-classes span{border:1px solid #284153;border-radius:4px;padding:2px 7px;color:#a9c1d2;background:#152635}
    .fr-features{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:7px}.fr-feature{border:1px solid #294153;border-radius:7px;background:#122432;padding:8px;min-width:0}.fr-feature strong{font-size:11px;font-weight:500;color:#a5d5df;overflow-wrap:anywhere}.fr-feature small{display:block;color:#7fa2ba;overflow-wrap:anywhere}.fr-feature svg{display:block;width:100%;height:32px;margin:5px 0;background:#0e1c27;border-radius:4px}.fr-feature img{display:block;width:100%;height:55px;object-fit:contain;background:#0e1c27}.fr-dims{color:#8bdbcb;font-family:ui-monospace,monospace;font-size:11px;margin:3px 0}
    .fusion-research details{margin-top:6px;color:#7797ac;font-size:11px}.fusion-research summary{cursor:pointer}.fr-provenance{overflow-wrap:anywhere;padding:5px 0}
    @media(min-height:1000px){.fr-pictures img{height:210px}}@media(max-width:800px){.fr-features{grid-template-columns:1fr}.fr-pictures img{height:120px}}
  `;
  function Spark({values}) {
    if(!values?.length)return null;
    const lo=Math.min(...values),hi=Math.max(...values),stride=Math.max(1,Math.ceil(values.length/160));
    const v=values.filter((_,i)=>i%stride===0);
    const points=v.map((x,i)=>`${i*300/Math.max(1,v.length-1)},${30-28*(x-lo)/Math.max(1e-8,hi-lo)}`).join(' ');
    return h('svg',{viewBox:'0 0 300 32',preserveAspectRatio:'none',role:'img','aria-label':'实际特征通道均值曲线'},
      h('polyline',{points,fill:'none',stroke:'#65dbc2',strokeWidth:1.2}));
  }
  function Feature({row}) {
    return h('div',{className:'fr-feature'},h('strong',null,row.name),h('div',{className:'fr-dims'},shape(row.shape)),
      h('small',null,row.operation),h(Spark,{values:row.vector}),
      row.heatmap?h('img',{src:'data:image/jpeg;base64,'+row.heatmap,alt:row.name+' 特征能量图'}):null,
      h('small',null,`均值 ${row.mean.toFixed(3)} · σ ${row.std.toFixed(3)} · L2 ${row.l2.toFixed(2)}`));
  }
  function Component({task='pmf'}) {
    const [snapshot,setSnapshot]=React.useState(null),[error,setError]=React.useState(''),[tab,setTab]=React.useState('results');
    const [showAnnotations,setShowAnnotations]=React.useState(true);
    const [secondView,setSecondView]=React.useState('no_rgb');
    const [now,setNow]=React.useState(Date.now());
    const [clockOffset,setClockOffset]=React.useState(0);
    React.useEffect(()=>{
      let disposed=false,busy=false,controller=null;
      const poll=async()=>{
        if(busy||disposed)return;busy=true;controller=new AbortController();
        const timer=setTimeout(()=>controller?.abort(),3000);
        try{
          const started=Date.now();
          const response=await fetch('/research/'+task+'.json',{cache:'no-store',signal:controller.signal});
          if(!response.ok)throw Error('等待3001推理服务');
          const value=parse(await response.json());if(!value)throw Error('推理消息格式不兼容');
          if(!disposed){const offset=browserClockOffset(response.headers.get('date'),started,Date.now());
            if(offset!==null)setClockOffset(offset);setSnapshot(value);setError('');}
        }catch(e){if(!disposed)setError(e.name==='AbortError'?'推理服务响应超时':e.message);}
        finally{clearTimeout(timer);busy=false;}
      };
      poll();const interval=setInterval(poll,1000);const clock=setInterval(()=>setNow(Date.now()),500);
      return()=>{disposed=true;controller?.abort();clearInterval(interval);clearInterval(clock);};
    },[task]);
    const data=snapshot?.result,replay=snapshot?.source_mode==='offline_replay',nirTask=task==='rgbnir';
    const serverNow=now/1000-clockOffset;
    const offline=!snapshot||(!replay&&serverNow-(snapshot.display_published_at??snapshot.published_at)>4),preview=data?.mode==='rgb_only';
    const odin=snapshot?.model?.backend_kind==='rgbd_lidar_odin';
    const indoor=odin||snapshot?.model?.backend_kind==='rgb_coloured_lidar';
    const experimental=task==='pmf'&&['tf_preview','auto_estimated'].includes(data?.context?.calibration);
    const stale=!replay&&(offline||!data||serverNow-(data.display_source_stamp??data.source_stamp)>Math.max(6,task==='avs'?14:8)||['error','model_error'].includes(snapshot?.state)||(snapshot?.state==='waiting'&&!preview));
    const state=error|| (replay?'离线回放 · 已保存真实数据':offline?'等待3001推理服务':preview&&snapshot?.state!=='error'?'RGB单分支 · 待配准':snapshot?.state==='ready'?nirTask?'RGB 90% + NIR 10% · 实验融合':odin?'室内RGB-D + LiDAR · 实验融合':indoor?'室内RGB + LiDAR · 实验对照':experimental?'RGB + LiDAR · 实验融合':'实时推理':snapshot?.state==='running'?'推理中':snapshot?.state==='loading'?'加载模型':snapshot?.state==='model_error'?'模型加载失败':'等待输入条件');
    const input=(name,label)=>{
      const v=snapshot?.inputs[name],age=v?serverNow-(v.display_stamp??v.stamp):Infinity,missing=!v||age>3||age<-.5||offline;
      return h('div',{className:'fr-input',key:name,'data-stale':!replay&&missing},h('strong',null,label),h('span',null,replay?'回放':missing?'等待':age.toFixed(1)+'s'),
        h('small',null,v?['rgb','nir'].includes(name)?`${v.width} × ${v.height} · ${v.frame}`:name==='lidar'?`${v.points} 点 · ${v.frame}`:`${v.rate}Hz · ${v.channels}ch · ${finite(v.rms_dbfs)?v.rms_dbfs.toFixed(1):'—'}dBFS`:'尚未收到数据'));
    };
    const picture=(key,label)=>{
      const source=key==='lidar_overlay'&&!showAnnotations&&data?.images?.lidar_plain?'lidar_plain':key==='no_rgb_overlay'&&!showAnnotations&&data?.images?.no_rgb_plain?'no_rgb_plain':key==='reference_overlay'&&!showAnnotations&&data?.images?.reference_plain?'reference_plain':key==='overlay'&&showAnnotations&&data?.images?.annotated_overlay?'annotated_overlay':key;
      return data?.images?.[source]?h('figure',{key},h('img',{src:'data:image/jpeg;base64,'+data.images[source],alt:label}),h('figcaption',null,label)):null;
    };
    const views=nirTask?['rgb','projection']:odin?['lidar','ptv3','projection']:indoor?['no_rgb','pmf','projection']:['rgb','projection'];
    const activeView=views.includes(secondView)?secondView:views[0];
    const viewLabel=activeView==='lidar'?'LiDAR预测':activeView==='ptv3'?'PTv3对照':activeView==='no_rgb'?'移除RGB对照':activeView==='pmf'?'PMF对照':activeView==='projection'?nirTask?'配准NIR':'雷达投影':'RGB对照';
    const secondKey=nirTask?activeView==='rgb'?'reference_overlay':'projection':odin?activeView==='lidar'?'lidar_overlay':activeView==='ptv3'?'reference_overlay':'projection':indoor?activeView==='no_rgb'?'no_rgb_overlay':activeView==='pmf'?'reference_overlay':'projection':task==='pmf'&&!preview?activeView==='rgb'&&data?.images?.rgb_comparison?'rgb_comparison':'projection':'raw';
    const secondCaption=nirTask?activeView==='rgb'?'同帧RGB基线 · 原YOLOv8':'真实配准NIR':odin?activeView==='lidar'?'ODIN · 原始LiDAR点预测':activeView==='ptv3'?'原PTv3 · 相同LiDAR点对照':'相机视野内的LiDAR投影':indoor?activeView==='no_rgb'?'同几何、同类别 · 移除RGB输入':activeView==='pmf'?'原PMF对照 · 道路类别':'相机视野内的LiDAR投影':task==='pmf'&&!preview?secondKey==='rgb_comparison'?'同帧RGB分支 · 共同投影区域':'相机视野内的LiDAR投影':'同一采集帧 · RGB';
    const pipeline=nirTask?['RGB-D / NIR配准','共享冻结骨干','P3 / P4 / P5','90/10固定特征融合','原分割头']:odin?['RGB-D多帧','几何邻域交互','LiDAR / 有效深度点','特征与点坐标对齐','室内分割']:task==='pmf'?indoor?['RGB投影颜色','LiDAR位置 / 法向','6通道真实输入','点注意力编码','室内点分割']:preview?['RGB输入','冻结编码器','四级视觉特征','RGB分支分割','LiDAR待配准']:['LiDAR投影到RGB','四级H/W匹配','通道拼接','残差注意力','点云分割']:['10秒PCM / 10帧','128→256','多尺度token拼接','音频查询 / Mixer','发声区域分割'];
    return h('div',{className:'fusion-research','data-task':task},h('style',null,css),
      h('div',{className:'fr-bar'},h('div',{className:'fr-tabs'},...[["results","分割结果"],["features","特征融合"]].map(([id,title])=>h('button',{key:id,onClick:()=>setTab(id),'aria-pressed':tab===id},title)),h('button',{onClick:()=>setShowAnnotations(v=>!v),'aria-pressed':showAnnotations},showAnnotations?'标注：开':'标注：关'),(task==='pmf'&&!preview)||nirTask?h('button',{onClick:()=>setSecondView(views[(views.indexOf(activeView)+1)%views.length])},viewLabel):null),h('span',{className:'fr-status','data-wait':stale,role:'status'},state)),
      h('div',{className:'fr-inputs'},input('rgb','RGB'),input(nirTask?'nir':task==='pmf'?'lidar':'audio',nirTask?'NIR':task==='pmf'?'LiDAR':'麦克风')),
      h('div',{className:'fr-path'},...pipeline.flatMap((text,i)=>[i?h('b',{key:'arrow'+i},'→'):null,h('span',{key:text},text)])),
      snapshot?.detail&&snapshot.state!=='ready'?h('div',{className:'fr-note'},snapshot.detail):null,
      replay?h('div',{className:'fr-note'},'离线检查：使用已保存的真实输入，原始采集时间 '+new Date(data.source_stamp*1000).toLocaleString()+'；本页不代表当前实机状态。'):null,
      experimental?h('div',{className:'fr-note'},data.context.calibration==='auto_estimated'?'使用已做帧间复核的自动外参估计，尚未完成标定验收。有效投影 '+data.projected_points+' 点。':'已融合原始LiDAR；沿用当前SLAM TF，安装外参尚未实测标定。有效投影 '+data.projected_points+' 点。'):null,
      data?.fusion_quality?h('div',{className:'fr-note'},'投影覆盖 '+(data.fusion_quality.projection_coverage*100).toFixed(1)+'%'+(finite(data.fusion_quality.rgb_branch_disagreement)?' · 与同帧RGB分支差异 '+(data.fusion_quality.rgb_branch_disagreement*100).toFixed(1)+'%（不是准确率）。':indoor?' · '+data.observation_count+'组真实帧 / '+data.window_s.toFixed(1)+'s · '+data.scene_points+'个有色点。':'')):null,
      data?.comparison?h('div',{className:'fr-note'},odin?'与原PTv3在共同LiDAR点上的类别差异 '+(data.comparison.projected_disagreement*100).toFixed(1)+'%；这是模型对照，尚无独立人工真值。':'移除RGB输入后，共同投影点类别差异 '+(data.comparison.projected_disagreement*100).toFixed(1)+'%；属于输入消融，尚无独立训练的LiDAR基线。'):null,
      odin&&data?.input_statistics?h('div',{className:'fr-note'},'全图仅标记 '+data.input_statistics.depth_query_points+' 个真实有效深度像素；右侧另示原始LiDAR预测。深度补洞仅用于模型上下文，不显示为实测像素。'):null,
      nirTask&&data?h('div',{className:'fr-note'},'NIR有效配准覆盖 '+(data.valid_coverage*100).toFixed(1)+'% · 实验 '+data.detection_count+' 个实例 / RGB基线 '+data.baseline_count+' 个实例；有效区域采用90/10，无效区域保持RGB特征。'):null,
      !data?h('div',{className:'fr-empty'},h('span',null,nirTask?'等待同步RGB-D、mono8 NIR及相机标定':task==='pmf'?indoor?'等待多帧RGB-D、LiDAR及同步位姿':'等待RGB与LiDAR输入':'等待连续10秒音频及对应视频'),h('small',null,snapshot?.model?'完整预训练权重已加载 · 冻结推理':'正在准备作者预训练模型')):
        tab==='features'?h('div',null,h('div',{className:'fr-note'},'曲线为实际张量的通道均值，按各自范围显示。特征来自本次真实输入；完整保存内容见NPZ。'),h('div',{className:'fr-features'},...data.features.map(row=>h(Feature,{key:row.name,row})))):
        h('div',null,h('div',{className:'fr-pictures'},picture('overlay',nirTask?'RGB 90% + NIR 10% · 实例预测':odin?'ODIN · RGB + 深度 + LiDAR · 有效深度区域':task==='pmf'?indoor?'室内PTv3实验结果 · 有效投影点':preview?'PMF RGB分支 · 尚未融合LiDAR':experimental?'RGB + LiDAR实验融合 · 有效投影点':'融合分割 · 仅标记有效投影点':'音视频语义分割 · 第10帧'),picture(secondKey,secondCaption)),
          h('div',{className:'fr-classes'},...data.classes.map(c=>h('span',{key:c.id,style:{borderLeft:'3px solid rgb('+c.color.join(',')+')'}},(c.display_label||c.label)+' · '+(c.fraction*100).toFixed(1)+'%')),task==='avs'&&!data.classes.length?h('span',null,'本段未检测到发声前景'):null),
          showAnnotations?h('div',{className:'fr-note'},task==='pmf'&&!preview&&!odin?'框与轮廓由有效LiDAR投影点聚合，标记模型预测区域。':'框与轮廓标记模型预测的语义区域。',data.annotations?.length?' 共 '+data.annotations.length+' 处。':''):null),
      stale&&data?h('div',{className:'fr-note'},'显示上次结果，当前输入未满足实时推理条件。'):null,
      h('div',{className:'fr-foot'},h('span',null,snapshot?.model?`${snapshot.model.name} · ${snapshot.model.classes} 类 · 冻结权重`:'模型准备中'),
        data?h('span',null,`#${data.sequence} · ${Math.round(data.inference_s*1000)}ms · ${finite(data.context.sync_gap_s)?'时间偏差 '+(data.context.sync_gap_s*1000).toFixed(0)+'ms':'等待同步输入'}`):null,
        data?.feature_download?h('a',{href:data.feature_download,download:true},'下载特征 NPZ'):null),
      snapshot?.offline_report?h('div',{className:'fr-foot'},h('a',{href:'/research/offline/index.html',target:'_blank',rel:'noopener'},'查看已保存真实帧的离线对照')):null,
      h('div',{className:'fr-note'},'精度待评估：需要人工真值与独立单模态基线。'+(task==='pmf'?odin?' ScanNet20没有机器狗类别；PTv3对照使用相同类别与LiDAR点，模型仍可能误分类。':indoor?' ScanNet20没有机器狗类别；PMF为不同类别空间的对照，仍可能误分类。':' SemanticKITTI权重的室内效果需验证。':'')),
      h('details',null,h('summary',null,'模型与输入详情'),h('div',{className:'fr-provenance'},
        snapshot?.model?h('div',null,`${snapshot.model.backend} · strict=${snapshot.model.strict} · ${snapshot.model.tensors} 个张量 · SHA256 ${snapshot.model.sha256}`):null,
        data?h('div',null,`输出 ${shape(data.output_shape)} · ${data.baseline_note}`):null,
        snapshot?.ros_error?h('div',null,'ROS: '+snapshot.ros_error):null,
        task==='avs'&&data?h('div',null,'音频时间戳质量：'+data.context.stamp_quality+'；本模型输出画面中的发声区域。'):null)));
  }
  return {Component,parse,Spark,browserClockOffset};
}
