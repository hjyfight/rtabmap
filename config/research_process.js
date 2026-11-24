/* Same-origin, read-only explorer. Presentation state never sends ROS commands. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const names = {pmf:'RGB + LiDAR（深度辅助）',rgbnir:'RGB + 近红外',avs:'RGB + 声学'};
  const steps = ['模态输入','标定与预处理','时间与接口','特征提取','匹配、拼接与对齐','融合输出与精度'];
  const titles = {
    pmf:['图像与真实LiDAR点云','点—像素对应与坐标基准','同组观测的时间与字段','图像特征与三维几何特征','颜色与法向量的实际拼接','三维感知结果与对照'],
    rgbnir:['可见光与近红外输入','深度辅助配准与有效区域','同帧配对与接口','两路输入的多尺度特征','特征尺度匹配与加权融合','同一RGB样本的融合与对照'],
    avs:['图像与声学输入','PCM到log-Mel的预处理','声学窗口与图像帧配对','声学嵌入与视觉特征','视觉token拼接与声学条件交互','声学关联的视觉分割']
  };
  const descriptions = {
    pmf:['真实LiDAR扫描与RGB-D图像分别输入；点云保留其采集来源。','使用已记录内外参、位姿与深度建立几何对应；外参状态与验收结论分别显示。','每组观测展示原始时间、已记录校正时间与配对时间差。','图像多尺度特征参与三维几何交互；点特征在真实XYZ位置上展示。','对照分支将每点RGB颜色与LiDAR表面法向量拼为6维输入；XYZ单独提供位置。','主分支使用RGB-D几何上下文与LiDAR点查询；对照分支独立运行。'],
    rgbnir:['近红外来自独立单通道图像；深度用于将近红外配准到RGB网格。','查看原始近红外、配准结果及有效像素。无效区域不参与近红外融合。','展示同一输入组的时间关系，保留图像尺寸、格式与坐标系信息。','共享骨干分别处理RGB与近红外，查看P3、P4、P5特征响应。','同尺度特征使用RMS增益匹配幅值，再加权组合；该分支使用加权融合。','同一RGB输入的纯RGB结果与RGB＋近红外融合结果并排查看。'],
    avs:['工作站WheelTec采集声学数据；仅展示与RGB帧匹配的声学窗口。','实际PCM波形、频谱与log-Mel转换；声学预处理不等于声学空间标定。','十个图像帧分别匹配0.96秒log-Mel片段的中心时刻。','声学嵌入从128维投影到256维；视觉多尺度特征统一到256通道。','视觉token沿序列维拼接；声学条件查询通过注意力与视觉区域交互。','输出声学关联的图像分割。需要同样本评测记录才能判定融合精度。']
  };
  let branch='pmf',step=0,frozen=false,status=null,polling=false,featureChoice={},plotObserver=null;
  const samples={},pending={};
  const esc = value => String(value??'—').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const fmt = (value, digits=3) => Number.isFinite(Number(value)) && value!=null ? Number(value).toFixed(digits) : '—';
  const shape = row => (row?.saved_shape||row?.shape||[]).join(' × ');
  const empty = text => `<div class="empty">${esc(text)}</div>`;
  const note = text => `<div class="inline-note">${esc(text)}</div>`;
  const figure = (jpg,label,sub='') => `<figure>${jpg?`<img src="data:image/jpeg;base64,${esc(jpg)}" alt="${esc(label)}">`:empty('本样本未归档：'+label)}<figcaption><strong>${esc(label)}</strong>${sub?'<br>'+esc(sub):''}</figcaption></figure>`;
  const plot = (id,label,compact=false) => `<figure class="plot ${compact?'compact-plot':''}"><canvas id="${id}" aria-label="${esc(label)}" role="img"></canvas><figcaption>${esc(label)}</figcaption></figure>`;
  const table = (headers,rows) => `<div class="table-wrap"><table class="data-table"><thead><tr>${headers.map(x=>`<th>${esc(x)}</th>`).join('')}</tr></thead><tbody>${rows.map(row=>`<tr>${row.map(x=>`<td>${esc(x)}</td>`).join('')}</tr>`).join('')}</tbody></table></div>`;
  const matrix = (name,values) => values?`<h3>${esc(name)}</h3><pre>${esc(values.map(r=>r.map(x=>fmt(x,5).padStart(10)).join(' ')).join('\n'))}</pre>`:note(name+'未归档');
  const flow = (boxes,join=false) => `<div class="flow">${boxes.map((r,i)=>(i?'<span class="arrow">'+(join&&i===1?'＋':'→')+'</span>':'')+`<div class="flow-box"><strong>${esc(r[0])}</strong><span>${esc(r[1])}</span></div>`).join('')}</div>`;
  const allFeatures = s => s?.features||[];
  const feature = (s,name) => allFeatures(s).find(f=>f.name===name);
  const heat = (s,name,label) => {const f=feature(s,name);return figure(f?.heatmap,label||name,shape(f));};
  const sampleId = s => s?`${s.session_id}:${s.sequence}`:'';
  const stamp = t => Number.isFinite(t)?new Date(t*1000).toLocaleString('zh-CN',{hour12:false,timeZone:'Asia/Shanghai'}):'—';

  function canvasContext(id) {
    const c=$(id);if(!c)return null;
    const dpr=Math.min(window.devicePixelRatio||1,2),w=c.clientWidth,h=c.clientHeight;
    if(!w||!h)return null;
    c.width=Math.round(w*dpr);c.height=Math.round(h*dpr);
    const ctx=c.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,h);
    return {c,ctx,w,h};
  }
  function lineChart(id,series,xLabel,yLabel) {
    const cc=canvasContext(id);if(!cc)return;const {ctx,w,h}=cc;
    const left=50,right=w-15,top=16,bottom=h-40;
    const points=series.flatMap(s=>s.points).filter(p=>Number.isFinite(p[0])&&Number.isFinite(p[1]));
    if(!points.length)return;
    let x0=Math.min(...points.map(p=>p[0])),x1=Math.max(...points.map(p=>p[0]));
    let y0=Math.min(...points.map(p=>p[1])),y1=Math.max(...points.map(p=>p[1]));
    if(x1===x0)x1=x0+1;if(y1===y0){y0-=.5;y1+=.5;}
    const pad=(y1-y0)*.08;y0-=pad;y1+=pad;
    const x=t=>left+(t-x0)/(x1-x0)*(right-left),y=t=>bottom-(t-y0)/(y1-y0)*(bottom-top);
    ctx.font='11px system-ui';ctx.strokeStyle='#253e52';ctx.fillStyle='#99adbc';
    for(let i=0;i<4;i++){const t=y0+(y1-y0)*i/3,yy=y(t);ctx.beginPath();ctx.moveTo(left,yy);ctx.lineTo(right,yy);ctx.stroke();ctx.fillText(fmt(t,2),4,yy+4);}
    for(let i=0;i<4;i++){const t=x0+(x1-x0)*i/3;ctx.textAlign=i===3?'right':i===0?'left':'center';ctx.fillText(fmt(t,1),x(t),bottom+17);}
    ctx.textAlign='center';ctx.fillText(xLabel,(left+right)/2,h-5);ctx.textAlign='left';ctx.fillText(yLabel,left,12);
    series.forEach(s=>{ctx.strokeStyle=s.color;ctx.lineWidth=1.4;ctx.beginPath();s.points.forEach((p,i)=>i?ctx.lineTo(x(p[0]),y(p[1])):ctx.moveTo(x(p[0]),y(p[1])));ctx.stroke();});
  }
  function cloudPlot(id,cloud) {
    if(!cloud?.xyz?.length)return;
    let yaw=.6,pitch=.45,drag=null;
    const axes=['X','Y','Z'];
    const xyz=cloud.xyz,mins=[0,1,2].map(k=>Math.min(...xyz.map(p=>p[k]))),maxs=[0,1,2].map(k=>Math.max(...xyz.map(p=>p[k])));
    const center=mins.map((m,k)=>(m+maxs[k])/2),extent=Math.max(...maxs.map((m,k)=>m-mins[k]),.01);
    const rotate=p=>{const a=p[0]*Math.cos(yaw)-p[1]*Math.sin(yaw),b=p[0]*Math.sin(yaw)+p[1]*Math.cos(yaw);return [a,b*Math.cos(pitch)-p[2]*Math.sin(pitch),b*Math.sin(pitch)+p[2]*Math.cos(pitch)];};
    function draw(){const cc=canvasContext(id);if(!cc)return;const {ctx,w,h}=cc,scale=Math.min(w,h)*.78/extent;
      const energy=cloud.energy||[],emin=energy.length?Math.min(...energy):0,emax=energy.length?Math.max(...energy):1;
      const points=xyz.map((p,i)=>({p:rotate(p.map((v,k)=>v-center[k])),i})).sort((a,b)=>a.p[2]-b.p[2]);
      points.forEach(({p,i})=>{const rgb=cloud.rgb?.[i],a=(energy[i]-emin)/Math.max(emax-emin,1e-9);ctx.fillStyle=rgb?`rgb(${rgb.join(',')})`:energy.length?`hsl(${190-150*a} 80% ${50+10*a}%)`:'#70d8c2';ctx.fillRect(w/2+p[0]*scale,h/2-p[1]*scale,2.2,2.2);});
      ctx.font='11px system-ui';const origin=[38,h-35];axes.forEach((label,k)=>{const a=[0,0,0];a[k]=23;const p=rotate(a);ctx.strokeStyle=['#70d8c2','#7bb5f2','#efc17f'][k];ctx.fillStyle=ctx.strokeStyle;ctx.beginPath();ctx.moveTo(...origin);ctx.lineTo(origin[0]+p[0],origin[1]-p[1]);ctx.stroke();ctx.fillText(label,origin[0]+p[0]+4,origin[1]-p[1]);});
      ctx.fillStyle='#99adbc';ctx.fillText(`${xyz.length} / ${cloud.count??xyz.length} 点 · 拖动旋转`,15,20);
    }
    draw();const c=$(id);c.onpointerdown=e=>{drag=[e.clientX,e.clientY];c.setPointerCapture(e.pointerId);};c.onpointermove=e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch+=(e.clientY-drag[1])*.008;drag=[e.clientX,e.clientY];draw();};c.onpointerup=()=>{drag=null;};c.onpointercancel=()=>{drag=null;};c._redraw=draw;
  }
  function timeline(data,raw=false) {
    const rows=data.rows||[];if(!rows.length)return empty('本样本没有配对时间记录');
    const width=720,height=rows.length*31+58,left=115,right=685;
    const vals=rows.flatMap(r=>raw?[r.raw_a,r.raw_b]:[r.a,r.b]).filter(Number.isFinite);
    if(!vals.length)return empty('旧样本未记录应用偏移，无法还原该分支原始时间');
    const origin=Math.min(...vals),range=Math.max(Math.max(...vals)-origin,.01),x=t=>left+(t-origin)/range*(right-left);
    return `<div class="legend"><span>${esc(data.a_name)}</span><span>${esc(data.b_name)}</span></div><svg class="timeline" viewBox="0 0 ${width} ${height}" role="img" aria-label="${raw?'原始':'记录'}时间轴">${rows.map((r,i)=>{const a=raw?r.raw_a:r.a,b=raw?r.raw_b:r.b,yy=i*31+22;if(!Number.isFinite(a)||!Number.isFinite(b))return '';return `<text x="8" y="${yy+4}">输入组 ${i+1}</text><line x1="${x(a)}" x2="${x(b)}" y1="${yy}" y2="${yy}"/><circle class="a" cx="${x(a)}" cy="${yy-3}" r="4"><title>RGB ${a}</title></circle><circle class="b" cx="${x(b)}" cy="${yy+3}" r="4"><title>${esc(data.b_name)} ${b}</title></circle>`;}).join('')}<line x1="${left}" x2="${right}" y1="${height-30}" y2="${height-30}"/>${[0,.5,1].map(f=>`<text x="${left+f*(right-left)}" y="${height-10}" text-anchor="${f===0?'start':f===1?'end':'middle'}">${fmt(f*range,3)} s</text>`).join('')}</svg><p class="note">时间轴起点：${fmt(origin,6)} s</p>`;
  }
  function axesPlot(id,matrix,sourceName){
    const cc=canvasContext(id);if(!cc||!matrix)return;const {ctx,w,h}=cc;
    const m=matrix,base=[0,0,0],other=m.slice(0,3).map(r=>r[3]),axisLength=.15;
    const endpoints=[base,other,...[0,1,2].map(k=>{const p=[0,0,0];p[k]=axisLength;return p;}),...[0,1,2].map(k=>other.map((v,j)=>v+m[j][k]*axisLength))];
    const yaw=.7,pitch=.6,rotate=p=>{const x=p[0]*Math.cos(yaw)-p[1]*Math.sin(yaw),y=p[0]*Math.sin(yaw)+p[1]*Math.cos(yaw);return[x,y*Math.cos(pitch)-p[2]*Math.sin(pitch)];};
    const rotated=endpoints.map(rotate),mins=[0,1].map(k=>Math.min(...rotated.map(p=>p[k]))),maxs=[0,1].map(k=>Math.max(...rotated.map(p=>p[k]))),center=mins.map((v,k)=>(v+maxs[k])/2),scale=Math.min((w-140)/Math.max(maxs[0]-mins[0],.05),(h-90)/Math.max(maxs[1]-mins[1],.05));
    const screen=p=>{const r=rotate(p);return[w/2+(r[0]-center[0])*scale,h/2-(r[1]-center[1])*scale];};
    ctx.font='12px system-ui';ctx.lineWidth=2;
    for(const [origin,offset,label] of [[base,2,'RGB光学坐标系'],[other,5,sourceName]]){
      const xy=screen(origin);ctx.fillStyle='#d6e7ee';ctx.fillText(label,xy[0]-45,xy[1]+26);
      for(let k=0;k<3;k++){const point=screen(endpoints[offset+k]);ctx.strokeStyle=['#70d8c2','#7bb5f2','#efc17f'][k];ctx.fillStyle=ctx.strokeStyle;ctx.beginPath();ctx.moveTo(...xy);ctx.lineTo(...point);ctx.stroke();ctx.fillText(['X','Y','Z'][k],point[0]+5,point[1]);}
    }
    ctx.fillStyle='#99adbc';ctx.fillText('记录变换 · 轴长 0.15 m · 三维斜投影',14,20);
  }
  function inputTable(s){
    const relevant=branch==='pmf'?['rgb','registered_depth','lidar']:branch==='rgbnir'?['rgb','nir','registered_depth']:['rgb','audio'];
    const rows=relevant.filter(k=>s.interface_example[k]).map(k=>{const r=s.interface_example[k];return[k,r.topic||r.source_id||(k==='audio'?'工作站声学输入':'相机 / LiDAR数据'),r.encoding||(k==='audio'?`${r.rate} Hz / ${r.channels} ch`:k==='lidar'?'PointCloud2 / '+(r.fields||[]).join(','):r.width+' × '+r.height),r.frame||r.quality,fmt(r.raw_stamp??r.stamp,6)];});
    return '<details><summary>输入来源、格式、坐标与时间字段</summary>'+table(['模态','来源','格式 / 尺寸','坐标 / 时间质量','记录时间 / s'],rows)+'<p class="note">接口示例来自来源快照，实际样本配对时间见“时间与接口”。</p></details>';
  }
  function dots(jpg,uv,label) {
    if(!jpg||!uv?.length)return figure(jpg,label);
    // Native dimensions read from the image, keeping coordinates independent of CSS.
    return `<figure><div class="image-dots"><img src="data:image/jpeg;base64,${esc(jpg)}" alt="${esc(label)}" data-dots='${esc(JSON.stringify(uv))}'><svg aria-hidden="true"></svg></div><figcaption>${esc(label)} · 抽样对应点</figcaption></figure>`;
  }
  function mountDots(){document.querySelectorAll('[data-dots]').forEach(img=>{const draw=()=>{const svg=img.nextElementSibling;svg.setAttribute('viewBox',`0 0 ${img.naturalWidth} ${img.naturalHeight}`);const uv=JSON.parse(img.dataset.dots);svg.innerHTML=uv.map((p,i)=>`<circle cx="${p[0]}" cy="${p[1]}" r="3"/><text x="${p[0]+4}" y="${p[1]-4}">${i+1}</text>`).join('');};if(img.complete)draw();else img.onload=draw;});}
  function selectFeatures(s,choices,label='特征层') {
    const available=choices.filter(n=>feature(s,n));if(!available.length)return '';
    let choice=featureChoice[branch];if(!available.includes(choice))choice=available[0];featureChoice[branch]=choice;
    return `<div class="local-controls"><label for="feature-select">${esc(label)}</label><select id="feature-select">${available.map(n=>`<option value="${esc(n)}"${choice===n?' selected':''}>${esc(n)} · ${esc(shape(feature(s,n)))}</option>`).join('')}</select></div>`;
  }
  function setEvidence(s) {
    if(branch==='coverage'){$('evidence').innerHTML='<p>每项要求可以在不同分支查看。已保存的处理证据、现场能力验证和精度评测分别确认。</p>'+note('没有新输入的分支使用已保存样本，不自动宣称四类传感器同时在线。');return;}
    if(!s){$('evidence').innerHTML='<p>等待该分支生成可对应的输入与特征归档。</p>';return;}
    const context=s.context||{},b=status?.branches?.[branch];
    const values=[['分支',names[branch]],['样本序号',s.sequence],['输入时间',stamp(s.source_stamp)],['会话',s.session_id.slice(0,12)],['当前采集状态',b?.detail||'未读取'],['标定记录',context.calibration||'本样本未归档空间标定'],['配对最大时间差',fmt(s.timing.maximum_gap_ms)+' ms'],['本次模型计算耗时',fmt(s.inference_ms,1)+' ms'],['输出形状',(s.output_shape||[]).join(' × ')]];
    $('evidence').innerHTML='<dl>'+values.map(([k,v])=>`<div><dt>${esc(k)}</dt><dd>${esc(v)}</dd></div>`).join('')+'</dl><p class="note">'+esc(context.geometry_verified?'标定记录已测量，查看对应证据。':branch==='pmf'?'现有LiDAR外参为估计参数，尚未完成标定验收。':branch==='rgbnir'?'相机内参和外参来自工厂参数记录，现场配准效果需另行验证。':'声学时间戳质量：'+(context.stamp_quality||'未记录'))+'</p><a href="/research/'+branch+'_features.npz">下载原始特征归档 ↗</a><details><summary>本样本模型与处理来源</summary><pre>'+esc(JSON.stringify(s.model,null,2))+'</pre></details>';
  }
  function coverage() {
    const items=[['四类模态输入','RGB、近红外、真实LiDAR与声学分别在三条分支输入端展示。',[['pmf',0],['rgbnir',0],['avs',0]]],['特征提取、拼接与融合','提取：三条分支；拼接：颜色＋法向量、视觉token；融合：几何交互、加权、声学条件交互。',[['pmf',4],['rgbnir',3],['avs',4]]],['接口与数据同步','来源、格式、尺寸、特征形状、坐标系、输入组与输出关系。',[['pmf',2],['rgbnir',2],['avs',2]]],['时间同步与融合精度','时间轴和配对时间差已有记录；硬件同步误差、融合精度与增益仍需实测评价。',[['pmf',2],['rgbnir',5],['avs',5]]],['标定、时间对齐与预处理','相机参数、外参、点—像素映射、配准有效区、点云筛选及声学预处理。',[['pmf',1],['rgbnir',1],['avs',1]]],['特征尺度匹配与跨模态语义对齐','近红外展示尺寸与RMS幅值匹配；声学展示128→256及声学条件查询对视觉token的注意力。',[['rgbnir',4],['avs',4]]]];
    return '<div class="coverage-list">'+items.map((r,i)=>`<div class="coverage-item"><span class="index">${i+1}</span><div><strong>${esc(r[0])}</strong><p>${esc(r[1])}</p>${r[2].map(([b,st])=>`<button data-jump="${b}:${st}">${esc(names[b])} ↗</button>`).join('')}</div></div>`).join('')+'</div>'+note('共享骨干和维度统一只能说明结构兼容；声学条件注意力可展示跨模态语义交互机制，语义对齐效果需要与任务输出及评测记录共同验证。');
  }
  function stageMarkup(s) {
    const v=s.visuals,t=s.tables,img=s.images,c=s.context;
    if(step===0){
      if(branch==='pmf')return '<div class="gallery">'+figure(img.raw,'RGB图像')+(v.filter?plot('raw-cloud','真实LiDAR原始扫描 · '+v.filter.frame):v.cloud?plot('scene-cloud','已保留的真实LiDAR点 · icp_odom'):empty('该样本无LiDAR归档'))+'</div>'+inputTable(s)+note('深度参与几何处理；点云来自LiDAR采集归档。');
      if(branch==='rgbnir')return '<div class="gallery">'+figure(img.raw,'RGB图像')+figure(v.nir_raw,'原始近红外 · mono8')+'</div>'+inputTable(s)+note('数据来源与时间来自本分支保存记录，当前在线状态见上方。');
      return '<div class="gallery">'+figure(img.raw,'匹配窗口内的RGB帧')+(v.waveform?plot('waveform','WheelTec原始PCM波形',true):figure(null,'原始PCM波形'))+'</div><div style="margin-top:16px">'+figure(v.log_mel||img.spectrum,'实际log-Mel频谱')+'</div>'+inputTable(s)+(!v.waveform?note('旧样本仅保存log-Mel；新增采集会保留原始PCM，波形与功率频谱随后显示。'):'');
    }
    if(step===1){
      if(branch==='pmf')return '<div class="gallery">'+dots(img.raw,v.correspondence?.uv,'LiDAR投影到RGB')+plot('filter-cloud','同一扫描筛选前后 · '+(v.filter?.frame||''))+'</div>'+table(['对应点','X / m','Y / m','Z / m','u / px','v / px'],(v.correspondence?.xyz||[]).slice(0,8).map((p,i)=>[i+1,...p.map(x=>fmt(x)),...v.correspondence.uv[i].map(x=>fmt(x,1))]))+note(v.filter?.note||'此样本没有同扫描筛选记录')+'<details><summary>相机参数与坐标变换</summary>'+matrix('RGB内参 K',t.intrinsics)+matrix('camera_from_lidar · 平移单位 m',t.extrinsics)+note(c.calibration_evidence||'无独立标定证据')+'</details>';
      if(branch==='rgbnir')return '<div class="gallery three">'+figure(v.nir_raw,'配准前近红外')+figure(v.nir_registered,'配准到RGB网格')+figure(v.valid_mask,'有效区域','白色有效；黑色不使用近红外')+'</div>'+table(['对应点','RGB u','RGB v','NIR u','NIR v'],(v.correspondence?.rgb_uv||[]).slice(0,8).map((p,i)=>[i+1,...p.map(x=>fmt(x,1)),...v.correspondence.nir_uv[i].map(x=>fmt(x,1))]))+'<details><summary>相机参数与坐标变换</summary>'+matrix('RGB内参 K',t.intrinsics)+matrix('nir_from_rgb · 平移单位 m',t.extrinsics)+note('标定版本：'+c.calibration)+'</details>';
      return flow([['PCM','采样率 '+(v.waveform?.rate||c.sample_rate||'未归档')+' Hz'],['STFT','时频变换'],['log-Mel','96 × 64 / 片段']])+'<div class="gallery">'+(v.waveform?plot('waveform','原始波形 · 幅值',true):empty('该样本未保存原始PCM'))+figure(v.log_mel,'log-Mel · 时间×频率响应')+'</div>'+(v.spectrum?plot('spectrum','实际PCM Welch功率谱密度',true):note('功率频谱将在新的PCM归档产生后显示。'))+note('工作站声学时间戳采用采集端记录；空间定位、麦克风阵列标定或口令执行没有在该流程中验证。');
    }
    if(step===2){
      const rows=s.timing.rows||[];
      return timeline(s.timing)+table(['输入组','RGB时间 / s','另一模态时间 / s','配对差 / ms'],rows.map((r,i)=>[i+1,fmt(r.a,6),fmt(r.b,6),fmt(r.gap_ms,3)]))+'<details><summary>原始时间轴与时钟校正记录</summary>'+timeline(s.timing,true)+'<pre>'+esc(JSON.stringify(s.timing.explicit_offsets||s.timing.clock,null,2))+'</pre></details>'+note(s.timing.note)+'<details><summary>接口字段与数据流</summary>'+flow([['输入','来源 / 时间 / 坐标'],['特征','形状 / 层级 / 归档'],['输出','结果 / 同一样本标识']])+table(['特征字段','归档形状','处理方式'],allFeatures(s).map(f=>[f.name,shape(f),f.operation]))+'<p class="note">以下为来源快照中的接口字段示例，采集时间可能晚于当前归档样本。</p><pre>'+esc(JSON.stringify(s.interface_example,null,2))+'</pre></details>';
    }
    if(step===3){
      if(branch==='rgbnir'){const controls=selectFeatures(s,['P3.rgb','P4.rgb','P5.rgb']);const level=(featureChoice.rgbnir||'P3.rgb').split('.')[0];return controls+'<div class="gallery">'+heat(s,level+'.rgb',level+' RGB原始特征')+heat(s,level+'.nir',level+' 近红外原始特征')+'</div>'+note('热图使用各张特征图自身范围归一化，只展示空间响应；幅值比较请查看下一步RMS数值。');}
      if(branch==='pmf'){const controls=selectFeatures(s,allFeatures(s).filter(f=>f.heatmap_layout!=='point_index_grid'&&f.heatmap).map(f=>f.name));return controls+'<div class="gallery">'+heat(s,featureChoice.pmf,'图像/跨视角特征响应')+(v.geometry?plot('geometry-cloud','真实查询XYZ上的三维特征能量'):empty('点特征空间映射未归档'))+'</div>'+note('三维特征通过归档的点—体素索引关联到真实XYZ，点序网格不作为空间投影图。');}
      const controls=selectFeatures(s,['visual.scale1','visual.scale2','visual.scale3','visual.scale4']);return flow([['128','声学嵌入'],['256','声学维度投影'],['256','视觉各尺度通道']])+'<div class="gallery">'+plot('audio-vectors','声学嵌入/投影的逐维数值',true)+heat(s,featureChoice.avs||'visual.scale1','视觉尺度特征')+'</div>'+controls+note('图中的维度投影是实际特征处理；统一维度本身不证明跨模态语义一致。');
    }
    if(step===4){
      if(branch==='pmf'){const concat=t.concatenation;return flow([['3','RGB归一化颜色'],['3','LiDAR表面法向量'],['6','沿特征维拼接']],true)+table(['点索引','R','G','B','nx','ny','nz'],(concat?.values||[]).map((r,i)=>[concat.indices[i],...r.map(x=>fmt(x,4))]))+note(concat?.origin||'无拼接输入归档')+note(concat?.note||'本样本未记录该对照分支')+note('主分支执行RGB-D几何交互与LiDAR点查询；主分支、对照分支的特征分别处理。');}
      if(branch==='rgbnir'){const controls=selectFeatures(s,['P3.rgb','P4.rgb','P5.rgb'],'特征尺度');const level=(featureChoice.rgbnir||'P3.rgb').split('.')[0];return controls+'<div class="gallery">'+heat(s,level+'.rgb','RGB特征')+heat(s,level+'.nir','近红外匹配前')+heat(s,level+'.nir_scaled','近红外幅值匹配后')+heat(s,level+'.fused','加权融合特征')+'</div>'+table(['层级 / 形状','RGB RMS','NIR RMS','匹配增益','有效区域','NIR权重'],s.feature_statistics.map(r=>[r.level+' / '+r.shape.join('×'),fmt(r.rgb_rms,4),fmt(r.nir_rms,4),fmt(r.nir_rms_gain,4),fmt(r.valid_fraction*100,1)+'%',fmt(r.nir_weight,2)]))+note('有效区域采用(1−α)RGB＋α·匹配后NIR；无效区域保留RGB。本分支没有独立训练的近红外语义对齐模型。');}
      return flow([['784 + 196 + 49','视觉三尺度token'],['1029 × 256','沿序列维拼接'],['300 × 256','声学条件查询']])+'<div class="gallery">'+figure(img.raw,'匹配的RGB帧')+figure(v.audio_attention,'声学Q—视觉K空间注意力','实际投影激活复算；注意力头平均')+'</div>'+(v.query_attention_28?'<h3 style="margin-top:20px">声学条件查询与各尺度视觉token的关联</h3><div class="gallery three">'+figure(v.query_attention_28,'28 × 28尺度')+figure(v.query_attention_14,'14 × 14尺度')+figure(v.query_attention_7,'7 × 7尺度')+'</div>':note('此旧样本未保存注意力权重；新推理样本生成后显示真实注意力图。'))+(t.channel_weights?plot('channel-weights','实际sigmoid通道权重',true):'')+note('声学通过条件查询和注意力参与视觉交互；视觉token之间进行拼接。该图展示语义交互机制，语义对齐效果尚需评测。');
    }
    let gallery='<div class="gallery">'+figure(img.annotated_overlay||img.overlay,'融合结果');
    gallery+=branch==='rgbnir'?figure(img.reference_overlay,'同样本RGB对照'):branch==='pmf'?figure(img.reference_overlay,'独立颜色＋法向量对照'):figure(img.raw,'对应RGB输入');gallery+='</div>';
    if(branch==='avs')gallery='<div class="gallery">'+heat(s,'visual.before_mixer','声学交互前的视觉特征')+heat(s,'audiovisual.fused','声学交互后的视觉特征')+'</div><div style="margin-top:16px">'+gallery+'</div>';
    return gallery+table(['输出类别','预测点/像素数','预测占比'],s.classes.map(r=>[r.display_label||r.label,r.count,fmt(r.fraction*100,1)+'%']))+table(['验证项目','记录状态'],[['本次模型计算耗时',fmt(s.inference_ms,1)+' ms（不等同于端到端处理周期）'],['配对最大时间差',fmt(s.timing.maximum_gap_ms,3)+' ms（不等同于硬件同步误差）'],['融合与最佳单模态精度','待同样本评测'],['任务周期限值与精度增益','待任务要求及实测记录']])+note(branch==='pmf'?'此处对照是两个多源模型的结果差异，不能作为融合与最佳单模态精度比较。':s.evaluation.note);
  }
  function drawPlots(s){
    if(!s)return;const v=s.visuals,t=s.tables;
    if($('raw-cloud')&&v.filter)cloudPlot('raw-cloud',{xyz:v.filter.before,count:v.filter.before_count});
    if($('scene-cloud'))cloudPlot('scene-cloud',v.cloud);
    if($('geometry-cloud'))cloudPlot('geometry-cloud',v.geometry);
    if($('filter-cloud')&&v.filter){const rgb=[...v.filter.before.map(()=>[75,102,126]),...v.filter.after.map(()=>[112,216,194])];cloudPlot('filter-cloud',{xyz:[...v.filter.before,...v.filter.after],rgb,count:v.filter.before_count});}
    if($('waveform')&&v.waveform)lineChart('waveform',[{color:'#70d8c2',points:v.waveform.envelope.map(r=>[r[0],r[1]])},{color:'#7bb5f2',points:v.waveform.envelope.map(r=>[r[0],r[2]])}],'时间 / s','幅值');
    if($('spectrum')&&v.spectrum)lineChart('spectrum',[{color:'#70d8c2',points:v.spectrum.hz.map((hz,i)=>[hz,v.spectrum.db[i]])}],'频率 / Hz','PSD / dB/Hz');
    if($('audio-vectors'))lineChart('audio-vectors',Object.entries(t.audio_vectors||{}).map(([name,values],i)=>({color:i?'#7bb5f2':'#70d8c2',points:values.map((x,j)=>[j,x])})),'特征维度','数值');
    if($('channel-weights')&&t.channel_weights)lineChart('channel-weights',[{color:'#70d8c2',points:t.channel_weights.map((x,i)=>[i,x])}],'通道','权重');
    if($('calibration-axes'))axesPlot('calibration-axes',t.extrinsics,branch==='pmf'?'LiDAR坐标系':'近红外参考坐标系');
    mountDots();
  }
  function render(){
    plotObserver?.disconnect();
    document.querySelectorAll('[data-branch]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.branch===branch)));
    $('stages').innerHTML=branch==='coverage'?'':steps.map((name,i)=>`<button data-step="${i}" aria-pressed="${i===step}"><span class="step-number">0${i+1}</span>${name}</button>`).join('');
    $('stage-kicker').textContent=branch==='coverage'?'TASK BOOK COVERAGE':names[branch]+' · STEP 0'+(step+1);
    $('stage-title').textContent=branch==='coverage'?'三条分支共同覆盖任务书':titles[branch][step];
    $('stage-description').textContent=branch==='coverage'?'点击条目跳转到对应分支证据。技术性能仍以实测记录判定。':descriptions[branch][step];
    $('stage-tag').textContent=branch==='coverage'?'六项要求':step===5?'精度待测':'实际处理记录';
    const s=samples[branch];$('visual').innerHTML=branch==='coverage'?coverage():s?stageMarkup(s):empty('等待该分支的同会话输入与特征归档；其他分支可独立查看。');
    if(s&&step===1&&branch!=='avs'&&s.tables.extrinsics){
      const summary=[...$('visual').querySelectorAll('summary')].find(e=>e.textContent.includes('相机参数'));
      if(summary){summary.insertAdjacentHTML('afterend',plot('calibration-axes','内外参记录对应的传感器坐标轴'));summary.nextElementSibling.classList.add('calibration-plot');}
    }
    if(step===3||step===4)$('visual').querySelectorAll('.gallery').forEach(e=>e.classList.add('feature-gallery'));
    $('download').disabled=!s;setEvidence(s);updateStatus();
    const select=$('feature-select');if(select)select.onchange=()=>{featureChoice[branch]=select.value;render();};
    drawPlots(s);
    plotObserver=new ResizeObserver(()=>{document.querySelectorAll('canvas').forEach(c=>{if(c._redraw)c._redraw();});if(s){
      // Canvas line plots need their CSS dimensions after a responsive reflow.
      drawLines(s);if($('calibration-axes'))axesPlot('calibration-axes',s.tables.extrinsics,branch==='pmf'?'LiDAR坐标系':'近红外参考坐标系');
    }});document.querySelectorAll('canvas').forEach(c=>plotObserver.observe(c));
  }
  function drawLines(s){const v=s.visuals,t=s.tables;if($('waveform')&&v.waveform)lineChart('waveform',[{color:'#70d8c2',points:v.waveform.envelope.map(r=>[r[0],r[1]])},{color:'#7bb5f2',points:v.waveform.envelope.map(r=>[r[0],r[2]])}],'时间 / s','幅值');if($('spectrum')&&v.spectrum)lineChart('spectrum',[{color:'#70d8c2',points:v.spectrum.hz.map((x,i)=>[x,v.spectrum.db[i]])}],'频率 / Hz','PSD / dB/Hz');if($('audio-vectors'))lineChart('audio-vectors',Object.values(t.audio_vectors||{}).map((values,i)=>({color:i?'#7bb5f2':'#70d8c2',points:values.map((x,j)=>[j,x])})),'特征维度','数值');if($('channel-weights')&&t.channel_weights)lineChart('channel-weights',[{color:'#70d8c2',points:t.channel_weights.map((x,i)=>[i,x])}],'通道','权重');}
  function updateStatus(){
    const s=samples[branch],b=status?.branches?.[branch],recent=status&&Date.now()/1000-status.published_at<15;
    const saved=!recent||!['running','ok','ready'].includes(b?.state);
    if(branch==='coverage'){$('sample-status').innerHTML=Object.entries(names).map(([key,name])=>`<span class="tag ${samples[key]?'saved':''}">${esc(name)} · ${samples[key]?'已有样本':'等待归档'}</span>`).join('');return;}
    $('sample-status').innerHTML=`<span class="tag ${saved?'saved':'live'}">${frozen?'已冻结样本':s?saved?'已保存样本 · 当前等待输入':'正在接收记录':'等待样本'}</span>`+(s?`<span>序号 ${esc(s.sequence)} · ${esc(stamp(s.source_stamp))} · 同会话归档</span>`:'')+`<span>${esc(!recent?'采集/导出服务当前未更新':b?.detail||'等待3001服务')}</span>`+(b?.exporter_error?`<span class="error">归档检查：${esc(b.exporter_error)}</span>`:'');
  }
  async function fetchJson(url){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),5000);try{const r=await fetch(url+'?t='+Date.now(),{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error('HTTP '+r.status);return await r.json();}finally{clearTimeout(timer);}}
  async function poll(){if(polling||frozen||document.hidden)return;polling=true;try{const nextStatus=await fetchJson('/research/process/status.json');if(frozen)return;status=nextStatus;for(const task of Object.keys(names)){
    const identity=status.branches?.[task]?.sample_identity;
    if(identity&&identity.join(':')!==sampleId(samples[task])&&!pending[task]){pending[task]=true;try{const s=await fetchJson('/research/process/'+task+'.json');if(s.schema!=='research_process/1'||s.task!==task)throw Error('Invalid process snapshot');if(frozen)return;samples[task]=s;if(task===branch)render();}finally{pending[task]=false;}}
    }updateStatus();setEvidence(samples[branch]);}catch(error){if(!samples[branch])$('sample-status').textContent='尚未取得过程导出记录，可继续查看已有分支；启动3001后会自动读取。';else updateStatus();}finally{polling=false;}}
  document.addEventListener('click',e=>{const button=e.target.closest('button');if(!button)return;if(button.dataset.branch){branch=button.dataset.branch;step=0;render();}if(button.dataset.step){step=Number(button.dataset.step);render();}if(button.dataset.jump){const parts=button.dataset.jump.split(':');branch=parts[0];step=Number(parts[1]);render();}});
  $('freeze').onclick=()=>{frozen=!frozen;$('freeze').setAttribute('aria-pressed',String(frozen));$('freeze').textContent=frozen?'恢复自动更新':'冻结当前样本';updateStatus();if(!frozen)poll();};
  $('download').onclick=()=>{const s=samples[branch];if(!s)return;const blob=new Blob([JSON.stringify({...s,display_capture:{frozen,stage:steps[step],captured_at:new Date().toISOString()}},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`${branch}-process-${s.sequence}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});render();poll();setInterval(poll,4000);
})();
