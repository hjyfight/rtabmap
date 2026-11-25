/* Same-origin, read-only explorer. Presentation state never sends ROS commands. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const names = {pmf:'RGB + LiDAR（深度辅助）',rgbnir:'RGB + 近红外',avs:'RGB + 声学'};
  const steps = ['模态输入','标定与预处理','时间与接口','特征提取','匹配、拼接与对齐','融合结果与对照'];
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
  let visualDisclosureScope=null,evidenceDisclosureScope=null;
  const disclosureState=new Map(),disclosureStorageKey='research-process-disclosures-v1';
  try{
    const stored=JSON.parse(sessionStorage.getItem(disclosureStorageKey)||'{}');
    for(const [scope,values] of Object.entries(stored||{})){
      if(values&&typeof values==='object'&&!Array.isArray(values))
        disclosureState.set(scope,new Map(Object.entries(values).filter(([key,value])=>typeof value==='boolean')));
    }
  }catch(_){/* Storage may be unavailable; in-memory state still works. */}
  function persistDisclosures(){
    try{sessionStorage.setItem(disclosureStorageKey,JSON.stringify(Object.fromEntries(
      [...disclosureState].map(([scope,values])=>[scope,Object.fromEntries(values)]))));}catch(_){}
  }
  function captureDisclosures(root,scope){
    if(!root||!scope)return;
    const states=disclosureState.get(scope)||new Map();let changed=false;
    root.querySelectorAll('details[data-disclosure]').forEach(detail=>{
      const key=detail.dataset.disclosure;
      if(states.get(key)!==detail.open){states.set(key,detail.open);changed=true;}
    });
    if(changed){disclosureState.set(scope,states);persistDisclosures();}
  }
  function restoreDisclosures(root,scope){
    if(!root||!scope)return;
    root.querySelectorAll('details[data-disclosure]').forEach(detail=>{
      const key=detail.dataset.disclosure,states=disclosureState.get(scope);
      if(states?.has(key))detail.open=states.get(key);
      detail.addEventListener('toggle',()=>{
        // A queued event from an old, replaced subtree must not overwrite state.
        if(detail.isConnected===false)return;
        const current=disclosureState.get(scope)||new Map();
        if(current.get(key)!==detail.open){
          current.set(key,detail.open);disclosureState.set(scope,current);persistDisclosures();
        }
      });
    });
  }
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
  const sampleId = s => s?[s.session_id,s.sequence,s.validation_revision].filter(x=>x!=null).join(':'):'';
  const stamp = t => Number.isFinite(t)?new Date(t*1000).toLocaleString('zh-CN',{hour12:false,timeZone:'Asia/Shanghai'}):'—';
  const mapping=window.ResearchMapping.create({$,esc,fmt,empty,note,figure,plot,table,flow,cloudPlot,canvasContext,stamp});
  function standard(value){
    const active=value==='mapping';
    if(!mapping.isActive()){captureDisclosures($('visual'),visualDisclosureScope);captureDisclosures($('evidence'),evidenceDisclosureScope);}
    visualDisclosureScope=null;evidenceDisclosureScope=null;
    plotObserver?.disconnect();mapping.activate(active);
    document.querySelectorAll('[data-standard]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.standard===value)));
    $('fusion-branches').hidden=active;$('mapping-branches').hidden=!active;
    $('page-title').textContent=active?'看见环境数据如何形成地图':'看见输入如何成为融合结果';
    $('page-intro').textContent=active?'按建图大纲查看输入、环境要素、关联与优化、地图输出及验证记录。':'三条分支共同展示任务书流程，按实际处理环节逐步查看。';
    $('page-kicker').textContent=active?'ENVIRONMENT MAPPING EXPLORER':'MULTIMODAL PROCESS EXPLORER';
    if(!active){$('freeze').textContent=frozen?'恢复自动更新':'冻结当前样本';$('freeze').setAttribute('aria-pressed',String(frozen));render();poll();}
    const url=new URL(location.href);active?url.searchParams.set('standard','mapping'):url.searchParams.delete('standard');history.replaceState(null,'',url);
  }

  const checkLabel = state => ({pass:'核对通过',fail:'不满足 / 不一致',missing:'归档缺失',pending:'待验证',not_applicable:'不适用',defined:'已明确'})[state]||'未记录';
  const checkTag = (state,label) => `<span class="check-state ${esc(state||'pending')}">${esc(label||checkLabel(state))}</span>`;
  function interfaceMarkup(s){
    const audit=s.interface_validation;
    if(!audit)return note('该旧样本尚未生成接口规范与核查记录。');
    const contracts=(audit.protocol?.layers||[]).map(layer=>'<details class="contract" data-disclosure="contract-'+esc(layer.name)+'"><summary>'+esc(layer.name)+' · '+esc(layer.transport)+'</summary>'+table(['字段','类型','单位','要求','含义'],layer.fields.map(f=>[f.label+' · '+f.key,f.type,f.unit,f.required,f.meaning]))+'</details>').join('');
    const inputs=(audit.inputs||[]).map((r,i)=>[i+1,r.modality,r.topic,r.source_id,r.format,(r.shape||[]).join('×')+' / '+r.dtype,r.frame||'不涉及空间坐标',fmt(r.raw_stamp,6),fmt(r.aligned_stamp,6),r.stamp_quality]);
    const features=(audit.features||[]).map(f=>[f.name,f.archive_key,(f.full_shape||[]).join('×'),(f.saved_shape||[]).join('×'),f.dtype,f.operation]);
    return '<section class="verification-block"><h3>接口规范与当前批次核查</h3><div class="verification-summary">'+checkTag(audit.state)+'<span>通过 '+audit.counts.pass+' · 不一致 '+audit.counts.fail+' · 缺失 '+audit.counts.missing+'</span></div><p class="note mono">批次：'+esc(audit.batch_key)+'</p>'+contracts+'<details data-disclosure="inputs-used"><summary>本次实际参与处理的输入字段（'+inputs.length+'条）</summary>'+table(['输入','模态','话题','来源标识','格式','形状 / 类型','坐标系','原始时间 / s','对齐时间 / s','时间质量'],inputs)+'<p class="note">字段来自选中输入的归档；重叠声学窗口只记录实际贡献的采样区间。旧归档缺少的字段明确列为缺失。</p></details><details data-disclosure="feature-arrays"><summary>中间特征接口与数组对应（'+features.length+'项）</summary>'+table(['特征','数组键','完整模型形状','保存形状','类型','处理模块'],features)+'</details><details data-disclosure="interface-checks"><summary>逐项字段与输入—特征—输出核查</summary>'+table(['环节','核查项','状态','实际记录'],audit.checks.map(c=>[c.layer,c.name,checkLabel(c.state),c.detail]))+'</details>'+note(audit.note)+'</section>';
  }
  function syncDetailsMarkup(s){
    const v=s.sync_validation;if(!v)return note('该旧样本尚未生成同步指标核查记录。');
    const stats=v.statistics||{},measurement=v.measurement;
    const requirementRows=[['通用同步误差限值',fmt(v.sync_limit_ms)+' ms',v.reference],['当前配对核查门限',fmt(v.pair_limit_ms)+' ms','以校正后输入时间差核查，不等同于同步精度'],['本分支实际运行容差',fmt(v.runtime_tolerance_ms)+' ms','归档中的推理配对参数，不作为验收限值']];
    const measuredRows=[['同步误差测量状态',checkLabel(v.measurement_state)],['实测最大误差',measurement?fmt(measurement.max_error_ms)+' ms':'尚无覆盖采集时间的测量记录'],['测量不确定度',measurement?fmt(measurement.uncertainty_ms)+' ms':'—'],['测量方法',measurement?.method||'—'],['证据编号 / 文件',measurement?.evidence||'—'],['本批次同步精度结论',checkLabel(v.state)]];
    return '<section class="verification-block"><h3>时间同步要求与测量</h3>'+table(['指标 / 参数','规定值或状态','依据 / 口径'],requirementRows)+'<p class="formula mono">'+esc(v.formula)+'</p><div class="verification-summary">'+checkTag(v.pair_state,v.pair_state==='pass'?'配对门限核查通过':v.pair_state==='fail'?'存在超门限配对':'配对门限待核查')+'<span>'+stats.count+'组 · 最大 '+fmt(stats.maximum_ms)+' ms · 超门限 '+(stats.over_limit_count??'—')+'组</span></div>'+table(['配对统计','实测值'],[['平均时间差',fmt(stats.mean_ms)+' ms'],['P95时间差',fmt(stats.p95_ms)+' ms'],['最大时间差',fmt(stats.maximum_ms)+' ms']])+table(['同步精度测量','记录'],measuredRows)+'<details data-disclosure="clock-quality"><summary>时间基准与偏移测量质量</summary>'+table(['记录项','本次记录'],[['时间基准',v.timebase],['偏移方法',v.clock_method],['往返估计不确定度',v.clock_uncertainty_ms!=null?'±'+fmt(v.clock_uncertainty_ms)+' ms':'未提供有界估计'],['偏移观测跨度',v.clock_spread_ms!=null?fmt(v.clock_spread_ms)+' ms':'—'],['采集时间质量',v.acquisition_timestamp_quality]])+'<pre>'+esc(JSON.stringify(s.timing.explicit_offsets||{},null,2))+'</pre></details>'+(v.measurement_error?note('同步测量记录未采用：'+v.measurement_error):'')+note(v.note)+'</section>';
  }

  function pairingChart(s,limit){
    const values=(s.timing?.rows||[]).map(r=>Number.isFinite(r.a)&&Number.isFinite(r.b)?Math.abs(r.a-r.b)*1000:null);
    if(!values.some(Number.isFinite))return '';
    const w=720,h=180,left=45,right=700,top=20,bottom=145;
    const domain=Math.max(10,Math.ceil(Math.max(limit||0,...values.filter(Number.isFinite))*1.2/10)*10);
    const y=v=>bottom-v/domain*(bottom-top),cell=(right-left)/values.length,width=Math.min(44,cell*.58);
    return '<figure class="pairing-chart"><svg viewBox="0 0 '+w+' '+h+'" role="img" aria-label="各组配对时间差与门限">'+[0,domain/2,domain].map(v=>'<line x1="'+left+'" x2="'+right+'" y1="'+y(v)+'" y2="'+y(v)+'" class="chart-grid"/><text x="'+(left-8)+'" y="'+(y(v)+4)+'" text-anchor="end">'+fmt(v,0)+'</text>').join('')+(Number.isFinite(limit)?'<line x1="'+left+'" x2="'+right+'" y1="'+y(limit)+'" y2="'+y(limit)+'" class="chart-limit"/><text x="'+right+'" y="'+(y(limit)-6)+'" text-anchor="end">门限 '+fmt(limit,0)+' ms</text>':'')+values.map((v,i)=>Number.isFinite(v)?'<rect x="'+(left+cell*(i+.5)-width/2)+'" y="'+Math.min(bottom-2,y(v))+'" width="'+width+'" height="'+Math.max(2,bottom-y(v))+'" class="'+(Number.isFinite(limit)&&v>limit+1e-6?'over-limit':'within-limit')+'"><title>第'+(i+1)+'组：'+fmt(v)+' ms</title></rect><text x="'+(left+cell*(i+.5))+'" y="166" text-anchor="middle">'+(i+1)+'</text>':'').join('')+'<text x="8" y="13">ms</text></svg><figcaption>各组配对时间差 · 横轴为输入组号</figcaption></figure>';
  }
  function syncMarkup(s){
    const v=s.sync_validation;if(!v)return note('等待本样本的同步记录。');
    const stats=v.statistics||{},result=v.pair_state==='pass'?'满足配对门限':v.pair_state==='fail'?'存在超限配对':'暂无配对记录';
    const measured=v.state==='pass'?'已验证，满足限值':v.state==='fail'?'实测未满足限值':'待验证';
    return '<section class="verification-block sync-core"><h3>时间同步</h3><p class="note">时间基准：按本样本记录的采集时间与偏移配对。</p><div class="sync-core-grid"><div class="core-metric"><span>同步限值</span><strong>≤ '+fmt(v.sync_limit_ms,0)+' <small>ms</small></strong></div><div class="core-metric"><span>最大配对时间差</span><strong>'+fmt(stats.maximum_ms,2)+' <small>ms</small></strong></div><div class="core-metric core-status"><span>配对核查结果</span>'+checkTag(v.pair_state,result)+'</div></div><p class="core-context">本次 '+stats.count+' 组配对'+(stats.over_limit_count?'，'+stats.over_limit_count+' 组超限':'')+' · 同步精度：'+esc(measured)+'</p>'+pairingChart(s,v.pair_limit_ms)+'<p class="note">配对核查按时间戳计算；同步精度依据独立测量记录。</p></section><details class="record-details" data-disclosure="sync-statistics"><summary>查看同步测量与统计详情</summary>'+syncDetailsMarkup(s)+'</details>';
  }

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
    const relevant=branch==='pmf'?['rgb','depth','lidar']:branch==='rgbnir'?['rgb','nir','depth']:['rgb','audio'];
    const rows=relevant.filter(k=>s.interface_example[k]).map(k=>{const r=s.interface_example[k];return[k,r.topic,r.format+' / '+(r.shape||[]).join('×'),r.frame||r.stamp_quality,fmt(r.raw_stamp,6)];});
    return '<details data-disclosure="sample-inputs"><summary>本样本输入来源、格式、坐标与时间字段</summary>'+table(['模态','来源','格式 / 尺寸','坐标 / 时间质量','原始时间 / s'],rows)+'<p class="note">仅展示本次选中输入的归档字段；完整批次与缺失项见“时间与接口”。</p></details>';
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
    const evidenceRoot=$('evidence');
    captureDisclosures(evidenceRoot,evidenceDisclosureScope);
    evidenceDisclosureScope=branch+':evidence';
    if(branch==='coverage'){evidenceRoot.innerHTML='<p>每项要求可以在不同分支查看。已保存的处理证据、现场能力验证和精度评测分别确认。</p>'+note('没有新输入的分支使用已保存样本，不自动宣称四类传感器同时在线。');return;}
    if(!s){evidenceRoot.innerHTML='<p>等待该分支生成可对应的输入与特征归档。</p>';return;}
    const context=s.context||{},b=status?.branches?.[branch];
    const values=[['分支',names[branch]],['样本序号',s.sequence],['接口核查',checkLabel(s.interface_validation?.state)],['标定记录',context.calibration||'本样本未归档空间标定'],['输出形状',(s.output_shape||[]).join(' × ')]];
    const extra=[['输入时间',stamp(s.source_stamp)],['会话',s.session_id],['当前采集状态',b?.detail||'未读取'],['最大配对时间差',fmt(s.timing.maximum_gap_ms)+' ms'],['同步精度',checkLabel(s.sync_validation?.state)],['模型计算耗时',fmt(s.inference_ms,1)+' ms'],['采集时间质量',context.stamp_quality||'传感器消息时间']];
    $('evidence').innerHTML='<dl>'+values.map(([k,v])=>`<div><dt>${esc(k)}</dt><dd>${esc(v)}</dd></div>`).join('')+'</dl><details data-disclosure="sample-runtime"><summary>样本时间与运行详情</summary>'+table(['记录项','本次记录'],extra)+'</details><p class="note">'+esc(context.geometry_verified?'标定记录已测量，查看对应证据。':branch==='pmf'?'现有LiDAR外参为估计参数，尚未完成标定验收。':branch==='rgbnir'?'相机内参和外参来自工厂参数记录，现场配准效果需另行验证。':'声学输入来自本样本匹配的音频窗口。')+'</p><a href="/research/'+branch+'_features.npz">下载原始特征归档 ↗</a><details data-disclosure="model-source"><summary>本样本模型与处理来源</summary><pre>'+esc(JSON.stringify(s.model,null,2))+'</pre></details>';
    restoreDisclosures(evidenceRoot,evidenceDisclosureScope);
    snapshotNotice();
  }
  function coverage() {
    const items=[['四类模态输入','RGB、近红外、真实LiDAR与声学分别在三条分支输入端展示。',[['pmf',0],['rgbnir',0],['avs',0]]],['特征提取、拼接与融合','提取：三条分支；拼接：颜色＋法向量、视觉token；融合：几何交互、加权、声学条件交互。',[['pmf',4],['rgbnir',3],['avs',4]]],['接口与数据同步','接口字段规范、实际选中输入、特征数组及同批次输出核查。',[['pmf',2],['rgbnir',2],['avs',2]]],['时间同步与融合精度','同步限值与当前配对核查集中展示；融合结果通过同样本图片进行定性对照。',[['pmf',2],['rgbnir',5],['avs',5]]],['标定、时间对齐与预处理','相机参数、外参、点—像素映射、配准有效区、点云筛选及声学预处理。',[['pmf',1],['rgbnir',1],['avs',1]]],['特征尺度匹配与跨模态语义对齐','近红外展示尺寸与RMS幅值匹配；声学展示128→256及声学条件查询对视觉token的注意力。',[['rgbnir',4],['avs',4]]]];
    const verificationSummary='<h3 style="margin-top:24px">接口与时间同步记录</h3>'+table(['分支','接口核查','配对门限','同步精度'],Object.entries(names).map(([key,name])=>[name,checkLabel(samples[key]?.interface_validation?.state),checkLabel(samples[key]?.sync_validation?.pair_state),checkLabel(samples[key]?.sync_validation?.state)]));
    return '<div class="coverage-list">'+items.map((r,i)=>`<div class="coverage-item"><span class="index">${i+1}</span><div><strong>${esc(r[0])}</strong><p>${esc(r[1])}</p>${r[2].map(([b,st])=>`<button data-jump="${b}:${st}">${esc(names[b])} ↗</button>`).join('')}</div></div>`).join('')+'</div>'+verificationSummary+note('共享骨干和维度统一只能说明结构兼容；声学条件注意力可展示跨模态语义交互机制，语义对应应结合任务输出及验证记录核对。');
  }
  function stageMarkup(s) {
    const v=s.visuals,t=s.tables,img=s.images,c=s.context;
    if(step===0){
      if(branch==='pmf')return '<div class="gallery">'+figure(img.raw,'RGB图像')+(v.filter?plot('raw-cloud','真实LiDAR原始扫描 · '+v.filter.frame):v.cloud?plot('scene-cloud','已保留的真实LiDAR点 · icp_odom'):empty('该样本无LiDAR归档'))+'</div>'+inputTable(s)+note('深度参与几何处理；点云来自LiDAR采集归档。');
      if(branch==='rgbnir')return '<div class="gallery">'+figure(img.raw,'RGB图像')+figure(v.nir_raw,'原始近红外 · mono8')+'</div>'+inputTable(s)+note('数据来源与时间来自本分支保存记录，当前在线状态见上方。');
      return '<div class="gallery">'+figure(img.raw,'匹配窗口内的RGB帧')+(v.waveform?plot('waveform','WheelTec原始PCM波形',true):figure(null,'原始PCM波形'))+'</div><div style="margin-top:16px">'+figure(v.log_mel||img.spectrum,'实际log-Mel频谱')+'</div>'+inputTable(s)+(!v.waveform?note('旧样本仅保存log-Mel；新增采集会保留原始PCM，波形与功率频谱随后显示。'):'');
    }
    if(step===1){
      if(branch==='pmf')return '<div class="gallery">'+dots(img.raw,v.correspondence?.uv,'LiDAR投影到RGB')+plot('filter-cloud','同一扫描筛选前后 · '+(v.filter?.frame||''))+'</div>'+table(['对应点','X / m','Y / m','Z / m','u / px','v / px'],(v.correspondence?.xyz||[]).slice(0,8).map((p,i)=>[i+1,...p.map(x=>fmt(x)),...v.correspondence.uv[i].map(x=>fmt(x,1))]))+note(v.filter?.note||'此样本没有同扫描筛选记录')+'<details data-disclosure="calibration-parameters"><summary>相机参数与坐标变换</summary>'+matrix('RGB内参 K',t.intrinsics)+matrix('camera_from_lidar · 平移单位 m',t.extrinsics)+note(c.calibration_evidence||'无独立标定证据')+'</details>';
      if(branch==='rgbnir')return '<div class="gallery three">'+figure(v.nir_raw,'配准前近红外')+figure(v.nir_registered,'配准到RGB网格')+figure(v.valid_mask,'有效区域','白色有效；黑色不使用近红外')+'</div>'+table(['对应点','RGB u','RGB v','NIR u','NIR v'],(v.correspondence?.rgb_uv||[]).slice(0,8).map((p,i)=>[i+1,...p.map(x=>fmt(x,1)),...v.correspondence.nir_uv[i].map(x=>fmt(x,1))]))+'<details data-disclosure="calibration-parameters"><summary>相机参数与坐标变换</summary>'+matrix('RGB内参 K',t.intrinsics)+matrix('nir_from_rgb · 平移单位 m',t.extrinsics)+note('标定版本：'+c.calibration)+'</details>';
      return flow([['PCM','采样率 '+(v.waveform?.rate||c.sample_rate||'未归档')+' Hz'],['STFT','时频变换'],['log-Mel','96 × 64 / 片段']])+'<div class="gallery">'+(v.waveform?plot('waveform','原始波形 · 幅值',true):empty('该样本未保存原始PCM'))+figure(v.log_mel,'log-Mel · 时间×频率响应')+'</div>'+(v.spectrum?plot('spectrum','实际PCM Welch功率谱密度',true):note('功率频谱将在新的PCM归档产生后显示。'))+note('声学原始时间、应用偏移及时间质量在“时间与接口”中按本批次记录。');
    }
    if(step===2){
      const rows=s.timing.rows||[];
      return syncMarkup(s)+'<details class="record-details" data-disclosure="source-timestamps"><summary>查看逐组原始与校正时间</summary>'+timeline(s.timing)+table(['输入组','RGB对齐时间 / s','另一模态对齐时间 / s','配对差 / ms'],rows.map((r,i)=>[i+1,fmt(r.a,6),fmt(r.b,6),fmt(r.gap_ms,3)]))+'<h3>原始时间轴</h3>'+timeline(s.timing,true)+'</details>'+interfaceMarkup(s);
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
    return gallery+note(branch==='pmf'?'当前图片分别展示两个多源模型的结果，按实际模型标识进行定性对照。':branch==='rgbnir'?'对比同一RGB样本的融合结果与原RGB结果，观察类别、目标区域及漏检或误检情况。':'查看匹配RGB帧与音视频融合结果，观察发声区域及类别对应情况。')+'<details data-disclosure="output-records"><summary>输出类别统计与模型记录</summary>'+table(['输出类别','预测点/像素数','预测占比'],s.classes.map(r=>[r.display_label||r.label,r.count,fmt(r.fraction*100,1)+'%']))+note('图片对照用于定性观察；此处不自动生成定量精度或合格结论。')+'</details>';
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
    captureDisclosures($('visual'),visualDisclosureScope);
    visualDisclosureScope=branch+':'+step+':visual';
    plotObserver?.disconnect();
    document.querySelectorAll('[data-branch]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.branch===branch)));
    $('stages').innerHTML=branch==='coverage'?'':steps.map((name,i)=>`<button data-step="${i}" aria-pressed="${i===step}"><span class="step-number">0${i+1}</span>${name}</button>`).join('');
    $('stage-kicker').textContent=branch==='coverage'?'TASK BOOK COVERAGE':names[branch]+' · STEP 0'+(step+1);
    $('stage-title').textContent=branch==='coverage'?'三条分支共同覆盖任务书':titles[branch][step];
    $('stage-description').textContent=branch==='coverage'?'点击条目跳转到对应分支证据。技术性能仍以实测记录判定。':descriptions[branch][step];
    $('stage-tag').textContent=branch==='coverage'?'六项要求':step===5?'图片定性对照':'实际处理记录';
    const s=samples[branch];$('visual').innerHTML=branch==='coverage'?coverage():s?stageMarkup(s):empty('等待该分支的同会话输入与特征归档；其他分支可独立查看。');
    if(s&&step===1&&branch!=='avs'&&s.tables.extrinsics){
      const summary=[...$('visual').querySelectorAll('summary')].find(e=>e.textContent.includes('相机参数'));
      if(summary){summary.insertAdjacentHTML('afterend',plot('calibration-axes','内外参记录对应的传感器坐标轴'));summary.nextElementSibling.classList.add('calibration-plot');}
    }
    restoreDisclosures($('visual'),visualDisclosureScope);
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
    $('sample-status').innerHTML=`<span class="tag ${saved?'saved':'live'}">${frozen?'已冻结样本':s?saved?'已保存样本 · 当前等待输入':'正在接收记录':'等待样本'}</span>`+(s?`<span>序号 ${esc(s.sequence)} · ${esc(stamp(s.source_stamp))} · ${s.archive_level==='result_snapshot_only'?'同一结果快照':'同会话归档'}</span>`:'')+`<span>${esc(!recent?'采集/导出服务当前未更新':b?.detail||'等待3001服务')}</span>`+(b?.exporter_error?`<span class="error">归档检查：${esc(b.exporter_error)}</span>`:'');
    snapshotNotice();
  }
  function snapshotNotice(){
    const s=samples[branch];if(!s||s.archive_level!=='result_snapshot_only')return;
    if(!document.getElementById('snapshot-archive-notice'))$('sample-status').insertAdjacentHTML('beforeend','<span id="snapshot-archive-notice">完整输入与张量未归档</span>');
    const link=$('evidence').querySelector('a[href$="_features.npz"]');
    if(link){link.removeAttribute('href');link.textContent='原运行未保存完整特征文件';}
  }
  async function fetchJson(url){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),5000);try{const r=await fetch(url+'?t='+Date.now(),{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error('HTTP '+r.status);return await r.json();}finally{clearTimeout(timer);}}
  async function poll(){if(mapping.isActive()||polling||frozen||document.hidden)return;polling=true;try{const nextStatus=await fetchJson('/research/process/status.json');if(frozen||mapping.isActive())return;status=nextStatus;for(const task of Object.keys(names)){
    const identity=status.branches?.[task]?.sample_identity;
    if(identity&&identity.join(':')!==sampleId(samples[task])&&!pending[task]){pending[task]=true;try{const s=await fetchJson('/research/process/'+task+'.json');if(s.schema!=='research_process/1'||s.task!==task)throw Error('Invalid process snapshot');if(frozen||mapping.isActive())return;samples[task]=s;if(task===branch)render();}finally{pending[task]=false;}}
    }updateStatus();setEvidence(samples[branch]);}catch(error){if(!samples[branch])$('sample-status').textContent='尚未取得过程导出记录，可继续查看已有分支；启动3001后会自动读取。';else updateStatus();}finally{polling=false;}}
  document.addEventListener('click',e=>{const button=e.target.closest('button');if(!button)return;if(button.dataset.standard){standard(button.dataset.standard);return;}if(mapping.isActive())return;if(button.dataset.branch){branch=button.dataset.branch;step=0;render();}if(button.dataset.step){step=Number(button.dataset.step);render();}if(button.dataset.jump){const parts=button.dataset.jump.split(':');branch=parts[0];step=Number(parts[1]);render();}});
  $('freeze').onclick=()=>{if(mapping.isActive()){mapping.freeze();return;}frozen=!frozen;$('freeze').setAttribute('aria-pressed',String(frozen));$('freeze').textContent=frozen?'恢复自动更新':'冻结当前样本';updateStatus();if(!frozen)poll();};
  $('download').onclick=()=>{if(mapping.isActive()){mapping.download();return;}const s=samples[branch];if(!s)return;const blob=new Blob([JSON.stringify({...s,display_capture:{frozen,stage:steps[step],captured_at:new Date().toISOString()}},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=`${branch}-process-${s.sequence}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});render();new URL(location.href).searchParams.get('standard')==='mapping'?standard('mapping'):poll();setInterval(poll,4000);
})();
