/* Storage is scoped to the independent 3001 browser origin. */
function rtabmapSyncWorkspaceStorage(storage,location,settings) {
  if(!storage||String(location?.port)!=='3001')return {applied:false};
  if(typeof document!=='undefined'){
    document.documentElement.classList.add('rtabmap-dashboard');document.title='融合研究 · 室内LiDAR / 音视频 / RGB-NIR';
    if(!document.getElementById('rtabmap-dashboard-style')){
      const style=document.createElement('link');style.id='rtabmap-dashboard-style';style.rel='stylesheet';
      style.href='/rtabmap-dashboard.css?v='+encodeURIComponent(settings.revision);document.head.appendChild(style);
    }
    rtabmapResearchProcessEntry();
  }
  const marker='rtabmap_workspace_sync_3001';
  if(storage.getItem(marker)===settings.revision)return {applied:false};
  const previous=Object.fromEntries(['miti_layout_config','miti_bridge_url','miti_stream_mode',marker].map(k=>[k,storage.getItem(k)]));
  storage.setItem('rtabmap_research_backup_'+Date.now(),JSON.stringify(previous));
  const hostname=location.hostname||'localhost',host=hostname.includes(':')?'['+hostname+']':hostname;
  const bridge=(location.protocol==='https:'?'wss':'ws')+'://'+host+':'+settings.bridgePort;
  try {
    storage.setItem('miti_layout_config',JSON.stringify(settings.layout));
    storage.setItem('miti_bridge_url',JSON.stringify(bridge));
    storage.setItem('miti_stream_mode',JSON.stringify('synchronized'));
    storage.setItem(marker,settings.revision);
  }catch(error){for(const [key,value] of Object.entries(previous)){value===null?storage.removeItem(key):storage.setItem(key,value);}throw error;}
  return {applied:true,bridge};
}

function rtabmapResearchProcessEntry(){
  if(window.__researchProcessEntry)return;
  window.__researchProcessEntry=true;
  function mount(){
    const bar=document.querySelector('.workspace-titlebar');
    if(!bar||document.getElementById('research-process-entry'))return;
    const entry=document.createElement('a');entry.id='research-process-entry';
    entry.href='/research-process.html';entry.textContent='流程与验证 ↗';
    entry.title='查看多源输入、标定、特征、拼接和融合记录';
    entry.style.cssText='display:inline-flex;align-items:center;flex:none;padding:6px 11px;border:1px solid #355c61;border-radius:6px;color:#9ee9d7;font-size:12px;text-decoration:none;white-space:nowrap;';
    const brand=bar.querySelector('.workspace-brand');
    if(brand)brand.insertAdjacentElement('afterend',entry);else bar.prepend(entry);
  }
  mount();
  // Workspace is mounted after storage initialization. Keep one lightweight observer.
  let scheduled=false;
  const observer=new MutationObserver(()=>{if(scheduled)return;scheduled=true;requestAnimationFrame(()=>{scheduled=false;mount();if(document.getElementById('research-process-entry'))observer.disconnect();});});
  observer.observe(document.documentElement,{childList:true,subtree:true});
}
