/* Storage is scoped to the independent 3001 browser origin. */
function rtabmapSyncWorkspaceStorage(storage,location,settings) {
  if(!storage||String(location?.port)!=='3001')return {applied:false};
  if(typeof document!=='undefined'){
    document.documentElement.classList.add('rtabmap-dashboard');document.title='融合研究 · 室内LiDAR / 音视频 / RGB-NIR';
    if(!document.getElementById('rtabmap-dashboard-style')){
      const style=document.createElement('link');style.id='rtabmap-dashboard-style';style.rel='stylesheet';
      style.href='/rtabmap-dashboard.css?v='+encodeURIComponent(settings.revision);document.head.appendChild(style);
    }
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
