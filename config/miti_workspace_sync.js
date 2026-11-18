/* Apply a requested workspace update once in each browser's own storage. */
function rtabmapSyncWorkspaceStorage(storage, location, settings) {
  if(!storage||!location||String(location.port)!==String(settings.port))return {applied:false};
  const marker='rtabmap_workspace_sync_'+settings.port;
  const keys=['miti_layout_config','miti_bridge_url','miti_stream_mode',marker];
  const previous=Object.fromEntries(keys.map(key=>[key,storage.getItem(key)]));
  const copy=value=>JSON.parse(JSON.stringify(value));
  let layout;
  try{layout=JSON.parse(previous.miti_layout_config);}catch(_){}
  const valid=layout&&Array.isArray(layout.widgets)&&layout.widgets.length<=256&&layout.mosaic;
  const vendorIds=new Set(['urdf-viewer-1','camera-viewer-1','camera-viewer-2','log-1']);
  const vendorDefault=valid&&layout.widgets.length>0&&layout.widgets.every(w=>vendorIds.has(w.i));
  if(!valid||vendorDefault)layout=copy(settings.layout);
  else layout=copy(layout);
  function leaves(tree) {
    const result=[],pending=[tree];let count=0;
    while(pending.length&&count++<2048){
      const node=pending.pop();
      if(typeof node==='string')result.push(node);
      else if(node&&typeof node==='object')pending.push(node.first,node.second);
    }
    return new Set(result);
  }
  let visible=leaves(layout.mosaic);
  const existingMap=layout.widgets.find(w=>(w.i===settings.mapWidget.i||w.rtabmapWorkspaceMap===true)&&w.type==='urdf-viewer');
  const existingAudio=layout.widgets.find(w=>w.type==='audio-events');
  if(previous[marker]===settings.revision&&existingMap&&visible.has(existingMap.i)&&
    existingAudio&&visible.has(existingAudio.i))return {applied:false};
  function uniqueId(base){let id=base,suffix=2;while(layout.widgets.some(w=>w.i===id))id=base+'-'+suffix++;return id;}
  let map=existingMap;
  if(map){
    map.props={...map.props,frame:{...map.props?.frame,...settings.mapWidget.props.frame},
      topics:{...settings.mapWidget.props.topics,...map.props?.topics,
        '/cloud_map':{...settings.mapWidget.props.topics['/cloud_map'],...map.props?.topics?.['/cloud_map'],visible:true}}};
  }else{
    map=copy(settings.mapWidget);map.i=uniqueId(map.i);layout.widgets.push(map);
  }
  map.rtabmapWorkspaceMap=true;
  let audio=existingAudio;
  if(!audio){audio={i:uniqueId('rtabmap-audio-events'),type:'audio-events',title:'哨兵巡逻 · 声音事件',
    x:12,y:0,w:8,h:12,props:{topic:'/perception/audio/events'}};layout.widgets.push(audio);}
  else audio.props={...audio.props,topic:'/perception/audio/events'};
  if(!visible.has(audio.i))layout.mosaic={direction:'column',first:layout.mosaic,second:audio.i,splitPercentage:68};
  if(!visible.has(map.i))layout.mosaic={direction:'row',first:map.i,second:layout.mosaic,splitPercentage:48};
  const hostname=location.hostname||'localhost';
  const host=hostname.includes(':')?'['+hostname+']':hostname;
  const bridge=(location.protocol==='https:'?'wss':'ws')+'://'+host+':'+settings.bridgePort;
  const backup='rtabmap_miti_backup_sync_'+settings.port+'_'+Date.now();
  try{
    storage.setItem(backup,JSON.stringify(previous));
    storage.setItem('miti_layout_config',JSON.stringify(layout));
    storage.setItem('miti_bridge_url',JSON.stringify(bridge));
    if(!previous.miti_stream_mode)storage.setItem('miti_stream_mode',JSON.stringify('synchronized'));
    storage.setItem(marker,settings.revision);
  }catch(error){
    for(const key of keys){try{previous[key]===null?storage.removeItem(key):storage.setItem(key,previous[key]);}catch(_){}}
    throw error;
  }
  return {applied:true,mapId:map.i,audioId:audio.i,bridge,backup};
}
