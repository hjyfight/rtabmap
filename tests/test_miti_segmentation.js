const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.resolve(__dirname,'..');
const vendor=require('node:module').createRequire(path.join(root,'.tools/miti/root/usr/lib/miti/package.json'));
const React=vendor('react'),{renderToStaticMarkup}=vendor('react-dom/server.browser');
const context=vm.createContext({});vm.runInContext(fs.readFileSync(path.join(root,'config/miti_segmentation.js'),'utf8'),context);
const panel=context.rtabmapSegmentationFactory(React);
function snapshot(){return {schema:1,session_id:'test',sequence:1,revision:0,width:640,height:480,
  stamp:{sec:1,nanosec:0},result_source:'rgb_yolo',inference_s:.4,images:{raw:'YWJj',overlay:'ZGVm',classes:'aGlq'},
  instances:[{instance_id:1,class_id:0,label:'人',confidence:.9,classification_score:-1,bbox_xyxy:[10,20,100,200],color:[20,80,255]}]};}
test('atomic image snapshot and controls use the same frame and view',()=>{
  const v=snapshot(), parsed=panel.parseSnapshot({data:JSON.stringify(v)});assert.ok(parsed);
  for(const mode of ['raw','overlay','classes']){
    const html=renderToStaticMarkup(React.createElement(panel.View,{sample:{snapshot:parsed},mode}));
    assert.ok(html.includes('data:image/jpeg;base64,'+v.images[mode]));
    assert.match(html,/采集帧 #1/);assert.match(html,/1 个目标/);assert.match(html,/400 ms/);assert.match(html,/<details>/);
  }
  delete v.images.raw;assert.equal(panel.parseSnapshot({data:JSON.stringify(v)}),null);
  assert.equal(panel.parseSnapshot({data:'{}'}),null);
});
test('no fabricated image; heartbeat loss, input loss and session restart mark retained image stale',()=>{
  assert.doesNotMatch(renderToStaticMarkup(React.createElement(panel.View)),/<image/);
  const sample={snapshot:snapshot(),receivedAt:0},status={snapshot:{schema:1,session_id:'test',state:'ready'},receivedAt:0};
  assert.equal(panel.deriveState(sample,status,1000,true).stale,false);
  assert.equal(panel.deriveState(sample,status,5000,true).stale,true);
  for(const state of ['input_stale','inference_error','waiting_image']){
    assert.equal(panel.deriveState(sample,{snapshot:{...status.snapshot,state},receivedAt:0},1000,true).stale,true);
  }
  assert.equal(panel.deriveState(sample,{snapshot:{...status.snapshot,session_id:'new'},receivedAt:0},1000,true).stale,true);
  assert.equal(panel.deriveState(sample,{snapshot:{...status.snapshot,frame_age_s:16},receivedAt:0},1000,true).stale,true);
});
test('subscription cleanup and one-time migration preserve user state',()=>{
  let callback,removed,count=0;
  const client={subscribe(t,f){callback=f;},unsubscribe(t,f){removed=f;}};
  const stop=panel.subscribe(client,'/test',panel.parseSnapshot,()=>count++,()=>{},()=>0);
  callback({data:JSON.stringify(snapshot())});stop();callback({data:JSON.stringify(snapshot())});
  assert.equal(count,1);assert.equal(removed,callback);
  const original={widgets:[{i:'features',type:'fourmodal-features',props:{test:42}}],mosaic:'features'};
  const migrated=context.rtabmapMigrateSegmentationLayout(original);
  assert.equal(migrated.widgets.length,2);assert.equal(migrated.widgets[0].props,original.widgets[0].props);
  assert.equal(context.rtabmapMigrateSegmentationLayout(migrated),migrated);
  const removedLayout={widgets:migrated.widgets.slice(0,1),mosaic:'features'};
  assert.equal(context.rtabmapMigrateSegmentationLayout(removedLayout),removedLayout);
});
test('fusion comparison shows independent inputs, evaluation state and retained-result warning',()=>{
  const fusion={schema:1,experimental:true,accuracy_status:'unlabeled',session_id:'test',sequence:1,
    width:640,height:480,inference_s:16,coverage:.93,objects:[],class_names:['人'],
    images:{rgb:'YWJj',nir:'YWJj',fused:'YWJj'}};
  assert.ok(panel.parseFusion({data:JSON.stringify(fusion)}));
  const sample={snapshot:fusion,receivedAt:0};
  let html=renderToStaticMarkup(React.createElement(panel.FusionComparison,{sample,now:1000}));
  assert.match(html,/精度未验证/);assert.match(html,/近红外单模态/);assert.match(html,/节点未更新/);
  fusion.accuracy_status='validated_baseline';
  html=renderToStaticMarkup(React.createElement(panel.FusionComparison,{sample,now:1000,
    status:{snapshot:{state:'input_stale'},receivedAt:0}}));
  assert.match(html,/评估选择 RGB 基线/);assert.match(html,/未更新/);
  fusion.width=99999;assert.equal(panel.parseFusion({data:JSON.stringify(fusion)}),null);
});
test('each fusion picture labels its own visible classes inside image bounds',()=>{
  const data={schema:1,experimental:true,accuracy_status:'unlabeled',session_id:'test',sequence:1,
    width:424,height:240,inference_s:16,coverage:.93,objects:[],class_names:['地面','机器人','瓶子'],
    images:{rgb:'YWJj',nir:'YWJj',fused:'YWJj'},image_labels:{
      rgb:[{class_id:0,x:0,y:239}],nir:[{class_id:1,x:423,y:0}],fused:[{class_id:2,x:210,y:120}]}};
  assert.ok(panel.parseFusion({data:JSON.stringify(data)}));
  for(const [kind,name] of [['rgb','地面'],['nir','机器人'],['fused','瓶子']]){
    const html=renderToStaticMarkup(React.createElement(panel.FusionPicture,{data,kind,title:kind}));
    assert.match(html,new RegExp('<text[^>]*>'+name+'</text>'));
    assert.match(html,/<rect[^>]*fill-opacity="0.85"/);
    for(const other of data.class_names.filter(n=>n!==name))assert.doesNotMatch(html,new RegExp('>'+other+'</text>'));
    const rect=html.match(/<rect[^>]+>/)[0],attribute=n=>Number(rect.match(new RegExp(' '+n+'="([^"]+)"'))[1]);
    assert.ok(attribute('x')>=0 && attribute('y')>=0);
    assert.ok(attribute('x')+attribute('width')<=data.width && attribute('y')+attribute('height')<=data.height);
  }
  data.image_labels.nir[0].x=999;assert.equal(panel.parseFusion({data:JSON.stringify(data)}),null);
});
test('native instance labels outside the task vocabulary stay visible and are counted separately',()=>{
  const data={schema:1,experimental:true,accuracy_status:'unlabeled',session_id:'test',sequence:42,
    width:424,height:240,inference_s:16,coverage:.93,objects:[],class_names:['人'],
    images:{rgb:'YWJj',nir:'YWJj',fused:'YWJj'},image_label_vocabulary:'native_yolo',
    image_instance_counts:{rgb:2,nir:0,fused:2},image_labels:{
      rgb:[{instance_id:1,class_id:4097,label:'技术员',x:100,y:100,pixels:1},
        {instance_id:2,class_id:4097,label:'技术员',x:180,y:100,pixels:1}],nir:[],
      fused:[{instance_id:1,class_id:4097,label:'技术员',x:100,y:100},
        {instance_id:2,class_id:4097,label:'技术员',x:180,y:100}]}};
  const parsed=panel.parseFusion({data:JSON.stringify(data)});assert.ok(parsed);
  const html=renderToStaticMarkup(React.createElement(panel.FusionComparison,{sample:{snapshot:parsed},now:0}));
  assert.match(html,/同一采集帧 #42/);assert.match(html,/RGB 单模态 · 2 个目标/);
  assert.equal((html.match(/>技术员<\/text>/g)||[]).length,4);
  delete data.image_labels.rgb[0].label;
  assert.equal(panel.parseFusion({data:JSON.stringify(data)}),null);
});
test('displayed frame age advances while retaining the same images and computation time',()=>{
  const rgb={...snapshot(),source_age_s:.2};
  const fusion={schema:1,experimental:true,accuracy_status:'unlabeled',session_id:'test',sequence:1,
    width:640,height:480,inference_s:.35,source_age_s:.8,coverage:.93,objects:[],class_names:['人'],
    images:{rgb:'YWJj',nir:'YWJj',fused:'YWJj'}};
  assert.ok(panel.parseSnapshot({data:JSON.stringify(rgb)}));
  assert.ok(panel.parseFusion({data:JSON.stringify(fusion)}));
  const html=renderToStaticMarkup(React.createElement(panel.View,{
    sample:{snapshot:rgb,receivedAt:1000},now:2000}));
  assert.match(html,/画面帧龄 1.2 s/);assert.match(html,/400 ms/);
  const compare=renderToStaticMarkup(React.createElement(panel.FusionComparison,{
    sample:{snapshot:fusion,receivedAt:1000},now:2000}));
  assert.match(compare,/画面帧龄 1.8 s/);assert.match(compare,/融合计算 0.35 s/);
  rgb.source_age_s=-1;fusion.source_age_s='fast';
  assert.equal(panel.parseSnapshot({data:JSON.stringify(rgb)}),null);
  assert.equal(panel.parseFusion({data:JSON.stringify(fusion)}),null);
});
