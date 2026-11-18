const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.resolve(__dirname,'..'),vendor=require('node:module').createRequire(path.join(root,'.tools/miti/root/usr/lib/miti/package.json'));
const React=vendor('react'),{renderToStaticMarkup}=vendor('react-dom/server.browser');
const context=vm.createContext({});vm.runInContext(fs.readFileSync(path.join(root,'config/miti_object_multimodal.js'),'utf8'),context);
const panel=context.rtabmapObjectMultimodalFactory(React);
function snapshot(){return JSON.parse(fs.readFileSync(path.join(root,'logs/object_multimodal_validation/frontend_fixture/result_1.json')));}
test('target panel displays actual participating modalities, source association and restrictions',()=>{
  const v=snapshot(),parsed=panel.parseSnapshot({data:JSON.stringify(v)});assert.ok(parsed);
  const html=renderToStaticMarkup(React.createElement(panel.View,{snapshot:parsed,offline:true}));
  assert.match(html,/单目标四模态融合/);assert.match(html,/四模态齐全/);assert.match(html,/离线模型结果/);
  assert.match(html,/待实测验证/);assert.match(html,/精度待验证/);assert.match(html,/目标 ID/);
});
test('malformed identities, weights and provenance cannot be displayed as valid',()=>{
  for(const mutate of [v=>v.targets.push(v.targets[0]),v=>v.targets[0].features.weights[0]=-1,
    v=>v.targets[0].features.valid_mask[0]=1,v=>v.targets[0].modalities.audio.associated=false,
    v=>v.safety.prohibit_automatic_execution=false,v=>v.targets[0].features.joint[0]+=1]){
    const v=snapshot();mutate(v);assert.equal(panel.parseSnapshot({data:JSON.stringify(v)}),null);
  }
  assert.equal(panel.parseSnapshot({data:'{}'}),null);
});
test('stale retained target results are labelled waiting and never reported live',()=>{
  const html=renderToStaticMarkup(React.createElement(panel.View,{snapshot:snapshot(),receivedAt:1,now:20000}));
  assert.match(html,/等待目标融合结果/);assert.doesNotMatch(html,/目标融合已更新/);
  assert.match(renderToStaticMarkup(React.createElement(panel.View,{snapshot:null})),/等待目标融合结果/);
});
