// Pure rendering checks in a VM; this does not operate a browser or the user's UI.
const fs=require('fs'), vm=require('vm'), assert=require('assert'), path=require('path');
const root=path.resolve(__dirname,'..');
const nodes=new Map();
const element=id=>{if(!nodes.has(id))nodes.set(id,{innerHTML:'',textContent:'',dataset:{},
  clientWidth:0,clientHeight:0,classList:{add(){}},setAttribute(){},querySelectorAll(){return []},addEventListener(){}});return nodes.get(id);};
const context={document:{hidden:true,getElementById:element,querySelectorAll(){return []},addEventListener(){}},
  window:{devicePixelRatio:1},ResizeObserver:class {observe(){}disconnect(){}},setInterval(){},setTimeout(){},clearTimeout(){},console};
vm.createContext(context);
let code=fs.readFileSync(path.join(root,'config/research_process.js'),'utf8');
code=code.replace(/\}\)\(\);\s*$/, 'globalThis.testRender={interfaceMarkup,syncMarkup,stageMarkup,setEvidence,setBranch:(value,index=2)=>{branch=value;step=index;}};})();');
vm.runInContext(code,context);
const sample=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
function visibleMarkup(html){
  let depth=0,last=0,out='';
  for(const m of html.matchAll(/<details\b[^>]*>|<\/details>/g)){
    if(depth===0)out+=html.slice(last,m.index);
    if(m[0].startsWith('</'))depth--;else depth++;
    last=m.index+m[0].length;
  }
  if(depth===0)out+=html.slice(last);
  return out;
}
for(const branch of ['rgbnir','pmf','avs']){
  context.testRender.setBranch(branch);
  const markup=context.testRender.stageMarkup(sample);
  assert.ok(markup.includes('接口规范与当前批次核查'));
  assert.ok(markup.includes('时间同步要求与测量'));
  assert.ok(markup.includes('当前配对核查门限'));
  assert.ok(markup.includes('同步精度测量'));
  assert.ok(markup.includes('本次实际参与处理的输入字段'));
  assert.ok(!markup.includes('采集时间可能晚于当前归档样本'));
  assert.ok(!markup.includes('/newer'));
  assert.ok(!markup.includes('999999.000000'));
  assert.ok(markup.includes('尚无覆盖采集时间的测量记录'));
  const visible=visibleMarkup(markup);
  assert.ok(visible.includes('同步限值'));
  assert.ok(visible.includes('最大配对时间差'));
  assert.ok(visible.includes('配对核查结果'));
  assert.ok(visible.includes('同步精度：待验证'));
  assert.ok(visible.includes('各组配对时间差'));
  for(const unused of ['P95','平均时间差','运行容差','往返估计不确定度','测量不确定度','原始时间轴','RGB对齐时间 / s','公式'])assert.ok(!visible.includes(unused),unused+' should be folded');
  context.testRender.setEvidence(sample);
  const evidence=visibleMarkup(nodes.get('evidence').innerHTML);
  assert.ok(!evidence.includes('耗时'));
  assert.ok(!evidence.includes('输入时间'));
  context.testRender.setBranch(branch,5);
  const output=context.testRender.stageMarkup(sample);
  assert.ok(output.includes('定性'));
  assert.ok(!output.includes('同步测量状态'));
  assert.ok(!output.includes('本次模型计算耗时'));
}
sample.interface_validation.state='missing';
assert.ok(context.testRender.interfaceMarkup(sample).includes('归档缺失'));
sample.sync_validation.pair_state='fail';
assert.ok(context.testRender.syncMarkup(sample).includes('存在超门限配对'));
assert.ok(context.testRender.interfaceMarkup({}).includes('旧样本'));
console.log('Interface and synchronization rendering checks passed');
