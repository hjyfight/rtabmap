// Integration check of auto-rendered disclosure state using a small virtual DOM.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const root=path.resolve(__dirname,'..'),saved=new Map();
const sample=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
class Detail{
  constructor(key,open=false){this.dataset={disclosure:key};this.open=open;this.isConnected=true;this.listeners={};}
  addEventListener(type,fn){(this.listeners[type]??=[]).push(fn);}
  toggle(){for(const fn of this.listeners.toggle||[])fn();}
}
class Element{
  constructor(){this.details=[];this._html='';this.dataset={};this.clientWidth=0;this.clientHeight=0;this.classList={add(){}};}
  set innerHTML(value){
    this.details.forEach(d=>d.isConnected=false);this._html=value;this.details=[];
    for(const m of value.matchAll(/<details\b([^>]*)>/g)){
      const key=m[1].match(/data-disclosure="([^"]+)"/);
      if(key)this.details.push(new Detail(key[1],/\bopen(?:\s|=|$)/.test(m[1])));
    }
  }
  get innerHTML(){return this._html;}
  querySelectorAll(selector){return selector==='details[data-disclosure]'?this.details:[];}
  addEventListener(){}setAttribute(){}
}
function page(blockStorage=false){
  const nodes=new Map(),element=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  const context={document:{hidden:true,getElementById:element,querySelectorAll(){return []},addEventListener(){}},
    window:{devicePixelRatio:1},ResizeObserver:class{observe(){}disconnect(){}},setInterval(){},setTimeout(){},clearTimeout(){},
    sessionStorage:{getItem(key){if(blockStorage)throw Error('storage disabled');return saved.get(key)||null;},
      setItem(key,value){if(blockStorage)throw Error('storage disabled');saved.set(key,value);}}};
  vm.createContext(context);
  let source=fs.readFileSync(path.join(root,'config/research_process.js'),'utf8');
  source=source.replace(/\}\)\(\);\s*$/,'globalThis.testView={render,setEvidence,setView:(next,index,value)=>{branch=next;step=index;samples[next]=value;render();}};})();');
  vm.runInContext(source,context);
  const find=(key,area='visual')=>{const d=element(area).details.find(d=>d.dataset.disclosure===key);assert.ok(d,'Missing disclosure '+key);return d;};
  return {api:context.testView,find,element};
}
const p=page();p.api.setView('pmf',2,sample);
const old=p.find('inputs-used');old.open=true;old.toggle();
for(const key of ['feature-arrays','sync-statistics','clock-quality']){const d=p.find(key);d.open=true;d.toggle();}
const side=p.find('sample-runtime','evidence');side.open=true;side.toggle();
const update=JSON.parse(JSON.stringify(sample));update.sequence++;
update.interface_validation.inputs=update.interface_validation.inputs.slice(0,-1);
p.api.setView('pmf',2,update);
assert.notEqual(p.element('visual').innerHTML.includes('（'+sample.interface_validation.inputs.length+'条）'),true);
for(const key of ['inputs-used','feature-arrays','sync-statistics','clock-quality'])assert.equal(p.find(key).open,true,key+' must survive sample refresh');
assert.equal(p.find('sample-runtime','evidence').open,true);
p.api.setEvidence(update);assert.equal(p.find('sample-runtime','evidence').open,true,'independent evidence refresh');
old.open=false;old.toggle();
assert.equal(JSON.parse(saved.get('research-process-disclosures-v1'))['pmf:2:visual']['inputs-used'],true,'detached toggle cannot overwrite saved state');
p.find('inputs-used').open=false; // Capture a close even before its queued toggle runs.
p.api.setView('pmf',2,update);assert.equal(p.find('inputs-used').open,false);
p.api.setView('rgbnir',2,update);assert.equal(p.find('feature-arrays').open,false,'branches have separate state');
p.api.setView('pmf',2,update);assert.equal(p.find('feature-arrays').open,true);assert.equal(p.find('inputs-used').open,false);
p.api.setView('pmf',0,update);p.find('sample-inputs').open=true;p.find('sample-inputs').toggle();
p.api.setView('pmf',2,update);assert.equal(p.find('feature-arrays').open,true);
p.api.setView('pmf',0,update);assert.equal(p.find('sample-inputs').open,true,'steps keep independent state');
const reload=page();reload.api.setView('pmf',2,update);assert.equal(reload.find('feature-arrays').open,true);assert.equal(reload.find('inputs-used').open,false);
const privatePage=page(true);privatePage.api.setView('pmf',2,sample);privatePage.find('inputs-used').open=true;
privatePage.api.setView('pmf',2,update);assert.equal(privatePage.find('inputs-used').open,true,'in-memory fallback');
console.log('Disclosure refresh, nested sections, branch/step changes and reload state passed');
