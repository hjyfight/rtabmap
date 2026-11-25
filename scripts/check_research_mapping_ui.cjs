// Isolated UI checks with an actual saved mapping capture; no ROS or robot control.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(__dirname,'..'),directory=path.join(root,'.tools/miti-research/root/usr/lib/miti/public/research/mapping');
const snapshot=JSON.parse(fs.readFileSync(path.join(directory,'sample.json'),'utf8'));
const resources={'/research-process.html':['research_process.html','text/html'],'/research-process.js':['research_process.js','text/javascript'],'/research-mapping.js':['research_mapping.js','text/javascript'],'/research-process.css':['research_process.css','text/css']};
const out=path.join(root,'logs/research/mapping-ui');fs.mkdirSync(out,{recursive:true});
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:path.join(root,'.tools/research-browsers/chrome-linux64/chrome'),args:['--no-sandbox','--disable-dev-shm-usage']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];let version=0,requests=0;
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('**/*',route=>{
   const u=new URL(route.request().url());if(resources[u.pathname]){const [file,type]=resources[u.pathname];return route.fulfill({body:fs.readFileSync(path.join(root,'config',file)),contentType:type});}
   if(u.pathname.startsWith('/research/mapping/')){requests++;const s=JSON.parse(JSON.stringify(snapshot));s.sequence+=version;
    const value=u.pathname.endsWith('status.json')?{schema:'research_mapping_status/1',published_at:Date.now()/1000,state:'saved',session_id:s.session_id,sequence:s.sequence}:s;
    return route.fulfill({contentType:'application/json',body:JSON.stringify(value)});
   }
   return route.fulfill({contentType:'application/json',body:JSON.stringify({schema:'research_process_status/1',published_at:Date.now()/1000,branches:{}})});
  });
  await page.goto('http://mapping.test:3001/research-process.html?standard=mapping');await page.waitForFunction(()=>!document.querySelector('#download').disabled);
  assert.equal(await page.locator('[data-standard="mapping"]').getAttribute('aria-pressed'),'true');
  assert.equal(await page.locator('#fusion-branches').isVisible(),false);
  const report={mode:'saved_mapping_ui',steps:[],errors};
  for(let step=0;step<6;step++){
   await page.locator(`[data-map-step="${step}"]`).click();await page.waitForTimeout(50);
   assert.ok((await page.locator('#visual').innerText()).length>40);
   report.steps.push(await page.locator('#stage-title').innerText());
   const views=await page.locator('[data-map-option]').evaluateAll(items=>items.map(e=>e.dataset.mapOption));
   for(const view of views){await page.locator(`[data-map-option="${view}"]`).click();await page.waitForTimeout(40);}
   if(step===3||step===5)await page.screenshot({path:path.join(out,`step-${step}.png`),fullPage:true});
  }
  await page.selectOption('#map-scene-type','structured');await page.fill('#map-scene-name','工位周边建图');await page.selectOption('#map-scene-terrain','平整地面');
  await page.locator('#map-scene-name').blur();
  await page.getByRole('button',{name:'冻结当前样本',exact:true}).click();const count=requests;version=1;
  await page.waitForTimeout(4300);assert.equal(requests,count,'freeze must suspend mapping requests');
  await page.getByRole('button',{name:'恢复自动更新',exact:true}).click();await page.waitForFunction(seq=>document.querySelector('#sample-status').innerText.includes('记录 '+seq),snapshot.sequence+1);
  assert.equal(await page.inputValue('#map-scene-name'),'工位周边建图');
  const event=page.waitForEvent('download');await page.getByRole('button',{name:'导出展示记录',exact:true}).click();const downloaded=await event;
  const filename=path.join(out,'download-check.json');await downloaded.saveAs(filename);const record=JSON.parse(fs.readFileSync(filename,'utf8'));
  assert.equal(record.display_capture.scene.registration,'manual');assert.equal(record.display_capture.scene.type,'structured');
  assert.ok(record.performance.rows.every(r=>r.value==null&&r.state==='pending'),'unmeasured accuracy must stay pending');
  await page.locator('[data-map-view="coverage"]').click();assert.equal(await page.locator('.coverage-item').count(),8);
  await page.screenshot({path:path.join(out,'coverage.png'),fullPage:true});
  await page.locator('[data-map-jump="3"]').first().click();assert.equal(await page.locator('[data-map-step="3"]').getAttribute('aria-pressed'),'true');
  await page.locator('[data-map-step="5"]').click();await page.setViewportSize({width:390,height:844});await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),'mobile form must not overflow');
  await page.locator('[data-standard="fusion"]').click();assert.equal(await page.locator('#fusion-branches').isVisible(),true);
  assert.equal(await page.locator('#page-title').innerText(),'看见输入如何成为融合结果');
  if(process.argv.includes('--live')){
   const live=await browser.newPage({viewport:{width:1500,height:1000}});live.on('pageerror',e=>errors.push('live: '+e.message));
   await live.goto('http://127.0.0.1:3001/research-process.html?standard=mapping');await live.waitForFunction(()=>!document.querySelector('#download').disabled);
   await live.locator('[data-map-step="3"]').click();await live.locator('[data-map-option="cloud"]').click();await live.screenshot({path:path.join(out,'live-map.png'),fullPage:true});
   report.live={page:live.url(),status:await live.locator('#sample-status').innerText()};
  }
  assert.deepEqual(errors,[]);report.freeze=true;report.export=true;report.scene_preserved=true;report.mobile=true;
  fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
