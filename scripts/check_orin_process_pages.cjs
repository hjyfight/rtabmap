// Isolated browser verification against the actual Orin-hosted 3001 service.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(__dirname,'..'),base='http://192.168.1.103:3001';
const out=path.join(root,'logs/orin_visualization_deploy/20261125/browser');fs.mkdirSync(out,{recursive:true});
(async()=>{
 const browser=await chromium.launch({headless:true,executablePath:path.join(root,'.tools/research-browsers/chrome-linux64/chrome'),args:['--no-sandbox','--disable-dev-shm-usage']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.goto(base+'/');await page.locator('#research-process-entry').waitFor({timeout:20000});
  await page.waitForFunction(()=>document.querySelectorAll('.panel-shell').length===9,null,{timeout:20000});
  assert.equal(await page.evaluate(()=>JSON.parse(localStorage.getItem('miti_layout_config')).widgets.length),9);
  await page.locator('#research-process-entry').click();await page.locator('#download').waitFor();
  await page.waitForFunction(()=>{const button=document.querySelector('#download');return button&&!button.disabled;});
  const report={root_panels:9,fusion:{},mapping:[],errors};
  for(const task of ['pmf','rgbnir','avs']){
   await page.locator(`[data-branch="${task}"]`).click();await page.waitForFunction(()=>!document.querySelector('#download').disabled);
   for(let step=0;step<6;step++){await page.locator(`[data-step="${step}"]`).click();assert.ok((await page.locator('#visual').innerText()).length>30);}
   report.fusion[task]=await page.locator('#sample-status').innerText();
   assert.equal(await page.locator('#evidence a[href$="_features.npz"]').count(),0,'missing NPZ must not have an active download link');
  }
  await page.waitForTimeout(4200);
  assert.equal(await page.locator('#snapshot-archive-notice').count(),1);
  assert.equal(await page.locator('#evidence a[href$="_features.npz"]').count(),0);
  await page.screenshot({path:path.join(out,'fusion.png'),fullPage:true});
  await page.locator('[data-standard="mapping"]').click();await page.waitForFunction(()=>!document.querySelector('#download').disabled);
  for(let step=0;step<6;step++){await page.locator(`[data-map-step="${step}"]`).click();assert.ok((await page.locator('#visual').innerText()).length>40);report.mapping.push(await page.locator('#stage-title').innerText());}
  await page.locator('[data-map-step="3"]').click();await page.locator('[data-map-option="cloud"]').click();await page.screenshot({path:path.join(out,'mapping.png'),fullPage:true});
  await page.getByRole('button',{name:'冻结当前样本',exact:true}).click();assert.ok((await page.locator('#sample-status').innerText()).includes('已冻结'));
  const downloadEvent=page.waitForEvent('download');await page.getByRole('button',{name:'导出展示记录',exact:true}).click();const download=await downloadEvent;assert.match(download.suggestedFilename(),/^mapping-process-/);
  await page.getByRole('button',{name:'恢复自动更新',exact:true}).click();
  await page.locator('[data-map-view="coverage"]').click();assert.equal(await page.locator('.coverage-item').count(),8);
  await page.setViewportSize({width:390,height:844});await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
  assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
  assert.deepEqual(errors,[]);report.mapping_status=await page.locator('#sample-status').innerText();report.freeze=true;report.export=true;report.mobile=true;
  fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
 }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
