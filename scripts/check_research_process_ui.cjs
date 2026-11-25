// Isolated browser test of the real static page and real saved-process exports.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(__dirname,'..'),exportsDir=path.join(root,'.tools/miti-research/root/usr/lib/miti/public/research/process');
const out=path.join(root,'logs/research/process-ui');fs.mkdirSync(out,{recursive:true});
const resources={'/research-process.html':['config/research_process.html','text/html'],'/research-process.js':['config/research_process.js','text/javascript'],'/research-mapping.js':['config/research_mapping.js','text/javascript'],'/research-process.css':['config/research_process.css','text/css']};
const fixtures=Object.fromEntries(['status','pmf','rgbnir','avs'].map(key=>[key,JSON.parse(fs.readFileSync(path.join(exportsDir,key+'.json'),'utf8'))]));
const baseSequence=fixtures.pmf.sequence;
for(const key of ['pmf','rgbnir','avs'])fixtures.status.branches[key].sample_identity=[fixtures[key].session_id,fixtures[key].sequence];
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:path.join(root,'.tools/research-browsers/chrome-linux64/chrome'),args:['--no-sandbox','--disable-dev-shm-usage']});
  try{
    const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[],report={mode:'saved_process_ui',stages:{}};
    page.on('pageerror',e=>errors.push(e.message));
    let requests=0,version=0;
    await page.route('**/*',route=>{
      const u=new URL(route.request().url());let body,type;
      if(resources[u.pathname]){const [file,mime]=resources[u.pathname];body=fs.readFileSync(path.join(root,file));type=mime;}
      else if(u.pathname.startsWith('/research/process/')){
        requests++;type='application/json';const d=JSON.parse(JSON.stringify(fixtures[path.basename(u.pathname,'.json')]));
        if(u.pathname.endsWith('status.json')){d.published_at=Date.now()/1000;for(const row of Object.values(d.branches))row.state='waiting';if(version)d.branches.pmf.sample_identity[1]+=version;}
        else if(u.pathname.endsWith('pmf.json'))d.sequence+=version;
        body=JSON.stringify(d);
      }else return route.fulfill({status:404,body:'missing test resource'});
      return route.fulfill({body,contentType:type});
    });
    await page.goto('http://process.test:3001/research-process.html');
    await page.waitForFunction(()=>!document.querySelector('#download').disabled);
    await page.getByRole('button',{name:'RGB + 声学',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('#evidence').innerText.includes('128')||!document.querySelector('#download').disabled);
    await page.waitForTimeout(200);
    for(const branch of ['pmf','rgbnir','avs']){
      await page.locator(`[data-branch="${branch}"]`).click();report.stages[branch]=[];
      for(let step=0;step<6;step++){
        await page.locator(`[data-step="${step}"]`).click();
        await page.waitForTimeout(80);
        const content=await page.locator('#visual').innerText();assert.ok(content.length>30,`${branch}/${step} is empty`);
        report.stages[branch].push(await page.locator('#stage-title').innerText());
        assert.ok(!(await page.locator('#sample-status').innerText()).includes('正在接收记录'),'waiting snapshots must be labelled saved');
        if(await page.locator('#feature-select').count()){
          const options=await page.locator('#feature-select option').evaluateAll(o=>o.map(x=>x.value));
          if(options.length>1){await page.selectOption('#feature-select',options[1]);assert.equal(await page.locator('#feature-select').inputValue(),options[1]);}
        }
        if(branch==='pmf'&&step===0){
          const canvas=page.locator('#raw-cloud'),box=await canvas.boundingBox();
          const before=await canvas.evaluate(c=>c.toDataURL());
          await page.mouse.move(box.x+box.width/2,box.y+box.height/2);await page.mouse.down();await page.mouse.move(box.x+box.width/2+70,box.y+box.height/2+25);await page.mouse.up();
          assert.notEqual(await canvas.evaluate(c=>c.toDataURL()),before,'point-cloud rotation must update canvas');
        }
        if(step===4||step===1)await page.screenshot({path:path.join(out,`${branch}-${step}.png`),fullPage:true});
      }
    }
    await page.locator('[data-branch="coverage"]').click();assert.equal(await page.locator('.coverage-item').count(),6);
    await page.screenshot({path:path.join(out,'coverage.png'),fullPage:true});
    await page.locator('[data-jump="pmf:4"]').click();assert.equal(await page.locator('[data-step="4"]').getAttribute('aria-pressed'),'true');
    await page.getByRole('button',{name:'冻结当前样本',exact:true}).click();const text=await page.locator('#sample-status').innerText(),n=requests;version=1;
    await page.waitForTimeout(4300);assert.equal(requests,n,'freeze must stop all polling');assert.equal(await page.locator('#sample-status').innerText(),text);
    await page.getByRole('button',{name:'恢复自动更新',exact:true}).click();await page.waitForFunction(seq=>document.querySelector('#sample-status').innerText.includes('序号 '+seq),baseSequence+1);
    const downloadEvent=page.waitForEvent('download');await page.getByRole('button',{name:'导出展示记录',exact:true}).click();const download=await downloadEvent;
    assert.equal(download.suggestedFilename(),`pmf-process-${baseSequence+1}.json`);
    await page.setViewportSize({width:390,height:844});await page.screenshot({path:path.join(out,'mobile.png'),fullPage:true});
    assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=window.innerWidth),'mobile page must not overflow');
    const missing=await browser.newPage({viewport:{width:1280,height:900}});
    await missing.route('**/*',route=>{const u=new URL(route.request().url());if(resources[u.pathname]){const [file,mime]=resources[u.pathname];return route.fulfill({body:fs.readFileSync(path.join(root,file)),contentType:mime});}return route.fulfill({contentType:'application/json',body:JSON.stringify({schema:'research_process_status/1',published_at:Date.now()/1000,branches:{pmf:{state:'waiting',detail:'无传感器输入'}}})});});
    await missing.goto('http://process.test:3001/research-process.html');await missing.waitForTimeout(100);
    assert.ok(await missing.locator('#visual').innerText().then(x=>x.includes('等待该分支')));assert.equal(await missing.locator('#download').isDisabled(),true);
    assert.deepEqual(errors,[]);report.errors=errors;report.freeze=true;report.mobile=true;report.missing_input=true;
    if(process.argv.includes('--live')){
      const live=await browser.newPage({viewport:{width:1500,height:1000}});
      live.on('pageerror',e=>errors.push('live: '+e.message));
      await live.goto('http://127.0.0.1:3001/');
      await live.locator('#research-process-entry').waitFor({timeout:20000});
      await live.waitForFunction(()=>document.querySelectorAll('.panel-shell').length===9,{},{timeout:20000});
      const layout=await live.evaluate(()=>JSON.parse(localStorage.getItem('miti_layout_config')));
      assert.equal(layout.widgets.length,9,'the original nine-panel layout must remain');
      assert.equal(await live.locator('#research-process-entry').getAttribute('href'),'/research-process.html');
      await live.screenshot({path:path.join(out,'live-dashboard.png'),fullPage:true});
      await live.locator('#research-process-entry').click();await live.locator('#stage-title').waitFor();
      await live.waitForFunction(()=>!document.querySelector('#download').disabled);
      await live.locator('[data-branch="avs"]').click();await live.locator('[data-step="4"]').click();
      await live.screenshot({path:path.join(out,'live-process.png'),fullPage:true});
      report.live={entry:true,nine_panels:true,url:live.url(),status:await live.locator('#sample-status').innerText()};
      assert.deepEqual(errors,[]);
    }
    fs.writeFileSync(path.join(out,'report.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
