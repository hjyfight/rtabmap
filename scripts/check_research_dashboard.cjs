// Fresh browser context: checks only the independent 3001 dashboard.
const fs=require('node:fs');
const path=require('node:path');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(__dirname,'..');
const waiting=process.argv.includes('--waiting');
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:path.join(root,'.tools/research-browsers/chrome-linux64/chrome'),
    args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-dev-shm-usage']});
  try {
    const page=await browser.newPage({viewport:{width:1920,height:1080}}),errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    await page.goto('http://127.0.0.1:3001/',{waitUntil:'domcontentloaded'});
    if(!waiting){
      for(const task of ['pmf','avs','rgbnir'])await page.locator(`.fusion-research[data-task="${task}"] img`).first().waitFor({timeout:60000});
    }
    await page.waitForTimeout(2500);
    const widgets=await page.evaluate(()=>JSON.parse(localStorage.getItem('miti_layout_config')));
    console.log(JSON.stringify(widgets.widgets.map(w=>({id:w.i,type:w.type,title:w.title}))));
    await page.screenshot({path:path.join(root,'logs/research/dashboard-before-check.png')});
    if(widgets.widgets.length!==9)throw Error('Expected nine panels including RGB/NIR features');
    const panels=await page.locator('.panel-shell').evaluateAll(nodes=>nodes.map(node=>({text:node.innerText.slice(0,100),
      x:node.getBoundingClientRect().x,y:node.getBoundingClientRect().y,w:node.getBoundingClientRect().width,h:node.getBoundingClientRect().height})));
    if(panels.length!==9||panels.some(p=>p.w<80||p.h<70))throw Error('Missing or collapsed panel');
    if(waiting){
      for(const task of ['pmf','avs','rgbnir']){
        const panel=page.locator(`.fusion-research[data-task="${task}"]`);
        await panel.getByRole('link',{name:'查看已保存真实帧的离线对照',exact:true}).waitFor();
      }
      if(errors.length)throw Error(errors.join('\n'));
      await page.screenshot({path:path.join(root,'logs/research/dashboard-waiting.png')});
      const report={mode:'live_dashboard_waiting_for_sensors',url:page.url(),panels,page_errors:errors,
        bridge:await page.evaluate(()=>JSON.parse(localStorage.getItem('miti_bridge_url')))};
      fs.writeFileSync(path.join(root,'logs/research/browser-layout-waiting.json'),JSON.stringify(report,null,2));
      console.log(JSON.stringify({panels:panels.length,page_errors:errors,bridge:report.bridge,mode:report.mode}));
      return;
    }
    const loaded=await page.locator('.fusion-research img').evaluateAll(nodes=>nodes.every(n=>n.complete&&n.naturalWidth>0));
    if(!loaded)throw Error('Inference images did not decode');
    await page.screenshot({path:path.join(root,'logs/research/dashboard-results.png')});
    for(const task of ['pmf','avs','rgbnir'])await page.locator(`.fusion-research[data-task="${task}"]`).getByRole('button',{name:'特征融合',exact:true}).click();
    const counts={};
    for(const task of ['pmf','avs','rgbnir'])counts[task]=await page.locator(`.fusion-research[data-task="${task}"] .fr-feature`).count();
    if(counts.pmf<4||counts.avs!==10||counts.rgbnir!==12)throw Error('Actual feature traces missing');
    await page.screenshot({path:path.join(root,'logs/research/dashboard-features.png')});
    if(errors.length)throw Error(errors.join('\n'));
    const report={url:page.url(),panels,feature_cards:counts,page_errors:errors,bridge:await page.evaluate(()=>JSON.parse(localStorage.getItem('miti_bridge_url')))};
    fs.writeFileSync(path.join(root,'logs/research/browser-check.json'),JSON.stringify(report,null,2));
    console.log(JSON.stringify({panels:panels.length,feature_cards:counts,page_errors:errors,bridge:report.bridge}));
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
