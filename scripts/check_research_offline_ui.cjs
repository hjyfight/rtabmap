// Render the actual research component with saved-input snapshots, without ROS or HTTP servers.
const fs=require('node:fs'),path=require('node:path');
const {chromium}=require('/usr/lib/chatgpt/resources/cua_node/lib/node_modules/playwright');
const root=path.resolve(__dirname,'..'),directory=path.join(root,'logs/research/offline');
const modules=path.join(root,'.tools/miti-research/root/usr/lib/miti/node_modules');
const sources={react:fs.readFileSync(path.join(modules,'react/cjs/react.production.min.js'),'utf8'),
  scheduler:fs.readFileSync(path.join(modules,'react-dom/node_modules/scheduler/cjs/scheduler.production.min.js'),'utf8'),
  'react-dom':fs.readFileSync(path.join(modules,'react-dom/cjs/react-dom.production.min.js'),'utf8')};
const loader=`const sources=${JSON.stringify(sources)},cache={};function require(name){if(cache[name])return cache[name].exports;const module={exports:{}};cache[name]=module;new Function('module','exports','require',sources[name])(module,module.exports,require);return module.exports;}`;
(async()=>{
  const browser=await chromium.launch({headless:true,executablePath:path.join(root,'.tools/research-browsers/chrome-linux64/chrome'),
    args:['--no-sandbox','--disable-dev-shm-usage']});
  try{
    const page=await browser.newPage({viewport:{width:1920,height:1080}}),errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    await page.route('**/*',async route=>{
      const url=new URL(route.request().url()),match=url.pathname.match(/^\/research\/(pmf|avs|rgbnir)\.json$/);
      if(match)return route.fulfill({contentType:'application/json',body:fs.readFileSync(path.join(directory,match[1]+'.json'),'utf8')});
      return route.fulfill({contentType:'text/html',body:'<html lang="zh-CN"><meta charset="utf-8"><body style="margin:0;background:#101b27"><h1 style="color:#e2edf6;font:24px system-ui;padding:12px">已保存真实数据 · 离线检查</h1><main id="root"></main></body></html>'});
    });
    await page.goto('http://offline.local:3001/');
    await page.addScriptTag({content:loader+'\n'+fs.readFileSync(path.join(root,'config/miti_research_panel.js'),'utf8')+`
      const React=require('react'),DOM=require('react-dom'),Panel=rtabmapResearchFactory(React).Component;
      DOM.render(React.createElement('div',{style:{display:'grid',gridTemplateColumns:'repeat(3,minmax(0,1fr))',gap:12,height:900}},
        ...['pmf','avs','rgbnir'].map(task=>React.createElement(Panel,{task,key:task}))),document.getElementById('root'));`});
    await page.locator('.fusion-research[data-task="rgbnir"] .fr-pictures img').first().waitFor();
    await page.locator('.fusion-research[data-task="pmf"] .fr-pictures img').first().waitFor();
    await page.locator('.fusion-research[data-task="avs"] .fr-pictures img').first().waitFor();
    await page.waitForFunction(()=>[...document.querySelectorAll('.fr-pictures img')].every(image=>image.complete&&image.naturalWidth>0));
    const report={mode:'offline_component_render',page_errors:errors,panels:{}};
    for(const task of ['pmf','avs','rgbnir']){
      const panel=page.locator(`.fusion-research[data-task="${task}"]`);
      if(!(await panel.innerText()).includes('离线回放'))throw Error(task+' lacks replay label');
      const original=await panel.locator('.fr-pictures img').first().getAttribute('src');
      await panel.getByRole('button',{name:'标注：开',exact:true}).click();
      if(original===await panel.locator('.fr-pictures img').first().getAttribute('src'))throw Error(task+' annotation toggle has no effect');
      await panel.getByRole('button',{name:'标注：关',exact:true}).click();
      report.panels[task]={captions:await panel.locator('figcaption').allTextContents()};
    }
    await page.screenshot({path:path.join(directory,'panels.png'),fullPage:true});
    const pointPanel=page.locator('.fusion-research[data-task="pmf"]');
    await pointPanel.getByRole('button',{name:fs.readFileSync(path.join(directory,'pmf.json'),'utf8').includes('rgbd_lidar_odin')?'LiDAR预测':'移除RGB对照',exact:true}).click();
    await page.locator('.fusion-research[data-task="rgbnir"]').getByRole('button',{name:'RGB对照',exact:true}).click();
    for(const task of ['pmf','avs','rgbnir']){
      const panel=page.locator(`.fusion-research[data-task="${task}"]`);
      await panel.getByRole('button',{name:'特征融合',exact:true}).click();
      report.panels[task].features=await panel.locator('.fr-feature').count();
      if(report.panels[task].features<4)throw Error(task+' feature cards missing');
    }
    if(report.panels.rgbnir.features!==12||report.panels.avs.features<10)throw Error('Missing feature traces');
    await page.screenshot({path:path.join(directory,'features.png'),fullPage:true});
    if(errors.length)throw Error(errors.join('\n'));
    fs.writeFileSync(path.join(directory,'browser-check.json'),JSON.stringify(report,null,2));
    console.log(JSON.stringify(report));
  }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1;});
