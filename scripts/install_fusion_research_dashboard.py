#!/usr/bin/env python3
"""Copy MITI 3000 and install research widgets only into the independent copy."""
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
from fusion_research_display import private_display_source, private_layout

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'.tools/miti/root/usr/lib/miti'
TARGET=ROOT/'.tools/miti-research/root/usr/lib/miti'


def layout():
    value=json.loads((ROOT/'config/miti_dashboard_layout.json').read_text())
    value['rtabmapResearch']=True
    keep={'rtabmap-scene','rtabmap-grid-scene','rtabmap-loop-status','rtabmap-color','rtabmap-infrared','rtabmap-depth'}
    value['widgets']=[w for w in value['widgets'] if w['i'] in keep]
    for w in value['widgets']:
        w.pop('rtabmapHiddenAuxiliaryMaps',None)
    value['mosaic']['splitPercentage']=54
    value['mosaic']['second']={'direction':'row','splitPercentage':50,'first':'research-pmf','second':'research-avs'}
    backend=json.loads((ROOT/'config/fusion_research.json').read_text()).get('pmf_backend')
    title='室内 RGB + 深度 + LiDAR · ODIN-Swin / PTv3' if backend=='odin_compare' else '室内 RGB + LiDAR · PTv3 / PMF' if backend=='ptv3_compare' else 'PMF · RGB + LiDAR'
    for task,title in [('pmf',title),('avs','AVSegFormer · RGB + 音频')]:
        value['widgets'].append(dict(i='research-'+task,type='fusion-research',title=title,x=0 if task=='pmf' else 6,y=12,w=6,h=10,props=dict(task=task)))
    if json.loads((ROOT/'config/fusion_research.json').read_text()).get('rgbnir_enabled'):
        value['mosaic']['second']={'direction':'column','splitPercentage':58,
            'first':value['mosaic']['second'],'second':'research-rgbnir'}
        value['widgets'].append(dict(i='research-rgbnir',type='fusion-research',title='YOLOv8-seg · RGB 90% + NIR 10%',
            x=0,y=22,w=12,h=8,props=dict(task='rgbnir')))
    return private_layout(value)


def install():
    if not SOURCE.exists():raise SystemExit('请先安装项目现有MITI')
    marker=TARGET/'.research-copy-complete'
    if not marker.exists():
        # The apt bundle's .next/cache points outside the app into /var/cache.
        # It is runtime state, and must neither be shared nor dereferenced.
        def ignore(directory,names):
            return {'cache'} if Path(directory)==SOURCE/'.next' else {'.miti'} if Path(directory)==SOURCE else set()
        shutil.copytree(SOURCE,TARGET,symlinks=False,dirs_exist_ok=True,ignore=ignore)
        marker.write_text('Independent copy; never run a research installer on the source app.\n')
    # All inputs and mutable .next/public files are private to this installation.
    spec=importlib.util.spec_from_file_location('base_installer',ROOT/'scripts/install_miti_loop_panel.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original_extend=module.extend_bundle
    panel=(ROOT/'config/miti_research_panel.js').read_text()
    def extend(name,*args,**kwargs):
        source=original_extend(name,*args,**kwargs)
        if name.endswith('453.e8d48709a26a0ed3.js'):
            source=module.replace_once(source,'function rO(e){',panel+'\nconst RtabmapResearchPanel=rtabmapResearchFactory(n).Component;\nfunction rO(e){')
            return module.replace_once(source,'switch(t.type){case"loop-status":',
                'switch(t.type){case"fusion-research":return(0,r.jsx)(RtabmapResearchPanel,{task:t.props&&t.props.task});case"loop-status":')
        source=module.replace_once(source,'let k=[{type:"loop-status",',
            'let k=[{type:"fusion-research",label:"融合研究",icon:"◈",defaultSize:{w:8,h:10},minSize:{minW:4,minH:5}},{type:"loop-status",')
        source=module.replace_once(source,'"loop-status":l.Z,','"fusion-research":l.Z,"loop-status":l.Z,')
        source=module.replace_once(source,'es(window.location.port!=="3000"&&window.innerWidth>=1024)','es(false)')
        # Preserve the research layout through legacy migrations.
        return source.replace('function rtabmapMigrateAudioEventsLayout(layout) {',
                              'function rtabmapMigrateAudioEventsLayout(layout) {if(typeof window!=="undefined"&&window.location.port==="3001")return layout;')
    module.extend_bundle=extend
    with tempfile.TemporaryDirectory(prefix='miti_research_install_') as temporary:
        stage=Path(temporary);shutil.copytree(ROOT/'config',stage/'config')
        for path in (stage/'config').iterdir():
            if path.suffix in ('.js', '.json', '.html'):
                path.write_text(private_display_source(path.read_text()))
        research=layout()
        settings=dict(schema=1,revision='fusion-research-3001-v12',port=3001,bridgePort=8766,replaceLayout=True,
                      stylesheet='/rtabmap-dashboard.css',defaultLayout='miti_dashboard_layout.json',
                      mapLayout='miti_live_layout.json',mapWidgetId='rtabmap-scene')
        (stage/'config/miti_dashboard_layout.json').write_text(json.dumps(research,ensure_ascii=False,indent=2))
        (stage/'config/miti_workspace_sync.json').write_text(json.dumps(settings))
        shutil.copyfile(ROOT/'config/miti_research_workspace.js',stage/'config/miti_workspace_sync.js')
        (stage/'config/miti_branding.json').write_text(json.dumps(dict(title='多源感知 · 融合研究实验室')))
        module.PROJECT=stage
        module.install(TARGET)
    public=TARGET/'public'
    (public/'rtabmap-research-layout.json').write_text(json.dumps(research,ensure_ascii=False,indent=2))
    (public/'rtabmap.html').write_text('<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>融合研究</title><script>location.replace("/")</script><a href="/">进入 PMF / AVSegFormer 融合研究页面</a></html>')
    directory=public/'research';directory.mkdir(exist_ok=True)
    offline=ROOT/'logs/research/offline'
    if (offline/'index.html').exists():
        shutil.copytree(offline,directory/'offline',dirs_exist_ok=True)
    for key in ('pmf','avs','rgbnir'):
        path=directory/f'{key}.json'
        if not path.exists():
            path.write_text(json.dumps(dict(schema='downstream_fusion/1',session_id='initial',task=key,published_at=0,state='waiting',detail='等待3001推理服务启动',inputs={},result=None,model=None,ros_error='')))
        artifact=directory/f'{key}_features.npz'
        if not artifact.exists():artifact.write_bytes(b'')
    print('独立研究页面已安装：3001；3000源目录未修改')


if __name__=='__main__':install()
