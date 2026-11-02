#!/usr/bin/env python3
"""Install project-owned panels and point-cloud controls into MITI 3.3.6.

MITI 3.3.6 has no external panel loader. Keep readable panel source in config/,
verify upstream bundle hashes, and retain originals before adding the component,
panel registry entry and icon. No npm download or modification of the SLAM stack.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
APP = PROJECT / '.tools/miti/root/usr/lib/miti'
UPSTREAM = {
    '.next/static/chunks/453.e8d48709a26a0ed3.js':
        '58944591c246585141396dfc5938f0c0cd795b357fefe0e8b6f43ba5ed6bba2d',
    '.next/static/chunks/app/page-a3758698d17bc097.js':
        'cda0901d64d8d42ffb66b0088a992402c02049b656a60a3482e8561911d67375',
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise ValueError('MITI panel integration point changed: ' + old[:70])
    return source.replace(old, new, 1)


def extend_bundle(path, source, panel, height_clip):
    if path.endswith('453.e8d48709a26a0ed3.js'):
        source = replace_once(source, 'function rO(e){',
            panel + '\nconst RtabmapLoopStatusPanel = rtabmapLoopStatusFactory(n).Component;\n' +
            height_clip + '\nconst RtabmapHeightClip = rtabmapHeightClipFactory(n);\nfunction rO(e){')
        source = replace_once(source, 'switch(t.type){case"topics":',
            'switch(t.type){case"loop-status":return(0,r.jsx)(RtabmapLoopStatusPanel,'
            '{client:a,topic:t.props&&t.props.topic});case"topics":')
        source = replace_once(source, 'function te(e){let{points:t,style:a}=e,',
            'function te(e){const rtabmapClip=n.useContext(RtabmapHeightClip.Context);let{points:t,style:a}=e,')
        source = replace_once(source,
            'for(let e of((0,n.useEffect)(()=>()=>{j.dispose(),w.dispose()},[j,w]),',
            'RtabmapHeightClip.applyMaterial(j,rtabmapClip);RtabmapHeightClip.applyMaterial(w,rtabmapClip);'
            'for(let e of((0,n.useEffect)(()=>()=>{j.dispose(),w.dispose()},[j,w]),')
        # The provider is inside the Canvas, below the display-follow group. Its inverse
        # transform keeps clipping in the fixed frame even while following the robot.
        scene = '(0,r.jsx)(ab,{client:t,topics:a,topicRows:W,layers:l.layers,tfTree:A,fixedFrame:D,' \
                'displayFrame:L,sync:s.syncTimestamps,transforms:u,scene:c,meshEpoch:S,onStatus:h,onInfo:g,onFrame:y})'
        source = replace_once(source, scene,
            '(0,r.jsx)(RtabmapHeightClip.Provider,{settings:c.heightClip,follow:B,children:' + scene + '})')
        return replace_once(source, 'className:d.u4.urdfViewer.container,children:[',
            'className:d.u4.urdfViewer.container,children:['
            '(0,r.jsx)(RtabmapHeightClip.Controls,{settings:c.heightClip,frame:D,'
            'onChange:e=>o({...l,scene:{...c,heightClip:e}})}),')
    source = replace_once(source, 'let k=[{type:"topics",',
        'let k=[{type:"loop-status",label:"回环状态",icon:"↻",'
        'defaultSize:{w:4,h:8},minSize:{minW:2,minH:4}},{type:"topics",')
    return replace_once(source, '"button-switch":c.Z,log:d.Z}',
                        '"button-switch":c.Z,"loop-status":l.Z,log:d.Z}')


def atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.rtabmap-tmp')
    temporary.write_bytes(data)
    temporary.replace(path)


def install(app=APP):
    version = json.loads((app / 'package.json').read_text())['version']
    if version != '3.3.6':
        raise ValueError(f'Loop panel supports MITI 3.3.6, found {version}; update the integration first.')
    panel = (PROJECT / 'config/miti_loop_status.js').read_text()
    height_clip = (PROJECT / 'config/miti_height_clip.js').read_text()
    state_dir = app / 'rtabmap-panel-originals'
    state_file = state_dir / 'installed.json'
    previous = json.loads(state_file.read_text()) if state_file.exists() else {}
    prepared = []
    # Validate every input before modifying any frontend bundle.
    for name, expected in UPSTREAM.items():
        target, backup = app / name, state_dir / name
        current = target.read_bytes()
        if digest(current) == expected:
            original = current
        else:
            if digest(current) != previous.get(name):
                raise ValueError(f'Unexpected MITI bundle change: {target}. No files were modified.')
            original = backup.read_bytes()
        if digest(original) != expected:
            raise ValueError(f'Original MITI bundle checksum mismatch: {backup}')
        updated = extend_bundle(name, original.decode(), panel, height_clip).encode()
        prepared.append((name, target, backup, original, updated, current))
    # Syntax-check the actual combined bundles, not just the standalone component.
    with tempfile.TemporaryDirectory(prefix='miti_panel_check_') as directory:
        for name, _, _, _, updated, _ in prepared:
            check = Path(directory) / Path(name).name
            check.write_bytes(updated)
            subprocess.run([str(app / 'node/bin/node'), '--check', str(check)], check=True)
    changed = False
    for _, target, backup, original, updated, current in prepared:
        if not backup.exists():
            atomic_write(backup, original)
        if updated != current:
            atomic_write(target, updated)
            changed = True
    hashes = {name: digest(updated) for name, _, _, _, updated, _ in prepared}
    atomic_write(state_file, (json.dumps(hashes, indent=2) + '\n').encode())
    # The launcher refreshes these two cached bundles before applying the new layout.
    assets = {'version': digest((panel + '\n' + height_clip).encode())[:12],
              'assets': [name.replace('.next/', '/_next/', 1) for name in UPSTREAM]}
    atomic_write(app / 'public/rtabmap-panel-assets.json',
                 (json.dumps(assets, indent=2) + '\n').encode())
    print('MITI project panels and height clipping: ' + ('installed' if changed else 'up to date'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path, default=APP)
    install(parser.parse_args().app)
