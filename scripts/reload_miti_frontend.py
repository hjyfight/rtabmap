#!/usr/bin/env python3
"""Reload a standalone MITI web instance and verify its served frontend version."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
from urllib.request import urlopen

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'.tools/miti/root/usr/lib/miti'


def fetch(url):
    with urlopen(url,timeout=3) as response:return response.read()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=3002)
    parser.add_argument('--bridge-port',type=int,help='Use this existing Foxglove bridge in the reloaded web instance')
    args=parser.parse_args()
    if not 1024<=args.port<=65535:raise ValueError('Invalid web port')
    if args.bridge_port is not None and not 1024<=args.bridge_port<=65535:raise ValueError('Invalid bridge port')
    subprocess.run(['python3',str(ROOT/'scripts/install_miti_loop_panel.py')],check=True)
    manifest=json.loads((APP/'public/rtabmap-panel-assets.json').read_text())
    base='http://127.0.0.1:'+str(args.port)
    listing=subprocess.check_output(['ss','-ltnp','sport = :'+str(args.port)],text=True)
    matches=re.findall(r'pid=(\d+)',listing)
    if len(set(matches))!=1:raise ValueError('Cannot identify exactly one web listener')
    web_pid=int(matches[0]);status=(Path('/proc')/str(web_pid)/'status').read_text()
    parent_pid=int(re.search(r'^PPid:\s+(\d+)',status,re.M)[1])
    proc=Path('/proc')/str(parent_pid)
    argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    if len(argv)<2 or Path(argv[1]).resolve()!=APP/'scripts/start-miti.mjs':
        raise ValueError('Web listener is not managed by this workspace MITI instance')
    if '--bridge' in argv:
        raise ValueError('This instance owns the ROS bridge; use a separate web-only instance for frontend reload')
    if '--port' not in argv or argv[argv.index('--port')+1]!=str(args.port):
        raise ValueError('Supervisor preferred port differs from the selected listener')
    cwd=proc.joinpath('cwd').resolve()
    env=dict(v.decode().split('=',1) for v in (proc/'environ').read_bytes().split(b'\0') if b'=' in v)
    if args.bridge_port is not None:
        env['BRIDGE_PORT']=str(args.bridge_port)
    output=ROOT/'logs/miti';output.mkdir(parents=True,exist_ok=True)
    log=output/('frontend-'+str(args.port)+'.log')
    print('Reloading the web-only MITI instance on port '+str(args.port),flush=True)
    os.kill(parent_pid,signal.SIGTERM)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with socket.socket() as connection:
            if connection.connect_ex(('127.0.0.1',args.port))!=0:break
        time.sleep(.1)
    else:raise TimeoutError('Old web listener did not stop')
    with log.open('ab') as stream:
        child=subprocess.Popen(argv,cwd=cwd,env=env,stdin=subprocess.DEVNULL,
            stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+25;last_error=''
    while time.monotonic()<deadline:
        if child.poll() is not None:raise RuntimeError('Replacement web instance exited; see '+str(log))
        try:
            html=fetch(base+'/?rtabmap_ui='+manifest['version']).decode()
            if manifest['assets'][1] not in html:raise ValueError('Server still advertises an older page chunk')
            if args.bridge_port is not None:
                bootstrap=re.search(r'window\.__MITI__=(\{[^;]+\});',html)
                if bootstrap is None or json.loads(bootstrap[1]).get('bridgePort')!=args.bridge_port:
                    raise ValueError('Server still advertises a different bridge port')
            for asset in manifest['assets']:
                actual=fetch(base+asset)
                disk=APP/asset.split('?',1)[0].replace('/_next/','.next/',1).lstrip('/')
                if actual!=disk.read_bytes():raise ValueError('Served frontend asset differs from installed content')
            report=dict(passed=True,port=args.port,previous_supervisor_pid=parent_pid,
                supervisor_pid=child.pid,version=manifest['version'],page_asset=manifest['assets'][1],
                lazy_renderer_asset=manifest['assets'][0],served_html_matches=True,served_assets_match=True,
                html_sha256=hashlib.sha256(html.encode()).hexdigest(),
                browser_visual_verified=False,ros_bridge_restarted=False)
            report['bridge_port']=args.bridge_port
            (output/('frontend-'+str(args.port)+'-verified.json')).write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2));return
        except (OSError,ValueError) as error:last_error=str(error)
        time.sleep(.2)
    raise TimeoutError('Frontend verification failed: '+last_error)


if __name__=='__main__':main()
