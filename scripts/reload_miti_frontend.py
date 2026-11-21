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
    parser.add_argument('--port',type=int,default=3000)
    parser.add_argument('--bridge-port',type=int,help='Use this existing Foxglove bridge in the reloaded web instance')
    parser.add_argument('--restart-display',action='store_true',help='Restart a project-managed display that owns its ROS bridge')
    parser.add_argument('--dds-config',type=Path,help='Use this Cyclone DDS XML configuration for the restarted display')
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
    if '--bridge' in argv and not args.restart_display:
        raise ValueError('This instance owns the ROS bridge; pass --restart-display to reload the project display')
    if '--port' not in argv or argv[argv.index('--port')+1]!=str(args.port):
        raise ValueError('Supervisor preferred port differs from the selected listener')
    cwd=proc.joinpath('cwd').resolve()
    env=dict(v.decode().split('=',1) for v in (proc/'environ').read_bytes().split(b'\0') if b'=' in v)
    if '--bridge' in argv:
        launcher_pid=int(re.search(r'^PPid:\s+(\d+)',proc.joinpath('status').read_text(),re.M)[1])
        launcher=Path('/proc')/str(launcher_pid)
        launcher_argv=[v.decode() for v in launcher.joinpath('cmdline').read_bytes().split(b'\0') if v]
        if len(launcher_argv)<2 or Path(launcher_argv[1]).resolve()!=ROOT/'scripts/start_miti.sh':
            raise ValueError('Cannot safely restart a bridge-owning instance outside the project launcher')
        parent_pid=launcher_pid
        argv=launcher_argv
        cwd=launcher.joinpath('cwd').resolve()
        env=dict(v.decode().split('=',1) for v in launcher.joinpath('environ').read_bytes().split(b'\0') if b'=' in v)
        if 'BRIDGE_TOPIC_WHITELIST' in env:
            import ast
            topics=ast.literal_eval(env['BRIDGE_TOPIC_WHITELIST'])
            if not isinstance(topics,list) or not all(isinstance(x,str) for x in topics):
                raise ValueError('Cannot preserve the existing bridge topic filter')
            topics.append('^/perception/audio/(events|status|direction)$')
            env['BRIDGE_TOPIC_WHITELIST']=repr(list(dict.fromkeys(topics)))
    if args.bridge_port is not None:
        env['BRIDGE_PORT']=str(args.bridge_port)
    if args.dds_config is not None:
        import xml.etree.ElementTree as ET
        config=args.dds_config.resolve(strict=True)
        tree=ET.parse(config).getroot()
        if tree.tag!='CycloneDDS':raise ValueError('DDS configuration must be a CycloneDDS XML document')
        for interface in tree.findall('.//NetworkInterface'):
            name=interface.get('name')
            if name and not Path('/sys/class/net',name).exists():
                raise ValueError('DDS interface is not available: '+name)
        env['CYCLONEDDS_URI']=config.as_uri()
        env['ROS_LOCALHOST_ONLY']='0'
    output=ROOT/'logs/miti';output.mkdir(parents=True,exist_ok=True)
    log=output/('frontend-'+str(args.port)+'.log')
    print('Reloading the project display on port '+str(args.port),flush=True)
    workers=[]
    if args.restart_display:
        children=Path('/proc',str(parent_pid),'task',str(parent_pid),'children')
        for pid in map(int,children.read_text().split()):
            command=Path('/proc',str(pid),'cmdline')
            try:
                data=command.read_bytes()
                if any(str(ROOT/'scripts'/name).encode() in data for name in (
                        'miti_scene_adapter.py','dynamic_cloud_node.py','map_geometry_node.py',
                        'grid_projection_node.py','graph_feature_map_node.py','navigation_topology_node.py')):
                    workers.append(pid);os.kill(pid,signal.SIGINT)
            except (FileNotFoundError,ProcessLookupError):pass
    os.kill(parent_pid,signal.SIGTERM)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        with socket.socket() as connection:
            if connection.connect_ex(('127.0.0.1',args.port))!=0:break
        time.sleep(.1)
    else:raise TimeoutError('Old web listener did not stop')
    # The launcher releases its domain lock after its last display child exits.
    deadline=time.monotonic()+3
    while Path('/proc',str(parent_pid)).exists() and time.monotonic()<deadline:
        time.sleep(.1)
    for pid in workers:
        if Path('/proc',str(pid)).exists():
            try:os.kill(pid,signal.SIGKILL)
            except ProcessLookupError:pass
    if args.restart_display:
        import fcntl
        with (ROOT/'logs/miti'/('domain-'+env.get('ROS_DOMAIN_ID','0')+'.lock')).open('a') as lock:
            deadline=time.monotonic()+8
            while True:
                try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);break
                except BlockingIOError:
                    if time.monotonic()>=deadline:raise TimeoutError('Old display lock was not released')
                    time.sleep(.1)
            fcntl.flock(lock,fcntl.LOCK_UN)
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
                browser_visual_verified=False,ros_bridge_restarted=args.restart_display,
                orin_capture_restarted=False)
            report['bridge_port']=args.bridge_port
            report['dds_config']=env.get('CYCLONEDDS_URI')
            (output/('frontend-'+str(args.port)+'-verified.json')).write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report,indent=2));return
        except (OSError,ValueError) as error:last_error=str(error)
        time.sleep(.2)
    raise TimeoutError('Frontend verification failed: '+last_error)


if __name__=='__main__':main()
