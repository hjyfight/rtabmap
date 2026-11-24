#!/usr/bin/env python3
"""Own only the 3001 process groups; no global pkill, mapping or 3000 changes."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from fusion_research_display import display_commands

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'.tools/miti-research/root/usr/lib/miti'
PYTHON=ROOT/'.venv-fusion-research/bin/python3'


def port_open(port):
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=.5):return True
    except OSError:return False


def bridge_open(port):
    try:
        with socket.create_connection(('127.0.0.1',port),timeout=1) as stream:
            stream.sendall(b'GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n')
            response=stream.recv(4096).lower()
            if response.startswith(b'http/1.1 101') and b'sec-websocket-protocol: foxglove' in response:
                stream.sendall(bytes([0x88,0x80,0,0,0,0]));return True
    except OSError:pass
    return False


def stop_process(process):
    if process.poll() is None:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:pass
    try:process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        process.wait()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audio',choices=['auto','reuse'],default='auto')
    parser.add_argument('--alsa-device',default='hw:CARD=XFMDPV0018,DEV=0')
    parser.add_argument('--remote-research-url', help='Display Orin research outputs instead of running local models')
    parser.add_argument('--remote-clock-offset', type=float, default=0., help='Workstation wall clock minus Orin wall clock')
    parser.add_argument('--remote-display-host', help='Receive Orin display topics over compressed TCP instead of running local adapters')
    args=parser.parse_args()
    directory=ROOT/'logs/research';directory.mkdir(exist_ok=True)
    with (directory/'dashboard-3001.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('3001研究服务已启动，请访问 http://localhost:3001/')
        if port_open(3001):raise SystemExit('3001端口已被占用；不会移动到3000或其他端口')
        if not args.remote_research_url and not PYTHON.exists():raise SystemExit('缺少 .venv-fusion-research 推理环境')
        subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/install_fusion_research_dashboard.py')],check=True)
        env=dict(os.environ,PORT='3001',HOSTNAME='0.0.0.0',MITI_BRIDGE_PORT='8766',BRIDGE_PORT='8766',BRIDGE_PORT_FIXED='1',
                 MITI_STATE_DIR=str(directory/'miti-state'),PYTHONUNBUFFERED='1',OMP_NUM_THREADS='2',MKL_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1',
                 BRIDGE_TOPIC_WHITELIST="['^/(tf|tf_static|cloud_map|map|grid_map|robot_pose|icp_odom|info)$','^/miti/.*$','^/camera/.*(image_raw|image_rect|image_rect_raw|camera_info)(/compressed)?$','^/livox/lidar$']")
        # Never expose 3000's bridge control token to the independent web server.
        env.pop('MITI_BRIDGE_CONTROL',None);env.pop('MITI_BRIDGE_TOKEN',None)
        children=[];streams=[]
        def spawn(name,command,cwd=ROOT):
            stream=(directory/(name+'.log')).open('a');streams.append(stream)
            process=subprocess.Popen([str(x) for x in command],cwd=cwd,env=env,stdin=subprocess.DEVNULL,
                                     stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            children.append((name,process));return process
        def interrupted(*_):raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,interrupted);signal.signal(signal.SIGINT,interrupted)
        try:
            if args.remote_display_host:
                spawn('remote-display',['/usr/bin/python3','-u',ROOT/'scripts/research_display_transport.py',
                    'receive','--host',args.remote_display_host,'--port','7449','--socket-timeout','10'])
            else:
                library=ROOT/'build/dynamic_cloud/libdynamic_cloud.so'
                if not library.exists() or (ROOT/'scripts/dynamic_cloud_filter.cpp').stat().st_mtime>library.stat().st_mtime:
                    subprocess.run(['bash',str(ROOT/'scripts/build_dynamic_cloud.sh')],env=env,check=True)
                for name, command in display_commands(ROOT):
                    spawn(name, command)
            if port_open(8766):
                if not bridge_open(8766):raise RuntimeError('8766被其他服务占用')
            else:
                bridge=spawn('bridge',['bash',APP/'ros/foxglove-bridge.sh'])
                deadline=time.monotonic()+15
                while not bridge_open(8766):
                    if bridge.poll() is not None or time.monotonic()>deadline:
                        raise RuntimeError('3001独立ROS桥启动失败，请查看 logs/research/bridge.log')
                    time.sleep(.3)
            config=json.loads((ROOT/'config/fusion_research.json').read_text())
            if not args.remote_research_url and config.get('clock_mode')=='ssh_preview':
                report_path=directory/'clock.json'
                try:
                    subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/measure_fourmodal_clock.py'),
                        '--sensor-fallback','--output',str(report_path)],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,
                        timeout=28,check=True)
                    report=json.loads(report_path.read_text())
                    offset=report['workstation_minus_robot_s']
                    config['rgb_clock_offset_s']=config['lidar_clock_offset_s']=offset
                    config['nir_clock_offset_s']=offset
                    config['clock_alignment']=report
                    uncertainty=report.get('round_trip_uncertainty_s')
                    quality=(f'往返不确定度 ±{uncertainty:.3f}s' if uncertainty is not None
                             else '接收时间估计，包含未知采集与传输延迟')
                    print(f'3001时钟偏移估计：{offset:.3f}s；{quality}（非硬件同步）',flush=True)
                except (subprocess.SubprocessError,ValueError,KeyError) as error:
                    print('3001未取得当前时钟估计；保留配置偏移，时间不匹配时等待：'+str(error),flush=True)
            runtime=directory/'runtime.json';runtime.write_text(json.dumps(config,indent=2))
            inference_python=ROOT/'.venv-indoor-research/bin/python3' if config.get('pmf_backend') in ('ptv3_compare','odin_compare') else PYTHON
            if args.remote_research_url:
                spawn('remote-research',['/usr/bin/python3','-u',ROOT/'scripts/research_result_transport.py',
                    'pull','--directory',APP/'public/research','--url',args.remote_research_url,
                    '--clock-offset',str(args.remote_clock_offset)])
            else:
                if not inference_python.exists():raise RuntimeError('缺少所选室内模型的独立推理环境')
                spawn('inference',[inference_python,'-u',ROOT/'scripts/run_fusion_research.py','--config',runtime])
                if args.audio=='auto':spawn('audio',[PYTHON,'-u',ROOT/'scripts/research_audio_capture.py','--alsa-device',args.alsa_device])
            exporter_python=ROOT/'.venv-indoor-research/bin/python3'
            if not exporter_python.exists():exporter_python=PYTHON
            spawn('process-export',[exporter_python,'-u',ROOT/'scripts/export_research_process.py',
                  '--directory',APP/'public/research'])
            spawn('web',[APP/'node/bin/node',APP/'server.js'],APP)
            (directory/'supervisor.json').write_text(json.dumps(dict(pid=os.getpid(),web_port=3001,bridge_port=8766,
                 children={name:process.pid for name,process in children}),indent=2))
            print('3001已启动：http://localhost:3001/  （两张建图视图、室内LiDAR、音视频、RGB/NIR）\n日志：logs/research/\nCtrl+C只停止本次3001服务。',flush=True)
            while True:
                for name,process in children:
                    if name!='audio' and process.poll() is not None:raise RuntimeError(f'{name}退出，请查看 logs/research/{name}.log')
                time.sleep(1)
        except KeyboardInterrupt:print('停止3001研究服务。',flush=True)
        finally:
            for _,process in reversed(children):stop_process(process)
            for stream in streams:stream.close()


if __name__=='__main__':main()
