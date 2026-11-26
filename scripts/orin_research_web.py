#!/usr/bin/env python3
"""Host MITI on Orin; any LAN computer can display it using a browser."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

from orin_network_perception import environment

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / '.tools/miti-research/root/usr/lib/miti'
NODE = ROOT / '.tools/node-v22.16.0-linux-arm64/bin/node'
LOOPBACK = ('<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces>'
    '<AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex>'
    '<MaxAutoParticipantIndex>50</MaxAutoParticipantIndex><Peers><Peer Address="127.0.0.1"/>'
    '</Peers></Discovery></Domain></CycloneDDS>')


def web_check():
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    try:
        bridge = Path(get_package_prefix('foxglove_bridge')) / 'lib/foxglove_bridge/foxglove_bridge'
    except PackageNotFoundError:
        bridge = Path('/missing/foxglove_bridge')
    checks = dict(arm64_node=NODE.is_file(), web_server=(APP / 'server.js').is_file(),
        next_runtime=(APP / 'node_modules/next/package.json').is_file(), native_ros_bridge=bridge.is_file())
    return dict(ready=all(checks.values()), checks=checks, bridge=str(bridge))


def link_results():
    meta = ROOT / 'logs/go2_orin_research/current_run.json'
    if not meta.is_file():
        return None
    run = Path(json.loads(meta.read_text())['run']).resolve()
    if ROOT / 'logs/go2_orin_research' not in run.parents:
        raise ValueError('Research directory must be inside this project')
    source = run / 'public'
    if not source.is_dir():
        return None
    target = APP / 'public/research'
    if target.is_symlink() and target.resolve() == source:
        return str(source)
    if target.exists() and not target.is_symlink():
        target.rename(target.with_name('research_before_orin_web_' + str(time.time_ns())))
    temporary = target.with_name('.research-current-link')
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(source, target_is_directory=True)
    temporary.replace(target)
    return str(source)


def port_open(port):
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=.5):
            return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    checked = web_check()
    if args.check:
        print(json.dumps(checked, indent=2))
        return 0 if checked['ready'] else 1
    if not checked['ready']:
        raise RuntimeError('Deploy the portable ARM64 web runtime first: ' + str(checked))
    base = ROOT / 'logs/orin_web'
    base.mkdir(parents=True, exist_ok=True)
    with (base / 'server.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if any(port_open(port) for port in (3001, 8766, 7450)):
            raise RuntimeError('Orin web port 3001/8766/7450 is already occupied')
        env = environment({'network': dict(interface='lo', display_host='127.0.0.1', ros_domain_id=74)})
        env.update(CYCLONEDDS_URI=LOOPBACK, PORT='3001', HOSTNAME='0.0.0.0',
            MITI_STATE_DIR=str(base / 'miti-state'), MITI_BRIDGE_PORT='8766')
        env.pop('MITI_BRIDGE_CONTROL', None)
        env.pop('MITI_BRIDGE_TOKEN', None)
        viewer_env = dict(env, ROS_DOMAIN_ID='75')
        whitelist = "['^/(tf|tf_static|icp_odom)$','^/miti/research/display/.*$','^/camera/.*$']"
        commands = [
            ('process_export', [str(ROOT/'.venv-indoor-research/bin/python3'), '-u',
                str(ROOT/'scripts/export_research_process.py'), '--directory', str(APP/'public/research'),
                '--allow-snapshot-only', '--interval', '1'], env),
            ('mapping_export', ['/usr/bin/python3', '-u', str(ROOT/'scripts/export_research_mapping.py'),
                '--directory', str(APP/'public/research/mapping')], env),
            ('display_send', ['/usr/bin/python3', '-u', str(ROOT / 'scripts/research_display_transport.py'),
                'send', '--host', '127.0.0.1', '--port', '7450', '--socket-timeout', '10'], env),
            ('display_receive', ['/usr/bin/python3', '-u', str(ROOT / 'scripts/research_display_transport.py'),
                'receive', '--host', '127.0.0.1', '--port', '7450', '--socket-timeout', '10'], viewer_env),
            ('bridge', [checked['bridge'], '--ros-args', '-p', 'port:=8766', '-p', 'address:=0.0.0.0',
                '-p', 'topic_whitelist:=' + whitelist, '-p', "capabilities:=['connectionGraph']",
                '-p', 'send_buffer_limit:=10000000', '-p', 'max_qos_depth:=10'], viewer_env),
            ('web', [str(NODE), str(APP / 'server.js')], env),
        ]
        children, outputs = [], []
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            link_results()
            # Initialize the active run before Next indexes its static public paths.
            subprocess.run(['/usr/bin/python3', str(ROOT / 'scripts/install_fusion_research_dashboard.py')], check=True)
            # Next does not index newly produced files inside the live symlink.
            # Fall back to the existing read-only result service for those paths.
            manifest_path = APP / '.next/routes-manifest.json'
            manifest = json.loads(manifest_path.read_text())
            rewrites = manifest.get('rewrites', [])
            if isinstance(rewrites, list):
                rewrites = dict(beforeFiles=[], afterFiles=rewrites, fallback=[])
            source = '/research/:path*'
            rewrites['afterFiles'] = [row for row in rewrites.get('afterFiles', []) if row.get('source') != source]
            rewrites['afterFiles'].append(dict(source=source,
                destination='http://127.0.0.1:7448/:path*',
                regex=r'^/research(?:/((?:[^/]+?)(?:/(?:[^/]+?))*))?(?:/)?$'))
            manifest['rewrites'] = rewrites
            temporary = manifest_path.with_suffix('.startup.tmp')
            temporary.write_text(json.dumps(manifest))
            temporary.replace(manifest_path)
            # Register initial static JSON paths before Next starts scanning public/.
            subprocess.run([str(ROOT/'.venv-indoor-research/bin/python3'),str(ROOT/'scripts/export_research_process.py'),
                '--directory',str(APP/'public/research'),'--allow-snapshot-only','--once'],env=env,check=True)
            for name, command, child_env in commands:
                output = (base / (name + '.log')).open('a')
                outputs.append(output)
                child = subprocess.Popen(command, cwd=APP if name == 'web' else ROOT, env=child_env,
                    stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                children.append((name, child))
            (base / 'current_run.json').write_text(json.dumps(dict(pid=os.getpid(), web_port=3001,
                bridge_port=8766, browser_ros_domain=75, children={name: child.pid for name, child in children}), indent=2))
            print('Orin web: http://<Orin LAN IP>:3001/ ; Windows needs only Edge/Chrome.', flush=True)
            while True:
                for name, child in children:
                    if child.poll() is not None:
                        raise RuntimeError(name + ' exited; inspect logs/orin_web/' + name + '.log')
                link_results()
                time.sleep(.5)
        except KeyboardInterrupt:
            pass
        finally:
            for _, child in reversed(children):
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            for output in outputs:
                output.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
