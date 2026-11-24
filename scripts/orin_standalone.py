#!/usr/bin/env python3
"""Local sensors, existing semantic/research compute and Orin-hosted MITI."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from orin_research_web import web_check, port_open

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/orin_direct_research.json')
    args = parser.parse_args()
    core = ['/usr/bin/python3', str(ROOT / 'scripts/orin_research_wireless.py'), '--profile', str(args.profile)]
    if args.check:
        result = subprocess.run(core + ['--check'], check=False)
        checked = web_check()
        print(json.dumps(dict(web=checked, windows_installation='Edge/Chrome and SSH client only'), indent=2))
        return 0 if result.returncode == 0 and checked['ready'] else 1
    base = ROOT / 'logs/orin_standalone'
    base.mkdir(parents=True, exist_ok=True)
    with (base / 'supervisor.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        subprocess.run(core + ['--check', '--require-hardware'], check=True)
        if not web_check()['ready']:
            raise RuntimeError('Orin web runtime is incomplete')
        commands = [('compute', core)]
        if port_open(3001):
            meta = ROOT / 'logs/orin_web/current_run.json'
            proc = Path('/proc', str(json.loads(meta.read_text())['pid']), 'cmdline') if meta.is_file() else Path('/missing')
            if not proc.is_file() or b'orin_research_web.py' not in proc.read_bytes():
                raise RuntimeError('Port 3001 is occupied by another service')
        else:
            commands.append(('web', ['/usr/bin/python3', str(ROOT / 'scripts/orin_research_web.py')]))
        children, outputs = [], []
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        try:
            for name, command in commands:
                output = (base / (name + '.log')).open('a')
                outputs.append(output)
                child = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                    stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                children.append((name, child))
            (base / 'current_run.json').write_text(json.dumps(dict(pid=os.getpid(),
                children={name: child.pid for name, child in children}), indent=2))
            print('Orin standalone started; visit http://<Orin LAN IP>:3001/', flush=True)
            while all(child.poll() is None for _, child in children):
                time.sleep(.5)
            raise RuntimeError('A standalone worker exited; inspect logs/orin_standalone')
        except KeyboardInterrupt:
            pass
        finally:
            for _, child in children:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGTERM)
            for _, child in children:
                try:
                    child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
            for output in outputs:
                output.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
