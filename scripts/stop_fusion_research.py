#!/usr/bin/env python3
"""Signal the verified project 3001 supervisor, which cleans only its children."""
import json
import os
from pathlib import Path
import signal

ROOT=Path(__file__).resolve().parents[1]


def main():
    path=ROOT/'logs/research/supervisor.json'
    if not path.exists():print('3001尚未启动');return
    info=json.loads(path.read_text());pid=info['pid']
    try:command=Path('/proc',str(pid),'cmdline').read_bytes().split(b'\0')
    except FileNotFoundError:print('3001已停止');return
    expected=str(ROOT/'scripts/supervise_fusion_research.py').encode()
    if not any(c==expected or c==b'scripts/supervise_fusion_research.py' for c in command):
        raise SystemExit('PID已改变；拒绝停止其他进程')
    if Path('/proc',str(pid),'cwd').resolve()!=ROOT:raise SystemExit('进程工作目录不匹配')
    os.kill(pid,signal.SIGTERM);print('已通知3001服务停止')


if __name__=='__main__':main()
