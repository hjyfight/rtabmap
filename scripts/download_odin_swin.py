#!/usr/bin/env python3
"""Resume the fixed official ODIN checkpoint and publish only after SHA256 verification."""
import hashlib
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
URL='https://huggingface.co/katefgroup/odin/resolve/1def72841925a64069dd42eceb9f0fde2ea01a57/scannet_swin_semantic_77.8_64k_2k.pth'
SIZE=1421630345
SHA256='fc4e7ffd4507ea334ed7d96c5be146b786fd18c4abd42cb39138bd39c71856f5'
TARGET=ROOT/'models/odin_scannet_swin_semantic.pth.download'


def digest(path):
    value=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):value.update(block)
    return value.hexdigest()


def main():
    if TARGET.exists() and TARGET.stat().st_size==SIZE and digest(TARGET)==SHA256:
        print('Official ODIN Swin checkpoint already verified');return
    partial=TARGET.with_suffix('.part')
    failures=0
    for attempt in range(20):
        start=partial.stat().st_size if partial.exists() else 0
        if start>SIZE:raise ValueError('Unexpected oversized partial checkpoint')
        if start==SIZE:break
        print(f'Official checkpoint download: {start}/{SIZE} bytes, attempt {attempt+1}',flush=True)
        result=subprocess.run(['curl','-fsSL','--connect-timeout','20','--max-time','600',
            '--retry','0','--http1.1','-C','-','-o',str(partial),URL])
        end=partial.stat().st_size if partial.exists() else 0
        if result.returncode==0 and end==SIZE:break
        failures=0 if end>start else failures+1
        if failures>=5:raise RuntimeError('Five consecutive download attempts made no progress')
        time.sleep(2)
    if not partial.exists() or partial.stat().st_size!=SIZE or digest(partial)!=SHA256:
        raise RuntimeError('Incomplete or incorrect official checkpoint; not published')
    partial.replace(TARGET)
    print(f'Official checkpoint verified: {SHA256}',flush=True)


if __name__=='__main__':main()
