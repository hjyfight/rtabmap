#!/usr/bin/env python3
"""Share an existing PCM publisher or capture M260C under its existing lock."""
import argparse
import fcntl
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]


def main():
    import rclpy
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--alsa-device',default='hw:CARD=XFMDPV0018,DEV=0')
    args=parser.parse_args()
    directory=ROOT/'logs/audio/yamnet/host';directory.mkdir(parents=True,exist_ok=True)
    key=re.sub(r'[^A-Za-z0-9_.-]','_',args.alsa_device)
    with (directory/(key+'.lock')).open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print('M260C已有采集服务；3001复用 /multimodal/audio',flush=True)
            return
        rclpy.init();node=rclpy.create_node('research_audio_source_check')
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        count=node.count_publishers('/multimodal/audio')
        node.destroy_node();rclpy.shutdown()
        if count:
            print('已有PCM发布者；3001复用 /multimodal/audio',flush=True)
            return
        process=None
        def stop(*_):raise KeyboardInterrupt
        signal.signal(signal.SIGTERM,stop)
        try:
            process=subprocess.Popen([sys.executable,'-u',str(ROOT/'scripts/publish_fourmodal_audio.py'),
                '--alsa-device',args.alsa_device,'--frame-id','host_microphone_array'],cwd=ROOT)
            process.wait()
        except KeyboardInterrupt:pass
        finally:
            if process and process.poll() is None:
                process.terminate()
                try:process.wait(timeout=4)
                except subprocess.TimeoutExpired:process.kill();process.wait()


if __name__=='__main__':main()
