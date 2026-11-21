#!/usr/bin/env python3
"""Keep the workstation's 3000 perception inputs running; reuse existing ROS producers."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

import rclpy

ROOT = Path(__file__).resolve().parents[1]
SERVICES = {
    'audio': ('/multimodal/audio', '/perception/audio/events', '/perception/audio/direction'),
    'features': ('/miti/fourmodal_features',),
    'segmentation': ('/miti/segmentation/view',),
    'fusion': ('/miti/segmentation/fusion_view',),
}


def service_state(node):
    return {name: {topic: len(node.get_publishers_info_by_topic(topic)) for topic in topics}
            for name, topics in SERVICES.items()}


def commands(gpu):
    device = ['--gpu'] if gpu else []
    return {
        'audio': ['bash', 'scripts/start_host_m260c.sh'],
        'features': ['bash', 'scripts/start_fourmodal_live.sh', '--reuse-audio', *device],
        'segmentation': ['bash', 'scripts/start_yoloe_segmentation_live.sh', *device],
        'fusion': ['bash', 'scripts/start_segmentation_fusion.sh', *device],
        'display': ['bash', 'scripts/start_dashboard_miti.sh'],
    }


def display_running(domain):
    try:
        http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with http.open('http://127.0.0.1:3000/api/bridge', timeout=2) as response:
            state = json.load(response)
        if state.get('domainId') != domain:
            raise RuntimeError('3000 展示使用不同 ROS 域；请先统一网络配置')
        return state.get('state') == 'running'
    except (OSError, ValueError):
        return False


def stop_owned(children):
    # Every child starts in a new process group. Existing services never enter this list.
    for child in children:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
    deadline = time.monotonic() + 20
    for child in children:
        try:
            child.wait(timeout=max(.1, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gpu', action='store_true')
    parser.add_argument('--check', action='store_true', help='Inspect existing publishers without starting services')
    args = parser.parse_args()
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if domain != 0:
        parser.error('当前主机统一控制台使用 domain 0；此入口不用于 Orin domain 74')
    directory = ROOT / 'logs/dashboard_perception'
    directory.mkdir(parents=True, exist_ok=True)
    lock = (directory / 'domain-0.lock').open('a')
    if not args.check:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('复用已经运行的 3000 感知服务管理器。', flush=True)
            return
    rclpy.init()
    node = rclpy.create_node('dashboard_perception_supervisor')
    children, outputs = [], []
    run = directory / (time.strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        state = service_state(node)
        display = display_running(domain)
        if args.check:
            print(json.dumps({'services': state, 'display_running': display}, ensure_ascii=False, indent=2))
            return
        run.mkdir()
        record = {'pid': os.getpid(), 'dds_config': os.environ.get('CYCLONEDDS_URI'),
                  'ros_domain_id': domain, 'run_directory': str(run), 'services': {}}
        planned = commands(args.gpu)
        for name, command in planned.items():
            available = display if name == 'display' else all(state[name].values())
            if available:
                record['services'][name] = {'reused': True}
                print('复用 ' + name + '。', flush=True)
                continue
            if name == 'audio' and any(state['audio'].values()):
                raise RuntimeError('音频服务只启动了一部分，请检查现有音频进程，避免重复打开麦克风')
            output = (run / (name + '.log')).open('w')
            outputs.append(output)
            child = subprocess.Popen(command, cwd=ROOT, stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
            children.append(child)
            record['services'][name] = {'reused': False, 'pid': child.pid, 'log': str(run / (name + '.log'))}
            print('启动 ' + name + '；日志：' + str(run / (name + '.log')), flush=True)
            if name == 'audio':
                deadline = time.monotonic() + 40
                while not all(service_state(node)['audio'].values()):
                    if child.poll() is not None or time.monotonic() >= deadline:
                        raise RuntimeError('M260C 音频未就绪：' + str(run / 'audio.log'))
                    rclpy.spin_once(node, timeout_sec=.1)
        (directory / 'current.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
        print('3000 感知服务已启动；UniBind 正在加载真实权重。', flush=True)
        while True:
            for name, service in record['services'].items():
                if service.get('reused'):
                    continue
                child = next(child for child in children if child.pid == service['pid'])
                if child.poll() is not None:
                    raise RuntimeError(name + ' 已退出；请查看 ' + service['log'])
            rclpy.spin_once(node, timeout_sec=.5)
    finally:
        stop_owned(children)
        for output in outputs:
            output.close()
        node.destroy_node()
        rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        main()
    except KeyboardInterrupt:
        print('已停止本入口启动的感知服务；复用的服务保持运行。', flush=True)
