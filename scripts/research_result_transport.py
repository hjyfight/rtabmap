#!/usr/bin/env python3
"""Read-only Orin research snapshots and a bandwidth-bounded MITI file relay."""
import argparse
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

TASKS = ('pmf', 'avs', 'rgbnir')
MAX_JSON = 16 * 1024 * 1024


def atomic_json(path, value):
    temporary = path.with_suffix('.relay.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False))
    temporary.replace(path)


def result_id(value):
    result = value.get('result') or {}
    model = value.get('model') or {}
    fields = [value.get('session_id'), result.get('sequence'), result.get('source_stamp'), model.get('sha256')]
    return hashlib.sha256(json.dumps(fields, sort_keys=True).encode()).hexdigest()


def manifest(directory):
    rows = {}
    for task in TASKS:
        path = directory / (task + '.json')
        if not path.is_file():
            continue
        value = json.loads(path.read_text())
        if value.get('schema') != 'downstream_fusion/1' or value.get('task') != task:
            raise ValueError('Invalid research snapshot: ' + task)
        # Images/features are fetched only when the actual inference result changes.
        status = {key: value.get(key) for key in ('schema', 'session_id', 'task', 'published_at',
                  'state', 'detail', 'inputs', 'ros_error', 'evaluation', 'offline_report')}
        rows[task] = dict(result_id=result_id(value), status=status)
    return dict(schema='orin_research_manifest/1', server_time=time.time(), tasks=rows)


def display_snapshot(value, clock_offset, base_url):
    result = copy.deepcopy(value)
    result['source_mode'] = 'live_ros'
    result['display_published_at'] = result['published_at'] + clock_offset
    for item in result.get('inputs', {}).values():
        item['display_stamp'] = item['stamp'] + clock_offset
    if result.get('result'):
        data = result['result']
        data['display_source_stamp'] = data['source_stamp'] + clock_offset
        if data.get('feature_download'):
            name = Path(data['feature_download']).name
            data['feature_download'] = base_url.rstrip('/') + '/' + name
    result['remote_compute'] = dict(url=base_url, device='cuda:0', host_minus_orin_s=clock_offset)
    return result


def handler_for(directory):
    directory = directory.resolve()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            name = urllib.parse.urlparse(self.path).path.lstrip('/')
            try:
                if name == 'manifest.json':
                    payload = json.dumps(manifest(directory), ensure_ascii=False, allow_nan=False).encode()
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json; charset=utf-8')
                    self.send_header('Content-Length', str(len(payload)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                allowed = [task + suffix for task in TASKS for suffix in ('.json', '_features.npz')]
                if name not in allowed:
                    self.send_error(404)
                    return
                path = directory / name
                if not path.is_file() or path.resolve().parent != directory:
                    self.send_error(404)
                    return
                with path.open('rb') as stream:
                    size = path.stat().st_size
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json' if name.endswith('.json') else 'application/octet-stream')
                    self.send_header('Content-Length', str(size))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    while True:
                        block = stream.read(64 * 1024)
                        if not block:
                            break
                        self.wfile.write(block)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except (OSError, ValueError, KeyError):
                self.send_error(503, 'Research output is not ready')
    return Handler


def pull(args):
    directory = args.directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    # LAN requests must bypass the workstation's Internet proxy/TUN settings.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    cached, ids = {}, {}
    stopped = threading.Event()
    def stop(*_):
        stopped.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    for task in TASKS:
        atomic_json(directory / (task + '.json'), dict(schema='downstream_fusion/1',
            session_id='remote-pending', task=task, published_at=0, state='waiting',
            detail='等待 Orin 无线推理结果', inputs={}, result=None, model=None, ros_error=''))
    stats = dict(manifests=0, result_fetches=0, bytes=0, errors=0, connected=False)
    def get(name):
        with opener.open(args.url.rstrip('/') + '/' + name, timeout=4) as response:
            payload = response.read(MAX_JSON + 1)
        if len(payload) > MAX_JSON:
            raise ValueError('Oversized research JSON')
        stats['bytes'] += len(payload)
        return json.loads(payload)
    last_report = 0
    while not stopped.is_set():
        try:
            value = get('manifest.json')
            if value.get('schema') != 'orin_research_manifest/1':
                raise ValueError('Unexpected research manifest')
            stats['manifests'] += 1
            for task, row in value['tasks'].items():
                if task not in TASKS:
                    continue
                fetched = False
                if task not in cached or ids.get(task) != row['result_id']:
                    snapshot = get(task + '.json')
                    if snapshot.get('schema') != 'downstream_fusion/1' or snapshot.get('task') != task:
                        raise ValueError('Unexpected task snapshot')
                    cached[task], ids[task] = snapshot, result_id(snapshot)
                    stats['result_fetches'] += 1
                    fetched = True
                # Keep a newer result fetched during a producer update coherent.
                if not fetched or ids[task] == row['result_id']:
                    cached[task].update(row['status'])
                atomic_json(directory / (task + '.json'), display_snapshot(cached[task], args.clock_offset, args.url))
            stats['connected'] = True
            stats.pop('last_error', None)
            stats['last_received_at'] = time.time()
        except (OSError, ValueError, KeyError, urllib.error.URLError) as error:
            stats['errors'] += 1
            stats['connected'] = False
            stats['last_error'] = str(error)
        atomic_json(directory / 'remote_transport.json', stats)
        if time.monotonic() - last_report > 10:
            print(json.dumps(stats), flush=True)
            last_report = time.monotonic()
        stopped.wait(args.interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    server = sub.add_parser('serve')
    server.add_argument('--directory', type=Path, required=True)
    server.add_argument('--host', default='0.0.0.0')
    server.add_argument('--port', type=int, default=7448)
    client = sub.add_parser('pull')
    client.add_argument('--directory', type=Path, required=True)
    client.add_argument('--url', required=True)
    client.add_argument('--clock-offset', type=float, default=0)
    client.add_argument('--interval', type=float, default=.5)
    args = parser.parse_args()
    if args.mode == 'pull':
        if not .1 <= args.interval <= 10:
            parser.error('Require an interval of 0.1 to 10 seconds')
        pull(args)
    else:
        args.directory.mkdir(parents=True, exist_ok=True)
        service = ThreadingHTTPServer((args.host, args.port), handler_for(args.directory))
        def stop(*_):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop)
        try:
            print('Orin research snapshots listening on ' + args.host + ':' + str(args.port), flush=True)
            service.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            service.server_close()


if __name__ == '__main__':
    main()
