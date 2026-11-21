#!/usr/bin/env python3
"""Publish M260C speech-wakeup bearings without changing its audio/wakeup settings."""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time

from m260c_serial_monitor import DEFAULT_PORT, run


def wakeup_result(payload):
    """Only accept a real wake event; beam commands and version replies are not DOA."""
    if not isinstance(payload, dict) or payload.get('type') != 'aiui_event':
        return None
    content = payload.get('content')
    if not isinstance(content, dict) or content.get('eventType') != 4:
        return None
    info = content.get('info')
    try:
        if isinstance(info, str):
            info = json.loads(info)
    except (ValueError, RecursionError):
        return None
    if not isinstance(info, dict):
        return None
    value = info.get('ivw', info)
    if not isinstance(value, dict):
        return None
    angle = value.get('angle')
    if (isinstance(angle, bool) or not isinstance(angle, (int, float)) or
            not math.isfinite(angle) or not 0 <= angle < 360):
        return None
    return {'angle_deg': float(angle), 'keyword': str(value.get('keyword', ''))[:128],
            'start_ms': value.get('start_ms'), 'end_ms': value.get('end_ms')}


class DirectionState:
    def __init__(self, clock=time.monotonic, frame_id='microphone_array'):
        self.clock = clock
        self.frame_id = frame_id
        self.sequence = 0
        self.state = 'connecting'
        self.last_error = ''
        self.last = None
        self.last_at = None
        self.firmware = None
        self.seen = {}
        self.lock = threading.Lock()

    def receive(self, event):
        with self.lock:
            if event.get('event') == 'opened':
                self.state = 'waiting_wakeup'
                self.last_error = ''
            if event.get('event') != 'frame':
                return
            payload = event.get('payload')
            if isinstance(payload, dict) and payload.get('type') == 'version':
                self.firmware = payload.get('content')
            result = wakeup_result(payload)
            if result is None or event.get('duplicate'):
                return
            now = self.clock()
            self.seen = {key: stamp for key, stamp in self.seen.items() if now-stamp < 30}
            key = (event.get('sid'), json.dumps(result, sort_keys=True))
            if key in self.seen:
                return
            self.seen[key] = now
            self.sequence += 1
            self.last, self.last_at = result, now
            self.state, self.last_error = 'tracking', ''

    def error(self, error):
        with self.lock:
            self.state, self.last_error = 'serial_error', str(error)[:512]

    def snapshot(self):
        with self.lock:
            age = None if self.last_at is None else max(0., self.clock()-self.last_at)
            state = self.state
            if state == 'tracking' and age >= 5:
                state = 'stale'
            return dict(schema='m260c_sound_direction/1', model='M260C',
                        sequence=self.sequence, published_at=time.time(), state=state,
                        angle_source='speech_wakeup',
                        angle_deg=None if self.last is None else self.last['angle_deg'],
                        keyword='' if self.last is None else self.last['keyword'], age_s=age,
                        frame_id=self.frame_id, coordinate_convention='array_native',
                        calibration_verified=False, distance_m=None, target_id=None,
                        accuracy_spec_deg=10, firmware=self.firmware, last_error=self.last_error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=DEFAULT_PORT)
    parser.add_argument('--journal', type=Path)
    parser.add_argument('--frame-id', default='microphone_array')
    args = parser.parse_args()
    import rclpy
    from std_msgs.msg import String
    state, stop = DirectionState(frame_id=args.frame_id), threading.Event()
    journal = None
    if args.journal:
        args.journal.parent.mkdir(parents=True, exist_ok=True)
        journal = args.journal.open('x')

    def receive(event):
        state.receive(event)
        if journal:
            journal.write(json.dumps(dict(received_at=time.time(), **event), ensure_ascii=False)+'\n')
            journal.flush()

    def listen():
        serial_args = argparse.Namespace(port=args.port, seconds=86400)
        while not stop.is_set():
            try:
                run(serial_args, receive, should_stop=stop.is_set)
            except (OSError, ValueError, TimeoutError) as error:
                state.error(error)
            stop.wait(2)

    rclpy.init()
    node = rclpy.create_node('m260c_sound_localization')
    publisher = node.create_publisher(String, '/perception/audio/direction', 10)
    worker = threading.Thread(target=listen, daemon=True)
    worker.start()
    timer = node.create_timer(.5, lambda: publisher.publish(String(data=json.dumps(state.snapshot(), ensure_ascii=False))))
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        stop.set()
        worker.join(timeout=3)
        with state.lock:
            state.state = 'stopped'
        publisher.publish(String(data=json.dumps(state.snapshot(), ensure_ascii=False)))
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.try_shutdown()
        if journal:
            journal.close()


if __name__ == '__main__':
    main()
