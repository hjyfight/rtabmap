#!/usr/bin/env python3
"""Monitor the connected M260C: handshake ACKs and version query only.

Protocol reference: https://github.com/jt-z/m260c-voice-assistant
No third-party Python packages required. No speech recognition or robot control.
"""

import argparse
import copy
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import select
import struct
import sys
import termios
import time


DEFAULT_PORT = "/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0004-if00"
HANDSHAKE = bytes.fromhex("a5 00 00 00")


def frame(kind, sid, payload):
    body = b"\xa5\x01" + struct.pack("<BHH", kind, len(payload), sid) + payload
    return body + bytes([(-sum(body)) & 255])


class FrameParser:
    """Consume complete checked frames, retaining only the incomplete tail."""

    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksums = 0
        self.discarded_bytes = 0
        self.discarded_preview = bytearray()

    def discard(self, size):
        remaining = max(0, 1024 - len(self.discarded_preview))
        self.discarded_preview.extend(self.buffer[:min(size, remaining)])
        self.discarded_bytes += size
        del self.buffer[:size]

    def feed(self, data):
        self.buffer.extend(data)
        output = []
        while self.buffer:
            start = self.buffer.find(b"\xa5\x01")
            if start < 0:
                keep = 1 if self.buffer[-1] == 0xA5 else 0
                self.discard(len(self.buffer) - keep)
                break
            if start:
                self.discard(start)
            if len(self.buffer) < 7:
                break
            kind, size, sid = struct.unpack_from("<BHH", self.buffer, 2)
            total = size + 8
            if len(self.buffer) < total:
                break
            packet = self.buffer[:total]
            if sum(packet) & 255:
                self.bad_checksums += 1
                self.discard(1)
                continue
            output.append((kind, sid, bytes(packet[7:-1])))
            del self.buffer[:total]
        return output


def usb_identity(port):
    tty_name = Path(port).resolve(strict=True).name
    node = (Path("/sys/class/tty") / tty_name / "device").resolve(strict=True)
    for parent in (node, *node.parents):
        if (parent / "idVendor").is_file():
            return tuple((parent / name).read_text().strip().lower()
                         for name in ("idVendor", "idProduct"))
    raise OSError(f"无法确认串口的 USB 标识：{port}")


def nested_values(value, key, depth=0):
    if depth > 12:
        return []
    found = []
    if isinstance(value, dict):
        if key in value:
            found.append(value[key])
        for item in value.values():
            found.extend(nested_values(item, key, depth + 1))
    elif isinstance(value, list):
        for item in value:
            found.extend(nested_values(item, key, depth + 1))
    elif isinstance(value, str) and value.lstrip().startswith(("{", "[")):
        try:
            found.extend(nested_values(json.loads(value), key, depth + 1))
        except (ValueError, RecursionError):
            pass
    return found


def write_packet(fd, packet):
    pending = memoryview(packet)
    deadline = time.monotonic() + 1.0
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("串口写入超时")
        if not select.select([], [fd], [], min(remaining, 0.1))[1]:
            continue
        try:
            count = os.write(fd, pending)
        except BlockingIOError:
            continue
        if count == 0:
            raise OSError("串口写入返回 0")
        pending = pending[count:]


def run(args, emit):
    identity = usb_identity(args.port)
    if identity != ("1a86", "55d4"):
        raise OSError(f"该串口不是已确认的 M260C USB 转串口（实际 {identity}）")
    parser = FrameParser()
    stats = dict(received_bytes=0, valid_frames=0, handshake_acks=0,
                 version_queries=0, angle_events=0)
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = None
    recent_events = {}
    interrupted = False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.ioctl(fd, termios.TIOCEXCL)
        original = termios.tcgetattr(fd)
        settings = copy.deepcopy(original)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.CRTSCTS)
        settings[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[4] = settings[5] = termios.B115200
        settings[6][termios.VMIN] = 1
        settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        emit(dict(event="opened", port=args.port, baud=115200,
                  seconds=args.seconds, note="请尝试设备原有唤醒词；Ctrl+C 可结束"))
        started = last_handshake = time.monotonic()
        next_query = started + 1.0
        version_reply = False
        while time.monotonic() - started < args.seconds:
            now = time.monotonic()
            if (stats["version_queries"] < 3 and not version_reply
                    and now >= next_query and now - last_handshake >= 1.0):
                write_packet(fd, frame(0x05, 1, b'{"type":"version"}'))
                stats["version_queries"] += 1
                next_query = now + 4.0
                emit(dict(event="version_query", attempt=stats["version_queries"]))
            if not select.select([fd], [], [], 0.1)[0]:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise OSError("串口已断开")
            stats["received_bytes"] += len(chunk)
            for kind, sid, payload in parser.feed(chunk):
                stats["valid_frames"] += 1
                if kind == 0x01 and payload == HANDSHAKE:
                    write_packet(fd, frame(0xFF, sid, HANDSHAKE))
                    stats["handshake_acks"] += 1
                    last_handshake = time.monotonic()
                    emit(dict(event="handshake_ack", sid=sid))
                    continue
                try:
                    obj = json.loads(payload.decode("utf-8"))
                except (UnicodeError, ValueError, RecursionError):
                    obj = None
                angles = [v for v in nested_values(obj, "angle")
                          if isinstance(v, (int, float)) and not isinstance(v, bool)
                          and math.isfinite(v) and 0 <= v < 360]
                stamps = nested_values(obj, "start_ms")
                stamp = json.dumps(stamps, sort_keys=True) if stamps else None
                event_time = time.monotonic()
                recent_events = {k: t for k, t in recent_events.items() if event_time - t < 30}
                duplicate = bool(angles and stamp and stamp in recent_events)
                if angles and not duplicate:
                    stats["angle_events"] += 1
                    if stamp:
                        recent_events[stamp] = event_time
                if kind == 0x04 and obj is not None:
                    kinds = nested_values(obj, "type")
                    if "version" in kinds or nested_values(obj, "version"):
                        version_reply = True
                event = dict(event="frame", kind=f"0x{kind:02x}", sid=sid,
                             payload_bytes=len(payload))
                if obj is None:
                    event["payload_hex_prefix"] = payload[:128].hex(" ")
                else:
                    event["payload"] = obj
                if angles:
                    event.update(angles_deg=sorted(set(angles)), duplicate=duplicate,
                                 note="阵列自身坐标系下的报告角度")
                emit(event)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        if original is not None:
            try:
                termios.tcsetattr(fd, termios.TCSANOW, original)
            except (termios.error, OSError):
                pass
        os.close(fd)
    emit(dict(event="summary", **stats, bad_checksums=parser.bad_checksums,
              discarded_bytes=parser.discarded_bytes,
              unframed_text_prefix=parser.discarded_preview.decode("utf-8", errors="replace"),
              buffered_tail_bytes=len(parser.buffer), interrupted=interrupted,
              note="未收到角度事件不能判定声源定位损坏；须有实际唤醒测试"))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=DEFAULT_PORT, help="指定已确认的阵列串口")
    ap.add_argument("--seconds", type=float, default=60, help="监听时长，默认 60 秒")
    ap.add_argument("--log", type=Path, help="可选：另存 JSONL 日志")
    args = ap.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        ap.error("--seconds 必须是有限正数")
    output = None
    try:
        if args.log:
            args.log.parent.mkdir(parents=True, exist_ok=True)
            output = args.log.open("x", encoding="utf-8")

        def emit(event):
            event["time"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            line = json.dumps(event, ensure_ascii=False)
            print(line, flush=True)
            if output:
                output.write(line + "\n")
                output.flush()

        run(args, emit)
    except (OSError, termios.error) as error:
        print(f"M260C 检查失败：{error}", file=sys.stderr)
        return 1
    finally:
        if output:
            output.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
