#!/usr/bin/env python3
"""Calibrate the onboard clock against this workstation using measured UDP RTT.

Start serve on the workstation, then run sync with sudo on the onboard computer.
No SSH transfer delay is included in the clock estimate. Existing sensors and
mapping must be stopped before a clock step.
"""
import argparse
import datetime
import socket
import struct
import time

PACKET = struct.Struct('!4sddd')
MAGIC = b'RCLK'


def serve(host, port, seconds):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((host, port))
        print('Workstation clock service: {}:{}'.format(host, port), flush=True)
        sock.settimeout(1)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                data, peer = sock.recvfrom(PACKET.size)
            except socket.timeout:
                continue
            received = time.time()
            if len(data) != PACKET.size or data[:4] != MAGIC:
                continue
            _, sent, _, _ = PACKET.unpack(data)
            sock.sendto(PACKET.pack(MAGIC, sent, received, time.time()), peer)


def sync(host, port):
    samples = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((host, port))
        sock.settimeout(2)
        for _ in range(8):
            sent = time.time()
            sock.send(PACKET.pack(MAGIC, sent, 0, 0))
            try:
                data = sock.recv(PACKET.size)
            except (socket.timeout, ConnectionRefusedError):
                continue
            received = time.time()
            if len(data) != PACKET.size:
                continue
            magic, echoed, remote_received, remote_sent = PACKET.unpack(data)
            if magic != MAGIC or echoed != sent:
                continue
            rtt = (received - sent) - (remote_sent - remote_received)
            offset = ((remote_received - sent) + (remote_sent - received)) / 2
            if 0 <= rtt < 2:
                samples.append((rtt, offset))
    if not samples:
        raise SystemExit('Clock sync failed: no valid response from workstation')
    rtt, offset = min(samples)
    time.clock_settime(time.CLOCK_REALTIME, time.time() + offset)
    print('Clock synchronized: offset={:.6f}s, minimum RTT={:.6f}s, UTC={}'.format(
        offset, rtt, datetime.datetime.now(datetime.timezone.utc).isoformat()), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['serve', 'sync'])
    parser.add_argument('--host', required=True)
    parser.add_argument('--port', type=int, default=7446)
    parser.add_argument('--seconds', type=float, default=180)
    args = parser.parse_args()
    if args.mode == 'serve':
        serve(args.host, args.port, args.seconds)
    else:
        sync(args.host, args.port)


if __name__ == '__main__':
    main()
