#!/usr/bin/env python3
"""Bidirectional ROS link probe; run on both peers in the same isolated domain."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('host', 'orin'), required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    peer = 'orin' if args.side == 'host' else 'host'
    rclpy.init(args=[])
    node = rclpy.create_node('environment_link_probe_' + args.side)
    received = []

    def accept(message):
        if message.data == args.session:
            received.append(time.monotonic())

    pub = node.create_publisher(String, '/rtabmap_environment/link/' + args.side, 10)
    sub = node.create_subscription(String, '/rtabmap_environment/link/' + peer, accept, 10)
    timer = node.create_timer(0.2, lambda: pub.publish(String(data=args.session)))
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = dict(passed=len(received) >= 5, side=args.side, peer=peer,
                      received=len(received), session=args.session,
                      note='Discovery and small-message transport only; not a sensor bandwidth test.')
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
