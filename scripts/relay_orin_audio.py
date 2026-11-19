#!/usr/bin/env python3
"""Forward only AudioFrame and YAMNet results between the Orin and host ROS domains."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

from orin_network_perception import environment, load_profile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT / 'config/orin_network_perception.yaml')
    parser.add_argument('--interface', default='wlx6c1ff712636d', help='Workstation NIC facing Orin')
    parser.add_argument('--orin-ip', default='192.168.1.103')
    parser.add_argument('--sink-domain', type=int, default=0)
    parser.add_argument('--sink-dds', type=Path, default=ROOT / 'config/wired_cyclonedds.xml')
    parser.add_argument('--output', type=Path, default=ROOT / 'logs/audio/yamnet/orin_relay.json')
    args = parser.parse_args()
    profile = load_profile(args.profile)
    profile['network'].update(interface=args.interface, display_host=args.orin_ip)
    # Validate overridden addresses using the same profile parser.
    import ipaddress
    import re
    ipaddress.IPv4Address(args.orin_ip)
    if not re.fullmatch(r'[A-Za-z0-9_.:-]+', args.interface):
        parser.error('Invalid source network interface')
    source_domain = profile['network']['ros_domain_id']
    if not 0 <= args.sink_domain <= 232 or source_domain == args.sink_domain:
        parser.error('Source and sink must be distinct ROS domains in 0..232')
    env = environment(profile)
    source_xml = ET.fromstring(env['CYCLONEDDS_URI']).find('Domain')
    sink_xml = ET.parse(args.sink_dds).getroot().find('Domain')
    if sink_xml is None:
        parser.error('Sink DDS config needs a Domain element')
    source_xml.set('Id', str(source_domain))
    sink_xml.set('Id', str(args.sink_domain))
    config = ET.Element('CycloneDDS')
    config.extend([source_xml, sink_xml])
    env['CYCLONEDDS_URI'] = ET.tostring(config, encoding='unicode')
    os.environ.update(env)

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rtabmap_features_msgs.msg import AudioFrame
    from std_msgs.msg import String

    topics = [('/multimodal/audio', AudioFrame, qos_profile_sensor_data),
              ('/perception/audio/events', String, 10), ('/perception/audio/status', String, 10)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = ROOT / 'logs/audio/yamnet' / f'relay_{source_domain}_to_{args.sink_domain}.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    contexts, nodes, executors = [], [], []
    with lock_path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('This Orin audio relay is already running') from None
        try:
            for domain, name in [(source_domain, 'orin_audio_relay_source'),
                                 (args.sink_domain, 'orin_audio_relay_sink')]:
                context = Context()
                context.init(args=[], domain_id=domain)
                contexts.append(context)
                node = rclpy.create_node(name, context=context)
                nodes.append(node)
                executor = SingleThreadedExecutor(context=context)
                executor.add_node(node)
                executors.append(executor)
            source, sink = nodes
            deadline = time.monotonic() + 2.5
            while time.monotonic() < deadline:
                executors[1].spin_once(timeout_sec=.1)
            existing = {topic: [info.node_name for info in sink.get_publishers_info_by_topic(topic)]
                        for topic, _, _ in topics}
            if any(existing.values()):
                raise ValueError('Stop the old workstation microphone/classifier before relaying: ' +
                                 json.dumps(existing))
            counts = {topic: 0 for topic, _, _ in topics}
            received_at = {}
            for topic, message_type, qos in topics:
                publisher = sink.create_publisher(message_type, topic, qos)
                def forward(message, topic=topic, publisher=publisher):
                    publisher.publish(message)
                    counts[topic] += 1
                    received_at[topic] = time.time()
                source.create_subscription(message_type, topic, forward, qos)
            running = True
            def stop(*_):
                nonlocal running
                running = False
            signal.signal(signal.SIGTERM, stop)
            signal.signal(signal.SIGINT, stop)
            print(f'Orin audio relay: domain {source_domain} -> {args.sink_domain}', flush=True)
            last_write = 0
            def write_report():
                report = dict(running=running, pid=os.getpid(), source_domain=source_domain,
                    sink_domain=args.sink_domain, orin_ip=args.orin_ip, forwarded=counts,
                    last_received_at=received_at, updated_at=time.time())
                temporary = args.output.with_suffix('.tmp')
                temporary.write_text(json.dumps(report, indent=2) + '\n')
                temporary.replace(args.output)
            while running:
                executors[0].spin_once(timeout_sec=.1)
                executors[1].spin_once(timeout_sec=0)
                if time.monotonic() - last_write >= 1:
                    write_report()
                    last_write = time.monotonic()
            write_report()
        finally:
            for executor in executors:
                executor.shutdown()
            for node in nodes:
                node.destroy_node()
            for context in contexts:
                context.try_shutdown()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
