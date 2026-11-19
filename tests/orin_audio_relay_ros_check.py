"""Isolated DDS check: preserve audio payloads, reject duplicate relays, stop cleanly."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rtabmap_features_msgs.msg import AudioFrame
from std_msgs.msg import String
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    contexts, nodes, executors = [], [], []
    process = None
    result_path = ROOT / 'logs/audio/yamnet/orin_relay_ros_check.json'
    result_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='orin-audio-relay-') as temporary:
        directory = Path(temporary)
        profile = yaml.safe_load((ROOT / 'config/orin_network_perception.yaml').read_text())
        profile['network'].update(interface='lo', display_host='127.0.0.1', ros_domain_id=97)
        (directory / 'profile.yaml').write_text(yaml.safe_dump(profile))
        domain_xml = '<General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery>'
        (directory / 'sink.xml').write_text('<CycloneDDS><Domain>' + domain_xml + '</Domain></CycloneDDS>')
        os.environ.update(RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', ROS_LOCALHOST_ONLY='0',
            CYCLONEDDS_URI='<CycloneDDS>' + ''.join('<Domain Id="' + str(d) + '">' + domain_xml + '</Domain>' for d in [97, 98]) + '</CycloneDDS>')
        command = [sys.executable, str(ROOT / 'scripts/relay_orin_audio.py'), '--profile', str(directory / 'profile.yaml'),
            '--interface', 'lo', '--orin-ip', '127.0.0.1', '--sink-domain', '98',
            '--sink-dds', str(directory / 'sink.xml'), '--output', str(directory / 'relay.json')]
        try:
            for domain in [97, 98]:
                context = Context()
                context.init(args=[], domain_id=domain)
                contexts.append(context)
                node = rclpy.create_node('audio_relay_check_' + str(domain), context=context)
                nodes.append(node)
                executor = SingleThreadedExecutor(context=context)
                executor.add_node(node)
                executors.append(executor)
            frame = AudioFrame()
            frame.header.stamp.sec, frame.header.stamp.nanosec = 1234, 5678
            frame.header.frame_id = 'orin_microphone_array'
            frame.source_id, frame.stamp_quality = 'test-orin-source', 'host_read_end_estimate'
            frame.sample_rate, frame.channels, frame.samples = 16000, 1, [.125, -.25, .5]
            expected = {'/multimodal/audio': frame,
                '/perception/audio/events': String(data='{"schema":"isolated_relay_test","sequence":7}'),
                '/perception/audio/status': String(data='{"state":"isolated_test"}')}
            received, publishers = {}, {}
            for topic, message in expected.items():
                qos = qos_profile_sensor_data if isinstance(message, AudioFrame) else 10
                publishers[topic] = nodes[0].create_publisher(type(message), topic, qos)
                nodes[1].create_subscription(type(message), topic, lambda value, t=topic: received.update({t: value}), qos)
            with (result_path.parent / 'orin_relay_ros_check.log').open('w') as log:
                process = subprocess.Popen(command, env=os.environ.copy(), stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 15
                while len(received) != len(expected) and time.monotonic() < deadline:
                    assert process.poll() is None, 'Relay exited before receiving messages'
                    for topic, publisher in publishers.items():
                        publisher.publish(expected[topic])
                    for executor in executors:
                        executor.spin_once(timeout_sec=.05)
                assert received == expected, 'Audio timestamps/samples or result strings changed during relay'
                duplicate = subprocess.run(command, env=os.environ.copy(), capture_output=True, text=True, timeout=15)
                assert duplicate.returncode == 1 and 'already running' in duplicate.stderr
                process.send_signal(signal.SIGTERM)
                assert process.wait(timeout=10) == 0
                report = json.loads((directory / 'relay.json').read_text())
                assert not report['running'] and all(report['forwarded'][topic] for topic in expected)
                result = dict(passed=True, source_domain=97, sink_domain=98,
                    payloads_and_timestamps_preserved=True, duplicate_rejected=True, shutdown_clean=True,
                    forwarded=report['forwarded'])
                result_path.write_text(json.dumps(result, indent=2) + '\n')
                print(json.dumps(result))
        finally:
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=10)
            for executor in executors:
                executor.shutdown()
            for node in nodes:
                node.destroy_node()
            for context in contexts:
                context.try_shutdown()


if __name__ == '__main__':
    main()
