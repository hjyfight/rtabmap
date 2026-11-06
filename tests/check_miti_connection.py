#!/usr/bin/env python3
"""Integration check against start_miti.sh on isolated ROS domain 91, ports 13000/18765.

Publishes synthetic data only in the test domain, receives the actual Foxglove CDR frames,
and decodes them with ROS. This does not validate browser rendering or sensor performance.
"""
import base64
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import urllib.request

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as Stamp
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rtabmap_msgs.msg import MapGraph, RGBDImage
from rtabmap_semantic_msgs.msg import Observation, SemanticMap, SemanticObject
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

PROJECT = Path(__file__).resolve().parents[1]
APP = PROJECT / '.tools/miti/root/usr/lib/miti'
TYPES = {'/cloud_map': PointCloud2, '/camera/color/image_raw': Image,
         '/camera/depth_registered/image_rect': Image, '/tf_static': TFMessage,
         '/miti/trajectory': RosPath, '/miti/semantic_markers': MarkerArray,
         '/miti/aligned_depth/image_raw': Image, '/miti/aligned_depth/camera_info': CameraInfo,
         '/miti/loop_status': String, '/robot_pose': PoseStamped,
         '/miti/walls': MarkerArray, '/miti/map_status': String, '/map': OccupancyGrid,
         '/miti/grid_map': OccupancyGrid, '/miti/grid_status': String}
CLIENT = r"""
const WS = require(process.argv[1]);
const wanted = new Set(JSON.parse(process.argv[2]));
const socket = new WS('ws://127.0.0.1:18765', ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
const subscriptions = new Map();
const frames = {};
let finishing = false;
const timer = setTimeout(() => {
  console.error('Missing topics: ' + [...wanted].filter(t => !frames[t]).join(', '));
  socket.terminate(); process.exit(1);
}, 25000);
socket.on('error', error => {console.error(error.message); process.exit(1);});
socket.on('message', (data, binary) => {
  if (!binary) {
    const message = JSON.parse(data.toString());
    if (message.op === 'advertise') {
      const added = [];
      for (const channel of message.channels) {
        if (wanted.has(channel.topic) && ![...subscriptions.values()].includes(channel.topic)) {
          const id = subscriptions.size + 1;
          subscriptions.set(id, channel.topic);
          added.push({id, channelId: channel.id});
        }
      }
      if (added.length) socket.send(JSON.stringify({op: 'subscribe', subscriptions: added}));
    }
  } else if (data[0] === 1 && data.length > 13) {
    const topic = subscriptions.get(data.readUInt32LE(1));
    if (topic) frames[topic] = data.subarray(13).toString('base64');
    // The adapter may publish an initial empty marker snapshot before its graph arrives.
    if (!finishing && [...wanted].every(t => frames[t])) {
      finishing = true;
      setTimeout(() => {
        require('node:fs').writeFileSync(process.argv[3], JSON.stringify(frames));
        clearTimeout(timer); socket.close(); process.exit(0);
      }, 1200).unref();
    }
  }
});
"""


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '91' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires ROS_DOMAIN_ID=91 and ROS_LOCALHOST_ONLY=1; never run on the live domain.')
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    for route, expected in [('/', b'MITI'), ('/rtabmap.html', '实时建图'.encode()),
                            ('/rtabmap-layout.json', b'rtabmap-scene')]:
        with http.open('http://127.0.0.1:13000' + route, timeout=5) as response:
            assert response.status == 200 and expected in response.read(), route
    with http.open('http://127.0.0.1:13000/api/bridge', timeout=5) as response:
        state = json.load(response)
    assert state['state'] == 'running' and state['domainId'] == 91, state
    with http.open('http://127.0.0.1:13000/rtabmap-panel-assets.json', timeout=5) as response:
        assets = json.load(response)['assets']
    bundled = []
    for asset in assets:
        with http.open('http://127.0.0.1:13000' + asset, timeout=5) as response:
            bundled.append(response.read().decode())
    assert any('rtabmapLoopStatusFactory' in source for source in bundled)
    assert any('type:"loop-status",label:' in source for source in bundled)
    assert any('rtabmapHeightClipFactory' in source and 'rtabmapClipDistance' in source for source in bundled)
    assert any('RtabmapCameraHeading.transform(' in source for source in bundled)
    assert any('.map(rtabmapMigrateDepthWidget)' in source for source in bundled)
    assert any('RtabmapMapDetails.Controls' in source and 'RtabmapMapDetails.WallMesh' in source for source in bundled)
    assert any('RtabmapGridMap.Controls' in source and 'enableRotate:!t.rtabmapTopDown' in source for source in bundled)
    assert any('t(rtabmapMigrateGridLayout({' in source for source in bundled)
    frame_fd, frame_file = tempfile.mkstemp(prefix='miti_connection_', suffix='.json')
    os.close(frame_fd)
    client = subprocess.Popen([str(APP / 'node/bin/node'), '-e', CLIENT,
                               str(APP / 'node_modules/next/dist/compiled/ws'),
                               json.dumps(list(TYPES)), frame_file], stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, text=True)
    rclpy.init()
    node = rclpy.create_node('miti_connection_test')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publications = [(topic, kind, node.create_publisher(kind, topic, latched))
                    for topic, kind in [('/cloud_map', PointCloud2), ('/tf_static', TFMessage),
                                        ('/rgbd_image', RGBDImage),
                                        ('/mapGraph', MapGraph), ('/semantic/objects', SemanticMap),
                                        ('/miti/loop_status', String), ('/tf', TFMessage), ('/icp_odom', Odometry),
                                        ('/map', OccupancyGrid)]]
    y, z = np.meshgrid(np.arange(-2, 2.01, .07), np.arange(-.5, 2.51, .07))
    wall = np.column_stack((np.full(y.size, 3.), y.ravel(), z.ravel()))
    wall = wall[~((np.abs(wall[:, 1]) < .55) & (wall[:, 2] < 1.5))]
    map_points = np.vstack(([(0, 0, 0), (1, 2, 3), (2, 3, 4)], wall))
    floor_x, floor_y = np.meshgrid(np.arange(-2, 4.01, .08), np.arange(-2, 2.01, .08))
    floor = np.column_stack((floor_x.ravel(), floor_y.ravel(), np.full(floor_x.size, -.5)))
    map_points = np.vstack((map_points, floor, floor+[0, 0, 3]))
    for topic in ['/camera/color/image_raw', '/camera/depth_registered/image_rect']:
        publications.append((topic, Image, node.create_publisher(Image, topic, qos_profile_sensor_data)))
    try:
        start = time.monotonic()
        while client.poll() is None and time.monotonic() - start < 28:
            header = Header(frame_id='map', stamp=node.get_clock().now().to_msg())
            cloud = create_cloud_xyz32(header, map_points.tolist())
            grid = OccupancyGrid(header=header)
            grid.info.resolution = 0.05
            grid.info.width, grid.info.height = 3, 2
            grid.info.origin.position.x, grid.info.origin.position.y = -1.5, 2.5
            grid.info.origin.orientation.w = 1.0
            grid.data = [-1, 0, 100, 100, 0, -1]
            pose = Pose()
            pose.position.x = 1.0
            graph = MapGraph(header=header, poses_id=[1, 2], poses=[Pose(), pose])
            anchor = Observation(node_id=2, weight=1.0)
            anchor.position.x = anchor.position.y = 1.0
            objects = SemanticMap(header=header, map_key='synthetic-connection-test', map_uuid='test',
                                  objects=[SemanticObject(object_id=1, label='test-chair', observations=[anchor])])
            transform = TransformStamped(header=header, child_frame_id='camera_color_optical_frame')
            transform.transform.rotation.w = 1.
            mount = TransformStamped(header=Header(frame_id='livox_frame'), child_frame_id='base_link')
            mount.transform.translation.x = .2
            mount.transform.rotation.w = 1.
            correction = TransformStamped(header=header, child_frame_id='icp_odom')
            correction.transform.translation.x, correction.transform.translation.y = 10., 20.
            correction.transform.rotation.z = correction.transform.rotation.w = 2**-.5
            source_ns = int((12345.0 + time.monotonic()-start)*1e9)
            odom = Odometry(header=Header(frame_id='icp_odom',
                            stamp=Stamp(sec=source_ns//10**9, nanosec=source_ns%10**9)), child_frame_id='livox_frame')
            odom.pose.pose.position.x = 1.
            odom.pose.pose.orientation.z = odom.pose.pose.orientation.w = 2**-.5
            camera_header = Header(frame_id=transform.child_frame_id, stamp=header.stamp)
            rgb = Image(header=camera_header, width=2, height=2, encoding='rgb8', step=6, data=[100] * 12)
            depth = Image(header=camera_header, width=2, height=2, encoding='16UC1', step=4, data=[232, 3] * 4)
            camera_info = CameraInfo(header=camera_header, width=2, height=2,
                                     k=[1., 0., 1., 0., 1., 1., 0., 0., 1.])
            rgbd = RGBDImage(rgb=rgb, depth=depth, rgb_camera_info=camera_info)
            loop_status = String(data=json.dumps({
                'schema': 1, 'monitor_id': 'isolated-miti-test', 'published_at': time.time(),
                'display_state': 'SUCCESS', 'title': '回环成功', 'phase': 'CONFIRMED',
                'phase_label': '已接受回环约束', 'nodes': 20,
                'counts': {'total': 1, 'global': 1, 'proximity': 0},
                'last_success': {'type': 'GLOBAL_LOOP', 'ref_id': 40, 'target_id': 5, 'time': time.time()},
                'rates_hz': {'lidar': 10.0, 'rgb': 6.0, 'depth': 6.0, 'slam_info': 1.0},
                'slam_age_seconds': 0.0, 'heartbeat_age_seconds': 0.0, 'success_hold_remaining': 8.0,
            }, ensure_ascii=False))
            messages = {'/cloud_map': cloud, '/map': grid, '/mapGraph': graph, '/semantic/objects': objects,
                        '/rgbd_image': rgbd,
                        '/tf_static': TFMessage(transforms=[transform, mount]),
                        '/tf': TFMessage(transforms=[correction]), '/icp_odom': odom,
                        '/camera/color/image_raw': rgb, '/camera/depth_registered/image_rect': depth,
                        '/miti/loop_status': loop_status}
            for topic, _, publisher in publications:
                publisher.publish(messages[topic])
            rclpy.spin_once(node, timeout_sec=0.2)
        output, error = client.communicate(timeout=4)
        assert client.returncode == 0, error
        frames = json.loads(Path(frame_file).read_text())
        decoded = {topic: deserialize_message(base64.b64decode(frames[topic]), kind)
                   for topic, kind in TYPES.items()}
        received_grid = decoded['/map']
        assert received_grid.header.frame_id == 'map'
        assert received_grid.info.width == 3 and received_grid.info.height == 2
        assert abs(received_grid.info.resolution - .05) < 1e-6
        assert received_grid.info.origin.position.x == -1.5 and received_grid.info.origin.position.y == 2.5
        assert list(received_grid.data) == [-1, 0, 100, 100, 0, -1]
        filtered = decoded['/miti/grid_map']
        grid_status = json.loads(decoded['/miti/grid_status'].data)
        assert grid_status['state'] == 'ready', grid_status
        assert grid_status['removed_high_points'] > len(floor)
        assert filtered.header.frame_id == 'map'
        assert abs(filtered.info.resolution-.05) < 1e-6
        values = np.array(filtered.data).reshape(filtered.info.height, filtered.info.width)
        assert (values == 0).sum() > 500 and (values == 100).sum() > 10
        assert (values == 100).sum() < 1000, 'ceiling must not fill the room with obstacles'
        point = list(read_points(decoded['/cloud_map'], field_names=('x', 'y', 'z')))[1]
        assert tuple(float(point[field]) for field in ('x', 'y', 'z')) == (1.0, 2.0, 3.0)
        assert decoded['/camera/color/image_raw'].encoding == 'rgb8'
        assert decoded['/camera/depth_registered/image_rect'].encoding == '16UC1'
        display_depth = decoded['/miti/aligned_depth/image_raw']
        assert (display_depth.width, display_depth.height, display_depth.encoding) == (2, 2, '16UC1')
        assert bytes(display_depth.data) == bytes([232, 3] * 4)
        display_info = decoded['/miti/aligned_depth/camera_info']
        assert (display_info.width, display_info.height) == (2, 2)
        assert display_depth.header == display_info.header
        assert len(decoded['/miti/trajectory'].poses) == 2
        marker = decoded['/miti/semantic_markers'].markers[-1]
        assert marker.text == 'test-chair' and marker.pose.position.x == 2.0
        assert marker.pose.position.y == 1.0
        assert decoded['/tf_static'].transforms[0].child_frame_id == 'camera_color_optical_frame'
        loop_snapshot = json.loads(decoded['/miti/loop_status'].data)
        assert loop_snapshot['counts']['total'] == 1 and loop_snapshot['last_success']['target_id'] == 5
        assert loop_snapshot['display_state'] == 'SUCCESS' and loop_snapshot['title'] == '回环成功'
        robot = decoded['/robot_pose']
        assert robot.header.frame_id == 'map'
        assert 12345 <= robot.header.stamp.sec < 12400, 'device acquisition timestamp was overwritten'
        np.testing.assert_allclose([robot.pose.position.x, robot.pose.position.y, robot.pose.position.z], [9.8, 21., 0.], atol=1e-6)
        state = json.loads(decoded['/miti/map_status'].data)
        assert state['pose']['valid'] and state['pose']['tracked_frame'] == 'base_link', state
        assert abs(abs(state['pose']['rpy_deg'][2])-180) < 1e-6
        surfaces = [m for m in decoded['/miti/walls'].markers if m.ns == 'rtabmap/walls/triangles' and m.action == 0]
        assert surfaces and surfaces[0].points, 'missing wall mesh'
        triangles = np.array([[p.x, p.y, p.z] for p in surfaces[0].points])
        assert np.abs(triangles[:, 0]-3).max() < .03
        centers = triangles.reshape(-1, 3, 3).mean(axis=1)
        assert not ((np.abs(centers[:, 1]) < .3) & (centers[:, 2] < 1.2)).any(), 'doorway filled'
        # Stop all input publishers: node heartbeat must report stale pose, not zeros or a fresh old pose.
        states = []
        subscription = node.create_subscription(String, '/miti/map_status', lambda msg: states.append(json.loads(msg.data)), 10)
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert states and not states[-1]['pose']['valid'], states[-1:]
        publishers = {topic: pub for topic, _, pub in publications}
        # Repeated packets with an unchanged source timestamp must also time out,
        # even while the map correction TF and network traffic remain fresh.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            correction.header.stamp = node.get_clock().now().to_msg()
            publishers['/tf'].publish(TFMessage(transforms=[correction]))
            publishers['/icp_odom'].publish(odom)
            rclpy.spin_once(node, timeout_sec=.1)
        assert not states[-1]['pose']['valid'] and states[-1]['pose']['reason'] == '定位数据已中断', states[-1]
        # Without a base transform expose the true localization reference point.
        odom.child_frame_id = 'unmounted_lidar'
        deadline = time.monotonic()+1.5
        while time.monotonic() < deadline:
            now = node.get_clock().now().to_msg()
            correction.header.stamp = now
            odom.header.stamp = Stamp(sec=12345+now.sec%30, nanosec=now.nanosec)
            publishers['/tf'].publish(TFMessage(transforms=[correction]))
            publishers['/icp_odom'].publish(odom)
            rclpy.spin_once(node, timeout_sec=.1)
        assert states[-1]['pose']['valid'] and states[-1]['pose']['tracked_frame'] == 'unmounted_lidar', states[-1]
        assert not states[-1]['pose']['body_frame']
        node.destroy_subscription(subscription)
        # A single replacement snapshot must remove prior projected obstacles,
        # including for subscribers joining while the map is otherwise static.
        grids = []
        grid_subscription = node.create_subscription(OccupancyGrid, '/miti/grid_map', grids.append, latched)
        publishers['/cloud_map'].publish(create_cloud_xyz32(
            Header(frame_id='map', stamp=node.get_clock().now().to_msg()), floor.tolist()))
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if grids and 100 not in grids[-1].data and list(grids[-1].data).count(0) > 500:
                break
        assert grids and 100 not in grids[-1].data and list(grids[-1].data).count(0) > 500
        publishers['/cloud_map'].publish(create_cloud_xyz32(
            Header(frame_id='map', stamp=node.get_clock().now().to_msg()), []))
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if grids and list(grids[-1].data) == [-1]:
                break
        assert list(grids[-1].data) == [-1], 'empty map must clear the old grid'
        node.destroy_subscription(grid_subscription)
        result = {'passed': True, 'scope': 'HTTP + Foxglove WebSocket CDR + synthetic ROS messages',
                  'miti': '3.3.6', 'bridge': '3.6.0', 'domain': 91, 'topics': list(decoded),
                  'wall_doorway_preserved': True, 'map_body_pose_verified': True, 'pose_timeout_verified': True,
                  'device_clock_verified': True, 'frozen_stamp_verified': True, 'unmounted_reference_verified': True,
                  'grid_height_filter_verified': True, 'grid_snapshot_replacement_verified': True,
                  'grid_empty_map_cleared': True,
                  'browser_rendering_tested': False, 'real_sensors_tested': False}
        report = PROJECT / 'logs/miti/connection_test.json'
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        Path(frame_file).unlink(missing_ok=True)
        if client.poll() is None:
            client.terminate()
            client.wait(timeout=4)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
