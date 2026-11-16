#!/usr/bin/env bash
# Start only when the normal SLAM-owned depth/RGB-D pipeline is absent.
set -eo pipefail
DEPTH_PREVIEW_PROJECT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$DEPTH_PREVIEW_PROJECT"
source scripts/ros2_wired_env.sh
python3 - <<'PY'
import time
import rclpy
rclpy.init()
node = rclpy.create_node('depth_preview_start_check')
try:
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    for topic in ('/camera/depth_registered/image_rect', '/rgbd_image'):
        if node.get_publishers_info_by_topic(topic):
            raise SystemExit('已有深度/RGB-D 发布端，请复用当前链路：' + topic)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
exec ros2 launch "$DEPTH_PREVIEW_PROJECT/launch/realsense_rgbd_preview.launch.py"
