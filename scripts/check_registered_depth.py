#!/usr/bin/env python3
"""Choose whether semantic startup needs a new registered-depth publisher."""
import argparse
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic',default='/camera/depth_registered/image_rect')
    args=parser.parse_args()
    import rclpy
    rclpy.init();node=rclpy.create_node('semantic_depth_publisher_check')
    try:
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        publishers=node.get_publishers_info_by_topic(args.topic)
        if len(publishers)>1:
            raise RuntimeError('配准深度发布端超过一个，请结束重复配准实例：'+args.topic)
        print('reuse' if publishers else 'start')
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
