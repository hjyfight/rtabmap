#!/usr/bin/env python3
"""3001 scene adapter, restoring the graph when opened after SLAM started."""
from copy import deepcopy
import time

import rclpy
from rtabmap_msgs.srv import GetMap
from miti_scene_adapter import MitiSceneAdapter


class ResearchSceneAdapter(MitiSceneAdapter):
    def __init__(self):
        super().__init__()
        self.bootstrap_future = None
        self.bootstrap_started = -float('inf')
        self.bootstrap_epoch = 0
        self.create_timer(.5, self.bootstrap_graph)

    def bootstrap_graph(self):
        if self.bootstrap_future is not None:
            future = self.bootstrap_future
            if future.done():
                self.bootstrap_future = None
                try:
                    data = future.result().data
                    if self.bootstrap_epoch == self.cache_epoch and self.graph is None:
                        graph = deepcopy(data.graph)
                        graph.header = deepcopy(data.header)
                        self.on_graph(graph)
                except Exception as error:
                    self.get_logger().warning('Cannot read initial research graph: '+str(error),
                                              throttle_duration_sec=5.)
            elif time.monotonic()-self.bootstrap_started > 5:
                self.map_client.remove_pending_request(future)
                future.cancel()
                self.bootstrap_future = None
            else:
                return
        if self.graph is not None or time.monotonic()-self.bootstrap_started < 5 or not self.map_client.service_is_ready():
            return
        self.bootstrap_epoch = self.cache_epoch
        self.bootstrap_started = time.monotonic()
        self.bootstrap_future = self.map_client.call_async(
            GetMap.Request(global_map=True, optimized=True, graph_only=True))


def main():
    rclpy.init()
    node = ResearchSceneAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
