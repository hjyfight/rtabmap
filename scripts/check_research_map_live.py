#!/usr/bin/env python3
"""Read-only capture of map display inputs and their diagnostics."""
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from map_geometry_node import cloud_xyz
from grid_projection import fit_ground,GridConfig

ROOT=Path(__file__).resolve().parents[1]


def main():
    rclpy.init();node=rclpy.create_node('research_map_readonly_check')
    statuses={};clouds={};pose=[]
    latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for key,topic in [('raw','/cloud_map'),('clean','/miti/research/display/clean_map')]:
        node.create_subscription(PointCloud2,topic,lambda message,key=key:clouds.update({key:cloud_xyz(message)}),latched)
    node.create_subscription(PoseStamped,'/miti/research/display/robot_pose',lambda msg:pose.append(
        [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]),qos_profile_sensor_data)
    for key,topic in [('grid','/miti/research/display/grid_status'),('geometry','/miti/research/display/map_status'),
                      ('dynamic','/miti/research/display/dynamic_cloud_status'),('loops','/miti/research/display/loop_status')]:
        node.create_subscription(String,topic,lambda message,key=key:statuses.update({key:json.loads(message.data)}),qos_profile_sensor_data)
    until=time.monotonic()+5
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
    report=dict(statuses=statuses,pose=pose[-1] if pose else None,clouds={})
    arrays={}
    for key,points in clouds.items():
        finite=points[np.isfinite(points).all(1)]
        reference=np.asarray(pose[-1]) if pose else np.zeros(3)
        plane=fit_ground(finite,reference,GridConfig()) if len(finite)>=100 else None
        report['clouds'][key]=dict(points=len(finite),z_quantiles=np.quantile(finite[:,2],[0,.05,.1,.2,.4,.8,1]).tolist(),
                                  ground_fit=plane.tolist() if plane is not None else None)
        arrays[key+'_xyz']=finite
    directory=ROOT/'logs/research/live';directory.mkdir(exist_ok=True)
    np.savez_compressed(directory/'map_inputs.npz',**arrays,reference=np.asarray(pose[-1] if pose else [0,0,0]))
    (directory/'map_status.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
