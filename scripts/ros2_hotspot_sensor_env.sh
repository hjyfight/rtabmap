#!/usr/bin/env bash
# Installed in the onboard user's home together with the matching XML.
source "$HOME/ros2_wifi_env.sh"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="file://$HOME/hotspot_sensor_cyclonedds.xml"
