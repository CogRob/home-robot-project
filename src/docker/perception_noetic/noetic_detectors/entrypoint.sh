#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/noetic/setup.bash
cd /root/detic_ws && catkin build && source devel/setup.bash

exec "$@"