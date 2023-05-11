#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/noetic/setup.bash
cd /root/yolovv5_ws && catkin build detection_msgs yolov5_ros && source devel/setup.bash
exec "$@"