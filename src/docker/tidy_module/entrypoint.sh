#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build tidy_module && source devel/setup.bash 
exec "$@"