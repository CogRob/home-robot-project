#!/bin/bash
set -e
# setup ROS environment

source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build perception_bringup && source devel/setup.bash 
# need to run roslaunch perception_bringup perception_launch.launch
exec "$@"