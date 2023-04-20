#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build fetch_description robot_controllers fetch_coppeliasim coppeliasim_run fetch_description rail_segmentation && source devel/setup.bash 
exec "$@"