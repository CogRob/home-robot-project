#!/bin/bash
set -e
# setup ROS environment
cd /catkin_ws/src/behavior_trees/py_trees/ && python setup.py install

source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build py_trees_ros service_robot_pipeline && source devel/setup.bash 
exec "$@"