#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build coppeliasim_ros_control coppeliasim_ros_services coppeliasim_msgs_srvs && cp devel/lib/libsimExtRosControl.so $COPPELIASIM_ROOT_DIR && cp devel/lib/libsimExtRosServices.so $COPPELIASIM_ROOT_DIR
cd /catkin_ws && catkin build fetch_description robot_controllers fetch_coppeliasim coppeliasim_run fetch_description rail_segmentation coppeliasim_zmq && source devel/setup.bash 
exec "$@"