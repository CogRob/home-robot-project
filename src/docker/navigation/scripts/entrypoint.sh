#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build fetch_navigation local_path_planner navigation_bringup semantic_localization semantic_navigation object_detector && source devel/setup.bash 
# catkin clean coppeliasim_ros_services coppeliasim_ros_control coppeliasim_msgs_srvs fetch_description robot_controllers fetch_coppeliasim coppeliasim_run fetch_description rail_segmentation
exec "$@"