#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build fetch_navigation local_path_planner navigation_bringup room_graph_navigator receptacle_navigator semantic_localization && source devel/setup.bash 
exec "$@"