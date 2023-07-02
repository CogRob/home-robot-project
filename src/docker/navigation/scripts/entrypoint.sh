#!/bin/bash
set -e
# setup ROS environment
source /opt/ros/melodic/setup.bash
cd /catkin_ws && catkin build fetch_navigation local_path_planner navigation_bringup semantic_localization object_goal_navigation receptacle_navigator room_graph_navigator && source devel/setup.bash 
exec "$@"