#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive

# ROS1 (Melodic) Install Packages
apt-get update
apt-get install -y zip unzip

apt-get install -y cmake


#   geometry_msgs
#   moveit_msgs
#   roscpp
#   rospy
#   std_msgs
#   vision_msgs
#   actionlib
#   home_robot_msgs

# install image proc for working with AVT cameras
apt-get install -y ros-melodic-image-proc
apt-get install -y ros-melodic-moveit-msgs

# Setup Workspaces
source /opt/ros/melodic/setup.bash

# Final update before being done
apt-get update

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*