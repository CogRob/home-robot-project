#!/usr/bin/env bash
set -e
export DEBIAN_FRONTEND=noninteractive

# ROS1 (Melodic) Install Packages
apt-get update
apt-get install -y zip unzip

apt-get install -y cmake


apt-get install -y ros-melodic-image-proc
apt-get install -y ros-melodic-moveit-msgs

# Setup Workspaces
source /opt/ros/melodic/setup.bash

# Final update before being done
apt-get update

unset DEBIAN_FRONTEND
rm -rf /var/lib/apt/lists/*