#!/bin/bash
set -e

# setup ROS environment
source /opt/ros/melodic/setup.bash
exec "$@"
