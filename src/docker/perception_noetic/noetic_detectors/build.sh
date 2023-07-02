#!/bin/bash
cp -r ../../../perception/detic_ros download/
cp -r ../../../perception/vision_msgs download/
cp -r ../../../home_robot_msgs download/

docker build -t ros_perception:noetic . 
rm -rf download/detic_ros download/vision_msgs download/home_robot_msgs