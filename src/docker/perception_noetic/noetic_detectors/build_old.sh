#!/bin/bash
cp -r ../../../perception/yolov5_ros download/
cp -r ../../../perception/detection_msgs download/

cp -r ../../../perception/receptacle_detection download/
cp -r ../../../home_robot_msgs download/

docker build -t ros_perception:noetic . 
rm -rf download/yolov5_ros download/detection_msgs download/receptacle_detection download/home_robot_msgs