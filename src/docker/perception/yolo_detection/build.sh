#!/bin/bash
cp -r ../../../perception/yolov5_ros download/
cp -r ../../../perception/detection_msgs download/
docker build -t ros_yolo_homerobot:noetic . 
rm -rf download/yolov5_ros download/detection_msgs