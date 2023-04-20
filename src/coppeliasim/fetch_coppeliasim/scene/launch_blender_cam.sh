#!/bin/bash

# Check if the blend file path is provided
if [ -z "$1" ]; then
	  echo "Usage: ./launch_blender_cam.sh path/to/blend/file.blend"
	    exit 1
    fi

/root/Blender/blender -b "$1" --python /catkin_ws/src/coppeliasim/fetch_coppeliasim/scripts/blender_camera_pub.py

