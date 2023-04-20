#!/bin/bash
set -ex
xhost +
docker run --name="ros_fetch_sim" -v $PWD/../../../../:/catkin_ws/ \
	-e DISPLAY=$DISPLAY \
	-e QT_X11_NO_MITSHM=1 \
	-e XAUTHORITY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	--ipc=host \
	--gpus all \
	--network="host" \
	-p 8888:8888 \
	--privileged=true \
	-v /etc/localtime:/etc/localtime:ro \
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -p 19997:19997 -it fetch_sim:melodic bash
