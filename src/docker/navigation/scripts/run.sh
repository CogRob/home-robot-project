#!/bin/bash
set -ex
xhost +
CONTAINER_NAME="$1"
DOCKER=docker
CONTAINER_NAME=ros_coppeliasim_homerobot_navigation

if [ "$($DOCKER ps -a | grep -w $CONTAINER_NAME)" ]; then
    read -p "container %s already exists, remove it? (Y/n)" choice
    : ${choice:="y"}
    case "$choice" in
        y|Y) 
            $DOCKER stop -t 0 $CONTAINER_NAME > /dev/null
            $DOCKER rm -f $CONTAINER_NAME > /dev/null
            echo "container $CONTAINER_NAME removed."
        ;;
        *) 
            echo "Aborted"
            exit
        ;;
    esac
fi

# sync passwd database for the container
function sync_etc_files {
  PREV_UMASK=`umask`
  umask 002
  mkdir -p $HOME/.dockerlogin
  getent passwd > $HOME/.dockerlogin/passwd
  getent group > $HOME/.dockerlogin/group
  umask $PREV_UMASK
}
PASSWD_VOLUME_OPTION="--volume=$HOME/.dockerlogin:/mnt/dockerlogin:ro"

#sync_etc_files
docker run --name=$CONTAINER_NAME -v $PWD:/catkin_ws/ \
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
	-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -p 19997:19997 -it ros_navigation_homerobot:melodic bash