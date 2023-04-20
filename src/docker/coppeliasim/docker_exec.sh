#!/usr/bin/env bash

: ${DOCKER:=docker}

# Determine the conatiner shell
: ${CONTAINER_SHELL:="/bin/bash"}

# Load container name
: ${CONTAINER_NAME:=$1}
if [ ! "$($DOCKER ps -a | grep -w $CONTAINER_NAME)" ]
then
    echo "container $CONTAINER_NAME doesn't exit! Please create it at first!"
    exit 1
fi
DOCKERLOGIN_IMAGE=$(docker inspect --format='{{.Config.Image}}' $CONTAINER_NAME)

# allow qt applications in the container to use MIT-SHM extension of the X server
DOCKERLOGIN_X11_RUN_OPTIONS='--env=DISPLAY --env=QT_X11_NO_MITSHM=1' 

# Determine if we want sudo privileges
user=""
line5=""
line5_sp=""
if [ "$2" == "super" ]; then
        line5_sp="$(seq 21 | awk '{printf " "}')="
        line5="= Entering with super user privileges!"
        user="-u 0"
fi

exec $DOCKER exec \
    --privileged \
    --interactive \
    --tty \
    $user \
    --env SHELL=$CONTAINER_SHELL \
    --env TERM=$TERM \
    --env USER=$USER \
    --env IN_DOCKERLOGIN=$CONTAINER_NAME \
    ${DOCKERLOGIN_X11_RUN_OPTIONS} \
    ${CONTAINER_NAME} \
    ${CONTAINER_SHELL} -l