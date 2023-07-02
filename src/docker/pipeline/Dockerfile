FROM ros-melodic

ENV ROS_DISTRO melodic
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Copy over specific version of pacmod msgs (for pacmod2)
#COPY astuff_sensor_msgs-2.3.1.zip /opt/astuff_sensor_msgs-2.3.1.zip

# Install base ROS drivers 
# RUN apt-get install -y ros-melodic-costmap-2d ros-melodic-rail-manipulation-msgs

RUN apt-get update
RUN apt-get install -y ros-melodic-uuid-msgs ros-melodic-unique-id

RUN pip install termcolor

COPY ./entrypoint.sh /entrypoint.sh

RUN chmod 755 /entrypoint.sh && \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh" ]
WORKDIR /catkin_ws
