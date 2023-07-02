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
RUN apt-get update && apt-get install -y ros-melodic-open-karto ros-melodic-costmap-2d ros-melodic-move-base-msgs ros-melodic-sparse-bundle-adjustment ros-melodic-map-server ros-melodic-move-base ros-melodic-amcl ros-melodic-carrot-planner ros-melodic-teleop-twist-keyboard
RUN apt install -y coinor-libipopt-dev
RUN apt-get install -y ros-melodic-costmap-converter ros-melodic-mbf-msgs ros-melodic-mbf-costmap-core ros-melodic-teb-local-planner ros-melodic-tf2-eigen
RUN git clone https://github.com/rst-tu-dortmund/control_box_rst.git && cd control_box_rst && mkdir build && cd build && cmake .. && make -j$(nproc) && make install
RUN apt-get install -y ros-melodic-pointcloud-to-laserscan ros-melodic-ira-laser-tools
RUN apt-get install -y ros-melodic-mpc-local-planner ros-melodic-ros-numpy
# RUN cd /root && mkdir -p mpc_planner_ws/src && cd mpc_planner_ws/src && git clone -b melodic-devel https://github.com/rst-tu-dortmund/mpc_local_planner.git
# RUN ls /root && ls /root/mpc_planner_ws
# RUN rosdep update
# RUN cd /root/mpc_planner_ws catkin config --extend /opt/ros/melodic && catkin build

# Entrypoint
COPY ./entrypoint.sh /entrypoint.sh

RUN chmod 755 /entrypoint.sh && \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh" ]
WORKDIR /catkin_ws