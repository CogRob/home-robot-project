FROM ros-melodic
# RUN echo 'export ROS_MASTER_URI=http://localhost:11311' >> $HOME/.bashrc
# RUN echo 'export ROS_HOSTNAME=localhost' >> $HOME/.bashrc

# # install the coppeliasim
RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        vim tar xz-utils \
        libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
        xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
        libavcodec-dev libavformat-dev libswscale-dev \
	libsodium23 libopenexr22 \
	libraw1394-11 libraw1394-dev libraw1394-tools \
	libusb-1.0-0 libusb-1.0-0-dev \
        && \
    apt-get autoclean -y && apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /shared /opt

COPY ./download/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz /root/
RUN tar -xf /root/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz -C /root && \
    rm /root/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04.tar.xz && cd /root && mv CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04 CoppeliaSim

ENV COPPELIASIM_ROOT_DIR=/root/CoppeliaSim

EXPOSE 19997

RUN apt-get update
RUN apt-get install -y ros-melodic-controller-manager ros-melodic-transmission-interface ros-melodic-control-toolbox ros-melodic-joint-limits-interface ros-melodic-cv-bridge ros-melodic-joint-trajectory-controller ros-melodic-diff-drive-controller
RUN apt-get install -y ros-melodic-depth-image-proc ros-melodic-rail-manipulation-msgs ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-image-proc

# setup the python client for remote control in Coppeliasim
RUN python3 -m pip install pyzmq
RUN python3 -m pip install cbor
RUN git clone --recursive https://github.com/CoppeliaRobotics/zmqRemoteApi && cd zmqRemoteApi && git fetch && git checkout tags/coppeliasim-v4.4.0-rev0 && \
    mkdir -p build && cd build && cmake -DMAKE_BUILD_TYPE=Release .. && cmake --build . && cmake --install .
RUN apt-get update

RUN mkdir -p /tmp/catkin_ws/src && cd /tmp/catkin_ws/src/ && git clone --recursive https://github.com/tud-cor/coppeliasim_ros_control.git && git clone --recursive https://github.com/tud-cor/coppeliasim_ros_services.git && git clone --recursive https://github.com/tud-cor/coppeliasim_msgs_srvs.git

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /tmp/catkin_ws; catkin clean; catkin build'

RUN cp /tmp/catkin_ws/devel/lib/libsimExtRosControl.so $COPPELIASIM_ROOT_DIR
RUN cp /tmp/catkin_ws/devel/lib/libsimExtRosServices.so $COPPELIASIM_ROOT_DIR

# install blender
RUN apt-get update && apt-get install -y libc6 libc6-dev

COPY ./download/blender-3.4.0-linux-x64.tar.xz /root/

RUN tar -xf /root/blender-3.4.0-linux-x64.tar.xz -C /root && \
    rm /root/blender-3.4.0-linux-x64.tar.xz && cd /root && mv blender-3.4.0-linux-x64 Blender

# Download the Python Source since it is not bundled with Blender
RUN wget https://www.python.org/ftp/python/3.10.8/Python-3.10.8.tgz \
	&& tar -xzf Python-3.10.8.tgz \
	&& cp -r Python-3.10.8/Include/* /root/Blender/3.4/python/include/python3.10/ \
	&& rm -rf Python-3.10.8.tgz \
	&& rm -rf Python-3.10.8
	
# install ros related packages in blender
RUN /root/Blender/3.4/python/bin/python3.10 -m ensurepip && /root/Blender/3.4/python/bin/python3.10 -m pip install pyyaml bpycv rospkg opencv-python netifaces

# set alias for blender
RUN echo "alias blender='$HOME/Blender/blender'" >> $HOME/.bashrc


# install python3 package required for ROS
RUN apt-get install -y python3-pip python3-yaml python3-netifaces && pip3 install rospkg catkin_pkg

# # recompile tf2_ros for python3
RUN apt-get install -y python3-catkin-pkg-modules python3-rospkg-modules python3-empy

RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && mkdir -p $HOME/ros_python3_ws/src && cd $HOME/ros_python3_ws && wstool init && wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel && wstool up && rosdep install --from-paths src --ignore-src -y -r'
RUN sed -i '/from __future__ import absolute_import/aimport sys\nsys.path[:] = [x for x in sys.path if "python3" not in x] if sys.version_info[0] < 3 else sys.path' /root/ros_python3_ws/src/geometry2/tf2_ros/src/tf2_ros/__init__.py
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && cd $HOME/ros_python3_ws && \
	catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so'



COPY ./entrypoint.sh /entrypoint.sh

RUN chmod 755 /entrypoint.sh && \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh" ]
WORKDIR /catkin_ws
