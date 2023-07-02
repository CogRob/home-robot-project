FROM ros-melodic

ENV ROS_DISTRO melodic
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /root
# install ompl
# first, need to instal ODE
RUN apt-get update
RUN apt-get install -y ros-melodic-moveit ros-melodic-rail-manipulation-msgs
# RUN git clone https://github.com/thomasmarsh/ODE.git
# RUN cd ODE && mkdir -p build && cd build && cmake .. && make -j$(nproc) && make install

# # second, need to install Py++
# COPY ./download/castxml.tar.xz $HOME
# RUN tar -xf $HOME/castxml.tar.xz -C $HOME && rm $HOME/castxml.tar.xz
# ENV PATH=/root/castxml/bin:${PATH}

# RUN sudo -H pip3 install -vU pygccxml pyplusplus numpy

# # then install ompl
# RUN git clone https://github.com/ompl/ompl.git && cd ompl && mkdir -p build/Release && cd build/Release && \
#     cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic ../.. -DPYTHON_EXEC=/usr/bin/python3  -DCASTXML=/root/castxml/bin/castxml && make update_bindings && make -j$(nproc) && make install

# # install moveit
# # setup the rosdep
# RUN sudo apt -y install python-wstool python-catkin-tools clang-format-10 python-rosdep
# RUN mkdir -p ws_moveit/src && cd ws_moveit/src && git clone https://github.com/jih189/moveit_cbirrt.git
# RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
# RUN rosdep init && rosdep update
# RUN cd ws_moveit && rosdep install -y --from-paths src --ignore-src --rosdistro melodic

# # remove the ompl from ROS
# RUN apt-get -y remove $(apt list --installed | awk -F/ '{print $1}' | grep ompl)

# # reinstall the ompl in its build directory
# RUN cd ompl/build/Release && make -j$(nproc) install

# # compile moveit
# RUN cd ws_moveit && catkin config --extend /opt/ros/melodic --install --install-space /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build

# # install jupyter
# RUN python3 -m pip install --upgrade pip
# RUN pip3 install jupyter

# # install python2 kernel into jupyter notebook
# RUN apt install python-pip -y
# RUN python2 -m pip install ipykernel
# RUN python2 -m ipykernel install

# # Expose the port Jupyter will run on
# EXPOSE 8888

# # Install necessary for building
# RUN apt-get install -y ros-melodic-costmap-2d ros-melodic-rail-manipulation-msgs

# # Install fcl library
# RUN git clone https://github.com/flexible-collision-library/fcl.git && cd fcl && mkdir build && cd build && cmake .. && make -j$(nproc) && make install

# install trimesh
RUN pip2 install trimesh
RUN apt-get install -y python-scipy python-rtree

COPY ./entrypoint.sh /entrypoint.sh

RUN chmod 755 /entrypoint.sh && \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh" ]
WORKDIR /catkin_ws