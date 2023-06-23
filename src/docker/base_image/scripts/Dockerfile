FROM ubuntu:18.04

# Install common 
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    ca-certificates \
    cmake \
    vim \
    git \
    bzip2 \
    libx11-6 \
    gnupg \
    lsb-release \
    gnupg2 \
    build-essential \
    tmux \
 && rm -rf \var/lib/apt/lists/*

# ROS Melodic Installation
RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US

# set to noninteractive to avoid prompts during image creation
RUN DEBIAN_FRONTEND=noninteractive TZ=America/US apt-get -y install tzdata

# ROS sources and key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install
RUN apt-get update 
RUN apt install -y ros-melodic-desktop
RUN apt install -y ros-melodic-ros-base
RUN apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
RUN apt-get install -y autotools-dev automake libtool python3-pip
# Add to our bashrc for convenience 
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo '# export ROS_MASTER_URI="http://localhost:11311"' >> ~/.bashrc
RUN echo '# export ROS_IP=127.0.0.1' >> ~/.bashrc
RUN echo '# export ROS_HOSTNAME=127.0.0.1' >> ~/.bashrc
RUN echo 'export ROS_MASTER_URI="http://192.168.1.2:11311"' >> ~/.bashrc
RUN echo 'export ROS_IP=192.168.1.12' >> ~/.bashrc
RUN echo 'export ROS_HOSTNAME=192.168.1.12' >> ~/.bashrc
# Install & Initialize rosdep for ease of installation of workspace dependencies
RUN apt-get update

# install cmake
RUN apt-get update && \
    apt-get install -y build-essential wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.22.0/cmake-3.22.0.tar.gz && \
    tar -xf cmake-3.22.0.tar.gz && \
    rm cmake-3.22.0.tar.gz && \
    cd cmake-3.22.0 && \
    ./bootstrap && \
    make -j$(nproc) && \
    make install

RUN rosdep init
RUN rosdep update
RUN apt-get update