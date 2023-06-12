FROM ubuntu:20.04

# Install common software
RUN apt-get update
RUN apt-get install -y \
    curl \
    wget \
    ca-certificates \
    vim \
    sudo \
    git \
    bzip2 \
    libx11-6 \
    gnupg \
    gnupg2 \
    lsb-release \
    python3 \
    python3-pip \
    tmux \
 && rm -rf \var/lib/apt/lists/*

# ROS Noetic Installation
RUN sudo apt update && sudo apt install -y locales
RUN sudo locale-gen en_US en_US.UTF-8
RUN sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US

RUN DEBIAN_FRONTEND=noninteractive TZ=America/US apt-get -y install tzdata
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN sudo apt-get update
RUN DEBIAN_FRONTEND=noninteractive sudo apt install -y ros-noetic-desktop
RUN sudo apt install -y ros-noetic-ros-base
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Install Rosdep
RUN sudo apt install -y python3-rosdep
RUN sudo apt-get update

RUN rosdep init
RUN rosdep update

# Final update
RUN sudo apt-get update

RUN apt-get install -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
 && rm -rf \var/lib/apt/lists/*


RUN pip3 install --upgrade pip
COPY download/requirements.txt /tmp/requirements.txt
RUN pip3 install -r /tmp/requirements.txt

RUN mkdir -p /root/yolovv5_ws/src
COPY download/yolov5_ros /root/yolovv5_ws/src/yolov5_ros
COPY download/detection_msgs /root/yolovv5_ws/src/detection_msgs
COPY download/best.pt /root/yolovv5_ws/src/yolov5_ros/src/yolov5/

COPY ./entrypoint.sh /entrypoint.sh

RUN chmod 755 /entrypoint.sh && \
  rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh" ]
WORKDIR /root