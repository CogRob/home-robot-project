FROM nvidia/cuda:11.6.0-cudnn8-devel-ubuntu20.04

# Arguments to build Docker Image using CUDA
ARG USE_CUDA=0
ARG TORCH_ARCH=

ENV AM_I_DOCKER True
ENV BUILD_WITH_CUDA "${USE_CUDA}"
ENV TORCH_CUDA_ARCH_LIST "${TORCH_ARCH}"
ENV CUDA_HOME /usr/local/cuda-11.6/

RUN mkdir -p /home/appuser/Grounded-Segment-Anything
COPY . /home/appuser/Grounded-Segment-Anything/

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
RUN pip3 install torch==1.13.1+cu116 torchvision==0.14.1+cu116 \
    --extra-index-url https://download.pytorch.org/whl/cu116

WORKDIR /home/appuser/Grounded-Segment-Anything
RUN python3 -m pip install --no-cache-dir -e segment_anything && \
    python3 -m pip install --no-cache-dir -e GroundingDINO
WORKDIR /home/appuser
RUN pip3 install --no-cache-dir diffusers[torch]==0.15.1 opencv-python==4.7.0.72 \
    pycocotools==2.0.6 matplotlib==3.5.3 \
    onnxruntime==1.14.1 onnx==1.13.1 ipykernel==6.16.2 scipy gradio openai
