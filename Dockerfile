FROM ros:noetic-ros-core

ENV ROS_WS ros_ws

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt upgrade -y

RUN apt install -y curl \
    git

RUN curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -

RUN apt install -y \
    python3-dev \
    python3-pip \
    python3-rospkg \
    nodejs

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg

# Create and initialise ROS workspace
RUN mkdir -p /$ROS_WS/src
COPY ./webui-ros-joystick /$ROS_WS/src/webui-ros-joystick
WORKDIR /$ROS_WS

RUN cd src/webui-ros-joystick/nodejs && \
    npm install rosnodejs@3.0.2 socket.io@2.4.1 yargs@16.2.0 express@4.17.1 && \
    npm install

RUN mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    catkin_make

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /