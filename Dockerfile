FROM ros:noetic-ros-base

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

WORKDIR /ros_ws

# Update Ubuntu Software and Install components
RUN curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash - && \
    apt update && \
    apt upgrade -y && \
    apt install -y \
        nodejs \
        npm && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# Create and initialise ROS workspace
COPY ./webui-ros-joystick ./src/webui-ros-joystick

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y && \
    cd src/webui-ros-joystick/nodejs && \
    npm cache clean --force && \
    rm -rf package-lock.json && \
    npm install rosnodejs@3.0.2 socket.io@2.4.1 yargs@16.2.0 express@4.17.1 && \
    npm install && \
    cd /ros_ws && \
    catkin_make

COPY ./ros_entrypoint.sh /
ENTRYPOINT [ "/ros_entrypoint.sh" ]