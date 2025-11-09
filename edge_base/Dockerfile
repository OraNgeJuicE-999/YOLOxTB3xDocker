FROM ros:noetic-ros-base

# Switch shell to bash
SHELL [ "/bin/bash", "-c" ]

# Install shared packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-turtlebot3-bringup \
    ros-noetic-rosserial-python \
    ros-noetic-tf \
    ros-noetic-cv-bridge \
    ros-noetic-std-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-image-transport-plugins \
 && rm -rf /var/lib/apt/lists/*

# Creates Catkin Workspace
ENV CATKIN_WS=/root/ws
RUN mkdir -p $CATKIN_WS/src &&\
    source /opt/ros/noetic/setup.bash && \
    catkin_make -C $CATKIN_WS

WORKDIR $CATKIN_WS