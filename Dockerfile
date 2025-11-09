FROM yellowandorange/yoloxtb3:base3

SHELL ["/bin/bash", "-c"]
# ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-noetic-nav-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-dynamic-reconfigure \
 && rm -rf /var/lib/apt/lists/*

# Python
RUN pip3 install --no-cache-dir \
    numpy opencv-python-headless

# Launch Files
WORKDIR /root/ws/src
RUN mkdir -p tb3_motion
COPY ./tb3_motion /root/ws/src/tb3_motion

RUN find /root/ws/src/tb3_motion -type f -name "*.py" -exec chmod +x {} \; || true

# Catkin Make
WORKDIR /root/ws
RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/devel/setup.bash" >> /root/.bashrc


CMD ["bash", "-lc", "source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && roslaunch tb3_motion motion.launch"]
