# yellowandorange/yoloxtb3:yolo2
FROM yellowandorange/yoloxtb3:base

# Type of OS
ARG TARGETARCH

# ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libedgetpu1-std python3-tflite-runtime python3-pycoral\
 && rm -rf /var/lib/apt/lists/*

 RUN python3 -m pip install --no-cache-dir \
      "networkx==3.1" \
      "fsspec<2024.0.0"
RUN pip3 install --no-cache-dir "ultralytics==8.3.205" --no-deps
# Python
RUN if [ "$TARGETARCH" == "arm64" ]; then\
        pip3 install --no-cache-dir \
            torch==2.2.2 torchvision==0.17.2; \
    else \
        pip3 install --no-cache-dir \
            --extra-index-url https://download.pytorch.org/whl/cpu \
            torch==2.2.2+cpu torchvision==0.17.2+cpu; \
    fi

RUN pip3 install --no-cache-dir opencv-python-headless numpy

# Install ultralytics with no auto-deps


# Launch Files
RUN mkdir -p /root/ws/src/yolo_detection
COPY ./yolo_detection /root/ws/src/yolo_detection

# Obtain model
RUN mkdir -p /root/ws/models
COPY ./models /root/ws/models

# Making the python script runnable
RUN find /root/ws/src/yolo_detection -type f -name "*.py" -exec chmod +x {} \; || true

# Catkin Make
WORKDIR /root/ws
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make" && \
    echo "source /root/ws/devel/setup.bash" >> /root/.bashrc

CMD [ "bash", "-lc", "source /opt/ros/noetic/setup.bash && roslaunch yolo_detection yolo_detection.launch" ]