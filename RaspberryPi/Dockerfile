FROM ros:foxy-ros-base

ENV ROS_VERSION=2
ENV ROS_DISTRO=foxy
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DISPLAY=:0

# Install system + ROS + Python packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    ros-foxy-rclpy \
    ros-foxy-std-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# App setup
WORKDIR /home/dev/Delta-Robot
COPY . /home/dev/Delta-Robot/

ENTRYPOINT []
CMD ["python3", "GUIMultiplePages.py"]
