FROM arm64v8/ros:humble

ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DISPLAY=:0

# Add ROS package sources and keys
RUN apt-get update && apt-get install -y curl gnupg lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS and system packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Copy app
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /home/dev/Delta-Robot
COPY . /home/dev/Delta-Robot/

ENTRYPOINT []
CMD ["python3", "GUIMultiplePages.py"]
