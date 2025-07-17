FROM ros:foxy-ros-base

ENV ROS_VERSION=2
ENV ROS_DISTRO=foxy
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DISPLAY=:0

# Add ROS 2 apt repo (critical!)
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# Now install system and ROS packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    ros-foxy-rclpy \
    ros-foxy-std-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install Python requirements
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Copy your GUI app
WORKDIR /mnt/app
COPY . /mnt/app

ENTRYPOINT []
CMD ["python3", "GUIMultiplePages.py"]
