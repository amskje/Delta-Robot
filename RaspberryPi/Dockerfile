FROM arm64v8/ros:humble

ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_LOCALHOST_ONLY=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV DISPLAY=:0

#Install pip and system packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*


#Install Python libaries
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

#Set working directory inside the container
WORKDIR /home/dev/Delta-Robot
COPY . /home/dev/Delta-Robot/

# Maybe fix
ENTRYPOINT []

#Default command
#CMD ["bash"]

#This is how to run the python script direclty
CMD ["python3", "GUIMultiplePages.py"]
