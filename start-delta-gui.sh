#!/bin/bash

# OPTIONAL: delay to give X11 some time to start
sleep 3

# Launch the GUI container
docker run -it --rm --net=host \
  -e DISPLAY=:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/Delta-Robot/RaspberryPi:~/Delta-Robot \
  -e ROS_VERSION=2 \
  -e ROS_DISTRO=humble \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_NAMESPACE="" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_IP=192.168.1.10 \
  -e ROS_HOSTNAME=192.168.1.10 \
  delta-robot-app
