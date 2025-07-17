#!/bin/bash

sleep 3

# Log output to file
docker run --rm --net=host \
  -e DISPLAY=:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/delta/Desktop/Delta-Robot/RaspberryPi:/mnt/app \
  -e ROS_VERSION=2 \
  -e ROS_DISTRO=foxy \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_NAMESPACE="" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_IP=192.168.1.10 \
  -e ROS_HOSTNAME=192.168.1.10 \
  delta-robot-app > /home/delta/docker_gui.log 2>&1
