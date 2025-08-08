#!/bin/bash

export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGLdispatch.so.0

#Set Jetson as working directory
cd home/delta/Documents/code/Delta-Robot/Jetson
export PYTHONPATH=$PYTHONPATH:/home/delta/Documents/code/Delta-Robot/Jetson

#Bring up ros connection
sudo -u delta dbus-launch nmcli connection up ros-direct

#wait for the connection to be up
until nmcli -t -f GENERAL.STATE connection show ros-direct | grep -q "activated"; do
    echo "Waiting for ros-direct connection ..."
    sleep 1
done 

source /opt/ros/foxy/setup.bash
/usr/bin/python3 /home/delta/Documents/code/Delta-Robot/Jetson/main.py