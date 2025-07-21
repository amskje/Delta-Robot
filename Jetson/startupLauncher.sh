#!/bin/bash

export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libGLdispatch.so.0

source /opt/ros/foxy/setup.bash
/usr/bin/python3 /home/delta/Documents/code/Delta-Robot/Jetson/main.py
