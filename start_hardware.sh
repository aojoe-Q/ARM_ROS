#!/bin/bash
cd /home/yaq/ARM_ROS
source install/setup.bash
ros2 launch dm_mc02_hardware bringup.launch.xml allow_enable:=0 device:=/dev/ttyACM1
