#!/usr/bin/bash

source /opt/ros/humble/setup.sh
source install/setup.bash
ros2 launch robot_gazebo sim.launch.py
