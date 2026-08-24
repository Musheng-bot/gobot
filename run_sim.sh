#!/usr/bin/bash

source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
ros2 launch robot_gazebo sim.launch.py
