#!/bin/bash
set -e

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-xacro
colcon build