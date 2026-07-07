#!/bin/bash
# ROS 2 Humble + Gazebo installation for Ubuntu 22.04
set -e

echo "=== Adding ROS 2 repository ==="
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== Updating package lists ==="
sudo apt update

echo "=== Installing ROS 2 Humble Desktop (this will take a while) ==="
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

echo "=== Installing Gazebo ROS packages ==="
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

echo "=== Done! Source ROS with: source /opt/ros/humble/setup.bash ==="
