#!/bin/bash
set -e

echo "Installing missing ROS dependencies..."
sudo apt-get update
sudo apt-get install -y ros-noetic-interactive-marker-twist-server \
                        ros-noetic-twist-mux \
                        ros-noetic-joy \
                        ros-noetic-teleop-twist-joy \
                        ros-noetic-rviz-imu-plugin

echo "Dependencies installed successfully."
