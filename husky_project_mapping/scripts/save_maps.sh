#!/bin/bash

mkdir -p ~/ros_ws/maps

echo "----------------------------------------"
echo "1. Saving Resolution 0.1m (High Res)..."

rosrun octomap_server octomap_saver ~/ros_ws/maps/octomap_0_1.bt octomap_binary:=/octomap_0_1

echo "----------------------------------------"
echo "2. Saving Resolution 0.2m (Medium Res)..."
rosrun octomap_server octomap_saver ~/ros_ws/maps/octomap_0_2.bt octomap_binary:=/octomap_0_2

echo "----------------------------------------"
echo "3. Saving Resolution 0.4m (Low Res)..."
rosrun octomap_server octomap_saver ~/ros_ws/maps/octomap_0_4.bt octomap_binary:=/octomap_0_4

echo "----------------------------------------"
echo "KONTROL (Dosya boyutlari farkli olmali):"
ls -lh ~/ros_ws/maps/octomap_*.bt