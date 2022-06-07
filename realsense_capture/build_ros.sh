#!/bin/bash
cd /maya-slam/realsense_capture/catkin_ws
export ROS_VERSION=melodic
catkin init
catkin config --extend /opt/ros/"$ROS_VERSION"
catkin build -j$(($(nproc)-1))

cd ..
chmod +x start_camera.sh
chmod +x stop_camera.sh
