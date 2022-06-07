#!/bin/bash
cd catkin_ws
source devel/setup.bash
roslaunch realsense_d435i_capture realsense_rosbag_record.launch
