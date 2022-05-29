cd catkin_ws
source devel/setup.bash
roslaunch rovioli realsense.launch
# rosrun rovioli run_realsense realsense_output

#optional step
#rosrun rviz rviz -d $(rospack find rovioli)/rviz/config_file.rviz