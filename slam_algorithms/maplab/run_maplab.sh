cd catkin_ws
source devel/setup.bash
roslaunch rovioli realsense.launch
# rosbag record /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /camera/color/camera_info /camera/color/image_raw /tf /tf_static -O /home/olorin/Desktop/UCSD/SP22/CSE237D/bags/poseimage_k.bag
# rosrun rovioli run_realsense realsense_output

#optional step
#rosrun rviz rviz -d $(rospack find rovioli)/rviz/config_file.rviz