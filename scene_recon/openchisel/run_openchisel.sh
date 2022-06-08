cd catkin_ws
source devel/setup.bash
roslaunch chisel_ros launch_realsense_maplab.launch rosbag_path:="$1"
