cd catkin_ws
export ROS_VERSION=melodic
catkin init

catkin build chisel_msgs -j$(($(nproc)-1))
catkin build -j$(($(nproc)-1))