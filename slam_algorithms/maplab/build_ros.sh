cd catkin_ws
export ROS_VERSION=melodic
catkin init
catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
catkin config --extend /opt/ros/$ROS_VERSION
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

catkin build yaml_cpp_catkin -j$(($(nproc)-1))
catkin build opencv3_catkin -j$(($(nproc)-1))
catkin build opengv -j$(($(nproc)-1))
catkin build rovioli -j$(($(nproc)-1))
catkin build maplab -j$(($(nproc)-1))