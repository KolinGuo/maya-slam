cd catkin_ws
export ROS_VERSION=melodic
catkin build -j$(($(nproc)-1))

cd ..
chmod +x start_camera.sh
chmod +x start_recording.sh