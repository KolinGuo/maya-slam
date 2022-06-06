# Tracking using Maplab


Container running the SLAM algorithm. In this work, [maplab](https://github.com/ethz-asl/maplab) is used, which is a Stereo-IMU based SLAM. 

### Building
1. `./docker_setup.sh maplab -l` to build the container.
2. `cd slam_algorithms/maplab && ./build_ros.sh` to build the packages.

### Running
Run `./slam_algorithms/maplab/run_maplab.sh <bagname>` to start the camera tracking. It begins tracking the camera. Displays the trajectory in RVIZ. Also saves the output of depth, color images, IMU and trajectort pose to a rosbag. Pass rosbag name for shell script

To configure options, go to modify the contents, go to `slam_algorithms/maplab/catkin_ws/src/maplab/applications/rovioli/launch`. Edit `realsense.launch` to change the launch file. Edit the `realsense .yaml` files inside `share` to change calibration parameters.
