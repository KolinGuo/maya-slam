#!/bin/bash
cd /maya-slam/scene_recon/open3d_tsdf
source /opt/ros/noetic/setup.bash
python3 open3d_tsdf_reconstruction.py /maya-slam/slam_algorithms/rosbags/"$1" --voxel-length "$2" "$3"
