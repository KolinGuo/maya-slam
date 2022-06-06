cd /maya-slam/scene_recon/open3d_tsdf
python3 open3d_tsdf_reconstruction.py "/maya-slam/slam_algorithms/rosbags/d435i_$2_2022-06-04-21-28-59.bag --voxel-length $1 --pose-refine --save-pose-traj"