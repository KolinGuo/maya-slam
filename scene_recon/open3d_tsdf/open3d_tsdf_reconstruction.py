#!/usr/bin/env python3
"""Open3D TSDF integration from rosbag"""
from pathlib import Path
from copy import deepcopy
from typing import Tuple, List, Dict

import numpy as np
import open3d as o3d
from transforms3d.quaternions import quat2mat, mat2quat
from tqdm import tqdm

import rosbag
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform

def transform_msg2mat(transform: Transform) -> np.ndarray:
    """Convert a geometry_msgs/Transform to a 4x4 transformation matrix
    :param transform: a geometry_msgs.msg.Transform.
    """
    p = transform.translation
    q = transform.rotation

    T = np.eye(4)
    T[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])  # [w, x, y, z]
    T[:3, 3] = [p.x, p.y, p.z]
    return T

def read_frame_data_from_bag(bag: rosbag.Bag) -> Dict[str, list]:
    """Read frame data from a rosbag"""
    frame_data = {
        'rgb_imgs': [],  # [H, W, 3] uint8 np.ndarray
        'rgb_img_timestamps': [],
        'rgb_intrinsics': [],  # [3, 3] float64 np.ndarray
        'rgb_intr_timestamps': [],
        'depth_imgs': [],  # [H, W] uint16 np.ndarray
        'depth_img_timestamps': [],
        'depth_intrinsics': [],  # [3, 3] float64 np.ndarray
        'depth_intr_timestamps': [],
        'poses': [],  # T_world_cam, [4, 4] float64 np.ndarray
        'pose_timestamps': [],
        'timestamps': [],  # unique timestamps with imgs and poses
    }

    cvbridge = CvBridge()
    topic_msgs = bag.read_messages(topics=['/tf',
                                           '/camera/color/image_raw',
                                           '/camera/color/camera_info',
                                           '/camera/aligned_depth_to_color/image_raw',
                                           '/camera/aligned_depth_to_color/camera_info'])
    for i, (topic, msg, t) in enumerate(topic_msgs):
        # Extract msg timestamp
        if hasattr(msg, 'header'):
            msg_timestamp = round(msg.header.stamp.to_sec(), 6)
        else:  # topic == '/tf'
            msg_timestamp = round(msg.transforms[0].header.stamp.to_sec(), 6)

        if topic == '/tf' and msg.transforms[0].header.frame_id == "map" \
                and msg.transforms[0].child_frame_id == 'imu':
            frame_data['poses'].append(transform_msg2mat(msg.transforms[0].transform))
            frame_data['pose_timestamps'].append(msg_timestamp)
        elif topic == '/camera/color/image_raw':
            frame_data['rgb_imgs'].append(cvbridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
            frame_data['rgb_img_timestamps'].append(msg_timestamp)
        elif topic == '/camera/color/camera_info':
            frame_data['rgb_intrinsics'].append(np.asanyarray(msg.P).reshape(3, 4)[:3, :3])
            frame_data['rgb_intr_timestamps'].append(msg_timestamp)
        elif topic == '/camera/aligned_depth_to_color/image_raw':
            frame_data['depth_imgs'].append(cvbridge.imgmsg_to_cv2(msg, desired_encoding='16UC1'))
            frame_data['depth_img_timestamps'].append(msg_timestamp)
        elif topic == '/camera/aligned_depth_to_color/camera_info':
            frame_data['depth_intrinsics'].append(np.asanyarray(msg.P).reshape(3, 4)[:3, :3])
            frame_data['depth_intr_timestamps'].append(msg_timestamp)

    # Find timestamps associations
    from functools import reduce
    timestamps = reduce(np.intersect1d, [v for k, v in frame_data.items() if '_timestamps' in k]).tolist()
    frame_data['timestamps'] = timestamps
    print(f"Found {len(timestamps)} timestamps with images and poses")

    return frame_data

def get_frame_data_at_timestamp(frame_data: Dict[str, list], timestamp: float):
    """Get frame data at timestamp"""
    rgb_im = frame_data['rgb_imgs'][frame_data['rgb_img_timestamps'].index(timestamp)]
    rgb_K = frame_data['rgb_intrinsics'][frame_data['rgb_intr_timestamps'].index(timestamp)]

    depth_im = frame_data['depth_imgs'][frame_data['depth_img_timestamps'].index(timestamp)]
    depth_K = frame_data['depth_intrinsics'][frame_data['depth_intr_timestamps'].index(timestamp)]

    np.testing.assert_allclose(rgb_K, depth_K,
                               err_msg='Different intrinsic for rgb and depth, possibly misaligned')
    K = rgb_K

    cam_pose = frame_data['poses'][frame_data['pose_timestamps'].index(timestamp)]

    return rgb_im, depth_im, K, cam_pose

def _save_pose_traj(save_pose_traj_dir: Path, frame_data: Dict[str, list],
                    cam_poses_stamped, odometry_results_stamped):
    """Save pose trajectory and its visualization"""
    # Saving refined cam_pose
    cam_poses = {
        'timestamps': np.array([timestamp for timestamp, cam_pose in cam_poses_stamped]),
        'cam_poses': np.array([cam_pose for timestamp, cam_pose in cam_poses_stamped]),
    }
    np.savez(save_pose_traj_dir / 'refined_pose_traj.npz', **cam_poses)

    # Saving odometry_results
    odo_results = {
        'timestamps': np.array([timestamp for timestamp, odo_res in odometry_results_stamped]),
        'fitness': np.array([odo_res.fitness for timestamp, odo_res in odometry_results_stamped]),
        'inlier_rmse': np.array([odo_res.inlier_rmse for timestamp, odo_res in odometry_results_stamped]),
        'transformation': np.array([odo_res.transformation.numpy() for timestamp, odo_res in odometry_results_stamped]),
    }
    np.savez(save_pose_traj_dir / 'consecutive_frame_odometry_results.npz', **odo_results)

    # Visualize pose trajectory
    import plotly.express as px

    df = {'method': [], 'pos_x': [], 'pos_y': [], 'pos_z': [], 'timestamp': []}
    ### o3d rgbd_odometry_multi_scale ###
    for timestamp, cam_pose in cam_poses_stamped:
        df['method'].append('Open3D pose refine')
        df['timestamp'].append(timestamp)
        df['pos_x'].append(cam_pose[0, 3])
        df['pos_y'].append(cam_pose[1, 3])
        df['pos_z'].append(cam_pose[2, 3])

    ### rosbag ###
    for timestamp, cam_pose in zip(frame_data['pose_timestamps'], frame_data['poses']):
        df['method'].append('MapLab')
        df['timestamp'].append(timestamp)
        df['pos_x'].append(cam_pose[0, 3])
        df['pos_y'].append(cam_pose[1, 3])
        df['pos_z'].append(cam_pose[2, 3])

    df = {k: np.array(v) for k, v in df.items()}
    fig = px.line_3d(df, x='pos_x', y='pos_y', z='pos_z', color='method',
                     hover_data=['timestamp'], markers=True,
                     height=720, width=950)
    #fig.show()
    fig.write_html(save_pose_traj_dir / 'pose_trajectory.html')

def _tsdf_integration(volume: o3d.pipelines.integration.ScalableTSDFVolume,
                      frame_data: Dict[str, list], pose_refine=False) \
        -> Tuple[List[Tuple[float, np.ndarray]],
                 List[Tuple[float, o3d.t.pipelines.odometry.OdometryResult]]]:
    """Integrate frame_data into TSDF volume
    :return cam_poses_stamped : List of tuples, (timestamp, cam_pose)
    :return odometry_results_stamped : List of tuples, (timestamp, odometry_res)
    """
    cam_poses_stamped = []  # [(timestamp, cam_pose)]
    odometry_results_stamped = []  # [(timestamp, odometry_res)]
    for i, timestamp in enumerate(tqdm(frame_data['timestamps'])):
        rgb_im, depth_im, K, T_world_cam_i_hat = get_frame_data_at_timestamp(frame_data, timestamp)
        height, width, channels = rgb_im.shape

        rgbd_im_i = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rgb_im), o3d.geometry.Image(depth_im),
            convert_rgb_to_intensity=False
        )
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width, height, K[0,0], K[1,1], K[0,2], K[1,2]
        )

        T_world_cam_i = T_world_cam_i_hat
        if pose_refine:
            # o3d.t.geometry.RGBDImage.to_legacy() does not work
            t_rgbd_im_i = o3d.t.geometry.RGBDImage(o3d.t.geometry.Image(rgb_im),
                                                   o3d.t.geometry.Image(depth_im), aligned=True)

            if i > 0:
                T_cam_i_cam_in1_hat = np.linalg.inv(T_world_cam_i_hat) @ T_world_cam_in1_hat
                res = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
                    t_rgbd_im_in1, t_rgbd_im_i, K,
                    T_cam_i_cam_in1_hat,
                    depth_scale=1e3,
                    depth_max=6.0,
                    method=o3d.t.pipelines.odometry.Method.Hybrid
                )
                # Store odometry data
                odometry_results_stamped.append((timestamp, res))

                T_cam_i_cam_in1 = res.transformation.numpy()
                T_world_cam_i = T_world_cam_in1 @ np.linalg.inv(T_cam_i_cam_in1)

            # Save current frame data for next frame
            t_rgbd_im_in1 = t_rgbd_im_i
            T_world_cam_in1_hat = T_world_cam_i_hat
            T_world_cam_in1 = T_world_cam_i
        cam_poses_stamped.append((timestamp, T_world_cam_i))

        volume.integrate(rgbd_im_i, intrinsic, np.linalg.inv(T_world_cam_i))

    return cam_poses_stamped, odometry_results_stamped

def tsdf_reconstruction(bag_path: Path, save_mesh_path: Path,
                        voxel_length: float, pose_refine=False, save_pose_traj=False):
    """Use Open3D TSDF integration to reconstruct mesh from a rosbag.

    :param bag_path       : Path to the rosbag file to reconstruct
    :param save_mesh_path : Path to save the reconstructed PLY mesh
    :param voxel_length   : TSDF voxel length in meters
    :param pose_refine    : whether or not to perform consecutive frame pose refinement
    :param save_pose_traj : whether or not to save the pose trajectory
    """
    bag = rosbag.Bag(bag_path)

    # Read frame data from rosbag
    frame_data = read_frame_data_from_bag(bag)

    # Reference:
    # http://www.open3d.org/docs/0.15.1/tutorial/reconstruction_system/integrate_scene.html
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=args.voxel_length,
        sdf_trunc=5*args.voxel_length,  # 5x voxel_length
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
        volume_unit_resolution=16,
        depth_sampling_stride=4
    )

    # Integrate frame_data into volume
    print('\nBegin integrating frames .....')
    cam_poses_stamped, odometry_results_stamped = _tsdf_integration(
        volume, frame_data, pose_refine
    )

    print('\nBegin extracting triangular mesh .....')
    mesh = volume.extract_triangle_mesh()
    o3d.io.write_triangle_mesh(str(save_mesh_path), mesh)

    if save_pose_traj:
        print('\nBegin saving pose trajectory .....')

        save_pose_traj_dir = Path(str(save_mesh_path.with_suffix('')) + '_pose')
        save_pose_traj_dir.mkdir(parents=False, exist_ok=True)

        _save_pose_traj(save_pose_traj_dir, frame_data,
                        cam_poses_stamped, odometry_results_stamped)
        print(f'Pose trajectory is saved to {save_pose_traj_dir}')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Open3D TSDF integration from rosbag and saved as a PLY mesh.\n\t')
    parser.add_argument(
        'bag_path', type=str, help="Path to the rosbag for reconstruction")
    parser.add_argument(
        "--save-dir", type=str, default='/maya-slam/scene_recon/output_plys',
        help="Path to saving directory. If not provided, save to scene_recon/output_plys.")
    parser.add_argument(
        "--voxel-length", type=float, default=0.005,
        help="TSDF voxel length (in meters)")
    parser.add_argument(
        "--pose-refine", action='store_true',
        help="If specified, perform additional consecutive frame pose refinement")
    parser.add_argument(
        "--save-pose-traj", action='store_true',
        help="If specified, save the pose trajectory")
    #parser.add_argument(
    #    "--box-size", type=float, default=1.0, help="Chunk cube side length")
    args = parser.parse_args()

    # Verify bag_path
    args.bag_path = Path(args.bag_path).resolve()
    assert args.bag_path.is_file(), f"Path {args.bag_path} does not exist"

    args.save_dir = Path(args.save_dir).resolve()
    args.save_mesh_path = args.save_dir / f"{args.bag_path.stem}_O3DTSDF{args.voxel_length}m.ply"

    print(f'\nUsing TSDF voxel_length of {args.voxel_length} meter')

    tsdf_reconstruction(args.bag_path, args.save_mesh_path,
                        args.voxel_length, args.pose_refine, args.save_pose_traj)

    print(f'\nSuccessfully reconstruct from "{args.bag_path.name}" and saved as "{args.save_mesh_path}"\n')
