#!/usr/bin/env python3
"""Convert a rosbag to an MP4 video
Usage: python3 rosbag_to_video.py ../slam_algorithms/rosbags/d435i_corridor1.bag
"""
import sys
from pathlib import Path
from typing import Dict

import numpy as np
import matplotlib.pyplot as plt
import cv2
from tqdm import tqdm

import rosbag
from cv_bridge import CvBridge

BASEPATH = Path(__file__).resolve().parents[1]  # maya-slam/
sys.path.insert(0, str(BASEPATH))

from tools.image_helper import ImageHelper

def read_frame_data_from_bag(bag: rosbag.Bag) -> Dict[str, list]:
    """Read frame data from a rosbag"""
    frame_data = {
        'rgb_imgs': [],  # [H, W, 3] uint8 np.ndarray
        'rgb_img_timestamps': [],
        'depth_imgs': [],  # [H, W] uint16 np.ndarray
        'depth_img_timestamps': [],
        'timestamps': [],  # unique timestamps with imgs and poses
    }

    cvbridge = CvBridge()
    topic_msgs = bag.read_messages(topics=['/camera/color/image_raw',
                                           '/camera/aligned_depth_to_color/image_raw'])
    for i, (topic, msg, t) in enumerate(topic_msgs):
        # Extract msg timestamp
        msg_timestamp = round(msg.header.stamp.to_sec(), 6)

        if topic == '/camera/color/image_raw':
            frame_data['rgb_imgs'].append(cvbridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
            frame_data['rgb_img_timestamps'].append(msg_timestamp)
        elif topic == '/camera/aligned_depth_to_color/image_raw':
            frame_data['depth_imgs'].append(cvbridge.imgmsg_to_cv2(msg, desired_encoding='16UC1'))
            frame_data['depth_img_timestamps'].append(msg_timestamp)

    # Find timestamps with both rgb_im and depth_im
    from functools import reduce
    timestamps = reduce(np.intersect1d, [v for k, v in frame_data.items() if '_timestamps' in k]).tolist()
    frame_data['timestamps'] = timestamps
    print(f"Found {len(timestamps)} timestamps with RGB and depth images")

    return frame_data

def convert_rosbag_to_video(bag_path: Path, save_dir: Path,
                            save_frames=False, cmap=plt.get_cmap('jet')):
    """Convert a rosbag to an MP4 video

    :param bag_path    : Path to the rosbag file to convert
    :param save_dir    : Path to save the converted video
    :param save_frames : whether or not to save the image frames
    :param cmap        : colormap for plotting depth images
    """
    bag = rosbag.Bag(bag_path)

    # Read frame data from rosbag
    print('\nBegin reading rosbag .....')
    frame_data = read_frame_data_from_bag(bag)

    height, width, channels = frame_data['rgb_imgs'][0].shape
    max_depth = 10 * 1e3  # 10 meters

    ##### Create image frames #####
    print('\nBegin creating image frames .....')
    frame_images = []
    max_combined_img_size = [0, 0]
    for i, timestamp in enumerate(tqdm(frame_data['timestamps'], desc='Frame')):
        rgb_im = frame_data['rgb_imgs'][frame_data['rgb_img_timestamps'].index(timestamp)]
        depth_im = frame_data['depth_imgs'][frame_data['depth_img_timestamps'].index(timestamp)]
        depth_im = (cmap(depth_im / max_depth) * 255).astype('uint8')[..., :3]

        im = ImageHelper()
        _ = im.add_image(rgb_im, 'rgb', [-1, -1])
        _ = im.add_image(depth_im, 'depth', [width+20, 0], 'rgb')
        _ = im.add_multiline_text([im.width/2, im.sub_images_top-20],
                                   f'Frame {i:05d} - {timestamp:.6f}',
                                   font_size=30, anchor='mb')

        if save_frames:
            save_frame_path = save_dir / f'Frame_{i:05d}_{timestamp:.6f}.png'
            im.save_image(save_frame_path)
        frame_images.append(im.cv2_image)
        #im.show_image()

        max_combined_img_size = np.maximum(max_combined_img_size, [im.width, im.height])

    ##### Create video #####
    print('\nBegin creating video .....')
    video_size = tuple(max_combined_img_size)
    video_path = str(save_dir / f'{save_dir.name}.mp4')
    video_out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'MP4V'), 30, video_size)

    for image in tqdm(frame_images, desc='Video'):
        height, width, channel = image.shape[:3]

        out_image = np.zeros(video_size[::-1] + (channel,), image.dtype)
        out_image[:height, -width:, :] = image
        video_out.write(out_image)
    video_out.release()
    print(f'\nSuccessfully convert "{bag_path.name}" and saved as "{video_path}"\n')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Convert a rosbag to an MP4 video.\n\t')
    parser.add_argument(
        'bag_path', type=str, help="Path to the rosbag file")
    parser.add_argument(
        "--save-dir", type=str, default=None,
        help="Path to saving directory. If not provided, save to same directory as bag_path.")
    parser.add_argument(
        "--save-frames", action='store_true',
        help="If specified, save the image frames. Otherwise, only save the video")
    args = parser.parse_args()

    # Verify bag_path
    args.bag_path = Path(args.bag_path).resolve()
    assert args.bag_path.is_file(), f"Path {args.bag_path} does not exist"

    if args.save_dir is None:
        args.save_dir = args.bag_path.with_name(f'{args.bag_path.stem}_bag_frames')
    else:
        args.save_dir = Path(args.save_dir).resolve()
    args.save_dir.mkdir(parents=False, exist_ok=False)

    convert_rosbag_to_video(args.bag_path, args.save_dir, args.save_frames)
