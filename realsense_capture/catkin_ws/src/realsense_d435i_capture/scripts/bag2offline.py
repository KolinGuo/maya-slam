#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import cv2
import os
import tf2_ros
import numpy as np
import shutil


class BagTool(object):
    def __init__(self, direc_name, gray, get_transform=False):
        # Params
        self.br = CvBridge()
        self.direc_name = direc_name

        if os.path.exists(self.direc_name):
            shutil.rmtree(self.direc_name)           # Removes all the subdirectories!
        os.makedirs(self.direc_name)

        self.color_setup()
        self.depth_setup()
        self.imu_setup()

        self.get_transform = get_transform
        self.gray = gray

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.color_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.depth_callback)
        rospy.Subscriber("/camera/imu",Imu,self.imu_callback)

        if get_transform:
            self.tfBuffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tfBuffer)
            

    def shutdown(self):
        print("Entered shutdown!")
        self.color_csv_writer.close()
        self.depth_csv_writer.close()
        self.imu_csv_writer.close()


    def color_setup(self):
        os.makedirs(os.path.join(self.direc_name, "cam0/data"))
        self.color_csv_writer = open(os.path.join(self.direc_name, "cam0/data.csv"), 'wa')

    def depth_setup(self):
        os.makedirs(os.path.join(self.direc_name, "depth/data"))
        self.depth_csv_writer = open(os.path.join(self.direc_name, "depth/data.csv"), 'wa')

    def imu_setup(self):
        os.makedirs(os.path.join(self.direc_name, "imu/data"))
        self.imu_csv_writer = open(os.path.join(self.direc_name, "imu/data.csv"), 'wa')


    def color_callback(self, msg):
        # rospy.loginfo('Color Image received...')
        image = self.br.imgmsg_to_cv2(msg)
        ts = msg.header.stamp.to_nsec()
        if self.gray:
            image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        cv2.imwrite(os.path.join(self.direc_name, "cam0/data", str(ts) + '.png'), image)
        self.color_csv_writer.write(str(ts) + "," + str(ts) + '.png'+'\n')
        if self.get_transform:
            trans = tfBuffer.lookup_transform('map', 'imu', rospy.Time())



    def depth_callback(self, msg):
        # rospy.loginfo('Color Image received...')
        image = self.br.imgmsg_to_cv2(msg).astype(np.uint16)
        ts = msg.header.stamp.to_nsec()
        cv2.imwrite(os.path.join(self.direc_name, "depth/data", str(ts) + '.png'), image)
        self.depth_csv_writer.write(str(ts) + "," + str(ts) + '.png'+'\n')


    def imu_callback(self, msg):
        # rospy.loginfo("IMU data received")
        la = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        ang = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        ts = msg.header.stamp.to_nsec()
        # print(','.join(map(str, ang)))
        # print(','.join(ang))
        self.imu_csv_writer.write(str(ts)+','+','.join(map(str, ang))+ ','+ ','.join(map(str, la)) + '\n')
        # print(ts.to_nsec(), ang, la)
        # print(dir(msg))
        # done



if __name__ == '__main__':
    rospy.init_node("bag2data", anonymous=True)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('realsense_d435i_capture')
    my_node = BagTool(os.path.join(pkg_path, "extracted_bags", "home_1"), gray=True)
    print("INITIALIZED")
    rospy.on_shutdown(my_node.shutdown)
    rospy.spin()