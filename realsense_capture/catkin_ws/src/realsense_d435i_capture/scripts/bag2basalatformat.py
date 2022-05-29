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
    def __init__(self, direc_name, get_transform=False):
        # Params
        self.br = CvBridge()
        self.direc_name = direc_name


        self.get_transform = get_transform

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw",Image,self.color_callback)
        rospy.Subscriber("/camera/imu",Imu,self.imu_callback)

        self.image_pub = rospy.Publisher("/cam0/image_raw", Image, queue_size=1)
        self.imu_pub = rospy.Publisher("/cam0/imu", Imu, queue_size=1)



    def color_callback(self, msg):
        rospy.loginfo('Color Image received...')
        image = self.br.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        self.image_pub.publish(self.br.cv2_to_imgmsg(gray, "mono8"))



    def imu_callback(self, msg):
        # rospy.loginfo("IMU data received")
        self.imu_pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node("bag2data", anonymous=True)
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('realsense_d435i_capture')
    my_node = BagTool(os.path.join(pkg_path, "extracted_bags", "home_1"))
    print("INITIALIZED")
    rospy.spin()