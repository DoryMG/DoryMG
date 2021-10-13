#!/usr/bin/env python
# -*- coding: utf-8 -*-
""""
Created on Wed Oct 6 22:06:27 2021
@author: wiseman
"""

import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class ImageCapture:
    def __init__(self):
        self.pos_x = 3.3
        self.pos_y = 7.7
        self.theta = 3.14
        self.image = np.empty((480, 640), dtype=np.uint8)
        self.pub = rospy.Publisher('rpi_image', CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber('odom', Odometry, self.update_pose)

    def update_pose(self, msg):
        pose = msg.pose.pose
        self.pos_x = pose.position.x
        self.pos_y = pose.position.y
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        _, _, self.theta = euler_from_quaternion(quat)

    def capture_image(self, event=None):
        cam = cv2.VideoCapture("/deb-v/video0", cv2.CAP_V4L)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        ret = False
        while not ret:
            ret, frame = cam.read()
        self.image = frame
        rospy.loginfo("image was captured")
        cam.release()

    def publish_image(self, event=None):
        bridge = CvBridge()
        try:
            msg = bridge.cv2_to_compressed_imgmsg(self.image)
        except CvBridgeError as e:
            print(e)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'x:{self.pos_x:.2f}_y:{self.pos_y:.2f}_a{self.theta:.2f}'
        self.pub.publish(msg)
        rospy.loginfo("image was published")

    def run(self, capture_count=10.0, publish_count=50, duration=1.0):
        cap_timer = rospy.Timer(rospy.Duration(duration / capture_count), self.capture_image)
        pub_timer = rospy.Timer(rospy.Duration(duration / capture_count), self.capture_image)
        rospy.sleep(duration)
        cap_timer.shutdown()
        pub_timer.shutdown()


if __name__ == "__main__":
    rospy.init_node("image_capture")
    image_capture = ImageCapture()
    image_capture.run(capture_count=10.0, publish_count=5.0, duration=5.0)
    rospy.spin()
