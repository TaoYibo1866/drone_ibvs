#!/usr/bin/env python2.7

from sys import argv
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import MultiArrayDimension
from vs_control.msg import QuadCorners

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from detector import LogoDetector, MarkerDetector

class VSFrontEnd:
    def __init__(self, gimbal_camera_name):
        rospy.init_node("vs_front_end")
        self.node_name = rospy.get_name()
        self.state = 0
        self.bridge = CvBridge()
        self.detector = LogoDetector()
        rospy.Subscriber("{}/camera/image_raw".format(gimbal_camera_name), Image, self.image_callback, queue_size=3)
        self.img_pub = rospy.Publisher("{}/image_output".format(self.node_name), Image, queue_size=1)
    def image_callback(self, image_raw):
        try:
            src = self.bridge.imgmsg_to_cv2(image_raw, desired_encoding='bgr8')
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        img = np.copy(src)
        if self.state == 0:
            found, rect = self.detector.search(src)
            if found:
                x1, y1, w, h = rect
                img = cv2.rectangle(img, (x1, y1), (x1 + w, y1 + h), (0, 0, 255))
        try:
            imgmsg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        self.img_pub.publish(imgmsg)
        return
    def spin(self):
        rospy.spin()
        return

class MarkerVSFrontEnd:
    def __init__(self, gimbal_camera_name):
        rospy.init_node("vs_front_end")
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        self.detector = MarkerDetector()
        rospy.Subscriber("{}/camera/image_raw".format(gimbal_camera_name), Image, self.image_callback, queue_size=3)
        self.img_pub = rospy.Publisher("{}/image_output".format(self.node_name), Image, queue_size=1)
        self.corners_pub = rospy.Publisher("{}/0/corners".format(self.node_name), QuadCorners, queue_size=1)
    def image_callback(self, image_raw):
        try:
            src = self.bridge.imgmsg_to_cv2(image_raw, desired_encoding='bgr8')
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        img = np.copy(src)
        found, corners = self.detector.search(src)
        if found:
            corners_msg = QuadCorners()
            corners_msg.header.stamp = rospy.Time.now()
            # corners_msg.corners.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
            # corners_msg.corners.layout.dim[0].size = 4
            # corners_msg.corners.layout.dim[0].label = 'row'
            # corners_msg.corners.layout.dim[1].size = 2
            # corners_msg.corners.layout.dim[1].label = 'col'
            corners_msg.corners.data = np.reshape(corners, -1).tolist()
            self.corners_pub.publish(corners_msg)
            img = cv2.aruco.drawDetectedMarkers(img, [corners], borderColor=(0, 0, 255))
        try:
            imgmsg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        self.img_pub.publish(imgmsg)
        return
    def spin(self):
        rospy.spin()
        return

if __name__ == "__main__":
    vs_front_end = MarkerVSFrontEnd(argv[1])
    vs_front_end.spin()