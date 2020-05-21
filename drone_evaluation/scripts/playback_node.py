#!/usr/bin/env python2.7
from sys import argv
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class PlayBack:
    def __init__(self, gimbal_camera_name):
        rospy.init_node("playback")
        self.node_name = rospy.get_name()
        self.img_pub = rospy.Publisher("{}/camera/image_raw".format(gimbal_camera_name), Image, queue_size=1)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()
    def spin(self, video_path):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            rospy.logerr("Open video fail!")
            return
        while not rospy.is_shutdown():
            ret, img = cap.read()
            if not ret:
                rospy.logerr("End of the video!")
                return
            try:
                imgmsg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            except CvBridgeError, e:
                rospy.logerr("CvBridge Error: {}".format(e))
                return
            self.img_pub.publish(imgmsg)
            self.rate.sleep()
        return

if __name__ == "__main__":
    # node = PlayBack("pan_tilt_camera")
    # node.spin("testset/test.mp4")
    node = PlayBack(argv[1])
    node.spin(argv[2])