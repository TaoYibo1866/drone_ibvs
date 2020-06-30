#!/usr/bin/env python2.7

from sys import argv

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import sin, cos, tan, tanh
from numpy.linalg import norm
from threading import Lock

import roslib
roslib.load_manifest('drone_ibvs')
import rospy
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Vector3Stamped, Twist
from nav_msgs.msg import Odometry
from drone_control.msg import VelocityYawrate
from drone_ibvs.msg import DoIBVSAction, DoIBVSFeedback, DoIBVSResult
from drone_ibvs.srv import GetOdometry
from actionlib_msgs.msg import GoalStatus

from detector import LogoDetector, TagDetector
from transform import quaternion, transform_by_quaternion, euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply
from ibvs_controller import IBVSController

def draw_pts(img, pts):
    pts = np.reshape(pts, (-1, 2))
    for pt in pts:
        x = int(pt[0])
        y = int(pt[1])
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
    return img

class TagIBVSController(IBVSController):
    def __init__(self):
        IBVSController.__init__(self)
        self.detector = TagDetector()
    def image_callback(self, img_msg):
        if not self.do_ibvs_as.is_active():
            return
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # TAG DETECTOR
        errno, corners = self.detector.search(img)
        if errno != 0:
            rospy.loginfo("tagdetector errno: {}".format(errno))
            self.image_pub.publish(img_msg)
            return
        img = cv2.aruco.drawDetectedMarkers(img, [corners], borderColor=(0, 0, 255))
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_pub.publish(img_msg)

        try:
            cVc, _, e, _, _, _ = self.calc_cVc(corners.reshape(4, 2))
        except AssertionError:
            self.do_ibvs_as.set_aborted(DoIBVSResult(errno=1))
            return

        with self.cVc_mtx:
            self.cVc = cVc

        self.do_ibvs_as.publish_feedback(DoIBVSFeedback(pixel_err=norm(e)))
        if norm(e) < 10:
            self.do_ibvs_as.set_succeeded(DoIBVSResult(errno=0))
        return

class LogoIBVSController(IBVSController):
    def __init__(self):
        IBVSController.__init__(self)
        self.detector = LogoDetector()
    def image_callback(self, img_msg):
        if not self.do_ibvs_as.is_active():
            return
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        errno, corners = self.detector.search(img)
        if errno != 0:
            rospy.loginfo("detector errno: {}".format(errno))
            self.image_pub.publish(img_msg)
            return
        img = cv2.polylines(img, [np.reshape(np.int32(corners), (4, 2))], 1, (0, 255, 0), 2)
        img = draw_pts(img, self.detector.curr_pts)
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')

        self.image_pub.publish(img_msg)

        try:
            cVc, _, e, _, _, _ = self.calc_cVc(corners.reshape(4, 2))
        except AssertionError:
            self.do_ibvs_as.set_aborted(DoIBVSResult(errno=1))
            return

        with self.cVc_mtx:
            self.cVc = cVc

        self.do_ibvs_as.publish_feedback(DoIBVSFeedback(pixel_err=norm(e), nfeatures=self.detector.curr_pts.shape[0]))
        if norm(e) < 20:
            self.do_ibvs_as.set_succeeded(DoIBVSResult(errno=0))
        return
    def execute(self, goal):
        velocity_yawrate = VelocityYawrate()
        self.velocity_yawrate_pub.publish(velocity_yawrate)
        rospy.sleep(1)
        desired_pose = goal.desired_pose
        self.sd, self.Ld = self.calc_sd_Ld(desired_pose)
        gimbal_state = rospy.wait_for_message('gimbal_controller/state', Vector3Stamped)
        pitch_d, roll_d = self.avoid_pylint_err(gimbal_state)
        r = rospy.Rate(100)
        image_sub = rospy.Subscriber("{}/camera_0/image_raw".format(self.gimbal_camera_name), Image, self.image_callback, queue_size=1)
        while True:
            if not self.do_ibvs_as.is_active():
                break
            with self.cVc_mtx:
                cVc = self.cVc
            odometry = self.get_odometry_client()
            odometry = odometry.odometry
            q_i2b = [odometry.pose.pose.orientation.x,
                     odometry.pose.pose.orientation.y,
                     odometry.pose.pose.orientation.z,
                     odometry.pose.pose.orientation.w]
            w_b = [odometry.twist.twist.angular.x,
                   odometry.twist.twist.angular.y,
                   odometry.twist.twist.angular.z]
            q_b2c = quaternion_from_euler(0.0, pitch_d, roll_d, axes="rzyx")
            cVc = cVc.reshape(6)
            v_c = cVc[:3]
            w_c = cVc[3:]
            v_i = self.calc_iVb(v_c, q_i2b, q_b2c)
            w_rpy = self.calc_euler_w(w_c, w_b, pitch_d, roll_d)
            pitch_d = pitch_d + 0.01 * w_rpy[1, 0]
            roll_d = roll_d + 0.01 * w_rpy[0, 0]

            velocity_yawrate = VelocityYawrate()
            gimbal_attitude = Vector3Stamped()
            velocity_yawrate.header.stamp = rospy.Time.now()
            gimbal_attitude.header.stamp = rospy.Time.now()
            velocity_yawrate.vx = v_i[0]
            velocity_yawrate.vy = v_i[1]
            velocity_yawrate.vz = v_i[2]
            velocity_yawrate.yaw_rate = w_rpy[2, 0]
            gimbal_attitude.vector.x = roll_d
            gimbal_attitude.vector.y = pitch_d
            gimbal_attitude.vector.z = 0.0
            self.velocity_yawrate_pub.publish(velocity_yawrate)
            self.gimbal_attitude_pub.publish(gimbal_attitude)
            r.sleep()
        image_sub.unregister()
        self.detector.reset()
        self.cVc = np.zeros((6, 1))
        velocity_yawrate = VelocityYawrate()
        self.velocity_yawrate_pub.publish(velocity_yawrate)
        return

if __name__ == "__main__":
    rospy.init_node("ibvs_controller")
    if argv[1] == "tag":
        ibvs_controller = TagIBVSController()
    elif argv[1] == "logo":
        ibvs_controller = LogoIBVSController()
    rospy.spin()