#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
from drone_ibvs.srv import GetOdometry, GetOdometryResponse
from tf.transformations import quaternion_multiply

from threading import Lock
import numpy as np

class Telemetry:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.odometry = Odometry()
        self.odometry_mtx = Lock()
        self.odometry_sub = rospy.Subscriber('odometry_sensor1/odometry', Odometry, self.odometry_callback, queue_size=1)
        rospy.Service("{}/get_odometry".format(self.node_name), GetOdometry, self.get_odometry_handler)
    def odometry_callback(self, odometry):
        q = odometry.pose.pose.orientation
        w = odometry.twist.twist.angular
        w_b = np.float32([w.x, -w.y, -w.z])
        q_i2b = quaternion_multiply([-1.0, 0.0, 0.0, 0.0], quaternion_multiply([q.x, q.y, q.z, q.w], [1.0, 0.0, 0.0, 0.0]))
        odometry.pose.pose.orientation.x = q_i2b[0]
        odometry.pose.pose.orientation.y = q_i2b[1]
        odometry.pose.pose.orientation.z = q_i2b[2]
        odometry.pose.pose.orientation.w = q_i2b[3]
        odometry.twist.twist.angular.x = w_b[0]
        odometry.twist.twist.angular.y = w_b[1]
        odometry.twist.twist.angular.z = w_b[2]
        with self.odometry_mtx:
            self.odometry = odometry
        return
    def get_odometry_handler(self, request=None):
        with self.odometry_mtx:
            odometry = self.odometry
        return GetOdometryResponse(odometry)

if __name__ == "__main__":
    rospy.init_node("telemetry")
    telemetry = Telemetry()
    rospy.spin()