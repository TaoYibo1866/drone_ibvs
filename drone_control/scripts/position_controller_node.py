#!/usr/bin/env python2.7

import numpy as np
from numpy import sin, cos
from numpy.linalg import norm
import roslib
roslib.load_manifest("drone_control")
import rospy
import actionlib
from nav_msgs.msg import Odometry
from drone_control.msg import VelocityYawrate, GoToPointAction, GoToPointGoal, GoToPointFeedback, GoToPointResult

from controller import PIController

class PositionController:
    def __init__(self):
        self.node_name = rospy.get_name()
        Kp_x = rospy.get_param("{}/Kp_x".format(self.node_name))
        Ki_x = rospy.get_param("{}/Ki_x".format(self.node_name))
        Kp_y = rospy.get_param("{}/Kp_y".format(self.node_name))
        Ki_y = rospy.get_param("{}/Ki_y".format(self.node_name))
        Kp_z = rospy.get_param("{}/Kp_z".format(self.node_name))
        Ki_z = rospy.get_param("{}/Ki_z".format(self.node_name))
        self.px_controller = PIController(Kp_x, Ki_x, lower_limit=-2, upper_limit=2)
        self.py_controller = PIController(Kp_y, Ki_y, lower_limit=-2, upper_limit=2)
        self.pz_controller = PIController(Kp_z, Ki_z, lower_limit=-2, upper_limit=2)
        self.goal = GoToPointGoal()
        self.vel_pub = rospy.Publisher("velocity_controller/command", VelocityYawrate, queue_size=1)
        self.pos_as = actionlib.SimpleActionServer("{}/go_to_point".format(self.node_name), GoToPointAction, execute_cb=self.execute, auto_start=False)
        self.pos_as.register_preempt_callback(self.cancel)
        self.pos_as.start()
    def execute(self, goal):
        self.goal = goal
        odometry_sub = rospy.Subscriber('odometry_sensor1/odometry', Odometry, self.odometry_callback)
        r = rospy.Rate(1)
        while True:
            if not self.pos_as.is_active():
                break
            r.sleep()
        odometry_sub.unregister()
        self.vel_pub.publish(VelocityYawrate())
        return

    def cancel(self):
        if self.pos_as.is_preempt_requested():
            self.pos_as.set_aborted(GoToPointResult(errno=1))
        return

    def odometry_callback(self, odometry):
        if not self.pos_as.is_active():
            return
        ex = odometry.pose.pose.position.x - self.goal.desired_position.x
        ey = -odometry.pose.pose.position.y - self.goal.desired_position.y
        ez = -odometry.pose.pose.position.z - self.goal.desired_position.z
        velocity_yawrate = VelocityYawrate()
        velocity_yawrate.header.stamp = rospy.Time.now()
        velocity_yawrate.vx = self.px_controller.run(odometry.pose.pose.position.x, timestamp_s = odometry.header.stamp.to_sec(), reference=self.goal.desired_position.x)
        velocity_yawrate.vy = self.px_controller.run(-odometry.pose.pose.position.y, timestamp_s = odometry.header.stamp.to_sec(), reference=self.goal.desired_position.y)
        velocity_yawrate.vz = self.px_controller.run(-odometry.pose.pose.position.z, timestamp_s = odometry.header.stamp.to_sec(), reference=self.goal.desired_position.z)
        self.vel_pub.publish(velocity_yawrate)
        feedback = GoToPointFeedback()
        feedback.postion_err.x = ex
        feedback.postion_err.y = ey
        feedback.postion_err.z = ez
        self.pos_as.publish_feedback(feedback)
        # TODO set succeeded based on criteria in goal
        if norm(np.float32([ex, ey, ez])) < 0.05:
            self.pos_as.set_succeeded(GoToPointResult(errno=0))
        return



if __name__ == "__main__":
    rospy.init_node("position_controller")
    position_controller = PositionController()
    rospy.spin()