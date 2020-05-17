#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3Stamped, Pose
from drone_control.msg import VelocityYawrate

import dynamic_reconfigure.client

from tf.transformations import quaternion_from_euler

class JoyControl:
    def __init__(self):
        rospy.init_node("joy_control")
        self.node_name = rospy.get_name()
        self.pitch_d = 0.0
        self.roll_d = 0.0
        self.ibvs_switch = False
        self.reconfig_client = dynamic_reconfigure.client.Client("ibvs_controller", timeout=30)
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.velocity_yawrate_pub = rospy.Publisher("velocity_controller/command", VelocityYawrate, queue_size=1)
        self.gimbal_attitude_pub = rospy.Publisher("gimbal_controller/command", Vector3Stamped, queue_size=1)
        self.pose_pub = rospy.Publisher("ibvs_controller/command", Pose, queue_size=1)
        return
    def spin(self):
        rospy.spin()
        return
    def joy_callback(self, joy):
        joy_buttons = joy.buttons
        joy_axes = joy.axes
        if joy_buttons[7] == 1:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = -0.0
            pose.position.z = -0.4
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
        if joy_buttons[5] == 1:
            self.ibvs_switch = not self.ibvs_switch
            self.reconfig_client.update_configuration({"ibvs_switch": self.ibvs_switch})
        if self.ibvs_switch:
            return
        velocity_yawrate = VelocityYawrate()
        velocity_yawrate.header.stamp = rospy.Time.now()
        velocity_yawrate.yaw_rate = -3.14 / 3.0 * joy_axes[0]
        velocity_yawrate.vx = 2.0 * joy_axes[3]
        velocity_yawrate.vy = -2.0 * joy_axes[2]
        velocity_yawrate.vz = -2.0 * joy_axes[1]
        self.velocity_yawrate_pub.publish(velocity_yawrate)

        self.pitch_d -= joy_axes[4] * 3.14 / 90.0
        self.roll_d += joy_axes[5] * 3.14 / 90.0
        gimbal_attitude = Vector3Stamped()
        gimbal_attitude.header.stamp = rospy.Time.now()
        gimbal_attitude.vector.x = self.roll_d
        gimbal_attitude.vector.y = self.pitch_d
        gimbal_attitude.vector.z = 0.0
        self.gimbal_attitude_pub.publish(gimbal_attitude)
        return

if __name__ =="__main__":
    joy_control = JoyControl()
    joy_control.spin()
