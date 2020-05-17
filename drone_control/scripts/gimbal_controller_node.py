#!/usr/bin/env python2.7
from sys import argv

import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped

from threading import Lock

class GimbalController:
    def __init__(self, gimbal_camera_name):
        rospy.init_node('gimbal_controller')
        self.node_name = rospy.get_name()
        self.pitch_mtx = Lock()
        self.roll_mtx = Lock()
        self.command_mtx = Lock()
        self.pitch = None
        self.roll = None
        self.command = [0.0, 0.0, 0.0]
        rospy.Subscriber('{}/joint1_position_controller/state'.format(gimbal_camera_name), JointControllerState, self.joint1_state_callback, queue_size=1)
        rospy.Subscriber('{}/joint2_position_controller/state'.format(gimbal_camera_name), JointControllerState, self.joint2_state_callback, queue_size=1)
        rospy.Subscriber('{}/command'.format(self.node_name), Vector3Stamped, self.command_callback, queue_size=1)
        rospy.Timer(rospy.Duration(secs=0.01), self.timer_callback)
        self.gimbal_state_pub = rospy.Publisher('{}/state'.format(self.node_name), Vector3Stamped, queue_size=1)
        self.joint1_position_pub = rospy.Publisher('{}/joint1_position_controller/command'.format(gimbal_camera_name), Float64, queue_size=1)
        self.joint2_position_pub = rospy.Publisher('{}/joint2_position_controller/command'.format(gimbal_camera_name), Float64, queue_size=1)
        return
    def spin(self):
        rospy.spin()
    def joint1_state_callback(self, joint1_state):
        with self.pitch_mtx:
            self.pitch = joint1_state.process_value
        return
    def joint2_state_callback(self, joint2_state):
        with self.roll_mtx:
            self.roll = joint2_state.process_value
        return
    def command_callback(self, command):
        with self.command_mtx:
            self.command = [command.vector.x, command.vector.y, command.vector.z]
        return
    def timer_callback(self, event):
        with self.pitch_mtx:
            pitch = self.pitch
        with self.roll_mtx:
            roll = self.roll
        with self.command_mtx:
            command = self.command
        if pitch is not None and roll is not None:
            state = Vector3Stamped()
            state.header.stamp = rospy.Time.now()
            state.vector.x = roll
            state.vector.y = pitch
            state.vector.z = 0.0
            self.gimbal_state_pub.publish(state)
        self.joint1_position_pub.publish(command[1])
        self.joint2_position_pub.publish(command[0])

if __name__ == '__main__':
    gimbal_controller = GimbalController(argv[1])
    gimbal_controller.spin()