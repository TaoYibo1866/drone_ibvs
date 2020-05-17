#!/usr/bin/env python2.7

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from ibvs_control.msg import Velocity
from drone_control.msg import VelocityYawrate
from ibvs_control.cfg import IbvsControllerConfig
from dynamic_reconfigure.server import Server
import numpy as np
from numpy import sin, cos, tan, tanh
from numpy.linalg import norm

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply
from threading import Lock

from quaternion import transform_by_quaternion, quaternion

def inner_cmd(w_c, w_b, t1, t2):
    w_c = w_c.reshape(3,1)
    w_b_xy = np.asarray([[w_b[0]], [w_b[1]]], dtype=np.float32)
    sin_t1 = sin(t1)
    cos_t1 = cos(t1)
    tan_t1 = tan(t1)
    sin_t2 = sin(t2)
    cos_t2 = cos(t2)
    A = np.asarray([[cos_t1, 0],
                    [sin_t1*sin_t2, cos_t2],
                    [sin_t1*cos_t2, -sin_t2]], dtype=np.float32)
    M = np.asarray([[1, tan_t1*sin_t2, tan_t1*cos_t2],
                    [0, cos_t2, -sin_t2],
                    [0, sin_t2/cos_t1, cos_t2/cos_t1]], dtype=np.float32)
    return np.matmul(M, w_c - np.matmul(A, w_b_xy))

class CommandGenerator:
    def __init__(self):
        rospy.init_node('command_generator')
        self.node_name = rospy.get_name()
        self.odometry_mtx = Lock()
        self.odometry_ready = False
        self.q_i2b = None
        self.w_b = None
        self.gimbal_mtx = Lock()
        self.gimbal_ready = False
        self.q_b2c = None
        self.t1 = None
        self.t2 = None
        self.command_mtx = Lock()
        self.command_ready = False
        self.command = None
        # self.quaternion_update = QuaternionUpdate()
        self.gimbal_update = GimbalUpdate()
        rospy.Subscriber('odometry_sensor1/odometry', Odometry, self.odometry_callback, queue_size=1)
        rospy.Subscriber('gimbal_controller/state', Vector3Stamped, self.gimbal_state_callback, queue_size=1)
        rospy.Subscriber('ibvs_controller/velocity', Velocity, self.command_callback, queue_size=1)
        rospy.Timer(rospy.Duration(secs=0.01), self.timer_callback)
        self.velocity_yawrate_pub = rospy.Publisher('velocity_controller/command', VelocityYawrate, queue_size=1)
        self.gimbal_attitude_pub = rospy.Publisher('gimbal_controller/command', Vector3Stamped, queue_size=1)
        return
    def spin(self):
        rospy.spin()
        return
    def odometry_callback(self, odometry):
        q = odometry.pose.pose.orientation
        w = odometry.twist.twist.angular
        with self.odometry_mtx:
            self.w_b = np.asarray([w.x, -w.y, -w.z], np.float32)
            self.q_i2b = quaternion_multiply([-1.0, 0.0, 0.0, 0.0], quaternion_multiply([q.x, q.y, q.z, q.w], [1.0, 0.0, 0.0, 0.0]))
            if self.odometry_ready == False:
                self.odometry_ready = True
        return
    def gimbal_state_callback(self, state):
        roll = state.vector.x
        pitch = state.vector.y
        q_b2c = quaternion_from_euler(0.0, pitch, roll, axes="rzyx")
        with self.gimbal_mtx:
            self.q_b2c = q_b2c
            self.t1 = pitch
            self.t2 = roll
            if self.gimbal_ready == False:
                self.gimbal_ready = True
        return
    def command_callback(self, command):
        with self.command_mtx:
            if command.switch.data:
                self.command = command.twist
                self.command_ready = True
            else:
                self.command = None
                self.command_ready = False
        return
    def timer_callback(self, event):
        with self.odometry_mtx:
            odometry_ready = self.odometry_ready
            q_i2b = self.q_i2b
            w_b = self.w_b
        with self.gimbal_mtx:
            gimbal_ready = self.gimbal_ready
            q_b2c = self.q_b2c
            t1 = self.t1
            t2 = self.t2
        with self.command_mtx:
            command_ready = self.command_ready
            command = self.command
        if not odometry_ready or not gimbal_ready:
            return
        if not command_ready:
            # self.quaternion_update.reset(q_b2c)
            self.gimbal_update.reset(t1, t2)
            return
        v_c = np.asarray([command.linear.x, command.linear.y, command.linear.z], dtype=np.float32)
        w_c = np.asarray([command.angular.x, command.angular.y, command.angular.z], dtype=np.float32)

        v_c = 1 * tanh(1 * norm(v_c)) * v_c / norm(v_c)
        w_c = 0.5 * tanh(2 * norm(w_c)) * w_c / norm(w_c)

        q_i2c = quaternion_multiply(q_i2b, q_b2c)
        q_c2i = quaternion_inverse(q_i2c)

        v_i = transform_by_quaternion(q_i2c, v_c, q_c2i)

        # q_i2cd = self.quaternion_update.update(q_i2c, w_c, rospy.Time.now().to_nsec() / 1000000000.0)
        # roll, pitch, yaw = euler_from_quaternion(q_i2b, axes='rxyz')
        # q_xy = quaternion_from_euler(roll, pitch, 0.0, axes='rxyz')
        # q_d = quaternion_multiply(quaternion_inverse(q_xy), q_i2cd)
        # _, pitch_d, roll_d = euler_from_quaternion(q_d, axes="rzyx")

        cmd = inner_cmd(w_c, w_b, t1, t2)
        pitch_d, roll_d = self.gimbal_update.update(cmd[1, 0], cmd[0, 0])

        gimbal_attitude = Vector3Stamped()
        gimbal_attitude.header.stamp = rospy.Time.now()
        gimbal_attitude.vector.x = roll_d
        gimbal_attitude.vector.y = pitch_d
        gimbal_attitude.vector.z = 0.0
        self.gimbal_attitude_pub.publish(gimbal_attitude)

        velocity_yawrate = VelocityYawrate()
        velocity_yawrate.header.stamp = rospy.Time.now()
        velocity_yawrate.vx = v_i[0]
        velocity_yawrate.vy = v_i[1]
        velocity_yawrate.vz = v_i[2]
        velocity_yawrate.yaw_rate = cmd[2, 0]
        #velocity_yawrate.yaw_rate = w_c[2] #yaw_rate_ZYX(5.0 * (yaw_d - yaw))
        self.velocity_yawrate_pub.publish(velocity_yawrate)
        return

# class QuaternionUpdate:
#     def __init__(self):
#         self.first_call = True
#         self.last_call = 0.0
#         self.q = None
#         return
#     def reset(self, q):
#         self.first_call = True
#         self.last_call = 0.0
#         self.q = q
#         return
#     def update(self, q, w, timestamp_s):
#         assert len(w) == 3
#         assert len(q) == 4
#         if self.first_call:
#             self.last_call = timestamp_s
#             self.first_call = False
#             self.q = np.asarray(q, np.float32)
#             return self.q
#         dt = timestamp_s - self.last_call
#         self.last_call = timestamp_s
#         q = self.q + dt * 0.5 *quaternion_multiply(self.q, quaternion(w))
#         self.q = q / np.linalg.norm(q)
#         return self.q

class GimbalUpdate:
    def __init__(self):
        self.t1 = None
        self.t2 = None
        return
    def reset(self, t1, t2):
        self.t1 = t1
        self.t2 = t2
        return
    def update(self, dt1, dt2):
        self.t1 = self.t1 + 0.01 * dt1
        self.t2 = self.t2 + 0.01 * dt2
        return [self.t1, self.t2]

if __name__ == "__main__":
    command_generator = CommandGenerator()
    command_generator.spin()
