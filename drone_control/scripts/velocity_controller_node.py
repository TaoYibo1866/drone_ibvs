#!/usr/bin/env python2.7
import rospy
from dynamic_reconfigure.server import Server
from drone_control.cfg import VelocityControllerConfig
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from drone_control.msg import VelocityYawrate
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply
from quaternion import transform_by_quaternion
from numpy import cos
from controller import PIController

from threading import Lock

class VelocityController:
    def __init__(self):
        rospy.init_node('velocity_controller')
        self.node_name = rospy.get_name()
        self.G = rospy.get_param("{}/G".format(self.node_name))
        Kp_x = rospy.get_param("{}/Kp_x".format(self.node_name))
        Ki_x = rospy.get_param("{}/Ki_x".format(self.node_name))
        Kp_y = rospy.get_param("{}/Kp_y".format(self.node_name))
        Ki_y = rospy.get_param("{}/Ki_y".format(self.node_name))
        Kp_z = rospy.get_param("{}/Kp_z".format(self.node_name))
        Ki_z = rospy.get_param("{}/Ki_z".format(self.node_name))
        self.first_reconfigure_call = True
        self.vx_controller = PIController(Kp_x, Ki_x, lower_limit=-3.14 / 6, upper_limit=3.14 / 6)
        self.vy_controller = PIController(Kp_y, Ki_y, lower_limit=-3.14 / 6, upper_limit=3.14 / 6)
        self.vz_controller = PIController(Kp_z, Ki_z, lower_limit=-1 * self.G, upper_limit=1.5 * self.G)
        self.v_w_reference = [0.0, 0.0, 0.0]
        self.yawrate_reference = 0.0
        self.command_mtx = Lock()
        _ = Server(VelocityControllerConfig, self.reconfigure_callback, namespace="velocity_controller_tune")
        rospy.Subscriber('odometry_sensor1/odometry', Odometry, self.odometry_callback)
        rospy.Subscriber('{}/command'.format(self.node_name),VelocityYawrate, self.command_callback)
        self.roll_pitch_yawrate_thrust_pub = rospy.Publisher('command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
        self.velocity_yawrate_pub = rospy.Publisher('{}/state'.format(self.node_name), VelocityYawrate, queue_size=1)
    def spin(self):
        rospy.spin()
        return
    def reconfigure_callback(self, config, level):
        if self.first_reconfigure_call:
            self.first_reconfigure_call = False
            return config
        self.vx_controller.set_Kp_Ki(config.Kp_x, config.Ki_x)
        self.vy_controller.set_Kp_Ki(config.Kp_y, config.Ki_y)
        self.vz_controller.set_Kp_Ki(config.Kp_z, config.Ki_z)
        return config
    def command_callback(self, command):
        # command is in NED
        with self.command_mtx:
            self.v_w_reference = [command.vx, -command.vy, -command.vz]
            self.yawrate_reference = -command.yaw_rate
        return
    def odometry_callback(self, odometry):
        # body frame is gazebo base_link frame
        timestamp_s = odometry.header.stamp.to_sec()
        q = odometry.pose.pose.orientation
        v = odometry.twist.twist.linear
        w = odometry.twist.twist.angular

        q_w2b = [q.x, q.y, q.z, q.w]
        q_b2w = quaternion_inverse(q_w2b)

        psi, theta, phi = euler_from_quaternion(q_w2b, axes='rzyx')
        q_w2rb = quaternion_from_euler(psi, 0.0, 0.0, axes='rzyx')
        q_rb2w = quaternion_inverse(q_w2rb)

        q_b2rb = quaternion_multiply(q_b2w, q_w2rb)
        q_rb2b = quaternion_inverse(q_b2rb)

        v_b = [v.x, v.y, v.z]
        v_w = transform_by_quaternion(q_w2b, v_b, q_b2w)
        v_rb = transform_by_quaternion(q_rb2b, v_b, q_b2rb)
        with self.command_mtx:
            v_w_reference = self.v_w_reference
            yawrate_reference = self.yawrate_reference
        v_rb_reference = transform_by_quaternion(q_rb2w, v_w_reference, q_w2rb)

        roll = -1 * self.vy_controller.run(v_rb[1], timestamp_s, reference=v_rb_reference[1])
        pitch = self.vx_controller.run(v_rb[0], timestamp_s, reference=v_rb_reference[0])
        fz = self.G + self.vz_controller.run(v_rb[2], timestamp_s, reference=v_rb_reference[2])
        thrust = fz / (cos(phi) * cos(theta))

        roll_pitch_yawrate_thrust = RollPitchYawrateThrust()
        roll_pitch_yawrate_thrust.header.stamp = rospy.Time.now()
        roll_pitch_yawrate_thrust.roll = roll
        roll_pitch_yawrate_thrust.pitch = pitch
        roll_pitch_yawrate_thrust.yaw_rate = yawrate_reference
        roll_pitch_yawrate_thrust.thrust.x = 0.0
        roll_pitch_yawrate_thrust.thrust.y = 0.0
        roll_pitch_yawrate_thrust.thrust.z = thrust

        state = VelocityYawrate()
        state.header.stamp = rospy.Time.now()
        state.vx = v_w[0]
        state.vy = -v_w[1]
        state.vz = -v_w[2]
        state.yaw_rate = -w.z
        self.roll_pitch_yawrate_thrust_pub.publish(roll_pitch_yawrate_thrust)
        self.velocity_yawrate_pub.publish(state)
        return

if __name__ == '__main__':
    velocity_controller = VelocityController()
    velocity_controller.spin()
