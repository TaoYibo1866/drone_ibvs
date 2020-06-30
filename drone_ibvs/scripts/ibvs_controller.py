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

from detector import Detector
from transform import quaternion, transform_by_quaternion, euler_from_quaternion, quaternion_from_euler, quaternion_inverse, quaternion_multiply

class IBVSController:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.gimbal_camera_name = "pan_tilt_camera"
        self.detector = Detector()
        self.cv_bridge = CvBridge()

        self.w = 1.0 * rospy.get_param("{}/camera_intrinsic/w".format(self.gimbal_camera_name))
        self.h = 1.0 * rospy.get_param("{}/camera_intrinsic/h".format(self.gimbal_camera_name))
        self.f = 1.0 * rospy.get_param("{}/camera_intrinsic/f".format(self.gimbal_camera_name))
        self.cameraMatrix = np.reshape(np.float32(rospy.get_param("{}/camera_intrinsic/cameraMatrix/data".format(self.gimbal_camera_name))), (3, 3))
        self.distCoeffs = np.reshape(np.float32(rospy.get_param("{}/camera_intrinsic/distCoeffs/data".format(self.gimbal_camera_name))), (1, 4))
        
        self.u_min = 1.0 * rospy.get_param("{}/u_min".format(self.node_name))
        self.u_max = 1.0 * rospy.get_param("{}/u_max".format(self.node_name))
        self.v_min = 1.0 * rospy.get_param("{}/v_min".format(self.node_name))
        self.v_max = 1.0 * rospy.get_param("{}/v_max".format(self.node_name))
        self.k = 1.0 * rospy.get_param("{}/k".format(self.node_name))
        self.quadCorners3d = np.reshape(np.float32(rospy.get_param("{}/quadCorners3d/data".format(self.node_name))), (4, 3))
        
        self.sd = None
        self.Ld = None
        self.cVc_mtx = Lock()
        self.cVc = np.zeros((6, 1))

        self.image_pub = rospy.Publisher("{}/image_output".format(self.node_name), Image, queue_size=1)
        self.velocity_yawrate_pub = rospy.Publisher('velocity_controller/command', VelocityYawrate, queue_size=1)
        self.gimbal_attitude_pub = rospy.Publisher('gimbal_controller/command', Vector3Stamped, queue_size=1)
        
        self.get_odometry_client = rospy.ServiceProxy("telemetry/get_odometry", GetOdometry)
        self.do_ibvs_as = actionlib.SimpleActionServer("{}/do_ibvs".format(self.node_name), DoIBVSAction, self.execute, auto_start=False)
        self.do_ibvs_as.register_preempt_callback(self.cancel_cb)
        self.do_ibvs_as.start()
        return

    def execute(self, goal):
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
        self.cVc = np.zeros((6, 1))
        velocity_yawrate = VelocityYawrate()
        self.velocity_yawrate_pub.publish(velocity_yawrate)
        return

    def cancel_cb(self):
        if self.do_ibvs_as.is_preempt_requested():
            self.do_ibvs_as.set_aborted(DoIBVSResult(errno=2))
        return

    def image_callback(self, img_msg):
        if not self.do_ibvs_as.is_active():
            return
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        # TAG DETECTOR
        errno, corners = self.detector.search(img)
        if errno != 0:
            rospy.loginfo("detector errno: {}".format(errno))
            self.image_pub.publish(img_msg)
            return

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
    
    def calc_euler_w(self, w_c, w_b, t1, t2):
        w_c = w_c.reshape(3, 1)
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
    
    def calc_iVb(self, v_c, q_i2b, q_b2c):
        q_i2c = quaternion_multiply(q_i2b, q_b2c)
        q_c2i = quaternion_inverse(q_i2c)
        return transform_by_quaternion(q_i2c, v_c, q_c2i)

    def calc_cVc(self, corners):
        sd = self.sd
        Ld = self.Ld
        s = self.undistort_and_transform(corners)
        _, rvec, tvec = cv2.solvePnP(self.quadCorners3d.reshape(4, 1, 3),
                                    corners.reshape(4, 1, 2),
                                    self.cameraMatrix, self.distCoeffs)
        R_c2i, _ = cv2.Rodrigues(rvec)
        pts_c = (np.matmul(R_c2i, np.transpose(self.quadCorners3d)) + tvec).transpose()
        z = pts_c[:, 2]
        L0 = self.calc_Li(z[0], s[0])
        L1 = self.calc_Li(z[1], s[1])
        L2 = self.calc_Li(z[2], s[2])
        L3 = self.calc_Li(z[3], s[3])
        L = np.vstack((L0, L1, L2, L3))
        E0, e0, b0 = self.calc_Ei(s[0], sd[0])
        E1, e1, b1 = self.calc_Ei(s[1], sd[1])
        E2, e2, b2 = self.calc_Ei(s[2], sd[2])
        E3, e3, b3 = self.calc_Ei(s[3], sd[3])
        E = np.vstack((E0, E1, E2, E3))
        L_plus = 0.5 * np.linalg.pinv(L + Ld)
        #L_plus = np.linalg.pinv(Ld)
        cVc = -1.0 * self.k * np.matmul(L_plus, E)

        e = np.vstack((e0, e1, e2, e3))
        b = np.vstack((b0, b1, b2, b3))

        q_i2c = [0, 0, 0, 0]
        t_i2c = np.matmul(-R_c2i.transpose(), tvec)
        return cVc, E, e, b, q_i2c, t_i2c

    def undistort_and_transform(self, p_uv):
        # undistort
        # ...
        #transform
        s = p_uv - np.float32([self.w / 2.0, self.h / 2.0])
        return s

    def calc_Li(self, zi, si):
        u = 1.0 * si[0]
        v = 1.0 * si[1]
        z = 1.0 * zi
        f = self.f
        return np.float32([[-f / z, 0, u / z, u * v / f, -(f * f + u * u) / f, v],
                           [0, -f / z, v / z, (f * f + v * v) / f, -u * v / f, -u]])

    def calc_Ei(self, si, sdi):
        u = 1.0 * si[0]
        v = 1.0 * si[1]
        ud = 1.0 * sdi[0]
        vd = 1.0 * sdi[1]
        Mu_lb = ud - self.u_min
        Mu_ub = self.u_max - ud
        Mv_lb = vd - self.v_min
        Mv_ub = self.v_max - vd
        e_u = u - ud
        e_v = v - vd
        assert (1 + e_u / Mu_lb ) / (1 - e_u / Mu_ub ) > 0
        assert (1 + e_v / Mv_lb ) / (1 - e_v / Mv_ub ) > 0
        E_u = np.log((1 + e_u / Mu_lb ) / (1 - e_u / Mu_ub )) * (Mu_lb * Mu_ub / (Mu_lb + Mu_ub))
        E_v = np.log((1 + e_v / Mv_lb ) / (1 - e_v / Mv_ub )) * (Mv_lb * Mv_ub / (Mv_lb + Mv_ub))
        e = np.float32([[e_u], [e_v]])
        E = np.float32([[E_u], [E_v]])
        b = np.float32([[Mu_ub], [-Mu_lb], [Mv_ub], [-Mv_lb]])
        return E, e, b

    def calc_sd_Ld(self, pose_msg):
        q_w2c = np.float32([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])
        t_w2c = np.float32([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        sd0, Ld0 = self.calc_sdi_Ldi(self.quadCorners3d[0], q_w2c, t_w2c)
        sd1, Ld1 = self.calc_sdi_Ldi(self.quadCorners3d[1], q_w2c, t_w2c)
        sd2, Ld2 = self.calc_sdi_Ldi(self.quadCorners3d[2], q_w2c, t_w2c)
        sd3, Ld3 = self.calc_sdi_Ldi(self.quadCorners3d[3], q_w2c, t_w2c)
        sd = np.vstack((sd0, sd1, sd2, sd3))
        Ld = np.vstack((Ld0, Ld1, Ld2, Ld3))
        return sd, Ld

    def calc_sdi_Ldi(self, p_w, q_w2c, t_w2c):
        q_c2w = quaternion_inverse(q_w2c)
        q = np.append(p_w - t_w2c, 0.0)
        x, y, z = quaternion_multiply(q_c2w, quaternion_multiply(q, q_w2c))[:3]
        u = self.f * x / z
        v = self.f * y / z
        assert u > self.u_min and u < self.u_max
        assert v > self.v_min and v < self.v_max
        return np.float32([u, v]), self.calc_Li(z, [u, v])
    
    def avoid_pylint_err(self, gimbal_state):
        return gimbal_state.vector.y, gimbal_state.vector.x