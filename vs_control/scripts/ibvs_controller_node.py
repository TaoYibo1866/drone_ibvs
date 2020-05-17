#!/usr/bin/env python2.7

from sys import argv
import rospy
import numpy as np
from vs_control.msg import QuadCorners

from tf.transformations import quaternion_inverse, quaternion_multiply

class IbvsController:
    def __init__(self, gimbal_camera_name):
        rospy.init_node('ibvs_controller')
        self.node_name = rospy.get_name()
        rospy.loginfo(self.node_name)
        self.w = rospy.get_param("{}/w".format(self.node_name))
        self.h = rospy.get_param("{}/h".format(self.node_name))
        self.f = rospy.get_param("{}/f".format(self.node_name))
        self.u_min = rospy.get_param("{}/u_min".format(self.node_name))
        self.u_max = rospy.get_param("{}/u_max".format(self.node_name))
        self.v_min = rospy.get_param("{}/v_min".format(self.node_name))
        self.v_max = rospy.get_param("{}/v_max".format(self.node_name))
        self.k = rospy.get_param("{}/k".format(self.node_name))
        self.cameraMatrix = np.reshape(np.float32(rospy.get_param("{}/cameraMatrix/data".format(self.node_name))), (3, 3))
        self.distCoeffs = np.reshape(np.float32(rospy.get_param("{}/distCoeffs/data".format(self.node_name))), (1, 4))
        self.quadCorners3d = np.reshape(np.float32(rospy.get_param("{}/quadCorners3d/data".format(self.node_name))), (4, 3))
        return
    def spin(self):
        rospy.spin()
        return
    def corners_callback(self, corners):
        return
    def calc_sd_Ld(self, p_w, q_w2c, t_w2c):
        q_c2w = quaternion_inverse(q_w2c)
        q = [p_w[0] - t_w2c[0], p_w[1] - t_w2c[1], p_w[2] - t_w2c[2], 0.0]
        x, y, z = quaternion_multiply(q_c2w, quaternion_multiply(q, q_w2c))[:3]
        u = 1.0 * self.f * x / z
        v = 1.0 * self.f * y / z
        assert u > self.u_min and u < self.u_max
        assert v > self.v_min and v < self.v_max
        return np.float32([u, v]), self.calc_Li(z, [u, v])
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
        f = 1.0 * self.f
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
        e = np.float32([e_u, e_v])
        E = np.float32([[E_u], [E_v]])
        b = np.float32([Mu_ub, -Mu_lb, Mv_ub, -Mv_lb])
        return E, e, b

if __name__ == '__main__':
    ibvs_controller = IbvsController(argv[1])
    ibvs_controller.spin()