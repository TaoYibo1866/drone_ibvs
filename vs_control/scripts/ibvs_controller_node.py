#!/usr/bin/env python2.7

from sys import argv
import rospy
import cv2
import numpy as np
from threading import Lock
from drone_comm.msg import QuadCorners, IbvsState
from drone_comm.srv import SetDesiredPose, SetDesiredPoseResponse, SwitchOn, SwitchOnResponse

from tf.transformations import quaternion_inverse, quaternion_multiply

class IbvsController:
    def __init__(self, gimbal_camera_name, vs_front_end):
        rospy.init_node('ibvs_controller')
        self.node_name = rospy.get_name()

        self.w = 1.0 * rospy.get_param("{}/camera_intrinsic/w".format(gimbal_camera_name))
        self.h = 1.0 * rospy.get_param("{}/camera_intrinsic/h".format(gimbal_camera_name))
        self.f = 1.0 * rospy.get_param("{}/camera_intrinsic/f".format(gimbal_camera_name))
        self.cameraMatrix = np.reshape(np.float32(rospy.get_param("{}/camera_intrinsic/cameraMatrix/data".format(gimbal_camera_name))), (3, 3))
        self.distCoeffs = np.reshape(np.float32(rospy.get_param("{}/camera_intrinsic/distCoeffs/data".format(gimbal_camera_name))), (1, 4))
        
        self.u_min = 1.0 * rospy.get_param("{}/u_min".format(self.node_name))
        self.u_max = 1.0 * rospy.get_param("{}/u_max".format(self.node_name))
        self.v_min = 1.0 * rospy.get_param("{}/v_min".format(self.node_name))
        self.v_max = 1.0 * rospy.get_param("{}/v_max".format(self.node_name))
        self.k = 1.0 * rospy.get_param("{}/k".format(self.node_name))
        self.quadCorners3d = np.reshape(np.float32(rospy.get_param("{}/quadCorners3d/data".format(self.node_name))), (4, 3))
        
        self.set_desired_pose_mtx = Lock()
        self.sd = None
        self.Ld = None

        self.switch_mtx = Lock()
        self.ibvs_on = False

        rospy.Service('{}/set_desired_pose'.format(self.node_name), SetDesiredPose, self.set_desired_pose_handler)
        rospy.Service('{}/switch_on'.format(self.node_name), SwitchOn, self.ibvs_switch_handler)
        rospy.Subscriber('{}/corners'.format(vs_front_end), QuadCorners, self.corners_callback, queue_size=1)
        self.ibvs_state_pub = rospy.Publisher("{}/state".format(self.node_name), IbvsState, queue_size=1)
        return

    def spin(self):
        rospy.spin()
        return

    def corners_callback(self, corners):
        with self.switch_mtx:
            ibvs_on = self.ibvs_on
        if not ibvs_on:
            return
        with self.set_desired_pose_mtx:
            sd = self.sd
            Ld = self.Ld
        if sd is None or Ld is None:
            return
        pts_uv = np.reshape(np.float32(corners.corners.data), (4,2))
        s = self.undistort_and_transform(pts_uv)
        _, rvec, tvec = cv2.solvePnP(self.quadCorners3d.reshape(-1, 4, 3),
                                    s.reshape(-1, 4, 2),
                                    self.cameraMatrix, self.distCoeffs, flags=cv2.SOLVEPNP_EPNP)
        R_c2i, _ = cv2.Rodrigues(rvec)
        pts_c = (np.matmul(R_c2i, np.transpose(self.quadCorners3d)) + tvec).transpose()
        z = pts_c[:, 2]
        L0 = self.calc_Li(z[0], s[0])
        L1 = self.calc_Li(z[1], s[1])
        L2 = self.calc_Li(z[2], s[2])
        L3 = self.calc_Li(z[3], s[3])
        L = np.vstack((L0, L1, L2, L3))
        try:
            E0, e0, b0 = self.calc_Ei(s[0], sd[0])
            E1, e1, b1 = self.calc_Ei(s[1], sd[1])
            E2, e2, b2 = self.calc_Ei(s[2], sd[2])
            E3, e3, b3 = self.calc_Ei(s[3], sd[3])
        except AssertionError:
            rospy.logerr("Violate Bound")
            return
        E = np.hstack((E0, E1, E2, E3)).reshape(8,1)
        L_plus = 0.5 * np.linalg.pinv(L + Ld)
        #L_plus = np.linalg.pinv(Ld)
        V = -1.0 * self.k * np.matmul(L_plus, E).reshape(-1)
        msg = IbvsState()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = V[0]
        msg.twist.linear.y = V[1]
        msg.twist.linear.z = V[2]
        msg.twist.angular.x = V[3]
        msg.twist.angular.y = V[4]
        msg.twist.angular.z = V[5]
        msg.e.data = np.hstack((e0, e1, e2, e3)).tolist()
        msg.E.data = np.hstack((E0, E1, E2, E3)).tolist()
        msg.b.data = np.hstack((b0, b1, b2, b3)).tolist()
        self.ibvs_state_pub.publish(msg)
        return

    def calc_sd_Ld(self, p_w, q_w2c, t_w2c):
        q_c2w = quaternion_inverse(q_w2c)
        q = np.append(p_w - t_w2c, 0.0)
        x, y, z = quaternion_multiply(q_c2w, quaternion_multiply(q, q_w2c))[:3]
        u = self.f * x / z
        v = self.f * y / z
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
        e = np.float32([e_u, e_v])
        E = np.float32([E_u, E_v])
        b = np.float32([Mu_ub, -Mu_lb, Mv_ub, -Mv_lb])
        return E, e, b

    def set_desired_pose_handler(self, request):
        pose = request.pose
        q_w2c = np.float32([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        t_w2c = np.float32([pose.position.x, pose.position.y, pose.position.z])
        try:
            sd0, Ld0 = self.calc_sd_Ld(self.quadCorners3d[0], q_w2c, t_w2c)
            sd1, Ld1 = self.calc_sd_Ld(self.quadCorners3d[1], q_w2c, t_w2c)
            sd2, Ld2 = self.calc_sd_Ld(self.quadCorners3d[2], q_w2c, t_w2c)
            sd3, Ld3 = self.calc_sd_Ld(self.quadCorners3d[3], q_w2c, t_w2c)
        except AssertionError:
            rospy.logerr("Violate Bound")
            return SetDesiredPoseResponse(False)
        with self.set_desired_pose_mtx:
            self.sd = np.vstack((sd0, sd1, sd2, sd3))
            self.Ld = np.vstack((Ld0, Ld1, Ld2, Ld3))
        rospy.loginfo("sd: " + str(self.sd))
        return SetDesiredPoseResponse(True)

    def ibvs_switch_handler(self, request):
        with self.switch_mtx:
            self.ibvs_on = request.on
        rospy.loginfo("mode: " + str(self.ibvs_on))
        return SwitchOnResponse(True)


if __name__ == '__main__':
    ibvs_controller = IbvsController(argv[1], argv[2])
    ibvs_controller.spin()