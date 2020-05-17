#!/usr/bin/env python2.7
from sys import argv
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from ibvs_control.msg import Velocity, IbvsState

from dynamic_reconfigure.server import Server
from ibvs_control.cfg import IbvsControllerConfig

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from tf.transformations import quaternion_inverse, quaternion_multiply

from threading import Lock

class IbvsController:
    def __init__(self, gimbal_camera_name):
        rospy.init_node('ibvs_controller')
        self.node_name = rospy.get_name()
        self.w = 1.0 * rospy.get_param("{}/w".format(self.node_name))
        self.h = 1.0 * rospy.get_param("{}/h".format(self.node_name))
        self.f = 1.0 * rospy.get_param("{}/f".format(self.node_name))
        self.u_min = 1.0 * rospy.get_param("{}/u_min".format(self.node_name))
        self.u_max = 1.0 * rospy.get_param("{}/u_max".format(self.node_name))
        self.v_min = 1.0 * rospy.get_param("{}/v_min".format(self.node_name))
        self.v_max = 1.0 * rospy.get_param("{}/v_max".format(self.node_name))
        self.k = 1.0 * rospy.get_param("{}/k".format(self.node_name))
        self.cameraMatrix = np.asarray(rospy.get_param("{}/cameraMatrix/data".format(self.node_name)), dtype=np.float32).reshape(3, 3)
        self.distCoeffs = np.asarray(rospy.get_param("{}/distCoeffs/data".format(self.node_name)), dtype=np.float32).reshape(1, 4)
        self.pts_m = np.asarray(rospy.get_param("{}/pts_m/data".format(self.node_name)), dtype=np.float32).reshape(4, 3)
        self.bridge = CvBridge()
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        self.marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.command_mtx = Lock()
        self.command_ready = False
        self.sd = None
        self.Ld = None
        self.ibvs_switch_mtx = Lock()
        self.ibvs_switch = False
        _ = Server(IbvsControllerConfig, self.config_callback)
        rospy.Subscriber("{}/camera/image_raw".format(gimbal_camera_name), Image, self.image_callback)
        rospy.Subscriber("{}/command".format(self.node_name), Pose, self.command_callback)
        self.velocity_pub = rospy.Publisher('{}/velocity'.format(self.node_name), Velocity, queue_size=1)
        self.state_pub = rospy.Publisher('{}/state'.format(self.node_name), IbvsState, queue_size=1)
        return
    def config_callback(self, config, level):
        with self.ibvs_switch_mtx:
            self.ibvs_switch = config.ibvs_switch
        return config
    def spin(self):
        rospy.spin()
        return
    def model2pixel(self, pt_m, q_m2c, t_m2c, f):
        q_c2m = quaternion_inverse(q_m2c)
        q = [pt_m[0] - t_m2c[0], pt_m[1] - t_m2c[1], pt_m[2] - t_m2c[2], 0.0]
        x, y, z = quaternion_multiply(q_c2m, quaternion_multiply(q, q_m2c))[:3]
        u = 1.0 * f * x / z
        v = 1.0 * f * y / z
        assert u > self.u_min and u < self.u_max
        assert v > self.v_min and v < self.v_max
        return np.asarray([u, v], dtype=np.float32), self.calc_Li(z, [u, v])
    def command_callback(self, pose):
        #pose {m} to {c}
        q_m2c = np.asarray([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=np.float32)
        t_m2c = np.asarray([pose.position.x, pose.position.y, pose.position.z], dtype=np.float32)
        try:
            sd0, Ld0 = self.model2pixel(self.pts_m[0], q_m2c, t_m2c, self.f)
            sd1, Ld1 = self.model2pixel(self.pts_m[1], q_m2c, t_m2c, self.f)
            sd2, Ld2 = self.model2pixel(self.pts_m[2], q_m2c, t_m2c, self.f)
            sd3, Ld3 = self.model2pixel(self.pts_m[3], q_m2c, t_m2c, self.f)
        except AssertionError:
            rospy.logerr("Violate Bound")
            return
        with self.command_mtx:
            if self.command_ready == False:
                self.command_ready = True
            self.sd = np.vstack((sd0, sd1, sd2, sd3))
            self.Ld = np.vstack((Ld0, Ld1, Ld2, Ld3))
            rospy.loginfo(self.sd)
        return
    def image_callback(self, image_raw):
        with self.command_mtx:
            command_ready = self.command_ready
            sd = self.sd
            Ld = self.Ld
        with self.ibvs_switch_mtx:
            ibvs_switch = self.ibvs_switch
        try:
            img = self.bridge.imgmsg_to_cv2(image_raw, desired_encoding='bgr8')
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.marker_dict, parameters=self.detector_params)
        found = (ids is not None) and (23 in ids)
        if found and command_ready:
            if ibvs_switch == False:
                velocity = Velocity()
                velocity.header.stamp = rospy.Time.now()
                velocity.switch.data = False
                self.velocity_pub.publish(velocity)
            else:
                index = np.where(ids == 23)[0][0]
                pts_uv = corners[index].reshape(4,2)
                self.generate_outerloop_command(sd, Ld, pts_uv)
        img = cv2.aruco.drawDetectedMarkers(img, corners, ids=ids, borderColor=(0, 0, 255))
        img = cv2.resize(img, None, fx=0.8, fy=0.8)
        img = cv2.putText(img, "IBVS: {}".format(ibvs_switch), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), thickness=2)
        cv2.imshow("image_raw", img)
        cv2.waitKey(10)
        return
    def generate_outerloop_command(self, sd, Ld, pts_uv):
        s = self.undistort_and_transform(pts_uv)
        _, rvec, tvec = cv2.solvePnP(self.pts_m.reshape(-1, 4, 3),
                                    s.reshape(-1, 4, 2),
                                    self.cameraMatrix, self.distCoeffs, flags=cv2.SOLVEPNP_EPNP)
        R_c2i, _ = cv2.Rodrigues(rvec)
        pts_c = (np.matmul(R_c2i, np.transpose(self.pts_m)) + tvec).transpose()
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
        E = np.vstack((E0, E1, E2, E3)).reshape(8,1)
        L_plus = 0.5 * np.linalg.pinv(L + Ld)
        #L_plus = np.linalg.pinv(Ld)
        V = -1.0 * self.k * np.matmul(L_plus, E).reshape(-1)
        #rospy.loginfo(V)
        velocity = Velocity()
        velocity.header.stamp = rospy.Time.now()
        velocity.twist.linear.x = V[0]
        velocity.twist.linear.y = V[1]
        velocity.twist.linear.z = V[2]
        velocity.twist.angular.x = V[3]
        velocity.twist.angular.y = V[4]
        velocity.twist.angular.z = V[5]
        velocity.switch.data = True
        self.velocity_pub.publish(velocity)

        e = np.hstack((e0, e1, e2, e3)).tolist()
        E = E.reshape(-1).tolist()
        b = np.hstack((b0, b1, b2, b3)).tolist()
        state = IbvsState()
        state.header.stamp = rospy.Time.now()
        state.e.data = e
        state.E.data = E
        state.b.data = b
        self.state_pub.publish(state)
        return
    def undistort_and_transform(self, pts_uv):
        # undistort
        # ...
        #transform
        s = pts_uv - np.asarray([self.w / 2.0, self.h / 2.0])
        return s
    def calc_Li(self, zi, si):
        u = 1.0 * si[0]
        v = 1.0 * si[1]
        z = 1.0 * zi
        f = 1.0 * self.f
        return np.asarray([[-f / z, 0, u / z, u * v / f, -(f * f + u * u) / f, v],
                           [0, -f / z, v / z, (f * f + v * v) / f, -u * v / f, -u]], dtype=np.float32)
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
        e = np.asarray([e_u, e_v], dtype=np.float32)
        E = np.asarray([E_u, E_v], dtype=np.float32).reshape(2, 1)
        b = np.asarray([Mu_ub, -Mu_lb, Mv_ub, -Mv_lb], dtype=np.float32)
        return E, e, b
if __name__ == '__main__':
    ibvs_controller = IbvsController(argv[1])
    ibvs_controller.spin()
