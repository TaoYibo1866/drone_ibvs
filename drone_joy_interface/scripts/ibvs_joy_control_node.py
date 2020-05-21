#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from drone_comm.srv import SetDesiredPose, SetDesiredPoseRequest, SwitchOn, SwitchOnRequest

class IbvsJoyControl:
    def __init__(self):
        rospy.init_node("ibvs_joy_control")
        self.node_name = rospy.get_name()
        self.ibvs_on = False
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        rospy.wait_for_service("ibvs_controller/set_desired_pose", timeout=5)
        rospy.wait_for_service("ibvs_controller/switch_on", timeout=5)
        self.set_desired_pose = rospy.ServiceProxy('ibvs_controller/set_desired_pose', SetDesiredPose)
        self.switch_on = rospy.ServiceProxy('ibvs_controller/switch_on', SwitchOn)
        return
    def spin(self):
        rospy.spin()
        return
    def joy_callback(self, joy):
        joy_buttons = joy.buttons
        #joy_axes = joy.axes
        if joy_buttons[7] == 1:
            pose = Pose()
            pose.position.x = 0.0
            pose.position.y = -0.0
            pose.position.z = -0.4
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            if not self.set_desired_pose(SetDesiredPoseRequest(pose)):
                rospy.logerr("Set desired pose fail!")
        if joy_buttons[5] == 1:
            self.ibvs_on = not self.ibvs_on
            self.switch_on(SwitchOnRequest(self.ibvs_on))
        return

if __name__ =="__main__":
    ibvs_joy_control = IbvsJoyControl()
    ibvs_joy_control.spin()
