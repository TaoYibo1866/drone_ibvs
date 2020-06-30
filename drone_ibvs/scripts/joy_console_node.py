#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from drone_ibvs.srv import SwitchMode, SwitchModeRequest

class JoyConsole:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.ibvs_on = False
        rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.mission_switch_mode = rospy.ServiceProxy("mission/mode", SwitchMode)
        self.mission_switch_mode.wait_for_service()
        return
    def joy_callback(self, joy):
        joy_buttons = joy.buttons
        #joy_axes = joy.axes
        if joy_buttons[5] == 1:
            self.ibvs_on = not self.ibvs_on
            if self.ibvs_on:
                self.mission_switch_mode(SwitchModeRequest(1))
            else:
                self.mission_switch_mode(SwitchModeRequest(0))
        return

if __name__ =="__main__":
    rospy.init_node("joy_console")
    joy_console = JoyConsole()
    rospy.spin()
