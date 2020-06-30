#!/usr/bin/env python2.7

from sys import argv

import roslib
roslib.load_manifest('drone_ibvs')
import rospy
import actionlib

from drone_ibvs.msg import DoIBVSAction, DoIBVSGoal
from drone_ibvs.srv import SwitchMode, SwitchModeResponse
from drone_control.msg import GoToPointAction, GoToPointGoal

from threading import Lock

class IBVSJoyMission:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.mode = 0
        rospy.Service("{}/mode".format(self.node_name), SwitchMode, self.mode_handler)
        self.do_ibvs_ac = actionlib.SimpleActionClient("ibvs_controller/do_ibvs", DoIBVSAction)
    def do_ibvs_feedback_cb(self, feedback):
        rospy.loginfo(feedback)
        return
    def do_ibvs_done_cb(self, status, result):
        self.do_ibvs_ac.cancel_all_goals()
        errno = result.errno
        rospy.loginfo("status: {}, errno: {}".format(status, errno))
        if errno == 0:
            rospy.loginfo("Success!")
        elif errno == 1:
            rospy.loginfo("Abort! Violate Bound!")
        elif errno == 2:
            ## treat as exception, mode specified by mode handler 
            rospy.loginfo("Abort! Cancel by Client!")
        return
    def mode_handler(self, request):
        if self.mode == 1 and request.mode == 0:
            self.mode = 0
            self.do_ibvs_ac.cancel_all_goals()
        elif self.mode == 0 and request.mode == 1:
            self.mode = 1
        return SwitchModeResponse()
    def start(self):
        self.do_ibvs_ac.wait_for_server()
        while True:
            if self.mode == 0:
                rospy.sleep(0.1)
                continue
            elif self.mode == 1:
                goal = DoIBVSGoal()
                goal.desired_pose.position.z = -0.4
                goal.desired_pose.orientation.w = 1
                self.do_ibvs_ac.send_goal(goal, feedback_cb=self.do_ibvs_feedback_cb, done_cb=self.do_ibvs_done_cb)
                rospy.loginfo("Send Goal!")
                self.do_ibvs_ac.wait_for_result()
                result = self.do_ibvs_ac.get_result()
                if result.errno == 0:
                    self.mode = 0
                elif result.errno != 2:
                    self.mode = 0
            continue

class IBVSMission:
    def __init__(self):
        self.node_name = rospy.get_name()
        self.mode = 3
        rospy.Service("{}/mode".format(self.node_name), SwitchMode, self.mode_handler)
        self.do_ibvs_ac = actionlib.SimpleActionClient("ibvs_controller/do_ibvs", DoIBVSAction)
        self.pos_ac = actionlib.SimpleActionClient("position_controller/go_to_point", GoToPointAction)
    def do_ibvs_feedback_cb(self, feedback):
        rospy.loginfo(feedback)
        return
    def do_ibvs_done_cb(self, status, result):
        self.do_ibvs_ac.cancel_all_goals()
        errno = result.errno
        rospy.loginfo("status: {}, errno: {}".format(status, errno))
        if errno == 0:
            rospy.loginfo("Success!")
        elif errno == 1:
            rospy.loginfo("Abort! Violate Bound!")
        elif errno == 2:
            ## treat as exception, mode specified by mode handler 
            rospy.loginfo("Abort! Cancel by Client!")
        return
    def go_to_point_done_cb(self, status, result):
        self.pos_ac.cancel_all_goals()
        errno = result.errno
        rospy.loginfo("status: {}, errno: {}".format(status, errno))
        if errno == 0:
            rospy.loginfo("Success!")
        return
    def mode_handler(self, request):
        self.do_ibvs_ac.cancel_all_goals()
        return SwitchModeResponse()
    def start(self):
        self.do_ibvs_ac.wait_for_server()
        self.pos_ac.wait_for_server()
        while True:
            if self.mode == 0:
                rospy.sleep(0.1)
                continue
            elif self.mode == 1:
                goal = DoIBVSGoal()
                goal.desired_pose.position.z = -0.4
                goal.desired_pose.orientation.w = 1
                self.do_ibvs_ac.send_goal(goal, feedback_cb=self.do_ibvs_feedback_cb, done_cb=self.do_ibvs_done_cb)
                rospy.loginfo("Send Goal!")
                self.do_ibvs_ac.wait_for_result()
                result = self.do_ibvs_ac.get_result()
                if result.errno == 0:
                    self.mode = 2
                elif result.errno != 2:
                    self.mode = 2
            elif self.mode == 2:
                goal = GoToPointGoal()
                goal.desired_position.x = 3
                goal.desired_position.y = 3
                goal.desired_position.z = -2
                self.pos_ac.send_goal(goal, done_cb=self.go_to_point_done_cb)
                self.pos_ac.wait_for_result()
                result = self.pos_ac.get_result()
                if result.errno == 0:
                    self.mode = 4
            elif self.mode == 3:
                goal = GoToPointGoal()
                goal.desired_position.x = 1
                goal.desired_position.y = -1
                goal.desired_position.z = -4
                self.pos_ac.send_goal(goal, done_cb=self.go_to_point_done_cb)
                self.pos_ac.wait_for_result()
                result = self.pos_ac.get_result()
                if result.errno == 0:
                    self.mode = 1
            elif self.mode == 4:
                goal = GoToPointGoal()
                goal.desired_position.x = 3
                goal.desired_position.y = 3
                goal.desired_position.z = 0
                self.pos_ac.send_goal(goal, done_cb=self.go_to_point_done_cb)
                self.pos_ac.wait_for_result()
                result = self.pos_ac.get_result()
                if result.errno == 0:
                    self.mode = 0
            continue

if __name__ == "__main__":
    rospy.init_node("mission")
    rospy.sleep(3)
    if argv[1] == "ibvs_joy_mission":
        mission = IBVSJoyMission()
        mission.start()
    elif argv[1] == "ibvs_mission":
        mission = IBVSMission()
        mission.start()