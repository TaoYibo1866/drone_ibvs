#!/usr/bin/env python2.7

import roslib
roslib.load_manifest("drone_control")
import rospy
import actionlib
from nav_msgs.msg import Odometry
from drone_control.msg import GoToPointAction, GoToPointGoal

def feedback_cb(feedback):
    rospy.loginfo(feedback)
    return

if __name__ == "__main__":
    rospy.init_node("mission")
    pos_ac = actionlib.SimpleActionClient("position_controller/go_to_point", GoToPointAction)
    pos_ac.wait_for_server(rospy.Duration(secs=3))
    rospy.sleep(2)
    # TODO add success criteria to goal
    goal = GoToPointGoal()
    goal.desired_position.x = 0
    goal.desired_position.y = 0
    goal.desired_position.z = -4

    pos_ac.send_goal(goal, feedback_cb=feedback_cb)
    pos_ac.wait_for_result()
    result = pos_ac.get_result()
    if result.errno == 0:
        rospy.loginfo("success")