#!/usr/bin/env python2.7

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

if __name__ == "__main__":
    rospy.init_node("moving_gate")
    set_model_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState, True)
    set_model_client.wait_for_service()
    request = SetModelStateRequest()
    request.model_state.model_name = "square_gate"
    request.model_state.pose.position.x = -3
    request.model_state.pose.position.y = 15
    request.model_state.pose.position.z = 0

    t = 0
    sign = 1
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        request.model_state.pose.position.x = request.model_state.pose.position.x + sign * 0.5 / 100
        if sign == 1 and request.model_state.pose.position.x > 3:
            sign = -1
        if sign == -1 and request.model_state.pose.position.x < -3:
            sign = 1
        set_model_client(request)
        t = t + 0.01
        r.sleep()
