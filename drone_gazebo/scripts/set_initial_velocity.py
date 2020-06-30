#!/usr/bin/env python2.7

from sys import argv
import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, GetModelState, GetModelStateRequest

if __name__ == "__main__":
    rospy.init_node("set_initial_velocity")
    get_state_client = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState, True)
    set_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState, True)
    get_state_client.wait_for_service()
    set_state_client.wait_for_service()

    rospy.sleep(0)
    state = get_state_client(GetModelStateRequest(model_name=argv[1]))

    request = SetModelStateRequest()
    request.model_state.model_name = argv[1]
    request.model_state.pose = state.pose
    request.model_state.twist.linear.x = float(argv[2])
    request.model_state.twist.linear.y = float(argv[3])
    request.model_state.twist.linear.z = float(argv[4])

    set_state_client(request)