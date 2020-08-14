#!/usr/bin/env python
from sawyer_control.srv import ik
import rospy

def request_ik():
    rospy.wait_for_service('ik')
    try:
        execute_action = rospy.ServiceProxy('ik', ik, persistent=True)
        target_pose = [0.307,-1.0e-05, 0.59, 0.924,-0.383, 3.0e-05, 1.0e-05]
        obs = execute_action(target_pose)
        return obs
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    request_ik()