#!/usr/bin/env python

import rospy
from franka_env.srv import *
import numpy as np
'''
Author: Zqy
Date: 2020.07.14
Abstract: get_observation service test script
'''

def request_observation():
    rospy.wait_for_service('get_observation')
    try:
        execute_action = rospy.ServiceProxy('get_observation', observation, persistent=True)
        obs = execute_action()
        return (
                    np.array(obs.position),
                    np.array(obs.velocity),
                    np.array(obs.effort),
                    np.array(obs.end_effector_pose)
            )
    except rospy.ServiceException as e:
        print(e)

if __name__ == "__main__":
    position, velocity, effort, end_effector_pose  = request_observation()
    print("position:\n {}".format(position))
    print("velocity:\n {}".format(velocity))
    print("effort:\n {}".format(effort))
    print("effortend_effector_pose:\n {}".format(end_effector_pose))
    
    
