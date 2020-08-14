#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import moveit_commander
import moveit_msgs.msg
'''
Author: Zqy
Date: 2020.07.14
Abstract: fk service, return 1
'''

class fk_server:
    def __init__(self):

        # initialise robot commander
        self.robot = moveit_commander.RobotCommander()
        # initialise scene planning interface
        self.scene = moveit_commander.PlanningSceneInterface()
        # initialise move group commander
        self.group = moveit_commander.MoveGroupCommander('panda_arm')

        s = rospy.Service('fk', fk, self.handle_fk_request)
        print("fk_server online...")
    
    def handle_fk_request(self, req):
        try:
            joint_position = req.position
            self.group.go(joint_position, wait=True)
            self.group.stop()
            print("handle over")
            return fkResponse(1)
        except rospy.ServiceException as e:
            print(e)
        

if __name__ == "__main__":
    rospy.init_node('fk_server', anonymous=True)
    fk_server = fk_server()
    rospy.spin()
