#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs
'''
Author: Zqy
Date: 2020.07.14
Abstract: ik service, return 1
'''

class ik_server:
    def __init__(self):
        # initialise robot commander
        self.robot = moveit_commander.RobotCommander()
        # initialise scene planning interface
        self.scene = moveit_commander.PlanningSceneInterface()
        # initialise move group commander
        self.group = moveit_commander.MoveGroupCommander('panda_arm')

        s = rospy.Service('ik', ik, self.handle_ik_request)
        print("ik_server online...")
    
    def handle_ik_request(self, req):
        try:
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = req.pose[0]
            pose_target.position.y = req.pose[1]
            pose_target.position.z = req.pose[2]
            pose_target.orientation.x = req.pose[3]
            pose_target.orientation.y = req.pose[4]
            pose_target.orientation.z = req.pose[5]
            pose_target.orientation.w = req.pose[6]
            print("pose_target:\n {}".format(pose_target))
            self.group.set_pose_target(pose_target)
            self.group.go(wait=True)
            self.group.stop()
            print("ik handle over")
            return(1)
        except rospy.ServiceException as e:
            print(e)
        

if __name__ == "__main__":
    rospy.init_node('ik_server', anonymous=True)
    ik_server = ik_server()
    rospy.spin()
