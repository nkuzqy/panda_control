#!/usr/bin/env python
from sawyer_control.srv import *
import rospy
from sensor_msgs.msg import JointState
import sys
import moveit_commander
import moveit_msgs.msg


'''
Author: Zqy
Date: 2020.07.14
Abstract: get_observation service, return panda_arm position, velocity & effort
'''
class observation_server:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.position = []
        self.velocity = []
        self.effort = []
        self.end_effector_pose = []
        
        self.robot = moveit_commander.RobotCommander()
        self.group_name = "panda_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.eef_link = self.group.get_end_effector_link()

        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)
        s = rospy.Service('get_observation', observation, self.handle_ob_request)
        print("observation_server online...")
    def joint_states_cb(self, msg):
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        
        

    def handle_ob_request(self, req):
        pose = self.group.get_current_pose(self.eef_link).pose
        self.end_effector_pose = []
        self.end_effector_pose.extend([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        print("observation handle over")
        return observationResponse(self.position, self.velocity, self.effort, self.end_effector_pose)
        

if __name__ == "__main__":
    rospy.init_node('observation_server', anonymous=True)
    ob_server = observation_server()
    rospy.spin()
