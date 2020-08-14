#!/usr/bin/python3
import numpy as np
import rospy
import gym
from gym.spaces import Box
import sys
from core.serializable import Serializable
from core.multitask_env import MultitaskEnv
from configs.config import config_dict as config
from panda_control.srv import observation
from panda_control.srv import ik
from panda_control.srv import fk
from panda_control.srv import image
import abc
import cv2
import copy
import math

class PandaEnvBase(gym.Env, Serializable, MultitaskEnv, metaclass=abc.ABCMeta):
    def __init__(
            self,
            action_mode='position',
            use_safety_box=True,
            torque_action_scale=1,
            position_action_scale=1/10,
            config_name = 'base_config',
            fix_goal=False,
            max_speed = 0.05,
            reset_free=False,
            img_start_col=350, #can range from  0-999
            img_start_row=200, #can range from  0-999
            img_col_delta=300, #can range from  0-999
            img_row_delta=600, #can range from  0-999
        ):
        
        Serializable.quick_init(self, locals())
        MultitaskEnv.__init__(self)
        self.config = config[config_name]
        self.init_rospy(self.config.UPDATE_HZ)
        self.action_mode = action_mode
        self.max_speed = max_speed

        self.use_safety_box = use_safety_box
        # effort only
        #self.AnglePDController = AnglePDController(config=self.config)

        self._set_action_space()
        self._set_observation_space()
        # effort only
        # self.get_latest_pose_jacobian_dict()

        self.torque_action_scale = torque_action_scale
        self.position_action_scale = position_action_scale
        # effort only
        #self.in_reset = True
        self._state_goal = None
        self.fix_goal = fix_goal

        self.pos_control_reset_angles = self.config.POSITION_RESET_ANGLES
        self.reset_free = reset_free

        self.goal_space_low = self.config.TARGET_POS_LOWS
        self.goal_space_high = self.config.TARGET_POS_HIGHS
        self.goal_space = Box(self.goal_space_low, self.goal_space_high, dtype = np.float32)

        self.img_start_col = img_start_col
        self.img_start_row = img_start_row
        self.img_col_delta = img_col_delta
        self.img_row_delta = img_row_delta
        

    # adjusted, delete the torque mode
    def _act(self, action):
        if self.action_mode == 'position':
            self._position_act(action * self.position_action_scale)
        return

    # adjusted, request_ik instead
    def _position_act(self, action):
        action = np.clip(action, self.config.POSITION_SAFETY_DELTA_LOWS, self.config.POSITION_SAFETY_DELTA_HIGHS)
        ee_pos = self._get_endeffector_pose()
        endeffector_pos = ee_pos[:3]
        target_ee_pos = (endeffector_pos + action)
        target_ee_pos = np.clip(target_ee_pos, self.config.POSITION_SAFETY_BOX_LOWS, self.config.POSITION_SAFETY_BOX_HIGHS)
        target_ee_pos = np.concatenate((target_ee_pos, [self.config.POSITION_CONTROL_EE_ORIENTATION.x, self.config.POSITION_CONTROL_EE_ORIENTATION.y, self.config.POSITION_CONTROL_EE_ORIENTATION.z, self.config.POSITION_CONTROL_EE_ORIENTATION.w]))
        self.request_ik(list(target_ee_pos)) 

    # added for fk and robot reset(incremental)
    def _joint_act(self, action):
        joint_angles = self._get_joint_angles()
        target_joint_angles = (joint_angles + action)
        target_joint_angles = np.clip(target_joint_angles, self.config.JOINT_ANGLES_LOW, self.config.JOINT_VEL_HIGH)
        self.request_fk(list(target_joint_angles))


    def _torque_act(self, action):
        pass

    def _wrap_angles(self, angles):
        return angles % (2*np.pi)

    def _get_joint_angles(self):
        angles, _, _, _= self.request_observation()
        return angles[:7]

    # adjusted
    def _get_endeffector_pose(self):
        _, _, _, endpoint_pose = self.request_observation()
        return endpoint_pose[:3]


    def step(self, action):
        self._act(action)
        observation = self._get_obs()
        reward = self.compute_reward(action, self.convert_ob_to_goal(observation), self._state_goal)
        info = self._get_info()
        done = False
        return observation, reward, done, info
    
    # adjusted, joint angles radius unit
    def _get_obs(self):
        angles, velocities, _, endpoint_pose = self.request_observation()
        # hstack to vstack
        obs = np.vstack((
            self._wrap_angles(angles[:7]),
            #velocities,
            endpoint_pose,
        ))
        return obs

    @abc.abstractmethod
    def compute_rewards(self, actions, obs, goals):
        pass
    
    def _get_info(self):
        return dict()
    

    def _reset_robot(self):
        if not self.reset_free:
            if self.action_mode == "position":
                
                for _ in range(1):
                    self._joint_act(self.pos_control_reset_angles - self._get_joint_angles())

    def reset(self):
        self._reset_robot()
        self._state_goal = self.sample_goal()
        return self._get_obs()

    def _pose_in_box(self, pose, safety_box):
        within_box = safety_box.contains(pose)
        return within_box

    def _compute_joint_distance_outside_box(self, pose, safety_box):
        curr_x = pose[0]
        curr_y = pose[1]
        curr_z = pose[2]
        if(self._pose_in_box(pose, safety_box)):
            x, y, z = 0, 0, 0
        else:
            x, y, z = 0, 0, 0
            if curr_x > safety_box.high[0]:
                x = np.abs(curr_x - safety_box.high[0])
            elif curr_x < safety_box.low[0]:
                x = np.abs(curr_x - safety_box.low[0])
            if curr_y > safety_box.high[1]:
                y = np.abs(curr_y - safety_box.high[1])
            elif curr_y < safety_box.low[1]:
                y = np.abs(curr_y - safety_box.low[1])
            if curr_z > safety_box.high[2]:
                z = np.abs(curr_z - safety_box.high[2])
            elif curr_z < safety_box.low[2]:
                z = np.abs(curr_z - safety_box.low[2])
        return np.linalg.norm([x, y, z])

    @abc.abstractmethod
    def get_diagnostics(self, paths, prefix=''):
        pass

    def _set_action_space(self):
        if self.action_mode == 'position':
            self.action_space = Box(
                self.config.POSITION_CONTROL_LOW,
                self.config.POSITION_CONTROL_HIGH,
                dtype=np.float32,
            )
        else:
            self.action_space = Box(
                self.config.JOINT_TORQUE_LOW,
                self.config.JOINT_TORQUE_HIGH,
                dtype=np.float32,
            )

    def _set_observation_space(self):
        lows = np.vstack((
            self.config.JOINT_VALUE_LOW['position'],
            self.config.JOINT_VALUE_LOW['velocity'],
            self.config.END_EFFECTOR_VALUE_LOW['position'],
            self.config.END_EFFECTOR_VALUE_LOW['angle'],
        ))
        highs = np.vstack((
            self.config.JOINT_VALUE_HIGH['position'],
            self.config.JOINT_VALUE_HIGH['velocity'],
            self.config.END_EFFECTOR_VALUE_HIGH['position'],
            self.config.END_EFFECTOR_VALUE_HIGH['angle'],
        ))
        self.observation_space = Box(
            lows,
            highs,
            dtype=np.float32,
        )
            
    """ 
    ROS Functions 
    """

    def init_rospy(self, update_hz):
        rospy.init_node('panda_env', anonymous=True)
        self.rate = rospy.Rate(update_hz)
    
    def send_angle_action(self, action):
        self.request_fk(action)

    def request_image(self):
        rospy.wait_for_service('images')
        try:
            request = rospy.ServiceProxy('images', image, persistent=True)
            obs = request()
            return (
                    obs.image
            )
        except rospy.ServiceException as e:
            print(e)

    def crop_image(self, img):
        endcol = self.img_start_col + self.img_col_delta
        endrow = self.img_start_row + self.img_row_delta
        img = copy.deepcopy(img[self.img_start_row:endrow, self.img_start_col:endcol])
        return img

    def get_image(self, width=84, height=84):
        image = self.request_image()
        if image is None:
            raise Exception('Unable to get image from image server')
        image = np.array(image).reshape(1000, 1000, 3)
        image = copy.deepcopy(image)
        image = cv2.resize(image, (0, 0), fx=width/1000, fy=height/1000, interpolation=cv2.INTER_AREA)
        image = np.asarray(image).reshape(width, height, 3)
        return image

    # adjusted
    def request_observation(self):
        rospy.wait_for_service('get_observation')
        try:
            request = rospy.ServiceProxy('get_observation', observation, persistent=True)
            obs = request()
            return (
                    np.array(obs.position),
                    np.array(obs.velocity),
                    np.array(obs.effort),
                    np.array(obs.end_effector_pose)
            )
        except rospy.ServiceException as e:
            print(e)

    def request_fk(self, angles):
        rospy.wait_for_service('fk')
        try:
            execute_action = rospy.ServiceProxy('fk', fk, persistent=True)
            obs = execute_action(angles)
            return obs
        except rospy.ServiceException as e:
            print(e)

    # request_ik_angles adjusted to request_ik    
    def request_ik(self, ee_pos):
        rospy.wait_for_service('ik')
        try:
            request = rospy.ServiceProxy('ik', ik, persistent=True)
            resp = request(ee_pos)
            return (resp)
        except rospy.ServiceException as e:
            print(e)

    """
    Multitask functions
    """

    @property
    def goal_dim(self):
        raise NotImplementedError()

    def get_goal(self):
        return self._state_goal

    def set_goal(self, goal):
        self._state_goal = goal

    def sample_goals(self, batch_size):
        if self.fix_goal:
            goals = np.repeat(
                self._state_goal.copy()[None],
                batch_size,
                0
            )
        else:
            goals = np.random.uniform(
                self.goal_space.low,
                self.goal_space.high,
                size=(batch_size, self.goal_space.low.size),
            )
        return goals

    @abc.abstractmethod
    def set_to_goal(self, goal):
        pass

    """
    Image Env Functions
    """

    def get_env_state(self):
        return self._get_joint_angles(), self._get_endeffector_pose()

    def set_env_state(self, env_state):
        angles, ee_pos = env_state
        for _ in range(3):
            self.send_angle_action(angles)

    def initialize_camera(self, init_fctn):
        pass
