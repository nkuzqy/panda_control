import numpy as np
from gym.spaces import Box
from configs.ros_config import *
from math import pi

#SPACES
JOINT_ANGLES_HIGH = np.array([
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973
])

JOINT_ANGLES_LOW = np.array([
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973
])

JOINT_VEL_HIGH = np.array([
    2.1750, 	
    2.1750, 	
    2.1750, 	
    2.1750, 	
    2.6100, 	
    2.6100, 	
    2.6100
])
JOINT_VEL_LOW = np.array([
    -2.1750, 	
    -2.1750, 	
    -2.1750, 	
    -2.1750, 	
    -2.6100, 	
    -2.6100, 	
    -2.6100
])

MAX_TORQUES = np.array([87,87,87,87,12,12,12])

JOINT_TORQUE_HIGH = MAX_TORQUES
JOINT_TORQUE_LOW = -1*MAX_TORQUES

JOINT_VALUE_HIGH = {
    'position': JOINT_ANGLES_HIGH,
    'velocity': JOINT_VEL_HIGH,
    'torque': JOINT_TORQUE_HIGH,
}
JOINT_VALUE_LOW = {
    'position': JOINT_ANGLES_LOW,
    'velocity': JOINT_VEL_LOW,
    'torque': JOINT_TORQUE_LOW,
}
# 由仿真测试得到，为保证安全，假设机械臂仅在x>0空间内运动
END_EFFECTOR_POS_LOW = np.array([0.1,-0.6,0.05])
END_EFFECTOR_POS_HIGH = np.array([0.6,0.6,1.0])

END_EFFECTOR_ANGLE_LOW = -1*np.ones(4)
END_EFFECTOR_ANGLE_HIGH = np.ones(4)

END_EFFECTOR_VALUE_LOW = {
    'position': END_EFFECTOR_POS_LOW,
    'angle': END_EFFECTOR_ANGLE_LOW,
}

END_EFFECTOR_VALUE_HIGH = {
    'position': END_EFFECTOR_POS_HIGH,
    'angle': END_EFFECTOR_ANGLE_HIGH,
}

POSITION_CONTROL_LOW = -1*np.ones(3)
POSITION_CONTROL_HIGH = np.ones(3)

RESET_TORQUE_LOW = -5
RESET_TORQUE_HIGH = 5

#SAFETY BOX SETTINGS
SAFETY_FORCE_MAGNITUDE = 5
SAFETY_FORCE_TEMPERATURE = 5

# overwrite these for your setup
RESET_SAFETY_BOX_LOWS = np.array([-.2, -0.6, 0.2])
RESET_SAFETY_BOX_HIGHS = np.array([.9, 0.4, 1])
RESET_SAFETY_BOX = Box(RESET_SAFETY_BOX_LOWS, RESET_SAFETY_BOX_HIGHS, dtype=np.float32)

TORQUE_SAFETY_BOX_LOWS = np.array([0.3, -0.4, 0.2])
TORQUE_SAFETY_BOX_HIGHS = np.array([0.7, 0.4, 0.7])
TORQUE_SAFETY_BOX = Box(TORQUE_SAFETY_BOX_LOWS, TORQUE_SAFETY_BOX_HIGHS, dtype=np.float32)
# NEED TO BE CHECKED
POSITION_SAFETY_BOX_LOWS = np.array([0.1,-0.6,0.05])
POSITION_SAFETY_BOX_HIGHS = np.array([0.6,0.6,1.0])
POSITION_SAFETY_BOX = Box(POSITION_SAFETY_BOX_LOWS, POSITION_SAFETY_BOX_HIGHS, dtype=np.float32)
# 位置模式下单步位移限幅
POSITION_SAFETY_DELTA_HIGHS = np.array([0.01,0.01,0.01])
POSITION_SAFETY_DELTA_LOWS = np.array([-0.01,-0.01,-0.01])
# 数据由panda仿真测试得到
POSITION_RESET_POS = np.array([0.307, -2.35e-05,  0.59])
POSITION_RESET_ANGLES = np.array([0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4])
# 目标位置分布区域
TARGET_POS_LOWS = np.array([0.1,-0.6,0.05])
TARGET_POS_HIGHS = np.array([0.6,0.6,1.0])

#MISCELLANEOUS
RESET_LENGTH = 200
RESET_ERROR_THRESHOLD = .15*np.ones(7)
UPDATE_HZ = 20

#JOINT_CONTROLLER_SETTINGS
JOINT_POSITION_SPEED = .1
JOINT_POSITION_TIMEOUT = .5
