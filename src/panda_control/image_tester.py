import sys
sys.path.append('/home/dell/miniconda3/lib/python3.7/site-packages/gym/envs/robotics/panda_control')
from panda_reaching import PandaReachXYZEnv
import cv2
from image_env import ImageEnv

env = ImageEnv(SawyerReachXYZEnv())
img = env.get_image(width=84, height=84)
cv2.imwrite("test.png", img)
