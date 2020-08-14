# PandaEnv Standard Workflow
## Pre-requirements
miniconda

libfranka

franka-ros

moveit

gym

[A Panda Setup Tutorial in Chminese simplify](https://www.jianshu.com/p/664b4cf76606) 

## Step1: Modify the bashrc file
`gedit ~/.bashrc`

Add in the bashrc:
```
source /home/dell/catkin_ws/devel/setup.bash
# some useful aliases
alias de='conda deactivate'
alias ac='conda activate base'
alias cv='export PYTHONPATH="/home/dell/miniconda3/lib/python3.7/site-packages:$PYTHONPATH"'
```
<font color='red'>**hint: change the '/home/dell' due to your own PC configuration**</font>

## Step2: Bringup the robot
For testing position mode in simulation:
```
roslaunch panda_moveit_config demo.launch
```

## Step3: Openup the services
```
de
roslaunch sawyer_control exp_nodes.launch
```

## Step4: Create the standard gym env
Copy the env files into gym.env
```
cp -r /home/dell/catkin_ws/src/sawyer-control/src/sawyer-control /home/dell/miniconda3/lib/python3.7/site-packages/gym/envs/robotics/
```
<font color='red'>**hint: change the '/home/dell' due to your own PC configuration, use 'pip show gym' to see the gym.env path**</font>

Modify the __init__.py in both robotics folder and gym.env folder:

an example of ../robotics/\__init\__.py
```python
from gym.envs.robotics.hand.manipulate_touch_sensors import HandEggTouchSensorsEnv
from gym.envs.robotics.hand.manipulate_touch_sensors import HandPenTouchSensorsEnv
# user add
from gym.envs.robotics.sawyer_control.sawyer_reaching import SawyerReachXYZEnv
```
../gym/envs/\__init\__.py
```python
...
# Robotics
# ----------------------------------------
# user add
register(
    id="PandaReach-v1",
    entry_point='gym.envs.robotics:SawyerReachXYZEnv',
    max_episode_steps=50,
)

def _merge(a, b):
    a.update(b)
    return a
...
```
Change the PYTHONHOME sys.path due to cv requests:
```
cv
```
Test the gym env:
```
python
import gym
env=gym.make('PandaReach-v1')
env.reset()
```




