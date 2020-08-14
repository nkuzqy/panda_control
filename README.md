# PandaEnv Standard Workflow
## Pre-requirements
miniconda

libfranka

franka-ros

moveit

gym

[A Panda Setup Tutorial in Chminese simplify](https://www.jianshu.com/p/664b4cf76606) 

One Good Reference: [sawyer-control](https://github.com/mihdalal/sawyer_control)

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
<font color='red'>**trick: to reduce the render load, try not to use the rviz:**</font>

add the following code in demo.lauch:
```xml
<!--add a arg named rviz_on-->
<arg name="rviz_on" default="false"/>
<!-- Run Rviz -->
<!--add the rviz_on arg after if -->
  <include file="$(find panda_moveit_config)/launch/moveit_rviz.launch" if="$(arg rviz_on)">
    <arg name="rviz_tutorial" value="$(arg rviz_tutorial)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>
```

assign the rviz_on value to enable the rviz window:
```
roslaunch panda_moveit_config demo.launch rviz_on:=True
```

## Step3: Openup the services
```
de
roslaunch sawyer_control exp_nodes.launch
```

## Step4: Create the standard gym env
Copy the env files into gym.env
```
cp -r /panda-control/src/panda-control /home/dell/miniconda3/lib/python3.7/site-packages/gym/envs/robotics/
```
in panda_reaching.py:
```
import sys
sys.path.append('/home/dell/miniconda3/lib/python3.7/site-packages/gym/envs/robotics/panda_control')
```
<font color='red'>**hint: change the '/home/dell' due to your own PC configuration, use 'pip show gym' to see the gym.env path.**</font>

<font color='red'>**Use the sys.path.append line if "no module named 'panda_env_base'" error comes up**</font>

Modify the __init__.py in both robotics folder and gym.env folder:

an example of ../robotics/\__init\__.py
```python
from gym.envs.robotics.hand.manipulate_touch_sensors import HandEggTouchSensorsEnv
from gym.envs.robotics.hand.manipulate_touch_sensors import HandPenTouchSensorsEnv
# user add
from gym.envs.robotics.panda_control.panda_reaching import PandaReachXYZEnv
```
../gym/envs/\__init\__.py
```python
...
# Robotics
# ----------------------------------------
# user add
register(
    id="PandaReach-v1",
    entry_point='gym.envs.robotics:PandaReachXYZEnv',
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




