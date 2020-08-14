import gym
env=gym.make('PandaReach-v1')
import numpy as np
obs=env.reset()
for step in range(40):
    action = np.random.rand(3)
    obs, reward, done, _ = env.step(action)
    print("step {}: action={}, observation={} => reward = {}, done = {}".format(step, action, obs, reward, done))

