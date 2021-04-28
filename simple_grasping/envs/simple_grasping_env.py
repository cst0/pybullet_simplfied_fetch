import gym
from gym.spaces import Box
import numpy as np
import pybullet as p
from simple_grasping.resources.simplefetch import SimpleFetch

class SimpleGraspingEnv(gym.Env):
    def __init__(self):
        self.action_space = Box(
            low=np.array([-.1, -.1, -.1]),
            high=np.array([.1, .1, .1])
        )

        self.action_space = Box(
            low=np.array([-.1, -.1, -.1]),
            high=np.array([.1, .1, .1])
        )

        self.client = p.connect(p.DIRECT)
        self.simplefetch = SimpleFetch(self.client)
        self.goal = None
        self.done = False
        p.setGravity(0, 0, -9.8)
        self.reset()

    def step(self, action):
        self.simplefetch.apply_action(action)
        p.stepSimulation()
        observation = self.simplefetch.get_observation()

    def reset(self):
        p.resetSimulation(self.client)

    def close(self):
        p.disconnect(self.client)
