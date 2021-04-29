import gym
from gym.spaces import Box
import numpy as np
import pybullet as p
from simple_grasping.resources.simplefetch import SimpleFetch

class SimpleFetchEnv(gym.Env):
    def __init__(self):
        self.action_space = Box(
            low=np.array([
                -.1, # gripper x velocity
                -.1, # gripper y velocity
                -.1, # gripper z velocity
                -.0  # gripper theta velocity
                ]),
            high=np.array([
                .1, # gripper x velocity
                .1, # gripper y velocity
                .1, # gripper z velocity
                .0  # gripper theta velocity
                ]),
        )

        self.observation_space = Box(
            low=np.array([
                -.5, -.5, -.5,  # gripper x, y, z
                -.5, -.5, -.5,  # cube 1 x, y, z
                -.5, -.5, -.5,  # cube 2 x, y, z
                -.5, -.5, -.5,  # cube 3 x, y, z
                -3.14             # gripper angle
                ]),
            high=np.array([
                .5, .5, .5,     # gripper x, y, z
                .5, .5, .5,     # cube 1 x, y, z
                .5, .5, .5,     # cube 2 x, y, z
                .5, .5, .5,     # cube 3 x, y, z
                3.14            # gripper angle
                ]),
        )

        self.observation = np.array([
                .0, .0, .0,  # gripper x, y, z
                .0, .0, .0,  # cube 1 x, y, z
                .0, .0, .0,  # cube 2 x, y, z
                .0, .0, .0,  # cube 3 x, y, z
                0            # gripper angle
                ])

        self.client = p.connect(p.DIRECT)
        self.simplefetch = SimpleFetch(self.client)

        self.goal = None
        self.finish = False
        p.setGravity(0, 0, -9.8)
        self.reset()

    def observe(self):
        gripper_observation = self.simplefetch.get_observation()
        self.observation[0] = gripper_observation.gripper.x
        self.observation[1] = gripper_observation.gripper.y
        self.observation[2] = gripper_observation.gripper.z

    def step(self, action):
        self.simplefetch.apply_action(action)
        p.stepSimulation()
        self.observe()
        return self.observation_space, None, self.finish, None

    def reset(self):
        p.resetSimulation(self.client)
        self.observe()
        return self.observation_space

    def close(self):
        p.disconnect(self.client)
