from simple_grasping.standard_interfaces import Action
import gym
from gym.spaces import Box
import numpy as np
import pybullet as p
from simple_grasping.resources.simplefetch import SimpleFetch

class SimpleFetchEnv(gym.Env):
    def __init__(self):
        self.action_space = Box(
            low=np.array([
                -.1, # gripper x relative position change
                -.1, # gripper y relative position change
                -.1, # gripper z velocity
                ], dtype=np.float32),
            high=np.array([
                .1, # gripper x relative position change
                .1, # gripper y relative position change
                .1, # gripper z velocity
                ], dtype=np.float32),
        )

        self.observation_space = Box(
            low=np.array([
                -.5, -.5, -.5,  # gripper x, y, z
                -.5, -.5, -.5,  # cube 1 x, y, z
                -.5, -.5, -.5,  # cube 2 x, y, z
                -.5, -.5, -.5,  # cube 3 x, y, z
                ], dtype=np.float32),
            high=np.array([
                .5, .5, .5,     # gripper x, y, z
                .5, .5, .5,     # cube 1 x, y, z
                .5, .5, .5,     # cube 2 x, y, z
                .5, .5, .5,     # cube 3 x, y, z
                ], dtype=np.float32),
        )

        self.observation = np.array([
                .0, .0, .0,  # gripper x, y, z
                .0, .0, .0,  # cube 1 x, y, z
                .0, .0, .0,  # cube 2 x, y, z
                .0, .0, .0,  # cube 3 x, y, z
                0            # gripper angle
                ], dtype=np.float32)

        self.client = p.connect(p.DIRECT)

        self.goal = None
        self.finish = False
        p.setGravity(0, 0, -9.8)
        self.reset()

    def observe(self):
        gripper_observation = self.simplefetch.get_observation()
        self.observation[0] = gripper_observation.gripper.x
        self.observation[1] = gripper_observation.gripper.y
        self.observation[2] = gripper_observation.gripper.z

    def step(self, action: Action):
        self.simplefetch.apply_action(action)
        p.stepSimulation()
        self.observe()
        print(self.observation_space)
        return self.observation_space, self.compute_reward(), self.finish, None

    def compute_reward(self):
        return 0

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)
        self.simplefetch = SimpleFetch(self.client)

        self.observe()
        return self.observation_space

    def close(self):
        p.disconnect(self.client)
