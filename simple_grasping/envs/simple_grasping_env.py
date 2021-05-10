import gym
import numpy as np
import os
import pybullet as p
from gym.spaces import Box
from simple_grasping.resources.simplefetch import SimpleFetch
from simple_grasping.standard_interfaces import Action, Pose, Block, block_size_data
from typing import List, Tuple

class SimpleFetchEnv(gym.Env):
    def __init__(self):
        self.table_x_min = -0.5
        self.table_x_max = 0.5
        self.table_y_min = -0.5
        self.table_y_max = 0.5
        self.padding_space = 0.01

        self.blocks:List[Block] = []
        self.block_ids:List[int] = []

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

        self.client = p.connect(p.GUI)

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
        self.simplefetch.inform_world_states(self.blocks, self.block_ids)
        self.simplefetch.apply_action(action)
        p.stepSimulation()
        self.observe()
        print(self.observation_space)
        return self.observation_space, self.compute_reward(), self.finish, None

    def compute_reward(self):
        return 0

    def place_objects(self, blocks:List[Block], block_positions:List[Pose]=None):
        """
        place objects in the environment. Specify blocks to be placed, and
        optionally specify the locations to place them in (z will be ignored,
        theta defaulting to normal). If no locations are provided, location will be random.
        """
        # handle case where nothing is specified
        if len(blocks) == 0:
            print("Blocklist empty, returning immediately")
            return

        # handle cases where there's a length/position mismatch
        if block_positions is None or len(block_positions) < len(blocks):
            if block_positions is None:
                block_positions = []
            else:
                print("You've specified more blocks than block positions. Some random positions will be selected for you.")
            for _ in blocks:
                block_positions.append(self.generate_valid_table_position())
        if len(block_positions) > len(blocks):
            print("You've specified more block positions than blocks. Some positions will be ignored.")

        # we have some z values to fix based off of the block sizes, and we'll
        # assume all theta's should be 0 and then we actually get to work
        # placing that stuff
        for n in range(0, len(blocks)):
            block_positions[n].z = (block_size_data[blocks[n]].height / 2) + self.padding_space
            block_positions[n].theta = 0
            p.setAdditionalSearchPath("../resources/")
            meshname = block_size_data[blocks[n]].mesh
            meshname = meshname if meshname is not None else ""
            path_as_list = __file__.split(os.sep)
            stripped = path_as_list[:-2 if path_as_list[-1] is not '' else -3]
            joinedpath = os.sep.join(stripped)
            filename = os.path.join(joinedpath, 'resources', meshname)
            print("Going to load URDF file "+str(filename))
            self.block_ids.append(p.loadURDF(fileName=filename,
                    basePosition=[block_positions[n].x, block_positions[n].y, block_positions[n].z],
                    physicsClientId=self.client))
            self.blocks.append(blocks[n])

    def generate_valid_table_position(self):
        return Pose(
                np.random.uniform(self.table_x_min, self.table_x_max),
                np.random.uniform(self.table_y_min, self.table_y_max),
                0, 0)

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)
        self.simplefetch = SimpleFetch(self.client)

        self.observe()
        return self.observation_space

    def close(self):
        p.disconnect(self.client)
