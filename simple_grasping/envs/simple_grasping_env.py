import gym
import numpy as np
import pybullet as p
import time
from gym.spaces import Box
from simple_grasping.resources.simplefetch import SimpleFetch
from simple_grasping.resources.blockobject import BlockObject
from simple_grasping.standard_interfaces import Action, Pose, Block
from typing import List


class SimpleFetchEnv(gym.Env):
    def __init__(self):
        self.steps_taken = 0

        self.table_x_min = -0.5
        self.table_x_max = 0.5
        self.table_y_min = -0.5
        self.table_y_max = 0.5
        self.padding_space = 0.01
        self.TABLE_HEIGHT = 0.325

        self.blocks:List[BlockObject] = []

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
        self.steps_taken += 1
        self.simplefetch.inform_world_states(self.blocks)
        self.simplefetch.apply_action(action)
        p.stepSimulation()
        self.observe()

        print("loaded blocks:")
        for b in self.blocks:
            print(b.position())

        return self.observation_space, self.compute_reward(), self.finish, None

    def compute_reward(self):
        return 0

    def place_objects(self, blocklist:List[Block], block_positions:List[Pose]=None):
        """
        place objects in the environment. Specify blocks to be placed, and
        optionally specify the locations to place them in (z will be ignored,
        theta defaulting to normal). If no locations are provided, location will be random.
        """
        print("placing blocks")
        # handle case where nothing is specified
        if len(blocklist) == 0:
            print("Blocklist empty, returning immediately")
            return

        # handle cases where there's a length/position mismatch
        if block_positions is None or len(block_positions) < len(blocklist):
            if block_positions is None:
                block_positions = []
            else:
                print("You've specified more blocks than block positions. Some random positions will be selected for you.")
            for _ in blocklist:
                block_positions.append(self.generate_valid_table_position())
        if len(block_positions) > len(blocklist):
            print("You've specified more block positions than blocks. Some positions will be ignored.")

        # we have some z values to fix based off of the block sizes, and we'll
        # assume all theta's should be 0 and then we actually get to work
        # placing that stuff
        for n in range(0, len(blocklist)):
            #block_positions[n].z = (b.shape.height / 2) + self.padding_space + self.TABLE_HEIGHT
            block_positions[n].z = self.TABLE_HEIGHT + .1
            block_positions[n].theta = 0
            thisblock = BlockObject(self.client, Pose(block_positions[n].x, block_positions[n].y, block_positions[n].z), blocklist[n])
            self.blocks.append(thisblock)
        p.stepSimulation()

    def generate_valid_table_position(self):
        if self.simplefetch is None:
            self.reset()

        return Pose(
                np.random.uniform(-self.simplefetch.X_LIMIT, self.simplefetch.X_LIMIT),
                np.random.uniform(-self.simplefetch.Y_LIMIT, self.simplefetch.Y_LIMIT),
                0, 0)

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)
        self.simplefetch = SimpleFetch(self.client)

        saveblocks = self.blocks.copy()
        self.blocks.clear()
        for b in saveblocks:
            self.place_objects([b.btype], [b.start_position])

        self.observe()
        return self.observation_space

    def close(self):
        p.disconnect(self.client)
