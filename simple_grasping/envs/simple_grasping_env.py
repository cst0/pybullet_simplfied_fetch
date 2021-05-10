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
        self.table_height = 0.3625
        self.table_x_min = -0.5
        self.table_x_max = 0.5
        self.table_y_min = -0.5
        self.table_y_max = 0.5
        self.padding_space = 0.01

        self.blocks = []
        self.block_ids = List(Block)

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
            block_positions[n].z = self.table_height + (block_size_data[blocks[n]].height / 2) + self.padding_space
            block_positions[n].theta = 0
            p.setAdditionalSearchPath("../resources/")
            meshname = block_size_data[blocks[n]].mesh
            meshname = meshname if meshname is not None else ""
            path_as_list = __file__.split(os.sep)
            stripped = path_as_list[:-2 if path_as_list[-1] is not '' else -3]
            joinedpath = os.sep.join(stripped)
            filename = os.path.join(joinedpath, 'resources', meshname)
            print("Going to load URDF file "+str(filename))
            self.blocks.append(p.loadURDF(fileName=filename,
                    basePosition=[block_positions[n].x, block_positions[n].y, block_positions[n].z],
                    physicsClientId=self.client))
            self.block_ids.append(blocks[n])

    def generate_valid_table_position(self):
        return Pose(
                np.random.uniform(self.table_x_min, self.table_x_max),
                np.random.uniform(self.table_y_min, self.table_y_max),
                0, 0)

    def get_block_position(self, block:Block) -> Pose:
        return Pose(0,0,0)

    def check_collision_height(self) -> Tuple[float, bool]:
        """
        if we place a block right now, at what height will it collide with
        something? When we place it, are we placing it at an x/y position that
        lends itself to the block staying there, or are we placing it
        off-coverage (leading to an unstable placement)?
        @returns (collision_height, True if fully covered else False)
        """
        if self.simplefetch.grasped_block is None:
            return self.table_height, True

        for block in self.block_ids:
            if self.simplefetch.grasped_block == block:
                # this is the block we're already holding, skip
                pass
            # at what distance between two blocks can we guarantee that a collision is not taking place?
            clearance_distance = block_size_data[self.simplefetch.grasped_block].width + block_size_data[block].width
            # at what distance between two blocks can we guarantee that a block is fully supported by the other?
            coverage_distance = (block_size_data[self.simplefetch.grasped_block].width - block_size_data[block].width)/2
            block_distance = self.get_block_position(block)

            # we make use of the fact that blocks have no rotation here, so we
            # can check x and y independently to check for not-fully-covered
            # collision independently. Check first for a lack of clearance,
            # then check for lack of full coverage.
            if block_distance.x < clearance_distance or \
               block_distance.y < clearance_distance:
                   return (block_size_data[block].height, (coverage_distance < block_distance.x and coverage_distance < block_distance.y))

        # no block collisions, we're only over the table.
        return (self.table_height, True)

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)
        self.simplefetch = SimpleFetch(self.client)

        self.observe()
        return self.observation_space

    def close(self):
        p.disconnect(self.client)
