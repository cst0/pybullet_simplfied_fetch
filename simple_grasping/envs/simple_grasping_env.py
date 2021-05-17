import gym
import numpy as np
import pybullet as p
from gym.spaces import Box
from simple_grasping.resources.simplefetch import SimpleFetch
from simple_grasping.standard_interfaces import *
from typing import List


DEBUGMODE = False


class SimpleFetchEnv(gym.Env):
    def __init__(self):
        self.steps_taken = 0

        self.table_x_min = -0.15
        self.table_x_max = 0.15
        self.table_y_min = -0.15
        self.table_y_max = 0.15
        self.padding_space = 0.01
        self.TABLE_HEIGHT = 0.725
        self.NO_SPAWN_IN_CENTER = False

        self.blocks:List[BlockObject] = []
        self.everything_in_tower = False

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
                -.5, -.5,  # gripper- cube 1 x, y, z
                -.5, -.5,  # gripper- cube 2 x, y, z
                -.5, -.5,  # gripper- cube 3 x, y, z
                0,0,0
                ], dtype=np.float32),
            high=np.array([
                .5, .5,     # gripper x, y, z
                .5, .5,     # cube 1 x, y, z
                .5, .5,     # cube 2 x, y, z
                3,3,3                
                ], dtype=np.float32),
        )

        self.observation = np.array([
                .0, .0,  # gripper x, y, z
                .0, .0,  # cube 1 x, y, z
                .0, .0,  # cube 2 x, y, z
                0,0,0
                ], dtype=np.float32)

        # self.client = p.connect(p.GUI)
        self.client = p.connect(p.DIRECT)

        print("setup worldstate")
        self.worldstate              = Observation(self.client)
        self.worldstate.gripper      = Pose(0,0,0)
        self.worldstategrasping      = Block.NONE
        self.worldstate.block_small  = BlockObject(self.client, nonetype = True)
        self.worldstate.block_medium = BlockObject(self.client, nonetype = True)
        self.worldstate.block_large  = BlockObject(self.client, nonetype = True)
        self.tower:List[BlockObject]  = []

        print("done setting up worldstate")
        self.goal = None
        self.finish = False
        p.setGravity(0, 0, -9.8)
        self.reset()

    def get_block(self, b:Block) -> BlockObject:
        for block in self.blocks:
            if block.btype == b:
                return block

        return BlockObject(self.client, nonetype=True)

    def check_everything_in_tower(self):
        for b in self.blocks:
            if b not in BLOCKTOWER:
                return False
        return True

    def observe(self):
        self.worldstate.gripper      = self.simplefetch.get_ee_position()
        self.worldstate.grasping     = self.simplefetch.grasped_block
        self.worldstate.block_small  = self.get_block(Block.SMALL)
        self.worldstate.block_medium = self.get_block(Block.MEDIUM)
        self.worldstate.block_large  = self.get_block(Block.LARGE)
        self.tower:List[BlockObject] = BLOCKTOWER
        self.everything_in_tower = self.check_everything_in_tower()

    def get_observations(self):
        observations= []
        observations.append(self.worldstate.gripper.x - self.worldstate.block_small.position().x)
        observations.append(self.worldstate.gripper.y - self.worldstate.block_small.position().y)
        observations.append(self.worldstate.gripper.x - self.worldstate.block_medium.position().x)
        observations.append(self.worldstate.gripper.y - self.worldstate.block_medium.position().y)
        observations.append(self.worldstate.gripper.x - self.worldstate.block_large.position().x)
        observations.append(self.worldstate.gripper.y - self.worldstate.block_large.position().y)
        if self.simplefetch.grasped_block == Block.SMALL:
            observations.append(1)
        elif self.simplefetch.grasped_block == Block.MEDIUM:
            observations.append(2)
        elif self.simplefetch.grasped_block == Block.LARGE:
            observations.append(3)
        else:
            observations.append(0)
        if get_tower_top_type() == Block.LARGE:
            observations.append(3)
        if get_tower_top_type() == Block.MEDIUM:
            observations.append(2)
        if get_tower_top_type() == Block.SMALL:
            observations.append(1)
        if get_tower_top_type() == Block.NONE:
            observations.append(0)            
        if get_tower_second_type() == Block.LARGE:
            observations.append(3)
        if get_tower_second_type() == Block.MEDIUM:
            observations.append(2)
        if get_tower_second_type() == Block.SMALL:
            observations.append(1)
        if get_tower_second_type() == Block.NONE:
            observations.append(0)       
        return np.asarray(observations)        

    def step(self, action: Action):
        self.steps_taken += 1
        self.simplefetch.inform_world_states(self.blocks)
        self.simplefetch.apply_action(action)
        p.stepSimulation()
        self.observe()

        #if DEBUGMODE:
        #    statestring = str(self.worldstate.gripper     ) + ", "+\
        #                  str(self.worldstate.grasping    ) + ", "+\
        #                  str(self.worldstate.block_small ) + ", "+\
        #                  str(self.worldstate.block_medium) + ", "+\
        #                  str(self.worldstate.block_large )
        #    print(statestring)

        obs = self.get_observations()

        self.finish = self.everything_in_tower
        return obs, self.compute_reward(), self.finish, None

    def compute_reward(self):
        reward = -1

        # two consistently maintained blocks that can be accessed for computing reward
        self.worldstate # Observation object
        self.tower # list of BlockObjects representing stacked blocks

        # the current top of the tower can be large, medium, small, and that can be rewarded
        if get_tower_top_type() == Block.LARGE:
            reward += 1
        if get_tower_top_type() == Block.MEDIUM:
            reward += 2
        if get_tower_top_type() == Block.SMALL:
            reward += 3

        # verbose_action_results is updated every step to indicate things that happened last step
        for r in self.simplefetch.verbose_action_results:
            if r == ActionOutcomes.FAILED_INTERACT_NO_OBJECT:
                reward -= 1
            if r == ActionOutcomes.FAILED_INTERACT_STACKED_OBJECT:
                reward -= 2
            if r == ActionOutcomes.FAILED_INTERACT_NOT_TOWER:
                reward -= 3
            if r == ActionOutcomes.FAILED_INTERACT_WRONG_ORDER:
                reward -= 4
            if r == ActionOutcomes.FAILED_MOVE_OUT_OF_BOUNDS:
                reward -= 5
            if r == ActionOutcomes.FAILED_MOVE_TIMEOUT:
                reward -= 10
            if r == ActionOutcomes.ACTION_JUST_GRABBED_BLOCK:
                reward += 10
            if r == ActionOutcomes.ACTION_JUST_RELEASED_BLOCK:
                reward += 10
        return reward

        if DEBUGMODE:
            print('* That step:')
            for r in self.simplefetch.verbose_action_results:
                print(r)
            print('---')


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
                print("Randomly generating positions")
                pos = self.generate_valid_table_position(block_positions)
                block_positions.append(pos)
        if len(block_positions) > len(blocklist):
            print("You've specified more block positions than blocks. Some positions will be ignored.")

        # we have some z values to fix based off of the block sizes, and we'll
        # assume all theta's should be 0 and then we actually get to work
        # placing that stuff
        for n in range(0, len(blocklist)):
            if blocklist[n] == Block.NONE:
                continue
            block_positions[n].z = self.TABLE_HEIGHT + block_size_data[blocklist[n]].height/2
            block_positions[n].theta = 0
            thisblock = BlockObject(self.client, Pose(block_positions[n].x, block_positions[n].y, block_positions[n].z), blocklist[n])
            self.blocks.append(thisblock)
            # make sure large block starts as base of tower
            if thisblock.btype == Block.LARGE:
                BLOCKTOWER.clear()
                BLOCKTOWER.append(thisblock)
        p.stepSimulation()

    def generate_valid_table_position(self, block_positions):
        if self.simplefetch is None:
            print("simplefetch is none when attempting to place blocks, resetting")
            self.reset()

        keep_checking = True
        returnme = Pose(0,0,-1)
        # continue randomly generating poses until we get one that isn't in collision
        while keep_checking:
            keep_checking = False
            returnme = Pose(
                    np.random.uniform(-self.simplefetch.X_LIMIT, self.simplefetch.X_LIMIT),
                    np.random.uniform(-self.simplefetch.Y_LIMIT, self.simplefetch.Y_LIMIT),
                    0, 0)
            if self.NO_SPAWN_IN_CENTER:
                if abs(returnme.x) < 0.05 or abs(returnme.y) < 0.05: # 1/2 medium + 1/2 large block size
                    keep_checking = True
            for b in block_positions:  #FIXME: magic numbers
                if abs(max(b.x, returnme.x) - min(b.x, returnme.x)) < 0.05 or \
                   abs(max(b.y, returnme.y) - min(b.y, returnme.y)) < 0.05:
                       keep_checking = True

        return returnme


    def reset(self) -> Observation:
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.8)
        self.simplefetch = SimpleFetch(self.client)

        saveblocks = self.blocks.copy()
        self.blocks.clear()
        for b in saveblocks:
            self.place_objects([b.btype], [b.start_position])

        self.observe()
        # this hack makes the rest of the grasping work better for some reason.
        self.simplefetch.to_position_by_velocity(
                Action(
                    _x_dist=-0.001,
                    _y_dist=-0.001,
                    _z_interact=False))

        self.simplefetch.to_position_by_velocity(
                Action(
                    _x_dist=0.001,
                    _y_dist=0.001,
                    _z_interact=False))

        #return self.observation_space
        obs = self.get_observations()
     
        return obs

    def close(self):
        p.disconnect(self.client)
