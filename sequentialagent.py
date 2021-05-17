import numpy as np
from time import sleep
from simple_grasping.standard_interfaces import Action, BLOCKTOWER, Observation

class SequentialAgent:
    def __init__(self):
        self.stepindex = 0

    def choose_action(self, observation:Observation) -> Action:
        self.stepindex += 1

        print("Current tower state: "+str([x.btype for x in BLOCKTOWER]))
        if self.stepindex == 1:
            print('** GRAB LARGE BLOCK')
            return Action(
                    _x_dist=observation.block_large.position().x - observation.gripper.x,
                    _y_dist=observation.block_large.position().y - observation.gripper.y,
                    _z_interact=True)
        if self.stepindex == 2:
            print('** DROP LARGE BLOCK')
            return Action(
                    _x_dist=-observation.gripper.x,
                    _y_dist=-observation.gripper.y,
                    _z_interact=True
                )
        if self.stepindex == 3:
            print('** GRAB MEDIUM BLOCK')
            return Action(
                    _x_dist=observation.block_medium.position().x - observation.gripper.x,
                    _y_dist=observation.block_medium.position().y - observation.gripper.y,
                    _z_interact=True)
        if self.stepindex == 4:
            print('** DROP MEDIUM BLOCK')
            return Action(
                    _x_dist=observation.block_large.position().x - observation.gripper.x,
                    _y_dist=observation.block_large.position().y - observation.gripper.y,
                    _z_interact=True
                )
        if self.stepindex == 5:
            print('** GRAB SMALL BLOCK')
            return Action(
                    _x_dist=observation.block_small.position().x - observation.gripper.x,
                    _y_dist=observation.block_small.position().y - observation.gripper.y,
                    _z_interact=True)
        if self.stepindex == 6:
            print('** DROP SMALL BLOCK')
            return Action(
                    _x_dist=observation.block_large.position().x - observation.gripper.x,
                    _y_dist=observation.block_large.position().y - observation.gripper.y,
                    _z_interact=True
                )

        self.stepindex = 0
        return self.choose_action(observation)
