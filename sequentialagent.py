import numpy as np
from time import sleep
from simple_grasping.standard_interfaces import Action, Observation

class SequentialAgent:
    def __init__(self):
        self.stepindex = 0

    def choose_action(self, observation:Observation) -> Action:
        self.stepindex += 1

#        if self.stepindex == 1:
#            return Action(
#                    _x_dist=observation.block_large.position().x - observation.gripper.x,
#                    _y_dist=observation.block_large.position().y - observation.gripper.y,
#                    _z_interact=True)
#        if self.stepindex == 2:
#            return Action(
#                    _x_dist=-observation.gripper.x,
#                    _y_dist=-observation.gripper.y,
#                    _z_interact=True
#                )
#        if self.stepindex == 3:
#            return Action(
#                    _x_dist=observation.block_medium.position().x - observation.gripper.x,
#                    _y_dist=observation.block_medium.position().y - observation.gripper.y,
#                    _z_interact=True)
#        if self.stepindex == 4:
#            return Action(
#                    _x_dist=-observation.gripper.x,
#                    _y_dist=-observation.gripper.y,
#                    _z_interact=True
#                )
        if self.stepindex == 5:
            return Action(
                    _x_dist=observation.block_small.position().x - observation.gripper.x,
                    _y_dist=observation.block_small.position().y - observation.gripper.y,
                    _z_interact=True)
        if self.stepindex == 6:
            self.stepindex = 0
            return Action(
                    _x_dist=-observation.gripper.x,
                    _y_dist=-observation.gripper.y,
                    _z_interact=True
                )

        return Action(0,0,False)
