import numpy as np
from time import sleep
from simple_grasping.standard_interfaces import Action, Observation

class SequentialAgent:
    def __init__(self):
        self.stepindex = 0

    def choose_action(self, observation:Observation) -> Action:
        self.stepindex += 1
        if self.stepindex == 1: return Action(_x_dist=0, _y_dist=0, _z_interact=False)
        if self.stepindex == 2: return Action(_x_dist=0.1, _y_dist=0, _z_interact=False)
        if self.stepindex == 3: return Action(_x_dist=0, _y_dist=0.1, _z_interact=False)
        if self.stepindex == 4: return Action(_x_dist=-0.1, _y_dist=0, _z_interact=False)
        if self.stepindex == 5: return Action(_x_dist=0.0, _y_dist=-0.1, _z_interact=False)
        return Action(0,0,False)
