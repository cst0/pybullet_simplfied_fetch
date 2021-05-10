import numpy as np
from time import sleep
from simple_grasping.standard_interfaces import Action, Observation

class SequentialAgent:
    def __init__(self):
        self.actions = []
        self.actions.append(Action(0, 0, False))
        self.actions.append(Action(0, 0, True))
        self.actions.append(Action(0.5, 0.5, False))
        self.actions.append(Action(0, 0, True))

    def choose_action(self, observation:Observation) -> Action:
        return Action(0,0,False) if len(self.actions) is 0 else self.actions.pop(0)
