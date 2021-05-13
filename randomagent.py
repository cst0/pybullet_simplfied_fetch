from simple_grasping import standard_interfaces
import numpy as np

class RandomAgent:
    def __init__(self):
        pass

    def choose_action(self, observation:standard_interfaces.Observation) -> standard_interfaces.Action:
        a = standard_interfaces.Action(
                _x_dist=np.random.uniform(-0.4, 0.4),
                _y_dist=np.random.uniform(-0.4, 0.4),
                _z_interact=(np.random.random() < 0.05) # x in 100 chance of interacting
            )
        return a
