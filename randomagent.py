from simple_grasping import standard_interfaces
import numpy as np

class RandomAgent:
    def __init__(self):
        pass

    def choose_action(self, observation:standard_interfaces.Observation) -> standard_interfaces.Action:
        a = standard_interfaces.Action(
                _x_dist=np.random.random(),
                _y_dist=np.random.random(),
                _z_interact=np.random.choice([True, False])
            )
        return a
