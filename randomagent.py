from simple_grasping import standard_interfaces
import numpy as np

class RandomAgent:
    def __init__(self):
        pass

    def choose_action(self, observation:standard_interfaces.Observation) -> standard_interfaces.Action:
        print(observation)
        a = standard_interfaces.Action(
                _x_vel=np.random.random(),
                _y_vel=np.random.random(),
                _z_vel=np.random.random(),
                _theta_vel=np.random.random()
            )
        return a
