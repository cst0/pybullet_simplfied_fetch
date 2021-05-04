from typing import List

class Pose:
    def __init__(self, _x:float, _y:float, _z:float, _theta:float):
        self.x = _x
        self.y = _y
        self.z = _z
        self.theta = _theta

class Observation:
    def __init__(self, _gripper:Pose, _block_positions:List[Pose]):
        self.gripper = _gripper
        self.block_positions = _block_positions

class Action:
    def __init__(self, _x_dist:float, _y_dist:float, _z_dist:float, _theta_dist:float):
        self.x_dist = _x_dist
        self.y_dist = _y_dist
        self.z_dist = _z_dist
        self.theta_vel = _theta_dist
