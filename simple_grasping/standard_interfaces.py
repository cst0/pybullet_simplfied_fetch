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
    def __init__(self, _x_vel:float, _y_vel:float, _z_vel:float, _theta_vel:float):
        self.x_vel = _x_vel
        self.y_vel = _y_vel
        self.z_vel = _z_vel
        self.theta_vel = _theta_vel
