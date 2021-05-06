from typing import List
from enum import Enum


block_size_data = {
        Block.SMALL  : Shape(50, 50, 50, _mesh="small_block.urdf"),
        Block.MEDIUM : Shape(70, 70, 70, _mesh="medium_block.urdf"),
        Block.LARGE  : Shape(90, 90, 90, _mesh="large_block.urdf"),
}


class Pose:
    def __init__(self, _x:float, _y:float, _z:float, _theta:float):
        self.x = _x
        self.y = _y
        self.z = _z
        self.theta = _theta


class Shape:
    def __init__(self, _length:float, _width:float, _height:float, _mesh:str=None):
        self.length = _length
        self.width = _width
        self.height = _height
        self.mesh = _mesh


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


class Block(Enum):
    SMALL = 1
    MEDIUM = 2
    LARGE = 3
