import pybullet as p
from typing import List
from enum import Enum

class Block(Enum):
    SMALL = 1
    MEDIUM = 2
    LARGE = 3


class Shape:
    def __init__(self, _length:float, _width:float, _height:float, _mesh:str=None):
        self.length = _length
        self.width = _width
        self.height = _height
        self.mesh = _mesh


block_size_data = {
        Block.SMALL  : Shape(50, 50, 50, _mesh="small_block.urdf"),
        Block.MEDIUM : Shape(70, 70, 70, _mesh="medium_block.urdf"),
        Block.LARGE  : Shape(90, 90, 90, _mesh="large_block.urdf"),
}


urdf_string_data = {
        "start_pose_offset_fixed_joint" : 0,
        "table_to_gripper_x"            : 1,
        "table_to_gripper_y"            : 2,
        "table_to_gripper_z"            : 3,
        "prismatic_to_gripper"          : 4,
        "gripper_to_finger_left"        : 5,
        "gripper_to_finger_right"       : 6
}


class Pose:
    def __init__(self, _x:float, _y:float, _z:float, _theta:float=None):
        self.x = _x
        self.y = _y
        self.z = _z
        self.theta = _theta

    def __str__(self):
        return ""+\
            "[x:"+str(round(self.x, 3))+\
            ",y:"+str(round(self.y, 3))+\
            ",z:"+str(round(self.z, 3))+\
            "]"


class Observation:
    def __init__(self, _gripper:Pose, _block_positions:List[Pose], _now_grasping:bool=False):
        self.gripper = _gripper
        self.block_positions = _block_positions
        self.now_grasping = _now_grasping


class Action:
    def __init__(self, _x_dist:float, _y_dist:float, _z_interact:bool, _theta_dist:float=0):
        self.x_dist = _x_dist
        self.y_dist = _y_dist
        self.z_interact = _z_interact
        self.theta_vel = _theta_dist  # ignored atm


class AgentState:
    def __init__(self, urdf, joint:int=4):
        self.urdf = urdf
        self.pose = Pose(
                _x = self.get_xyz_from_index(joint)[0],
                _y = self.get_xyz_from_index(joint)[1],
                _z = self.get_xyz_from_index(joint)[2],
        ) # ee link
        print(self.pose)
        self.finger_distance = self.get_xyz_from_index(5)[1] - self.get_xyz_from_index(6)[1]

    def get_xyz_from_index(self, index):
        return p.getLinkStates(self.urdf, [index])[0][0] # this garbage of an api is why we have a wrapper

# joint-based search of positions by data type
#        if type(joint) is bytes:
#            joint = bytes.decode(joint)
#        if type(joint) is str:
#            for n in range(0, p.getNumJoints(urdf)):
#                if bytes.decode(p.getJointInfo(urdf, n)[1]) == joint:
#                    joint = n
#            if type(joint) is str:
#                # couldn't find that joint :/
#                return
#
#        if type(joint) is int:
#            thisjoint = p.getJointInfo(urdf, joint)
#            self.pose = Pose(
#                _x = thisjoint[13][0],
#                _y = thisjoint[13][1],
#                _z = thisjoint[13][2],
#                _theta = 0
#            )
