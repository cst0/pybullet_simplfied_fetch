from enum import Enum
from typing import List
import numpy as np
import os
import pybullet as p
import pybullet as p


# constants as indexes
X           = 0
Y           = 1
Z           = 2
NAME        = 1
POSITION    = 14
ORIENTATION = 15


class Block(Enum):
    NONE = 0
    SMALL = 1
    MEDIUM = 2
    LARGE = 3


class Shape:
    def __init__(self, _length:float, _width:float, _height:float, _mesh:str=''):
        self.length = _length
        self.width = _width
        self.height = _height
        self.mesh = _mesh


block_size_data = {
        Block.NONE   : Shape(0,  0,  0),
        Block.SMALL  : Shape(0.050, 0.050, 0.050, _mesh="resources/small_block.urdf"),
        Block.MEDIUM : Shape(0.070, 0.070, 0.070, _mesh="resources/medium_block.urdf"),
        Block.LARGE  : Shape(0.090, 0.090, 0.090, _mesh="resources/large_block.urdf"),
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


class BlockObject:
    def __init__(self, client, location:Pose=Pose(0,0,0), _type:Block=Block.NONE, nonetype:bool=False):
        self.client = client
        self.nonetype = nonetype

        self.shape = block_size_data[_type]
        self.btype = _type
        self.start_position = location

        if self.nonetype:
            return

        filename = os.path.join(os.path.dirname(__file__), block_size_data[_type].mesh)
        print("Loading block URDF: "+filename+" at "+str(location))
        self.id = p.loadURDF(fileName=filename,
                basePosition=[location.x, location.y, location.z],
                physicsClientId=client)

    def position(self):
        return Pose(_x=p.getLinkStates(self.id, [0])[0][0][0],
             _y=p.getLinkStates(self.id, [0])[0][0][1],
             _z=p.getLinkStates(self.id, [0])[0][0][2])

    def __str__(self):
        if self.btype == Block.NONE or self.nonetype:
            return "ID=None"

        typestr = "ID="
        if self.btype == Block.SMALL:
            typestr += "S"
        elif self.btype == Block.MEDIUM:
            typestr += "M"
        elif self.btype == Block.LARGE:
            typestr += "L"

        return typestr + "@" + str(self.position())


urdf_string_data = {
        "start_pose_offset_fixed_joint" : 0,
        "table_to_gripper_x"            : 1,
        "table_to_gripper_y"            : 2,
        "table_to_gripper_z"            : 3,
        "prismatic_to_gripper"          : 4,
        "gripper_to_finger_left"        : 5,
        "gripper_to_finger_right"       : 6
}


class Observation:
    def __init__(self):
        self.gripper:Pose
        self.grasping:Block
        self.block_small:BlockObject
        self.block_medium:BlockObject
        self.block_large:BlockObject


class Action:
    def __init__(self, _x_dist:float, _y_dist:float, _z_interact:bool, _theta_dist:float=0):
        self.x_dist = _x_dist
        self.y_dist = _y_dist
        self.z_interact = _z_interact
        self.theta_vel = _theta_dist  # ignored atm

    def __str__(self):
        return  ""+\
                "[x_relative:"+str(round(self.x_dist, 3))+\
                ",y_relative:"+str(round(self.y_dist, 3))+\
                ",z_interact:"+str(self.z_interact)+\
                "]"


class AgentState:
    def __init__(self, urdf, index:int=4):
        self.urdf = urdf

        self.pose = Pose(
                _x = self.get_xyz_from_index(index)[0],
                _y = self.get_xyz_from_index(index)[1],
                _z = self.get_xyz_from_index(index)[2],
        ) # ee link
        #print(self.pose)  # dbg
        self.finger_distance = self.get_xyz_from_index(5)[1] - self.get_xyz_from_index(6)[1]

    def __str__(self):
        return  " " + str(self.pose.x) +\
                " " + str(self.pose.y) +\
                " " + str(self.pose.z)

    def get_xyz_from_index(self, index):
        return list([p.getLinkStates(self.urdf, [index])[0][0][0],
                        p.getLinkStates(self.urdf, [index])[0][0][1],
                        p.getLinkStates(self.urdf, [index])[0][0][2]])

    def get_index_by_name(self, joint):
        # joint-based search of positions by data type
        if type(joint) is bytes:
            joint = bytes.decode(joint)
        if type(joint) is str:
            for n in range(0, p.getNumJoints(self.urdf)):
                if bytes.decode(p.getJointInfo(self.urdf, n)[1]) == joint:
                    joint = n
            if type(joint) is str:
                # couldn't find that joint :/
                return None

# utils
def random_within(minimum, maximum):
    rand_float = np.random.random()
    _range = abs(maximum - minimum)
    scaled = _range * rand_float
    shifted = scaled + minimum
    return shifted

