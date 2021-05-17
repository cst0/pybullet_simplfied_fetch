from enum import Enum
from typing import List
import numpy as np
import os
import pybullet as p


# constants as indexes
X           = 0
Y           = 1
Z           = 2
NAME        = 1
POSITION    = 14
ORIENTATION = 15
TABLE_HEIGHT = 0.725


class Block(Enum):
    NONE = 0
    SMALL = 1
    MEDIUM = 2
    LARGE = 3


class ActionOutcomes(Enum):
    FAILED_INTERACT_NO_OBJECT = 0
    FAILED_INTERACT_STACKED_OBJECT = 1
    FAILED_MOVE_TIMEOUT = 10
    FAILED_MOVE_OUT_OF_BOUNDS = 11
    ACTION_JUST_GRABBED_BLOCK = 20
    ACTION_JUST_RELEASED_BLOCK = 21

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
                  basePosition=[0, 0, TABLE_HEIGHT + self.shape.height/2],
                  physicsClientId=client)

        p.setJointMotorControl2(self.id, 0, p.POSITION_CONTROL, targetPosition=location.x)
        p.setJointMotorControl2(self.id, 1, p.POSITION_CONTROL, targetPosition=location.y)

    def position(self) -> Pose:
        if self.nonetype:
            print("block is nonetype: you're incorrectly trying to get position of block that doesn't exist.")
            return Pose(0,0,0)

        #position, orientation = p.getBasePositionAndOrientation(self.id)
        #return Pose(
        #    _x=position[0],
        #    _y=position[1],
        #    _z=position[2],
        #)
        return Pose(_x=p.getJointState(self.id, 0)[0],
                    _y=p.getJointState(self.id, 1)[0],
                    _z=p.getJointState(self.id, 2)[0])

    def set_xy_vel(self, x_vel, y_vel):
        if self.nonetype:
            return

        p.setJointMotorControl2(self.id, 0,
                p.VELOCITY_CONTROL, targetVelocity=x_vel)
        p.setJointMotorControl2(self.id, 1,
                p.VELOCITY_CONTROL, targetVelocity=y_vel)

    def set_xy_pos(self, x_pos, y_pos):
        if self.nonetype:
            return

        p.setJointMotorControl2(self.id, 0,
                p.POSITION_CONTROL, targetPosition=x_pos)
        p.setJointMotorControl2(self.id, 1,
                p.POSITION_CONTROL, targetPosition=y_pos)

    def set_z(self, vel):
        if self.nonetype:
           return

        p.setJointMotorControl2(self.id, 2,
                p.VELOCITY_CONTROL, targetPosition=vel)

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


BLOCKTOWER:List[BlockObject] = []

def get_tower_top():
    if len(BLOCKTOWER) == 0:
        return 0
    return BLOCKTOWER[-1].position().z + BLOCKTOWER[-1].shape.height/2


def get_tower_top_type():
    if len(BLOCKTOWER) == 0:
        return Block.NONE
    return BLOCKTOWER[-1].btype


def set_tower_top(bo: BlockObject):
    BLOCKTOWER.append(bo)


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
    def __init__(self, client):
        self.gripper:Pose             = Pose(0,0,0)
        self.grasping:Block           = Block.NONE
        self.block_small:BlockObject  = BlockObject(client, nonetype = True)
        self.block_medium:BlockObject = BlockObject(client, nonetype = True)
        self.block_large:BlockObject  = BlockObject(client, nonetype = True)
        self.tower:List[BlockObject]  = []
        self.walled_this_step:bool    = False
        self.interact_success:bool    = False

    def __str__(self):
        return "Observing:"+str(self.gripper)


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

        self.pose = self.pose_from_joint_state()
        #self.pose_old = Pose(
        #        _x = self.get_xyz_from_index(index)[0],
        #        _y = self.get_xyz_from_index(index)[1],
        #        _z = self.get_xyz_from_index(index)[2],
        #) # ee link
        #print(self.pose, self.pose_old, Pose(self.pose.x - self.pose_old.x, self.pose.y - self.pose_old.y, self.pose.z, self.pose_old.z))  # dbg

        self.finger_distance = self.get_xyz_from_index(5)[1] - self.get_xyz_from_index(6)[1]

    def __str__(self):
        return  " " + str(self.pose.x) +\
                " " + str(self.pose.y) +\
                " " + str(self.pose.z)

    def vel_from_joint_state(self):
        return p.getJointState(self.urdf, 3)[1]

    def pose_from_joint_state(self):
        return Pose(_x=p.getJointState(self.urdf, 1)[0],
                    _y=p.getJointState(self.urdf, 2)[0],
                    _z=p.getJointState(self.urdf, 3)[0])

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


def random_within(minimum, maximum):
    rand_float = np.random.random()
    _range = abs(maximum - minimum)
    scaled = _range * rand_float
    shifted = scaled + minimum
    return shifted

