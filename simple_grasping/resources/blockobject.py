from simple_grasping.standard_interfaces import Block, Pose, block_size_data
import pybullet as p
import os


class BlockObject:
    def __init__(self, client, location:Pose, _type:Block):
        self.client = client
        filename = os.path.join(os.path.dirname(__file__), block_size_data[_type].mesh)
        print("Loading block URDF: "+filename+" at "+str(location))
        self.id = p.loadURDF(fileName=filename,
                basePosition=[location.x, location.y, location.z],
                physicsClientId=client)

        self.shape = block_size_data[_type]
        self.btype = _type

        self.start_position = location

    def position(self):
        return Pose(_x=p.getLinkStates(self.id, [0])[0][0][0],
             _y=p.getLinkStates(self.id, [0])[0][0][1],
             _z=p.getLinkStates(self.id, [0])[0][0][2])

