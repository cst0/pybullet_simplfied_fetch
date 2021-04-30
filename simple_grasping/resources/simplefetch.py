from gym.core import ObservationWrapper
from simple_grasping.standard_interfaces import Action, Observation, Pose
import pybullet as p
import os

class SimpleFetch:
    def __init__(self, client):
        self.client = client
        #p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath("./resources/")
        p.setAdditionalSearchPath("./resources/meshes")
        filename = os.path.join(os.path.dirname(__file__), 'simplefetch.urdf')
        print("Going to load URDF file "+str(filename))
        self.simplefetch = p.loadURDF(fileName=filename,
                basePosition=[0,0,0.3625],
                physicsClientId=client)
        #p.stepSimulation()

        self.x_axis_joint = 1
        self.y_axis_joint = 2
        self.z_axis_joint = 3
        self.x_speed = 0
        self.y_speed = 0
        self.z_speed = 0

    def get_ids(self):
        return self.simplefetch, self.client

    def apply_action(self, action):
        pass
        #x_speed = action.x_vel
        #y_speed = action.y_vel
        #z_speed = action.z_vel

        #try:
        #    p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=x_speed);
        #    p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=y_speed);
        #    p.setJointMotorControl2(self.simplefetch, self.z_axis_joint, p.VELOCITY_CONTROL, targetVelocity=z_speed);
        #except Exception as e:
        #    print("caught exception when setting joint motor control")
        #    raise e

    def get_observation(self) -> Observation:
        position = [0, 0, 0]
        angle = [0, 0, 0]
        try:
            position, angle = p.getBasePositionAndOrientation(self.simplefetch)
        except Exception as e:
            print("getting position and orientation failed")
            raise e

        return Observation(Pose(position[0], position[1], position[2], angle[0]), [Pose(0,0,0,0)])
