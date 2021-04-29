from simple_grasping.standard_interfaces import Action, Observation, Pose
import pybullet as p
import os

class SimpleFetch:
    def __init__(self, client):
        self.client = client
        #p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath("./resources/")
        p.setAdditionalSearchPath("./resources/meshes")
        f_name = os.path.join(os.path.dirname(__file__), 'simplefetch.urdf')
        self.simplefetch = p.loadURDF(fileName=f_name,
                basePosition=[0,0,0.3625],
                physicsClientId=client)
        p.stepSimulation()

        self.x_axis_joint = 1
        self.y_axis_joint = 2
        self.z_axis_joint = 3
        self.x_speed = 0
        self.y_speed = 0
        self.z_speed = 0

    def get_ids(self):
        return self.simplefetch, self.client

    def apply_action(self, action: Action):
        x_speed = action.x_vel
        y_speed = action.y_vel
        z_speed = action.z_vel

        try:
            p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=x_speed);
            p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=y_speed);
            p.setJointMotorControl2(self.simplefetch, self.z_axis_joint, p.VELOCITY_CONTROL, targetVelocity=z_speed);
        except Exception:
            print("caught exception when setting joint motor control")

    def get_observation(self) -> Observation:
        position = [0, 0, 0]
        angle = [0, 0, 0]
        try:
            position, angle = p.getBasePositionAndOrientation(self.simplefetch)
        except Exception:
            print("getting position and orientation failed")

        return Observation(Pose(position[0], position[1], position[2], angle[0]), [Pose(0,0,0,0)])
