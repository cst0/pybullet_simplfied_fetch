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
        self.MAXSPEED = 0.1
        self.POSITION_THRESHOLD = 0.025

    def get_ids(self):
        return self.simplefetch, self.client

    def apply_action(self, action: Action):
        x_dist = action.x_dist
        y_dist = action.y_dist
        z_dist = action.z_dist

        position, _ = p.getBasePositionAndOrientation(self.simplefetch)
        x_goal = position[0] + x_dist
        y_goal = position[1] + y_dist
        z_goal = position[2] + z_dist

        x_vel = self.MAXSPEED if x_dist is not 0 else 0
        y_vel = self.MAXSPEED if y_dist is not 0 else 0
        z_vel = self.MAXSPEED if z_dist is not 0 else 0

        x_vel *= 1 if x_dist >= 0 else -1
        y_vel *= 1 if y_dist >= 0 else -1
        z_vel *= 1 if z_dist >= 0 else -1

        try:
            p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=x_vel)
            p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=y_vel)
            p.setJointMotorControl2(self.simplefetch, self.z_axis_joint, p.VELOCITY_CONTROL, targetVelocity=z_vel)

            keep_moving = True
            while keep_moving:
                # TODO-- refactor to remove duplicate code
                position, _ = p.getBasePositionAndOrientation(self.simplefetch)
                x_now = position[0]
                y_now = position[1]
                z_now = position[2]
                x_finished = False
                y_finished = False
                z_finished = False
                if abs(max(x_now, x_goal) - min(x_now, x_goal)) < self.POSITION_THRESHOLD:
                    x_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)
                if abs(max(y_now, y_goal) - min(y_now, y_goal)) < self.POSITION_THRESHOLD:
                    y_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)
                if abs(max(z_now, z_goal) - min(z_now, z_goal)) < self.POSITION_THRESHOLD:
                    z_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.z_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)

                keep_moving = not ( x_finished and y_finished and z_finished )

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def get_observation(self) -> Observation:
        position = [0, 0, 0]
        angle = [0, 0, 0]
        try:
            position, angle = p.getBasePositionAndOrientation(self.simplefetch)
        except Exception as e:
            print("getting position and orientation failed")
            raise e

        return Observation(Pose(position[0], position[1], position[2], angle[0]), [Pose(0,0,0,0)])
