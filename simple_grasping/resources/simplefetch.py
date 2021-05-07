from gym.core import ObservationWrapper
from simple_grasping.standard_interfaces import Action, AgentState, Observation, Pose
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
        print("Provided new goal to obtain.")
        x_dist = action.x_dist
        y_dist = action.y_dist

        # breaking this into two components where we won't be violating our top speed.
        w = x_dist / (x_dist + y_dist)
        h = y_dist / (x_dist + y_dist)
        x_speed = w * self.MAXSPEED
        y_speed = h * self.MAXSPEED

        position, _ = p.getBasePositionAndOrientation(self.simplefetch)
        x_goal = position[0] + x_dist
        y_goal = position[1] + y_dist

        x_vel = x_speed if x_dist is not 0 else 0
        y_vel = y_speed if y_dist is not 0 else 0

        x_vel *= 1 if x_dist >= 0 else -1
        y_vel *= 1 if y_dist >= 0 else -1

        try:
            p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=x_vel)
            p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=y_vel)

            keep_moving = True
            while keep_moving:
                # TODO-- refactor to remove duplicate code
                pose = AgentState(self.simplefetch)
                x_now = pose.pose.x
                y_now = pose.pose.y
                x_finished = False
                y_finished = False
                # If we got there on any axis, set that axis to 0 and the other to max just to wrap things up here
                if abs(max(x_now, x_goal) - min(x_now, x_goal)) < self.POSITION_THRESHOLD:
                    x_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)
                    #p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=self.MAXSPEED)

                if abs(max(y_now, y_goal) - min(y_now, y_goal)) < self.POSITION_THRESHOLD:
                    y_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)
                    #p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=self.MAXSPEED)

                keep_moving = not ( x_finished and y_finished )

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
