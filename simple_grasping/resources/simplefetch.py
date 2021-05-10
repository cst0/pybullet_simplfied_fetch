#from gym.core import ObservationWrapper
from simple_grasping.standard_interfaces import Action, AgentState, Block, Observation, Pose, urdf_string_data, block_size_data
import pybullet as p
import os
from time import sleep

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
#        sleep(1)
        p.stepSimulation()

        self.grasped_block:Block = Block.NONE

#        print("Loaded fetch with joints:")
#        for n in range(0, p.getNumJoints(self.simplefetch)):
#            joint = p.getJointState(self.simplefetch, n)
#            print(joint)
#            print(str(joint[0]) + ": " + joint[2].decode())

        self.x_axis_joint = urdf_string_data["table_to_gripper_x"]
        self.y_axis_joint = urdf_string_data["table_to_gripper_y"]
        self.z_axis_joint = urdf_string_data["table_to_gripper_z"]
        self.MAXSPEED = 0.5
        self.POSITION_THRESHOLD = 0.025
        self.GRIPPER_OFFSET = 0.2
        self.MOVEMENT_PLANE = 1

    def get_ids(self):
        return self.simplefetch, self.client

    def produce_xyvel(self, action:Action):
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

        return x_goal, x_vel, y_goal, y_vel

    def to_position_by_velocity(self, action:Action):
        x_goal, x_vel, y_goal, y_vel = self.produce_xyvel(action)
        try:
            print("setting velocity to "+str(x_vel)+", "+str(y_vel)+" on "+str(self.simplefetch))
            p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=x_vel)
            p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=y_vel)

            keep_moving = True
            while keep_moving:
                p.stepSimulation()
                # TODO-- refactor to remove duplicate code
                pose = AgentState(self.simplefetch)
                x_now = pose.pose.x
                y_now = pose.pose.y
                x_finished = False
                y_finished = False
                # If we got there on any axis, set that axis to 0 and the other to max just to wrap things up here
                if abs(max(x_now, x_goal) - min(x_now, x_goal)) < self.POSITION_THRESHOLD:
                    print("completed x")
                    x_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)
                    if not y_finished:
                        p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=self.MAXSPEED)

                if abs(max(y_now, y_goal) - min(y_now, y_goal)) < self.POSITION_THRESHOLD:
                    print("completed y")
                    y_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.VELOCITY_CONTROL, targetVelocity=0)
                    if not x_finished:
                        p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=self.MAXSPEED)

                keep_moving = not ( x_finished and y_finished )

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def to_position_by_pose(self, action:Action):
        agent_state = AgentState(self.simplefetch)
        x_abs = agent_state.x + action.x_dist
        y_abs = agent_state.y + action.y_dist

        try:
            p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.POSITION_CONTROL, targetPosition=x_abs)
            p.setJointMotorControl2(self.simplefetch, self.y_axis_joint, p.POSITION_CONTROL, targetPosition=y_abs)
            p.stepSimulation()
            sleep(1/60)
            #while \
            #        abs(max(x_abs, agent_state.x) - min(x_abs, agent_state.x)) > self.POSITION_THRESHOLD and \
            #        abs(max(y_abs, agent_state.y) - min(y_abs, agent_state.y)) > self.POSITION_THRESHOLD:
            #            agent_state = AgentState(self.simplefetch)

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def interact(self, collision_height:float, target_block:Block=None) -> bool:
        # either we're holding a block, in which case we go down to the
        # appropriate height plus half of the currently held block then release
        # and go back up, or we aren't holding a block, in which case we go
        # down to the appropriate height minus half of the goal holding block
        # to close to the block height and go back up
        if self.grasped_block is Block.NONE:
            # holding a block: the height we care about will be
            # collision_height plus what's sticking out of the gripper
            grasp_height = collision_height + self.GRIPPER_OFFSET + (block_size_data[self.grasped_block].height/2)
            p.setJointMotorControl2(self.simplefetch, self.z_axis_joint,
                    p.POSITION_CONTROL, targetPosition=grasp_height)
            self.open_gripper()
            p.setJointMotorControl2(self.simplefetch, self.z_axis_joint,
                    p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)
        else:
            # not holding a block: the height we care about will be the
            # collision height minus half the block height to ensure we're
            # interacting with it
            if target_block is None:
                target_block = Block.NONE
            grasp_height = collision_height + self.GRIPPER_OFFSET - (block_size_data[target_block].height/2)
            p.setJointMotorControl2(self.simplefetch, self.z_axis_joint,
                    p.POSITION_CONTROL, targetPosition=grasp_height)
            self.close_gripper(target_block)
            p.setJointMotorControl2(self.simplefetch, self.z_axis_joint,
                    p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)

    def open_gripper(self):
        pass

    def close_gripper(self, target:Block):
        pass

    def apply_action(self, action: Action):
        print("Provided new goal to obtain: "+str(action))
        return self.to_position_by_pose(action)

    def get_observation(self) -> Observation:
        position = [0, 0, 0]
        angle = [0, 0, 0]
        try:
            position, angle = p.getBasePositionAndOrientation(self.simplefetch)
        except Exception as e:
            print("getting position and orientation failed")
            raise e

        return Observation(Pose(position[0], position[1], position[2], angle[0]), [Pose(0,0,0,0)])
