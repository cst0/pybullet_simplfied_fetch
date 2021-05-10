#from gym.core import ObservationWrapper
from simple_grasping.standard_interfaces import Action, AgentState, Block, Observation, Pose, urdf_string_data, block_size_data
import pybullet as p
import os
from time import sleep
from typing import List, Tuple

class SimpleFetch:
    def __init__(self, client):
        self.client = client

        self.X_AXIS_JOINT = urdf_string_data["table_to_gripper_x"]
        self.Y_AXIS_JOINT = urdf_string_data["table_to_gripper_y"]
        self.Z_AXIS_JOINT = urdf_string_data["table_to_gripper_z"]
        self.GRIPPER_LEFT_JOINT = urdf_string_data["gripper_to_finger_left"]
        self.GRIPPER_RIGHT_JOINT = urdf_string_data["gripper_to_finger_right"]

        self.MAXSPEED = 0.5
        self.POSITION_THRESHOLD = 0.025
        self.GRIPPER_OFFSET = 0.2
        self.MOVEMENT_PLANE = 0.1
        for b in block_size_data.keys():
            self.MOVEMENT_PLANE += block_size_data[b].height
        self.OPEN = 0.048
        self.CLOSE = 0.0
        self.TABLE_HEIGHT = 0.0 # handled by the urdf, so 0


        #p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath("./resources/")
        p.setAdditionalSearchPath("./resources/meshes")
        filename = os.path.join(os.path.dirname(__file__), 'simplefetch.urdf')
        print("Going to load URDF file "+str(filename))
        self.simplefetch = p.loadURDF(fileName=filename,
                basePosition=[0,0,0.3625],
                physicsClientId=client)

        self.open_gripper()

        p.stepSimulation()

        self.grasped_block:Block = Block.NONE
        self.blocks:List[Block] = []
        self.block_ids:List[int] = []

#        print("Loaded fetch with joints:")
#        for n in range(0, p.getNumJoints(self.simplefetch)):
#            joint = p.getJointState(self.simplefetch, n)
#            print(joint)
#            print(str(joint[0]) + ": " + joint[2].decode())

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
            p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=x_vel)
            p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=y_vel)

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
                    p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=0)
                    if not y_finished:
                        p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=self.MAXSPEED)

                if abs(max(y_now, y_goal) - min(y_now, y_goal)) < self.POSITION_THRESHOLD:
                    print("completed y")
                    y_finished = True
                    p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=0)
                    if not x_finished:
                        p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=self.MAXSPEED)

                keep_moving = not ( x_finished and y_finished )

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def to_position_by_pose(self, action:Action):
        agent_state = AgentState(self.simplefetch)
        x_abs = agent_state.x + action.x_dist
        y_abs = agent_state.y + action.y_dist

        try:
            p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.POSITION_CONTROL, targetPosition=x_abs)
            p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.POSITION_CONTROL, targetPosition=y_abs)
            p.stepSimulation()
            sleep(1/60)
            #while \
            #        abs(max(x_abs, agent_state.x) - min(x_abs, agent_state.x)) > self.POSITION_THRESHOLD and \
            #        abs(max(y_abs, agent_state.y) - min(y_abs, agent_state.y)) > self.POSITION_THRESHOLD:
            #            agent_state = AgentState(self.simplefetch)

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def get_block_position(self, block:Block) -> Pose:
        return Pose(0,0,0)

    def check_collision_height(self) -> Tuple[float, bool]:
        """
        if we place a block right now, at what height will it collide with
        something? When we place it, are we placing it at an x/y position that
        lends itself to the block staying there, or are we placing it
        off-coverage (leading to an unstable placement)?
        @returns (collision_height, True if fully covered else False)
        """
        if self.grasped_block is None:
            return self.TABLE_HEIGHT, True

        for block in self.blocks:
            if self.grasped_block == block:
                # this is the block we're already holding, skip
                pass
            # at what distance between two blocks can we guarantee that a collision is not taking place?
            clearance_distance = block_size_data[self.grasped_block].width + block_size_data[block].width
            # at what distance between two blocks can we guarantee that a block is fully supported by the other?
            coverage_distance = (block_size_data[self.grasped_block].width - block_size_data[block].width)/2
            block_distance = self.get_block_position(block)

            # we make use of the fact that blocks have no rotation here, so we
            # can check x and y independently to check for not-fully-covered
            # collision independently. Check first for a lack of clearance,
            # then check for lack of full coverage.
            if block_distance.x < clearance_distance or \
               block_distance.y < clearance_distance:
                   return (block_size_data[block].height, (coverage_distance < block_distance.x and coverage_distance < block_distance.y))

        # no block collisions, we're only over the table.
        return (self.TABLE_HEIGHT, True)

    def interact(self, collision_height:float, target_block:Block=None):
        # either we're holding a block, in which case we go down to the
        # appropriate height plus half of the currently held block then release
        # and go back up, or we aren't holding a block, in which case we go
        # down to the appropriate height minus half of the goal holding block
        # to close to the block height and go back up
        if self.grasped_block is Block.NONE:
            # holding a block: the height we care about will be
            # collision_height plus what's sticking out of the gripper
            grasp_height = collision_height + self.GRIPPER_OFFSET + (block_size_data[self.grasped_block].height/2)
            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                    p.POSITION_CONTROL, targetPosition=grasp_height)
            self.open_gripper()
            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                    p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)
        else:
            # not holding a block: the height we care about will be the
            # collision height minus half the block height to ensure we're
            # interacting with it
            if target_block is None:
                target_block = Block.NONE
            grasp_height = collision_height + self.GRIPPER_OFFSET - (block_size_data[target_block].height/2)
            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                    p.POSITION_CONTROL, targetPosition=grasp_height)
            self.close_gripper()
            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                    p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)

    def open_gripper(self):
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_LEFT_JOINT,
                p.POSITION_CONTROL, targetPosition=-self.OPEN)
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_RIGHT_JOINT,
                p.POSITION_CONTROL, targetPosition=self.OPEN)

    def close_gripper(self):
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_LEFT_JOINT,
                p.POSITION_CONTROL, targetPosition=self.CLOSE)
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_RIGHT_JOINT,
                p.POSITION_CONTROL, targetPosition=self.CLOSE)

    def inform_world_states(self, blocks:List[Block], block_ids):
        self.blocks = blocks
        self.block_ids = block_ids

    def apply_action(self, action: Action):
        print("Provided new goal to obtain: "+str(action))
        self.to_position_by_pose(action)
        height, covereage = self.check_collision_height()
        if action.z_interact:
            self.interact(height)

        return covereage

    def get_observation(self) -> Observation:
        position = [0, 0, 0]
        angle = [0, 0, 0]
        try:
            position, angle = p.getBasePositionAndOrientation(self.simplefetch)
        except Exception as e:
            print("getting position and orientation failed")
            raise e

        return Observation(Pose(position[0], position[1], position[2], angle[0]), [Pose(0,0,0,0)])
