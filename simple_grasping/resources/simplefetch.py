import pybullet as p
import pybullet_data
import os
from simple_grasping.standard_interfaces import *
from typing import List
from math import sqrt


DEBUGMODE = False


class SimpleFetch:
    def __init__(self, client):
        self.client = client

        self.X_AXIS_JOINT = urdf_string_data["table_to_gripper_x"]
        self.Y_AXIS_JOINT = urdf_string_data["table_to_gripper_y"]
        self.Z_AXIS_JOINT = urdf_string_data["table_to_gripper_z"]
        self.GRIPPER_LEFT_JOINT = urdf_string_data["gripper_to_finger_left"]
        self.GRIPPER_RIGHT_JOINT = urdf_string_data["gripper_to_finger_right"]

        self.MAXSPEED = 0.01
        self.POSITION_THRESHOLD = 0.015
        self.GRIPPER_BOUNDS_THRESHOLD = 0.05
        self.GRIPPER_OFFSET = 0.2 # 0.934 when in active collision with object
        self.MOVEMENT_PLANE = 0.1
        for b in block_size_data.keys():
            self.MOVEMENT_PLANE += block_size_data[b].height

        self.OPEN = 0.048
        self.CLOSE = 0.0
        self.TABLE_HEIGHT = 0.0 # handled by the urdf, so 0
        self.X_LIMIT = 0.15
        self.Y_LIMIT = 0.15

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        filename = os.path.join(os.path.dirname(__file__), 'simplefetch.urdf')
        print("Going to load URDF file "+str(filename))
        self.simplefetch = p.loadURDF(fileName=filename,
                basePosition=[0.0, 0.0, 0.725/2],
                physicsClientId=client)

        p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)

        self.open_gripper()
        p.stepSimulation()

        self.grasped_block:Block = Block.NONE
        self.blocks:List[BlockObject] = []

        self.GRASP_TOLERANCE = 0.01 # quarter of smallest block

#        print("Loaded fetch with joints:")
#        for n in range(0, p.getNumJoints(self.simplefetch)):
#            joint = p.getJointState(self.simplefetch, n)
#            print(joint)
#            print(str(joint[0]) + ": " + joint[2].decode())

    def get_ids(self):
        return self.simplefetch, self.client

    def force_within_bounds(self, goal:Pose):
        if goal.x > self.X_LIMIT:
            goal.x = self.X_LIMIT
        if goal.x < -self.X_LIMIT:
            goal.x = -self.X_LIMIT

        if goal.y > self.Y_LIMIT:
            goal.y = self.Y_LIMIT
        if goal.y < -self.Y_LIMIT:
            goal.y = -self.Y_LIMIT

        return goal

    def produce_xyvel(self, goal:Pose):
        state = AgentState(self.simplefetch)
        x_dist = max(state.pose.x, goal.x) - min(state.pose.x, goal.x)
        y_dist = max(state.pose.y, goal.y) - min(state.pose.y, goal.y)

        # breaking this into two components where we won't be violating our top speed.
        w = x_dist / (x_dist + y_dist)
        h = y_dist / (x_dist + y_dist)
        x_speed = w * self.MAXSPEED
        y_speed = h * self.MAXSPEED

        p = AgentState(self.simplefetch)
        x_goal = p.pose.x + x_dist
        y_goal = p.pose.y + y_dist

        x_vel = x_speed if x_dist is not 0 else 0
        y_vel = y_speed if y_dist is not 0 else 0

        x_vel *= 1 if x_dist >= 0 else -1
        y_vel *= 1 if y_dist >= 0 else -1

        print(
                "["+
                str(round(p.pose.x, 2))+
                "+"+
                str(round(x_dist, 2))+
                "->"+
                str(round(x_goal, 2))+
                "@"+
                str(round(x_speed, 2))+
                ","+
                str(round(p.pose.y, 2))+
                "+"+
                str(round(y_dist, 2))+
                "->"+
                str(round(y_goal, 2))+
                "@"+
                str(round(y_speed, 2))+
                "]"
                )
        return x_goal, x_vel, y_goal, y_vel

    def to_position_by_velocity(self, action:Action, override_z=None):
        current = AgentState(self.simplefetch)
        goal = Pose(
                current.pose.x + action.x_dist,
                current.pose.y + action.y_dist,
                self.MOVEMENT_PLANE if override_z is None else override_z
                )
        goal = self.force_within_bounds(goal)
        try:
            x_finished = False
            y_finished = False
            z_finished = False if override_z is not None else True

            while not x_finished or not y_finished or not z_finished:
                if self.out_of_bounds():
                    print("need to reset fetch position (out of bounds): "+str(goal))
                    self.reset_position()
                    print("new goal is: "+str(goal))

                # TODO-- refactor to remove duplicate code
                now = AgentState(self.simplefetch)
                x_dir = 1 if now.pose.x < goal.x else -1
                y_dir = 1 if now.pose.y < goal.y else -1

                x_diff = abs(max(goal.x, now.pose.x) - min(goal.x, now.pose.x))
                y_diff = abs(max(goal.y, now.pose.y) - min(goal.y, now.pose.y))
                z_diff = abs(max(goal.z, now.pose.z) - min(goal.z, now.pose.z))

                sum_diff = x_diff + y_diff
                if sum_diff == 0:
                    x_finished = True
                    y_finished = True
                    continue

                x_vel = (x_diff / sum_diff) * self.MAXSPEED * x_dir if x_diff != 0 else 0
                y_vel = (y_diff / sum_diff) * self.MAXSPEED * y_dir if y_diff != 0 else 0

                p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=x_vel)
                p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=y_vel)

                if override_z is not None and x_finished and y_finished: # hold off on z movement until we're safely above
                    p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.POSITION_CONTROL, targetPosition=goal.z)
                    z_finished = True
                else:
                    p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.POSITION_CONTROL, targetPosition=goal.z)

                # If we got there on any axis, set that axis to 0 and the other to max just to wrap things up here
                if x_diff < self.POSITION_THRESHOLD:
                    x_finished = True

                if y_diff < self.POSITION_THRESHOLD:
                    y_finished = True

                if z_diff < self.POSITION_THRESHOLD:
                    z_finished = True

                if DEBUGMODE:
                    print(
                            "Goal: ["+str(round(goal.x, 4))+","+str(round(goal.y, 4))+","+str(round(goal.z, 4))+"]",
                            "Curr: ["+str(round(now.pose.x, 4))+","+str(round(now.pose.y, 4))+","+str(round(now.pose.z))+"]",
                            "Vel: ["+str(round(x_vel, 4))+","+str(round(y_vel, 4))
                            )
                p.stepSimulation()

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def to_position_by_pose(self, action:Action):
        agent_state = AgentState(self.simplefetch)
        x_abs = agent_state.pose.x + action.x_dist
        y_abs = agent_state.pose.y + action.y_dist

        try:
            p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=x_abs)
            p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=y_abs)
            for _ in range(0, 10):
                p.stepSimulation()
            #while \
            #        abs(max(x_abs, agent_state.x) - min(x_abs, agent_state.x)) > self.POSITION_THRESHOLD and \
            #        abs(max(y_abs, agent_state.y) - min(y_abs, agent_state.y)) > self.POSITION_THRESHOLD:
            #            agent_state = AgentState(self.simplefetch)

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

#    def check_collision_height(self) -> Tuple[float, bool]:
#        """
#        if we place a block right now, at what height will it collide with
#        something? When we place it, are we placing it at an x/y position that
#        lends itself to the block staying there, or are we placing it
#        off-coverage (leading to an unstable placement)?
#        @returns (collision_height, True if fully covered else False)
#        """
#        if self.grasped_block is None:
#            print("coverage says no blocks")
#            return self.TABLE_HEIGHT, True
#
#        grasp_candidates = []
#        for block in self.blocks:
#            if self.grasped_block == block:
#                # this is the block we're already holding, skip
#                pass
#            # at what distance between two blocks can we guarantee that a collision is not taking place?
#            clearance_distance = block_size_data[self.grasped_block].width + block.shape.width
#            # at what distance between two blocks can we guarantee that a block is fully supported by the other?
#            coverage_distance = (block_size_data[self.grasped_block].width - block.shape.width)/2
#            block_distance = block.position()
#            block_distance = Pose(
#                _x = abs(max(block.position().x, self.get_ee_position().x) - min(block.position().x, self.get_ee_position().x)),
#                _y = abs(max(block.position().y, self.get_ee_position().y) - min(block.position().y, self.get_ee_position().y)),
#                _z = 0
#                )
#
#            # we make use of the fact that blocks have no rotation here, so we
#            # can check x and y independently to check for not-fully-covered
#            # collision independently. Check first for a lack of clearance,
#            # then check for lack of full coverage.
#            if block_distance.x < clearance_distance or \
#               block_distance.y < clearance_distance:
#                   grasp_candidates.append([block, (coverage_distance < block_distance.x and coverage_distance < block_distance.y), min(block_distance.x, block_distance.y)])
#
#        if len(grasp_candidates) == 0:
#            # no block collisions, we're only over the table.
#            print("coverage says above table")
#            return (self.TABLE_HEIGHT, True)
#        elif len(grasp_candidates) == 1:
#            print("coverage says above "+str(grasp_candidates[0][0]))
#            return grasp_candidates[0][0].shape.height, grasp_candidates[0][1]
#        else:
#            max_index = 0
#            for n in range(0, len(grasp_candidates)):
#                if grasp_candidates[max_index][2] < grasp_candidates[n][2]:
#                    max_index = n
##                if grasp_candidates[max_index][0].shape.height < grasp_candidates[n][0].shape.height:
##                    max_index = n
#            print("coverage says above "+str(grasp_candidates[max_index][0]))
#            return grasp_candidates[max_index][0].shape.height, grasp_candidates[max_index][1]


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

    def distance_from_gripper(self, b:BlockObject):
        ee = self.get_ee_position()
        bp = b.position()
        dx = ee.x - bp.x
        dy = ee.y - bp.y
        return sqrt(dx*dx + dy*dy)

    def pickup(self):
        # we're not currently holding a block. Are we near one we can grab?
        min_index = 0
        for n in range(0, len(self.blocks)):
            if self.distance_from_gripper(self.blocks[n]) < self.distance_from_gripper(self.blocks[min_index]):
                min_index = n
        print("picking up "+str(self.blocks[min_index]))
        p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=-self.MAXSPEED)
        for _ in range(0, 10):
            p.stepSimulation()
        self.grasped_block = self.blocks[min_index].btype
        self.to_position_by_velocity(Action(0,0,False))

    def place(self):
        print("placing "+str(self.grasped_block))
        self.grasped_block = Block.NONE
        pass

    def interact(self):
        if self.grasped_block is Block.NONE:
            self.pickup()
        else:
            self.place()

#    def interact(self):
#        if self.grasped_block is not Block.NONE:
#            # holding a block: the height we care about will be
#            # collision_height plus what's sticking out of the gripper
#            grasp_height = collision_height + self.GRIPPER_OFFSET + (block_size_data[self.grasped_block].height/2)
#            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
#                    p.POSITION_CONTROL, targetPosition=grasp_height)
#            self.open_gripper()
#            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
#                    p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)
#        else:
#            grasp_candidates = []
#            for block in self.blocks:
#                if abs(max(block.position().x, self.get_ee_position().x) - min(block.position().x, self.get_ee_position().x)) < self.GRASP_TOLERANCE and \
#                   abs(max(block.position().y, self.get_ee_position().y) - min(block.position().y, self.get_ee_position().y)) < self.GRASP_TOLERANCE:
#                    grasp_candidates.append(block)

    def inform_world_states(self, blocks:List[BlockObject]):
        self.blocks = blocks

    def out_of_bounds(self):
        state = AgentState(self.simplefetch)
        return abs(state.pose.x) > self.X_LIMIT + self.GRIPPER_BOUNDS_THRESHOLD or abs(state.pose.y) > self.Y_LIMIT + self.GRIPPER_BOUNDS_THRESHOLD

    def reset_position(self):
        p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT,
                p.VELOCITY_CONTROL, targetVelocity=0)
        p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT,
                p.VELOCITY_CONTROL, targetVelocity=0)
        p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                p.VELOCITY_CONTROL, targetVelocity=0)

        p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT,
                p.POSITION_CONTROL, targetPosition=0)
        p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT,
                p.POSITION_CONTROL, targetPosition=0)
        p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)
        p.stepSimulation()

    def apply_action(self, action: Action):
        self.to_position_by_velocity(action)
        result = True
        if action.z_interact:
            result = self.interact()

        return result

    def get_ee_position(self) -> Pose:
        state = AgentState(self.simplefetch)
        return state.pose
