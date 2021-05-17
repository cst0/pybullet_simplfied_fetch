import pybullet as p
import pybullet_data
import os
import time
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

        self.MAXSPEED = 0.05
        self.STOPPED_SPEED = 0.001
        self.Z_MAXSPEED = 0.01
        self.POSITION_THRESHOLD = 0.0001
        self.GRIPPER_BOUNDS_THRESHOLD = 0.05
        self.GRIPPER_OFFSET = 0.0025 # 0.934 when in active collision with object
        self.ACCEPTABLE_GRAB_SLOP = 0.01

        self.MOVEMENT_PLANE = 0.01
        for b in block_size_data.keys():
            self.MOVEMENT_PLANE += block_size_data[b].height

        self.OPEN = 0.048
        self.CLOSE = 0.0
        self.TABLE_HEIGHT = 0.0 # handled by the urdf, so 0
        self.X_LIMIT = 0.15
        self.Y_LIMIT = 0.15
        self.MAX_PSTEPS = 10000 # how many times should pybullet step before we decide it's a lost cause

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        filename = os.path.join(os.path.dirname(__file__), 'simplefetch.urdf')
        print("Going to load URDF file "+str(filename))
        self.simplefetch = p.loadURDF(fileName=filename,
                basePosition=[0.0, 0.0, 0.725/2],
                physicsClientId=client)

        self.START_POSE = AgentState(self.simplefetch).pose
        print("start pose"+str(self.START_POSE))
        p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT,
                p.POSITION_CONTROL, targetPosition=self.MOVEMENT_PLANE)

        self.open_gripper()
        p.stepSimulation()

        self.grasped_block:Block = Block.NONE
        self.blocks:List[BlockObject] = []
        self.verbose_action_results:List[ActionOutcomes] = []

    def get_ids(self):
        return self.simplefetch, self.client

    def force_within_bounds(self, goal:Pose):
        goal_change = False
        if goal.x > self.X_LIMIT:
            goal_change = True
            goal.x = self.X_LIMIT
        if goal.x < -self.X_LIMIT:
            goal_change = True
            goal.x = -self.X_LIMIT

        if goal.y > self.Y_LIMIT:
            goal_change = True
            goal.y = self.Y_LIMIT
        if goal.y < -self.Y_LIMIT:
            goal_change = True
            goal.y = -self.Y_LIMIT

        if goal_change:
            self.verbose_action_results.append(ActionOutcomes.FAILED_MOVE_OUT_OF_BOUNDS)
        return goal

    def goto_xy_goal(self, goal:Pose):
        x_finished = False
        y_finished = False

        timeout_counter = 0
        while not x_finished or not y_finished:
            timeout_counter += 1
            if timeout_counter > self.MAX_PSTEPS:
                self.verbose_action_results.append(ActionOutcomes.FAILED_MOVE_TIMEOUT)
                print('failed to complete that movement.')
                return

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

            sum_diff = x_diff + y_diff
            if sum_diff == 0:
                x_finished = True
                y_finished = True
                continue

            x_vel = (x_diff / sum_diff) * self.MAXSPEED * x_dir if x_diff != 0 else 0
            y_vel = (y_diff / sum_diff) * self.MAXSPEED * y_dir if y_diff != 0 else 0

            p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=x_vel)
            p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=y_vel)

            if self.grasped_block != Block.NONE:
                self.get_block(self.grasped_block).set_xy_vel(x_vel, y_vel)

            p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.POSITION_CONTROL, targetPosition=goal.z)

            # If we got there on any axis, set that axis to 0 and the other to max just to wrap things up here
            if x_diff < self.POSITION_THRESHOLD:
                x_finished = True

            if y_diff < self.POSITION_THRESHOLD:
                y_finished = True

            if DEBUGMODE:
                print(
                        "Goal: ["+str(round(goal.x, 4))+","+str(round(goal.y, 4))+","+str(round(goal.z, 4))+"]",
                        "Curr: ["+str(round(now.pose.x, 4))+","+str(round(now.pose.y, 4))+","+str(round(now.pose.z))+"]",
                        "Vel: ["+str(round(x_vel, 4))+","+str(round(y_vel, 4))+"]"
                        )
            p.stepSimulation()

    def halt(self):
        p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=0)
        p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=0)
        self.get_block(self.grasped_block).set_xy_vel(0, 0)
        p.stepSimulation()

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

    def to_position_by_velocity(self, action:Action):
        current = AgentState(self.simplefetch)
        goal = Pose(
                current.pose.x + action.x_dist, # + 0.022,# + 0.125/2,
                current.pose.y + action.y_dist,
                self.MOVEMENT_PLANE
                )
        goal = self.force_within_bounds(goal)

        self.goto_xy_goal(goal)

        # snap to a more relevant goal location
        if action.z_interact:
            print('interact')
            min_index = 0
            for n in range(0, len(self.blocks)):
                if self.distance_from_gripper(self.blocks[n]) < self.distance_from_gripper(self.blocks[min_index]):
                    if self.blocks[n] is self.grasped_block:
                        pass
                    else:
                        min_index = n
            if len(self.blocks) != 0:
                nearest_block = self.blocks[min_index]
                if self.distance_from_gripper(nearest_block) < max(self.ACCEPTABLE_GRAB_SLOP, nearest_block.shape.width/2):
                    # cool, this is a block we are in-range to interact with.
                    goal = Pose(
                            nearest_block.position().x,
                            nearest_block.position().y,
                            self.MOVEMENT_PLANE
                            )
                else:
                    action.z_interact = False
                    self.verbose_action_results.append(ActionOutcomes.FAILED_INTERACT_NO_OBJECT)

        self.goto_xy_goal(goal)
        self.halt()

        if action.z_interact:
            if self.grasped_block is Block.NONE:
                # we're not currently holding a block. Are we near one we can grab?
                min_index = 0
                for n in range(0, len(self.blocks)):
                    if self.distance_from_gripper(self.blocks[n]) < self.distance_from_gripper(self.blocks[min_index]):
                        min_index = n
                for towered_block in BLOCKTOWER:
                    if towered_block.btype == self.blocks[min_index].btype:
                        # this is a block we've already stacked
                        print('cannot interact with already-stacked object.')
                        self.verbose_action_results.append(ActionOutcomes.FAILED_INTERACT_STACKED_OBJECT)
                        return
                    if self.out_of_order_grab(self.blocks[min_index]):
                        # this is a block which is out of order to grab
                        print('cannot interact with objects out of order.')
                        self.verbose_action_results.append(ActionOutcomes.FAILED_INTERACT_WRONG_ORDER)
                        return
                print("picking up "+str(self.blocks[min_index]))
                self.grasped_block = self.blocks[min_index].btype

                p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=-self.Z_MAXSPEED)
                current_position = self.get_ee_position()

                #goal_z = self.START_POSE.z + 3*(self.blocks[min_index].shape.height/4)
                goal_z = self.START_POSE.z + (self.blocks[min_index].shape.height/4) + self.GRIPPER_OFFSET

                timeout_counter = 0
                while abs(max(goal_z, current_position.z) - min(goal_z, current_position.z)) > self.POSITION_THRESHOLD:
                    timeout_counter += 1
                    if timeout_counter > self.MAX_PSTEPS:
                        self.verbose_action_results.append(ActionOutcomes.FAILED_MOVE_TIMEOUT)
                        print('failed to complete that movement.')
                        return
                    p.stepSimulation()
                    current_position = self.get_ee_position()
                p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=0)
                time.sleep(1/60)
                self.close_gripper(self.blocks[min_index].shape.width)
                self.verbose_action_results.append(ActionOutcomes.ACTION_JUST_GRABBED_BLOCK)

                p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=self.Z_MAXSPEED)
                goal_z = self.MOVEMENT_PLANE
                while abs(max(goal_z, current_position.z) - min(goal_z, current_position.z)) > self.POSITION_THRESHOLD:
                    p.stepSimulation()
                    current_position = self.get_ee_position()
                print("done picking up "+str(self.blocks[min_index]))

            else:
                print("placing "+str(self.grasped_block))
                current_position = self.get_ee_position()
                current_speed    = self.STOPPED_SPEED + 1
                p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=-self.Z_MAXSPEED)
                self.get_block(self.grasped_block).set_z(-self.Z_MAXSPEED)
                # check if we're about to place on top of the tower
                large_block = self.get_block(Block.LARGE)
                distance_from_large_block = self.distance_from_gripper(large_block)
                goal_start = 0
                onblock = False
                if self.grasped_block != Block.LARGE and\
                        distance_from_large_block < (large_block.shape.width/2 + self.get_block(self.grasped_block).shape.width/2):
                    goal_start = get_tower_top()
                    if get_tower_top_type() != Block.NONE:
                        onblock = True

                if not onblock:
                    self.verbose_action_results.append(ActionOutcomes.FAILED_INTERACT_NOT_TOWER)
                    return

                goal_z = goal_start + self.START_POSE.z + (self.get_block(self.grasped_block).shape.height/4) + self.GRIPPER_OFFSET
                timeout_counter = 0
                while \
                    abs(max(goal_z, current_position.z) - min(goal_z, current_position.z)) > self.POSITION_THRESHOLD\
                    and\
                    current_speed > self.STOPPED_SPEED:
                    timeout_counter += 1
                    if timeout_counter > self.MAX_PSTEPS:
                        self.verbose_action_results.append(ActionOutcomes.FAILED_MOVE_TIMEOUT)
                        print('failed to complete that movement.')
                        return
                    p.stepSimulation()
                    current_position = self.get_ee_position()
                    current_speed = abs(self.get_ee_velocity())
                self.get_block(self.grasped_block).set_z(0)
                p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=0)
                if self.grasped_block == Block.LARGE or onblock:
                    set_tower_top(self.get_block(self.grasped_block))
                self.open_gripper()
                self.verbose_action_results.append(ActionOutcomes.ACTION_JUST_RELEASED_BLOCK)
                set_tower_top(self.get_block(self.grasped_block))
                self.grasped_block = Block.NONE
                p.stepSimulation()
                time.sleep(1/60)
                p.stepSimulation()

                p.setJointMotorControl2(self.simplefetch, self.Z_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=self.Z_MAXSPEED)
                goal_z = self.MOVEMENT_PLANE
                while abs(max(goal_z, current_position.z) - min(goal_z, current_position.z)) > self.POSITION_THRESHOLD:
                    p.stepSimulation()
                    current_position = self.get_ee_position()
                print("done placing")

    def to_position_by_pose(self, action:Action):
        agent_state = AgentState(self.simplefetch)
        x_abs = agent_state.pose.x + action.x_dist
        y_abs = agent_state.pose.y + action.y_dist

        try:
            p.setJointMotorControl2(self.simplefetch, self.X_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=x_abs)
            p.setJointMotorControl2(self.simplefetch, self.Y_AXIS_JOINT, p.VELOCITY_CONTROL, targetVelocity=y_abs)
            for _ in range(0, 10):
                p.stepSimulation()

        except Exception as e:
            print("caught exception when setting joint motor control")
            raise e

    def open_gripper(self):
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_LEFT_JOINT,
                p.POSITION_CONTROL, targetPosition=-self.OPEN)
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_RIGHT_JOINT,
                p.POSITION_CONTROL, targetPosition=self.OPEN)

    def close_gripper(self, distance=None):
        if distance is None:
            distance = self.CLOSE
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_LEFT_JOINT,
                p.POSITION_CONTROL, targetVelocity=self.CLOSE)
        p.setJointMotorControl2(self.simplefetch, self.GRIPPER_RIGHT_JOINT,
                p.POSITION_CONTROL, targetVelocity=self.CLOSE)
        for _ in range(0, 20):
            p.stepSimulation

    def distance_from_gripper(self, b:BlockObject):
        ee = self.get_ee_position()
        bp = b.position()
        dx = ee.x - bp.x
        dy = ee.y - bp.y
        return sqrt(dx*dx + dy*dy)

    def inform_world_states(self, blocks:List[BlockObject]):
        self.blocks = blocks

    def get_block(self, b:Block) -> BlockObject:
        for block in self.blocks:
            if block.btype == b:
                return block

        return BlockObject(self.client, nonetype=True)

    def out_of_bounds(self):
        state = AgentState(self.simplefetch)
        return abs(state.pose.x) > self.X_LIMIT + self.GRIPPER_BOUNDS_THRESHOLD or abs(state.pose.y) > self.Y_LIMIT + self.GRIPPER_BOUNDS_THRESHOLD

    def in_tower(self, checkme):
        for b in BLOCKTOWER:
            if b.btype.value == checkme.value:
                return True
        return False

    def in_environment(self, checkme:Block):
        for b in self.blocks:
            if b.btype.value == checkme.value:
                return True

    def out_of_order_grab(self, attempt_placement:BlockObject):
        # order is always large -> medium -> small.
        # is the block the agent is about to grab/release the wrong one to do here?
        current_top = get_tower_top_type()
        expected_top = Block.NONE
        # TODO: there's 100% a better way to do this, esp given that the blocks are ordered enums.
        if current_top == Block.NONE:
            if self.in_environment(Block.LARGE):
                expected_top = Block.LARGE
            elif self.in_environment(Block.MEDIUM):
                expected_top = Block.MEDIUM
            elif self.in_environment(Block.SMALL):
                expected_top = Block.SMALL
            else:
                print('not sure what block to expect for Block.NONE given your env.')
        elif current_top == Block.LARGE:
            if self.in_environment(Block.MEDIUM):
                expected_top = Block.MEDIUM
            elif self.in_environment(Block.SMALL):
                expected_top = Block.SMALL
            else:
                print('not sure what block to expect for Block.LARGE given your env.')
        elif current_top == Block.MEDIUM:
            if self.in_environment(Block.SMALL):
                expected_top = Block.SMALL
            else:
                print('not sure what block to expect for Block.MEDIUM given your env.')
        elif current_top == Block.SMALL:
            for b in self.blocks:
                if b not in BLOCKTOWER:
                    # trying to grab the smallest block when there's other blocks around? nah.
                    return True
            else:
                print('not sure what block to expect here for Block.SMALL given your env.')

        # given the current top of the tower, we know what we think the top should now be.
        # is this not it?
        return not expected_top == attempt_placement.btype

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
        self.verbose_action_results.clear()
        return self.to_position_by_velocity(action)

    def get_ee_position(self) -> Pose:
        curr = AgentState(self.simplefetch).pose
        return Pose(
            _x=curr.x, # - self.START_POSE.x,
            _y=curr.y, # - self.START_POSE.y,
            _z=curr.z, # - self.START_POSE.z,
        )

    def get_ee_velocity(self) -> float:
        return AgentState(self.simplefetch).vel_from_joint_state()
