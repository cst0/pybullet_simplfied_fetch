import pybullet as p
import os

class SimpleFetch:
    def __init__(self, client):
        self.client = client
        p.setAdditionalSearchPath("./resources/")
        p.setAdditionalSearchPath("./resources/meshes")
        f_name = os.path.join(os.path.dirname(__file__), 'simplefetch.urdf')
        self.simplefetch = p.loadURDF(fileName=f_name,
                basePosition=[0,0,0.3625],
                physicsClientId=client)

        #number_of_joints = p.getNumJoints(envId)
        #for joint_number in range(number_of_joints):
        #    info = p.getJointInfo(envId, joint_number)
        #    print(info[0], ": ", info[1])
        ## Running the above shows us the below:
        # 0 :  'start_pose_offset_fixed_joint'
        # 1 :  'table_to_gripper_x'
        # 2 :  'table_to_gripper_y'
        # 3 :  'table_to_gripper_z'
        # 4 :  'prismatic_to_gripper'
        # 5 :  'gripper_to_finger_left'
        # 6 :  'gripper_to_finger_right'

        self.x_axis_joint = 1
        self.y_axis_joint = 2
        self.z_axis_joint = 3
        self.x_speed = 0
        self.y_speed = 0
        self.z_speed = 0

    def get_ids(self):
        return self.simplefetch, self.client

    def apply_action(self, action):
        x_speed, y_speed, z_speed = action
        p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=x_speed);
        p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=y_speed);
        p.setJointMotorControl2(self.simplefetch, self.x_axis_joint, p.VELOCITY_CONTROL, targetVelocity=z_speed);

    def get_observation(self):
        position, angle = p.getBasePositionAndOrientation(self.simplefetch, self.client)
        return position, angle

