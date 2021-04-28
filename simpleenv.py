import pybullet as p
import pybullet_data
import time
# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.8, physicsClientId=client)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

x_axis = p.addUserDebugParameter('x_axis', -0.1, 0.1, 0)
y_axis = p.addUserDebugParameter('y_axis', -0.1, 0.1, 0)
z_axis = p.addUserDebugParameter('z_axis', -0.1, 0.1, 0)

p.setAdditionalSearchPath("./urdf")
p.setAdditionalSearchPath("./urdf/meshes")
envId = p.loadURDF("./urdf/environment.urdf", basePosition=[0,0,0.3625])

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


position, orientation = p.getBasePositionAndOrientation(envId)

for _ in range(10000):
    user_x_axis = p.readUserDebugParameter(x_axis)
    user_y_axis = p.readUserDebugParameter(y_axis)
    user_z_axis = p.readUserDebugParameter(z_axis)

    pos, ori = p.getBasePositionAndOrientation(envId)
    p.setJointMotorControl2(envId, 1, p.VELOCITY_CONTROL, targetVelocity=user_x_axis);
    p.setJointMotorControl2(envId, 2, p.VELOCITY_CONTROL, targetVelocity=user_y_axis);
    p.setJointMotorControl2(envId, 3, p.VELOCITY_CONTROL, targetVelocity=user_z_axis);
    p.stepSimulation()
    time.sleep(1.0/240.0)

