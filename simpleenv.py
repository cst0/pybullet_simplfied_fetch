import pybullet as p
import pybullet_data
import time
# Can alternatively pass in p.DIRECT
client = p.connect(p.GUI)
p.setGravity(0, 0, -9.8, physicsClientId=client)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath("./meshes")
envId = p.loadURDF("environment.urdf", basePosition=[0,0,0.3625])

position, orientation = p.getBasePositionAndOrientation(envId)

for _ in range(10000):
    pos, ori = p.getBasePositionAndOrientation(envId)
    p.stepSimulation()
    time.sleep(1.0/240.0)

