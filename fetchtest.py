import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartQuat = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath("/opt/ros/melodic/share/fetch_description/robots/")
boxId = p.loadURDF("fetch.urdf", cubeStartPos, cubeStartQuat)

for i in range(10000):
    p.stepSimulation()
    time.sleep(1.0/240.0)

cubeP, cubeQ = p.getBasePositionAndOrientation(boxId)
print(cubeP, cubeQ)

p.disconnect()
