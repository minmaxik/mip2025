import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

guiFlag = True

dt = 1/240
g = 10
L = 0.8
L1 = L
L2 = L
m = 1

xd = 1.4
zd = 2

physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)
boxId = p.loadURDF("./two-link.urdf.xml", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=0, controlMode=p.POSITION_CONTROL)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=3, targetPosition=0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
  p.stepSimulation()

pos0 = p.getLinkState(boxId, 4)[0]
X0 = np.array([pos0[0], pos0[2]])

maxTime = 5
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
logXsim = np.zeros(sz)
logZsim = np.zeros(sz)
idx = 0
T = 2

for t in logTime:
  pos = p.getLinkState(boxId, 4)[0]
  logXsim[idx] = pos[0]
  logZsim[idx] = pos[2]

  s = 1
  if t < T:
    s = (3/T**2) * t**2 - 2/(T**3) * t**3
  
  target_x = X0[0] + s * (xd - X0[0])
  target_z = X0[1] + s * (zd - X0[1])

  target_pos = [target_x, 0, target_z]

  joint_angles = p.calculateInverseKinematics(
    boxId,
    4,
    target_pos
  )

  p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=joint_angles[0], controlMode=p.POSITION_CONTROL, force=50)
  p.setJointMotorControl2(bodyIndex=boxId, jointIndex=3, targetPosition=joint_angles[1], controlMode=p.POSITION_CONTROL, force=50)

  p.stepSimulation()

  idx += 1
  if guiFlag:
    time.sleep(dt)

p.disconnect()

plt.figure(figsize=(10,8))
plt.subplot(2,1,1)
plt.plot(logTime,logXsim, label="X position")
plt.axhline(y=xd, color='r', linestyle="--", label="X target")
plt.legend()
plt.grid(True)

plt.subplot(2,1,2)
plt.plot(logTime, logZsim, label='Z position')
plt.axhline(y = zd, color='r', linestyle="--", label="Z target")
plt.legend()
plt.grid(True)

plt.show()