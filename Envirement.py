import numpy as np 
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase = True)
obj_of_focus = targid

for step in range(300):
    j2_targ = np.random.uniform(jlower, jupper)
    j4_targ = np.random.uniform(jlower, jupper)
    p.setJointMotorControlArray(targid, [2, 4], p.POSITION_CONTROL, targetPositions = [j2_targ, j4_targ])
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebug