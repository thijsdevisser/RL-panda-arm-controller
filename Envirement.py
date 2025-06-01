import numpy as np 
import pybullet as p
import pybullet_data
import time
import gymnasium as gym
from gymnasium import Env, spaces

p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
targid = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase = True)
obj_of_focus = targid

jointid = 4
jlower = p.getJointInfo(targid, jointid)[8]
jupper = p.getJointInfo(targid, jointid)[9]

for step in range(300):
    j2_targ = np.random.uniform(jlower, jupper)
    j4_targ = np.random.uniform(jlower, jupper)
    p.setJointMotorControlArray(targid, [2, 4], p.POSITION_CONTROL, targetPositions = [j2_targ, j4_targ])
    focus_position, _ = p.getBasePositionAndOrientation(targid)
    p.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 0, cameraPitch = -40, cameraTargetPosition = focus_position)
    p.stepSimulation()
    time.sleep(.1)

class BulletEnv(Env):
    def __init__(self):
        self.state = self.initial_state()
        self.step_count = 0

    def initial_state(self):
        p.connect(p.DIRECT)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.pandaUid = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], [0, 0, 0, 1], useFixedBase = True)
        p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        finger_pos = p.getLinkState(self.pandaUid, 9)[0]
        obs = np.array([finger_pos]).flatten()
        return obs
    
    def step(self, action):
        self.step_count += 1
        p.setJointMotorControlArray(self.pandaUid, [4], p.POSITION_CONTROL, [action])
        p.stepSimulation()
        finger_pos = p.getLinkState(self.pandaUid, 9)[0]
        self.state = np.array([finger_pos]).flatten()
        reward = -1 #need fix

        if self.step_count >= 50:
            done = True
        else:
            done = False
            
        truncated = False
        info = {}
        return self.state, reward, done, truncated, info
    
    def reset(self):
        print("Envirement reset")
        p.disconnect()
        self.state = self.initial_state()
        self.step_count = 0
        info = {}
        return self.state, info
    
    def render(self):
        pass
    

