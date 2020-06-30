import pybullet as p
import pybullet_data
import time
import numpy as np
from threading import Thread
#simulatiing a bouncing ball to understand ground contact pybullet APIs
ball_pos = np.array([0,0,2])

def simulate():
    physicsClient = p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    ground = p.loadURDF("plane.urdf")
    ballStartPos = [0,0,2]
    ballStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    ball = p.loadURDF("updated_sphere_with_restitution.urdf", ballStartPos, ballStartOrientation)
    sim = True;
    while sim:

        p.stepSimulation()
        time.sleep(0.001)
    p.disconnect()

thread = Thread(target = simulate)
thread.start()






