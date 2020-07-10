import quadcopter_sim
import numpy as np
import pybullet as p
import datalogger
configurations = quadcopter_sim.Config()
Kp = np.zeros(6)
Ki = np.zeros(6)
Kd = np.zeros(6)

# give them values:osition) and quaternion_meas (measured
# orientation), the function returns the forces for the 4 actuators and
# the moment for the yaw-control
# x-y-z controlers:
Kp[0] = 0.001
Kp[1] = 0.001
Kp[2] = 0.0025

Kd[0] = 0.005
Kd[1] = 0.005
Kd[2] = 0.00001
Ki[0] = 0
Ki[1] = 0
Ki[2] = 0.0000001

    # roll-pitch-yaw controlers (yaw is already prefilled):
Kp[3] = 0.1
Kp[4] = 0.1
Kp[5] = 0.000

Kd[3] = 0.01
Kd[4] = 0
Kd[5] = 0.00

Ki[3] = 0
Ki[4] = 0
Ki[5] = 0
qcc = quadcopter_sim.quadcopter_control(Kp,Ki,Kd, configurations)
physicsClient = p.connect(p.GUI)  # pybullet only for computations no visualisation
p.setGravity(0, 0, -configurations.gravity)
p.setTimeStep(configurations.Tsample_physics)

# disable real-time simulation, we want to step through the
# physics ourselves with p.stepSimulation()
p.setRealTimeSimulation(0)
# p.setAdditionalSearchPath("~/Documents/research/quadcopter_sim/")
# planeId = p.loadURDF("plane.urdf",[0,0,0],p.getQuaternionFromEuler([0,0,0]))
quadcopterId = p.loadURDF("quadrotor.urdf", [0, 0, 1], p.getQuaternionFromEuler([0, 0, 0]))
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))

targets1 = [np.array([0, 0, 0]), np.array([0, 0, 3]), np.array([9,0,3]), np.array([9,9,3]),
           np.array([-9,9,3]),np.array([-9,-9,3]),np.array([9,-9,3]), np.array([0,0,0])]
targets0 = [np.array([0,0,3]), np.array([1,2,2]), np.array([-1,-2,4]),np.array([3,3,1])]
targets2 = [np.array([0,0,3]),np.array([0,0,0.5]),np.array([0,0,0.3]),np.array([0,0,0.2])]
targets=targets2

graph = datalogger.datalogger()
quadcopter_sim.startSimulation(configurations, qcc, quadcopterId, graph)
i = 0
while i < len(targets):
    qcc.ref_pos = targets[i]
    pos_meas, orientation_meas = p.getBasePositionAndOrientation(quadcopterId)
    while (any(abs(pos_meas - qcc.ref_pos) > np.array([0.05, 0.05, 0.3]))):
        pos_meas, _ = p.getBasePositionAndOrientation(quadcopterId)
        #print(p.getEulerFromQuaternion(orientation_meas))
        print("not there yet")
    print("moving to next target")
    i += 1
