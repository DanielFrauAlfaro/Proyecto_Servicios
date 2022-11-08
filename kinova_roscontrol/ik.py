from spatialmath.base import *
from spatialmath import *
import roboticstoolbox as rtb
from math import pi
import matplotlib as plt
import cv2 as cv
import numpy as np

kinova = rtb.models.DH.Jaco()
print(kinova)
T_tool = transl(0,0,0.1214)

q = [0.0, 2.9, 1.3 ,-2.07, 1.4, 0.0]
q0 = [-pi/2, pi, pi, 0.0, 0.0, 0.0]
T = kinova.fkine(q0)
print(T)
sol = kinova.ikine_LM(T,q0 = q)
print(sol)

traj = rtb.jtraj(kinova.qz, q0, 100)
# kinova.plot(traj.q)

t = np.arange(0, 1, 0.10)

T0 = SE3(0.6, -0.5, 0.0)

T1 = SE3(0.4, 0.5, 0.2)

#Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))

# sol = kinova.ikine_LM(Ts)   
#kinova.plot(sol.q)

q = [pi/2, pi, pi ,0.0, 0.0, 0.0]
        
T1 = SE3(0.5, 0, 1) * SE3.Rx(-90, unit='deg')
q1 = kinova.ikine_LM(T1)
print(T1)
print(q1)
traj = rtb.jtraj(q, q1.q, 300)
# kinova.plot(traj.q)