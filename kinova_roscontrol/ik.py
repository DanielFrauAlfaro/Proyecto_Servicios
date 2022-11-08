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
T = kinova.fkine(q)

sol = kinova.ikine_LM(T,q0 = q)
# print(sol)

traj = rtb.jtraj(kinova.qr, q, 100)
# kinova.plot(traj.q)

t = np.arange(0, 2, 0.010)

T0 = SE3(0.6, -0.5, 0.0)

T1 = SE3(0.4, 0.5, 0.2)

# Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))

# sol = kinova.ikine_LM(Ts)   
# kinova.plot(sol.q)

T = SE3(-0.39, -0.14, 0.09) * SE3.RPY(-178.625, 0.24, 84.517)
print(T)
sol = kinova.ikine_LM(T, ilimit=1000, rlimit=1000)  
print(sol)

kinova.teach()