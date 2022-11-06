import numpy as np # Scientific computing library
from math import pi, sin, cos
# Project: Coding Denavit-Hartenberg Tables Using Python - 6DOF Robotic Arm
#          This code excludes the servo motor that controls the gripper.
# Author: Addison Sears-Collins
# Date created: August 22, 2020
 
# Link lengths in centimeters
a1 = 0.2755 # Length of link 1
a2 = 0.41 # Length of link 2
a3 = 0.2073 # Length of link 3
a4 = 0.0743 # Length of link 4
a5 = 0.0743 # Length of link 5
a6 = 0.1687 # Length of link 6

aa = 11*pi/72.0
e2 = 0.0098
sa = sin(aa)
s2a = cos(2*aa)
d4b = (a3 + sa/s2a*a4)
d5b = sa/s2a*a4 + sa/s2a*a5
d6b = sa/s2a*a5 + a6
# Initialize values for the joint angles (degrees)
theta_1 = -pi/2 # Joint 1
theta_2 = pi - pi/2 # Joint 2
theta_3 = pi + pi/2 # Joint 3
theta_4 = 0 # Joint 4
theta_5 = 0 - pi # Joint 5
theta_6 = 0 + 100*pi/180.0

# Declare the Denavit-Hartenberg table. 
# It will have four columns, to represent:
# theta, alpha, r, and d
# We have the convert angles to radians.
d_h_table = np.array([[(theta_1),   a1,      0,        pi/2],
                      [(theta_2),    0,     a2,          pi],
                      [(theta_3),  -e2,      0,        pi/2],
                      [(theta_4), -0.2502,   0,       1.047],
                      [(theta_5), -0.08579,  0,       1.047],
                      [(theta_6), -0.2116,   0,         pi,]]) 

# Homogeneous transformation matrix from frame 0 to frame 1
i = 0
homgen_0_1 = np.array([[cos(d_h_table[i,0]), -sin(d_h_table[i,0]) * cos(d_h_table[i,3]), sin(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * cos(d_h_table[i,0])],
                      [sin(d_h_table[i,0]), cos(d_h_table[i,0]) * cos(d_h_table[i,3]), -cos(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * sin(d_h_table[i,0])],
                      [0, sin(d_h_table[i,3]), cos(d_h_table[i,3]), d_h_table[i,1]],
                      [0, 0, 0, 1]])  
 
# Homogeneous transformation matrix from frame 1 to frame 2
i = 1
homgen_1_2 = np.array([[cos(d_h_table[i,0]), -sin(d_h_table[i,0]) * cos(d_h_table[i,3]), sin(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * cos(d_h_table[i,0])],
                      [sin(d_h_table[i,0]), cos(d_h_table[i,0]) * cos(d_h_table[i,3]), -cos(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * sin(d_h_table[i,0])],
                      [0, sin(d_h_table[i,3]), cos(d_h_table[i,3]), d_h_table[i,1]],
                      [0, 0, 0, 1]])  
 
# Homogeneous transformation matrix from frame 2 to frame 3
i = 2
homgen_2_3 = np.array([[cos(d_h_table[i,0]), -sin(d_h_table[i,0]) * cos(d_h_table[i,3]), sin(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * cos(d_h_table[i,0])],
                      [sin(d_h_table[i,0]), cos(d_h_table[i,0]) * cos(d_h_table[i,3]), -cos(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * sin(d_h_table[i,0])],
                      [0, sin(d_h_table[i,3]), cos(d_h_table[i,3]), d_h_table[i,1]],
                      [0, 0, 0, 1]])  
 
# Homogeneous transformation matrix from frame 3 to frame 4
i = 3
homgen_3_4 = np.array([[cos(d_h_table[i,0]), -sin(d_h_table[i,0]) * cos(d_h_table[i,3]), sin(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * cos(d_h_table[i,0])],
                      [sin(d_h_table[i,0]), cos(d_h_table[i,0]) * cos(d_h_table[i,3]), -cos(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * sin(d_h_table[i,0])],
                      [0, sin(d_h_table[i,3]), cos(d_h_table[i,3]), d_h_table[i,1]],
                      [0, 0, 0, 1]])  
 
# Homogeneous transformation matrix from frame 4 to frame 5
i = 4
homgen_4_5 = np.array([[cos(d_h_table[i,0]), -sin(d_h_table[i,0]) * cos(d_h_table[i,3]), sin(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * cos(d_h_table[i,0])],
                      [sin(d_h_table[i,0]), cos(d_h_table[i,0]) * cos(d_h_table[i,3]), -cos(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * sin(d_h_table[i,0])],
                      [0, sin(d_h_table[i,3]), cos(d_h_table[i,3]), d_h_table[i,1]],
                      [0, 0, 0, 1]])  
 
i = 5
homgen_5_6 = np.array([[cos(d_h_table[i,0]), -sin(d_h_table[i,0]) * cos(d_h_table[i,3]), sin(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * cos(d_h_table[i,0])],
                      [sin(d_h_table[i,0]), cos(d_h_table[i,0]) * cos(d_h_table[i,3]), -cos(d_h_table[i,0]) * sin(d_h_table[i,3]), d_h_table[i,2] * sin(d_h_table[i,0])],
                      [0, sin(d_h_table[i,3]), cos(d_h_table[i,3]), d_h_table[i,1]],
                      [0, 0, 0, 1]])  
 
homgen_0_5 = homgen_0_1 @ homgen_1_2 @ homgen_2_3 @ homgen_3_4 @ homgen_4_5 @ homgen_5_6
 
# Print the homogeneous transformation matrices
print("Homogeneous Matrix Frame 0 to Frame 6:")
print(homgen_0_5)