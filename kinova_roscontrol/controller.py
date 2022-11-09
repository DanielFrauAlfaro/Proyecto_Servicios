#! /usr/bin/python3
# ROSBRIDGE

import rospy
from spatialmath.base import *
import roboticstoolbox as rtb
from math import pi
from std_msgs.msg import Float64
import numpy as np
from spatialmath import *
from geometry_msgs.msg import Pose
import moveit_commander

class Controller():
    def __init__(self):
        self.publishers = []
        rospy.init_node("main_controller", anonymous=False)
        for i in range(1,7):
            self.publishers.append(rospy.Publisher("/j2n6s300_joint_" + str(i) + "_controller/command", Float64,queue_size=10))
        
        for j in range(1,4):
            self.publishers.append(rospy.Publisher("/j2n6s300_joint_finger_" + str(j) + "_controller/command", Float64,queue_size=10))
            
            self.publishers.append(rospy.Publisher("/j2n6s300_joint_finger_tip_" + str(j) + "_controller/command", Float64,queue_size=10))
            
        self.model = rtb.models.DH.Jaco()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.gripper = moveit_commander.MoveGroupCommander("gripper")
        
    def action(self):
        q = [pi/2, pi, pi ,0.0, 0.0, 0.0]
        
        T0 = self.model.fkine(q)
        T1 = SE3(0, 0.5, 0.1)
        print(T1)
        q1 = self.model.ikine_LM(T1)
        
        print(q1)
        
        print("---------- Cartesian trajectory ---------")
        t = np.arange(0, 1, 0.20)
        # Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))
        
        print("------------ Joint trajectory -----------")        
        # traj = self.model.ikine_LM(Ts, rlimit=500)
        traj = rtb.jtraj(q,q1.q,300)

        print(traj)
        rate = rospy.Rate(20)
        j = 0
        
        while not rospy.is_shutdown() and j<300:
            
            for i in range(6):
                self.publishers[i].publish(traj.q[j,i])
            j = j + 1
            rate.sleep()
    
    def home(self):
        q = [pi/2, pi, pi ,0.0, 0.0, 0.0]
        
        rate = rospy.Rate(5)
    
        for i in range(6):
            print(i)
            self.publishers[i].publish(q[i])
            rate.sleep()

if __name__ == '__main__':
    controller = Controller()
    # controller.action()