#! /usr/bin/python3
# ROSBRIDGE

import rospy
from spatialmath.base import *
import roboticstoolbox as rtb
from math import pi
from std_msgs.msg import Float64
import numpy as np
from spatialmath import *

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
    
    def action(self):
        q = [pi/2, pi, pi ,0.0, 0.0, 0.0]
        # traj = rtb.jtraj(q,self.model.qr,300)
        
        T0 = self.model.fkine(q)
        T1 = SE3(0.3, 0, 0.5) * SE3.RPY(0.1, 0.2, 0.3)
                
        t = np.arange(0, 1, 0.10)
        Ts = rtb.tools.trajectory.ctraj(T0, T1, len(t))
        
        traj = self.model.ikine_LM(Ts, rlimit=500)   
        print(traj)
        rate = rospy.Rate(20)
        j = 0
        while not rospy.is_shutdown() and j<len(t):
            for i in range(6):
                self.publishers[i].publish(traj.q[j,i])
            j = j + 1
            rate.sleep()


if __name__ == '__main__':
    controller = Controller()
    controller.action()