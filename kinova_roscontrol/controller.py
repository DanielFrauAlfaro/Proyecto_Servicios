#! /usr/bin/python3

import rospy
from spatialmath.base import *
import roboticstoolbox as rtb
from math import pi
from std_msgs.msg import Float64
import numpy as np

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
        q = [0.0, 2.9, 1.3 ,-2.07, 1.4, 0.0]
        traj = rtb.jtraj(q,self.model.qr,300)
        
        rate = rospy.Rate(20)
        j = 0
        while not rospy.is_shutdown() and j<300:
            for i in range(6):
                self.publishers[i].publish(traj.q[j,i])
            j = j + 1
            rate.sleep()


if __name__ == '__main__':
    print("hola")
    controller = Controller()
    
    controller.action()